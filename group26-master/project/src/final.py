#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# TurtleBot must have minimal.launch & amcl_demo.launch
# running prior to starting this script
# For simulation: launch gazebo world & amcl_demo prior to run this script
#from __future__ import division
import rospy
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
import cv2.cv as cv
from kobuki_msgs.msg import BumperEvent


import yaml
import cv2
import sys
import math



from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class GoToPose():
    def __init__(self):

        self.goal_sent = False
        #self.contours_green = 0
        #self.contours_red = 0

        self.bump = False
        self.mask_green=0
        self.image_b = 0
        self.flag_green = 0
        self.flag_red = 0
        self.sensitivity = 10
        self.bridge = CvBridge()
        rospy.Subscriber('/camera/rgb/image_raw',Image,self.callback)
        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)

        rospy.Subscriber('/mobile_base/events/bumper',BumperEvent,self.callback_bump)

        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
        self.rate = rospy.Rate(10) #10hz
        self.desired_velocity = Twist()


# Method used to draw the matched features between the cluedo character and the images provided.
#It was used in the function match_feature. It was taken from the stack overflow website. Reference is provided for it.

    def drawMatches(self, img1, kp1, img2, kp2, matches):
        """
        My own implementation of cv2.drawMatches as OpenCV 2.4.9
        does not have this function available but it's supported in
        OpenCV 3.0.0

        This function takes in two images with their associated
        keypoints, as well as a list of DMatch data structure (matches)
        that contains which keypoints matched in which images.

        An image will be produced where a montage is shown with
        the first image followed by the second image beside it.

        Keypoints are delineated with circles, while lines are connected
        between matching keypoints.

        img1,img2 - Grayscale images
        kp1,kp2 - Detected list of keypoints through any of the OpenCV keypoint
                  detection algorithms
        matches - A list of matches of corresponding keypoints through any
                  OpenCV keypoint matching algorithm
        """

        # Create a new output image that concatenates the two images together
        # (a.k.a) a montage
        rows1 = img1.shape[0]
        cols1 = img1.shape[1]
        rows2 = img2.shape[0]
        cols2 = img2.shape[1]

        # Create the output image
        # The rows of the output are the largest between the two images
        # and the columns are simply the sum of the two together
        # The intent is to make this a colour image, so make this 3 channels
        out = np.zeros((max([rows1,rows2]),cols1+cols2,3), dtype='uint8')

        # Place the first image to the left
        out[:rows1,:cols1] = np.dstack([img1, img1, img1])

        # Place the next image to the right of it
        out[:rows2,cols1:] = np.dstack([img2, img2, img2])

        # For each pair of points we have between both images
        # draw circles, then connect a line between them
        for mat in matches:

            # Get the matching keypoints for each of the images
            img1_idx = mat.queryIdx
            img2_idx = mat.trainIdx

            # x - columns
            # y - rows
            (x1,y1) = kp1[img1_idx].pt
            (x2,y2) = kp2[img2_idx].pt

            # Draw a small circle at both co-ordinates
            # radius 4
            # colour blue
            # thickness = 1
            cv2.circle(out, (int(x1),int(y1)), 4, (255, 0, 0), 1)
            cv2.circle(out, (int(x2)+cols1,int(y2)), 4, (255, 0, 0), 1)

            # Draw a line in between the two points
            # thickness = 1
            # colour blue
            cv2.line(out, (int(x1),int(y1)), (int(x2)+cols1,int(y2)), (255,0,0), 1)


        # Show the image
        cv2.imshow('Matched Features', out)
        cv2.imwrite("/home/csunix/sc19gs/Desktop/matched_image.png",out)
        cv2.waitKey(3)
        cv2.destroyWindow('Matched Features')

        # Also return the image if you'd like a copy
        return out

#Method used to set the bump state when the robot is bumped.
    def callback_bump(self, data):
    	self.bump
    	if data.state == 1:
    		self.bump = True

#Method used for taking in the raw image from the robot's camera and finding various contours.
    def callback(self, data):

		try:
			self.image_b = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print (e)

		hsv_green_lower = np.array([40-self.sensitivity,100,0])
		hsv_green_upper = np.array([80+self.sensitivity,255,255])
		hsv_red_lower = np.array([0,255,100])
		hsv_red_upper = np.array([0,255,150])
		hsv_skin_lower = np.array([15,35,170])
		hsv_skin_upper = np.array([35,60,250])
		hsv_m_yellow_lower = np.array([24,70,141])
		hsv_m_yellow_upper = np.array([26,255,212])
		hsv_p_blue_lower = np.array([100,113,95])
		hsv_p_blue_upper = np.array([110,183,212])
		hsv_s_red_lower = np.array([0, 29, 69])
		hsv_s_red_upper = np.array([5, 115, 230])
		hsv_p_purple_lower = np.array([150,50,40])
		hsv_p_purple_upper = np.array([160,255,255])

		hsv = cv2.cvtColor(self.image_b,cv2.COLOR_BGR2HSV)

		self.mask_green = cv2.inRange(hsv, hsv_green_lower, hsv_green_upper)
		self.mask_red = cv2.inRange(hsv, hsv_red_lower, hsv_red_upper)
		self.mask_skin = cv2.inRange(hsv, hsv_skin_lower, hsv_skin_upper)
		self.mask_m_yellow = cv2.inRange(hsv, hsv_m_yellow_lower, hsv_m_yellow_upper)
		self.mask_p_blue = cv2.inRange(hsv, hsv_p_blue_lower, hsv_p_blue_upper)
		self.mask_s_red = cv2.inRange(hsv, hsv_s_red_lower, hsv_s_red_upper)
		self.mask_p_purple = cv2.inRange(hsv, hsv_p_purple_lower, hsv_p_purple_upper)

		self.contours_green = cv2.findContours(self.mask_green.copy(),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[0]
		self.contours_red = cv2.findContours(self.mask_red.copy(),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[0]
		self.contours_skin = cv2.findContours(self.mask_skin.copy(),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[0]
		self.contours_m_yellow = cv2.findContours(self.mask_m_yellow.copy(),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[0]
		self.contours_p_blue = cv2.findContours(self.mask_p_blue.copy(),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[0]
		self.contours_s_red = cv2.findContours(self.mask_s_red.copy(),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[0]
		self.contours_p_purple = cv2.findContours(self.mask_p_purple.copy(),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[0]

#Method used to give the goal coordinates to the robot and moving the robot to this co-ordinates
    def goto(self, pos, quat):

		# Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
        Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
        # Start moving
        self.move_base.send_goal(goal)
        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60))

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
            rospy.Subscriber('/camera/rgb/image_raw',Image,self.callback)
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

#Method to detect the image based on the background color of the image and printing the name of the cluedo cluedo_character
#in the text file and taking the snapshot of that cluedo character
    def detect_image(self):

        img0 = cv2.imread('/home/csunix/sc19gs/Desktop/peacock.png')
        img1 = cv2.imread('/home/csunix/sc19gs/Desktop/mustard.png')
        img2 = cv2.imread('/home/csunix/sc19gs/Desktop/plum.png')
        img3 = cv2.imread('/home/csunix/sc19gs/Desktop/scarlet.png')
        file = open('/home/csunix/sc19gs/Desktop/cluedo_character.txt','w')


        if(len(self.contours_p_blue)>0):
            print("Peacock")
            cv2.imshow("Detected Image",img0)
            cv2.imwrite("/home/csunix/sc19gs/Desktop/detected_image.png",img0)
            cv2.imwrite("/home/csunix/sc19gs/Desktop/cluedo_character.png",self.image_b)
            file.write('Peacock')

        elif(len(self.contours_p_purple)>0):
            print("Plum")
            cv2.imshow("Detected Image",img2)
            cv2.imwrite("/home/csunix/sc19gs/Desktop/detected_image.png",img2)
            cv2.imwrite("/home/csunix/sc19gs/Desktop/cluedo_character.png",self.image_b)
            file.write('Plum')

        elif(len(self.contours_m_yellow)>0):
            print("Mustard")
            cv2.imshow("Detected Image",img1)
            cv2.imwrite("/home/csunix/sc19gs/Desktop/detected_image.png",img1)
            cv2.imwrite("/home/csunix/sc19gs/Desktop/cluedo_character.png",self.image_b)
            file.write('Mustard')

        elif(len(self.contours_s_red)>0):
            print("Scarlet")
            cv2.imshow("Detected Image",img3)
            cv2.imwrite("/home/csunix/sc19gs/Desktop/detected_image.png",img3)
            cv2.imwrite("/home/csunix/sc19gs/Desktop/cluedo_character.png",self.image_b)
            file.write('Scarlet')

#Method used for detecting the cluedo character based on the feature mapping and homography
    def match_feature(self):
        img0 = cv2.imread('/home/csunix/sc19gs/Desktop/peacock.png',0)
        img1 = cv2.imread('/home/csunix/sc19gs/Desktop/mustard.png',0)
        img2 = cv2.imread('/home/csunix/sc19gs/Desktop/plum.png',0)
        img3 = cv2.imread('/home/csunix/sc19gs/Desktop/scarlet.png',0)


        height, width = self.image_b.shape[:2]
        reSize1 = cv2.resize(self.image_b, (4*width, 4*height), interpolation=cv2.INTER_CUBIC)
        cv2.imwrite("/home/csunix/sc19gs/Desktop/photo.png",reSize1)
        img = cv2.imread('/home/csunix/sc19gs/Desktop/photo.png',0)


        orb = cv2.ORB()
        kp, des = orb.detectAndCompute(img,None)


        kp0, des0 = orb.detectAndCompute(img0,None)
        kp1, des1 = orb.detectAndCompute(img1,None)
        kp2, des2 = orb.detectAndCompute(img2,None)
        kp3, des3 = orb.detectAndCompute(img3,None)

        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)



        matches0 = bf.match(des, des0)
        matches1 = bf.match(des, des1)
        matches2 = bf.match(des, des2)
        matches3 = bf.match(des, des3)
        matches0 = sorted(matches0, key = lambda x:x.distance)
        matches1 = sorted(matches1, key = lambda x:x.distance)
        matches2 = sorted(matches2, key = lambda x:x.distance)
        matches3 = sorted(matches3, key = lambda x:x.distance)

        max=0
        j=0


        for i in range(4):
            if(i==0):
                print(len(matches0))
                if(len(matches0)>max):
                    max=len(matches0)
                    j=i
            elif(i==1):
                print(len(matches1))
                if(len(matches1)>max):
                    max=len(matches1)
                    j=i
            elif(i==2):
                print(len(matches2))
                if(len(matches2)>max):
                    max=len(matches2)
                    j=i
            elif(i==3):
                print(len(matches3))
                if(len(matches3)>max):
                    max=len(matches3)
                    j=i

        if(j==0):
            cv2.imshow("Detected Image",img0)
            self.drawMatches(img,kp,img0,kp0,matches0[:])
            cv2.waitKey(3)
        elif(j==1):
            cv2.imshow("Detected Image",img1)
            self.drawMatches(img,kp,img1,kp1,matches1[:])
            cv2.waitKey(3)
        elif(j==2):
            cv2.imshow('Detected Image',img2)
            self.drawMatches(img,kp,img2,kp2,matches2[:])
            cv2.waitKey(3)
        elif(j==3):
            self.drawMatches(img,kp,img3,kp3,matches3[:])
            cv2.imshow('Detected Image',img3)
            cv2.waitKey(3)


        return j


#Method used to detect the second largest rectangle in the image
    def detect_rectangle(self):


        colour_max_area2 = 0

        img = cv2.cvtColor(self.image_b,cv2.COLOR_BGR2GRAY)
        _,threshold = cv2.threshold(img,40,255,cv2.THRESH_BINARY)
        contours = cv2.findContours(threshold,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

        if(len(self.contours_skin)>0 or len(self.contours_p_purple)>0):
            t1=0
            colour_max_area1=0
            areas1 = []
            try:
                for i in range(len(self.contours_skin)):
                    c1=navigator.contours_skin[i]
                    areas1.append(cv2.contourArea(c1))
                    if areas1[i]>colour_max_area1:
                        colour_max_area1=areas1[i]
                        t1=i
                print("detect skin color",colour_max_area1)
            except IndexError:
                print("contour skin out of range")

            if(len(contours)>0):
                font = cv2.FONT_HERSHEY_COMPLEX

                for cnt in contours:
                    approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
                    cv2.drawContours(img, [approx], 0, (0,255,0), 5)

                    if len(approx) == 4:
                        x = approx.ravel()[0]
                        y = approx.ravel()[1]
                        cv2.putText(img, "Rectangle", (x,y), font, 1, (0))

                        t1=0
                        t2 = 0
                        colour_max_area1=0
                        colour_max_area2=0
                        areas1 = []
                        areas2 = []
                        for i in range(len(contours)):
                            c1=contours[i]
                            areas1.append(cv2.contourArea(c1))
                            if areas1[i]>colour_max_area1:
                                colour_max_area1=areas1[i]
                                t1=i
                        cnt1 = contours[t1]

                        for i in range(len(contours)):
                            c2=contours[i]
                            areas2.append(cv2.contourArea(c2))
                            if (areas2[i]>colour_max_area2 and areas2[i] != colour_max_area1):
                                colour_max_area2=areas2[i]
                                t2=i

                            print("Area of rectangle", colour_max_area2)
                        cnt2 = contours[t2]

                        if(len(contours)>0):
                            M1= cv2.moments(cnt2)
                            try:
                                cx1, cy1 = int(M1['m10']/M1['m00']), int(M1['m01']/M1['m00'])
                            except ZeroDivisionError:
                                return -1;


                        #cv2.imshow("shapes", img)
                        #cv2.waitKey(3)
                        if(colour_max_area2>0 and colour_max_area2<15000):
                            return cx1;
                        elif(colour_max_area2>15000):
                            return -2
                        else:
                            return -1

        return -1;


if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=True)
        navigator = GoToPose()




        with open("/home/csunix/sc19gs/catkin_ws/src/group_project/project/example/input_points.yaml",'r') as stream:
		points = yaml.safe_load(stream)

        # Coordinates for the entrance of the first room
        x = points['room1_entrance_xy'][0] # SPECIFY X COORDINATE HERE
        y = points['room1_entrance_xy'][1]# SPECIFY Y COORDINATE HERE
        theta = 0# SPECIFY THETA (ROTATION) HERE
        position = {'x': x, 'y' : y}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position, quaternion)
        if success:
            rot=0
            while(rot<200):
                navigator.desired_velocity.linear.x = 0.0 # making spiral, forward velocity
                navigator.desired_velocity.angular.z = 0.4 # making spiral, angular velocity
                navigator.pub.publish(navigator.desired_velocity)
                navigator.rate.sleep()
                if(len(navigator.contours_green)>0): # Detecting the green color for detecting the green circle
                    t1=0
                    colour_max_area1=0
                    areas1 = []
                    for i in range(len(navigator.contours_green)):
                        c1=navigator.contours_green[i]
                        areas1.append(cv2.contourArea(c1))
                        if areas1[i]>colour_max_area1:
                            colour_max_area1=areas1[i]
                            t1=i
                    print("Green Contour max area",colour_max_area1)


                    if(colour_max_area1>2000): # Putting the threshold for the green circle area
                        r1=math.sqrt(areas1[t1]/3.14)
                        cnt1 = navigator.contours_green[t1]
                        M1= cv2.moments(cnt1)
                        cx1, cy1 = int(M1['m10']/M1['m00']), int(M1['m01']/M1['m00'])

                        img = cv2.cvtColor(navigator.image_b,cv2.COLOR_BGR2GRAY)
                        img = cv2.medianBlur(img,5)
                        cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

                        #Trying to align the circle center with the window frame center to take a better image of the green circle
                        while(cx1 < 310 or cx1 > 330 ):
                            if(310>cx1):
                                navigator.desired_velocity.angular.z = 0.025
                            elif(330<cx1):
                                navigator.desired_velocity.angular.z = -0.025
                            navigator.pub.publish(navigator.desired_velocity)
                            navigator.rate.sleep()
                            for i in range(len(navigator.contours_green)):
                                c1=navigator.contours_green[i]
                                areas1.append(cv2.contourArea(c1))
                                if areas1[i]>colour_max_area1:
                                    colour_max_area1=areas1[i]
                                    t1=i
                            r1=math.sqrt(areas1[t1]/3.14)
                            cnt1 = navigator.contours_green[t1]
                            M1= cv2.moments(cnt1)
                            try:
                                cx1, cy1 = int(M1['m10']/M1['m00']), int(M1['m01']/M1['m00'])
                            except ZeroDivisionError:
                                print("Division by zero")

                        cv2.imshow('Camera_Feed',navigator.image_b)

                        #After detecting the green circle, trying to find the circles around that colour using hough circles method
                        circles = cv2.HoughCircles(img,cv.CV_HOUGH_GRADIENT,0.1,1000,
                        param1=50,param2=30,minRadius=60,maxRadius=1200)        # trying to detect circles for green contours

                        circles = np.uint16(np.around(circles))
                        for i in circles[0,:]:
                            # draw the outer circle
                            cv2.circle(navigator.image_b,(cx1,cy1),int(r1),(0,255,0),2)
                            # draw the center of the circle
                            cv2.circle(navigator.image_b,(cx1,cy1),2,(0,0,255),3)
                            cv2.imshow('Green Circle',navigator.image_b)
                            cv2.imwrite("/home/csunix/sc19gs/Desktop/green_circle.png",navigator.image_b)

                            #After detecting the right room moving to the center coordinates of that room
                            x = points['room1_centre_xy'][0] # SPECIFY X COORDINATE HERE
                            y = points['room1_centre_xy'][1]# SPECIFY Y COORDINATE HERE
                            theta = 1.5# SPECIFY THETA (ROTATION) HERE
                            navigator.flag_green=1
                            while(theta<9.5):
                                position = {'x': x, 'y' : y}
                                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}
                                rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
                                success = navigator.goto(position, quaternion)

                                if success:

                                    l=10
                                    for f in range(10000000):
                                        for y in range(4):
                                            if not navigator.bump:
                                                navigator.desired_velocity.linear.x = 0.05  # making spiral, forward velocity
                                                navigator.desired_velocity.angular.z = 0    # making spiral, forward velocity
                                                for i in range (l):
                                                    if not navigator.bump:
                                                        navigator.pub.publish(navigator.desired_velocity)
                                                        navigator.rate.sleep()

                                    #Trying to detect the rectangle and making it align with the center coordinates of the window frame
                                                        loc_x = navigator.detect_rectangle()
                                                        print("Loc - x", loc_x)
                                                        while (loc_x != -1 and loc_x !=-2 and loc_x != None):
                                                            if loc_x < 310:
                                                                navigator.desired_velocity.angular.z = math.pi/60
                                                                navigator.desired_velocity.linear.x = 0.0
                                                            elif loc_x >330:
                                                                navigator.desired_velocity.angular.z = -math.pi/60
                                                                navigator.desired_velocity.linear.x = 0.0
                                                            else:
                                                                navigator.desired_velocity.angular.z = 0
                                                                navigator.desired_velocity.linear.x = 0.1

                                                            navigator.pub.publish(navigator.desired_velocity)
                                                            navigator.rate.sleep()
                                                            loc_x = navigator.detect_rectangle()
                                                            print("Loc - x", loc_x)
                                                    #If the robot is close enough to the image then stopping the robot and detecting the character
                                                        while(loc_x == -2):
                                                            navigator.desired_velocity.angular.z = 0.0
                                                            navigator.desired_velocity.linear.x = 0
                                                            navigator.pub.publish(navigator.desired_velocity)
                                                            navigator.rate.sleep()
                                                            navigator.detect_image()
                                                            exit()


                                                for i in range (30):
                                                    if not navigator.bump:
                                                        navigator.desired_velocity.linear.x = 0.0       # making spiral, rotate velocity
                                                        navigator.desired_velocity.angular.z = 0.22     # making spiral, rotate velocity
                                                        navigator.pub.publish(navigator.desired_velocity)
                                                        navigator.rate.sleep()

                                                    #Trying to detect the rectangle and making it align with the center coordinates of the window frame
                                                        loc_x = navigator.detect_rectangle()
                                                        print("Loc - x", loc_x)
                                                        while (loc_x != -1 and loc_x !=-2 and loc_x != None):
                                                            if loc_x < 310:
                                                                navigator.desired_velocity.angular.z = math.pi/60
                                                                navigator.desired_velocity.linear.x = 0.0
                                                            elif loc_x >330:
                                                                navigator.desired_velocity.angular.z = -math.pi/60
                                                                navigator.desired_velocity.linear.x = 0.0
                                                            else:
                                                                navigator.desired_velocity.angular.z = 0
                                                                navigator.desired_velocity.linear.x = 0.1

                                                            navigator.pub.publish(navigator.desired_velocity)
                                                            navigator.rate.sleep()
                                                            loc_x = navigator.detect_rectangle()
                                                            print("Loc - x", loc_x)

                                                        #If the robot is close enough to the image then stopping the robot and detecting the character
                                                        while(loc_x == -2):
                                                            navigator.desired_velocity.angular.z = 0.0
                                                            navigator.desired_velocity.linear.x = 0
                                                            navigator.pub.publish(navigator.desired_velocity)
                                                            navigator.rate.sleep()
                                                            navigator.detect_image()
                                                            exit()
                                            #If robot is bumped the robot goes back and turns in other direction
                                            if navigator.bump:
                                                navigator.desired_velocity.angular.z = 0
                                                navigator.desired_velocity.linear.x = -0.2
                                                for i in range(20):
                                                    navigator.pub.publish(navigator.desired_velocity)
                                                    navigator.rate.sleep()
                                                navigator.desired_velocity.angular.z = 0.2
                                                navigator.desired_velocity.linear.x = 0
                                                for i in range (30):
                                                    navigator.pub.publish(navigator.desired_velocity)
                                                    navigator.rate.sleep()
                                                navigator.bump = 0
                                        l+=10

                                    rospy.loginfo("Hooray, reached the desired pose")
                                    theta+=0.3
                                else:
                                    rospy.loginfo("The base failed to reach the desired pose")
                                    navigator.shutdown()

                rot+=1

        #If the robot cannot find the green circle in the entrance coordinates of the first room then it moves
        #to the entrance coordinates of the next room
        if(navigator.flag_green==0):
            print("Green Flag =", navigator.flag_green)
            #entrance coordinates of the next room
            x = points['room2_entrance_xy'][0] # SPECIFY X COORDINATE HERE
            y = points['room2_entrance_xy'][1]# SPECIFY Y COORDINATE HERE
            theta = 0# SPECIFY THETA (ROTATION) HERE
            position = {'x': x, 'y' : y}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

            rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
            success = navigator.goto(position, quaternion)
            if success:
                rot=0
                while(rot<200):
                    navigator.desired_velocity.linear.x = 0.0 # making spiral, forward velocity
                    navigator.desired_velocity.angular.z = 0.4 # making spiral, angular velocity
                    navigator.pub.publish(navigator.desired_velocity)
                    navigator.rate.sleep()
                    if(len(navigator.contours_green)>0): # Detecting the green color for detecting the green circle
                        t1=0
                        colour_max_area1=0
                        areas1 = []
                        for i in range(len(navigator.contours_green)):
                            c1=navigator.contours_green[i]
                            areas1.append(cv2.contourArea(c1))
                            if areas1[i]>colour_max_area1:
                                colour_max_area1=areas1[i]
                                t1=i
                        print("Green Contour max area",colour_max_area1)


                        if(colour_max_area1>2000):  # Putting the threshold for the green circle area
                            r1=math.sqrt(areas1[t1]/3.14)
                            cnt1 = navigator.contours_green[t1]
                            M1= cv2.moments(cnt1)
                            cx1, cy1 = int(M1['m10']/M1['m00']), int(M1['m01']/M1['m00'])

                            img = cv2.cvtColor(navigator.image_b,cv2.COLOR_BGR2GRAY)
                            img = cv2.medianBlur(img,5)
                            cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
#Trying to align the circle center with the window frame center to take a better image of the green circle
                            while(cx1 < 310 or cx1 > 330 ):
                                if(310>cx1):
                                    navigator.desired_velocity.angular.z = 0.025
                                elif(330<cx1):
                                    navigator.desired_velocity.angular.z = -0.025
                                navigator.pub.publish(navigator.desired_velocity)
                                navigator.rate.sleep()
                                for i in range(len(navigator.contours_green)):
                                    c1=navigator.contours_green[i]
                                    areas1.append(cv2.contourArea(c1))
                                    if areas1[i]>colour_max_area1:
                                        colour_max_area1=areas1[i]
                                        t1=i
                                r1=math.sqrt(areas1[t1]/3.14)
                                cnt1 = navigator.contours_green[t1]
                                M1= cv2.moments(cnt1)
                                try:
                                    cx1, cy1 = int(M1['m10']/M1['m00']), int(M1['m01']/M1['m00'])
                                except ZeroDivisionError:
                                    print("Division by zero")

                            cv2.imshow('Camera_Feed',navigator.image_b)
                            # trying to detect circles for green contours
                            circles = cv2.HoughCircles(img,cv.CV_HOUGH_GRADIENT,0.1,1000,
                            param1=50,param2=30,minRadius=60,maxRadius=1200)

                            circles = np.uint16(np.around(circles))
                            for i in circles[0,:]:
                                # draw the outer circle
                                cv2.circle(navigator.image_b,(cx1,cy1),int(r1),(0,255,0),2)
                                # draw the center of the circle
                                cv2.circle(navigator.image_b,(cx1,cy1),2,(0,0,255),3)
                                cv2.imshow('Green Circle',navigator.image_b)
                                cv2.imwrite("/home/csunix/sc19gs/Desktop/green_circle.png",navigator.image_b)

                                #After detecting the right room moving to the center coordinates of that room
                                x = points['room2_centre_xy'][0] # SPECIFY X COORDINATE HERE
                                y = points['room2_centre_xy'][1]# SPECIFY Y COORDINATE HERE
                                theta = 1.5# SPECIFY THETA (ROTATION) HERE
                                navigator.flag_green=1
                                while(theta<9.5):
                                    position = {'x': x, 'y' : y}
                                    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}
                                    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
                                    success = navigator.goto(position, quaternion)


                                    if success:

                                        l=10
                                        for f in range(10000000):
                                            for y in range(4):
                                                if not navigator.bump:
                                                    navigator.desired_velocity.linear.x = 0.05  # making spiral, forward velocity
                                                    navigator.desired_velocity.angular.z = 0    # making spiral, forward velocity
                                                    for i in range (l):
                                                        if not navigator.bump:
                                                            navigator.pub.publish(navigator.desired_velocity)
                                                            navigator.rate.sleep()
                        #Trying to detect the rectangle and making it align with the center coordinates of the window frame
                                                            loc_x = navigator.detect_rectangle()
                                                            print("Loc - x", loc_x)
                                                            while (loc_x != -1 and loc_x !=-2 and loc_x != None):
                                                                if loc_x < 310:
                                                                    navigator.desired_velocity.angular.z = math.pi/60
                                                                    navigator.desired_velocity.linear.x = 0.0
                                                                elif loc_x >330:
                                                                    navigator.desired_velocity.angular.z = -math.pi/60
                                                                    navigator.desired_velocity.linear.x = 0.0
                                                                else:
                                                                    navigator.desired_velocity.angular.z = 0
                                                                    navigator.desired_velocity.linear.x = 0.1

                                                                navigator.pub.publish(navigator.desired_velocity)
                                                                navigator.rate.sleep()
                                                                loc_x = navigator.detect_rectangle()
                                                                print("Loc - x", loc_x)

                                        #If the robot is close enough to the image then stopping the robot and detecting the character
                                                            while(loc_x == -2):
                                                                navigator.desired_velocity.angular.z = 0.0
                                                                navigator.desired_velocity.linear.x = 0
                                                                navigator.pub.publish(navigator.desired_velocity)
                                                                navigator.rate.sleep()
                                                                navigator.detect_image()
                                                                exit()


                                                    for i in range (30):
                                                        if not navigator.bump:
                                                            navigator.desired_velocity.linear.x = 0.0       # making spiral, rotate velocity
                                                            navigator.desired_velocity.angular.z = 0.22     # making spiral, rotate velocity
                                                            navigator.pub.publish(navigator.desired_velocity)
                                                            navigator.rate.sleep()

                                #Trying to detect the rectangle and making it align with the center coordinates of the window frame
                                                            loc_x = navigator.detect_rectangle()
                                                            print("Loc - x", loc_x)
                                                            while (loc_x != -1 and loc_x !=-2 and loc_x != None):
                                                                if loc_x < 310:
                                                                    navigator.desired_velocity.angular.z = math.pi/60
                                                                    navigator.desired_velocity.linear.x = 0.0
                                                                elif loc_x >330:
                                                                    navigator.desired_velocity.angular.z = -math.pi/60
                                                                    navigator.desired_velocity.linear.x = 0.0
                                                                else:
                                                                    navigator.desired_velocity.angular.z = 0
                                                                    navigator.desired_velocity.linear.x = 0.1

                                                                navigator.pub.publish(navigator.desired_velocity)
                                                                navigator.rate.sleep()
                                                                loc_x = navigator.detect_rectangle()
                                                                print("Loc - x", loc_x)
                                    #If the robot is close enough to the image then stopping the robot and detecting the character
                                                            while(loc_x == -2):
                                                                navigator.desired_velocity.angular.z = 0.0
                                                                navigator.desired_velocity.linear.x = 0
                                                                navigator.pub.publish(navigator.desired_velocity)
                                                                navigator.rate.sleep()
                                                                navigator.detect_image()
                                                                exit()
                                        #If robot is bumped the robot goes back and turns in other direction
                                                if navigator.bump:
                                                    navigator.desired_velocity.angular.z = 0
                                                    navigator.desired_velocity.linear.x = -0.2
                                                    for i in range(20):
                                                        navigator.pub.publish(navigator.desired_velocity)
                                                        navigator.rate.sleep()
                                                    navigator.desired_velocity.angular.z = 0.2
                                                    navigator.desired_velocity.linear.x = 0
                                                    for i in range (30):
                                                        navigator.pub.publish(navigator.desired_velocity)
                                                        navigator.rate.sleep()
                                                    navigator.bump = 0
                                            l+=10

                                        rospy.loginfo("Hooray, reached the desired pose")
                                        theta+=0.3
                                    else:
                                        rospy.loginfo("The base failed to reach the desired pose")
                                        navigator.shutdown()

                    rot+=1

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
        navigator.shutdown()
