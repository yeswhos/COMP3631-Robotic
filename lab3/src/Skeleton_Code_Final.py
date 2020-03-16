#!/usr/bin/env python
# This final piece fo skeleton code will be centred around gettign the students to follow a colour and stop upon sight of another one.

from __future__ import division
import cv2
import numpy as np
import rospy
import sys
import math
from geometry_msgs.msg import Twist

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

class colourIdentifier():

	def __init__(self):
		# Initialise a publisher to publish messages to the robot base
		# We covered which topic receives messages that move the robot in the 2nd Lab Session
		self.pub = rospy.Publisher('chatter', String, queue_size=10)

		# Initialise any flags that signal a colour has been detected in view
		self.blueflag = False
		self.redflag = False
		self.greenflag = False
		# Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
		self.sensitivity = 10

		# Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)
		self.pubb = rospy.Publisher('mobile_base/commands/velocity', Twist)


		#rospy.Subscriber('mobile_base/events/bumper', BumperEvent, callback)
		rate = rospy.Rate(10) #10hz


		self.desired_velocity = Twist()

		# Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
		self.bridge = CvBridge()
		# We covered which topic to subscribe to should you wish to receive image data
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image,self.callback)

	def callback(self, data):
		# Convert the received image into a opencv image
		# But remember that you should always wrap a call to this conversion method in an exception handler
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)

		# Set the upper and lower bounds for the two colours you wish to identify
		hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
		green_lower = np.array([60 - self.sensitivity, 100, 100])
		green_upper = np.array([60 + self.sensitivity, 255, 255])
		mask = cv2.inRange(hsv,green_lower,green_upper)

		red_lower = np.array([0 - self.sensitivity, 100, 100])
		red_upper = np.array([0 + self.sensitivity, 255, 255])
		mask1 = cv2.inRange(hsv,red_lower,red_upper)

		blue_lower = np.array([120 - self.sensitivity, 100, 100])
		blue_upper = np.array([120 + self.sensitivity, 255, 255])

		mask2 = cv2.inRange(hsv,blue_lower,blue_upper)
		# Convert the rgb image into a hsv image


		# Filter out everything but particular colours using the cv2.inRange() method
		# Be sure to do this for the second or third colour as well

		# To combine the masks you should use the cv2.bitwise_or() method
		# You can only bitwise_or two image at once, so multiple calls are necessary for more than two colours

		# Apply the mask to the original image using the cv2.bitwise_and() method

		# As mentioned on the worksheet the best way to do this is to bitwise and an image with itself and pass the mask to the mask parameter
		# As opposed to performing a bitwise_and on the mask and the image.



		# Find the contours that appear within the certain colours mask using the cv2.findContours() method
		# For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
		contours, hierarchy = cv2.findContours(mask1,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
		greencontours, hierarchy = cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
		bluecontours, hierarchy = cv2.findContours(mask2,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
		# Loop over the contours
		# There are a few different methods for identifying which contour is the biggest
		# Loop throguht the list and keep track of which contour is biggest or
		# Use the max() method to find the largest contour

		# M = cv2.moments(c)
		# cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
		self.blueflag = False
		self.redflag = False
		self.greenflag = False
		for c in greencontours:
			self.greenflag = True
		for c in bluecontours:
			self.blueflag = True

		for c in contours:
			self.redflag = True
			M = cv2.moments(c)
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			cv2.drawContours(cv_image, [c], -1, (0, 255,0), 2)
			cv2.circle(cv_image, (cx, cy), 7, (255, 255, 255), -1)
			max_len = max(map(lambda x: len(x), contours))
			longest_counters = list(filter(lambda x: len(x) == max_len, contours))
			cv2.drawContours(cv_image, longest_counters, -1,(255,0,0),3)
		#Check if the area of the shape you want is big enough to be considered
		# If it is then change the flag for that colour to be True(1)
			if (self.blueflag == True or self.greenflag == True):
				self.desired_velocity.linear.x = 0
				self.desired_velocity.angular.z = 0
			else:
				self.pub.publish('reddetected')
				if cx < 340:
					self.desired_velocity.angular.z = math.pi/10
				elif cx >360:
					self.desired_velocity.angular.z = -math.pi/10
				else:
					self.desired_velocity.angular.z = 0
				if cv2.contourArea(c) > 15000:
					self.desired_velocity.linear.x = -0.2
				elif cv2.contourArea(c) < 10000:
					self.desired_velocity.linear.x = 0.2
				else:
					self.desired_velocity.linear.x = 0
			self.pubb.publish(self.desired_velocity)
		cv2.imshow('cv_image', cv_image)
		cv2.waitKey(3)


			 #<What do you think is a suitable area?>:
			# draw a circle on the contour you're identifying as a blue object as well
			# cv2.circle(<image>, (<center x>,<center y>), <radius>, <colour (rgb tuple)>, <thickness (defaults to 1)>)
			# Then alter the values of any flags

		#Check if a flag has been set for the stop message
		#if self.blueflag == True:

				# Too close to object, need to move backwards
				# linear = positive
				# angular = radius of minimum enclosing circle

				# Too far away from object, need to move forwards
				# linear = positive
				# angular = radius of minimum enclosing circle

			# self.<publisher_name>.publish(<Move>)

		# Be sure to do this for the other colour as well
		#Show the resultant images you have created. You can show all of them or just the end result if you wish to.

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
	# Instantiate your class
	rospy.init_node('colourIdentifier', anonymous = True)
	#rospy.init_node('Walker', anonymous=True)
	# And rospy.init the entire node
	#cI = colourIdentifier()init_node
	cI = colourIdentifier()
	# Ensure that the node continues running with rospy.spin()
	rospy.spin()
	# You may need to wrap it in an exception handler in case of KeyboardInterrupts
	# Remember to destroy all image windows before closing node
	cv2.destroyAllWindows()
# Check if the node is executing in the main path
if __name__ == '__main__':
	main(sys.argv)
