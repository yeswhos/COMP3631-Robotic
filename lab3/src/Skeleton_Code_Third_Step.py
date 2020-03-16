#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import rospy
import sys
import collections
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

		# Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
		self.sensitivity = 10

		# Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)

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

		imag2 = cv2.bitwise_or(mask, mask2)
		imag3 = cv2.bitwise_or(imag2, mask1)
		imag1 = cv2.bitwise_and(cv_image, cv_image, mask = imag3)
		# Convert the rgb image into a hsv image


		# Filter out everything but particular colours using the cv2.inRange() method


		# To combine the masks you should use the cv2.bitwise_or() method
		# You can only bitwise_or two image at once, so multiple calls are necessary for more than two colours


		# Apply the mask to the original image using the cv2.bitwise_and() method
		# As mentioned on the worksheet the best way to do this is to bitwise and an image with itself and pass the mask to the mask parameter
		# As opposed to performing a bitwise_and on the mask and the image.



		# Find the contours that appear within the certain colours mask using the cv2.findContours() method
		# For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE

		#ret, binary = cv2.threshold(imag3,127,255,cv2.THRESH_BINARY)
		contours, hierarchy = cv2.findContours(mask2,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

		#cv2.drawContours(cv_image,contours,-1,(0,0,255),3)
		#cv2.imshow('cv_image',cv_image)
		#cv2.waitKey(0)
		# Loop over the contours
		# There are a few different methods for identifying which contour is the biggest
		# Loop throguht the list and keep track of whioch contour is biggest or
		# Use the max() method to find the largest contour
		# M = cv2.moments(c)
		# cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
		print(len(contours))
		for c in contours:
			M = cv2.moments(c)
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			cv2.drawContours(cv_image, [c], -1, (0, 255,0), 2)
			cv2.circle(cv_image, (cx, cy), 7, (255, 255, 255), -1)
			#cv2.putText(cv_image, "center", (cx - 20, cy - 20),
			#cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

			max_len = max(map(lambda x: len(x), contours))
			longest_counters = list(filter(lambda x: len(x) == max_len, contours))
			cv2.drawContours(cv_image, longest_counters, -1,(0,0,255),3)

		#Check if the area of the shape you want is big enough to be considered
		# If it is then change the flag for that colour to be True(1)
			#colour_max_area = max(contours, key = cv2.contourArea)
			#print(colour_max_area)
			#print(contours)
			if cv2.contourArea(c) > 100:
				self.blueflag = True
				print('true')
			else:
				self.blueflag = False
		if self.blueflag:
			self.pub.publish('bluedetect')
		cv2.imshow('cv_image', cv_image)
		cv2.waitKey(3)

				 #<What do you think is a suitable area?>:
			# draw a circle on the contour you're identifying as a blue object as well
			# cv2.circle(<image>,(<center x>,<center y>),<radius>,<colour (rgb tuple)>,<thickness (defaults to 1)>)
			# Then alter the values of any flags


		#Show the resultant images you have created. You can show all of them or just the end result if you wish to.

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
	# Instantiate your class
	rospy.init_node('colourIdentifier', anonymous = True)
	# And rospy.init the entire node
	cI = colourIdentifier()init_node
	# Ensure that the node continues running with rospy.spin()
	# You may need to wrap it in an exception handler in case of KeyboardInterrupts
	rospy.spin()
	# Remember to destroy all image windows before closing node
	cv2.destroyAllWindows()

# Check if the node is executing in the main path
if __name__ == '__main__':
	main(sys.argv)
