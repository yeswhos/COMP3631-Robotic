#!/usr/bin/env python

from __future__ import division
import cv2

import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class colourIdentifier():

	def __init__(self):
		# Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
		self.bridge = CvBridge()

		# We covered which topic to subscribe to should you wish to receive image data
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image,self.callback)
		self.sensitivity = 10
	def callback(self, data):
		# Convert the received image into a opencv image
		# But remember that you should always wrap a call to this conversion method in an exception handler
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)
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





	#Show the resultant images you have created. You can show all of them or just the end result if you wish to.
		cv2.namedWindow('Camera_Feed')
		cv2.imshow('Camera_Feed', cv_image)


		cv2.imshow('imageHSV',hsv)
		#cv2.imshow('mask',mask)
		cv2.imshow('imag1', imag1)
		cv2.imshow('imag2', imag2)
		cv2.imshow('imag3', imag3)
		cv2.waitKey(3)


# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
	# Instantiate your class
	rospy.init_node('colourIdentifier', anonymous = True)
	# And rospy.init the entire node
	cI = colourIdentifier()
	# Ensure that the node continues running with rospy.spin()



	# You may need to wrap rospy.spin() in an exception handler in case of KeyboardInterrupts
	rospy.spin()

	# Remember to destroy all image windows before closing node
	cv2.destroyAllWindows()

# Check if the node is executing in the main path
if __name__ == '__main__':
	main(sys.argv)
