#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from kobuki_msgs.msg import BumperEvent
import math

#callback+importbumperevent

		
def publisher():
	pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
	
	rospy.init_node('Walker', anonymous=True)
	#rospy.Subscriber('mobile_base/events/bumper', BumperEvent, callback)
	rate = rospy.Rate(10) #10hz
	
	while not rospy.is_shutdown():
		desired_velocity = Twist()
		desired_velocity.linear.x = 0.2 # Forward with 0.2 m/sec.
		desired_velocity.angular.z = -math.pi/10
		
		
		
		pub.publish(desired_velocity)
	
		
		#pubb.publish('mobile_base/events/bumper')
		
		rate.sleep()

#def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)

    #rospy.Subscriber('mobile_base/events/bumper', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()
    
if __name__ == "__main__":
	#listener()
	#Subscriber()
	
	try:
		publisher()
		
		#rospy.Subscriber('chatter', String, callback)ostopic info /mobile_base/events/bumper
	except rospy.ROSInterruptException:
		pass
