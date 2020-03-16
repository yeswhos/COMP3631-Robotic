

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from math import radians
import math

bump = False
def processBumper(data):
	global bump
	if not data.state == 0:
		bump = 0
	else:
		bump = 1

def publisher():
	pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
	rospy.init_node('Walker', anonymous=True)
	rate = rospy.Rate(10) #10hz
	desired_velocity = Twist()
	rospy.Subscriber('mobile_base/events/bumper',BumperEvent,processBumper)

	#desired_velocity.linear.x = 0.2 # Forward with 0.2 m/sec.
	#desired_velocity.angular.z = math.pi/10
	while not rospy.is_shutdown():	
		global bump
		if (bump == 0):
			
		
			desired_velocity = Twist()
			desired_velocity.linear.x = 0.2# Forward with 0.2 m/sec.
			for i in range(50):
				if (bump == 1):
					desired_velocity.linear.x = 0
					#desired_velocity.angular.z = 0
					pub.publish(desired_velocity)
					rate.sleep()
				pub.publish(desired_velocity)
				rate.sleep()



			desired_velocity.linear.x = 0# Forward with 0.2 m/sec.
			desired_velocity.angular.z = radians(90)
			for i in range(10):
				if (bump == 1):
					desired_velocity.linear.x = 0
					#desired_velocity.angular.z = 0
					pub.publish(desired_velocity)
					rate.sleep()
				pub.publish(desired_velocity)
				rate.sleep()
				
	

if __name__ == "__main__":
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
