import math, numpy, time
import rospy, roslib, tf
import tf.transformations
import nav_msgs.msg, geometry_msgs.msg, std_msgs.msg, kobuki_msgs.msg, actionlib_msgs.msg
import robotStartup

if __name__ == '__main__':
	
	rospy.init_node('finalProject', anonymous=True)
	try:
		print "Starting"
		rospy.sleep(1)
		robotStartup.setupRobot()
	except rospy.ROSInterruptException:
		pass