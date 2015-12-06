import math, numpy, time
import rospy, roslib, tf
import tf.transformations
import nav_msgs.msg, geometry_msgs.msg, std_msgs.msg, kobuki_msgs.msg, actionlib_msgs.msg
import robotStartup

#publishTwist: publishes the Twist message to the cmd_vel_mux/input/teleop topic using the given linear(u) and angular(w) velocity
def publishTwist(u,w):
    odomPub = robotStartup._getOdomPub()
    twist = geometry_msgs.msg.Twist()
    twist.linear.x = u; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w
    odomPub.publish(twist)

#rotate: rotates the robot around its center by a certain angle (in degrees)
def rotate(angle):
	theta = robotStartup._getTheta()
	init_angle = theta
	desired_angle = init_angle + angle
	p = 0.01
	error = 0
	errorband = 2 #degrees
	minspeed = 35 #minimum turning speed
	delay = 0.15
	if(desired_angle < -180) or (desired_angle >= 180):
		if(angle > 0):
			desired_angle = desired_angle - 360
		else:
			desired_angle = desired_angle + 360
	#keep turning until target angle is within +- errorband 
	while(((theta > desired_angle + errorband) or (theta < desired_angle - errorband)) and not rospy.is_shutdown()):
		theta = robotStartup._getTheta()
		error = theta-desired_angle #error controller
		print "theta %f" %(theta) + " desired %f" %(desired_angle) + " error %f" %(error)
		if (error > 180 or error < -180):
			if (error > 0):
			    error = (360 - error) - minspeed
			else:
			    error = (error + 360) + minspeed
		else:
			if (error > 0):
				error += minspeed
			else:
				error -= minspeed
		publishTwist(0,-error*p) #publish Twist msg
		time.sleep(delay) 
	publishTwist(0, 0) #stop rotating, reached target