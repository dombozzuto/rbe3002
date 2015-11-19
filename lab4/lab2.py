#!/usr/bin/env python

#Author: Robert Edwards
#edited: Joe St. Germain
#Lab 2 Code

#Imports

import rospy
import roslib

#roslib.load_manifest('lab2')

import time
import math
import numpy
import tf

from tf.transformations import euler_from_quaternion

#Message Types
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion

from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from kobuki_msgs.msg import BumperEvent

#Kobuki Dimensions
wheel_rad  = 3.5  #cm
wheel_base = 23.0 #cm

#Odometry Data Variables
xPos = 0;
yPos = 0;
theta = 0;

init_xPos = 2;
init_yPos = 2;
init_theta = 0;

#Bumper State
bumperState = 0;
bumperBumper = 4;

   
#gets the goal data and nav
def nav():
    sub = rospy.Subscriber('/move_base_simple/goalrbe', PoseStamped, navToPose)
    initposeSub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, readInitPose)

#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
    global xPos
    global yPos
    global theta
    goal_x = goal.pose.position.x
    goal_y = goal.pose.position.y
    #print "goals x %f" %(goal_x) + "goals y %f" %(goal_y)
    quat = goal.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    goal_orientation = yaw * (180/3.14)
    init_dist_x = xPos
    init_dist_y = yPos
    init_theta = theta
    rel_x = goal_x-init_dist_x
    rel_y = goal_y-init_dist_y
    goal_theta = math.atan2(rel_y,rel_x) * (180/3.14)
    print goal_orientation
    #print "goal theta %f" %(goal_theta)
    distance = math.sqrt(pow(goal_x-init_dist_x,2) + pow(goal_y-init_dist_y,2))
    #print "spin!"
    rotate(goal_theta-init_theta)
    #print "move!"
    #print "distance %f" %(distance)
    driveStraight(0.2, distance)
    #print "spin!" 
    rotate(goal_orientation-goal_theta)
    #print "done"
    print xPos,yPos,theta

def navToPosePoint(goal_x,goal_y):
    global xPos
    global yPos
    global theta
    #print "goals x %f" %(goal_x) + "goals y %f" %(goal_y)
    init_dist_x = xPos
    init_dist_y = yPos
    init_theta = theta
    rel_x = goal_x-init_dist_x
    rel_y = goal_y-init_dist_y
    goal_theta = math.atan2(rel_y,rel_x) * (180/3.14)
    #print "goal theta %f" %(goal_theta)
    distance = math.sqrt(pow(goal_x-init_dist_x,2) + pow(goal_y-init_dist_y,2))
    #print "spin!"
    rotate(goal_theta-init_theta)
    driveStraight(0.2, distance)
    print xPos,yPos,theta

def readInitPose(initpose):
    px = initpose.pose.pose.position.x
    py = initpose.pose.pose.position.y
    quat = initpose.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    global init_xPos
    global init_yPos
    global init_theta
    init_xPos = px
    init_yPos = py
    init_theta = yaw * 180.0 / math.pi
    print init_xPos, init_yPos, init_theta


#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
	driveStraight(0.2, 0.6)
	print "done straight"
	rotate(90)
	print "done rotate"
	driveStraight(0.2, 0.45)
	print "done straight 2"
	rotate(-135)
	print "done rotate 2"

#publishTwist: publishes the Twist message to the cmd_vel_mux/input/teleop topic using the given linear(u) and angular(w) velocity
def publishTwist(u,w):
    global pub
    twist = Twist()
    twist.linear.x = u; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w
    pub.publish(twist)

#spinWheels: uses the two wheel speeds to drive the robot for a certain amount of time
def spinWheels(u1, u2, timesec):
    u = (u1+u2)/2 #determine the linear velocity
    w = (u1-u2)/(wheel_base) #determine the angular velocity
    start = time.time()
    while(time.time() - start < timesec):
	publishTwist(u, w)

#getOdomData: creates the subscriber for the odometry data
def getBumpData():
    sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumpCallback) #adjust the topic

#OdomCallback: pulls the odometry data and finds the x position, y position, and angular displacement of the robot
def bumpCallback(data):
    global bumperState
    global bumperBumper
    bumperBumper = data.bumper
    bumperState = data.state

#getOdomData: creates the subscriber for the odometry data
# def getOdomData():
#     sub = rospy.Subscriber('/odom', Odometry, odomCallback)

#OdomCallback: pulls the odometry data and finds the x position, y position, and angular displacement of the robot
# def odomCallback(data):
#     px = data.pose.pose.position.x
#     py = data.pose.pose.position.y
#     quat = data.pose.pose.orientation
#     q = [quat.x, quat.y, quat.z, quat.w]
#     roll, pitch, yaw = euler_from_quaternion(q)
#     global xPos
#     global yPos
#     global theta
#     xPos = px
#     yPos = py
#     theta = yaw * (180/3.14)   #Determine theta. Hint: convert yaw to theta

def odomCallback(data):
    global pose
    #pose = Pose()
    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))
    x=position[0]
    y=position[1]
    w = orientation
    q = [w[0], w[1], w[2], w[3]]
    roll, pitch, yaw = euler_from_quaternion(q)
    #convert yaw to degrees
    global xPos
    global yPos
    global theta
    xPos = x
    yPos = y
    theta = math.degrees(yaw)
    #print xPos,yPos,theta

#driveStraight: drives thr robot forward at a desired speed for a certain amount of time
def driveStraight(maxspeed, distance):
    u = maxspeed;
    w = 0;
    minspeed = 0.05
    init_dist_x = xPos
    init_dist_y = yPos
    d_traveled = 0
    d_seg1 = 0.2 * distance
    d_seg2 = 0.8 * distance	

    while(d_traveled  < distance):
	#determine the distance and if you have gone far enough
	d_traveled = math.sqrt(pow(xPos-init_dist_x,2) + pow(yPos-init_dist_y,2))
	if (d_traveled < d_seg1):
		vel = (d_traveled/d_seg1)*maxspeed + minspeed
		print "d_seg1"
	elif (d_traveled < d_seg2):
		vel = maxspeed
		print "d_seg2"
	else:
		vel = ((distance - d_traveled)/(distance - d_seg2))*maxspeed + minspeed
	publishTwist(vel, 0)
	time.sleep(0.15)
	print "X %f" %(xPos) + ", Y %f" %(yPos) + ", V %f" %(vel)
    publishTwist(0, 0)
		

#rotate: rotates the robot around its center by a certain angle (in degrees)
#known to be buggy 
def rotate(angle):
    init_angle = theta
    print "%f" % (init_angle)
    desired_angle = init_angle + angle
    errorband = 1.5	
    if(desired_angle < -180) or (desired_angle >= 180):
        if(angle > 0):
            desired_angle = desired_angle - 360
        else:
            desired_angle = desired_angle + 360
    if(angle < 0):
        while(theta > desired_angle + errorband) or (theta < desired_angle - errorband):
            publishTwist(0,-0.25)
            print "%f" %(xPos) + ", %f" %(yPos) + ", %f" %(theta)
            time.sleep(0.10) 
    else:
        while(theta > desired_angle + errorband) or (theta < desired_angle - errorband):
            publishTwist(0,0.25)
            print "%f" %(xPos) + ", %f" %(yPos) + ", %f" %(theta)
        time.sleep(0.10)
    publishTwist(0, 0)

#driveArc: drives the robot in an arc with a given radius, linear velocity and angle of rotation
def driveArc(radius, speed, angle):
	v = speed
	r = radius
	w = v/r
	u1 = w * (r + wheel_base/2)
	u2 = w * (r - wheel_base/2)
	timesec = 0.1
	y_rot = r * math.cos(theta) + yPos
	x_rot = -r * math.sin(theta) + xPos
	init_x = xPos
	init_y = yPos
	C = math.sqrt(pow(x_rot-init_x,2) + pow(y_rot-init_y,2))
	B = 0
	A = 0
	current_angle = 0
	target_angle = angle-2 
	while ((current_angle < target_angle) and not rospy.is_shutdown()):
		B = math.sqrt(pow(xPos-init_x,2) + pow(yPos-init_y,2))
		A = math.sqrt(pow(x_rot-xPos,2) + pow(y_rot-yPos,2))
		current_angle = math.acos((pow(A,2)-pow(B,2)+pow(C,2))/(2*A*C))*(180/3.14)
		spinWheels(u1,u2,timesec)
		print "C angle %f" %(current_angle)
	publishTwist(0,0)
	

# #main code
# if __name__ == '__main__' :
#     rospy.init_node('lab2', anonymous=True)
#     try:
#         global pose
#         global odom_tf
#         global odom_list
#         odom_list = tf.TransformListener()
#         getOdomData()
#         nav()
#         pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 5)
# #	while(not(bumperBumper == 1 and bumperState == 1)): # and not rospy.is_shutdown()
# #	    getBumpData()
# #	    print "still getting bumper"
#         print "xPos, yPos"
#         time.sleep(1)       
#         while not rospy.is_shutdown():
#             #getOdom()
# #	executeTrajectory()
# 		#driveArc(0.5,0.1,180)
# #		driveStraight(0.3,1)
# #	nav()
#             continue
# #	print "not doing it"
#     except rospy.ROSInterruptException:
# 	pass

