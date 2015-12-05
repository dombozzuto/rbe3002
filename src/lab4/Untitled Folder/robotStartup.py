import math, numpy, time
import rospy, roslib, tf
import tf.transformations
import nav_msgs.msg, geometry_msgs.msg, std_msgs.msg, kobuki_msgs.msg, actionlib_msgs.msg
#from nav_msgs.msg import GridCells, Path
#from geometry_msgs.msg import 
#from nav_msgs.msg import GridCells
#contains all the files needed to startup the robot

def setupRobot():
	print "Setting up Misc"
	setupMisc()
	print "Setting up Publishers"
	setupPublishers()
	print "Setting up Subscribers"
	setupSubscribers()
	print "Sleeping for 2 seconds"
	rospy.sleep(2)
	print "Trying to rotate"
	initialRotate(45)
    
    

def initialRotate(degsToRotate):
	global xPos
	global yPos
	global theta
	print "Current angle is:", theta
	print "Want to rotate to:", theta + degsToRotate
	radsToRotate = degsToRotate * math.pi / 180
	newPose = geometry_msgs.msg.PoseStamped()
	newPose.header.frame_id = "/map"
	newPose.header.stamp = rospy.Time.now()
	newPose.pose.position.x = xPos
	newPose.pose.position.y = yPos
	#newPose.pose.orientation.z = (theta * math.pi / 180) - radsToRotate
	#newPose.pose.orientation.z = 1
	navStackPub.publish(newPose)
	rospy.sleep(1)

#setup the robots publishers
def setupPublishers():
	global openPub
	global closedPub
	global pathVizPub
	global astarVizPub
	global navStackPub
	global pathPub

	#setup all gridCell related publishers
	openPub = rospy.Publisher('/cell_path/open', nav_msgs.msg.GridCells, queue_size=10)
	closedPub = rospy.Publisher('/cell_path/closed', nav_msgs.msg.GridCells, queue_size=10)
	pathVizPub = rospy.Publisher('/cell_path/path', nav_msgs.msg.GridCells, queue_size=10)
	astarVizPub = rospy.Publisher('/cell_path/astar', nav_msgs.msg.GridCells, queue_size=10)

	#publishes path to RVz for robot path
	pathPub = rospy.Publisher('/path_path', nav_msgs.msg.Path,queue_size = 5)

	#setup a publisher to the nav stack
	navStackPub = rospy.Publisher('/move_base_simple/goal', geometry_msgs.msg.PoseStamped, queue_size=10)

#setup the robot's subsribers
def setupSubscribers():

	#read move base status
	baseStatusSub = rospy.Subscriber('/move_base/status', actionlib_msgs.msg.GoalStatusArray, readBaseStatus)

	#read in information from global costmap
	globalCostMapSub = rospy.Subscriber('/move_base/global_costmap/costmap', nav_msgs.msg.OccupancyGrid, readGlobalCostMap)

	#subscribe to odometry data
	odomSub = rospy.Subscriber('/odom', nav_msgs.msg.Odometry, odomCallback)


#dump anything that isnt a pub/sub to setup here
def setupMisc():
	global odom_list
	odom_list = tf.TransformListener()

#####################################################
 #                                                 #
## These may be able to be moved out of here later ##
 #                                                 #
#####################################################

#callback
def readGlobalCostMap(data):
# map listener
    global mapData, grid
    global width
    global height
    global resolution
    global originx
    global originy
    grid = data
    mapData = data.data
    width = data.info.width
    height = data.info.height
    resolution = data.info.resolution
    originx = data.info.origin.position.x
    originy = data.info.origin.position.y

#odom callback
def odomCallback(data):
    #global odom_list
    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))
    x=position[0]
    y=position[1]
    w = orientation
    q = [w[0], w[1], w[2], w[3]]
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(q)
    global xPos
    global yPos
    global theta
    xPos = x
    yPos = y
    theta = math.degrees(yaw)

def readBaseStatus(data):
	print "nothing to see here"
