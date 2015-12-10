import math, numpy, time
import rospy, roslib, tf
import tf.transformations
import nav_msgs.msg, geometry_msgs.msg, std_msgs.msg, kobuki_msgs.msg, actionlib_msgs.msg
import Node

import RobotMap
import GridMap
#from nav_msgs.msg import GridCells, Path
#from geometry_msgs.msg import 
#from nav_msgs.msg import GridCells
#contains all the files needed to startup the robot
#publishes a list of Point messages as GridCells

scale = 1.0

#publishes a list of Point messages as GridCells
def publishGridCellList(lst,typ):
    global resolution
    global scale
    gridCells = nav_msgs.msg.GridCells()
    gridCells.header.frame_id = "/map"
    gridCells.header.stamp = rospy.Time.now()
    gridCells.cell_width = resolution*scale
    gridCells.cell_height = resolution*scale
    xyscale = 1.0/(resolution*scale)
    
    pntList=[]
    for pnt in lst:
        p = geometry_msgs.msg.Point()
        # p.x= float(pnt.x/xyscale)+1/(2*xyscale)
        # p.y= float(pnt.y/xyscale)+1/(2*xyscale)
        p.x = float((pnt.x)/xyscale)+1/(2*xyscale) + originx
        p.y = float((pnt.y)/xyscale)+1/(2*xyscale) + originy
        p.z=0
        pntList.append(p)

    gridCells.cells=pntList
    if(typ==0):
        openPub.publish(gridCells)
    if(typ==1):
        closedPub.publish(gridCells)
    if(typ==2):
        pathVizPub.publish(gridCells)
    if(typ==3):
        astarVizPub.publish(gridCells)

def publishGridCellNodes(lst,typ):
    global resolution
    global scale
    gridCells = nav_msgs.msg.GridCells()
    gridCells.header.frame_id = "/map"
    gridCells.header.stamp = rospy.Time.now()
    gridCells.cell_width = resolution*scale
    gridCells.cell_height = resolution*scale
    xyscale = 1.0/(resolution*scale)
 
    pntList=[]
    for pnt in lst:
        p = geometry_msgs.msg.Point()
        # p.x= float(pnt.x/xyscale)+1/(2*xyscale)
        # p.y= float(pnt.y/xyscale)+1/(2*xyscale)
        p.x = float((pnt.xPos)/xyscale)+1/(2*xyscale) + originx
        p.y = float((pnt.yPos)/xyscale)+1/(2*xyscale) + originy
        p.z=0
        pntList.append(p)


    gridCells.cells=pntList
    if(typ==0):
        openPub.publish(gridCells)
    if(typ==1):
        closedPub.publish(gridCells)
    if(typ==2):
        pathVizPub.publish(gridCells)
    if(typ==3):
        astarVizPub.publish(gridCells)
    if(typ==4):
    	frontierPub.publish(gridCells)

def setupRobot():
	print "Setting up Misc"
	setupMisc()
	print "Setting up Publishers"
	setupPublishers()
	print "Setting up Subscribers"
	setupSubscribers()
	print "Sleeping for 2 seconds"
	rospy.sleep(2)
	# print "Trying to rotate"
	# initialRotate(45)

	startTime = time.time()
	filledMap = RobotMap.map1Dto2D(width, height, mapData)
	shrinkedMap = RobotMap.shrinkMap(width, height,filledMap)
	doneTime = time.time()	

	global gridMap
	gridMap = GridMap.GridMap(width, height, shrinkedMap)
	# gridMap.aStarSearch(0,0,4,4)



def getGridMap():
	return gridMap
    

def initialRotate(degsToRotate):
	global xPos
	global yPos
	global theta
	print "Current angle is:", theta
	print "Want to rotate to:", theta + degsToRotate
	radsToRotate = (degsToRotate + theta) * math.pi / 180.0
	newPose = geometry_msgs.msg.PoseStamped()
	newPose.header.frame_id = "/map"
	newPose.header.stamp = rospy.Time.now()
	newPose.pose.position.x = xPos
	newPose.pose.position.y = yPos
	newPose.pose.position.z = 0
	quaternion = tf.transformations.quaternion_from_euler(0,0,radsToRotate)
	newPose.pose.orientation.x = quaternion[0]
	newPose.pose.orientation.y = quaternion[1]
	newPose.pose.orientation.z = quaternion[2]
	newPose.pose.orientation.w = quaternion[3]
	navStackPub.publish(newPose)

def goToNode(node,xyscale,originx,originy):
	global xPos
	global yPos
	global theta

	newPose = geometry_msgs.msg.PoseStamped()
	newPose.header.frame_id = "/map"
	newPose.header.stamp = rospy.Time.now()
	newPose.pose.position.x = float(((node.xPos))/xyscale)+1/(2*xyscale) + originx
	newPose.pose.position.y = float(((node.yPos))/xyscale)+1/(2*xyscale) + originy
	newPose.pose.position.z = 0
	quaternion = tf.transformations.quaternion_from_euler(0,0,0)
	newPose.pose.orientation.x = quaternion[0]
	newPose.pose.orientation.y = quaternion[1]
	newPose.pose.orientation.z = quaternion[2]
	newPose.pose.orientation.w = quaternion[3]
	navStackPub.publish(newPose)

#setup the robots publishers
def setupPublishers():
	global openPub
	global closedPub
	global pathVizPub
	global astarVizPub
	global frontierPub
	global navStackPub
	global pathPub

	#setup all gridCell related publishers
	openPub = rospy.Publisher('/cell_path/open', nav_msgs.msg.GridCells, queue_size=10)
	closedPub = rospy.Publisher('/cell_path/closed', nav_msgs.msg.GridCells, queue_size=10)
	pathVizPub = rospy.Publisher('/cell_path/path', nav_msgs.msg.GridCells, queue_size=10)
	astarVizPub = rospy.Publisher('/cell_path/astar', nav_msgs.msg.GridCells, queue_size=10)
	frontierPub = rospy.Publisher('/cell_path/frontier', nav_msgs.msg.GridCells, queue_size=10)

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
    global map2D

    grid = data
    mapData = data.data
    width = data.info.width
    height = data.info.height
    resolution = data.info.resolution
    originx = data.info.origin.position.x
    originy = data.info.origin.position.y
    map2D = RobotMap.map1Dto2D(width, height, mapData)

#returns 2D map
def get2DMap():
	return [map2D, width, height, resolution, originx, originy]

def getNodeMap():
	nodeList = []
	for y in range(height):
		for x in range(width):
			# map2D = RobotMap.map1Dto2D(width, height, mapData)
			nodeList.append(Node.Node(x,y,map2D[y][x],width))

	return [nodeList, width, height, resolution, originx, originy]

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

def getPose():
	return [xPos, yPos, theta]

def readBaseStatus(data):
	return 0
