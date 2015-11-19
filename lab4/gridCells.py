import lab3
import rospy
import roslib

from numpy import *
from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, PoseWithCovarianceStamped, Quaternion
#from AStar import AStar
import time
from tf.transformations import euler_from_quaternion

resolution = 1
width = 0
height = 0
mapData = 0

def initGridCell():
    global openPub
    worldMapSub = rospy.Subscriber('/map', OccupancyGrid, readWorldMap)
    openPub = rospy.Publisher('/cell_path/open', GridCells, queue_size=10)

def map1Dto2D(height,width,data):
    h = height
    w = width
    map2D = [[0 for x in range(w)] for x in range(h)]
    i = 0
    for y in range(h):
	for x in range(w):
		map2D[x][y] = data[i]
		i = i + 1
    return map2D
		
	
def readWorldMap(data):
# map listener
    global mapData, grid
    global width
    global height
    global resolution
    grid = data
    mapData = data.data
    width = data.info.width
    height = data.info.height
    resolution = data.info.resolution

def publishGridCells():
    global resolution
    gridCells = GridCells()
    gridCells.header.frame_id = "/map"
    gridCells.header.stamp = rospy.Time.now()
    scale = 1
    gridCells.cell_width = resolution*scale
    gridCells.cell_height = resolution*scale
    xyscale = 1.0/(resolution)
    global openPub
    pointList = []
    for x in range(1,width):
        for y in range(1,height):
            p = Point()
            p.x = float(x/xyscale)
            p.y = float(y/xyscale)
            p.z = 0
            pointList.append(p)
    gridCells.cells = pointList
    openPub.publish(gridCells)
