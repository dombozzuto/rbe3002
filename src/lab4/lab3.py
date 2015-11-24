#!/usr/bin/env python
import lab2
import rospy
import roslib
import time
import math
import tf

from tf.transformations import euler_from_quaternion
from numpy import *
from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, PoseWithCovarianceStamped, Quaternion

import time
from tf.transformations import euler_from_quaternion

from std_msgs.msg import Empty
from kobuki_msgs.msg import BumperEvent

xInit = 0
yInit = 0
thetaInit = 0
xEnd = 0
yEnd = 0
thetaEnd = 0
totalPath=[]
aStarList = []

scale = 2

#Kobuki Dimensions
wheel_rad  = 3.5  #cm
wheel_base = 23.0 #cm

#Odometry Data Variables
xPos = 0;
yPos = 0;
theta = 0;

def readGoal(msg):
    px = msg.pose.position.x
    py = msg.pose.position.y
    quat = msg.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    global xEnd
    global yEnd
    global thetaEnd
    xEnd = px
    yEnd = py
    thetaEnd = yaw * 180.0 / math.pi

def readInitPose(initpose):
    px = initpose.pose.pose.position.x
    py = initpose.pose.pose.position.y
    quat = initpose.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    global xInit
    global yInit
    global thetaInit
    xInit = px
    yInit = py
    thetaInit = yaw * 180.0 / math.pi
    print xInit,yInit,thetaInit

	
def startCallBack(data):
    px = data.pose.pose.position.x
    py = data.pose.pose.position.y
    quat = data.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    global xInit
    global yInit
    global thetaInit
    xInit = px
    yInit = py
    thetaInit = yaw * 180.0 / math.pi


#represents instance on the 2D array of GridCells
#each cell stores its XY coordinate of the 2D array
#its 3 scores needed for the A* algorithm
#if there is an obstacle at the given location
#   (0 == No Obstacle, 100 == Obstacle, -1 == Unknown)
#cameFrom indicates which node led to this for path
#reconstruction
class Cell:
    def __init__(self,x,y, f, g, h, blocked):
        self.point = MyPoint(x,y)
        self.fScore = f
        self.gScore = g
        self.hScore = h
        self.blocked = blocked
        self.cameFrom = None

    def printCell(self):
        pass
        #print self.point.x, self.point.y, self.fScore, self.gScore, self.hScore

#defines an x,y coordinate as an object
class MyPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y

#grid map is a 2D array of a given width and height
#w and h correspond to columns and rows of 2D array
#map is the 2D array of cells
#open set and closed set represent the frontier and
#expanded nodes, respectively
class GridMap:
    # def __init__(self, width, height):
    #     self.width = width
    #     self.height = height
    #     self.openSet = []
    #     self.closedSet = []
    #     self.map = []
    #     #populate the map with cell objects
    #     self.map = [[Cell(-1,-1,-1,-1,-1, 0) for x in range(width)] for x in range(height)]
    #     #properly assign coordinates to the cell objects
    #     for y in range(self.height):
    #         for x in range(self.width):
    #             self.map[y][x] = Cell(x,y,-1,-1,-1,0)
    
    def __init__(self, width, height, data2D):
        self.width = width
        self.height = height
        self.openSet = []
        self.closedSet = []
        self.map = []
        #populate the map with cell objects
        self.map = [[Cell(-1,-1,-1,-1,-1, 0) for x in range(width)] for x in range(height)]
        #properly assign coordinates to the cell objects
        for y in range(self.height):
            for x in range(self.width):
                self.map[y][x] = Cell(x,y,-1,-1,-1,data2D[y][x])

    #update fScore to a given fScore
    def updateFScore(self, xPos, yPos, score):
        self.map[yPos][xPos].fScore = score

    #calculate new fScore and store this in fScore,
    #based on cell's gScore and hScore
    def calculateFScore(self, xPos, yPos):
        self.map[yPos][xPos].fScore = self.map[yPos][xPos].gScore + self.map[yPos][xPos].hScore

    #update a node a position x,y gScore
    def updateGScore(self, xPos, yPos, score):
        self.map[yPos][xPos].gScore = score
    #update a cell @ X,Y with new hScore
    def updateHScore(self, xPos, yPos, score):
        self.map[yPos][xPos].hScore = score

    #update all 3 scores @ once
    def updateScores(self, xPos, yPos, f, g, h):
        self.updateFScore(xPos, yPos, f)
        self.updateGScore(xPos, yPos, g)
        self.updateHScore(xPos, yPos, h)

    #formatted print of scores in grid
    def printScores(self):
        for y in range(self.height):
            for x in range(self.width):
                # print x,("[ %3d %3d %3d]" % (self.map[y][x].fScore, self.map[y][x].gScore, self.map[y][x].hScore)),
                if (self.map[y][x].fScore != 99999):
                    print ("[ %3d %3d %3d]" % (self.map[y][x].fScore, self.map[y][x].gScore, self.map[y][x].hScore)),
                #print("[",self.map[y][x].fScore, self.map[y][x].gScore, self.map[y][x].hScore, "]",
            print " "


    #formatted print of obstcles
    def printObstacles(self):
        for y in range(self.height):
            for x in range(self.width):
                print("[ %3d ]" % (self.map[y][x].blocked))
            print " "

    #formatted print of gridCells for verification
    def printCoords(self):
        for y in range(self.height):
            for x in range(self.width):
                print "[",self.map[y][x].point.x, self.map[y][x].point.y, "]",
            print " "

    #AStar search for a path from a start XY to a goal XY
    #returns a list of grid cells on successful completion
    def aStarSearch(self, startX, startY, goalX, goalY):
        #initialize open and closed sets to 0
        self.closedSet = []
        self.openSet = []

        # add the start node to the open set
        self.openSet.append(self.map[startY][startX])

        for y in range(self.height):
            for x in range(self.width):
                #initialize fScores and gScores to 'infinity'
                self.updateGScore(x,y,99999)
                self.updateFScore(x,y,99999)

                # calculate the heuristic score for each block
                # use diagonal distance
                diagonalDistance=sqrt( ((x-goalX)**2) + ((y-goalY)**2) )
                self.updateHScore(x,y, diagonalDistance)

        #set the gScore of start position to 0
        self.updateGScore(startX, startY, 0)
        self.calculateFScore(startX, startY)

        #while openSet is not empty...
        while(len(self.openSet) != 0):
            #print "Open set length: " ,len(self.openSet)
            #sort the list in order of increase fScore (lowest first)
            self.openSet.sort(key=lambda x: x.fScore)
            #pop the lowest off the open set and add it to the closed set
            currentCell = self.openSet.pop(0)
            
            for o in self.openSet:
                o.printCell()

            #print "Currently exploring:", currentCell.point.x, currentCell.point.y
            self.closedSet.append(currentCell)

            # publish currentCell as 'astar' cell to GridCells in Rviz
            pp=Point()
            pp.x=currentCell.point.x
            pp.y=currentCell.point.y
            pp.z=0
            aStarList.append(pp)
            publishGridCellList(aStarList,3)


            #if currentCell is the goal....
            if(currentCell.point.x == goalX and currentCell.point.y == goalY):
                self.reconstructPath(currentCell)
                return 0

            #check neighbors
            validNeighbors = self.getValidNeighbors(currentCell.point.x, currentCell.point.y)
            validNeighbors = validNeighbors + self.getValidDiagonalNeighbors(currentCell.point.x, currentCell.point.y)

            #expand each neighbor
            for neighbor in validNeighbors:
                continueFlag = False
                #if the neighbor hasn't been expanded yet
                if(not self.isMyPointInClosedSet(neighbor)):
                    # manhattan distance from start to currentCell
                    distToNeighbor = float(math.sqrt(pow(currentCell.point.x - neighbor.point.x, 2) + pow(currentCell.point.y - neighbor.point.y, 2)))
                    #print distToNeighbor
                    tentativeGScore = self.map[currentCell.point.y][currentCell.point.x].gScore + float(distToNeighbor) #0.5
                    # tentativeGScore = abs(x-goalX) + abs(y-goalY)
                    #add the neighbor to openset and update scores
                    if(not self.isMyPointInOpenSet(neighbor)):
                        self.openSet.append(neighbor)
                    #only update if this is a better path to the node
                    elif (tentativeGScore >= self.map[neighbor.point.y][neighbor.point.x].gScore):
                        continueFlag = True
                    
                    if(not continueFlag):
                        self.map[neighbor.point.y][neighbor.point.x].cameFrom = currentCell
                        self.updateGScore(neighbor.point.x, neighbor.point.y, tentativeGScore)
                        self.calculateFScore(neighbor.point.x, neighbor.point.y)


    

    #returns a list of the cells that needed to be visted to reach a goal
    #basically just recurse backwards through the list until you reach the 
    #first cell (which has Nothing in its camefrom field)
    def reconstructPath(self, currentCell):
        totalPath.append(currentCell)
        while(currentCell.cameFrom != None):
            currentCell = currentCell.cameFrom
            totalPath.append((currentCell))
        totalPath.reverse()
        for cell in totalPath:
            print "X:", cell.point.x, "Y:", cell.point.y


    #returns if a given point is in the closed set.
    #breaks immediately if the point is found
    #if nothing found, return false
    def isMyPointInClosedSet(self, p):
        for cell in self.closedSet:
            if(cell.point.x == p.point.x and cell.point.y == p.point.y):
                return True
        return False


    #returns if a given point is in the open set.
    #breaks immediately if the point is found
    #if nothing found, return false
    def isMyPointInOpenSet(self, p):
        for cell in self.openSet:
            if(cell.point.x == p.point.x and cell.point.y == p.point.y):
                return True
        return False


    #returns a list of valid neighbors that arent out of bounds
    #and arent blocked
    def getValidNeighbors(self, currentX, currentY):
        validNeighbors = []

        #check node above
        #make sure there's not a boundary issue
        if(currentY - 1 >= 0):
            #is this tile empty space? (== 0)
            if(self.map[currentY-1][currentX].blocked == 0):
                #print "This neighbor is marked as valid:", currentX, currentY-1
                validNeighbors.append(self.map[currentY-1][currentX]);

        #check node below
        if(currentY + 1 < self.height):
            if(self.map[currentY+1][currentX].blocked == 0):
                #print "This neighbor is marked as valid:", currentX, currentY+1
                validNeighbors.append(self.map[currentY+1][currentX]);

        #check node left
        if(currentX - 1 >= 0):
            if(self.map[currentY][currentX-1].blocked == 0):
                #print "This neighbor is marked as valid:", currentX-1, currentY
                validNeighbors.append(self.map[currentY][currentX-1]);

        #check node right
        if(currentX + 1 < self.width):
            if(self.map[currentY][currentX+1].blocked == 0):
                #print "This neighbor is marked as valid:", currentX+1, currentY
                validNeighbors.append(self.map[currentY][currentX+1]);

        return validNeighbors


    def getValidDiagonalNeighbors(self, currentX, currentY):
        validNeighbors = []
        nwBoundOK = (currentY-1 >= 0) and (currentX-1 >= 0)
        neBoundOK = (currentY-1 >= 0) and (currentX + 1 < self.width)
        swBoundOK = (currentY+1 < self.height) and (currentX-1 >= 0)
        seBoundOK = (currentY+1 < self.height) and (currentX+1 < self.width)

        if(nwBoundOK):
            if(self.map[currentY-1][currentX-1].blocked == 0):
                validNeighbors.append(self.map[currentY-1][currentX-1])

        if(neBoundOK):
            if(self.map[currentY-1][currentX+1].blocked == 0):
                validNeighbors.append(self.map[currentY-1][currentX+1])

        if(swBoundOK):
            if(self.map[currentY+1][currentX-1].blocked == 0):
                validNeighbors.append(self.map[currentY+1][currentX-1])

        if(seBoundOK):
            if(self.map[currentY+1][currentX+1].blocked == 0):
                validNeighbors.append(self.map[currentY+1][currentX+1])

        return validNeighbors

def wayPoints(path):
    wayPoints = []
    previousX = 0
    previousY = 0
    following = True
    for cell in path:
        if(following):
            if((cell.point.x == previousX) and (cell.point.y != previousY)):
                wayPoints.append(cell)
                following != following
        else:
            if((cell.point.y == previousY) and (cell.point.x != previousX)):
                wayPoints.append(cell)
                following != following
    pathList = []
    for cell in wayPoints:
        print cell.point.x, cell.point.y
        p = Point()
        p.x=cell.point.x
        p.y=cell.point.y
        p.z=0
        pathList.append(p)    
    publishGridCellList(pathList,0)
    print wayPoints
    return wayPoints

def printTotalPath():
    global resolution
    global scale
    fscale = float(scale)
    xyscale = 1/(resolution*scale)
    pathList = []
    for cell in totalPath:
        print cell.point.x, cell.point.y
        p = Point()
        p.x=cell.point.x
        p.y=cell.point.y
        p.z=0
        pathList.append(p)    
    publishGridCellList(pathList,2)
    for pnt in pathList:
        px = float((pnt.x+x0/scale)/xyscale)+1/(2*xyscale) + originx
        py = float((pnt.y+y0/scale)/xyscale)+1/(2*xyscale) + originy
        navToPosePoint(px,py)
    #wayPoints(totalPath)
    # PublishGridCellPath(totalPath)


# @typ: 0=open 1=closed 2=path
def publishGridCellPoint(pnt,typ):
    global resolution
    global scale
    gridCells = GridCells()
    gridCells.header.frame_id = "/map"
    gridCells.header.stamp = rospy.Time.now()
    gridCells.cell_width = resolution*scale
    gridCells.cell_height = resolution*scale
    xyscale = 1.0/(resolution)
    p = Point()
    p.x= float(pnt.x/xyscale)
    p.y= float(pnt.y/xyscale)
    gridCells.cells=[p]
    if(typ==0):
        openPub.publish(gridCells)
    if(typ==1):
        closedPub.publish(gridCells)
    if(typ==2):
        pathVizPub.publish(gridCells)
    if(typ==3):
        astarVizPub.publish(gridCells)

#publishes a list of Point messages as GridCells
def publishGridCellList(lst,typ):
    global resolution
    global scale
    gridCells = GridCells()
    gridCells.header.frame_id = "/map"
    gridCells.header.stamp = rospy.Time.now()
    gridCells.cell_width = resolution*scale
    gridCells.cell_height = resolution*scale
    xyscale = 1.0/(resolution*scale)
    
    pntList=[]
    for pnt in lst:
        p = Point()
        # p.x= float(pnt.x/xyscale)+1/(2*xyscale)
        # p.y= float(pnt.y/xyscale)+1/(2*xyscale)
        p.x = float((pnt.x+x0/scale)/xyscale)+1/(2*xyscale) + originx
        p.y = float((pnt.y+y0/scale)/xyscale)+1/(2*xyscale) + originy
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

    # resolution and offset of the map

    # create a new instance of the map

    # generate a path to the start and end goals

    # for each node in the path, process the nodes to generate GridCells and Path messages
  
    # transform coordinates for map resolution and offset

    # continue making messages

    # do not stop publishing

#callback for map data
def readWorldMap(data):
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

#callback for map data
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

def initGridCell():
    global openPub
    global closedPub
    global pathVizPub
    global astarVizPub
    #worldMapSub = rospy.Subscriber('/map', OccupancyGrid, readWorldMap)
    globalCostMapSub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, readGlobalCostMap)
    openPub = rospy.Publisher('/cell_path/open', GridCells, queue_size=10)
    closedPub = rospy.Publisher('/cell_path/closed', GridCells, queue_size=10)
    pathVizPub = rospy.Publisher('/cell_path/path', GridCells, queue_size=10)
    astarVizPub = rospy.Publisher('/cell_path/astar', GridCells, queue_size=10)

# takes in a 2D map and scales it
def shrinkMap(width, height, mapData2D):
    newMap = [[0 for x in range(width/scale)] for x in range(height/scale)]
    for x in range(width/scale):
        for y in range(height/scale):
            for j in range(scale):
                for k in range(scale):
                    if(mapData2D[y*scale+k][x*scale+j] > 80):
                        newMap[y][x] = 100
    return newMap

def expandWalls(width, height, mapData):

    #for each cell, if it is a wall, replace the nodes around it with
    #wall indicators
    upBound = 0
    leftBound = 0
    downBound = height-1
    rightBound = width-1
    print '-----------------------------------'
    print width, height
    print '-----------------------------------'
    #first pass expands borders with 'x' character
    for x in range(width):
        for y in range(height):
            upBoundOK = (y-1>=upBound)
            downBoundOK = (y+1<downBound)
            leftBoundOK = (x-1>=leftBound)
            rightBoundOK = (x+1<rightBound)
            print "X:", x, "Y:", y
            #if there is a wall at this position...
            if(mapData[y][x] == 100):

                # fill cardinal directions with spaces
                if(upBoundOK and (mapData[y-1][x] != 100)):
                    mapData[y-1][x] = 'x'
                if(downBoundOK and (mapData[y+1][x] != 100)):
                    mapData[y+1][x] = 'x'
                if(leftBoundOK and (mapData[y][x-1] != 100)):
                    mapData[y][x-1] = 'x'
                if(rightBoundOK and (mapData[y][x+1] != 100)):
                    mapData[y][x+1] = 'x'


                if(upBoundOK and leftBoundOK) and (mapData[y-1][x-1] != 100):
                    mapData[y-1][x-1] = 'x'
                if(downBoundOK and rightBoundOK) and (mapData[y+1][x+1] != 100):
                    mapData[y+1][x+1] = 'x'
                if(upBoundOK and rightBoundOK) and (mapData[y-1][x+1] != 100):
                    mapData[y-1][x+1] = 'x'
                if(downBoundOK and leftBoundOK) and (mapData[y+1][x-1] != 100):
                    mapData[y+1][x-1] = 'x'

    #second pass replaces x's with 100's
    for x in range(width):
        for y in range(height):
            if(mapData[y][x] == 'x'):
                mapData[y][x] = 100

    return mapData



# converts 1 d array of map data into a 2d array using height and width params
# of the given map
def map1Dto2D(width,height,data):
    map2D = [[0 for x in range(width)] for x in range(height)]
    i = 0
    for y in range(height):
        for x in range(width):
            map2D[y][x] = data[i]
            i = i + 1
    return map2D
#Gets a 2D array of map data and returns a 2D array of map data that reduces the size
#to a map that captures all the data in the minimum length dimensions
#param: width, height (of original 2D map), data2D (2D array map)
def reduceMap(width,height,data2D):

    global reducedHeight
    global reducedWidth

    global x0
    global x1
    global y0
    global y1
    #sets the extreme points to 0
    x0 = 0
    x1 = 0
    y0 = 0
    y1 = 0
    #sets extrema flags to 1 (if set to 0, means that it was found and shouldn't be checked again)
    is_x0_found = 1
    is_x1_found = 1
    is_y0_found = 1
    is_y1_found = 1
    # -1 means unexplored area
    #searches for extrema over height
    for y in range(height):
        for x in range(width):
            if(is_y0_found):
                if (data2D[y][x] != -1):
                    y0 = y
                    is_y0_found = 0
            if(is_y1_found):
                if (data2D[height-y-1][x] != -1):
                    y1 = height-y-1
                    is_y1_found = 0
    #searches for extrema over width
    for x in range(width):
        for y in range(height):
            if(is_x0_found):
                if (data2D[y][x] != -1):
                    x0 = x
                    is_x0_found = 0
            if(is_x1_found):
                if (data2D[y][width-x-1] != -1):
                    x1 = width-x-1
                    is_x1_found = 0
    #calculates the reduced height and width of the new array
    reducedHeight = abs(y1 - y0)
    reducedWidth = abs(x1 - x0)
    #create empty 2D map with reduced height and width
    map2D = [[0 for x in range(reducedWidth)] for x in range(reducedHeight)]
    #fill the 2D map with the data from the initial map, but only
    #from the appropriate cells
    for y in range(reducedHeight):
        for x in range(reducedWidth):
            map2D[y][x] = data2D[y0+y][x0+x]
    return map2D


def createOpenGrid():
    global resolution
    global scale
    gridCells = GridCells()
    gridCells.header.frame_id = "/map"
    gridCells.header.stamp = rospy.Time.now()
    gridCells.cell_width = resolution*scale
    gridCells.cell_height = resolution*scale
    xyscale = 1.0/(resolution*scale)
    pointList = []
    for x in range(1,width):
        for y in range(1,height):
            p = MyPoint()
            p.x = float(x/xyscale)
            p.y = float(y/xyscale)
            p.z = 0
            pointList.append(p)
    gridCells.cells = pointList
    openPub.publish(gridCells)

# publishes all closed cells in the given GridMap
def publishClosedCells(g):
    global resolution
    global scale
    gridCells = GridCells()
    gridCells.header.frame_id = "/map"
    gridCells.header.stamp = rospy.Time.now()
    gridCells.cell_width = resolution*scale
    gridCells.cell_height = resolution*scale
    xyscale = 1.0/(resolution*scale)
    pointList = []
    for x in range(1,width):
        for y in range(1,height):
            if(g.map[y][x].blocked == 100):
                p = Point()#float(x/xyscale),float(y/xyscale))
                p.x = float(x/xyscale)
                p.y = float(y/xyscale)
                p.z = 0
                pointList.append(p)
    gridCells.cells = pointList
    closedPub.publish(gridCells)

def publishClosedCellsShrink(map2D):
    global resolution
    global scale
    gridCells = GridCells()
    gridCells.header.frame_id = "/map"
    gridCells.header.stamp = rospy.Time.now()
    gridCells.cell_width = resolution*scale
    gridCells.cell_height = resolution*scale
    xyscale = 1.0/(resolution*scale)
    pointList = []
    for x in range(width/scale):
        for y in range(height/scale):
            if(map2D[y][x] > 80):
                p = Point()#float(x/xyscale),float(y/xyscale))
                p.x = float(x/xyscale)+1/(2*xyscale)
                p.y = float(y/xyscale)+1/(2*xyscale)
                p.z = 0
                pointList.append(p)
    gridCells.cells = pointList
    closedPub.publish(gridCells)

def publishClosedCellsReduce(map2D):
    global resolution
    global scale

    global reducedHeight
    global reducedWidth
    global x0
    global x1
    global y0
    global y1

    costThresh = 80 #when to consider it a wall

    gridCells = GridCells()
    gridCells.header.frame_id = "/map"
    gridCells.header.stamp = rospy.Time.now()
    gridCells.cell_width = resolution*scale
    gridCells.cell_height = resolution*scale
    xyscale = 1.0/(resolution*scale)
    pointList = []


    for x in range(reducedWidth/scale):
        for y in range(reducedHeight/scale):
            if(map2D[y][x] > costThresh):
                p = Point()
                p.x = float((x+x0/scale)/xyscale)+1/(2*xyscale) + originx
                p.y = float((y+y0/scale)/xyscale)+1/(2*xyscale) + originy
                p.z = 0
                pointList.append(p)
    gridCells.cells = pointList
    closedPub.publish(gridCells)

def navToPosePoint(goal_x,goal_y):
    global xPos
    global yPos
    global theta
    #print "goals x %f" %(goal_x) + "goals y %f" %(goal_y) + "theta %f" %(theta)
    init_dist_x = xPos
    init_dist_y = yPos
    init_theta = theta
    maxspeed = 0.2
    #print "init x %f" %(init_dist_x) + "init y %f" %(init_dist_y) + "init theta %f" %(init_theta)
    #This is the relative x,y needed to travel to get to goal from start
    rel_x = goal_x-init_dist_x
    rel_y = goal_y-init_dist_y
    #Calculate angle to turn
    goal_theta = math.atan2(rel_y,rel_x) * (180/3.14)
    #print "goal theta %f" %(goal_theta)
    #Calculate distance needed to travel
    distance = math.sqrt(pow(goal_x-init_dist_x,2) + pow(goal_y-init_dist_y,2))
    #|----Rotate----|
    rotate(goal_theta-init_theta)
    #|----DriveStraight----|
    driveStraight(maxspeed, distance)

#publishTwist: publishes the Twist message to the cmd_vel_mux/input/teleop topic using the given linear(u) and angular(w) velocity
def publishTwist(u,w):
    global pub
    twist = Twist()
    twist.linear.x = u; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w
    pub.publish(twist)

#odometry callback, gets odom data from subscribed topic
def odomCallback(data):
    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))
    x=position[0]
    y=position[1]
    w = orientation
    q = [w[0], w[1], w[2], w[3]]
    roll, pitch, yaw = euler_from_quaternion(q)
    global xPos
    global yPos
    global theta
    xPos = x
    yPos = y
    theta = math.degrees(yaw)
    #print xPos,yPos,theta

#driveStraight: drives thr robot forward at a desired speed for a certain amount of time
#param: maxspeed - speed limit, distance - desired travel length
def driveStraight(maxspeed, distance):
    u = maxspeed;
    w = 0;
    minspeed = 0.1 #minimum speed
    delay = 0.15
    #initial coordinates
    init_dist_x = xPos
    init_dist_y = yPos 
    d_traveled = 0 #initial distance traveled
    d_seg1 = 0.2 * distance #first 20% of distance
    d_seg2 = 0.8 * distance #first 80% of distance

    # |----seg1----|----------------seg2------------------|----seg3-----|
    # |---speedup--|------------constantspeed-------------|--slowdown---|

    while((d_traveled  < distance) and not rospy.is_shutdown()):
        #determine the distance and if you have gone far enough
        d_traveled = math.sqrt(pow(xPos-init_dist_x,2) + pow(yPos-init_dist_y,2))
        if (d_traveled < d_seg1):
            vel = (d_traveled/d_seg1)*maxspeed + minspeed
        elif (d_traveled < d_seg2):
            vel = maxspeed
        else:
            vel = ((distance - d_traveled)/(distance - d_seg2))*maxspeed + minspeed
        publishTwist(vel, w)
        time.sleep(delay)
        print "init x %f"%(init_dist_x) + "init y %f"%(init_dist_y) + "xPos %f" %xPos + "yPos %f" %yPos + "Dt %f" %d_traveled + "D %f" %distance 
    publishTwist(0, 0)

#rotate: rotates the robot around its center by a certain angle (in degrees)
#known to be buggy 
def rotate(angle):
    init_angle = theta
    desired_angle = init_angle + angle
    p = 0.025
    error = 0
    errorband = 2 #degrees
    minspeed = 20 #minimum turning speed
    delay = 0.10
    if(desired_angle < -180) or (desired_angle >= 180):
        if(angle > 0):
            desired_angle = desired_angle - 360
        else:
            desired_angle = desired_angle + 360
    #keep turning until target angle is within +- errorband 
    while(((theta > desired_angle + errorband) or (theta < desired_angle - errorband)) and not rospy.is_shutdown()):
        print "theta %f" %(theta) + " desired %f" %(desired_angle) + " error %f" %(error)
        error = theta-desired_angle #error controller
        if (error > 180 or error < -180):
            if (error > 0):
                error = (360 - error) + minspeed
            else:
                error = (error + 360) - minspeed
        else:
            if (error > 0):
                error += minspeed
            else:
                error -= minspeed
        publishTwist(0,-error*p) #publish Twist msg
        time.sleep(delay) 
    publishTwist(0, 0) #stop rotating, reached target


if __name__ == '__main__':
    rospy.init_node('lab3', anonymous=True)
    try:
        global worldMap
        global target
        global cellPub
        global scale
        global resolution
        global xInit, yInit, xEnd, yEnd
        global pose
        global odom_tf
        global odom_list
        print "Setup globals"

        odom_list = tf.TransformListener()

        print "Setup odom transformer"
        AMap = 0
        worldMap = 0
        path = 0
        scale = 4
        # rospy.init_node('lab3')
        sub = rospy.Subscriber('/odom', Odometry, odomCallback)
        pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 5)
        markerSub = rospy.Subscriber('/move_base_simple/goalrbe', PoseStamped, readGoal)
        pathPub = rospy.Publisher('/path_path', Path, queue_size = 5)
        initposeSub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, readInitPose)

        initGridCell()
        rospy.sleep(3)
        
        # allow subscriber time to callback
        filledMap = map1Dto2D(width, height, mapData)
        reducedMap = reduceMap(width, height, filledMap)

        #print len(filledMap), len(filledMap[0])
        shrinkedMap = shrinkMap(reducedWidth,reducedHeight,reducedMap)
        #shrinkedMap = shrinkMap(width,height,filledMap)
        #print len(shrinkedMap), len(shrinkedMap[0])

        #newHeight = len(shrinkedMap)
        #newWidth = len(shrinkedMap[0])

        #expandedMap = expandWalls(newWidth, newHeight, shrinkedMap)


        xyscale = 1.0/(resolution*scale)


        #publishClosedCellsReduce(reducedMap)
        publishClosedCellsReduce(shrinkedMap)
        #publishClosedCellsShrink(shrinkedMap)
        #publishClosedCellsShrink(expandedMap)
        ratio = 1.0/(resolution)
        while ((yEnd == 0) or (xEnd == 0)) and not rospy.is_shutdown():
            pass
        #g = GridMap(width/scale, height/scale,shrinkedMap)
        g = GridMap(reducedWidth/scale, reducedHeight/scale, shrinkedMap)
        #ratio = (resolution*scale)
        #print xEnd, yEnd
        print reducedWidth/scale, reducedHeight/scale
        endx = int(((-originx - 1/(2*xyscale)+xEnd)*xyscale - x0/scale))
        endy = int(((-originy - 1/(2*xyscale)+yEnd)*xyscale - y0/scale))
        initx = int(((-originx - 1/(2*xyscale)+xPos)*xyscale - x0/scale))
        inity = int(((-originy - 1/(2*xyscale)+yPos)*xyscale - y0/scale))
        print endx, endy
        #print int((xPos)*ratio/scale - originx - x0/scale),int((yPos)*ratio/scale - originy - y0/scale)
        g.aStarSearch(initx,inity,endx,endy)
        #g.printScores()
        printTotalPath()
    except rospy.ROSInterruptException:
        pass
