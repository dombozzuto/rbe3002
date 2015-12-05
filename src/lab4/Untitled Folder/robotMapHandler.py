import math, numpy, time
import rospy, roslib, tf
import tf.transformations
import nav_msgs.msg, geometry_msgs.msg, std_msgs.msg, kobuki_msgs.msg, actionlib_msgs.msg
import robotStartup

#publishes a list of Point messages as GridCells
def publishGridCellList(lst,typ):

    global resolution
    global scale

    #create GridCells msg
    gridCells = GridCells()
    gridCells.header.frame_id = "/map"
    gridCells.header.stamp = rospy.Time.now()
    gridCells.cell_width = resolution*scale
    gridCells.cell_height = resolution*scale
    xyscale = 1.0/(resolution*scale)
    
    pntList=[]
    for pnt in lst:
        p = Point() #from geometry msgs
        p.x = float((pnt.x+x0/scale)/xyscale)+1/(2*xyscale) + originx
        p.y = float((pnt.y+y0/scale)/xyscale)+1/(2*xyscale) + originy
        p.z=0
        pntList.append(p)
    #gridcells list of points
    gridCells.cells=pntList
    if(typ==0):
        openPub.publish(gridCells)
    if(typ==1):
        closedPub.publish(gridCells)
    if(typ==2):
        pathVizPub.publish(gridCells)
    if(typ==3):
        astarVizPub.publish(gridCells)

# takes in a 2D map and scales it
def shrinkMap(width, height, mapData2D):
    newMap = [[0 for x in range(width/scale)] for x in range(height/scale)]
    for x in range(width/scale):
        for y in range(height/scale):
            for j in range(scale):
                for k in range(scale):
                    #newMap[y][x] = mapData2D[y*scale+k][x*scale+j]
                    if(mapData2D[y*scale+k][x*scale+j] > obsThresh):
                        newMap[y][x] = 100
    return newMap

#expand all the walls to 
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

def publishClosedCellsReduce(map2D):
    global resolution
    global scale

    global reducedHeight
    global reducedWidth
    global x0
    global x1
    global y0
    global y1

    gridCells = GridCells()
    gridCells.header.frame_id = "/map"
    gridCells.header.stamp = rospy.Time.now()
    gridCells.cell_width = resolution*scale
    gridCells.cell_height = resolution*scale
    xyscale = 1.0/(resolution*scale)
    pointList = []


    for x in range(reducedWidth/scale):
        for y in range(reducedHeight/scale):
            if(map2D[y][x] > obsThresh):
                p = Point()
                p.x = float((x+x0/scale)/xyscale)+1/(2*xyscale) + originx
                p.y = float((y+y0/scale)/xyscale)+1/(2*xyscale) + originy
                p.z = 0
                pointList.append(p)
    gridCells.cells = pointList
    closedPub.publish(gridCells)