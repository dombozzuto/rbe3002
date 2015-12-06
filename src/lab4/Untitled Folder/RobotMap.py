
AMap = 0
worldMap = 0
path = 0
pathNotDone = 1
scale =1

obsThresh = 99
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
