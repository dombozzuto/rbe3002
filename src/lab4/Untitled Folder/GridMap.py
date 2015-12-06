from math import sqrt
from geometry_msgs.msg import Point
import robotStartup


aStarList=[]
totalPath=[]

#defines an x,y coordinate as an object
class MyPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y


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
        return totalPath


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
            robotStartup.publishGridCellList(aStarList,3)


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
                    distToNeighbor = float(sqrt(pow(currentCell.point.x - neighbor.point.x, 2) + pow(currentCell.point.y - neighbor.point.y, 2)))
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


    def astarDist(self):
        # self.aStarSearch(startX, startY, goalX, goalY)
        return len(totalPath)

    def printTotalPath(self):
        global resolution
        global scale
        global pathNotDone
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
        print "Got here Start"
        wayPointList = getWaypoints(pathList,5)
        px = float((wayPointList[1].x+x0/scale)/xyscale)+1/(2*xyscale) + originx
        py = float((wayPointList[1].y+y0/scale)/xyscale)+1/(2*xyscale) + originy
        print "wayPoint Length %f" %(len(wayPointList))
        if (len(wayPointList) > 1):
            navToPosePoint(px,py)
        else:
            pathNotDone = 0               