import rospy
#AStar search for a path from a start XY to a goal XY
    #returns a list of grid cells on successful completion
    def aStarSearch(self, startX, startY, goalX, goalY):

        startTime = rospy.Time.now()
        allottedTime = 5.0
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
        while(len(self.openSet) != 0 and (allottedTime + startTime > rospy.Time.now())):
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
        return False
