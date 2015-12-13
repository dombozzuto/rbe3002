






class Node:
    threshold = 70  # checked value to see if Node is likely an obstacle or not
    unknown = -1    # checked value to see if Node has an unkown obstacle value
    extended = 110  # checked value to see if Node was not an obstacle but was expanded to be one

    def __init__(self, X, Y, prob, mapwidth):
        self.xPos = X
        self.yPos = Y
        self.obs = prob
        self.parent = None
        self.cost = 1000000
        self.width = mapwidth

    # This function prints the contents of a Node.
    # The format is "(xPos,ypos)".
    # The complete string is printed and is followed by a comma so that a new
    # line is not printed.
    def printNode(self):
        print "(" + str(self.xPos) + "," + str(self.yPos) + "," + str(self.obs) + ")" ,

    def getNodeFromGrid(self, x, y, grid):
        if x >= 0 and x < self.width and y >= 0 and y < (len(grid)/self.width):
            return grid[int(x) + int(y * self.width)]
        else:
            return None


    # INPUT -> list of nodes
    # OUTPUT -> list of nodes
    def getNeighbors(self, grid):
        height = len(grid) / self.width
        #print height
        neighbors = []
        x = self.xPos-1
        y = self.yPos-1
        while x <= self.xPos+1:
            while y <= self.yPos+1:
                if (0 <= x and x <= self.width) and (0 <= y and y <= height):
                    temp = self.getNodeFromGrid(x, y, grid)
                    if temp != None:
                        if not (temp == self) and temp.obs < Node.threshold and temp.obs != Node.unknown:
                            neighbors.append(temp)
                y += 1
            x += 1
            y = self.yPos-1

        return neighbors

    def getStraightNeighbors(self, grid):
        height = len(grid) / self.width
        #print height
        neighbors = []

        x = self.xPos-1
        y = self.yPos
        if (0 <= x and x <= self.width) and (0 <= y and y <= height):
            temp = self.getNodeFromGrid(x, y, grid)
            if temp != None:
                if not (temp == self) and temp.obs < Node.threshold:
                    neighbors.append(temp)

        x = self.xPos+1
        y = self.yPos
        if (0 <= x and x <= self.width) and (0 <= y and y <= height):
            temp = self.getNodeFromGrid(x, y, grid)
            if temp != None:
                if not (temp == self) and temp.obs < Node.threshold:
                    neighbors.append(temp)

        x = self.xPos
        y = self.yPos-1
        if (0 <= x and x <= self.width) and (0 <= y and y <= height):
            temp = self.getNodeFromGrid(x, y, grid)
            if temp != None:
                if not (temp == self) and temp.obs < Node.threshold:
                    neighbors.append(temp)

        x = self.xPos
        y = self.yPos+1
        if (0 <= x and x <= self.width) and (0 <= y and y <= height):
            temp = self.getNodeFromGrid(x, y, grid)
            if temp != None:
                if not (temp == self) and temp.obs < Node.threshold:
                    neighbors.append(temp)
        
        ###############
        ## diagonals ##
        ###############

        x = self.xPos-1
        y = self.yPos-1
        if (0 <= x and x <= self.width) and (0 <= y and y <= height):
            temp = self.getNodeFromGrid(x, y, grid)
            if temp != None:
                if not (temp == self) and temp.obs < Node.threshold:
                    neighbors.append(temp)

        x = self.xPos+1
        y = self.yPos+1
        if (0 <= x and x <= self.width) and (0 <= y and y <= height):
            temp = self.getNodeFromGrid(x, y, grid)
            if temp != None:
                if not (temp == self) and temp.obs < Node.threshold:
                    neighbors.append(temp)

        x = self.xPos+1
        y = self.yPos-1
        if (0 <= x and x <= self.width) and (0 <= y and y <= height):
            temp = self.getNodeFromGrid(x, y, grid)
            if temp != None:
                if not (temp == self) and temp.obs < Node.threshold:
                    neighbors.append(temp)

        x = self.xPos-1
        y = self.yPos+1
        if (0 <= x and x <= self.width) and (0 <= y and y <= height):
            temp = self.getNodeFromGrid(x, y, grid)
            if temp != None:
                if not (temp == self) and temp.obs < Node.threshold:
                    neighbors.append(temp)

        return neighbors

    # INPUT -> two Nodes
    # OUTPUT -> boolean, are the two Nodes adjacent
    def adjacent(self, n1):
        return abs(self.xPos - n1.xPos) <= 1 and abs(self.yPos - n1.yPos) <= 1 and self != n1

    # technique from http://stackoverflow.com/questions/390250/elegant-ways-to-support-equivalence-equality-in-python-classes
    def __eq__(self, other):
        if isinstance(other, self.__class__):
            if self.xPos == other.xPos and self.yPos == other.yPos:
                return True
            else:
                return False
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)