import math, numpy, time, random
import rospy, roslib, tf
import tf.transformations
import nav_msgs.msg, geometry_msgs.msg, std_msgs.msg, kobuki_msgs.msg, actionlib_msgs.msg
import robotStartup
import Node
import GridMap
import subprocess



def d(node,goal):
	dx = abs(node.xPos - goal.xPos)
	dy = abs(node.yPos - goal.yPos)
	return math.sqrt(abs(node.xPos - goal.xPos)**2 + abs(node.yPos - goal.yPos)**2)

def findFrontiers(allnodes,width,height,resolution,originx,originy):
	unknownFrontierNoDist = []
	unknownFrontier = []
	
	# for node in allnodes:
	# 	if (not (node.obs == -1)) and (node.obs <50):
	# 		print node.printNode()

	for node in allnodes:                   # For each node on the map...
		if node.obs == -1:                  # if the node is unkown space
			# print node.obs
			# obtain the neighbors of that node
			nodeNeighbors = node.getStraightNeighbors(allnodes)
			frontierFlag = 0
			for neigh in nodeNeighbors:
				if (neigh.obs > -1) and (neigh.obs < 25):
					frontierFlag = 1
			if frontierFlag == 1:
				unknownFrontierNoDist.append(node)

	[x,y,w]=robotStartup.getPose()
	xyscale = 1.0/(resolution*1.0)
	currentNode=Node.Node(x,y,0,width)

	frontierGroup=[]
	frontierGroup.append([])


	# for each node in the frontier
	for node in unknownFrontierNoDist:
		leng=0
		#find all neighbors
		nodeNeighbors = node.getStraightNeighbors(allnodes)
		frontierNeigh=[]
		#for each node in the neighbors of frontier node
		flag=0
		for neigh in nodeNeighbors:
			for frontierSection in frontierGroup:
				if neigh in frontierSection:
					if flag == 0:
						leng=leng+1
						frontierSection.append(node)
						flag=1
		if flag == 0:
			l=[];l.append(node)
			frontierGroup.append(l)
	print len(frontierGroup)

	#Minimum Frontier Size
	for frontierSection in frontierGroup:
		print len(frontierSection)
		if len(frontierSection)<10:
			frontierGroup.remove(frontierSection)
	print len(frontierGroup)
	for frontierSection in frontierGroup:
		print len(frontierSection)
		if len(frontierSection)<10:
			frontierGroup.remove(frontierSection)
	print len(frontierGroup)

	#Display on Rviz removed frontiers
	lst=[]
	for m in frontierGroup:
		for n in m:
			lst.append(n)
	unknownFrontierNoDist=lst[:]

	robotStartup.publishGridCellNodes(lst,4)
	
	#make a list of centroids
	centroids = []
	for x in frontierGroup:
		if len(x)!=0:
			centroids.append(calcCentroid(x,width))
	robotStartup.publishGridCellNodes(centroids,5)

	centroidsdist=[]
	[x,y,w]=robotStartup.getPose()
	currentNode=Node.Node(x,y,0,width)
	for node in centroids:
		ppx=float((node.xPos)/xyscale)+1/(2*xyscale) + originx
		ppy=float((node.yPos)/xyscale)+1/(2*xyscale) + originy
		tmpnode = Node.Node(ppx,ppy,0,width)
		dist2Node=d(currentNode,tmpnode)
		centroidsdist.append((dist2Node, node))  # append to list

	centroidsdist.sort(key=lambda tup: tup[0])
	centroids = [(i[1]) for i in centroidsdist]

	return [lst,centroids]
		
	# g=robotStartup.getGridMap()
	# g.GridMap.astarSearch(g,0,0,4,4)
def calcCentroid(frontierNodes,width):
	xc=0
	yc=0
	cnt=0
	for cell in frontierNodes:
		xc=xc+cell.xPos
		yc=yc+cell.yPos
		cnt=cnt+1
	xc=float(xc/cnt)
	yc=float(yc/cnt)
	return Node.Node(xc,yc,1000,width) 
	#return Node.Node(xc,yc,1000,width)

def offsetCentroid(centroid, width, height, allnodes):
	print "Trying to offset centroid:", centroid.xPos,",", centroid.yPos
	print "Allnodes height:", len(allnodes)
	#need to BFS from centroid to find the first driveable node
	nodesToCheck = []
	checkedNodes = []
	nodesToCheck.append(centroid)
	depthLimit = 200
	while (len(nodesToCheck) > 0) and len(checkedNodes)< depthLimit:
		currentNode = nodesToCheck.pop(0)
		checkedNodes.append(currentNode)
		#this node is driveable! return this
		#print "CurrentNode.obs is:", currentNode.obs
		if(currentNode.obs != Node.Node.unknown and currentNode.obs != 1000 and currentNode.obs < Node.Node.threshold):
			#print "Found a valid node! It is:"
			#print "\t", currentNode.xPos, currentNode.yPos
			return currentNode
		#this node wasn't able to be driven to; add its neighbors to be checked
		else:
			neighbors = currentNode.getStraightNeighbors(allnodes)
			for n in neighbors:
				#make sure the node isnt already in the queue and hasnt been
				#checked already
				if((n not in nodesToCheck) and (n not in checkedNodes)):
					nodesToCheck.append(n)

	print "Node could not be offset, moving on"
	return None




if __name__ == '__main__':
	rospy.init_node('finalProject', anonymous=True)
	try:
		done=1
		print "Starting"
		rospy.sleep(1)
		robotStartup.setupRobot()

		attemptedCentroids = []

		while not rospy.is_shutdown() and done>0:
			print "starting the loop again"
			
			[allnodes, width, height, resolution, originx, originy]=  robotStartup.getNodeMap()
			global xyscale
			xyscale = 1.0/(resolution*1.0)
			#find frontiers and get centroids
			[frontiers, centroids] = findFrontiers(allnodes, width, height, resolution, originx, originy)
			if len(centroids)<1:
				done=len(centroids)
				break
			print len(frontiers), "frontiers have been found."
			print len(centroids), "centroids have been calculated."
			print "The centroids are:"
			for cent in centroids:
				print(("\t%d,%d") % (int(cent.xPos), int(cent.yPos)))

			walls = []
			print "looking for walls"
			for node in allnodes:
				if(node.obs > Node.Node.threshold):
					walls.append(node)
			print "done looking for walls"

			robotStartup.publishGridCellNodes(walls, 7)

			[x,y,w]=robotStartup.getPose()

			currx = int((x-originx-(1/(2*xyscale)))*xyscale)
			curry = int((y-originy-(1/(2*xyscale)))*xyscale)

			robotStartup.publishGridCellNodes(centroids,1)
			'''
			robotStartup.publishGridCellList(wypnts,4)
			'''
			offsetCentroids = []
			for c in centroids:
				newCentroid = offsetCentroid(c, width, height, allnodes)
				if(newCentroid != None):
					offsetCentroids.append(newCentroid)
			#print("Original:\t\tOffset:")
#			for i in range(len(centroids)):
#				print("[%d,%d]\t\t[%d,%d]" % (centroids[i].xPos, centroids[i].yPos, offsetCentroids[i].xPos, offsetCentroids[i].yPos))
			#remove centroids too close to the current one
			
			listLength = len(offsetCentroids)
			for c in offsetCentroids:
				if(d(c, Node.Node(currx, curry, 0, width)) < 10):
					#print "Trying to remove a node, first loop"
					offsetCentroids.remove(c)
			newListLength = len(offsetCentroids)
			#print listLength-newListLength, "centroids removed."

			for c in offsetCentroids:
				neighbors = c.getStraightNeighbors(allnodes)
				for n in neighbors:
					if(n.obs > Node.Node.threshold):
						#print "Trying to remove a node, second loop"
						offsetCentroids.remove(c)
						break
			newNewListLength = len(offsetCentroids)

			#print newListLength-newNewListLength, "more centroids removed"

			'''
			#find a valid centroid
			for c in offsetCentroids:
				print "Trying a new centroid"
				success = robotStartup.gridMap.aStarSearch(currx,curry,int(c.xPos),int(c.yPos))
				if(success != False):
					print "Found a successful path"
					break
				else:
					print "Trying again..."
				print "Done with AStar"
				#robotStartup.gridMap.aStarSearch(currx,curry,int(centroids[len(centroids)-1].xPos),int(centroids[len(centroids)-1].yPos))
			wypnts = robotStartup.gridMap.printTotalPath()
			mynode = Node.Node(wypnts[(len(wypnts)/2)].x,wypnts[(len(wypnts)/2)].y,0,width)

			'''


			robotStartup.publishGridCellNodes(offsetCentroids, 5)
			print "Going to Node"
			mynode = random.sample(offsetCentroids, 1)[0]
			attemptedCentroids.append(mynode)
			#if we've tried the same centroid 3 times, break the program
			
			centroidCount = 0
			for a in attemptedCentroids:
				if mynode == a:
					centroidCount += 1
			print "We've tried this centroid", centroidCount, "times."
			if(centroidCount > 3 or len(attemptedCentroids) > 20):
				print "Too many times...Exiting..."
				break


			robotStartup.publishGridCellNodes([mynode],6)
			robotStartup.goToNode(mynode,xyscale,originx,originy)
			rospy.sleep(15)
			#robotStartup.goToNode(mynode,xyscale,originx,originy)
			
		# print "WE DID IT WE DID IT WE DID IT HOORAY!"
		# print len(centroids)

	except rospy.ROSInterruptException:
		pass
	print "WE DID IT WE DID IT WE DID IT HOORAY!"
	print len(centroids)
	subprocess.call(["aplay ~/Desktop/doramap.wav"],shell=True)