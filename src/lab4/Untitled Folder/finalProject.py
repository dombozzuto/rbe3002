import math, numpy, time
import rospy, roslib, tf
import tf.transformations
import nav_msgs.msg, geometry_msgs.msg, std_msgs.msg, kobuki_msgs.msg, actionlib_msgs.msg
import robotStartup
import Node
import GridMap



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
				if (neigh.obs > -1) and (neigh.obs < 30):
					frontierFlag = 1
			if frontierFlag == 1:
				unknownFrontierNoDist.append(node)

	[x,y,w]=robotStartup.getPose()
	xyscale = 1.0/(resolution*1.0)
	currentNode=Node.Node(x,y,0,width)

	#publish GridCells to see on rviz
	# for node in unknownFrontier:
	# 	print(node[0],node[1].printNode())
	# for node in unknownFrontierNoDist:
	# 	node.printNode()

	frontierGroup=[]
	frontierGroup.append([])
	# frontierGroup[0].append(unknownFrontierNoDist[0])
	# print frontierGroup

	# for each node in the frontier
	for node in unknownFrontierNoDist:
		#find all neighbors
		nodeNeighbors = node.getStraightNeighbors(allnodes)
		frontierNeigh=[]
		#for each node in the neighbors of frontier node
		flag=0
		for neigh in nodeNeighbors:
			for frontierSection in frontierGroup:
				if neigh in frontierSection:
					if flag == 0:
						frontierSection.append(node)
						flag=1
		if flag == 0:
			l=[];l.append(node)
			frontierGroup.append(l)


	robotStartup.publishGridCellNodes(unknownFrontierNoDist,4)
	
	#make a list of centroids
	centroids = []
	for x in frontierGroup:
		if len(x)!=0:
			centroids.append(calcCentroid(x,width))
	robotStartup.publishGridCellNodes(centroids,3)

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

	return [unknownFrontier,centroids]
		
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
	return(Node.Node(xc,yc,1000,width))

	return Node.Node(xc,yc,1000,width)
if __name__ == '__main__':
	
	rospy.init_node('finalProject', anonymous=True)
	try:

		print "Starting"

		

		rospy.sleep(1)
		robotStartup.setupRobot()
		while not rospy.is_shutdown():
			[allnodes, width, height, resolution, originx, originy]=  robotStartup.getNodeMap()
			global xyscale
			xyscale = 1.0/(resolution*1.0)
			#find frontiers and get centroids
			[frontiers, centroids] = findFrontiers(allnodes, width, height, resolution, originx, originy)
			print frontiers, centroids

			[x,y,w]=robotStartup.getPose()

			currx = int((x-originx-(1/(2*xyscale)))*xyscale)
			curry = int((y-originy-(1/(2*xyscale)))*xyscale)
			# float( ((node.xPos))/xyscale) +1/(2*xyscale) + originx
			print currx, centroids[0].xPos
			robotStartup.gridMap.aStarSearch(currx,curry,int(centroids[0].xPos),int(centroids[0].yPos))
			
			wypnts = robotStartup.gridMap.printTotalPath()
			print wypnts[0],wypnts[1],wypnts[2]
			mynode = Node.Node(wypnts[(len(wypnts)/2)].x,wypnts[0].y,0,width)

			robotStartup.publishGridCellList(wypnts,4)
			robotStartup.goToNode(mynode,xyscale,originx,originy)



	except rospy.ROSInterruptException:
		pass