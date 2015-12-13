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
				if (neigh.obs > -1) and (neigh.obs < 25):
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

	for node in frontierGroup:
		if len(node)<5:
			frontierGroup.remove(node)

	robotStartup.publishGridCellNodes(unknownFrontierNoDist,4)
	
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

def getDestination(pop=0):
	print "getting new destination"
	global centroids
	global frontiers
	global mynode
	global width
	global height
	global resolution
	global originx
	global originy
	global allnodes
	#get node map
	[allnodes, width, height, resolution, originx, originy]=  robotStartup.getNodeMap()
	print "lol"
	#get map scale
	global xyscale
	xyscale = 1.0/(resolution*1.0)
	#find frontiers and get centroids
	if(pop==2):
		[frontiers, centroids] = findFrontiers(allnodes, width, height, resolution, originx, originy)
	if pop==1:
		centroids.pop()
	print "lol2"
	#get current pose of robot
	[x,y,w,currx,curry]=robotStartup.currPose()	
	print "lol3"
	#astar search to the first (closest) centroid
	robotStartup.gridMap.aStarSearch(currx,curry,int(centroids[0].xPos),int(centroids[0].yPos))
	print "lol4"
	#find a destination
	wypnts = robotStartup.gridMap.printTotalPath()
	print str(len(wypnts)),wayPointDivider
	destinationWaypointx = wypnts[int((len(wypnts)/wayPointDivider))].x
	destinationWaypointy = wypnts[int((len(wypnts)/wayPointDivider))].y
	mynode = Node.Node(destinationWaypointx,destinationWaypointy,0,width)
	destinations=[]
	destinations.append(mynode)


	robotStartup.goToNode(mynode,xyscale,originx,originy)
	goTime = rospy.get_time()
	moving =1
	
	# robotStartup.publishGridCellNodes(frontiers,4)
	robotStartup.publishGridCellList(wypnts,2)
	

	return [mynode,frontiers,xyscale,originx,originy,goTime,moving]

if __name__ == '__main__':
	
	rospy.init_node('finalProject', anonymous=True)
	try:
		print "Starting"
		done = 0
		#for subdividing waypoints
		wayPointDivider = 2
		robotStartup.setupRobot()
		rospy.sleep(1)
		
		timeout=0
		# [frontiers, centroids] = findFrontiers(allnodes, width, height, resolution, originx, originy)
		[mynode,frontiers,xyscale,originx,originy,goTime,moving] = getDestination(2)
		# moveBaseStraight(0.3)
		robotStartup.goToNode(mynode,xyscale,originx,originy)
		startTime = rospy.get_time()
		while not rospy.is_shutdown() and len(centroids) > 0:
			
			while rospy.get_time()-startTime<30 and moving:
				print mynode.xPos, mynode.yPos, "while time:",rospy.get_time()-startTime
			print "out of while"
			robotStartup.stopRobot()
			# robotStartup.rotate(45)
			robotStartup.driveStraight(0.2,0.4)
			# if(len(centroids)>1):
			# 	[mynode,frontiers,xyscale,originx,originy,goTime,moving] = getDestination(1)
			# else:
			[frontiers, centroids] = findFrontiers(allnodes, width, height, resolution, originx, originy)
			getDestination(1)
			startTime=rospy.get_time()
			robotStartup.goToNode(mynode,xyscale,originx,originy)
			# if not rospy.get_time()-startTime<30 and moving:
			# 	print "timeout"
			# 	timeout = 1
			# if timeout:
			# 	centroids.pop()
			# 	robotStartup.moveBaseStraight(0.2)
			# 	[mynode,frontiers,xyscale,originx,originy,goTime,moving] = getDestination()
			# 	moving =1
			# 	done=0
			# 	print "starttime"
			# 	startTime = rospy.get_time()
			# 	robotStartup.goToNode(mynode,xyscale,originx,originy)

		# # #get node map
		# # [allnodes, width, height, resolution, originx, originy]=  robotStartup.getNodeMap()
		# # #get map scale
		# # global xyscale
		# # xyscale = 1.0/(resolution*1.0)
		# # #find frontiers and get centroids
		# # [frontiers, centroids] = findFrontiers(allnodes, width, height, resolution, originx, originy)
		# # #get current robot pose
		# # [x,y,w]=robotStartup.getPose()
		# # currx = int((x-originx-(1/(2*xyscale)))*xyscale)
		# # curry = int((y-originy-(1/(2*xyscale)))*xyscale)

		# # #astar search to the first (closest) centroid
		# # robotStartup.gridMap.aStarSearch(currx,curry,int(centroids[0].xPos),int(centroids[0].yPos))
		# # # get a list of waypoints from the astar search
		# # wypnts = robotStartup.gridMap.printTotalPath()
		# # #find a destionation waypoint
		# # destinationWaypointx = wypnts[int((len(wypnts)/wayPointDivider))].x
		# # destinationWaypointy = wypnts[int((len(wypnts)/wayPointDivider))].y
		
		# #get node map
		# [allnodes, width, height, resolution, originx, originy]=  robotStartup.getNodeMap()
		# #get map scale
		# global xyscale
		# xyscale = 1.0/(resolution*1.0)
		# #find frontiers and get centroids
		# [frontiers, centroids] = findFrontiers(allnodes, width, height, resolution, originx, originy)
		# #get current robot pose
		# [x,y,w]=robotStartup.getPose()
		# currx = int((x-originx-(1/(2*xyscale)))*xyscale)
		# curry = int((y-originy-(1/(2*xyscale)))*xyscale)

		# #astar search to the first (closest) centroid
		# robotStartup.gridMap.aStarSearch(currx,curry,int(centroids[0].xPos),int(centroids[0].yPos))
		# # get a list of waypoints from the astar search
		# wypnts = robotStartup.gridMap.printTotalPath()
		# startTime=rospy.get_time()
		# goTime=0
		# mynode = Node.Node(0,0,0,0)
		# while not rospy.is_shutdown() and not done:
			
		# 	print "while"			
		# 	if goTime-rospy.get_time() < 9:
		# 		robotStartup.goToNode(mynode,xyscale,originx,originy)
		# 	else:
		# 		goTime=0
		# 		if rospy.get_time()-startTime > 10:
		# 			print "time-starttime>10"
		# 			startTime=rospy.get_time()
		# 			wayPointDivider=wayPointDivider+1
		# 			if wayPointDivider > 8:
		# 				print 'waypointdivider >8'
		# 				wayPointDivider=1
		# 				centroids.pop()
		# 				if len(centroids)!=0:
		# 					print "lencentroids"
		# 					#get current robot pose
		# 					[x,y,w]=robotStartup.getPose()
		# 					currx = int((x-originx-(1/(2*xyscale)))*xyscale)
		# 					curry = int((y-originy-(1/(2*xyscale)))*xyscale)

		# 					#astar search to the first (closest) centroid
		# 					robotStartup.gridMap.aStarSearch(currx,curry,int(centroids[0].xPos),int(centroids[0].yPos))
		# 					print "lenastardone"
		# 					# get a list of waypoints from the astar search
		# 					wypnts = robotStartup.gridMap.printTotalPath()
		# 					print str(len(wypnts)),wayPointDivider
		# 					destinationWaypointx = wypnts[int((len(wypnts)/wayPointDivider))].x
		# 					destinationWaypointy = wypnts[int((len(wypnts)/wayPointDivider))].y

		# 					mynode = Node.Node(destinationWaypointx,destinationWaypointy,0,width)
		# 					destinations=[]
		# 					destinations.append(mynode)
		# 					robotStartup.publishGridCellList(wypnts,4)
		# 					robotStartup.publishGridCellNodes(destinations,5)
		# 					goTime = rospy.get_time()
		# 		else:
		# 			print"else"
		# 			startTime=rospy.get_time()
					
		# 			print str(len(wypnts)),wayPointDivider
		# 			destinationWaypointx = wypnts[int((len(wypnts)/wayPointDivider))].x
		# 			destinationWaypointy = wypnts[int((len(wypnts)/wayPointDivider))].y

		# 			mynode = Node.Node(destinationWaypointx,destinationWaypointy,0,width)
		# 			destinations=[]
		# 			destinations.append(mynode)
		# 			robotStartup.publishGridCellList(wypnts,4)
		# 			robotStartup.publishGridCellNodes(destinations,5)
		# 			goTime = rospy.get_time()

	except rospy.ROSInterruptException:
		pass