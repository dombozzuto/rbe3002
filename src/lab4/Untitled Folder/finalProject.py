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

def findFrontiers():
	global allnodes
	unknownFrontierNoDist = []
	unknownFrontier = []
	allnodes = []

	[allnodes, width, height, resolution, originx, originy]=  robotStartup.getNodeMap()
	
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
				if (neigh.obs > -1) and (neigh.obs < 50):
					frontierFlag = 1
			if frontierFlag == 1:
				unknownFrontierNoDist.append(node)

	[x,y,w]=robotStartup.getPose()
	xyscale = 1.0/(resolution*1.0)
	currentNode=Node.Node(x,y,0,width)

	robotStartup.publishGridCellNodes(unknownFrontierNoDist,4)
	for node in unknownFrontierNoDist:
		# calculate the distance between the current node and the current
		# unknown frontier node
		ppx=float((node.xPos)/xyscale)+1/(2*xyscale) + originy
		ppy=float((node.yPos)/xyscale)+1/(2*xyscale) + originy
		tmpnode = Node.Node(ppx,ppy,0,width)
		dist2Node=d(currentNode,tmpnode)
		unknownFrontier.append((dist2Node, node))  # append to list

	unknownFrontier.sort(key=lambda tup: tup[0])
	unknownFrontierNoDist = [(i[1]) for i in unknownFrontier]
	
	print "len allnodes", len(allnodes)
	print "len unknownFrontier", len(unknownFrontier)
	print "len unknownFrontierNoDist", len(unknownFrontierNoDist)

	#publish GridCells to see on rviz
	for node in unknownFrontier:
		print(node[0],node[1].printNode())
	for node in unknownFrontierNoDist:
		node.printNode()

	frontierGroup=[]
	# for each node in the frontier
	for node in unknownFrontierNoDist:
		#find all neighbors
		nodeNeighbors = node.getStraightNeighbors(allnodes)
		frontierNeigh=[]
		#for each node in the neighbors of frontier node
		for n in nodeNeighbors:
			#if neighbor is in frontier
			if n in unknownFrontierNoDist:
				#add to frontier neighbors
				frontierNeigh.append(n)
		frontierGroup.append(frontierNeigh)

	print "\n",len(frontierGroup),len(frontierGroup[0])

	for x in frontierGroup:
		rospy.sleep(1)
		robotStartup.publishGridCellNodes(x,3)

	for m in frontierGroup:
		print "\n",
		for n in m:
			n.printNode()




	# for node in unknownFrontier:

	
	
	# g=robotStartup.getGridMap()
	# g.GridMap.astarSearch(g,0,0,4,4)
def calcCentroid(frontierNodes):
	xc=0
	yc=0
	cnt=0
	for cell in unknownFrontierNoDist:
		xc=xc+cell.xPos
		yc=yc+cell.yPos
		cnt=cnt+1
	xc=xc/cnt
	yc=yc/cnt
	centroid=[]
	centroid.append(Node.Node(xc,yc,1000,width))
	robotStartup.publishGridCellNodes(centroid,3)

	return Node.Node(xc,yc,1000,width)
if __name__ == '__main__':
	
	rospy.init_node('finalProject', anonymous=True)
	try:
		print "Starting"
		rospy.sleep(1)
		robotStartup.setupRobot()

		findFrontiers()



	except rospy.ROSInterruptException:
		pass