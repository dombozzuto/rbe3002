import Node

def findFrontiers():
	global allnodes
	unknownFrontierNoDist = []
	unknownFrontier = []
	allnodes = []

	n1 = Node.Node(0,0,1,1)
	n2 = Node.Node(0,1,1,1)
	n3 = Node.Node(0,2,1,1)
	n4 = Node.Node(2,2,1,1)
	n5 = Node.Node(2,3,1,1)
	n6 = Node.Node(1,1,1,1)
	n7 = Node.Node(5,5,1,1)

	for x in range(5):
		for y in range(5):
			allnodes.append(Node.Node(x,y,1,1))

	unknownFrontierNoDist.append(n1)
	unknownFrontierNoDist.append(n2)
	unknownFrontierNoDist.append(n3)
	unknownFrontierNoDist.append(n4)
	unknownFrontierNoDist.append(n5)
	unknownFrontierNoDist.append(n6)

	'''
	allnodes.append(n1)
	allnodes.append(n2)
	allnodes.append(n3)
	allnodes.append(n4)
	allnodes.append(n5)
	allnodes.append(n6)
	allnodes.append(n7)
	'''
	#[allnodes, width, height, resolution, originx, originy] =  robotStartup.getNodeMap()
	
	# for node in allnodes:
	# 	if (not (node.obs == -1)) and (node.obs <50):
	# 		print node.printNode()
	'''
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
	'''
	#robotStartup.publishGridCellNodes(unknownFrontierNoDist,4)
	'''
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
	'''

	frontierGroup=[]
	firstFrontier = []
	firstFrontier.append(unknownFrontierNoDist[0])
	# for each node in the frontier
	for node in unknownFrontierNoDist:

		print "Currently running on node:", node.xPos, node.yPos
		nodeNeighbors = node.getStraightNeighbors(allnodes)
		print "This node has", len(nodeNeighbors),"neighbors"
		#loop through each frontier to find where this node should be placed
		nodeNeighborsOnFrontier = []
		for n in nodeNeighbors:
			if n in unknownFrontierNoDist:
				nodeNeighborsOnFrontier.append(n)
		print "This node has", len(nodeNeighborsOnFrontier),"nodes on the frontier"
		#check each neighbor in each frontier to see 
		foundAPlaceToAdd = False
		for n in nodeNeighborsOnFrontier:
			#look in each existing frontier
			for frontier in frontierGroup:

				if n in frontier:
					frontier.append(node)
					foundAPlaceToAdd = True
		
		#there was no place to add this node
		if(not foundAPlaceToAdd):
			newFrontier = []
			newFrontier.append(node)
			print "Adding a new frontier for node", node.xPos, node.yPos
			frontierGroup.append(newFrontier)


		'''
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
		'''


	print "\n",len(frontierGroup),len(frontierGroup[0])
	'''
	for x in frontierGroup:
		rospy.sleep(1)
		robotStartup.publishGridCellNodes(x,3)
	'''
	for m in frontierGroup:
		print "\n",
		for n in m:
			n.printNode()

findFrontiers()