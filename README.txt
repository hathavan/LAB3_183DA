Lab 3 Readme

2.2a) 	The function create_environment plots the initial/goal points and the obstacles. 
		The function segment plots a line segment between two points, used for visualization. 
		The function show_robot() plots a circle around a point indicating robot position.
		
2.2b) 	To accommodate a rectangular robot body, we change show_robot to use the same code as 
		the function rectangle, which is used to plot obstacles. 

2.2c) 	The function trajectory plots a 1-second change in position from a start point to a target point.
		It depends on the velocity used, v.
		
2.2d)	The function valid_trajectory determines if a trajectory exists between two points and doesn't
		cut through an obstacle. Using the mathematical formulation in the report, it checks if three lines 
		between the two circles representing start and end points intersect a rectangle representing
		an obstacle: 
			line between center of circles
			2 lines between boundaries of circles

2.2e) 	The function closest computes the distance from the target point to each point in the list.
		It returns the point with the minimum distance and a valid trajectory between point and target.
	
2.2f)	Refer to the lab report to see the RRT evolution.

2.2g)	Refer to the lab report to see the final RRT path between various input and end points.

2.2h) 	To implement RRT*, each new point added to the graph needs to check if there is a better parent node.
		Each new point added to the graph is checked if it is a better parent for any other node. 
		
		Function to implement the first check: better parent for new_point
		
			Find min distance(node in V, new_point)
			Remove edge (parent, new_point) from E
			parent = node in V
			Add edge (parent, new_point)
		
		Function to check if new_point is a better parent
			
			For each node in V
			if distance(new_point, node) < distance(node_parent, node)
				Remove edge (node_parent, node)
				Add edge (new_point,node)

2.3a)	Refer to the report for performance.

2.3b) 	Finding the nearest vertice : number of vertices V
		Checking for collision: number of obstacles, S
		Other mathematical steps are O(1)
		This loops N times
		
		O( VSN) complexity

2.3c) 	Unfortunately our planner is quite slow in simulation, ranging from 1 second to 10 seconds. 
		It is faster if the goal is closer to the start point, so any obstacle changes have to happen in close distances
		and in few number, to keep the complexity low. 

2.3d) 	Our planner is somewhat effective and shows proof of concept, however it needs improvement with RRT* and a rectangular 
		model of the body. 