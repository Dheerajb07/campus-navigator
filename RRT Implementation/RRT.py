# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

import matplotlib.pyplot as plt
import numpy as np
from scipy import spatial

# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost

    def __eq__(self,other):
        return [self.row,self.col]==[other.row,other.col]

# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        self.points = []                      # co-ordinartes of vertices
        

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.points = []
        self.vertices.append(self.start)
        self.points.append((self.start.row,self.start.col))

    
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        ### YOUR CODE HERE ###
        p1 = np.array([node1.row,node1.col])
        p2 = np.array([node2.row,node2.col])
        return np.linalg.norm(p1-p2)

    def is_obstacle(self,point):
        return not self.map_array[point[0]][point[1]]
    
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''
        ### YOUR CODE HERE ###
        delta = 1 # spacing between each sample point
        x1,y1 = (node1.row,node1.col)
        x2,y2 = (node2.row,node2.col)
        theta = np.arctan2(y2-y1,x2-x1) # slope of the line
        sx=x1
        sy=y1
        for i in range(1,round(self.dis(node1,node2))+1):
            sx = round(x1+i*delta*np.cos(theta))
            sy = round(y1+i*delta*np.sin(theta))
            if self.is_obstacle([sx,sy]):
                return False
        return True


    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point - %

        return:
            point - the new point - a list [row,col]
        '''
        ### YOUR CODE HERE ###
        # generate random int
        p = np.random.randint(100)
        if p > goal_bias*100:
            q_rand = np.random.randint([self.size_row,self.size_col])
            return q_rand
        else:
            return [self.goal.row,self.goal.col]

    
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        ### YOUR CODE HERE ###
        self.kdtree = spatial.KDTree(self.points)
        _,nearest_pt_idx = self.kdtree.query(point,k=1)
        return self.vertices[nearest_pt_idx]
        
    def connect(self,node,parent_node):
        '''
        connect node with its parent
        '''
        node.parent = parent_node
        node.cost = parent_node.cost + self.dis(node,parent_node)
        self.vertices.append(node)
        self.points.append((node.row,node.col))

    def get_new_node(self,node1,node2,step_size):
                
        if self.dis(node1,node2)<=step_size:
            return node2
            
        x1,y1 = (node1.row,node1.col)
        x2,y2 = (node2.row,node2.col)
        theta = np.arctan2(y2-y1,x2-x1) # slope of the line

        sx = round(x1+step_size*np.cos(theta))
        sy = round(y1+step_size*np.sin(theta))
        
        return Node(sx,sy)

    def add_node(self,node1,node2,step_size):
        
        next_node = self.get_new_node(node1,node2,step_size)

        obstruction = False
        if self.is_obstacle([next_node.row,next_node.col]) or not self.check_collision(node1,next_node):
            obstruction = True
        
        if not obstruction:
            if next_node == self.goal:
                self.connect(self.goal,node1)
                self.found = True
            elif self.dis(next_node,self.goal)<=step_size:
                self.connect(self.goal,node1)
                self.found = True
            else:
                self.connect(next_node,node1)

            
    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        ### YOUR CODE HERE ###
        neighbours = []
        new_point = (new_node.row,new_node.col)
        self.kdtree = spatial.KDTree(self.points)
        neighbours_idxs = self.kdtree.query_ball_point(new_point,r=neighbor_size)
        for idx in neighbours_idxs:
            neighbours.append(self.vertices[idx])
        return neighbours

    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        ### YOUR CODE HERE ###
        for node in neighbors:
            if self.check_collision(new_node,node):
                new_cost = new_node.cost + self.dis(new_node,node)
                if new_cost<=node.cost:
                    #update parent of neighbour node
                    node.parent = new_node
                    node.cost = new_cost


    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col or cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()


    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, check if reach the neighbor region of the goal.
        step_size = 10
        for i in range(n_pts):
             q_rand = self.get_new_point(0)
             if self.is_obstacle(q_rand):
                 continue
             rand_node = Node(q_rand[0],q_rand[1])
             # find nearest node to the random node
             nearest_node = self.get_nearest_node(tuple(q_rand))
             # connect rand_node with nearest_node
             self.add_node(nearest_node,rand_node,step_size)
             # if extended to goal point - break loop
             if self.found:
                 break

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")
        
        # Draw result
        self.draw_map()


    def RRT_star(self, n_pts=1000, neighbor_size=20):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, rewire the node and its neighbors,
        # and check if reach the neighbor region of the goal if the path is not found.
        step_size = 10
        for i in range(n_pts):
            q_rand = self.get_new_point(0.05)
            if self.is_obstacle(q_rand):
                continue
            rand_node = Node(q_rand[0],q_rand[1])
            # extend a new_node towards the rand node
            nearest_node = self.get_nearest_node(tuple(q_rand))
            new_node = self.get_new_node(nearest_node,rand_node,step_size)
            # find neighbours to new_node
            neighbour_nodes = self.get_neighbors(new_node,neighbor_size)
            if len(neighbour_nodes)==0:
                continue
            ## connect new node to the tree
            # find min cost neighbour node
            costs = []
            for node in neighbour_nodes:
                costs.append(node.cost + self.dis(node,new_node))
            
            if not len(costs)==0:
                min_cost_idx = np.argmin(np.array(costs))
                min_cost_node = neighbour_nodes[min_cost_idx]
                if self.check_collision(new_node,min_cost_node):
                    if new_node == self.goal:
                        self.connect(self.goal,min_cost_node)
                        self.found = True
                    elif self.dis(new_node,self.goal)<=neighbor_size:
                        self.connect(new_node,min_cost_node)
                        self.rewire(new_node,neighbour_nodes)
                        self.connect(self.goal,new_node)
                        self.found = True
                    else:
                        self.connect(new_node,min_cost_node)
                        self.rewire(new_node,neighbour_nodes)
            
        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()
