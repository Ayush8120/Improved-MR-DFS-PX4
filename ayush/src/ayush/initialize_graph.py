import rospy
import pandas as pd
import numpy as np
from collections import defaultdict
import networkx as nx


'''
arguments = edges in a list of strings format
returns = graph, decomposed edges
'''
edges_decomp = []
def build_graph(edges):
    for ch in edges:
        edges_decomp.append(list(ch))
        
    graph = defaultdict(list)
    for edge in edges_decomp:
        a, b = edge[0], edge[1]
         
        # Creating the graph as adjacency list
        graph[a].append(b)
        graph[b].append(a)
    return graph , edges_decomp

'''
Robot class for organizing all robots and the required methods
Each instance of this class will have these Attributes:
    - robo_identifier :integer for identifying the robot
    - count : flag to detect when its exploration is completed
    - iteration : number of times the robot has updated its incidence matrix
    - next_edge_detected : string to store next edge detected
    - row_tags : list to store all the nodes it has a knowledge about although not necessarily travelled to them
    - edge_tags : list to store all the edge names it has a knowledge although not necessarily travelled to them
    - spawn : Spawn node name
    - present_location : node name of where the robot is currently
    - incidence_matrix : matrix containing incidence angles according to defined convention
    - setpoint_list : list to pass on to PX4 

And these Methods:
    - update_iteration : self explanatory
    - update_row_tags : " "
    - update_edge_tags : " "
    - update_present_location : " " 

'''

class Robot:
    def __init__(self,robo_number,leaf,angle_incidence_matrix= 0):

        self.robo_identifier = robo_number
        self.count = 0
        self.Q = []
        self.iteration = 0
        self.next_edge_decided = ''
        self.row_tags = []
        self.edge_tags = []
        self.spawn = leaf[robo_number]#same as robot name
        self.present_location = self.spawn
        self.incidence_matrix = pd.DataFrame(data= [],
                                        index = self.row_tags,
                                        columns= self.edge_tags)
        self.setpoint_list = [] 
    
def update_present_location(obj,new_present_location):
    obj.present_location = new_present_location

def update_iteration(obj,new_iter):
    obj.iteration = new_iter

def update_row_tags(obj,new_row_tags):
    obj.row_tags = new_row_tags
    obj.incidence_matrix.index = new_row_tags 

def update_edge_tags(obj,new_edge_tags):
    obj.edge_tags = new_edge_tags
    obj.incidence_matrix.columns = new_edge_tags 
     
'''
Vertex Class for managing all the methods and objects 
For each instance of this class we will need:
    - iteration : how many times has the vertex been updated
    - row_tags : list of node names connected to that node
    - edge_tags : list of edge names connected to that node
    - neighbors : list of neighbor node names
    - neighbors_index : list of neighbor nodes index i.e. node name - 65 
    - incidence_matrix 
    - E1_cap : for storing the modified dimension
'''
class Vertex:
    def __init__(self,vertex_name,edges,incidence_matrix):
        self.name = vertex_name #str
        
        self.iteration = 0
        self.row_tags = [vertex_name]
        self.edge_tags = []
        self.neighbors = []
        self.neighbors_index = []
        for i in range(len(edges)):
            if  edges[i].find(self.name) != -1:
                self.edge_tags.append(edges[i])#list
                
        for ch in self.edge_tags: 
            self.neighbors.append(ch.replace(self.name,""))
        
        for ch in self.neighbors:
            self.neighbors_index.append(ord(ch) - 65)
        
        slice = incidence_matrix[ord(vertex_name) - 65,self.neighbors_index]
        self.incidence_matrix = pd.DataFrame(data = np.reshape(np.array(slice),(1,len(slice))),
                                            index = self.row_tags,
                                            columns= self.edge_tags) ##TODO
        self.E1_cap = 0


'''
In Second_step_at_vertex we need to find the shortest route between the robot's present location and the chosen edge's known vertex
This function helps us get that route:
    arguments  = graph(returned from build_graph() function), start node, goal node
    returns    = shortest path between the start and goal node
'''
def find_shortest_path(graph, start, goal):
    explored = []
     
    # Queue for traversing the graph in the BFS
    queue = [[start]]
     
    # If the desired node is reached
    if start == goal:
        #print("Same Node")
        return []
     
    # Loop to traverse the graph with the help of the queue
    while queue:
        path = queue.pop(0)
        node = path[-1]
         
        # Condition to check if the current node is not visited
        if node not in explored:
            neighbours = graph[node]
             
            # Loop to iterate over the neighbours of the node
            for neighbour in neighbours:
                new_path = list(path)
                new_path.append(neighbour)
                queue.append(new_path)
                 
                # Condition to check if the neighbour node is the goal
                if neighbour == goal:
                    #print("Shortest path = ", *new_path)
                    return new_path
            explored.append(node)
 
    # Condition when the nodes are not connected
    print("So sorry, but a connecting"\
                "path doesn't exist :(")
    return