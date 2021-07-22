import rospy
import pandas as pd
import numpy as np
from collections import defaultdict
import networkx as nx



edges_decomp = []
def build_graph(edges):
    for ch in edges:
        edges_decomp.append(list(ch))
        #print(edges_decomp)
    graph = defaultdict(list)
    for edge in edges_decomp:
        a, b = edge[0], edge[1]
         
        # Creating the graph
        # as adjacency list
        graph[a].append(b)
        graph[b].append(a)
    return graph , edges_decomp


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
     

class Vertex:
    def __init__(self,vertex_name,edges,incidence_matrix):
        self.name = vertex_name #str
        #self.df = pd.DataFrame([9,2,3])
        self.iteration = 0
        self.row_tags = [vertex_name]
        self.edge_tags = []
        self.neighbors = []
        self.neighbors_index = []
        for i in range(len(edges)):
            if  edges[i].find(self.name) != -1:
                #print(edges[i])
                self.edge_tags.append(edges[i])#list
                #print(self.edge_tags)
        for ch in self.edge_tags: 
            self.neighbors.append(ch.replace(self.name,""))
        #print(self.neighbors)
        for ch in self.neighbors:
            self.neighbors_index.append(ord(ch) - 65)
        slice = incidence_matrix[ord(vertex_name) - 65,self.neighbors_index]
        self.incidence_matrix = pd.DataFrame(data = np.reshape(np.array(slice),(1,len(slice))),
                                            index = self.row_tags,
                                            columns= self.edge_tags) ##TODO
        self.E1_cap = 0

# def update_iteration(self,new_iter):
#     self.iteration = new_iter
# def update_row_tags(self,new_row_tags):
#     #self.df.Index = self.df.Index + [new_row_tags]
#     self.row_tags = new_row_tags

# def update_edge_tags(obj,new_edge_tags):
#     obj.edge_tags = new_edge_tags
#     obj.incidence_matrix.columns = new_edge_tags

def find_shortest_path(graph, start, goal):
    explored = []
     
    # Queue for traversing the
    # graph in the BFS
    queue = [[start]]
     
    # If the desired node is
    # reached
    if start == goal:
        #print("Same Node")
        return []
     
    # Loop to traverse the graph
    # with the help of the queue
    while queue:
        path = queue.pop(0)
        node = path[-1]
         
        # Condition to check if the
        # current node is not visited
        if node not in explored:
            neighbours = graph[node]
             
            # Loop to iterate over the
            # neighbours of the node
            for neighbour in neighbours:
                new_path = list(path)
                new_path.append(neighbour)
                queue.append(new_path)
                 
                # Condition to check if the
                # neighbour node is the goal
                if neighbour == goal:
                    #print("Shortest path = ", *new_path)
                    return new_path
            explored.append(node)
 
    # Condition when the nodes
    # are not connected
    print("So sorry, but a connecting"\
                "path doesn't exist :(")
    return











def rough():
# player_score = [1,0,1,1,0]
# cpu_score = [0,1,8,19,78]

# df = pd.DataFrame([player_score,cpu_score])
# df.columns = ['G1','G2','G3','G4','G5']
# df.index = ['Player','CPU']

# def initialize_graph(Robo_num,vertex_name,incidence_matrix,G):

# class graph:
#     def __init__(self,gdict=None):
#         if gdict is None:
#             gdict = {}
#         self.gdict = gdict

#     def getVertices(self):
#         return list(self.gdict.keys())

#     def getEdges(self):
#         return self.findedges()
    
#     def findEdges(self):
#         edgename = []
#         for vrtx in self.gdict:
#             for nxtvrtx in self.gdict[vrtx]:
#                 if {nxtvrtx, vrtx} not in edgename:
#                     edgename.append({vrtx, nxtvrtx})
#         return edgename


# graph_elements = { "A" : ["B","F","G"],
#           "B" : ["A" ,"C"],
#           "C" : ["B", "D", "E"],
#           "D" : ["C"],
#           "E" : ["C"],
#           "F" : ["A"],
#           "H" : ["A","G"],
#           "G" : ["H"]
#          }

#print(graph) - vertex - edge shows both

#g = graph(graph_elements)
    return 0
