import rospy
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import pandas as pd
import pprint
from initialize_graph import Vertex, build_graph, find_shortest_path
from initialize_graph import Robot
from collections import defaultdict
from second_step_on_vertex_visit import second_step_on_vertex_visit
from first_step_on_vertex_visit import Id,what_to_do_if_next_node_known,first_step_on_arriving_at_vertex
from get_incidence_matrix import get_incidence_matrix

pp = pprint.PrettyPrinter(indent=8)
'''
A standalone implementation of the algorithm on a smaller graph  
4 Leaf Nodes
8 Nodes
'''
#Topography 
K = 4 #Number of Leaf nodes = Number of Robots
J = 8 # Number of nodes
count = 0 #local flag to declare overall completion 
edges = ["AB","BC","CD","CE","AF","AH","GH"] # Edge name list
vertex = ["A","B","C","D","E","F","G","H"] # Vertex Node name list
robo_vertex = ["D","E","F","G"] # Spawn Locations of robots
XData = [2.5000, 1.5000, 1.5000, 1, 2, 2.5000, 3.5000, 3.5000] #Robot Spawn X cordinates 
YData = [4, 3, 2, 1, 1, 3, 2, 3] #Robot Spwn Y cordinates
[graph,edges_decomp] = build_graph(edges) #building the graph for computations

'''
Graph making for visualization
'''
G = nx.Graph()
for i in range(J):
    G.add_node(chr(i+65))
    
for ed in edges_decomp:
    print(*ed)
    G.add_edge(*ed)
nx.draw(G,with_labels = True, font_weight = 'bold')
plt.show

'''
Incidence Matrix Making
'''
incidence_matrix = get_incidence_matrix(XData,YData,G)
pp.pprint(incidence_matrix)

'''
Initializations
'''
#initializing of R : list of Robot objects
R = []
for j in range(K):
    R.append(Robot(j,robo_vertex,incidence_matrix))

#initializing of V : list of Vertex objects
V = []
for j in range(J):
    V.append(Vertex(vertex[j],edges,incidence_matrix))

'''
CORE ALGORITHM :
Each robot explores until its personal count flag turns ON. And the loop runs till the overall count flag turns ON.
'''
'''
As the robots are placed on leaf nodes thus the vertex will have only one neighbour at that vertex
Thus all the robots are given an initial puch to there immediate neighbors
'''
print("The first mandatory push:")
print('')
for k in range(K):
    
    start = R[k].present_location
    end = V[ord(R[k].present_location) - 65].neighbors[0]
    top = np.array([-1*incidence_matrix[ord(start) - 65,ord(end)-65]])
    bottom = np.array([-1*incidence_matrix[ord(end) - 65,ord(start)-65]])
    col_vector = np.vstack((top,bottom))
     
    print("The {e} robot is currently at {f}".format(e=k,f=R[k].present_location))
    print("The next node chosen is {}".format(V[ord(R[k].present_location) - 65].neighbors[0]))
    id,R,V = what_to_do_if_next_node_known(R,k,V,1,R[k].present_location,V[ord(R[k].present_location) - 65].neighbors[0],incidence_matrix=incidence_matrix)
    
    R[k].next_edge_decided,count = second_step_on_vertex_visit(graph, V,R,k,count)
    print('The next edge selected by - ' + str(k) + '- robot is' + str(R[k].next_edge_decided))
    print('')
    
print("This is the loop part which continues till the declaration of completion")
while(count != K):
     for z in range(K):
         if R[z].count != 1:
            print("{z} robot Travelling to the selected edge :{e}".format(z = z , e = R[z].next_edge_decided) )
            if R[z].next_edge_decided !=0:
               id,R,V = what_to_do_if_next_node_known(R,z,V,2,R[z].present_location, R[z].next_edge_decided.replace(R[z].present_location,''),incidence_matrix = incidence_matrix)
               R[z].next_edge_decided,count = second_step_on_vertex_visit(graph, V,R,z,count)
               if R[z].next_edge_decided != 0:
                    print('The next edge selected by : ' + str(z) + ' : robot is :' + str(R[z].next_edge_decided))
                    print('')
