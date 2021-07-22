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
#Topography 
K = 4
J = 8
count = 0
edges = ["AB","BC","CD","CE","AF","AH","GH"] # change this when implementing for a different topo
vertex = ["A","B","C","D","E","F","G","H"] # change this when implementing for a different topo
robo_vertex = ["D","E","F","G"]
XData = [2.5000, 1.5000, 1.5000, 1, 2, 2.5000, 3.5000, 3.5000]
YData = [4, 3, 2, 1, 1, 3, 2, 3]
[graph,edges_decomp] = build_graph(edges)

G = nx.Graph()

for i in range(J):
    G.add_node(chr(i+65))
    #print('LAMBA')

for ed in edges_decomp:
    print(*ed)
    G.add_edge(*ed)
nx.draw(G,with_labels = True, font_weight = 'bold')
#plt.subplot(111)
plt.show
print(G.nodes)
print(G.edges)
#---------------------------------------------------------------------------
incidence_matrix = get_incidence_matrix(XData,YData,G)
pp.pprint(incidence_matrix)
#J = input("Enter the number of nodes: ")
#K = input("Enter the number of robots: ")
#K = int(K)
#J = int(J)
#----------------------------------------------------------------------------
#leaf = np.zeros((1,int(K),numpy.s))
#leaf = []s
#for i in range(int(K)):
#    leaf.append(input("Node Name of {i} th robot"))
#print(leaf) #list of node names where robots are placed

#---------------------------------------------------------------------------
#incidence_matrix = np.zeros((J,J))
#get the cordinates and map them to vertex alphabet
#build the matrix accordingly
#incidence_matrix = np.ones((J,J))

#initializing of R : list of Robot objects
R = []
for j in range(K):
    R.append(Robot(j,robo_vertex,incidence_matrix))

#initializing of V : list of Vertex objects
V = []
for j in range(J):
    V.append(Vertex(vertex[j],edges,incidence_matrix))#asdf

#print(R[2].__dict__) # prints all attributes
#print(V[2].__dict__) # prints all attributes

#--------------------------------------------------------------------------
# graph = build_graph(edges)
# edge_path = []
# node_path = find_shortest_path(graph,'A','D')

# for i in range(len(node_path) -1):
#     edge_path.append(node_path[i] + node_path[i+1])
# print(edge_path)

#--------------------------------------------------------------------------

#As the robots are placed on leaf nodes thus the vertex will have only one neighbour at that vertex
#All the rbos are given an initial puch to there immediate neighbors
print("The first mandatory push:")
print('')
for k in range(K):
    #print(R[k].present_location)
    start = R[k].present_location
    #print(V[ord(R[k].present_location) - 65].neighbors[0])
    end = V[ord(R[k].present_location) - 65].neighbors[0]
    #print(-1*incidence_matrix[ord(start) - 65,ord(end)-65])
    top = np.array([-1*incidence_matrix[ord(start) - 65,ord(end)-65]])
    bottom = np.array([-1*incidence_matrix[ord(end) - 65,ord(start)-65]])
    col_vector = np.vstack((top,bottom))
    #print(col_vector)
    #print(np.shape(col_vector))
    #print(bottom)    
    
    #print(np.vstack((top,bottom))) 
    print("The {e} robot is currently at {f}".format(e=k,f=R[k].present_location))
    print("The next node chosen is {}".format(V[ord(R[k].present_location) - 65].neighbors[0]))
    id,R,V = what_to_do_if_next_node_known(R,k,V,1,R[k].present_location,V[ord(R[k].present_location) - 65].neighbors[0],incidence_matrix=incidence_matrix)
    #print(V[ord(R[k].present_location) - 65].__dict__)
    #print(R[k].__dict__)
    R[k].next_edge_decided,count = second_step_on_vertex_visit(graph, V,R,k,count)
    print('The next edge selected by - ' + str(k) + '- robot is' + str(R[k].next_edge_decided))
    print('')
    #publish the setpoint of the node to this drone command
    #id,R,V = what_to_do_if_next_node_known(R,k,V,2,R[k].present_location, R[k].next_edge_decided.replace(R[k].present_location,''),incidence_matrix = incidence_matrix)
    #print(R[k].present_location)
print("This is the loop part which continues till the declaration of completion")
while(count != K):
     for z in range(K):
         if R[z].count != 1:
            print("{z} robot Travelling to the selected edge :{e}".format(z = z , e = R[z].next_edge_decided) )
            if R[z].next_edge_decided !=0:
               id,R,V = what_to_do_if_next_node_known(R,z,V,2,R[z].present_location, R[z].next_edge_decided.replace(R[z].present_location,''),incidence_matrix = incidence_matrix)
               R[z].next_edge_decided,count = second_step_on_vertex_visit(graph, V,R,z,count)
               print('The next edge selected by : ' + str(z) + ' : robot is :' + str(R[z].next_edge_decided))
               print('')
