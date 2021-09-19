import rospy
import numpy as np
import pandas as pd
from ayush.initialize_graph import update_iteration, update_present_location
from ayush.mergematrices import MergeMatrices
from ayush.order_matrix import order_matrix

'''
Class for representing the Completed Edge Attributes, contains:
    - row_tags : tags of the nodes of that edge name
    - top: numpy array of the incidence angle from start node to end node
    - bottom : numpy array of the incidence angke from end node to start node
    - col_vector : numpy array of vertically stacked  entries of top and bottom numpy arrays
    - matrix : Dataframe that can be given as input to merge_matrices()
'''
class Id: 
        def __init__(self,start,end_node,edge,incidence_matrix):
            
            self.row_tags= []
            self.row_tags.append(start)
            self.row_tags.append(end_node)
            self.top = np.array([-1*incidence_matrix[ord(start) - 65,ord(end_node)-65]])
            self.bottom = np.array([-1*incidence_matrix[ord(end_node) - 65,ord(start)-65]])
            self.col_vector = np.vstack((self.top,self.bottom))        
            self.matrix = pd.DataFrame(data = self.col_vector,
                              index = self.row_tags,
                              columns = [edge])
'''
Once we get to know what our next node to travel is we call this function  to take care
Arguments : Robot Object List, robot index, Vertex Object List , iteration, start node name, end node name, full incidence_matrix 
Returns : completed edge incidence matrix, updated Robot Object List, updated Vertex Object List
'''                      
def what_to_do_if_next_node_known(R,k,V,n,start,end_node,incidence_matrix = 0):
    
    if(ord(start) < ord(end_node)):
        edge = start + end_node
    else:
        edge = end_node + start
    id = Id(start,end_node,edge,incidence_matrix)
    [V,R] = first_step_on_arriving_at_vertex(V,ord(end_node) - 65,R,k,id,n)
    return id,R,V

'''
#V : list of Vertex objects , j : arrived vertex index
#R : list of Robot objects , k : robot index 
#id: column vector corresponding to the completed edge traversed in reaching j vertex ; instance of Id class 
#n : at which update the kth robot reaches the jth vertex
'''
def first_step_on_arriving_at_vertex(V,j,R,k,id,n):
    
    if(V[j].iteration == 0):
        V[j].iteration = n-1
    
    [temp,E1_cap] = MergeMatrices(id.matrix,R[k].incidence_matrix) # temp : pandas Dataframe
    
    [R[k].incidence_matrix,E1_cap] = MergeMatrices(temp, V[j].incidence_matrix)
    update_present_location(R[k],id.row_tags[1])
    R[k].iteration += 1
    V[j].iteration += 1

    [R[k].incidence_matrix, C]  = order_matrix(R[k].incidence_matrix,E1_cap)
    V[j].incidence_matrix = R[k].incidence_matrix
    V[j].row_tags = R[k].row_tags
    V[j].edge_tags = R[k].edge_tags
    return V,R
