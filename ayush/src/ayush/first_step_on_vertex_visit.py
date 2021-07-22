#Python

import rospy
import numpy as np
import pandas as pd
from ayush.initialize_graph import update_iteration, update_present_location
from ayush.mergematrices import MergeMatrices
from ayush.order_matrix import order_matrix

class Id: #tick
        def __init__(self,start,end_node,edge,incidence_matrix):
            
            self.row_tags= []
            #self.row_tags = [start] + [end_node]
            self.row_tags.append(start)
            self.row_tags.append(end_node)
            #print(self.row_tags)

            self.top = np.array([-1*incidence_matrix[ord(start) - 65,ord(end_node)-65]])
            self.bottom = np.array([-1*incidence_matrix[ord(end_node) - 65,ord(start)-65]])
            #self.col_vector = np.vstack(((-1*incidence_matrix[ord(start) - 65,ord(end)-65]),(-1*incidence_matrix[ord(end) - 65,ord(start)-65])))
            self.col_vector = np.vstack((self.top,self.bottom))        
            self.matrix = pd.DataFrame(data = self.col_vector,
                              index = self.row_tags,
                              columns = [edge])
            
          
#Edits Required : incidence matrix, giving setpoints, passing dictionary with keys as vertex names and entry as setpoints
def what_to_do_if_next_node_known(R,k,V,n,start,end_node,incidence_matrix = 0):
    
    
    
    if(ord(start) < ord(end_node)):
        edge = start + end_node
    else:
        edge = end_node + start

    id = Id(start,end_node,edge,incidence_matrix)


    #highlight present node
    #getting edge traversed


    
    #giving setpoints to the drone
    #drone jayega -------> #highlight the edge 
    #landing
    #dropping beacon
    #publish ---> reached the vertex
    [V,R] = first_step_on_arriving_at_vertex(V,ord(end_node) - 65,R,k,id,n)
    #publish ----> updated the vertex_incidence_matrix
    
    return id,R,V


#V : list of objects , j : we arrive at the j vertex -index
#R : list of objects , k : the kth robot reaches here
#id: column vector corresponding to the completed edge traversed in reaching j vertex
#  : instance of Id class 
#n : at which update the kth robot reaches the jth vertex
def first_step_on_arriving_at_vertex(V,j,R,k,id,n):
    
    if(V[j].iteration == 0):
        V[j].iteration = n-1
    

    [temp,E1_cap] = MergeMatrices(id.matrix,R[k].incidence_matrix) # temp : pandas Dataframe
    #print(temp)
    #[I,E1_cap] = MergeMatrices(temp.incidence_matrix, V[j-64].incidence_matrix, temp.row_tags, V[j-64].row_tags, temp.edge_tags, V[j-64].edge_tags)
    [R[k].incidence_matrix,E1_cap] = MergeMatrices(temp, V[j].incidence_matrix)
    #R[k].present_location = id.row_tags[1] # updating present location
    update_present_location(R[k],id.row_tags[1])
    #update_iteration(R[k],)
    R[k].iteration += 1
    V[j].iteration += 1

    [R[k].incidence_matrix, C]  = order_matrix(R[k].incidence_matrix,E1_cap)
    V[j].incidence_matrix = R[k].incidence_matrix
    V[j].row_tags = R[k].row_tags
    V[j].edge_tags = R[k].edge_tags
    return V,R
