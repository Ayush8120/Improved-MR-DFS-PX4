import rospy
import numpy as np
import pandas as pd

from ayush.order_matrix import completed,out,unexplored
from ayush.initialize_graph import find_shortest_path

def second_step_on_vertex_visit(graph,Vobj,Robj,k,count):
    #check if the robot's incidence matrix's last column is complted edge
    #if yes then HALT this robot. 
    
    #send the control to next k(get out of this funnction + increase k )

    #if no:
    #Q = []
    Q = Robj[k].Q
    j = ord(Robj[k].present_location)-65 #Vertex Name of present location
    j_name = Robj[k].present_location
    #print(j_name)
    #I = Vobj[j].incidence_matrix
    I = Robj[k].incidence_matrix
    [r,c] = I.shape #returns the shape of numpy array if I is dataframe
    I_nparray = I.to_numpy()
    Edge_chosen = I.columns.values[c-1] # Edge_Name
    I_decision = np.array(I.loc[:,Edge_chosen]) #creates row vector
    I_decision = np.reshape(I_decision, (r,1))
    #[_,comp_ind] = completed(I_decision)
    [_,comp_ind] = completed(I_nparray)
    Ec = len(comp_ind)
    #print("Number of completed edges" + str(Ec)) #latst
    [known_vertex,_] = np.nonzero(I_decision)#indice
    
    known_vertex = I.index.values[known_vertex] #Alphabet
    #print(known_vertex) #latst
    
    if np.count_nonzero(I_decision)==2:
        print("Exploration done for the robot number: {}".format(k))
        count +=1
        Robj[k].count = 1
        return 0,count 

    elif len(Q) == 0 or np.count_nonzero(I.loc[:,Q[-1]])==2:#np.count_nonzero(I.loc[:,Edge_chosen] == 2): # time to update the Q
        
        edge_path = []
        node_path = find_shortest_path(graph, j_name,known_vertex)
        #print("The Node Path is :")
        #print(node_path)
        #latst
        #print("Below is node path")
        #print(node_path)
        #latst
        for i in range(len(node_path) -1):
            edge_path.append(node_path[i] + node_path[i+1])
        
        #print("Below is edge path") #latst
        #print(edge_path) #latst
        Q = edge_path
        Q.append(Edge_chosen)
        #print("Below is Q list") #latst
        #print(Q) #latst
    if len(node_path) == 0:
        next_edge = Q[0]
    else:
        next_edge = Q[0]
    if I_decision[np.nonzero(I_decision)] > 0:
        Robj[k].incidence_matrix.iloc[:,-1] = -1*Robj[k].incidence_matrix.iloc[:,-1]
    temp = pd.DataFrame(data = np.roll(Robj[k].incidence_matrix.iloc[:,Ec:c].values,1,1) , index = Robj[k].incidence_matrix.index, columns = np.roll(Robj[k].incidence_matrix.columns.values[Ec:c] , 1 , 0)) 
    const = np.array(Robj[k].incidence_matrix.iloc[:,0:Ec])
    
    #latst
    #print("This is const array - the un rotated one")
    #print(const)
    #print("This is temp array")
    #print(temp.to_numpy())
    #print(list(Robj[k].incidence_matrix.columns[0:Ec]) + list(temp.columns))
    #latst 
   
    Robj[k].incidence_matrix = pd.DataFrame(data = np.concatenate((const,temp.to_numpy()),axis = 1),
                                            index = Robj[k].incidence_matrix.index ,
                                            columns= list(Robj[k].incidence_matrix.columns[0:Ec]) + list(temp.columns) )

    Vobj[j].incidence_matrix = Robj[k].incidence_matrix
    return next_edge, count