#Requires re check
import rospy
import numpy as np
import pandas as pd

#Again here, the I_merged given by mergematrices.py is a pandas Dataframe
#we'll convert it to np array --> do computations and then reconvert it to pd.DataFrame 
def order_matrix(I_merged, E1_cap):
    #print(I_merged) #latst
    names_I_merged = list(I_merged.index.values) #extracting row_tags
    edges_I_merged = list(I_merged.columns.values) #extracting edge_tags
    #print(names_I_merged) ##latst
    #print(edges_I_merged) #latst
    I = I_merged.to_numpy() # converting to numpy array
    [V,E] = np.shape(I)
    E2_cap = E - E1_cap
    I1 = I[:,0:E1_cap]
    I2 = I[:,E1_cap:E]
    
    [a, comp_I1] = completed(I1)
    [a1, comp_I2] = completed(I2)

    if len(comp_I2) != 0:
        comp_I2 = [e + E1_cap for e in comp_I2]
    completed_edge_count = len(comp_I1) + len(comp_I2) 

    [b,out_I1] = out(I1)
    [b1, out_I2] = out(I2)

    if len(out_I2) != 0:
        out_I2 = [e + E1_cap for e in out_I2]

    [c,unexp_I1] = unexplored(I1)
    [c1, unexp_I2] = unexplored(I2)

    if len(unexp_I2) != 0:
        unexp_I2 = [e + E1_cap for e in unexp_I2]
    #print(a) #latst
    #print(a1) #latst
    #print(b) #latst
    #print(b1) #latst
    #print(c) #latst
    #print(c1) #latst
    temp_list = [a,a1,b,b1,c,c1]
    copy_list = [a,a1,b,b1,c,c1]
    for w in temp_list :
        if len(w) == 0:
            copy_list.remove(w)
            #print("YOYO") #latst
        #print(copy_list) #latst
    #I_ordered = np.concatenate((a,a1,b,b1,c,c1),axis=0) # converting back to pandas DataFrame
    I_ordered = np.concatenate(copy_list,axis=0)
    I_ordered = np.transpose(I_ordered)
    column_index = comp_I1+ comp_I2 + out_I1 + out_I2 + unexp_I1 + unexp_I2
    I_ordered = pd.DataFrame(data = I_ordered,
                index = names_I_merged,
                #columns = edges_I_merged[np.concatenate((comp_I1,comp_I2,out_I1,out_I2,unexp_I1,unexp_I2),axis=0)]
                columns = [edges_I_merged[i] for i in column_index]
                
                )
    return I_ordered, completed_edge_count

def completed(merged_matrix):
    [r,c] = np.shape(merged_matrix)
    temp = merged_matrix
    completed_edges = []
    comp_index = []
    for i in range(c):
        if(np.count_nonzero(temp[:,i]) ==2):
            completed_edges.append(merged_matrix[:,i]) #concatenated column vectors classified as completed edges
            comp_index.append(i) #list of index of columns classified as completed edges
    return completed_edges, comp_index

def out(merged_matrix):
    [r,c] = np.shape(merged_matrix)
    temp = merged_matrix
    out_edges = []
    out_index = []
    for i in range(c):
        if(np.sum(temp[:,i] < 0) ==1):
            out_edges.append(merged_matrix[:,i]) #concatenated column vectors classified as out edges
            out_index.append(i) #list of index of columns classified as out_edges
    return out_edges, out_index

def unexplored(merged_matrix):
    [r,c] = np.shape(merged_matrix)
    temp = merged_matrix
    unexplored_edges = []
    unexplored_index = []
    for i in range(c):
        if(np.sum(temp[:,i] < 0) !=1) and (np.count_nonzero(temp[:,i]) !=2):
            unexplored_edges.append(merged_matrix[:,i]) #concatenated column vectors classified as unexplored edges
            unexplored_index.append(i) #list of index of columns classified as unexp_edges
    return unexplored_edges,unexplored_index