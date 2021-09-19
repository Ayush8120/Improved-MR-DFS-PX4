import rospy
import numpy as np
import pandas as pd
#I1 , I2 can be incidence matrices of Robot,Vertex or Completed Edge,Vertex 
#outside of this function they are initialised as pandas DataFrame
#we'll need to convert them to nparray after calling this function
'''
Merge Matrices Algorithm : Helps merge 2 matrices by removing redundant entries. 
    - updates row tags
    - updates edge tags
    - updates the entries according to the convention defined in the paper
Returns: Merged Incidence Matrix , Updated count of columns of I1 Matrix     
'''

def MergeMatrices(I1,I2):
    #I1 : pandas Dataframe
    #I2 : pandas DataFrame
    
    I1_names = list(I1.index.values)
    I2_names = list(I2.index.values)

    I1_col_names = list(I1.columns)
    I2_col_names = list(I2.columns)

    names = I1_names + I2_names
    edge_names = I1_col_names + I2_col_names
    
    I1 = I1.to_numpy()
    I2 = I2.to_numpy()

    [V1,E1] = np.shape(I1)
    [V2,E2] = np.shape(I2)

    V_cap = V1 + V2
    E1_cap = E1
    E2_cap = E2

    
    I = np.zeros((V_cap,E1_cap + E2_cap),float)
    I[0:V1,0:E1_cap] = I1
    I[V1:V_cap,E1_cap : E1_cap+E2_cap] = I2
    delj = []
    deli = []
    
    for i1 in range(V1):
        for j1 in range(E1):
            for i2 in range(V1,V1+V2):
                for j2 in range(E1,E1 + E2):              

                    if((vertex_tag(names,i1) == vertex_tag(names,i2)) and (abs(I[i1,j1]) == abs(I[i2,j2])) and I[i1,j1] != 0 ): 
                        if(np.sign(I[i1,j1]) != np.sign(I[i2,j2])):
                            c = -abs(I[i1,j1])
                            I[i1,j1] = c
                            I[i2,j2] = c   
                            
                        if(np.count_nonzero(I[V1:V1+V2,j2]) == 2):
                            delj.append(j1)
                            E1_cap -= 1
                        elif (np.count_nonzero(I[0:V1,j1]) == 2):
                            delj.append(j2)
                            E2_cap -= 1
                        else:
                            delj.append(j1)
                            E1_cap -= 1
                        if (np.count_nonzero(I[V1:V1+V2,j1]) < 2):
                            I[V1:V1+V2,j1] += I[V1:V1+V2,j2]
                        if (np.count_nonzero(I[0:V1,j2]) < 2):
                            I[0:V1,j2] += I[0:V1,j1]
                        deli.append(i1)
    
    deli  = list(set(deli))
    delj = list(set(delj))
    
    I = np.delete(I,deli,0)
        
    deli.sort()
    delj.sort()
    for i in deli: 
        del names[i]
        deli[:] = [x-1 for x in deli]
    names_final = names
    
    I = np.delete(I,delj,1)
    for j in delj: 
        del edge_names[j]
        delj[:] = [y-1 for y in delj]
    edge_names_final = edge_names
    
#Re-converting to pandas Dataframe    
    I = pd.DataFrame(data = I,    # values
                index = names_final,    # 1st column as index
                columns = edge_names_final) 
    return I , E1_cap
    
'''
Gives the vertex tag(node name) corresponding to the 
    - name_matrix : list obtained after concatenation of node names of I1 and I2
    - index 
'''
def vertex_tag(names_matrix,i):
    return names_matrix[i]