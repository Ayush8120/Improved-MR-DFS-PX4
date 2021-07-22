#needs re-check(of utility)
import rospy
import numpy as np
import pandas as pd
#I1 , I2 are incidence matrices of R,V or Edge,V 
#out of this function they are initialised as pandas DataFrame
# we'll need to convert them to nparray after calling this function
def MergeMatrices(I1,I2):
    #I1 : pandas Dataframe
    #I2 : pandas DataFrame
    #count = 0
    
    #print(I1.index) #latst
    #print(I2.index) #latst

    ##I1_names = np.array(list(I1.index.values))
    ##I1_names = np.reshape(I1_names,(1,len(list(I1.index.values))))
    ##I2_names = np.array(list(I2.index.values))
    ##I2_names = np.reshape(I2_names,(1,len(list(I2.index.values))))

    ##I1_names = np.transpose(I1_names)
    ##I2_names = np.transpose(I2_names)

    # I1_names = np.array(list(I1.index))-----------------try2
    # I1_names = np.reshape(I1_names,(I1_names.size,1))
    # I2_names = np.array(list(I2.index))
    # I2_names = np.reshape(I2_names,(I2_names.size,1))

    I1_names = list(I1.index.values)
    I2_names = list(I2.index.values)

    #print(I1_names) #latst
    #print(I2_names) #latst



    ##I1_col_names = list(I1.columns.values)
    ##I1_col_names = np.reshape(I1_col_names,(1,len(list(I1.columns.values))))
    ##I2_col_names = list(I2.columns.values)
    ##I2_col_names = np.reshape(I2_col_names,(1,len(list(I2.columns.values))))

    # I1_col_names = np.array(list(I1.columns)) -----------------try2
    # I1_col_names = np.reshape(I1_col_names,(1,len(list(I1.columns))))
    # I2_col_names = np.array(list(I2.columns))
    # I2_col_names = np.reshape(I2_col_names,(1,len(list(I2.columns))))

    I1_col_names = list(I1.columns)
    I2_col_names = list(I2.columns)

    #names = np.vstack((I1_names,I2_names)) #get_row_tags undefined
    #edge_names = np.hstack((I1_col_names,I2_col_names))#get_edge_tags_undefined
    

    # names = np.concatenate((I1_names,I2_names),axis= 0 ) -----------------try2
    # edge_names = np.concatenate((I1_col_names,I2_col_names) , axis= 1)

    names = I1_names + I2_names
    edge_names = I1_col_names + I2_col_names
    # print(names)
    # print(edge_names)

    #print(I1)
    #print(I2)

    I1 = I1.to_numpy()
    I2 = I2.to_numpy()

    [V1,E1] = np.shape(I1)
    [V2,E2] = np.shape(I2)

    V_cap = V1 + V2
    E1_cap = E1
    E2_cap = E2

    #print(E1)
    #print(E2)
    #print(V1)
    #print(V2)

    
    I = np.zeros((V_cap,E1_cap + E2_cap),float)
    I[0:V1,0:E1_cap] = I1
    I[V1:V_cap,E1_cap : E1_cap+E2_cap] = I2
    #print("RAW I Matrix ::") #latst
    #print(I) # latst
    delj = []
    deli = []
    # print("I1")
    # print(I1)
    # print("I2")
    # print(I2)
    # #print()
    # print("V1 : " + str(V1))
    # print("E1 : " + str(E1))
    # print("V2 : " + str(V2))
    # print("E2 : " + str(E2))

    for i1 in range(V1):
        #print(i1)
        for j1 in range(E1):
            for i2 in range(V1,V1+V2):
                for j2 in range(E1,E1 + E2):
                                        

                    if((vertex_tag(names,i1) == vertex_tag(names,i2)) and (abs(I[i1,j1]) == abs(I[i2,j2])) and I[i1,j1] != 0 ): 
                        if(np.sign(I[i1,j1]) != np.sign(I[i2,j2])):
                            c = -abs(I[i1,j1])
                            I[i1,j1] = c
                            I[i2,j2] = c   
                            # print("C" + str(c))
                            # print("i1" + str(j1))
                            # print("i2" + str(j2)) 
                            #print("AYUSH") #latst
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
                        #print("i1 ki value:" + str(i1)) #latst
                        deli.append(i1)
    
    #latst
    #print("This is the I np matrix before deletion")
    #print(I)
    #print("This below is deli")
    # print(deli)
    # print(delj)
    deli  = list(set(deli))
    delj = list(set(delj))
    # print(deli)
    # print(delj)
    #latst

    I = np.delete(I,deli,0)
    
    #latst
    #print("This is the I np matrix after deletion of common vertex")
    #print(I)
    #latst
    
    
    #names_final = np.delete(names,deli,0)#------------------try2
    deli.sort()
    delj.sort()
    for i in deli: 
        del names[i]
        deli[:] = [x-1 for x in deli]
    names_final = names
    #names_final = np.transpose(names_final)
    #names_final = filter(lambda x: x != "", names_final)
    #names_final = list(names_final)
    
    #latst
    #print("This is the vertex names final after deletion of common vertex")
    #print(names_final)
    #print("This below is delj")
    #print(delj)
    #latst


    I = np.delete(I,delj,1)
    #print("This is the I np matrix after deletion of common edges") #latst
    #edge_names_final = np.delete(edge_names,delj,1)#-------------------try2 
    for j in delj: 
        del edge_names[j]
        delj[:] = [y-1 for y in delj]
    edge_names_final = edge_names
    #latst
    #print("This is the edge names final after deletion of common edges")
    #print(edge_names_final)
    #latst
    # print(edge_names_final)
    # print(names_final)
    # print(I)
#Re-converting to pandas Dataframe    
    I = pd.DataFrame(data = I,    # values
                index = names_final,    # 1st column as index
                columns = edge_names_final)
    #latst
    #print("This is the merged matrix-|>")
    #print(I)
    #latst 
    return I , E1_cap

def vertex_tag(names_matrix,i):
    #print(i)
    #print(names_matrix)
    return names_matrix[i]

