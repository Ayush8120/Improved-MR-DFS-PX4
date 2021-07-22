import rospy
import math
import numpy as np
from networkx.linalg.graphmatrix import incidence_matrix

def get_incidence_matrix(XData, YData, G):

    incidence_angle_matrix = np.zeros((len(XData),len(XData)))
   
    for i in range(len(XData)):                     #start point
        for j in range(len(XData)):                 #%end point
            if(i==j):
                incidence_angle_matrix[i,j] = 0
            else:
                if (math.atan2(YData[j] - YData[i],XData[j] - XData[i]) <= 0):
                    incidence_angle_matrix[i,j] = math.atan2(YData[j] - YData[i],XData[j] - XData[i]) + 2*math.pi
                elif (math.atan2(YData[j] - YData[i],XData[j] - XData[i]) > 0):
                    incidence_angle_matrix[i,j] = math.atan2(YData[j] - YData[i],XData[j] - XData[i])
                
    return incidence_angle_matrix
