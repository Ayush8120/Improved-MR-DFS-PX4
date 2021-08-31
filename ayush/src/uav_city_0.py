#!/usr/bin/env python3

import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *

import pprint
import networkx as nx
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

from ayush.second_step_on_vertex_visit import second_step_on_vertex_visit
from ayush.initialize_graph import Robot,Vertex, build_graph, find_shortest_path
from ayush.first_step_on_vertex_visit import Id,what_to_do_if_next_node_known,first_step_on_arriving_at_vertex
from ayush.get_incidence_matrix import get_incidence_matrix
from ayush.order_matrix import completed,out,unexplored
pp = pprint.PrettyPrinter(indent=8)

spawn_location = [-24.1095,284.1490,8.63479,
                  -22.5968,284.1330,8.63479,
                  -21.0750,283.9410,8.63479,
                  -24.1239,282.6820,8.63479,
                  -23.0024,282.5940,8.63479,
                  -22.1617,282.5870,8.63479,
                  -21.0457,282.5580,8.63479,
                  -24.1631,281.1710,8.63479,
                  -22.5350,281.0980,8.63479,
                  -20.9154,281.1750,8.63479
                  ]

# callback method for state sub
current_state = State() 
offb_set_mode = SetMode
new_pose = PoseStamped()

def state_cb(state):
    global current_state
    current_state = state

def position_cb(Pose):
    global new_pose
    new_pose = Pose

local_pos_pub = rospy.Publisher('uav0/mavros/setpoint_position/local', PoseStamped, queue_size=1)

rospy.Subscriber('uav0/mavros/state', State, state_cb)
rospy.Subscriber('uav0/mavros/local_position/pose', PoseStamped, position_cb)

#state_sub = rospy.Subscriber(mavros.get_topic('uav1','state'), State, state_cb)
arming_client = rospy.ServiceProxy('uav0/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
set_mode_client = rospy.ServiceProxy('uav0/mavros/set_mode', mavros_msgs.srv.SetMode)

pose = PoseStamped()
#Leaf node cordinates with elevation
pose.pose.position.x = spawn_location[0]             #CHANGE TODO # Hover over spawn location # personal
pose.pose.position.y = spawn_location[1]
pose.pose.position.z = spawn_location[2] + 1 
threshold = 0.1

# ANIMATION FUNCTION
def func(num, dataSet, line, redDots):
    # NOTE: there is no .set_data() for 3 dim data...
    line.set_data(dataSet[0:2, :num])    
    line.set_3d_properties(dataSet[2, :num])    
    redDots.set_data(dataSet[0:2, :num])    
    redDots.set_3d_properties(dataSet[2, :num]) 
    return line

def algorithm():
    #Topography 
    K = 10
    J = 24
    count = 0
    vertex = ["A","B","C","D","E","F","G","H","I","J","K","L","M","N","O","P","Q","R","S","T","U","V","W","X"]
    edges = ["AC","BC","CD","DE","EF","EK","FG","GH","HI","HJ","KL","KO","LM","LN","OP","PQ","PS","QR","ST","SU","UV","VW","VX"]
    robo_vertex = ["A","B","I","J","M","N","R","T","W","X"]
    XData = [2.50, 3.50,3,3,3,1.50,1.50,1.50,1,2,3.50,3,2.50,3.50,4.50,4.50,3.50,3.50,5,4.50,5.50,5.50,5,6]
    YData = [1,1,2,3,4,5,6,7,8,8,5,6,7,7,6,7,8,9,8,9,9,10,11,11] # Got from MATLAB
    [graph,edges_decomp] = build_graph(edges)
    global spawn_location
    G = nx.Graph()
    # --------------------------------------------------------------
    for i in range(J):
        G.add_node(chr(i+65))
        #print('LAMBA')

    for ed in edges_decomp:
        # print(*ed)
        G.add_edge(*ed)
    # nx.draw(G,with_labels = True, font_weight = 'bold')
    #--------------------------------------------------------------

    #plt.show
    #print(G.nodes)
    #print(G.edges)

    incidence_matrix = get_incidence_matrix(XData,YData,G)
    #pp.pprint(incidence_matrix)

    R = []
    for j in range(K):
        R.append(Robot(j,robo_vertex,incidence_matrix))

    #initializing of V : list of Vertex objects
    V = []
    for j in range(J):
        V.append(Vertex(vertex[j],edges,incidence_matrix))#asdf


    #print("The first mandatory push:")
    #print('')


    for k in range(K):

        R[k].setpoint_list.append(spawn_location[3*k])
        R[k].setpoint_list.append(spawn_location[3*k + 1])
        R[k].setpoint_list.append(spawn_location[3*k +2] + k+1)
        #print(R[k].present_location)
        start = R[k].present_location
        #print(V[ord(R[k].present_location) - 65].neighbors[0])
        end = V[ord(R[k].present_location) - 65].neighbors[0]
        #print(-1*incidence_matrix[ord(start) - 65,ord(end)-65])
        top = np.array([-1*incidence_matrix[ord(start) - 65,ord(end)-65]])
        bottom = np.array([-1*incidence_matrix[ord(end) - 65,ord(start)-65]])
        col_vector = np.vstack((top,bottom))
        #Setpoint - 1 as the leaf node location while the height is kept constant
        R[k].setpoint_list.append(XData[ord(R[k].present_location) - 65]*25)
        R[k].setpoint_list.append(YData[ord(R[k].present_location) - 65]*25)
        R[k].setpoint_list.append(spawn_location[3*k +2] + k+1)
        
        #print("The {e} robot is currently at {f}".format(e=k,f=R[k].present_location))
        #print("The next node chosen is {}".format(V[ord(R[k].present_location) - 65].neighbors[0]))
        
        R[k].setpoint_list.append(XData[ord(V[ord(R[k].present_location) - 65].neighbors[0]) - 65]*25)
        R[k].setpoint_list.append(YData[ord(V[ord(R[k].present_location) - 65].neighbors[0]) - 65]*25)
        R[k].setpoint_list.append(spawn_location[3*k +2] + k+1)
        rospy.loginfo('Setpoint Addded!')
        id,R,V = what_to_do_if_next_node_known(R,k,V,1,R[k].present_location,V[ord(R[k].present_location) - 65].neighbors[0],incidence_matrix=incidence_matrix)
        #flying_function(XData[ord(V[ord(R[k].present_location) - 65].neighbors[0]) - 65],YData[ord(V[ord(R[k].present_location) - 65].neighbors[0]) - 65],k+1)
        R[k].next_edge_decided,count = second_step_on_vertex_visit(graph, V,R,k,count)

        #print('The next edge selected by - ' + str(k) + '- robot is' + str(R[k].next_edge_decided))
        R[k].setpoint_list.append(XData[ord(R[k].next_edge_decided.replace(R[k].present_location,'')) - 65]*25)
        R[k].setpoint_list.append(YData[ord(R[k].next_edge_decided.replace(R[k].present_location,'')) - 65]*25)
        R[k].setpoint_list.append(spawn_location[3*k +2] + k+1)
        rospy.loginfo('Setpoint Addded!')
        #print('')

        #rospy.sleep(6)

    #print("This is the loop part which continues till the declaration of completion")
    while(count != K):
        for z in range(K):
            if R[z].count != 1:
                #print("{z} robot Travelling to the selected edge :{e}".format(z = z , e = R[z].next_edge_decided) )
                if R[z].next_edge_decided !=0:
                    id,R,V = what_to_do_if_next_node_known(R,z,V,2,R[z].present_location, R[z].next_edge_decided.replace(R[z].present_location,''),incidence_matrix = incidence_matrix)
                    #flying_function(XData[ord(R[z].next_edge_decided.replace(R[z].present_location,'')) - 65], YData[ord(R[z].next_edge_decided.replace(R[z].present_location,'')) - 65], z+1)              
                    R[z].next_edge_decided,count = second_step_on_vertex_visit(graph, V,R,z,count)
                    if R[z].next_edge_decided !=0:
                        R[z].setpoint_list.append(XData[ord(R[z].next_edge_decided.replace(R[z].present_location,'')) - 65]*25)
                        R[z].setpoint_list.append(YData[ord(R[z].next_edge_decided.replace(R[z].present_location,'')) - 65]*25)
                        R[z].setpoint_list.append(spawn_location[3*z +2] + z+1)
                        rospy.loginfo('Setpoint Addded!')
                        #print('The next edge selected by : ' + str(z) + ' : robot is :' + str(R[z].next_edge_decided))
                        #print('')
    for k in range(K):
        R[k].setpoint_list.append(spawn_location[3*k])
        R[k].setpoint_list.append(spawn_location[3*k + 1])
        R[k].setpoint_list.append(spawn_location[3*k + 2] +k+1)

    return R[0].setpoint_list,R[1].setpoint_list,R[2].setpoint_list, R[3].setpoint_list,R[4].setpoint_list,R[5].setpoint_list,R[6].setpoint_list,R[7].setpoint_list,R[8].setpoint_list,R[9].setpoint_list

def drone_reached(xdata, ydata, zdata):
    if abs(new_pose.pose.position.x - xdata) <= threshold and abs(new_pose.pose.position.y - ydata) <= threshold and abs(new_pose.pose.position.z - zdata) <= threshold:
        return True        

def position_control():
    
    list_of_setpoints0,_,_,_,_,_,_,_,_,_ = algorithm()
    
    num_of_points = len(list_of_setpoints0)//3
    print(list_of_setpoints0)
    rospy.init_node('offb_node0', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    dataSet = np.array(list_of_setpoints0)
    numDataPoints = len(list_of_setpoints0)//3
    dataSet = np.reshape(dataSet,(numDataPoints,3))
    dataSet = np.transpose(dataSet)
    print(dataSet)
    # GET SOME MATPLOTLIB OBJECTS
    fig = plt.figure()
    ax = Axes3D(fig)
    redDots = plt.plot(dataSet[0], dataSet[1], dataSet[2], lw=2, c='r', marker='o')[0] # For scatter plot
    # NOTE: Can't pass empty arrays into 3d version of plot()
    line = plt.plot(dataSet[0], dataSet[1], dataSet[2], lw=2, c='g')[0] # For line plot
    
    # AXES PROPERTIES]
    # ax.set_xlim3d([limit0, limit1])
    ax.set_xlabel('X(t)')
    ax.set_ylabel('Y(t)')
    ax.set_zlabel('Z(t)')
    ax.set_title('Trajectory of 0st UAV')
    
    # Creating the Animation object
    line_ani = animation.FuncAnimation(fig, func, frames=numDataPoints, fargs=(dataSet,line,redDots), interval=150, blit=False)
    line_ani.save(r'0_th_UAV_animation.gif')
    # send a few setpoints before starting
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()
    
    # wait for FCU connection
    while not current_state.connected:
        rate.sleep()
    # print(num_of_points)
    
    for i in range(num_of_points):
        # print(i)
        last_request = rospy.get_rostime()
        if i ==2:
            ayushtrial = input()
        else:
            ayushtrial = 1
        while not rospy.is_shutdown() and not drone_reached(list_of_setpoints0[3*i] - spawn_location[0],list_of_setpoints0[3*i+1] - spawn_location[1],list_of_setpoints0[3*i+2] - spawn_location[2]):
            now = rospy.get_rostime()
            if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(4.)):
                set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                last_request = now 
            else:
                if not current_state.armed and (now - last_request > rospy.Duration(4.)):
                    arming_client(True)
                    last_request = now 

            # older versions of PX4 always return success==True, so better to check Status instead
            if prev_state.armed != current_state.armed:
                rospy.loginfo("Vehicle armed: %r" % current_state.armed)
            if prev_state.mode != current_state.mode: 
                rospy.loginfo("Current mode: %s" % current_state.mode)
            prev_state = current_state
            
            # new_x,new_y,new_z = 
            # Update timestamp and publish pose 
            #pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = list_of_setpoints0[3*i] - spawn_location[0] #initial x
            pose.pose.position.y = list_of_setpoints0[3*i+1] - spawn_location[1]#initial y
            pose.pose.position.z = list_of_setpoints0[3*i+2] - spawn_location[2] #initial_height
            local_pos_pub.publish(pose)
            #print('here!!')
            # plt.scatter(new_pose.pose.position.x/50 , new_pose.pose.position.y/50, marker = '.', color='green')
            # plt.draw()
            
            rate.sleep()
    

if __name__ == '__main__':
    try:
        #print('D')
        position_control()
        # plt.savefig("0_city_uav_route_yo_final_run.png")

    except rospy.ROSInterruptException:
        
        pass
