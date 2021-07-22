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
from ayush.second_step_on_vertex_visit import second_step_on_vertex_visit
from ayush.initialize_graph import Robot,Vertex, build_graph, find_shortest_path
from ayush.first_step_on_vertex_visit import Id,what_to_do_if_next_node_known,first_step_on_arriving_at_vertex
from ayush.get_incidence_matrix import get_incidence_matrix
from ayush.order_matrix import completed,out,unexplored
pp = pprint.PrettyPrinter(indent=8)


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

local_pos_pub = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=1)

rospy.Subscriber('uav1/mavros/state', State, state_cb)
rospy.Subscriber('uav1/mavros/local_position/pose', PoseStamped, position_cb)

#state_sub = rospy.Subscriber(mavros.get_topic('uav1','state'), State, state_cb)
arming_client = rospy.ServiceProxy('uav1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
set_mode_client = rospy.ServiceProxy('uav1/mavros/set_mode', mavros_msgs.srv.SetMode)

pose = PoseStamped()
pose.pose.position.x = 4
pose.pose.position.y = 2
pose.pose.position.z = 2

threshold = 0.1

def algorithm():
    #Topography 
    K = 4
    J = 8
    count = 0
    edges = ["AB","BC","CD","CE","AF","AH","GH"] # change this when implementing for a different topo
    vertex = ["A","B","C","D","E","F","G","H"] # change this when implementing for a different topo
    robo_vertex = ["D","E","F","G"]
    XData = [5.000, 3.000, 3.000, 2, 4, 5.000, 7.000, 7.000]
    YData = [8, 6, 4, 2, 2, 6, 4, 6]
    [graph,edges_decomp] = build_graph(edges)

    G = nx.Graph()
    # --------------------------------------------------------------
    for i in range(J):
        G.add_node(chr(i+65))
        #print('LAMBA')

    for ed in edges_decomp:
        # print(*ed)
        G.add_edge(*ed)
    nx.draw(G,with_labels = True, font_weight = 'bold')
    #--------------------------------------------------------------

    # plt.show
    # print(G.nodes)
    # print(G.edges)

    incidence_matrix = get_incidence_matrix(XData,YData,G)
    # pp.pprint(incidence_matrix)

    R = []
    for j in range(K):
        R.append(Robot(j,robo_vertex,incidence_matrix))

    #initializing of V : list of Vertex objects
    V = []
    for j in range(J):
        V.append(Vertex(vertex[j],edges,incidence_matrix))#asdf


    # print("The first mandatory push:")
    # print('')


    for k in range(K):
        #print(R[k].present_location)
        start = R[k].present_location
        #print(V[ord(R[k].present_location) - 65].neighbors[0])
        end = V[ord(R[k].present_location) - 65].neighbors[0]
        #print(-1*incidence_matrix[ord(start) - 65,ord(end)-65])
        top = np.array([-1*incidence_matrix[ord(start) - 65,ord(end)-65]])
        bottom = np.array([-1*incidence_matrix[ord(end) - 65,ord(start)-65]])
        col_vector = np.vstack((top,bottom))

        
        #print("The {e} robot is currently at {f}".format(e=k,f=R[k].present_location))
        #print("The next node chosen is {}".format(V[ord(R[k].present_location) - 65].neighbors[0]))
        R[k].setpoint_list.append(XData[ord(V[ord(R[k].present_location) - 65].neighbors[0]) - 65])
        R[k].setpoint_list.append(YData[ord(V[ord(R[k].present_location) - 65].neighbors[0]) - 65])
        R[k].setpoint_list.append(k+1)
        rospy.loginfo('Setpoint Addded!')
        id,R,V = what_to_do_if_next_node_known(R,k,V,1,R[k].present_location,V[ord(R[k].present_location) - 65].neighbors[0],incidence_matrix=incidence_matrix)
        #flying_function(XData[ord(V[ord(R[k].present_location) - 65].neighbors[0]) - 65],YData[ord(V[ord(R[k].present_location) - 65].neighbors[0]) - 65],k+1)

        
        

        R[k].next_edge_decided,count = second_step_on_vertex_visit(graph, V,R,k,count)

        #print('The next edge selected by - ' + str(k) + '- robot is' + str(R[k].next_edge_decided))
        R[k].setpoint_list.append(XData[ord(R[k].next_edge_decided.replace(R[k].present_location,'')) - 65])
        R[k].setpoint_list.append(YData[ord(R[k].next_edge_decided.replace(R[k].present_location,'')) - 65])
        R[k].setpoint_list.append(k+1)
        
        rospy.loginfo('Setpoint Addded!')
        #print('')

        #rospy.sleep(6)

    # print("This is the loop part which continues till the declaration of completion")
    while(count != K):
        for z in range(K):
            if R[z].count != 1:
                # print("{z} robot Travelling to the selected edge :{e}".format(z = z , e = R[z].next_edge_decided) )
                #flying_function(XData[ord(R[z].next_edge_decided.replace(R[z].present_location,'')) - 65], YData[ord(R[z].next_edge_decided.replace(R[z].present_location,'')) - 65], z+1 )
                 
                if R[z].next_edge_decided !=0:
                    id,R,V = what_to_do_if_next_node_known(R,z,V,2,R[z].present_location, R[z].next_edge_decided.replace(R[z].present_location,''),incidence_matrix = incidence_matrix)
                    
                    #flying_function(XData[ord(R[z].next_edge_decided.replace(R[z].present_location,'')) - 65], YData[ord(R[z].next_edge_decided.replace(R[z].present_location,'')) - 65], z+1)
                    R[z].next_edge_decided,count = second_step_on_vertex_visit(graph, V,R,z,count)
                    if R[z].next_edge_decided !=0:
                        R[z].setpoint_list.append(XData[ord(R[z].next_edge_decided.replace(R[z].present_location,'')) - 65])
                        R[z].setpoint_list.append(YData[ord(R[z].next_edge_decided.replace(R[z].present_location,'')) - 65])
                        R[z].setpoint_list.append(z+1)
                        rospy.loginfo('Setpoint Addded!')
                        # print('The next edge selected by : ' + str(z) + ' : robot is :' + str(R[z].next_edge_decided))
                        # print('')
    return R[0].setpoint_list,R[1].setpoint_list,R[2].setpoint_list, R[3].setpoint_list

def drone_reached(xdata, ydata, zdata):
    if abs(new_pose.pose.position.x - xdata) <= threshold and abs(new_pose.pose.position.y - ydata) <= threshold and abs(new_pose.pose.position.z - zdata) <= threshold:
        return True        

def position_control():
    
    list_of_setpoints0, list_of_setpoints1, list_of_setpoints2, list_of_setpoints3 = algorithm()
    
    num_of_points = len(list_of_setpoints1)//3
    # print(list_of_setpoints1)
    rospy.init_node('offb_node1', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

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

        while not rospy.is_shutdown() and not drone_reached(list_of_setpoints1[3*i] - 4,list_of_setpoints1[3*i+1] - 2,list_of_setpoints1[3*i+2]):
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
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = list_of_setpoints1[3*i] - 4
            pose.pose.position.y = list_of_setpoints1[3*i+1] - 2
            pose.pose.position.z = list_of_setpoints1[3*i+2]
            local_pos_pub.publish(pose)
            #print('here!!')
            rate.sleep()

if __name__ == '__main__':
    try:
        # print('E')
        position_control()
    except rospy.ROSInterruptException:
        pass