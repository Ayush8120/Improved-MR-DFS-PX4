# Improved-MR-DFS-PX4
Improved Multi-Robot Depth First Search Algorithm implementation using iris drones using PX4.
Performed on Ubuntu 18.04 - Gazebo 9



[YouTube Video - City Simulation](https://youtu.be/wWeWIQ98zS8)

https://user-images.githubusercontent.com/72944387/127710362-c381c48e-98b4-404c-879e-27c8fcb8d026.mp4


The algorith has its implementation in [MATLAB](https://github.com/Ayush8120/MR-DFS) but in this repository we focus on python + ROS + Gazebo simulations


[Step 1](https://docs.px4.io/master/en/ros/mavros_installation.html) MAVROS installation
In case you feel stuck with the steps here is a YouTube video to sail you through this step -[Video](https://www.youtube.com/watch?v=jBTikChu02E) 


[Step 2](https://docs.px4.io/master/en/simulation/ros_interface.html) Getting 1 drone in world

---
	cd <PX4-Autopilot_clone>
	DONT_RUN=1 make px4_sitl_default gazebo
	source ~/catkin_ws/devel/setup.bash    # (optional)
	source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
	roslaunch px4 posix_sitl.launch	
---

[Bonus](https://docs.px4.io/master/en/simulation/gazebo.html#set_world) Loading a specific world and other additional features

[Step 3](https://docs.px4.io/master/en/simulation/multi_vehicle_simulation_gazebo.html) Multiple Vehicles with ROS and Gazebo 
---
	Clone the PX4-Autopilot Git Repository[link](https://github.com/PX4/PX4-Autopilot)
		cd Firmware_clone
		git submodule update --init --recursive
		DONT_RUN=1 make px4_sitl_default gazebo
		source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
		export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo
		roslaunch px4 multi_uav_mavros_sitl.launch
---

-------------
Simulation of our program
----------------------------
- Models of Police Station, landing mat and grass plane are kept in the [models](https://github.com/Ayush8120/Improved-MR-DFS-PX4/tree/main/models) file. Add them to  your models folder at the location PX4-firware-clone/Tools/sitl_gazebo/models
- Add the [world](https://github.com/Ayush8120/Improved-MR-DFS-PX4/tree/main/worlds) files at PX4-firware-clone/Tools/sitl_gazebo/worlds
- replace [these](https://github.com/Ayush8120/Improved-MR-DFS-PX4/tree/main/launch) launch files with the ones already present at PX4-firware-clone/launch
- make a catkin workspace and add [ayush](https://github.com/Ayush8120/Improved-MR-DFS-PX4/tree/main/ayush) package to it. This contains all code and nodes that need to be run for simulation
-------------

---
	roslaunch px4 multi_uav_macros_sitl.launch
	roslaunch ayush city.launch 
---
This will launch the empty world contatining grass plane, Police Station, Landing Mat and 10 Iris Drones.

-------------------------
Descripton of The World & Simulation Specifices
----------------------------
In ayush package there are 10 nodes corresponding to 10 UAVs used for city simulation. 
- 10 iris drones take off from the roof of the station
- go at the leaf nodes and wait for further command 
- as soon as they are given command they start exploration
- upon finishing the exploration they return back to the base station
  
Results for the city simulation are kept in [City Simulation Results](https://github.com/Ayush8120/Improved-MR-DFS-PX4/tree/main/City%20Simulation%20Results) folder.


If you wish to play around with the dimensions/color of the drone then you can edit them at PX4-firmware-clone/Tools/sutl_gazebo/models/iris/iris.sdf.jinja

![8th Robot Path GIF ](https://raw.githubusercontent.com/Ayush8120/Improved-MR-DFS-PX4/main/City%20Simulation/8_th_UAV_animation.gif)

The tree structure's cordinates were calculated using a MATLAB program (link to the MATLAB repository - with the 10 leaf node program)


![Tree STructure Used](https://user-images.githubusercontent.com/72944387/127738425-48e0018a-57c9-4310-83b7-173cfb439662.jpg)

