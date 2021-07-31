# Improved-MR-DFS-PX4
Improved Multi-Robot Depth First Search Algorithm implementation using iris drones using PX4.
Performed on Ubuntu 18.04 - Gazebo 9



[YouTube Video - City Simulation](https://youtu.be/0yq5XbTRqt4)

#City Tree Structure


https://user-images.githubusercontent.com/72944387/127738425-48e0018a-57c9-4310-83b7-173cfb439662.jpg

#City-Simulation

https://user-images.githubusercontent.com/72944387/127710362-c381c48e-98b4-404c-879e-27c8fcb8d026.mp4



The algorith has its implementation in [MATLAB](link) but in this repository we focus on python + ROS + Gazebo simulations


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
- Replace empty.world in the Tools/sitl_gazebo/worlds with [this](link to empty.world)
- Add the landing_station model in Tools/sitl_gazebo/models {these are repsonsible for the landing markers}
- replace [these](link to launch folder) launch files with the ones already present in launch folder in PX4 git clone
- make a catkin workspace and add ayush package to it. This contains all code and nodes that need to be run for simulation
-------------

---
	roslaunch px4 multi_uav_macros_sitl.launch
	roslaunch ayush best.launch 
---
This will launch the empty world contatining nodes and a basic implementation of the algorithm


