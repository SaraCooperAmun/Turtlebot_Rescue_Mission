# Turtlebot_Rescue_Mission

The aim of the project is to design a system that can build a map of the environment, navigate autonomously and recognise markers along the way, especially with the aim of recognising alive/dead people. 

The codes used for simulation in gazebo and real turtlebot are collected in this repository. The integration of the components has been in two ways: by autonomously navigating the robot and recognise markers in a pre-built map; and by building the map of the environment and marker recognition at the same time. 

## Autonomous navigation and object detection in a pre-built map

The first step in this demo is to build a map of the environment, and then to launch the navigation launch file and ROS node. 

### Simulation


3 mapping techniques have been tested. Whichever is chosen, finally the map is saved by roscd-ing to obstacle_avoidance/maps and doing,

$ rosrun map_server map_saver -f simulation_map

#####   Gmapping (with Hokuyo) with teleoperation

First ensure the Hokuyo laser is added to the turtlebot description and is working on simulation, as indicated in https://bharat-robotics.github.io/blog/adding-hokuyo-laser-to-turtlebot-in-gazebo-for-simulation/ , and edit the .bashrc to set the 3d sensor to "Hokuyo".

export TURTLEBOT_3D_SENSOR="hokuyo"

Then launch the gazebo world file by specifying the correct path.

$ roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/pathto/obstacle_avoidance/worlds/rss1.world

Launch the gmapping SLAM package by specifying the topic the laser publishes in (in simulation Hokuyo publishes to /laserscan and Kinect publishes to /scan). In case of doubt rostopic list. Also open Rviz from a separate command prompt. 

$ rosrun gmapping slam_gmapping scan:=/laserscan

$ roslaunch turtlebot_rviz_launchers view_navigation.launch

Teleoperate the robot around the environment with the following command (separate window) and observe the map being built. 

$ roslaunch turtlebot_teleop keyboard_teleop.launch

##### Frontier exploration


Follow instructions of https://github.com/JohanOsinga/RosTurtle/wiki/Turtlebot-autonomous-mapping

Modifications have been that so that it is sufficient to launch the following, which also launches the simulation world. 

$ roslaunch turtlebot_samples exploration_gazebo.launch

On rviz, select with “Point” a polygon and a goal location within the polygon. A quicker way  is to directly set the goal locations using 2D Nav goal

##### Autonomous mapping with PID controller

Follow steps indicated in  https://github.com/bnurbekov/Turtlebot_Navigation by changing the world of the launch file. In the event that the robot does not cover the whole mapped area (as is common), teleop the robot. 


#####  Autonomous navigation in pre-built map and object detection

It serves to localize the robot in the saved map by using the AMCL package and autonomously moving it to multiple goal locations using move_base, while reading data from find_object_2d to place markers on objects. 


Edit navigation_simulation.launch file to specify map. 
<arg name="map_file" default="$(find obstacle_avoidance)/maps/simulation_map.yaml"/>

Launch the following in separate command prompts:

$ roslaunch obstacle_avoidance navigation_simulation.launch

$ roslaunch rtabmap_ros kinect_simulation.launch save_objects:=true


Edit in navigation_simulation.py the ids that find_object_2d gives to each label, as it is different every time it is launched. For example, set 542 for “Radiactive”, etc. If desired, change the goal coordinates and/or orientation.

roscd to obstacle_avoidance/src and on a separate terminal run:

$ python navigation_simulation.py



### Turtlebot

On the real turtlebot the map is generated using gmapping (Hokuyo) and teleoperation, and then the robot is localized and moves autonomously to the set coordinate locations (it does not integrate object detection).

#####  Gmapping (hokuyo) + teleop

On turtlebot:

$ roslaunch turtlebot_bringup minimal_with_hokuyo.launch

$ rosrun hokuyo_node hokuyo_node


On PC:

To build the map of the environment, gmapping package was used by launching the following command on the turtlebot laptop as follows:
$ rosrun gmapping slam_gmapping scan:=/scan

After this, the command to move the robot around using keyboard was launched on the remote computer as follows:

$ roslaunch turtlebot_teleop keyboard_teleop.launch 
The visualisation of the mapping process was viewed by implementing on another terminal shell of the remote computer as follows:

$ roslaunch turtlebot_rviz_launchers view_navigation.launch

On the Rviz platform, the robot model edited by selecting the odom in the drop down lists as against map that was initially reflected.
The navigation of the turtlebot was then achieved by using the keyboard to move it around having switch over to the terminal shell where the ‘roslaunch turtlebot_teleop keyboard_teleop.launch’ command  was implemented.

The robot was moved around the environment and the built map was saved by roscd-ing to obstacle_avoidance/maps and with

$ rosrun map_server map_saver -f ggo.   The  map was saved both in pgm  and yaml  formats.   


##### Autonomous navigation in pre-built map

First edit navigation_robot.launch to update map.
<arg name="map_file" default="$(find obstacle_avoidance)/maps/ggo.yaml"/>

On turtlebot

$ roslaunch turtlebot_bringup minimal_with_hokuyo.launch

$  rosrun hokuyo_node hokuyo_node

(or roslaunch turtlebot_bringup minimal.launch if it is preferred to use the Kinect)

On pc

$ roslaunch obstacle_avoidance navigation_robot.launch

Roscd to obstacle_avoidance/src. If desired edit the goal coordinates/orientation. Should it be possible to launch find_object_2d uncomment the respective code. The robot will consecutively go to the specified locations while avoiding obstacles. 

$ python navigation_robot.py


 




