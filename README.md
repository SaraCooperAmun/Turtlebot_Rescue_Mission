# Turtlebot_Rescue_Mission

Project done by AUDRY Hanako, COOPER Sara, OLADIPUPO Gideon

The aim of the project is to design a system that can build a map of the environment, navigate autonomously and recognise markers along the way, especially with the aim of recognising alive/dead people. 

The codes used for simulation in gazebo and real turtlebot are collected in this repository. The integration of the components has been in two ways: by autonomously navigating the robot and recognise markers in a pre-built map; and by building the map of the environment and marker recognition at the same time. 

Links to videos are also attached. To download a video, simply follow the link, right-click and "Save video as". 
## Autonomous navigation and object detection in a pre-built map

The first step in this demo is to build a map of the environment, and then to launch the navigation launch file and ROS node. 

### Simulation


3 mapping techniques have been tested. Whichever is chosen, finally the map is saved by roscd-ing to obstacle_avoidance/maps and doing,
```bash
$ rosrun map_server map_saver -f simulation_map
```
#####   Gmapping (with Hokuyo) with teleoperation

First ensure the Hokuyo laser is added to the turtlebot description and is working on simulation, as indicated in https://bharat-robotics.github.io/blog/adding-hokuyo-laser-to-turtlebot-in-gazebo-for-simulation/ , and edit the .bashrc to set the 3d sensor to "Hokuyo".
```bash
export TURTLEBOT_3D_SENSOR="hokuyo"
```
To enable gmapping, make sure there is a copy of "hokuyo_gmapping.launch.xml" under turtlebot_navigation/launch/includes.

Then launch the gazebo world file by specifying the correct path.
```bash
$ roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/pathto/obstacle_avoidance/worlds/rss1.world
```
Launch the gmapping SLAM package by specifying the topic the laser publishes in (in simulation Hokuyo publishes to /laserscan and Kinect publishes to /scan). In case of doubt rostopic list. Also open Rviz from a separate command prompt. 
```bash
$ rosrun gmapping slam_gmapping scan:=/laserscan

$ roslaunch turtlebot_rviz_launchers view_navigation.launch
```
The robot was moved around the environment.
```bash
$ roslaunch turtlebot_teleop keyboard_teleop.launch
```
##### Frontier exploration

Install "Frontier Exploration" package: https://github.com/paulbovbel/frontier_exploration

Install "turtlebot_samples" package: https://github.com/130s/turtlebot_samples

Change world file from "exploration_gazebo.launch" file to 
```bash
<arg name="world_file" default="pathto/obstacle_avoidance/worlds/rss1.world"/>

$ roslaunch turtlebot_samples exploration_gazebo.launch
```

On rviz, select with “Point” a polygon and a goal location within the polygon. A quicker way  is to directly set the goal locations using 2D Nav goal.

See video of how it works here: https://streamable.com/mqdmx

##### Autonomous mapping with PID controller

Load the world:
```bash
$ roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(find obstacle_avoidance)/worlds/rss1.world
```
And follow steps indicated in  https://github.com/bnurbekov/Turtlebot_Navigation
```bash
$ rosrun rviz rviz'
$ roslaunch final_project final_project.launch
$ rosrun final_project mapping.py
$ rosrun final_project control.py
```

In the event that the robot does not cover the whole mapped area (as is common), teleop the robot. 
See video of how it works here: https://streamable.com/t068h



#####  Autonomous navigation in pre-built map and object detection

It serves to localize the robot in the saved map by using the AMCL package and autonomously moving it to multiple goal locations using move_base, while reading data from find_object_2d to place markers on objects. 
The code is loosely based on https://github.com/markwsilliman/turtlebot/blob/master/go_to_specific_point_on_map.py which includes the script to send one goal to the move_base.

In order to place the markers the "visualization_marker_tutorial" must be installed from https://github.com/ros-visualization/visualization_tutorials

Edit navigation_simulation.launch file to specify map. 
```bash
<arg name="map_file" default="$(find obstacle_avoidance)/maps/simulation_map.yaml"/>
```
Launch the following in separate command prompts (refer to next section for details on how to include find_object_2d):
```bash
$ roslaunch obstacle_avoidance navigation_simulation.launch

$ roslaunch rtabmap_ros kinect_simulation.launch save_objects:=true
```

Edit in navigation_simulation.py the ids that find_object_2d gives to each label, as it is different every time it is launched. For example, set 542 for “Radiactive”, etc. If desired, change the goal coordinates and/or orientation.

roscd to obstacle_avoidance/src and on a separate terminal run:
```bash
$ python navigation_simulation.py
```
See video of how it works here: https://youtu.be/lfxeDlCYsO0


### Turtlebot

On the real turtlebot the map is generated using gmapping (Hokuyo) and teleoperation, and then the robot is localized and moves autonomously to the set coordinate locations (it does not integrate object detection).


#####  Gmapping (hokuyo) + teleop

On turtlebot:

Edit the .bashrc to set the 3d sensor to "Hokuyo".
```bash
export TURTLEBOT_3D_SENSOR="hokuyo"

$ roslaunch turtlebot_bringup minimal_with_hokuyo.launch

$ rosrun hokuyo_node hokuyo_node
```

On PC:

Edit the .bashrc to set the 3d sensor to "Hokuyo".
```bash
export TURTLEBOT_3D_SENSOR="hokuyo"
```
Make sure there is a copy of "hokuyo_gmapping.launch.xml" under turtlebot_navigation/launch/includes.

To build the map of the environment, gmapping package was used by launching the following command on the turtlebot laptop as follows:
```bash
$ rosrun gmapping slam_gmapping scan:=/scan
```
After this, the command to move the robot around using keyboard was launched on the remote computer as follows:
```bash
$ roslaunch turtlebot_teleop keyboard_teleop.launch 
```
The visualisation of the mapping process was viewed by implementing on another terminal shell of the remote computer as follows:
```bash
$ roslaunch turtlebot_rviz_launchers view_navigation.launch
```
On the Rviz platform, the robot model edited by selecting the odom in the drop down lists as against map that was initially reflected.
The navigation of the turtlebot was then achieved by using the keyboard to move it around having switch over to the terminal shell where the ‘roslaunch turtlebot_teleop keyboard_teleop.launch’ command  was implemented.

The robot was moved around the environment and the built map was saved by roscd-ing to obstacle_avoidance/maps and with
```bash
$ rosrun map_server map_saver -f ggo.   The  map was saved both in pgm  and yaml  formats.   
```


##### Autonomous navigation in pre-built map

First edit navigation_robot.launch to update map.
```bash
<arg name="map_file" default="$(find obstacle_avoidance)/maps/ggo.yaml"/>
```
On turtlebot
```bash
$ roslaunch turtlebot_bringup minimal_with_hokuyo.launch

$  rosrun hokuyo_node hokuyo_node
```
(or roslaunch turtlebot_bringup minimal.launch if it is preferred to use the Kinect)

On pc
```bash
$ roslaunch obstacle_avoidance navigation_robot.launch
```
Roscd to obstacle_avoidance/src. If desired edit the goal coordinates/orientation. Should it be possible to launch find_object_2d uncomment the respective code. The robot will consecutively go to the specified locations while avoiding obstacles. 
```bash
$ python navigation_robot.py
```
See video of how it works here: https://youtu.be/UoQJvCftzLU

### RTAB-Map and find_object_2d

#####  Install RTAB-map

how install here: https://github.com/introlab/rtabmap_ros

#####  Install find_object_2d package

how install here: https://github.com/introlab/find-object

#####  Launch RTAB-map and find_object together 

Modify inside the CameraROS.cpp file  (find-object/src/ros/CameraROS.cpp) line 49 the subscriber if you can't have the camera of the turtlebot properly subscribed 
```bash
$ imageSub_ = it.subscribe(nh.resolveName("/name of the topic with the camera"), 1, &CameraROS::imgReceivedCallback, this);
```
if it is in simulation line 66 - 68
```bash
$rgbSub_.subscribe(rgb_it, rgb_nh.resolveName("/topic with the color image "), 1, hintsRgb);
$depthSub_.subscribe(depth_it, depth_nh.resolveName("/topic with the depth"), 1, hintsDepth);
$cameraInfoSub_.subscribe(depth_nh, "/topic with the camera_info", 1);
```
Add the file kinect_simulation.launch inside rtabmap_ros's package launch folder.
The launch file kinnect_simulation is a launch file the group created to have RTAB-map and find-object-2d package launched togethe.r
```bash
$ roslaunch rtabmap_ros kinect_simulation.launch save_objects:=true 
```
if you want to use it in simulation set the simulation to true : 
```bash
$ roslaunch rtabmap_ros kinect_simulation.launch save_objects:=true  simulation:=true
```





