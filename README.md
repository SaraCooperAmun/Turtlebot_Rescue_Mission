# Turtlebot_Rescue_Mission

SIMULATION

GMAPPING (HOKUYO)  + TELEOP 

roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/pathto rss1.world

rosrun gmapping slam_gmapping scan:=/laserscan

roslaunch turtlebot_rviz_launchers view_navigation.launch

roslaunch turtlebot_teleop keyboard_teleop.launch

FRONTIER EXPLORATION


Follow instructions of

https://github.com/JohanOsinga/RosTurtle/wiki/Turtlebot-autonomous-mapping

Modifications have been that so that it is sufficient to launch the following, which also launches the simulation world. 

roslaunch turtlebot_samples exploration_gazebo.launch

On rviz, select with “Point” a polygon and a goal location within the polygon. A quicker way (that shown on video) is to directly set the goal locations using 2D Nav goal

AUTONOMOUS MAPPING WITH PID CONTROLLER

Follow steps indicated in 

https://github.com/bnurbekov/Turtlebot_Navigation
by changing the world of the launch file


SAVE THE MAP

Roscd to obstacle_avoidance/maps

rosrun map_server map_saver -f map


NAVIGATION + FIND OBJECT DEMO

Edit navigation_simulation.launch file to specify map. 
<arg name="map_file" default="$(find obstacle_avoidance)/maps/simulation_map.yaml"/>



roslaunch obstacle_avoidance navigation_simulation.launch

roslaunch rtabmap_ros kinect_simulation.launch save_objects:=true




Edit in navigation_simulation.py the ids that find_object_2d gives to each label, as it is different every time it is launched. For example, set 542 for “Radiactive”, etc. If desired, change the goal coordinates and/or orientation.

Roscd to obstacle_avoidance/src and

python navigation_simulation.py



REAL TURTLEBOT

GMAPPING (hokuyo) + teleop

On turtlebot

Roslaunch turtlebot_bringup minimal_with_hokuyo.launch

Rosrun hokuyo_node hokuyo_node


On pc

rosrun gmapping slam_gmapping scan:=/scan

roslaunch turtlebot_rviz_launchers view_navigation.launch

roslaunch turtlebot_teleop keyboard_teleop.launch

Roscd to obstacle_avoidance/maps, and save map with

rosrun map_server map_saver -f map


NAVIGATION

First edit navigation_robot.launch to update map.
<arg name="map_file" default="$(find obstacle_avoidance)/maps/world_map.yaml"/>


On turtlebot

Roslaunch turtlebot_bringup minimal_with_hokuyo.launch

Rosrun hokuyo_node hokuyo_node


(or roslaunch turtlebot_bringup minimal.launch if it is preferred to use the Kinect=

On pc


Roslaunch obstacle_avoidance navigation_robot.launch

Roscd to obstacle_avoidance/src. If desired edit the goal coordinates/orientation. Should it be possible to launch find_object_2d uncomment the respective code. 

Python navigation_robot.py


 




