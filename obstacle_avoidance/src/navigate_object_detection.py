#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# This script works on Gazebo only. Prior to running this node run roslaunch rescue_robot navigation_simulation.launch and roslaunch rtabmap_ros kinect_simulation.launch

import rospy
import random
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_srvs.srv import Empty
from std_msgs.msg import Float32MultiArray
import tf
cov = [10, 10, 10]
pose_x = 0
pose_y = 0
laserData = []
humanCoordinate = []
detected_object = "Unknown"

def callbackPose(msg): #function which returns topic /amcl_pose pose and covariance
    global cov
    global pose_x
    global pose_y
    pose_x = msg.pose.pose.position.x
    pose_y = msg.pose.pose.position.y
    cov = msg.pose.covariance


def callbackLaser(msg):
    global laserData
    laserData = msg.ranges[350:450] # reads laser data only on the front side of the 

def detectObject(msg):  #determine name of object based on id returned by find_object_2d. These id must be changed whenever find_object_2d. Refer to the label it gives to each object. 
    global detected_object
    object_id = int(msg.data[0])
    if (object_id== 552):
       detected_object = "Alive"
    elif (object_id== 555):
      detected_object = "Radioactive"
    elif (object_id== 551):
      detected_object = "Danger"
    elif (object_id== 550):
     detected_object = "Biohazard"
    elif (object_id== 549):
     detected_object = "Dead"
    elif (object_id== 554):
     detected_object = "No Smoke"
    elif (object_id== 557):
     detected_object = "Toxic"
    elif (object_id== 553):
     detected_object = "Fire"
    else:
     detected_object = "Unknown"


#Class to send one goal to the move_base      
class GoToPose():
    def __init__(self):

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
	rospy.on_shutdown(self.shutdown)
	
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

	

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map' #goal is sent on map frame
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos[0], pos[1], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)



if __name__ == '__main__':
    try:
        rospy.init_node('navigator', anonymous=False)
	position = [[0.8, -0.2], [2, 4.8],[6.7, 1.8], [6, -4.5], [1, -6.5], [-5, -0.4], [-5.6, -6.8], [-9, -0.5], [-5, -5.2]] #specify all goal location coordinates, located on the south side of each object. To determine location select "Publish Point" in Rviz, move over the the desired goal location in the map, and record the coordinates here
	quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000} #assumes the robot should always be facing the object
        cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)#publish velocity
        sub = rospy.Subscriber('/laserscan', LaserScan, callbackLaser)#read Hokuyo laser data. Do rostopic list to make sure which topic the laser is publishing in and change accordingly
	odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callbackPose) #read amcl_pose data which estimates robot localization in the map
        rospy.Subscriber('/rtabmap/objects', Float32MultiArray, detectObject)#receive data of the objects detected with find_object_2d
	publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size = 1)#publish markers
        move_cmd = Twist()
	markerArray = MarkerArray()
        id = 0
	counter = 0
	max_objects = 9 #change depending on the number of objects that want to be analyzed

        move_cmd.linear.x = 0
   
  
        #Once robot initial pose is indicated in Rviz (2D Pose Estimate), keep rotating the robot in initial place until covariance returned by /amcl_pose is below 0.01, so that localization uncertainty is decreased

	while (max(cov) > 0.01): 
                move_cmd.angular.z = 1
                cmd_vel.publish(move_cmd)
                rospy.sleep(1)
		
      
        move_cmd.angular.z = 0.0     
        cmd_vel.publish(move_cmd)
        #For each marker/desired goal we send a new goal to the move_base
        for i in range(max_objects):
			navigator = GoToPose()
			rospy.loginfo("Go to next pose")
			success = navigator.goto(position[i], quaternion)
	 
			if success:
			    rospy.loginfo("Goal reached")
			    while  (min(laserData) > 1.3): #ascertain using laserData that an object is in front of robot. If not rotate robot until it faces it directly. This part is more useful when robot orientation is not specified to be exactly (0,0,0,1) as it may be the case with the real turtlebot
                                   distance = min(laserData)
				  
				   move_cmd.angular.z = 1
				   cmd_vel.publish(move_cmd)
				   rospy.sleep(1)
			  
                              
			    move_cmd.angular.z = 0.0
		            cmd_vel.publish(move_cmd)
                            rospy.sleep(2)
                            #Place text marker on object by taking into account the last id returned by find_object_2d
			    marker = Marker()
			    marker.header.frame_id = "/map"
                            marker.id = i
			    marker.type = marker.TEXT_VIEW_FACING
			    marker.action = marker.ADD
			    marker.scale.x = 0.7
			    marker.scale.y = 0.7
			    marker.scale.z = 0.5

                            marker.color.a = 1.0
		            marker.color.r = 1.0
		            marker.color.g = 0.0
		            marker.color.b = 0.0
			    marker.pose.orientation.w = 1.0
                            #Determine coordinate of the object by taking into account robot pose and laser data (offset of 0.5 to place marker more or less in the center of the box). 
                            distance = pose_x + min(laserData) +  0.5 
			    marker.pose.position.x = distance 
			    marker.pose.position.y = pose_y
			    marker.pose.position.z = 0.2
                            counter = 0
                 
                            #If marker name is "unknown" allow up to 6 seconds to recognise object.
                            
                            if (detected_object == "Unknown"):
		                        while (counter < 3):
		                              if (detected_object != "None"):
		                    		    marker.text= detected_object
                                                    break
                                              else:
                                                    counter += 1
                                                    marker.text= "Unknown"
                                                    rospy.sleep(2)
                            else:
                                marker.text = detected_object
                            print "Detected:", detected_object

			    markerArray.markers.append(marker)

			   # Renumber the marker IDs
			    
			    for m in markerArray.markers:
			       m.id = id
			       id += 1

			   # Publish the MarkerArray
                          
			    publisher.publish(markerArray)
                            #Specify coordinates of humans found
                            if (detected_object == "Alive"):
                                  print "Alive person found at", [distance, pose_y]
                            elif (detected_object == "Dead"):
                                  print "Dead person found at", [distance, pose_y]
                                
                           

			  
		            

			else:
			    rospy.loginfo("The base failed to reach the desired pose")

			# Sleep to give the last log messages time to be sent
	rospy.sleep(5)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
