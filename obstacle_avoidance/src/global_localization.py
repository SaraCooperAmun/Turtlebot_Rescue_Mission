#! /usr/bin/env python

#Attempt at using global_localization so that it is not necessary to use Rviz to set the initial 2D pose. Only tested on simulation and sometimes robot fails to localize itself. 
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
cov = [10, 10, 10]
def callback(msg):
    global cov
    global position
    position = msg.pose.pose
    cov = msg.pose.covariance


rospy.init_node('check_odometry')
odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback)
cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
move_cmd = Twist()
move_cmd.linear.x = 0
rospy.wait_for_service('/global_localization')
try:
        globalLoc = rospy.ServiceProxy('/global_localization', Empty) # this is the type of service
        result=globalLoc()

        rospy.sleep(3)
except rospy.ServiceException, e:
    print "Service call failed: %s"%e

while not rospy.is_shutdown(): 
       
        if (max(cov) > 0.01): # specify how big we accept the covariance until it has reduced we keep turning
                move_cmd.angular.z = 1
		
        else:
             move_cmd.angular.z = 0.0
      
        cmd_vel.publish(move_cmd)
        rospy.sleep(1)
	
