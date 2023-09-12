#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
from std_srvs.srv import Empty
import tf


def odomPoseCallback(odom_msg):
    
    print ("odom pose callback")
    print ('x = ',odom_msg.pose.pose.position.x)
    print ('y = ',odom_msg.pose.pose.position.y)

    print ('vx = ',odom_msg.twist.twist.linear.x)
    print ('vz = ',odom_msg.twist.twist.angular.z)

    print ('qx = ',odom_msg.pose.pose.orientation.x)
    print ('qy = ',odom_msg.pose.pose.orientation.y)
    print ('qz = ',odom_msg.pose.pose.orientation.z)
    print ('qw = ',odom_msg.pose.pose.orientation.w)

    quaternion ={
        odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z,
        odom_msg.pose.pose.orientation.w,
    }

    rpy = tf.transformations.euler_from_quaternion(quaternion)

    roll= rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]

    print (math.degrees(roll), ' ', math.degrees(pitch), ' ', math.degrees(yaw))

if __name__ == '__main__':
    try:
        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        position_topic= "/odom"
        pose_subscriber = rospy.Subscriber(position_topic,Odometry,odomPoseCallback)

        rospy.spin()
    
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")