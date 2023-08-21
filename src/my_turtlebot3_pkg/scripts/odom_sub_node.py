#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def odom_callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rospy.loginfo("Robot Position - X: %f, Y: %f", x, y)

def main():
    rospy.init_node("odom_subscriber_node")
    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
