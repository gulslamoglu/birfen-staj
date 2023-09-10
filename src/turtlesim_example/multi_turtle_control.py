#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

def move_turtle(turtle_name, linear_speed, angular_speed):
    pub = rospy.Publisher('/' + turtle_name + '/cmd_vel', Twist, queue_size=10)
    rospy.init_node('multi_turtle_control', anonymous=True)
    rate = rospy.Rate(10)  # Hz

    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        # İlk turtle için kontrol düğümünü başlatın
        move_turtle("turtle1", 1.0, 0.0)  # Örnek: İlk turtle ileri yönde hareket eder
        # İkinci turtle için kontrol düğümünü başlatın
        move_turtle("turtle2", -1.0, 0.0)  # Örnek: İkinci turtle geri yönde hareket eder
    except rospy.ROSInterruptException:
        pass
