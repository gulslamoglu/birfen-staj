#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node("cmd_vel_publisher_node", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        cmd = Twist()
        cmd.linear.x = 0.2  # Örnek: İleri yönde 0.2 m/s hız
        cmd.angular.z = 0.0  # Dönme yok
        pub.publish(cmd)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
