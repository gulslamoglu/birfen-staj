#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import radians

class ShapeDrawer:
    def __init__(self):
        rospy.init_node('shape_drawer', anonymous=True)
        self.rate = rospy.Rate(1)  # Çizim hızı
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.current_pose = Pose()

    def pose_callback(self, data):
        self.current_pose = data

    def move_forward(self, distance):
        twist = Twist()
        twist.linear.x = distance
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(distance / 1.0)

    def rotate(self, angle):
        twist = Twist()
        twist.angular.z = radians(angle)
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(abs(radians(angle) / 1.0))

    def draw_square(self):
        for _ in range(4):
            self.move_forward(2.0)
            self.rotate(90)

    def draw_circle(self):
        for _ in range(36):
            self.move_forward(0.1)
            self.rotate(10)

    def draw_triangle(self):
        for _ in range(3):
            self.move_forward(2.0)
            self.rotate(120)

    def run(self):
        while not rospy.is_shutdown():
            print("Çizmek istediğiniz şekli seçin:")
            print("1. Kare")
            print("2. Yuvarlak")
            print("3. Üçgen")
            print("4. Çıkış")

            choice = input("Seçiminizi yapın (1/2/3/4): ")

            if choice == "1":
                self.draw_square()
            elif choice == "2":
                self.draw_circle()
            elif choice == "3":
                self.draw_triangle()
            elif choice == "4":
                break
            else:
                print("Geçersiz seçim! Lütfen tekrar deneyin.")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        drawer = ShapeDrawer()
        drawer.run()
    except rospy.ROSInterruptException:
        pass
