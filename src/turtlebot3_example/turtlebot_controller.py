#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class TurtleBotController:
    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.rate = rospy.Rate(10)
        self.twist = Twist()

    def scan_callback(self, data):
        # Engelden kaçmak için engel tespiti yapın
        if min(data.ranges) < 0.5:  # Eğer en yakın engel 0.5 metre veya daha yakınsa
            self.twist.linear.x = 0  # Durdurun
            self.twist.angular.z = 1.0  # Saat yönünde 90 derece dönün
        else:
            self.twist.linear.x = 1.0  # Düz git
            self.twist.angular.z = 0

    def move_forward(self, distance):
        # Belirtilen mesafeyi ilerlemek için kullanılır
        initial_x = rospy.get_time()
        while rospy.get_time() - initial_x < distance / 0.2:
            self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep()
        self.twist.linear.x = 0
        self.cmd_vel_pub.publish(self.twist)

    def run(self):
        rospy.loginfo("TurtleBot Controller is running...")
        rospy.sleep(1)  # ROS ile ilgili konfigürasyonun tamamlanması için bir saniye bekle
        self.move_forward(1.0)  # 1 metre git
        while not rospy.is_shutdown():
            self.twist.angular.z = 1.0  # Saat yönünde 90 derece dön
            self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = TurtleBotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
