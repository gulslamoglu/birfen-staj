#!/usr/bin/env python3  

import rospy

if __name__ == '__main__':

    # ROS düğümünü başlat ve node ismini ver
    rospy.init_node("test_node")

    #terminalde yazdırılacak
    rospy.loginfo("Test node has been started")
    
    rate= rospy.Rate(10) #saniyede 10 kere yazdırmak için

    while not rospy.is_shutdown():
        rospy.loginfo("hello")
        rate.sleep() #saniyede 10 kere yazdıracak şekilde sen sonlandırana kadar döngü devam eder
