#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose

def pose_callback(msg):
    rospy.loginfo(msg)

#def pose_callback(msg: Pose)
#   rospy.loginfo( "(" + str(msg.x) + "," + str(msg.y) + ")" )
# bu değişiklik ile pose_callback fonksiyonu, Pose mesajının içindeki x ve y değerlerini alarak bunları bir pozisyon koordinatı olarak ekrana basar. Bu şekilde x ve y değerlerini daha anlamlı bir şekilde görebilirsiniz.


if __name__ == '__main__':
    rospy.init_node('turtle_pose_subscriber')

    sub = rospy.Subscriber('/turtle1/pose', Pose, callback=pose_callback)

    rospy.loginfo("Node has been started")

    rospy.spin()