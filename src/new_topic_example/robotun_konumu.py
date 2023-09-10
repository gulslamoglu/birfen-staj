#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
import math

# Robotun konumu
robot_konumu = None

def tf_callback(msg):
    global robot_konumu
    robot_konumu = msg.transform

def main():
    rospy.init_node('robot_konumunu_al', anonymous=True)

    # TF topic'ini dinlemek için bir tf listener oluşturun
    listener = tf.TransformListener()

    # Robotun konumunu almak için doğru frame'leri ayarlayın
    target_frame = "robot_base_frame"  # Hedef robotun frame adı
    source_frame = "world"  # Dünya koordinat sistemi frame'i

    rate = rospy.Rate(10)  # Döngü hızı (örneğin 10 Hz)

    while not rospy.is_shutdown():
        try:
            # TF'den robotun konumunu alın
            (trans, rot) = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            robot_konumu = trans
            rospy.loginfo("Robotun konumu: %s" % robot_konumu)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF bilgileri alınamadı.")

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
