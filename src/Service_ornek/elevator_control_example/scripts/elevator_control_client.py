#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from elevator_control_example.srv import ElevatorControl

def elevator_control_client(floor):
    rospy.wait_for_service('elevator_control')
    try:
        elevator_control = rospy.ServiceProxy('elevator_control', ElevatorControl)
        response = elevator_control(floor)
        return response.success, response.message
    except rospy.ServiceException as e:
        print("Hizmet çağrısı başarısız: %s" % e)

if __name__ == "__main__":
    try:
        floor = int(input("Asansörü hangi yükseklikte kaldırmak istersiniz? "))
        success, message = elevator_control_client(floor)
        print(message)
    except ValueError:
        print("Geçersiz yükseklik değeri.")
