#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from elevator_control_example.srv import ElevatorControl, ElevatorControlResponse

def handle_elevator_control(request):
    response = ElevatorControlResponse()

    if request.floor > 5:
        response.success = False
        response.message = "Asansör 5'ten daha büyük bir yüksekliği kaldıramaz."
    else:
        response.success = True
        response.message = "Asansör kaldırıldı - indirildi."

    return response

def elevator_control_server():
    rospy.init_node('elevator_control_server')
    service = rospy.Service('elevator_control', ElevatorControl, handle_elevator_control)
    rospy.spin()

if __name__ == "__main__":
    elevator_control_server()
