#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from elevator_control_example.msg import ElevatorControlAction, ElevatorControlResult, ElevatorControlFeedback

class ElevatorControlServer(object):
    def __init__(self):
        self.server = actionlib.SimpleActionServer('elevator_control', ElevatorControlAction, self.execute, False)
        self.server.start()
    
    def execute(self, goal):
        # Hedef yükseklik alınır
        target_floor = goal.floor

        # Asansörü hedef yüksekliğe taşıma simülasyonu yapılır (örneğin, zaman geçişi)
        success = self.move_elevator(target_floor)

        if success:
            self.server.set_succeeded(ElevatorControlResult(success=True, message="Asansör başarıyla kaldırıldı - indirildi."))
        else:
            self.server.set_aborted(ElevatorControlResult(success=False, message="Asansör hedef yüksekliği kaldıramadı."))

    def move_elevator(self, target_floor):
        # Bu işlev asansörün hedef yüksekliğe taşınmasını simüle eder.
        # Simülasyon başarılıysa True, başarısızsa False döndürür.
        # Gerçek asansör kontrol kodu buraya gelebilir.
        # Simülasyon olarak her zaman başarılı kabul ediyoruz.
        return True

if __name__ == "__main__":
    rospy.init_node('elevator_control_server')
    elevator_server = ElevatorControlServer()
    rospy.spin()
