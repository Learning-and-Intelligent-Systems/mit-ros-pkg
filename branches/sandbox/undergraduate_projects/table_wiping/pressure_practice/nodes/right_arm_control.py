#!/usr/bin/env python
import roslib; roslib.load_manifest('pressure_practice')
import rospy
from ee_cart_imped_action import *
from ee_cart_imped_control import *
below_table = .10
table_min = .35
class r_arm_control:
    def __init__(self, table_height, table_x_min = table_min):
        self.control = ee_cart_imped_client(False)
        self.table_height = table_height
        self.x_min = table_x_min
        self.time = 0
    def move_to(self, y):
        self.control.addTrajectoryPoint(self.x_min, y-.10, self.table_height - below_table,
                                          0.55689944425028592, -0.46464446219057154, -0.48170412462790518, -0.49186346587830748,
                                          1000,1000,1000,30,30,30,
                                          False, False, False, False, False, False, 2)
        self.control.sendGoal(False)
        self.control.resetGoal()
