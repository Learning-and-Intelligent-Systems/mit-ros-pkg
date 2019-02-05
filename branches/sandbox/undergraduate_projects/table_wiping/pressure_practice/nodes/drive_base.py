#!/usr/bin/env python
import roslib; roslib.load_manifest('pressure_practice')
import rospy
import math
from drive_base import drive_base_client
from drive_base import msg

class drive_base:
    def __init__(self):
	self.base_control = drive_base_client.DriveBaseClient()
	self.moveClient = actionlib.SimpleActionClient("move_base_action", msg.moveBaseAction)
	self.turnClient = actionlib.SimpleActionClient("turn_base_action", msg.moveBaseAction)
	rospy.init_node('base_control', anonymous=True)
    def move(self,x,y,vel):
	self.base_control.Move(x,y,vel)
    def turn(self,theta,clockwise):
	self.base_control.Turn(theta,clockwise)
    def cancel(self):
        self.moveClient.cancel_all_goals()
	self.turnClient.cancel_all_goals()
