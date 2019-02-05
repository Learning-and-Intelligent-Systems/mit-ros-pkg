#! /usr/bin/env python

import roslib; roslib.load_manifest('drive_base')
import math
import rospy
import actionlib
import drive_base.msg

class DriveBaseClient():
    def __init__(self):
	self.moveClient = actionlib.SimpleActionClient("move_base_action", drive_base.msg.moveBaseAction)
	self.turnClient = actionlib.SimpleActionClient("turn_base_action", drive_base.msg.turnBaseAction)
        print "waiting for servers"
	self.moveClient.wait_for_server()
	self.turnClient.wait_for_server()
        print "done wait"

    def move(self, x, y, vel):
	goal = drive_base.msg.moveBaseAction
	goal.goal_x = x
	goal.goal_y = y
	goal.velocity = vel
	self.moveClient.send_goal(goal)
	self.moveClient.wait_for_result()
	return self.moveClient.get_result()

    def turn(self, theta, clockwise):
	goal = drive_base.msg.turnBaseAction
	goal.goal_theta = theta
	goal.clockwise = clockwise
	self.turnClient.send_goal(goal)
	self.turnClient.wait_for_result()
	return self.turnClient.get_result()


if __name__ == '__main__':
    rospy.init_node('base_driver')
    Driver = DriveBaseClient()
