#!/usr/bin/env python
import roslib; roslib.load_manifest('drive_base')

import sys

import rospy
from drive_base.srv import *

def move_base_client(x,y,vel):
    rospy.wait_for_service('move_base')
    try:
	move_base = rospy.ServiceProxy('move_base',moveBase)
	resp1 = move_base(x,y,vel)
	return resp1.success
    except rospy.ServiceException, e:
	print "Service call failed: %s"%e

def turn_base_client(theta,clockwise):
    rospy.wait_for_service('turn_base')
    try:
	turn_base = rospy.ServiceProxy('turn_base',turnBase)
	resp1 = turn_base(theta,clockwise)
	return resp1.success
    except rospy.ServiceException, e:
	print "Service call failed: %s"%e

if __name__ == "__main__":
    move_base_client(0.5,0.5,0.25)
    turn_base_client(1.5,True)
