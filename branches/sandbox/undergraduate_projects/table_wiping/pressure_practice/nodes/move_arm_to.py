#!/usr/bin/env python
import roslib;roslib.load_manifest('pressure_practice')
import rospy
from arm_control import *
from pressure_listener import *
from sensor_msgs.msg import JointState
import sys
import time
import random

left_arm_min = 29
left_arm_max = 36

right_arm_min = 17
right_arm_max = 24

def callback(data):
    pass

def collect_data():
    f.close()


if __name__ == "__main__":
    rospy.init_node('arm_listener')
    joint_listener = threading.Thread(target = callback)
    joint_listener.start()
    rospy.Subscriber("/joint_states", JointState, callback)
    print "subscribed"
    arm = arm_control()
    print "moving arm to:\t(" + str(sys.argv[1]) + ", " + str(sys.argv[2]) + ", " + str(sys.argv[3]) + ")"
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])
    arm.add_trajectory_point(x, y, z,0,0,0,1,  
                             1000, 1000, 1000, 30, 30, 30,
                             False, False, False, False, False, False, 5)
    arm.execute()
    
