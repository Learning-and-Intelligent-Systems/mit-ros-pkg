#!/usr/bin/env python
import roslib;roslib.load_manifest('pressure_practice')
import rospy
from arm_control import *
from pressure_listener import *
from sensor_msgs.msg import JointState
import sys
import time
import random
import math

left_arm_min = 29
left_arm_max = 36

right_arm_min = 17
right_arm_max = 24

def callback(data):
    pass

def collect_data():
    f.close()

def normalize(a):
    s = 0
    for j in a:
        s += j**2
    s = math.sqrt(s)
    b = []
    for i in a:
        b.append(i/s)
    return b


if __name__ == "__main__":
    rospy.init_node('arm_listener')
    joint_listener = threading.Thread(target = callback)
    joint_listener.start()
    rospy.Subscriber("/joint_states", JointState, callback)
    print "subscribed"
    arm = arm_control()
    for i in range(100):
        x = random.uniform(.35, 1)
        y = random.random()
        z = random.uniform(-.5, .5)
        a = [random.uniform(-1, 1) for i in range(4)]
        a = normalize(a)
        print a
        print sum(a)
        arm.add_trajectory_point(x, y, z, a[0], a[1], a[2], a[3],  
                             1000, 1000, 1000, 30, 30, 30,
                             False, False, False, False, False, False, 5)
        arm.execute()
    
