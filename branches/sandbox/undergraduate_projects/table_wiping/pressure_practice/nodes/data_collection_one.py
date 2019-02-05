#!/usr/bin/env python
import roslib;roslib.load_manifest('pressure_practice')
import rospy
from arm_control import *
from pressure_listener import *
from sensor_msgs.msg import JointState
import sys
import time

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
    arm.add_trajectory_point(0.73, .2, .2,0,0,0,1,  
                             1000, 1000, 1000, 30, 30, 30,
                             False, False, False, False, False, False, 3)
    arm.execute()
    f = open('log.txt', 'w')
    f.write("time\ty\tz\tpressure")
    start = rospy.get_time()
    print start
    print rospy.get_time()
    y_vals = [-.2 + (i/25.0) for i in range(11)]
    print y_vals
    z_vals = [-.2 + (i/25.0) for i in range(11)]
    print z_vals
    a = raw_input("in position?")
    p = pressure_listener()
    for y in y_vals:
        for z in z_vals:
            print "y:\t" + str(y)
            print "z:\t" + str(z)
            arm.add_trajectory_point(0.73, y, z,0,0,0,1,  
                             1000, 1000, 1000, 30, 30, 30,
                             False, False, False, False, False, False, 3)
            arm.add_trajectory_point(0.73, y, z,0,0,0,1,  
                             7, 1000, 1000, 30, 30, 30,
                             True, False, False, False, False, False, 3)
            arm.execute()
            for i in range(100):
                s = str(rospy.get_time()) + " " + str(y) + " " + str(z) + " " + str(p.avg_pressure) + '\n'
                print s
                f.write(s)
                time.sleep(.01)
    f.close()
