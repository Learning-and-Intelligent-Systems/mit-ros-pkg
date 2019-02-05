#!/usr/bin/env python
import roslib; roslib.load_manifest('bakebot')
import rospy
from sensor_msgs.msg import JointState
from datetime import datetime, date, time

first_time = True
myfile = None

def callback(data):
    global first_time
    global myfile
    names = data.name[16:23]
    efforts = data.effort[16:23]
    if first_time:
        print 'writing first time'
        myfile.write(str(names)+'\n')
        first_time = False
    print efforts
    myfile.write(str(data.header.stamp.secs) +', '+str(efforts)+'\n')

def listener():
    rospy.init_node('listener', anonymous=True)
    filename = raw_input('enter filename: ')
    global myfile
    myfile = open(filename, 'a+')
    t = datetime.now()
    myfile.write('\n************************* ' + t.isoformat() + '\n')
    rospy.Subscriber("joint_states", JointState, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

