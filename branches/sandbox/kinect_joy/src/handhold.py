#!/usr/bin/env python
import roslib
#load in all the paths to the packages listed in 'manifest.xml'
roslib.load_manifest('simple_controller')

#basic ros python commands:
import rospy

#import message types:
from pr2_controllers_msgs.msg import JointTrajectoryControllerState

from math import  *



#[sensor_msgs/LaserScan]:
#Header header
#  uint32 seq
#  time stamp
#  string frame_id
#float32 angle_min
#float32 angle_max
#float32 angle_increment
#float32 time_increment
#float32 scan_time
#float32 range_min
#float32 range_max
#float32[] ranges
#float32[] intensities

def myStateCallback(state):
    print "%.03f %.03f %.03f %.03f %.03f %.03f"%(state.error.positions[0],state.error.positions[1],state.error.positions[2],
                                                 state.error.positions[3],state.error.positions[4],state.error.positions[5])





#this command registers this process with the ros master
rospy.init_node('my_controller')

#register a publisher, to topic '/base_controller/command', of type Twist
#pub = rospy.Publisher('/base_controller/command', Twist)

#register a callback for messages of type LaserScan, on the topic "/base_scan"
sub = rospy.Subscriber("/l_arm_controller/state", JointTrajectoryControllerState, myStateCallback)

#this line is equivalent to: 
# while(everything is ok)
#   check for messages, call callbacks
#   sleep
rospy.spin()

#in this example, we are controlling the robot at ~40Hz, 
#which is the rate at which we are receiving laser data


