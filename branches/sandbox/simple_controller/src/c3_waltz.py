#!/usr/bin/env python

import roslib
#load in all the paths to the packages listed in 'manifest.xml'
roslib.load_manifest('simple_controller')

#basic ros python commands:
import rospy

#import message types:
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

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
def getangle(scan,index):
    return scan.angle_min+scan.angle_increment*float(index)

#finds location of nearest foot to front of base
def findFeet(scan):
    minx=1.0
    miny=0
    found=False
    #find smallest range in laser scan:
    for i in range(len(scan.ranges)):
        angle=getangle(scan,i)
        y=scan.ranges[i]*sin(angle)
        x=scan.ranges[i]*cos(angle)
        if abs(y) < .5 and x>0.0 and scan.ranges[i] > scan.range_min +.001 and x<minx:
            minx=x
            miny=y
            found=True
    print minx,miny
    return (found,minx,miny)




def myLaserCallback(scan):
    (found,x,y)=findFeet(scan)
    
    cmd = Twist()
    
    if found:
       cmd.linear.x=.8*(x-.5) 
       
    #regardless of whether we set things, publish the command:
    pub.publish(cmd)   




#this command registers this process with the ros master
rospy.init_node('my_controller')

#register a publisher, to topic '/base_controller/command', of type Twist
pub = rospy.Publisher('/base_controller/command', Twist)

#register a callback for messages of type LaserScan, on the topic "/base_scan"
sub = rospy.Subscriber("/base_scan", LaserScan, myLaserCallback)

#this line is equivalent to: 
# while(everything is ok)
#   check for messages, call callbacks
#   sleep
rospy.spin()

#in this example, we are controlling the robot at ~40Hz, 
#which is the rate at which we are receiving laser data


