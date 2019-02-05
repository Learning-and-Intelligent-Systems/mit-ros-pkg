#!/usr/bin/env python
import roslib
#load in all the paths to the packages listed in 'manifest.xml'
roslib.load_manifest('bilicar')

#basic ros python commands:
import rospy

#import message types:
from geometry_msgs.msg import Twist

import time
import serial
import sys
import struct
import binascii
import os
import math
import signal

  

def getser(sport):
    ser = serial.Serial(
          port=sport,
          baudrate=115200,
          timeout=.5,
          parity=serial.PARITY_NONE,
          stopbits=serial.STOPBITS_ONE,
          bytesize=serial.EIGHTBITS
    )
    return ser;

ser=getser("/dev/ttyACM0")
ser.open()
ser.flushInput()

def sendSteering(num):
   out="s"+str(num)+';'
   print "sending ",out
   ser.write(out)
   print ser.read(5)
   
def sendThrottle(num):
   out="t"+str(num)+';'
   print "sending ",out
   ser.write(out)
   print ser.read(5)
  
def processCmd(cmd):
    throttle=int(cmd.linear.x*-50.0)+260
    throttle=min(max(throttle,200),320)
    steering = int(cmd.angular.z*200.0)+300
    steering=min(max(steering,200),400);
    sendSteering(steering)
    sendThrottle(throttle)
    
#time.sleep(10)
cmd=Twist()
processCmd(cmd)
#vels=[0,.5,0,-.5,0]
#for v in vels:
#    cmd.angular.z=v
#    processCmd(cmd)
#    time.sleep(1.0)
 
#this command registers this process with the ros master
rospy.init_node('bilicar_controller')

#register a callback for messages of type LaserScan, on the topic "/base_scan"
sub = rospy.Subscriber('/bilicar/command', Twist, processCmd)

#this line is equivalent to: 
# while(everything is ok)
#   check for messages, call callbacks
#   sleep

rospy.spin()
ser.close()
    
