#!/usr/bin/env python
import roslib
#load in all the paths to the packages listed in 'manifest.xml'
roslib.load_manifest('object_survey')

#basic ros python commands:
import rospy
import tf
#import message types:
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Transform
from sensor_msgs.msg import PointCloud
from laser_assembler.srv import AssembleScans
from furniture_ops.srv import AlignClouds

import random
from math import  *
from numpy import  *
import time

#this command registers this process with the ros master
rospy.init_node('odometry_tester')

listener = tf.TransformListener()
getscans = rospy.ServiceProxy('/assemble_scans', AssembleScans)
alignscans = rospy.ServiceProxy('/align_clouds', AlignClouds)
scanclouds=[]     #    time,      integral                 tf            alignment
position_estimates=[ [0.0,   array([0.0,0.0,0.0]),array([0.0,0.0,0.0]),array([0.0,0.0,0.0])]  ]


integral_estimate=array([0.0,0.0,0.0])
tf_estimate=array([0.0,0.0,0.0])
alignment_estimate=array([0.0,0.0,0.0])

def toTransform(pose):
    T=Transform()
    rot = tf.transformations.quaternion_from_euler(0.0,0.0,pose[2])
    T.rotation.x=rot[0]
    T.rotation.y=rot[1]
    T.rotation.z=rot[2]
    T.rotation.w=rot[3]
    T.translation.x=pose[0]
    T.translation.y=pose[1]
    T.translation.z=0.0
    return T
    

def yawFromQuaternion(q):
  return atan2(2*q.y*q.w-2*q.x*q.z , 1 - 2*(q.y*q.y) - 2*(q.z*q.z))

def yawFromQarray(q):
  return atan2(2*q[1]*q[3]-2*q[0]*q[2] , 1 - 2*(q[1]*q[1]) - 2*(q[2]*q[2]))
     
def fromTransform(trans):    
    return array(trans.translation.x,trans.translation.y,yawFromQuaternion(trans.rot))
 
  
def getRobotPosition():
    listener.waitForTransform('/base_link','/odom_combined', rospy.Time(), rospy.Duration(.20))
    t=listener.lookupTransform('/odom_combined','/base_link', rospy.Time())#Note that a time of zero means latest common time
    print t[1]
    print array(t[1])
    return array([t[0][0],t[0][1],yawFromQarray(array(t[1]))])

def moveBase(vel):
    global integral_estimate, current_estimate, position_estimates, tf_estimate, alignment_estimate
    initial_estimate=integral_estimate
    t0=time.time()
    cmd = Twist()
    cmd.linear.y=vel[0]
    cmd.linear.x=vel[1]
    cmd.angular.z = vel[2]
    while(time.time()-t0 <5.0):
        integral_estimate=initial_estimate+vel*(time.time()-t0)
        tf_estimate=getRobotPosition()
        cmd_pub.publish(cmd)
        print time.time(),integral_estimate[0],integral_estimate[1],integral_estimate[2],tf_estimate[0],tf_estimate[1],tf_estimate[2]
        time.sleep(.03)
    cmd2 = Twist()
    #cmd_pub.publish(cmd2)
    time.sleep(.5)
    tf_estimate=getRobotPosition()
    current_estimate=[time.time(),integral_estimate,tf_estimate,alignment_estimate]
    position_estimates.append(current_estimate)
    
def printPoses():
    print position_estimates[-2][0],"   "
    print position_estimates[-2][1],"   "
    print position_estimates[-2][2],"   "
    print position_estimates[-2][3]
        

def getScan():
    t0=rospy.Time.now()
    #use this time to align other scans:
    if len(scanclouds)>1:
        transdiff=toTransform(tf_estimate[-2]-tf_estimate[-3])
        transout=alignscans(scanclouds[-2],scanclouds[-1],transdiff)
        alignment_estimate+=fromTransform(transout)
        position_estimates[-2][2]=alignment_estimate
        printPoses()
    #now wait for scan to complete...
    while (rospy.Time.now()-t0).to_seconds() < 30:
        time.sleep(1)
    scanclouds.append(getscans(rospy.Time.now(),t0))
    



cmd_pub = rospy.Publisher('/base_controller/command', Twist)
integral_estimate=getRobotPosition()
time.sleep(1.0)
vels=array([.0,.40,0.0])
while not rospy.is_shutdown():
    vels*=-1.0
    moveBase(vels)
    getScan()
    


