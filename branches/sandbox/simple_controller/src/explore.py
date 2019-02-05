#!/usr/bin/env python
import roslib
#load in all the paths to the packages listed in 'manifest.xml'
roslib.load_manifest('object_survey')

#basic ros python commands:
import rospy
import tf
import pr2_controllers_msgs.msg
#import message types:
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import random
from math import  *
import time

#this command registers this process with the ros master
rospy.init_node('siftexplorer')

import pr2_controllers_msgs.msg
import actionlib

import geometry_msgs.msg


def pointHead(x,y,z):
    g = pr2_controllers_msgs.msg.PointHeadGoal()
    g.target.header.frame_id = '/base_link'
    g.target.point.x = x
    g.target.point.y = y
    g.target.point.z = z
    print "sending head goal of:",x,y,z
    g.min_duration = rospy.Duration(.5)
    client.send_goal(g)


transformer=tf.Transformer(True, rospy.Duration(10.0))
listener = tf.TransformListener()


def getRobotPosition():
    listener.waitForTransform('/base_link','/odom_combined', rospy.Time(), rospy.Duration(.20))
    t=listener.lookupTransform('/odom_combined','/base_link', rospy.Time())#Note that a time of zero means latest common time
    return (t[0][0],t[0][1])

def getTargetPose():
    target=PoseStamped()
    target.header.stamp=rospy.Time.now()
    listener.waitForTransform('/base_link','/odom_combined', target.header.stamp, rospy.Duration(.20))
    target.header.frame_id="/odom_combined"
    target.pose.position.x=last_direction[0]
    target.pose.position.y=last_direction[1]
    target.pose.orientation.w=1.0

    local=listener.transformPose('/base_link', target)#Note that a time of zero means latest common time
    return (local.pose.position.x,local.pose.position.y)


def vectorToTarget(target):
    pos=getRobotPosition()

    return (target[0]-pos[0],target[1]-pos[1])

def distanceToTarget(target):
    v=vectorToTarget(target)
    return sqrt(v[0]*v[0]+v[1]*v[1])

def getangle(scan,index):
    return scan.angle_min+scan.angle_increment*float(index)

last_direction=(0,0)
minrange=5.0
minangle=0.0
minx=0.0
miny=0.0

def myLaserCallback(scan):
    #print "laser delay: ",(rospy.Time.now()-scan.header.stamp).to_seconds()
    global minx, miny, minrange, minangle
    minrange=scan.ranges[0]
    minangle=0.0
    #find smallest range in laser scan:
    for i in range(len(scan.ranges)):
        if minrange > scan.ranges[i] and scan.ranges[i] > scan.range_min +.001:
            minrange = scan.ranges[i]
            minangle = getangle(scan,i)
	    minx=cos(minangle)*minrange
	    miny=sin(minangle)*minrange
	 
    #print minrange,minangle,minx,miny

def motorCmd(_minx,_miny,_minrange):
    _minrange=max(.1,_minrange)
    t0=rospy.Time.now()
    cmd = Twist()
    ptarget=getTargetPose()
    pdist=distanceToTarget(last_direction)
    if pdist<.2:
       print "reached point"
       pub.publish(cmd)
       return False
    print pdist
    #cmd.linear.y=.25*( - sin(minangle)*.50/minrange)
    #cmd.linear.x=.25*( - cos(minangle)*.50/minrange)
    #now turn that into a direction:
    cmd.linear.y=.15*(ptarget[1]/pdist - _miny*.80/(_minrange*_minrange))
    cmd.linear.x=.15*( ptarget[0]/pdist - _minx*.80/(_minrange*_minrange))
    #if the object is within .8 meters, turn to face
#    if minrange < .8:
    cmd.angular.z = atan2(ptarget[1],ptarget[0]) #copysign(.1,minangle) 
    
    if abs(cmd.linear.x) < .1:
      cmd.linear.x=0.0
    if abs(cmd.linear.y) < .1:
      cmd.linear.y=0.0
    if abs(cmd.angular.z) < .2:
      cmd.angular.z=0.0      
    if   abs(cmd.angular.z) < .2 and abs(cmd.linear.y) < .1 and abs(cmd.linear.x) < .1:
      return False
    #print cmd.linear.x,cmd.linear.y, atan2(ptarget[1],ptarget[0]), _minrange , _minx , _miny
       
    #regardless of whether we set things, publish the command:
    pub.publish(cmd)   
    target=PoseStamped()
    target.header.frame_id="/odom_combined"
    target.header.stamp=rospy.Time.now()
    target.pose.position.x=last_direction[0]
    target.pose.position.y=last_direction[1]
    target.pose.position.z=.3
    target.pose.orientation.w=1.0
    
    posepub.publish(target)
    target.header.frame_id="/base_link"
    #target.pose.position.x=_minx
    #target.pose.position.y=_miny
    target.pose.position.x=cmd.linear.x
    target.pose.position.y=cmd.linear.y
    posepub2.publish(target)
    return True
    #print "laser call took: ",(rospy.Time.now()-t0).to_seconds()
    
    

#register a publisher, to topic '/base_controller/command', of type Twist
pub = rospy.Publisher('/base_controller/command', Twist)
posepub = rospy.Publisher('/target_pose', PoseStamped)
posepub2 = rospy.Publisher('/offset_pose', PoseStamped)

#register a callback for messages of type LaserScan, on the topic "/base_scan"
sub = rospy.Subscriber("/base_scan", LaserScan, myLaserCallback)

from polled_camera.srv import GetPolledImage
trigger_camera = rospy.ServiceProxy('/prosilica/request_image', GetPolledImage)



"/prosilica/request_image"

client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', pr2_controllers_msgs.msg.PointHeadAction)
print "waiting for server... "
client.wait_for_server()
print "found server "

def lookAround():
    for angle in range(-30,30,3):
      if rospy.is_shutdown():
	return
      print float(angle)/10.0
      pointHead(cos(float(angle)/10.0),sin(float(angle)/10.0),1.0)
      time.sleep(.7)
      trigger_camera(response_namespace="polled_camera")
      time.sleep(.4)
      trigger_camera(response_namespace="polled_camera")
      time.sleep(.4)
    
    pointHead(1.0,0.0,1.0)


#this line is equivalent to: 
# while(everything is ok)
#   check for messages, call callbacks
#   sleep
time.sleep(1.0)
while not rospy.is_shutdown():
    #print "setting new goal"
    #pos=getRobotPosition()
    #last_direction=(pos[0]+(random.random()-.50)*5.0,pos[1]+(random.random()-.50)*5.0)
    ##drive to new position
    ##r = rospy.Rate(30) # every 10 sec
    #while motorCmd(minx,miny,minrange) and not rospy.is_shutdown():
      #time.sleep(.05)
    for i in range(30):
      print 30-i," Seconds left to move!"
      time.sleep(1.0)
      pointHead(cos(float(i)/-10.0),sin(float(i)/-10.0),1.0)
    print "done moving"
    #now look around:
    lookAround()
    


