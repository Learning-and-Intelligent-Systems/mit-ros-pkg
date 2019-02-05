#!/usr/bin/env python
import roslib; roslib.load_manifest('target_nav')

from target_nav.srv import *
import rospy
import actionlib
import time
import pr2_controllers_msgs.msg

#for chessboard:
import checkerboard_pose.srv
import geometry_msgs.msg

#for tf:
import tf
from tf import transformations
import numpy

import threading

def pointHead(x,y,z):
    g = pr2_controllers_msgs.msg.PointHeadGoal()
    g.target.header.frame_id = '/base_link'
    g.target.point.x = x
    g.target.point.y = y
    g.target.point.z = z
    print "sending head goal of:",x,y,z
    g.min_duration = rospy.Duration(.5)
    client.send_goal(g)
    


rospy.init_node('headbanger')
client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', pr2_controllers_msgs.msg.PointHeadAction)
print "waiting for server... "
client.wait_for_server()
print "found server "
while not rospy.is_shutdown():
  pointHead(-1.0,.01,10.0)
  time.sleep(1.5)
  pointHead(1.0,.01,0.0)
  time.sleep(.5)
  pointHead(-1.0,-.01,10.0)
  time.sleep(1.5)
  pointHead(1.0,.01,10.0)
  time.sleep(.5)
  pointHead(-1.0,.01,0.0)
  time.sleep(1.5)
  pointHead(1.0,.01,10.0)
  time.sleep(.5)
  pointHead(-1.0,-.01,0.0)
  time.sleep(1.5)
  pointHead(1.0,.01,0.0)
  time.sleep(.5)
#while not rospy.is_shutdown():
  #pointHead(-1.0,.01,2.0)
  #pointHead(-1.0,.1,2.0)
  #time.sleep(1.5)
  #pointHead(-1.0,-.01,2.0)
  #pointHead(-1.0,-.1,2.0)