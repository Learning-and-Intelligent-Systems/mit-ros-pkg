#!/usr/bin/env python

import roslib; 
roslib.load_manifest('imp_placing')
import rospy
from geometry_msgs.msg import PoseStamped
from modes import *
from place import *

rospy.init_node("test_dropping")

goal = PoseStamped()
goal.header.frame_id = '/torso_lift_link'
goal.pose.position.x = 0.6
goal.pose.position.y = 0.25#0.5#0.2
goal.pose.position.z = -0.30
goal.pose.orientation.x = -0.5#0.0
goal.pose.orientation.y = 0.5#0.0
goal.pose.orientation.z = 0.5#-0.707#0.0#-0.707#0.0
goal.pose.orientation.w = -0.5#0.707#1.0#0.707#1.0
modes = [1.0, 0.0]

# start = time.time()
my_placer = Placer(goal, modes)
my_placer.run("dropping", "box", right=False)
