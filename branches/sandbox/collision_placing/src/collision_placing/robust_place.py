#!/usr/bin/env python
import roslib; 
roslib.load_manifest('collision_placing')
import rospy
import sys
import doodie
import math
import time
from motion_prim import *

from geometry_msgs.msg import PoseStamped
import pr2_python.geometry_tools as gt

def get_goal(obj):
    goal = PoseStamped()
    goal.header.frame_id = '/torso_lift_link'
    prims = []
    if obj=="box":        
        orien = gt.euler_to_quaternion(0.0,0.0,math.pi/2)    
        goal.pose.position.x = 0.6
        goal.pose.position.y = -0.25#0.5#0.2
        goal.pose.position.z = -0.28
        goal.pose.orientation = orien
        prims = [0] #[MotionPrim.Tip]
    else:
        orien = gt.euler_to_quaternion(0.0,0.0,0.0)    
        goal.pose.position.x = 0.75#0.6
        goal.pose.position.y = -0.25#0.5#0.2
        goal.pose.position.z = -0.28
        goal.pose.orientation = orien
        prims = [MotionPrim.STICK]
    return (goal, prims)

rospy.init_node("robust_placing")
        
action = sys.argv[1] # 'dropping','passive','robust'
obj_type = sys.argv[2] # 'paper_plate','plastic_plate','box'
obj_name = sys.argv[3] # 'graspable_object_****'

(goal, prims) = get_goal(obj_type)

start = time.time()
my_placer = doodie.placer(goal, prims)
runtime = my_placer.run(action, obj_type, obj_name)
rospy.loginfo("Runtime=%s", str(runtime-start))
