#! /usr/bin/python
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('pr2_controllers_msgs')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('pressure_practice')

import rospy
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from geometry_msgs.msg import *

class head_control:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
        self.client.wait_for_server()
        print "head control initialized"

    def look_at(self,x,y,z):
        
        g = PointHeadGoal()
        g.target.header.frame_id = 'torso_lift_link'
        g.target.point.x = x
        g.target.point.y = y
        g.target.point.z = z
        g.min_duration = rospy.Duration(1.0)

        self.client.send_goal(g)
        self.client.wait_for_result()

        if self.client.get_state() == GoalStatus.SUCCEEDED:
            return True
        else:
            return False
