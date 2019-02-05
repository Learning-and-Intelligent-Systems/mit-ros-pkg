#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
import sys
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from tabletop_object_detector.msg import Table
import actionlib
from ee_cart_imped_control.msg import EECartImpedGoal, EECartImpedAction, StiffPoint
import move_arm_service.srv
import pr2_controllers_msgs.msg
import time
import trajectory_msgs.msg
import sensor_msgs
import tf
from pr2cm_client import *

class EECartImpedClient():
    def __init__(self, is_right_arm):
        self.is_right_arm = is_right_arm
        self.goal = EECartImpedGoal() 
        self.goal.header.frame_id = "torso_lift_link"
        self.goal.header.stamp = rospy.Time.now()

    def add_trajectory_point(x, y, z, ox, oy, oz, ow,
                             fx, fy, fz, tx, ty, tz, isfx, isfy, isfz,
                             istx, isty, istz, time):
        '''
        Creates a new point for force control
        '''
        new_point = len(self.goal.trajectory)
        self.goal.trajectory.append(StiffPoint())
        self.goal.trajectory[new_point].pose.position.x = x
        self.goal.trajectory[new_point].pose.position.y = y
        self.goal.trajectory[new_point].pose.position.z = z
        self.goal.trajectory[new_point].pose.orientation.x = ox
        self.goal.trajectory[new_point].pose.orientation.y = oy
        self.goal.trajectory[new_point].pose.orientation.z = oz
        self.goal.trajectory[new_point].pose.orientation.w = ow
        self.goal.trajectory[new_point].wrench_or_stiffness.force.x = fx
        self.goal.trajectory[new_point].wrench_or_stiffness.force.y = fy
        self.goal.trajectory[new_point].wrench_or_stiffness.force.z = fz
        self.goal.trajectory[new_point].wrench_or_stiffness.torque.x = tx
        self.goal.trajectory[new_point].wrench_or_stiffness.torque.y = ty
        self.goal.trajectory[new_point].wrench_or_stiffness.torque.z = tz
        self.goal.trajectory[new_point].is_wrench.append(isfx)
        self.goal.trajectory[new_point].is_wrench.append(isfy)
        self.goal.trajectory[new_point].is_wrench.append(isfz)
        self.goal.trajectory[new_point].is_wrench.append(istx)
        self.goal.trajectory[new_point].is_wrench.append(isty)
        self.goal.trajectory[new_point].is_wrench.append(istz)
        self.goal.trajectory[new_point].time_from_start = rospy.Duration(time)

    def execute(end_in_cartesian = True):
        pr2cmc = PR2CMClient.get_pr2cm_client()
        pr2cmc.load_ee_cart_imped(self.is_right_arm)
        armchar = 'r' if self.is_right_arm else 'l'
        action_string = armchar + '_arm_cart_imped_controller/ee_cart_imped_action'
        ee_cart_imped_client = actionlib.SimpleActionClient(action_string,EECartImpedAction)
        ee_cart_imped_client.wait_for_server(rospy.Duration(5.0))
        print "waiting for server"
        print "sending goal"
        ee_cart_imped_client.send_goal(self.goal)
        print "waiting for result"		
        result = ee_cart_imped_client.wait_for_result()
        print "RESULT=", result
        if end_in_cartesian:
            pr2cmc.load_cartesian(self.is_right_arm)
        return result
