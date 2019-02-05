#! /usr/bin/env python

import roslib; roslib.load_manifest('pressure_practice')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from ee_cart_imped_control.msg import *

class ee_cart_imped_client:
    def __init__(self, isLeft):
        if isLeft:
            self.client = actionlib.SimpleActionClient('/l_arm_cart_imped_controller/ee_cart_imped_action', EECartImpedAction)
        else:
            self.client = actionlib.SimpleActionClient('/r_arm_cart_imped_controller/ee_cart_imped_action', EECartImpedAction)
        rospy.loginfo("Waiting for ee_cart_imped action server blaaaah")
        self.client.wait_for_server()
        rospy.loginfo("Found ee_cart_imped action server")

    # Creates a goal to send to the action server.
        self.goal = EECartImpedGoal()
        self.resetGoal()
    def moveToPoint(self, trans, rot,time):
        self.addTrajectoryPoint(trans[0],trans[1],trans[2],rot[0],rot[1],rot[2],rot[3],
                                1000,1000,1000,30,30,30,
                                False,False,False,False,False,False,time)
    def addTrajectoryPoint(self, x, y, z, ox, oy, oz, ow,
                           fx, fy, fz, tx, ty, tz, isfx, isfy, isfz,
                           istx, isty, istz, time, frame_id=''):
        new_point = len(self.goal.trajectory);
        self.goal.trajectory.append(StiffPoint())
        self.goal.trajectory[new_point].header.stamp = rospy.Time(0)
        self.goal.trajectory[new_point].header.frame_id = frame_id
        self.goal.trajectory[new_point].pose.position.x = x;
        self.goal.trajectory[new_point].pose.position.y = y;
        self.goal.trajectory[new_point].pose.position.z = z;
        self.goal.trajectory[new_point].pose.orientation.x = ox;
        self.goal.trajectory[new_point].pose.orientation.y = oy;
        self.goal.trajectory[new_point].pose.orientation.z = oz;
        self.goal.trajectory[new_point].pose.orientation.w = ow;
        self.goal.trajectory[new_point].wrench_or_stiffness.force.x = fx;
        self.goal.trajectory[new_point].wrench_or_stiffness.force.y = fy; 
        self.goal.trajectory[new_point].wrench_or_stiffness.force.z = fz;
        self.goal.trajectory[new_point].wrench_or_stiffness.torque.x = tx;
        self.goal.trajectory[new_point].wrench_or_stiffness.torque.y = ty;
        self.goal.trajectory[new_point].wrench_or_stiffness.torque.z = tz;
        
        self.goal.trajectory[new_point].is_wrench.append(isfx);
        self.goal.trajectory[new_point].is_wrench.append(isfy);
        self.goal.trajectory[new_point].is_wrench.append(isfz);
        self.goal.trajectory[new_point].is_wrench.append(istx);
        self.goal.trajectory[new_point].is_wrench.append(isty);
        self.goal.trajectory[new_point].is_wrench.append(istz);
        self.goal.trajectory[new_point].time_from_start = rospy.Duration(time);

    def trajectoryTime(self):
        if len(self.goal.trajectory) == 0:
            return rospy.Duration(0)
        return self.goal.trajectory[len(self.goal.trajectory)-1].time_from_start

    def sendGoal(self, wait = True):
        print 'sending goal'
        self.goal.header.stamp = rospy.Time.now()
        self.client.send_goal(self.goal)
        if wait:
            self.client.wait_for_result()
        return self.client.get_state()

    def resetGoal(self):
        self.goal = EECartImpedGoal()
        self.goal.header.frame_id = 'torso_lift_link'
        
    def cancelGoal(self):
        self.client.cancel_goal()
