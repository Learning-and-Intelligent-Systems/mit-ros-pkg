#!/usr/bin/env python
from __future__ import with_statement

import roslib; roslib.load_manifest('openrave_pr2')
import rospy, time
import orrosplanning.srv
import geometry_msgs.msg
from numpy import *

#import pr2_gripper_reactive_approach.controller_manager as controller_manager
#from object_manipulator.convert_functions import *
import tf
import actionlib
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal


if __name__ == '__main__':
    rospy.init_node('or_armplanning_pr2_test')
    print 'waiting for service MoveToHandPosition'
    rospy.wait_for_service('MoveToHandPosition')
    MoveToHandPositionFn = rospy.ServiceProxy('MoveToHandPosition',orrosplanning.srv.MoveToHandPosition)
    req = orrosplanning.srv.MoveToHandPositionRequest()
    req.hand_frame_id = 'r_gripper_palm_link'
    #req.hand_goal.pose.position = geometry_msgs.msg.Point(.6,-.189,.8)
    req.hand_goal.pose.position = geometry_msgs.msg.Point(0.293, -0.677, 0.963)
    req.hand_goal.pose.orientation = geometry_msgs.msg.Quaternion(0,0,0,1)
    req.hand_goal.header.frame_id = 'base_footprint'
    req.manip_name = 'rightarm'
    print 'sending MoveToHandPosition request'
    res = MoveToHandPositionFn(req)
    #print res

    #use_slip_detection = 0                                                            
    #tf_listener = tf.TransformListener()
    #cms = [controller_manager.ControllerManager('r', tf_listener, use_slip_detection),
    #       controller_manager.ControllerManager('l', tf_listener, use_slip_detection)]
    whicharm = 'r'
    joint_trajectory_action_name = ''+whicharm+'_arm_controller/joint_trajectory_action'
    joint_action_client = actionlib.SimpleActionClient(joint_trajectory_action_name, JointTrajectoryAction)
    print 'waiting for',joint_trajectory_action_name
    print 'have you set ROS_IP?'
    joint_action_client.wait_for_server()

    
    goal = JointTrajectoryGoal()
    goal.trajectory = res.traj
    print 'sending JointTrajectoryGoal'
    joint_action_client.send_goal(goal)
