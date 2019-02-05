#!/usr/bin/env python
import roslib; roslib.load_manifest('pushing_test')
import rospy

from openravepy import *
from numpy import *
from openravepy.databases import inversekinematics

import sys
import os
import signal
import threading
import time

from itertools import izip

import actionlib
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from orrosplanning.srv import MoveToHandPositionResponse


if __name__=='__main__':
    def on_exit(sig, func=None):
        print 'ctrl+c handler triggered'
        RaveDestroy()

    signal.signal(signal.SIGINT, on_exit)

    threads = []
    

    rospy.init_node('orthreading_node')
    whicharm = 'l'
    arm_joint_trajectory_action_name = whicharm+'_arm_controller/joint_trajectory_action'
    arm_joint_action_client = actionlib.SimpleActionClient(arm_joint_trajectory_action_name, JointTrajectoryAction)
    print 'waiting for',arm_joint_trajectory_action_name
    print 'have you set ROS_IP?'
    arm_joint_action_client.wait_for_server()
    torso_joint_trajectory_action_name = 'torso_controller/joint_trajectory_action'
    torso_joint_action_client=actionlib.SimpleActionClient(torso_joint_trajectory_action_name, JointTrajectoryAction)
    print 'waiting for', torso_joint_trajectory_action_name
    torso_joint_action_client.wait_for_server()
    print 'done init ROS'


    env = Environment()

    env.SetViewer('qtcoin')
    Tcamera = matrixFromAxisAngle([0.564935, 0.581771, 0.585142],2.118629)
    Tcamera[:3,3] = [1.277192, 0.174227, 0.732486]
    env.GetViewer().SetCamera(Tcamera)

    env.Reset()
    env.Load('data/csail-pr2-table.env.xml')

    robot = env.GetRobot('pr2')
    robot.SetActiveManipulator('leftarm_torso')
    manip=robot.GetActiveManipulator()

    basemanip = interfaces.BaseManipulation(robot)

    realrobot = env.ReadRobotXMLFile(robot.GetXMLFilename())
    realrobot.SetName('shadow_pr2')
    for link in realrobot.GetLinks():
        for geom in link.GetGeometries():
            geom.SetTransparency(.8)
    env.AddRobot(realrobot, True)
    realrobot.SetTransform(robot.GetTransform())
    realrobot.SetDOFValues(robot.GetDOFValues())
    realrobot.Enable(False)

    def UpdateRobotJoints(msg):
        values = realrobot.GetDOFValues()
        for name,pos in izip(msg.name, msg.position):
            j = realrobot.GetJoint(name)
            if j is not None:
                values[j.GetDOFIndex()] = pos
        realrobot.SetDOFValues(values)
    
    rospy.Subscriber('/joint_states',JointState,UpdateRobotJoints,queue_size=1)

    

    rospy.spin()
    print 'haha'
