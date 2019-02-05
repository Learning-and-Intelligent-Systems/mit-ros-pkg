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
import select
import socket

from itertools import izip

import actionlib
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, PointHeadAction, PointHeadGoal, Pr2GripperCommandAction, Pr2GripperCommandGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from orrosplanning.srv import MoveToHandPositionResponse

def drawTransform(env,T,length=0.1,linewidth=.01):
    """draws a set of arrows around a coordinate system
    """
    return [env.drawarrow(p1=T[0:3,3],
                          p2=T[0:3,3]+length*T[0:3,0],
                          linewidth=linewidth,color=[1.0,0.0,0.0]),
            env.drawarrow(p1=T[0:3,3],
                          p2=T[0:3,3]+length*T[0:3,1],
                          linewidth=linewidth,color=[0.0,1.0,0.0]),
            env.drawarrow(p1=T[0:3,3],
                          p2=T[0:3,3]+length*T[0:3,2],
                          linewidth=linewidth,color=[0.0,0.0,1.0])]

def angleDistance(angle1, angle2):
    d = angle1 - angle2
    if d > pi:
        return d - 2 * pi
    elif d < -pi:
        return d + 2 * pi
    else:
        return d

if __name__=='__main__':
    def on_exit(sig, func=None):
        print 'ctrl+c handler triggered'
        #RaveDestroy()

    signal.signal(signal.SIGINT, on_exit)

    rospy.init_node('initsim_node')
    
    l_arm_joint_trajectory_action_name = 'l_arm_controller/joint_trajectory_action'
    l_arm_joint_action_client = actionlib.SimpleActionClient(l_arm_joint_trajectory_action_name, JointTrajectoryAction)
    print 'waiting for',l_arm_joint_trajectory_action_name
    print 'have you set ROS_IP?'
    l_arm_joint_action_client.wait_for_server()

    r_arm_joint_trajectory_action_name = 'r_arm_controller/joint_trajectory_action'
    r_arm_joint_action_client = actionlib.SimpleActionClient(r_arm_joint_trajectory_action_name, JointTrajectoryAction)
    print 'waiting for',r_arm_joint_trajectory_action_name
    r_arm_joint_action_client.wait_for_server()

    l_gripper_action_client_name = 'l_gripper_controller/gripper_action'
    l_gripper_action_client = actionlib.SimpleActionClient(l_gripper_action_client_name, Pr2GripperCommandAction)
    print 'waiting for',l_gripper_action_client_name
    l_gripper_action_client.wait_for_server()

    r_gripper_action_client_name = 'r_gripper_controller/gripper_action'
    r_gripper_action_client = actionlib.SimpleActionClient(r_gripper_action_client_name, Pr2GripperCommandAction)
    print 'waiting for',r_gripper_action_client_name
    r_gripper_action_client.wait_for_server()
    
    torso_joint_trajectory_action_name = 'torso_controller/joint_trajectory_action'
    torso_joint_action_client=actionlib.SimpleActionClient(torso_joint_trajectory_action_name, JointTrajectoryAction)
    print 'waiting for', torso_joint_trajectory_action_name
    torso_joint_action_client.wait_for_server()

    point_head_action_name = 'head_traj_controller/point_head_action'
    point_head_action_client = actionlib.SimpleActionClient(point_head_action_name, PointHeadAction)
    print 'waiting for', point_head_action_name
    point_head_action_client.wait_for_server()
    print 'done init ROS'


    env = Environment()

    env.SetViewer('qtcoin')
    Tcamera = matrixFromAxisAngle([0.564935, 0.581771, 0.585142],2.118629)
    Tcamera[:3,3] = [1.277192, 0.174227, 0.732486]
    env.GetViewer().SetCamera(Tcamera)

    env.Reset()
    env.Load('data/csail-pr2-table-pushing.env.xml')

    robot = env.GetRobot('pr2')
    robot.SetActiveManipulator('leftarm_torso')
    manip=robot.GetActiveManipulator()
    basemanip = interfaces.BaseManipulation(robot)


    print 'resetting joint limits'
    joint_names = ['torso_lift_joint',
                   'l_shoulder_lift_joint',
                   'l_shoulder_pan_joint',
                   'l_upper_arm_roll_joint',
                   'l_elbow_flex_joint',
                   'l_wrist_flex_joint',
                   'l_gripper_joint',
                   'r_shoulder_lift_joint',
                   'r_shoulder_pan_joint',
                   'r_upper_arm_roll_joint',
                   'r_elbow_flex_joint',
                   'r_wrist_flex_joint',
                   'r_gripper_joint']

    new_limits = [[.0115, .31-.005],
                  [-.5236+.17, 1.3963-.1],
                  [pi/4-1.5+.15, pi/4+1.5-.15],
                  [1.55-2.35+.15, 1.55+2.35-.15],
                  [-2.3213+.2, -.15],
                  [-2.18+.18, 0-.1],
                  [0-.01, .09-.002],
                  [-.5236+.17, 1.3963-.1],
                  [-pi/4-1.5+.15, -pi/4+1.5-.15],
                  [-1.55-2.35+.15, -1.55+2.35-.15],
                  [-2.3213+.2, -.15],
                  [-2.18+.18, 0-.1],
                  [0-.01, .09-.002]]
    
    for jn,limits in izip(joint_names,new_limits):
        print jn
        print 'old limits:',robot.GetJoint(jn).GetLimits()
        print 'new limits:', limits
        robot.GetJoint(jn).SetLimits([limits[0]],[limits[1]])

    realrobot = env.ReadRobotXMLFile(robot.GetXMLFilename())
    realrobot.SetName('real_pr2')
    for link in realrobot.GetLinks():
        for geom in link.GetGeometries():
            geom.SetTransparency(.8)
    env.AddRobot(realrobot, True)
    realrobot.SetTransform(robot.GetTransform())
    realrobot.SetDOFValues(robot.GetDOFValues())
    realrobot.Enable(False)

    def updateRobotJoints(msg):
        values = realrobot.GetDOFValues()
        for name,pos in izip(msg.name, msg.position):
            j = realrobot.GetJoint(name)
            if j is not None:
                values[j.GetDOFIndex()] = pos
        realrobot.SetDOFValues(values)
    
    rospy.Subscriber('/joint_states',JointState,updateRobotJoints,queue_size=1)

    time.sleep(1.0)
    robot.SetDOFValues(realrobot.GetDOFValues())

    def executeTraj(trajdata,whicharm='l'):
        torso_goal = JointTrajectoryGoal()
        arm_goal = JointTrajectoryGoal()
        tokens = trajdata.split()
        numpoints = int(tokens[0])
        dof = int(tokens[1])
        trajoptions = int(tokens[2])
        numvalues = dof
        offset = 0
        if trajoptions & 4: # timestamps
            numvalues += 1
            offset += 1
        if trajoptions & 8: # base transforms
            numvalues += 7
        if trajoptions & 16: # velocities
            numvalues += dof
        if trajoptions & 32: # torques
            numvalues += dof
        arm_res = MoveToHandPositionResponse()
        torso_res = MoveToHandPositionResponse()
        for i in range(numpoints):
            start = 3+numvalues*i
            torso_pt = JointTrajectoryPoint()
            arm_pt = JointTrajectoryPoint()
            k = 0
            for j in robot.GetJoints(manip.GetArmIndices()):
                pos = float(tokens[start+offset+j.GetDOFIndex()])
                if j.GetName()=='torso_lift_joint':
                    if i>0:
                        prevpos = torso_res.traj.points[-1].positions[0]
                        newpos = prevpos + angleDistance(pos, prevpos)
                        torso_pt.positions.append(newpos)
                    else:
                        torso_pt.positions.append(pos)
                else:
                    if i>0:
                        prevpos = arm_res.traj.points[-1].positions[k]
                        newpos = prevpos + angleDistance(pos, prevpos)
                        arm_pt.positions.append(newpos)
                    else:
                        arm_pt.positions.append(pos)
                    k = k+1
            if trajoptions & 4:
                arm_pt.time_from_start = rospy.Duration(float(tokens[start]))*2
                torso_pt.time_from_start = rospy.Duration(float(tokens[start]))*2
            arm_res.traj.joint_names = \
                filter(lambda jn: jn!='torso_lift_joint',
                       [j.GetName() for j in robot.GetJoints(manip.GetArmIndices())])
            arm_res.traj.points.append(arm_pt)
            torso_res.traj.joint_names = ['torso_lift_joint']
            torso_res.traj.points.append(torso_pt)
        arm_goal.trajectory = arm_res.traj
        torso_goal.trajectory = torso_res.traj
        print 'sending torso JointTrajectoryGoal'
        torso_joint_action_client.send_goal(torso_goal)

        print 'sending arm JointTrajectoryGoal'
        if whicharm == 'l':
            l_arm_joint_action_client.send_goal(arm_goal)
            l_arm_joint_action_client.wait_for_result()
        else:
            r_arm_joint_action_client.send_goal(arm_goal)
            r_arm_joint_action_client.wait_for_result()
        torso_joint_action_client.wait_for_result()


    def moveGripper(whichgripper, position, effort):
        gripperGoal = Pr2GripperCommandGoal()
        gripperGoal.command.position = position
        gripperGoal.command.max_effort = effort
        print 'sending gripper command goal'
        if whichgripper=='l':
            l_gripper_action_client.send_goal(gripperGoal)
            l_gripper_action_client.wait_for_result()
        elif whichgripper=='r':
            r_gripper_action_client.send_goal(gripperGoal)
            r_gripper_action_client.wait_for_result()
        else:
            print whichgripper,'needs to be either l or r'

    robot.SetActiveManipulator('leftarm_torso')
    manip=robot.GetActiveManipulator()
    basemanip = interfaces.BaseManipulation(robot)    
    robot.SetActiveDOFs(manip.GetArmIndices())
    # torso_lift, shoulder_pan, shoulder_lift, upper_arm_roll, elbow_flex, forearm_roll, wrist_flex, wrist_roll
    trajdata = basemanip.MoveActiveJoints([.305,pi/2,0,0,-.15,0,-.1,0], outputtraj=True)
    raw_input('press enter to move the robot')
    executeTraj(trajdata,'l')
    robot.SetActiveManipulator('rightarm_torso')
    manip=robot.GetActiveManipulator()
    basemanip = interfaces.BaseManipulation(robot)
    robot.SetActiveDOFs(manip.GetArmIndices())
    trajdata = basemanip.MoveActiveJoints([.305,-pi/2,0,0,-.15,0,-.1,0], outputtraj=True)
    raw_input('press enter to move the robot')
    executeTraj(trajdata,'r')
    
    moveGripper('l',.08,-1);
    moveGripper('r',0,50);

    raw_input('press enter to finish')
    RaveDestroy()
