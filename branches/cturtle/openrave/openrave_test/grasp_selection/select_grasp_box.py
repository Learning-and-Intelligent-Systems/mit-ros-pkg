#!/usr/bin/env python
from __future__ import with_statement # for python 2.5
__author__ = 'Huan Liu'
__copyright__ = 'Copyright (C) 2010 Huan Liu (liuhuanjim013@gmail.com)'
__license__ = 'Apache License, Version 2.0'

from openravepy import __build_doc__
if not __build_doc__:
    from openravepy import *
    from numpy import *
else:
    from openravepy import OpenRAVEModel
    from numpy import inf, array

import time
from itertools import izip
from openravepy.databases import grasping

def pause():
    raw_input('press ENTER to continue...')

def setObjectTransparency(target,transparency=.9):
    for link in target.GetLinks():
        for geom in link.GetGeometries():
            geom.SetTransparency(transparency)

def drawTransform(env,T,length=.01,width=.001):
    """draws a set of arrows around a coordinate system
    """
    return [env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,0],linewidth=width,color=[1.0,0.0,0.0]),
            env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,1],linewidth=width,color=[0.0,1.0,0.0]),
            env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,2],linewidth=width,color=[0.0,0.0,1.0])]
            
def showGrasp(robot,Tgrasp,angle=.548):
    """visualizes the robot configuration when robot.GetActiveManipulator().GetEndEffectorTransform()==Tgrasp
    
    :param Tgrasp: a row-major 4x4 matrix in numpy.array format
    :param angle: gripper angle
    """
    v = robot.GetActiveDOFValues()
    v[robot.GetJoint('l_gripper_joint').GetDOFIndex()] = angle
    
    robot.SetActiveDOFValues(v)
    O_T_R = robot.GetTransform() # robot transform R in global frame O 
    O_T_G = robot.GetActiveManipulator().GetEndEffectorTransform() # grasping frame G in global frame O
    G_T_O = linalg.inv(O_T_G) # global frame O in grasping frame G
    G_T_R = dot(G_T_O, O_T_R) # robot frame R in grasping frame G
    O_T_G_goal = Tgrasp # final grasping frame G_goal in global frame O 
    O_T_R_goal = dot(O_T_G_goal,G_T_R) # final robot transform R_goal in global frame O                
    robot.SetTransform(O_T_R_goal)
    pause()
    robot.SetTransform(O_T_R)

if __name__=='__main__':
    env = Environment()
    try:
        # sets collision checker to pqp for min distance calculation
        collision_checker = RaveCreateCollisionChecker(env,'pqp')
        collision_checker.SetCollisionOptions(CollisionOptions.Contacts)
        env.SetCollisionChecker(collision_checker)
        
        # init target
        target = env.ReadKinBodyXMLFile('bizzaro_mug_small.kinbody.xml')
        O_T_Target = array([[1.,0.,0.,1.],
                            [0.,1.,0.,0.],
                            [0.,0.,1.,1.],
                            [0.,0.,0.,1.]])
        target.SetTransform(O_T_Target)
        env.AddKinBody(target)
        setObjectTransparency(target)
        env.SetViewer('qtcoin')
        time.sleep(.2)

        # init robot
        robot = env.ReadRobotXMLFile('../robots/pr2mit.robot.xml')
        env.AddRobot(robot)
        # init robot pose: l_shoulder_pan, r_shoulder_pan, torso, l_gripper
        v = robot.GetActiveDOFValues()
        v[robot.GetJoint('l_shoulder_pan_joint').GetDOFIndex()]= 3.14/2
        v[robot.GetJoint('r_shoulder_pan_joint').GetDOFIndex()] = -3.14/2
        v[robot.GetJoint('torso_lift_joint').GetDOFIndex()] = 0
        v[robot.GetJoint('l_gripper_joint').GetDOFIndex()] = .54
        robot.SetActiveDOFValues(v)

        showGrasp(robot,O_T_Target)

    except openrave_exception, e:
        print e
    finally:
        # destroy planning environment and clean up
        #pass
        env.Destroy()
        RaveDestroy()
