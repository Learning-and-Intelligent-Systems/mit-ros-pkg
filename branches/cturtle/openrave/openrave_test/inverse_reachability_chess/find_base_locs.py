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

import time,sys
from optparse import OptionParser
from openravepy.databases import inversereachability,grasping
from itertools import izip

class IR:
    def __init__(self,env):
        self.env = env
        self.robot = self.env.GetRobot('pr2')
        self.manip = self.robot.GetActiveManipulator()

        # initialize robot pose
        v = self.robot.GetActiveDOFValues()
        v[35] = 3.14/2 # l shoulder pan
        v[56] = -3.14/2 # r shoulder pan
        """note here the torso height must be set to 0, because the database was generated for torso height=0"""
        v[14] = 0# torso  
        v[47] = .54 # l gripper
        self.robot.SetActiveDOFValues(v)
    
        # load inverserechability database
        self.irmodel = inversereachability.InverseReachabilityModel(robot=self.robot)
        starttime = time.time()
        print 'loading irmodel'
        if not self.irmodel.load():
            print 'do you want to generate irmodel for your robot? it might take several hours'
            print 'or you can go to http://people.csail.mit.edu/liuhuan/pr2/openrave/.openrave/ to get the database for PR2'
            input = raw_input('[Y/n]')
            if input == 'y' or input == 'Y' or input == '\n' or input == '':
                class IrmodelOption:
                    def __init__(self,robot,heightthresh=.05,quatthresh=.15,Nminimum=10,id=None,jointvalues=None):
                        self.robot = robot
                        self.heightthresh = heightthresh
                        self.quatthresh = quatthresh
                        self.Nminimum = Nminimum
                        self.id = id
                        self.jointvalues = jointvalues
                option = IrmodelOption(self.robot)
                self.irmodel.autogenerate(option)
                self.irmodel.load()
            else:
                self.env.Destroy()
                RaveDestroy()
                sys.exit()
        print 'time to load inverse-reachability model: %fs'%(time.time()-starttime)
        # make sure the robot and manipulator match the database
        assert self.irmodel.robot == self.robot and self.irmodel.manip == self.robot.GetActiveManipulator()   


    def showPossibleBasePoses(self,Tgrasp, gripper_angle=.548,N=1,overlay=False):
        """visualizes possible base poses for a grasp specified by Tgrasp and gripper_angle
        
        :param Tgrasp: 4x4 numpy.array, row major matrix, the grasp transform in global frame. equals manip.GetEndEffectorTransform() in the goal state
        :param gripper_angle: float, the gripper angle
        :param N: int, the number of sample poses we want to get 
        """
        # setting the gripper angle
        v = self.robot.GetActiveDOFValues()
        v[47] = gripper_angle # l gripper
        self.robot.SetActiveDOFValues(v)

        # find the robot base distribution for the grasp specified by Tgrasp
        # Input for computeBaseDistribution():
        #      Tgrasp: 4x4 numpy.array, row major matrix, the grasp transform in global frame
        #              equals manip.GetEndEffectorTransform() in the goal state
        #      logllthresh: float, lower this number if the sampler cannot return a sample 
        # Output for computeBaseDistribution():
        #      densityfn: gaussian kernel density function taking poses of openrave quaternion type, returns probabilities
        #      samplerfn: gaussian kernel sampler function taking number of sample and weight, returns robot base poses and joint states
        #      bounds: 2x3 array, bounds of samples, [[min rotation, min x, min y],[max rotation, max x, max y]]
        densityfn,samplerfn,bounds = self.irmodel.computeBaseDistribution(Tgrasp,logllthresh=2)
        if densityfn == None:
            print 'the specified grasp is not reachable!'
            return
        
        # Code fragment from `examples.mobilemanipulation`
        # initialize sampling parameters
        goals = []
        numfailures = 0
        starttime = time.time()
        timeout = inf
        with self.robot:
            while len(goals) < N:
                if time.time()-starttime > timeout:
                    break
                poses,jointstate = samplerfn(N-len(goals))
                for pose in poses:
                    self.robot.SetTransform(pose)
                    self.robot.SetJointValues(*jointstate)
                    # validate that base is not in collision
                    if not self.manip.CheckIndependentCollision(CollisionReport()):
                        q = self.manip.FindIKSolution(Tgrasp,filteroptions=IkFilterOptions.CheckEnvCollisions)
                        if q is not None:
                            values = self.robot.GetDOFValues()
                            values[self.manip.GetArmIndices()] = q
                            goals.append((Tgrasp,pose,values))
                        elif self.manip.FindIKSolution(Tgrasp,0) is None:
                            numfailures += 1
        #print 'showing %d results'%N
        for ind,goal in enumerate(goals):
            #raw_input('press ENTER to show goal %d'%ind)
            Tgrasp,pose,values = goal
            self.robot.SetTransform(pose)
            self.robot.SetJointValues(values)
            print 'grasp transform in global frame'
            # openrave pose quat [w x y z] trans [x y z]
            pose_grasp = poseFromMatrix(Tgrasp)
            print 'rot: [%8.4f, %8.4f, %8.4f, %8.4f] trans: [%8.4f, %8.4f, %8.4f]'%(pose_grasp[0],pose_grasp[1],pose_grasp[2],pose_grasp[3],pose_grasp[4],pose_grasp[5],pose_grasp[6])
            print 'base transform in global frame'
            print 'rot: [%8.4f, %8.4f, %8.4f, %8.4f] trans: [%8.4f, %8.4f, %8.4f]'%(pose[0],pose[1],pose[2],pose[3],pose[4],pose[5],pose[6])
            pause()

        if overlay:
            raw_input('press ENTER to show all results simultaneously')
            # Code fragment from `databases.inversereachability`
            transparency = .8
            with self.env: # save the environment state
                self.env.Remove(self.robot)
                newrobots = []
                for goal in goals:
                    Tgrasp,T,values = goal
                    newrobot = self.env.ReadRobotXMLFile(self.robot.GetXMLFilename())
                    newrobot.SetName(self.robot.GetName())
                    for link in newrobot.GetLinks():
                        for geom in link.GetGeometries():
                            geom.SetTransparency(transparency)
                    self.env.AddRobot(newrobot,True)
                    newrobot.SetTransform(T)
                    newrobot.SetJointValues(values)
                    newrobots.append(newrobot)
            print 'overlaying all results, wait for a few seconds for the viewer to update'
            time.sleep(10)
        #pause()

def createPieces(env):
    center_x = 0.65
    center_y = -0.188
    z        = .7
    pitch    = 0.05
    
    pieces = []
    
    for i in range(8):
        for j in range(8):
            x,y = (center_x + (i-3.5)*pitch,
                     center_y + (j-3.5)*pitch)
            name = 'piece_%d_%d'%(i,j)
            bounding_boxes = array([(0,0,.025/2.0,.006,.006,.0125)])
            piece = env.CreateKinBody()
            piece.SetName(name)
            piece.InitFromBoxes(bounding_boxes,True)
            O_T_piece=eye(4)
            O_T_piece[:3,3]=[x,y,z]
            piece.SetTransform(O_T_piece)
            pieces.append(piece)
            env.AddKinBody(piece)
    return pieces

def drawTransform(env,T,length=.01,width=.001):
    """draws a set of arrows around a coordinate system
    """
    return [env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,0],linewidth=width,color=[1.0,0.0,0.0]),
            env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,1],linewidth=width,color=[0.0,1.0,0.0]),
            env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,2],linewidth=width,color=[0.0,0.0,1.0])]

def pause():
    raw_input('press ENTER to continue...')

if __name__=='__main__':
    env = Environment()
    try:
        pieces = createPieces(env)
        env.SetViewer('qtcoin')
        time.sleep(.2)
        
        # init robot
        robot = env.ReadRobotXMLFile('robots/pr2-beta-static.robot.xml')
        env.AddRobot(robot)
        # init robot pose: l_shoulder_pan, r_shoulder_pan, torso, l_gripper
        robot.SetJointValues([pi/2,-pi/2,0,0.548],[35,56,14,47])

        
        ir = IR(env)
        for piece in pieces:
            Tgrasp = eye(4)
            Tgrasp[:3,3] = piece.GetTransform()[:3,3]
            Tgrasp[2,3] += .06
            Tgrasp[:3,:3] = [[0,0,1],[0,1,0],[-1,0,0]]
            gripper_angle = .5
            N=1
            ir.showPossibleBasePoses(Tgrasp, gripper_angle, N,False)
    except openrave_exception, e:
        print e
    finally:
        # destroy planning environment and clean up
        env.Destroy()
        RaveDestroy()
