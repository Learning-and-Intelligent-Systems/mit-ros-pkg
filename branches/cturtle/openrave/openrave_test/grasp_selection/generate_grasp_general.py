#!/usr/bin/env python
from __future__ import with_statement # for python 2.5
__author__ = 'Huan Liu'
__copyright__ = 'Copyright (C) 2011 Huan Liu (liuhuanjim013@gmail.com)'
__license__ = 'Apache License, Version 2.0'

from openravepy import __build_doc__
if not __build_doc__:
    from openravepy import *
    from numpy import *
else:
    from openravepy import OpenRAVEModel
    from numpy import inf, array

import time
import sys
#from itertools import izip
#import pickle
#import os
#from multiprocessing import Process, Manager

from openravepy.databases import grasping
from utilities import *
from general_grasp.grasp import Grasp

class GraspGenerator:
    def __init__(self,robot,target):
        self.env = robot.GetEnv()
        self.robot = robot
        self.target = target
    
    def generateGrasps(self):
        handles = []
        pre_grasps = []
        ind=0
        indices = self.target.GetLinks()[0].GetCollisionData().indices
        #random.shuffle(indices)
        #indices= [[5871,5872,5873]]
        for tri in indices:
            #print '%d/%d'%(ind,len(self.target.GetLinks()[0].GetCollisionData().indices))
            ind+=1
            #if i % 10 != 0:
            #    continue
            a,b,c = [self.target.GetLinks()[0].GetCollisionData().vertices[i] for i in tri]
            Target_P_center = [0,0,0,1.0]
            center = (a+b+c)/3.0
            Target_P_center[:3] = center
            Target_N_center = [0,0,0,1.0]
            #Target_N_center[:3] = vec_norm(cross_3d(center,b,c))
            center_N_center = vec_norm(cross_3d(center,b,c))
            Target_N_center[:3] = center + center_N_center
            O_N_center = dot(target.GetTransform(),Target_N_center)
            O_P_center = dot(target.GetTransform(),Target_P_center)
            handles.append(self.env.drawarrow(p1=O_P_center,p2=O_P_center+(O_N_center-O_P_center)*.01, 
                                              linewidth=.0001, color=[1,0,0]))
            a = array([0,0,-1])
            b = array(vec_norm(O_N_center[:3]-O_P_center[:3]))
            
            if (b==array([0,0,-1])).all() or (b==array([0,0,1])).all():
                axis = array([0,0,1])
                angle = 0
            else:
                angle = arccos(dot(a,b)/(vec_mag(a)*vec_mag(b)))
                axis = vec_norm(cross_3d(array([0,0,0]),a,b))
            
            Tgrasp = matrixFromAxisAngle(axis,angle)
            
            
            #print a,b,cross_3d(array([0,0,0]),a,b)
            #print axis,angle
            #print Tgrasp
            Tgrasp[:,3] = O_P_center
            num_approachangles = 8
            #pause()
            delta_approach_angle = pi/num_approachangles
            bad = False
            for j in xrange(num_approachangles):
                R = matrixFromAxisAngle(b,delta_approach_angle)
                #print b, approach_angle
                #print R
                #print Tgrasp
                Tgrasp[:3,:3] = dot(R,Tgrasp)[:3,:3]
                num_gripperangles = 10
                for i in xrange(num_gripperangles):
                    gripper_angle = float(num_gripperangles-i)/num_gripperangles * .548
                    #print Tgrasp
                    try:
                        if not showGrasp(self.robot,Tgrasp,angle=gripper_angle,wait=False,moveback=False,showbody=False,checkcollision=True):
                            pre_grasps.append([Tgrasp,b,(j+1)*delta_approach_angle,gripper_angle,tri])
                            #showGrasp(self.robot,Tgrasp,angle=gripper_angle,wait=True,moveback=True,showbody=False,checkcollision=False)
                            #print 'found one!'
                            print 'good pregrasp %d'%len(pre_grasps)
                            #bad = True
                            break
                        if self.env.CheckCollision(self.robot.GetLink('l_gripper_palm_link')):
                            bad= True
                            #print 'bad'
                            break
                    except openrave_exception,e:
                        print e
                        print 'tri',tri
                        print 'approach_angle',approach_angle
                        print 'gripper_angle',gripper_angle
                if bad:
                    break
                        
        return pre_grasps


if __name__=='__main__':
    env = Environment()
    try:
        # sets collision checker to pqp for min distance calculation
        collision_checker = RaveCreateCollisionChecker(env,'pqp')
        collision_checker.SetCollisionOptions(CollisionOptions.Contacts)
        env.SetCollisionChecker(collision_checker)
        
        # init target
        #target = env.ReadKinBodyXMLFile('bizzaro_objects/bizzaro_mug_small.kinbody.xml')
        #target = env.ReadKinBodyXMLFile('bizzaro_objects/bizzaro_block.kinbody.xml')
        target = env.ReadKinBodyXMLFile('data/mug2.kinbody.xml')
        O_T_Target = array([[1.,0.,0.,1.],
                            [0.,1.,0.,0.],
                            [0.,0.,1.,1.],
                            [0.,0.,0.,1.]])
        target.SetTransform(O_T_Target)
        env.AddKinBody(target)
        #setObjectTransparency(target)
        nohead = False
        forcegenerate = False
        for arg in sys.argv:
            if arg == 'headless':
                nohead = True
            if arg == 'forcegenerate':
                forcegenerate = True
        if not nohead:
            env.SetViewer('qtcoin')
            time.sleep(.2)
        target_mass = .5

        # init robot
        robot = env.ReadRobotXMLFile('robots/pr2mit.robot.xml')
        env.AddRobot(robot)
        # init robot pose: l_shoulder_pan, r_shoulder_pan, torso, l_gripper
        v = robot.GetActiveDOFValues()
        v[robot.GetJoint('l_shoulder_pan_joint').GetDOFIndex()]= 3.14/2
        v[robot.GetJoint('r_shoulder_pan_joint').GetDOFIndex()] = -3.14/2
        v[robot.GetJoint('torso_lift_joint').GetDOFIndex()] = 0
        v[robot.GetJoint('l_gripper_l_finger_joint').GetDOFIndex()] = .548
        robot.SetActiveDOFValues(v)
        robot.SetActiveManipulator('leftarm')

        saved_pregrasps_filename = 'target.'+target.GetKinematicsGeometryHash()+'.robot.'+robot.GetRobotStructureHash()+'.pregrasps'
        pre_grasps = None
        if os.path.isfile(saved_pregrasps_filename) and not forcegenerate:
            print 'loading pregrasps from',saved_pregrasps_filename
            saved_pregrasps_file = open(saved_pregrasps_filename,'r')
            pre_grasps = pickle.load(saved_pregrasps_file)
            saved_pregrasps_file.close()
        else:
            saved_pregrasps_filename = 'target.'+target.GetKinematicsGeometryHash()+'.robot.'+robot.GetRobotStructureHash()+'.pregrasps'
            saved_pregrasps_file = open(saved_pregrasps_filename,'w')
            gg = GraspGenerator(robot,target)
            pre_grasps = gg.generateGrasps()
            pickle.dump(pre_grasps,saved_pregrasps_file)
            saved_pregrasps_file.close()
        
        print 'loaded %d pregrasps'%len(pre_grasps)

        handles = []
        for Tgrasp,axis,approach_angle,gripper_angle,tri in pre_grasps:
            a,b,c = [target.GetLinks()[0].GetCollisionData().vertices[i] for i in tri]
            Target_P_center = [0,0,0,1.0]
            center = (a+b+c)/3.0
            Target_P_center[:3] = center
            Target_N_center = [0,0,0,1.0]
            #Target_N_center[:3] = vec_norm(cross_3d(center,b,c))
            center_N_center = vec_norm(cross_3d(center,b,c))
            Target_N_center[:3] = center + center_N_center
            O_N_center = dot(target.GetTransform(),Target_N_center)
            O_P_center = dot(target.GetTransform(),Target_P_center)
            handles.append(env.drawarrow(p1=O_P_center,p2=O_P_center+(O_N_center-O_P_center)*.01, 
                                         linewidth=.0001, color=[1,0,0]))
            showGrasp(robot,Tgrasp,gripper_angle,True)
        pause()
        
        
        maxScore = 30

        gmodel = grasping.GraspingModel(robot,target)
        print 'loading grasps from openrave'
        if not gmodel.load():
            gmodel.autogenerate()
        #pause()
        print 'loaded %d grasps from openrave'%len(gmodel.grasps)
        or_grasps = []
        print 'evaluating grasps'
        for g in gmodel.grasps:
            contacts,finalconfig,mindist,volume = gmodel.runGrasp(g)
            bad=False
            #print len(contacts)
            #pause()
            newgrasp = Grasp(robot,target,gmodel.getGlobalGraspTransform(g),
                             finalconfig[0][robot.GetActiveManipulator().GetGripperJoints()[0]],target_mass,effort=10,coefficient_of_friction=.2)
            #handle = newgrasp.show(wait=True,moveback=False)
            #if len(contacts)<50:
            #    continue
            if newgrasp.evaluateGrasp() < maxScore:
                or_grasps.append(newgrasp)

            #print grasp.getContactCounts()
            #time.sleep(.2)
            #utilities.pause()
            #handle = None
        print 'found %d better grasps'%len(or_grasps)            
        or_grasps = sorted(or_grasps,key=lambda g: g.score)

        i=0
        with GripperVisibility(robot.GetActiveManipulator()):
            for grasp in or_grasps:
                i+=1
                print 'showing grasp %d/%d with score %.2f'%(i,len(or_grasps),grasp.score)
                grasp.evaluateGrasp(verbose=True)
                handle = grasp.show(wait=False,moveback=False)
                #print grasp.getContactCounts()
                #time.sleep(.2)
                pause()
                handle = None

        
    except openrave_exception, e:
        print e
    finally:
        # destroy planning environment and clean up
        pass
        #env.Destroy()
        #RaveDestroy()
