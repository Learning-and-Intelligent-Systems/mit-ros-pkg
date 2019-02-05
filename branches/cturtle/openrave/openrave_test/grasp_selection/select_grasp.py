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

class GripperVisibility:
    """When 'entered' will hide all the non-gripper links in order to facilitate visiblity of the gripper"""
    def __init__(self,manip):
        self.manip = manip
        self.robot = self.manip.GetRobot()
        self.hiddengeoms = []
    def __enter__(self):
        self.hiddengeoms = []
        with self.robot.GetEnv():
            # stop rendering the non-gripper links
            childlinkids = [link.GetIndex() for link in self.manip.GetChildLinks()]
            for link in self.robot.GetLinks():
                if link.GetIndex() not in childlinkids:
                    for geom in link.GetGeometries():
                        self.hiddengeoms.append((geom,geom.IsDraw()))
                        geom.SetDraw(False)
                    link.Enable(False) # disable collision checking
    def __exit__(self,type,value,traceback):
        with self.robot.GetEnv():
            for geom,isdraw in self.hiddengeoms:
                geom.SetDraw(isdraw)
            childlinkids = [link.GetIndex() for link in self.manip.GetChildLinks()]
            for link in self.robot.GetLinks():
                if link.GetIndex() not in childlinkids:
                    link.Enable(True)
                    
class FingertipVisibility:
    def __init__(self,manip,fingertip='right'):
        self.manip = manip
        self.robot = self.manip.GetRobot()
        self.fingertip = self.manip.GetChildLinks()[-2] if fingertip=='left' else self.manip.GetChildLinks()[6]
        self.hiddengeoms = []
    def __enter__(self):
        self.hiddengeoms = []
        with self.robot.GetEnv():
            for link in self.robot.GetLinks():
                if link.GetIndex() != self.fingertip.GetIndex():
                    for geom in link.GetGeometries():
                        self.hiddengeoms.append((geom,geom.IsDraw()))
                        geom.SetDraw(False)
                    link.Enable(False)
                else:
                    for geom in link.GetGeometries():
                        geom.SetTransparency(.9)
    def __exit__(self,type,value,traceback):
        with self.robot.GetEnv():
            for geom,isdraw in self.hiddengeoms:
                geom.SetDraw(isdraw)
            for link in self.robot.GetLinks():
                if link.GetIndex() != self.fingertip.GetIndex():
                    link.Enable(True)

class Grasp:
    def __init__(self,Tgrasp,robot,target,gripper_angle_start = .548,target_mass = 1.0,coefficient_of_friction = .2, effort = .1):
        self.O_T_grasp = Tgrasp
        self.robot = robot
        self.target = target
        self.env = robot.GetEnv()
        self.left_contact_position = None
        self.right_contact_position = None
        self.left_contact_normal = array([0.,-1.,0.])
        self.right_contact_normal = array([0.,1.,0.])
        self.effort = effort
        self.gripper_angle_start = gripper_angle_start
        self.gripper_angle_final = None
        #self.closeUntilContact()
        self.closeAndAlign()
        O_T_leftTip = self.robot.GetLink('l_gripper_l_finger_tip_link').GetTransform()
        self.O_T_leftContact = copy(O_T_leftTip)
        self.O_T_leftContact[:3,3]=self.left_contact_position
        O_T_rightTip = self.robot.GetLink('l_gripper_r_finger_tip_link').GetTransform()
        self.O_T_rightContact = copy(O_T_rightTip)
        self.O_T_rightContact[:3,3]=self.right_contact_position
        self.target_weight = target_mass * 9.8
        self.target_center_of_gravity = self.getGlobalCenterOfGravity(target)
        self.O_T_centerOfGravity = eye(4)
        self.O_T_centerOfGravity[:3,3] = self.target_center_of_gravity
        self.coefficient_of_friction = coefficient_of_friction
        
    def getGlobalCenterOfGravity(self,kinbody):
        # center of axis-aligned bounding box
        return kinbody.ComputeAABB().pos()
        # center of all vertices
#    vertices = kinbody.GetLinks()[0].GetCollisionData().vertices
#    vertex_number = len(vertices)
#    average_vertex_position = [z/vertex_number for z in [sum([pos[y] for pos in vertices]) for y in [0,1,2]]]
#    average_vertex_position.insert(3, 1.)
#    T_P_V = transpose(mat(average_vertex_position))
#    O_T_T = target.GetTransform()
#    O_P_V = O_T_T*T_P_V
#    return array(transpose(O_P_V))[0][:-1]

    def evaluateGrasp(self):
        result = 0
        # make sure both fingertips have contacts
        if isnan(self.left_contact_position[0]) or isnan(self.right_contact_position[0]): 
            return 1000
        
        # check distance between average contacts and center of fingertip
        O_T_leftTip = self.robot.GetLink('l_gripper_l_finger_tip_link').GetTransform()
        LT_T_leftContact = dot(linalg.inv(O_T_leftTip),self.O_T_leftContact)
        O_T_rightTip = self.robot.GetLink('l_gripper_r_finger_tip_link').GetTransform()
        RT_T_rightContact = dot(linalg.inv(O_T_rightTip),self.O_T_rightContact)
        result += 1000*(abs(LT_T_leftContact[0,3]-.01)+abs(LT_T_leftContact[2,3]))
        result += 1000*(abs(RT_T_rightContact[0,3]-.01)+abs(RT_T_rightContact[2,3]))
        
        # calculate torques
        # transforms
        O_T_contactLeft,O_T_contactLeftNormalEnd, O_T_contactLeftFrictionEnd = self.getForces('left')
        O_T_contactRight,O_T_contactRightNormalEnd, O_T_contactRightFrictionEnd = self.getForces('right')
        # distances
        LR = self.getDistanceInXY(O_T_contactLeft, O_T_contactRight)
        GL = self.getDistanceInXY(O_T_contactLeft, self.O_T_centerOfGravity)
        GR = self.getDistanceInXY(O_T_contactRight, self.O_T_centerOfGravity)
        # forces
        Fz_G = -self.target_weight
        Fz_leftNormal = O_T_contactLeftNormalEnd[2,3]-O_T_contactLeft[2,3]
        Fz_leftFriction = O_T_contactLeftFrictionEnd[2,3]-O_T_contactLeft[2,3]
        Fz_rightNormal = O_T_contactRightNormalEnd[2,3]-O_T_contactRight[2,3]
        Fz_rightFriction = O_T_contactRightFrictionEnd[2,3]-O_T_contactRight[2,3]
        Fz_L = Fz_leftNormal+Fz_leftFriction
        Fz_R = Fz_rightNormal+Fz_rightFriction
        # torque around left contact
        T_L = LR*Fz_R+GL*Fz_G
        # torque around right contact
        T_R = LR*Fz_L+GR*Fz_G
        
        print 'score: %.2f'%result,
        print 'pos_delta: l %.2f %.2f r %.2f %.2f'%(1000*abs(LT_T_leftContact[0,3]-.01),1000*abs(LT_T_leftContact[2,3]),1000*abs(RT_T_rightContact[0,3]-.01),1000*abs(RT_T_rightContact[2,3]))
        print 'torque: l %.2f r %.2f'%(T_L,T_R)
        return result
    
    def getForces(self,fingertip='left'):
        if fingertip == 'left':
            LT_T_normalEnd = eye(4)
            LT_T_normalEnd[:3,3] = self.effort*self.left_contact_normal
            O_T_Contact = self.O_T_leftContact
            O_T_normalEnd = dot(O_T_Contact, LT_T_normalEnd)
            LT_T_frictionEnd = eye(4)
            LT_T_frictionEnd[:3,3] = self.coefficient_of_friction*self.effort*array([1,0,0])
            O_T_frictionEnd = dot(O_T_Contact, LT_T_frictionEnd)
        elif fingertip == 'right':
            RT_T_normalEnd = eye(4)
            RT_T_normalEnd[:3,3] = self.effort*self.right_contact_normal
            O_T_Contact = self.O_T_rightContact
            O_T_normalEnd = dot(O_T_Contact, RT_T_normalEnd)
            RT_T_frictionEnd = eye(4)
            RT_T_frictionEnd[:3,3] = self.coefficient_of_friction*self.effort*array([1,0,0])
            O_T_frictionEnd = dot(O_T_Contact, RT_T_frictionEnd)
        return O_T_Contact, O_T_normalEnd, O_T_frictionEnd

    def closeAndAlign(self):
        gripper_angle = self.gripper_angle_start
        self.transformGrasp(self.O_T_grasp, gripper_angle)
        minContactsNum = 50
        ml=0
        mr=0
        while gripper_angle > 0:
            left,right = self.getContactCounts()
            if left>=minContactsNum and right >=minContactsNum:
                self.O_T_grasp = self.robot.GetActiveManipulator().GetEndEffectorTransform()
                break
            if (left < minContactsNum and right < minContactsNum) or (ml>10 and mr>10):
                gripper_angle -= .005
                v = self.robot.GetActiveDOFValues()
                v[47] = gripper_angle
                self.robot.SetActiveDOFValues(v)
                continue
            O_T_R = self.robot.GetTransform()
            O_T_leftTip = self.robot.GetLink('l_gripper_l_finger_tip_link').GetTransform()
            LT_T_R = dot(linalg.inv(O_T_leftTip),O_T_R)
            LT_T_leftTipNew = eye(4)
            if left < minContactsNum:
                # move right
                #LT_T_R[1,3]+=-.0001
                LT_T_leftTipNew[1,3] += .0001
                ml+=1
            else:
                # move left
                #LT_T_R[1,3]+=.0001
                LT_T_leftTipNew[1,3] += -.0001
                mr+=1
            O_T_R = dot(O_T_leftTip,dot(linalg.inv(LT_T_leftTipNew),LT_T_R))
            self.robot.SetTransform(O_T_R)

        self.gripper_angle_final = gripper_angle
        self.left_contact_position = self.getFingerContactPosition('left')
        self.right_contact_position = self.getFingerContactPosition('right')
        print ml,mr
        
#    def closeUntilContact(self):
#        gripper_angle = self.gripper_angle_start
#        self.transformGrasp(self.O_T_grasp, gripper_angle)
#        while gripper_angle > 0:
#            left,right = self.getContactCounts()
#            if left>0 or right>0:
#                break
#            #print gripper_angle
#            #pause()
#            #time.sleep(.01)
#            gripper_angle -= .005
#            v = self.robot.GetActiveDOFValues()
#            v[47] = gripper_angle
#            self.robot.SetActiveDOFValues(v)
#            
#        self.gripper_angle_final = gripper_angle
#        self.left_contact_position = self.getFingerContactPosition('left')
#        self.right_contact_position = self.getFingerContactPosition('right')

    def getContactCounts(self):
        left = -1
        right = -1
        with FingertipVisibility(self.robot.GetActiveManipulator(),'left'):
            report = CollisionReport()
            self.env.CheckCollision(self.robot, report)
            left = len(report.contacts)
        with FingertipVisibility(self.robot.GetActiveManipulator(),'right'):
            report = CollisionReport()
            self.env.CheckCollision(self.robot, report)
            right = len(report.contacts)
        #print left, right
        return [left,right]
        
    def getForce(self,fingertip='left'):
        if fingertip == 'left':
            LT_T_forceEnd = eye(4)
            LT_T_forceEnd[:3,3] = self.effort*self.left_contact_normal
            O_T_Contact = self.O_T_leftContact
            O_T_forceEnd = dot(O_T_Contact, LT_T_forceEnd)
        elif fingertip == 'right':
            RT_T_forceEnd = eye(4)
            RT_T_forceEnd[:3,3] = self.effort*self.right_contact_normal
            O_T_Contact = self.O_T_rightContact
            O_T_forceEnd = dot(O_T_Contact, RT_T_forceEnd)
        return O_T_Contact, O_T_forceEnd

    def getFingerContactPosition(self,fingertip='left'):
        with FingertipVisibility(self.robot.GetActiveManipulator(),fingertip):
            report = CollisionReport()
            self.env.CheckCollision(self.robot, report)
            return self.getAverageContactPosition(report.contacts)
    
    def getAverageContactPosition(self,contacts):
        contact_number = len(contacts)
        contact_positions = [c.pos for c in contacts]
        #contact_normals= [c.norm for c in contacts]
        average_contact_position = [z/contact_number for z in [sum([pos[y] for pos in contact_positions]) for y in [0,1,2]]]
        #average_normal_position = [z/contact_number for z in [sum([norm[y] for norm in contact_normals]) for y in [0,1,2]]]
        return average_contact_position#, average_normal_position

    def transformGrasp(self,Tgrasp,angle=.548):
        """set the robot configuration so that robot.GetActiveManipulator().GetEndEffectorTransform()==Tgrasp
        
        :param Tgrasp: a row-major 4x4 matrix in numpy.array format
        :param angle: gripper angle
        """
        v = self.robot.GetActiveDOFValues()
        v[47] = angle
        self.robot.SetActiveDOFValues(v)
        O_T_R = self.robot.GetTransform() # robot transform R in global frame O 
        O_T_G = self.robot.GetActiveManipulator().GetEndEffectorTransform() # grasping frame G in global frame O
        G_T_O = linalg.inv(O_T_G) # global frame O in grasping frame G
        G_T_R = dot(G_T_O, O_T_R) # robot frame R in grasping frame G
        O_T_G_goal = Tgrasp # final grasping frame G_goal in global frame O 
        O_T_R_goal = dot(O_T_G_goal,G_T_R) # final robot transform R_goal in global frame O                
        self.robot.SetTransform(O_T_R_goal)
        
    def showGrasp(self):
        #time.sleep(.1)
        with GripperVisibility(self.robot.GetActiveManipulator()):
            O_T_R = self.robot.GetTransform()
            v = self.robot.GetActiveDOFValues()
            v[47] = self.gripper_angle_final
            self.robot.SetActiveDOFValues(v)
            self.transformGrasp(self.O_T_grasp, self.gripper_angle_final)
            handles = []
            O_T_contactLeft,O_T_contactLeftEnd = self.getForce('left')
            O_T_contactRight,O_T_contactRightEnd = self.getForce('right')
            handles.append(self.env.drawarrow(p1=O_T_contactLeft[:3,3],p2=O_T_contactLeftEnd[:3,3],linewidth=.001,color=[0,0,0]))
            handles.append(self.env.drawarrow(p1=O_T_contactRight[:3,3],p2=O_T_contactRightEnd[:3,3],linewidth=.001,color=[0,0,0]))
            pause()
            handles = None
            self.robot.SetTransform(O_T_R)

    def getDistanceInXY(self,A,B):
        Pxy_A = A[:2,3]
        Pxy_B = B[:2,3]
        return sqrt(sum([x*x for x in [Pxy_A[i]-Pxy_B[i] for i in [0,1]]])) 

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
    v[47] = angle
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


def pause():
    raw_input('press ENTER to continue...')

def setObjectTransparency(target,transparency=.9):
    for link in target.GetLinks():
        for geom in link.GetGeometries():
            geom.SetTransparency(transparency)

class MyGrasperOptions:
    def __init__(self):
        self.friction = .2
        self.preshapes = ['.548','.2']
        self.graspingnoise = 0 #.02
        self.approachrays = None
        self.standoffs = None
        self.rolls = None
        self.avoidlinks = None
        self.plannername = None
        self.updateenv=False
        self.normalanglerange = 0
        self.directiondelta=0
        self.boxdelta = None
        self.spheredelta = None
        self.useviewer = True


if __name__ == '__main__':
    env = Environment()
    try:
        # sets collision checker to pqp for min distance calculation
        collision_checker = env.CreateCollisionChecker('pqp')
        collision_checker.SetCollisionOptions(CollisionOptions.Contacts)
        env.SetCollisionChecker(collision_checker)
        
        # init target
        target = env.ReadKinBodyXMLFile('data/mug2.kinbody.xml')
        env.AddKinBody(target)
        O_T_Target = array([[1.,0.,0.,1.],
                            [0.,1.,0.,0.],
                            [0.,0.,1.,1.],
                            [0.,0.,0.,1.]])
        target.SetTransform(O_T_Target)
        target_mass = .5
        setObjectTransparency(target)
        env.SetViewer('qtcoin')
        time.sleep(.2)
        
        # init robot
        robot = env.ReadRobotXMLFile('robots/pr2-beta-static.robot.xml')
        env.AddRobot(robot)
        # init robot pose: l_shoulder_pan, r_shoulder_pan, torso, l_gripper
        robot.SetJointValues([pi/2,-pi/2,0.31,0.548],[35,56,14,47])

        # init gmodel
        gmodel = grasping.GraspingModel(robot,target)
        if not gmodel.load(): gmodel.autogenerate(options=MyGrasperOptions())
        
        center_of_gravity = target.ComputeAABB().pos()
        handle = env.drawarrow(p1=center_of_gravity, p2=center_of_gravity+.01*array([0,0,-1]),linewidth=.001,color=[1,0,0])
#        handle=drawTransform(env,robot.GetLink('l_gripper_l_finger_tip_link').GetTransform())
#        pause()
#        handle=drawTransform(env,robot.GetLink('l_gripper_r_finger_tip_link').GetTransform())
#        pause()
#        handle = None
        #{'grasptrans_nocol': [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11], 
        #'igrasptrans': [21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32], 
        #'igrasproll': [33], 'igraspdir': [12, 13, 14], 'igraspstandoff': [15], 
        #'forceclosure': [16], 'igrasppos': [17, 18, 19], 'igrasppreshape': [20]}
        grasps = []
        O_T_R=robot.GetTransform()
        
        i=0
        maxScore = 30
        for grasp in gmodel.grasps:
            i+=1 
            newgrasp = Grasp(gmodel.getGlobalGraspTransform(grasp),robot,target, grasp[20],target_mass,effort=.01,coefficient_of_friction=.2)
            #print newgrasp.evaluateGrasp()
            if newgrasp.evaluateGrasp()<=maxScore:
                print '%d/%d'%(i,len(gmodel.grasps))
                grasps.append(newgrasp)
                newgrasp.showGrasp()
        
        # hand selected grasp 1
        Tgrasp = array ([[0,0,1,1],
                         [0,1,0,.04],#[0,1,0,.0468],
                         [-1,0,0,1.12],
                         [0,0,0,1]])
        #gripperangle =   0.09
        newgrasp = Grasp(Tgrasp,robot,target,target_mass=target_mass,effort=.01)
        newgrasp.showGrasp()
        grasps.append(object)
        # hand selected grasp 2
        Tgrasp = array ([[ -1., 0., 0., 1.1],
                         [ 0., -1., 0., 0.],
                         [ 0., 0., 1., 1.12025630e+00-.03],
                         [ 0., 0., 0., 1.]])
        #gripperangle =   0.09
        newgrasp = Grasp(Tgrasp,robot,target,target_mass=target_mass,effort=.01)
        newgrasp.showGrasp()
        grasps.append(object)
        pause()
        robot.SetTransform(O_T_R)
        handle = None
    except openrave_exception, e:
        print e
    finally:
        # destroy planning environment and clean up
        pass
        #env.Destroy()
        #RaveDestroy()
