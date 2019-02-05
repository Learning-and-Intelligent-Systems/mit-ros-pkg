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
from itertools import izip
import pickle
import os
from multiprocessing import Process, Manager

from openravepy.databases import grasping
from utilities import *
from bizzaro_grasp.face import Face
from bizzaro_grasp.face_pair import FacePair
from bizzaro_grasp.simple_grasp import SimpleGrasp
from bizzaro_grasp.grasp import Grasp

class BoxGraspGenerator:
    class Sample:
        def __init__(self,position,gripper_direction):
            self.position = position
            self.gripper_direction = gripper_direction

    def __init__(self,robot,target):
        self.env = robot.GetEnv()
        self.robot = robot
        self.target = target
        self.faces = self.getFaces(target)
        self.sorted_faces = self.sortFaces(self.faces)
        self.loadGripperIK()
        self.face_pairs = self.getFacePairs(self.sorted_faces)

    def showFace(self,face):
        body = self.env.CreateKinBody()
        body.SetName('tmp')
        #boxes=array([r_[Target_center_pos, abs(multiply(.5,subtract(mins,maxs)))]])
        boxes = array([r_[face.Target_T_center[:3,3],face.Center_extents]])
        #print boxes,boxes.size
        body.InitFromBoxes(boxes,True)
        body.SetTransform(self.target.GetTransform())
        self.env.AddKinBody(body)
        pause()
        self.env.Remove(body)
        
    def showFacePair(self,face_pair,wait=True,name='facepair'):
        faces_body = self.env.CreateKinBody()
        faces_body.SetName('faces')
        faces = face_pair.faces
        boxes = array([r_[faces[0].Target_T_center[:3,3],faces[0].Center_extents],
                       r_[faces[1].Target_T_center[:3,3],faces[1].Center_extents]])
        faces_body.InitFromBoxes(boxes,True)
        faces_body.SetTransform(self.target.GetTransform())
        self.env.AddKinBody(faces_body)
        body = self.env.CreateKinBody()
        body.SetName(name)
        boxes=array([r_[face_pair.Target_T_center[:3,3], abs(multiply(.5,subtract(face_pair.mins,face_pair.maxs)))]])
        #print boxes,boxes.size
        body.InitFromBoxes(boxes,True)
        body.SetTransform(self.target.GetTransform())
        self.env.AddKinBody(body)
        setObjectTransparency(body, .9)

        if wait:
            pause()
            self.env.Remove(body)
            self.env.Remove(faces_body)
        else:
            return [body,faces_body]

    def showPoints(self,Target_points_pos,global_frame=False):
        body = self.env.CreateKinBody()
        body.SetName('tmp')
        extents = [.001,.001,.001]
        #print r_[points_pos[0],extents]
        boxes=array([r_[pos[:3],extents] for pos in Target_points_pos])
        #print boxes,boxes.size
        body.InitFromBoxes(boxes,True)
        if not global_frame:
            body.SetTransform(self.target.GetTransform())
        self.env.AddKinBody(body)
        pause()
        self.env.Remove(body)
    
    def loadGripperIK(self):
        saved_gripper_ik_filename = 'robot.'+robot.GetRobotStructureHash()+'.ik'

        if os.path.isfile(saved_gripper_ik_filename):
            saved_gripper_ik_file = open(saved_gripper_ik_filename,'r')
            self.distance_to_gripperangle = pickle.load(saved_gripper_ik_file)
            saved_gripper_ik_file.close()
        else:
            print 'generating gripper ik file',saved_gripper_ik_filename
            self.distance_to_gripperangle = self.generateDistanceToGripperangle()
            saved_gripper_ik_file = open(saved_gripper_ik_filename,'w')
            pickle.dump(self.distance_to_gripperangle,saved_gripper_ik_file)
            saved_gripper_ik_file.close()
    
    def generateGrasps(self,face_pairs):
        Target_sample_init_dir = [rotationMatrixFromAxisAngle([0,0,1],pi/2), eye(3),rotationMatrixFromAxisAngle([1,0,0],pi/2)] 
        simple_grasps = []
        trans_sample_delta = .01
        angle_sample_num = 8
    
        for pair in face_pairs:
            faces = pair.faces
            mins = pair.mins
            maxs = pair.maxs
            Target_dir = pair.Target_dir
            
            Target_center_init_pos = [0,0,0]
            Target_init_rot = eye(3)
            for dim in xrange(3):
                if Target_dir[dim]!= 0:
                    Target_center_init_pos[dim] = average([face.Target_T_center[dim,3] for face in faces])
                    Target_init_rot = Target_sample_init_dir[dim]
                else:
                    Target_center_init_pos[dim] = average([mins[dim],maxs[dim]])
            for Target_center_pos in self.getTargetCenterPosSamples(Target_center_init_pos,Target_dir,trans_sample_delta,pair):
                Target_rot = eye(3)
                with GripperVisibility(self.robot.GetActiveManipulator()):            
                    for i in xrange(angle_sample_num):
                        angle = i*2*pi/angle_sample_num
                        if faces[0].Center_dir[0]!=0:
                            Target_rot = rotationMatrixFromAxisAngle([1,0,0],angle)
                        elif faces[0].Center_dir[1]!=0:
                            Target_rot = rotationMatrixFromAxisAngle([0,1,0],angle)
                        elif faces[0].Center_dir[2]!=0:
                            Target_rot = rotationMatrixFromAxisAngle([0,0,1],angle)
                        Target_rot = dot(Target_rot,Target_init_rot)
                        Target_T_center = eye(4)
                        Target_T_center[:3,3] = Target_center_pos
                        Target_T_center[:3,:3] = Target_rot
                        sample = self.Sample(Target_center_pos,Target_rot)
                        Tgrasp = self.getGraspFromSample(sample)
                        gripper_angle = pair.gripper_angle
                        #showGrasp(self.robot,Tgrasp,gripper_angle+.01,wait=False,moveback=False)
                        #report = CollisionReport()
                        #self.env.CheckCollision(self.robot, report)
                        # make sure the initial grasp doesn't collide with the object
                        #if len(report.contacts) > 0:
                            #handles = []
                            #handles.append(drawTransform(env,dot(O_T_Target,Target_T_center)))
                            #handles.append([env.drawlinestrip(points=array((c.pos,c.pos-.01*c.norm)), linewidth=3.0, colors=array((1,0,0,1))) for c in report.contacts])
                            #pause()
                            #handles = None
                            #continue
                        showGrasp(self.robot,Tgrasp,gripper_angle,wait=False,moveback=False)
                        # make sure the contacts are only on the fingertips
                        if not self.checkContactsOnFingertipsOnly(Tgrasp,gripper_angle): 
                            continue
                        grasp =SimpleGrasp(Tgrasp,gripper_angle)
                        grasp.face_pair = pair
                        #self.showFacePair(pair)
                        simple_grasps.append(grasp)
        return simple_grasps
    
    def checkContactsOnFingertipsOnly(self, Tgrasp,gripper_angle):
        with FingertipVisibility(self.robot.GetActiveManipulator(),'both',True):
            report = CollisionReport()
            self.env.CheckCollision(self.robot, report)
            if len(report.contacts)>0:
                #handles = []
                #handles.append([env.drawarrow(p1=c.pos,p2=c.pos-.002*c.norm, linewidth=.0005, color=[1,0,0]) for c in report.contacts])                
                #pause()
                #handles = None
                return False        
        
        #print 'checking contacts on fingertips'
        for fingertip in ['left','right']:
            with FingertipVisibility(self.robot.GetActiveManipulator(),fingertip):
                O_T_tip = None
                if fingertip == 'left':
                    O_T_tip = self.robot.GetLink('l_gripper_l_finger_tip_link').GetTransform()
                    Tip_center = [.0135, -.018, 0]
                elif fingertip == 'right':
                    O_T_tip = self.robot.GetLink('l_gripper_r_finger_tip_link').GetTransform()
                    Tip_center = [.0135, .018, 0]
    
                Tip_T_O = linalg.inv(O_T_tip)
                TipCenter_extents = [.018,.0041,.012]
                Tip_mins = subtract(Tip_center,TipCenter_extents)
                Tip_maxs = add(Tip_center,TipCenter_extents)
                Tip_mins,Tip_maxs = Tip_mins[:3],Tip_maxs[:3]
                for i in xrange(3):
                    if Tip_mins[i]>Tip_maxs[i]:
                        Tip_mins[i],Tip_maxs[i] =Tip_maxs[i],Tip_mins[i]
                
                report = CollisionReport()
                self.env.CheckCollision(self.robot, report)
                #tmpbody = self.env.CreateKinBody()
                #tmpbody.SetName('tmp_tip')
                #tmpbody.InitFromBoxes(array([r_[Tip_center,TipCenter_extents]]),True)
                #tmpbody.SetTransform(O_T_tip)
                #setObjectTransparency(tmpbody)
                #self.env.AddKinBody(tmpbody)
                for contact in report.contacts:
                    Tip_pos = dot(Tip_T_O,r_[contact.pos,1])
                    Tip_pos = Tip_pos[:3]
                    if (Tip_pos>Tip_mins).all() and (Tip_pos<Tip_maxs).all():
                        #print 'contact within valid region'
                        continue
                    else:
                        #print 'contact out of valid region'
                        #print 'Tip_mins',Tip_mins
                        #print 'Tip_maxs',Tip_maxs
                        #print 'Tip_pos',Tip_pos
                        #handles = []
                        #handles.append([env.drawarrow(p1=c.pos,p2=c.pos-.002*c.norm, linewidth=.0005, color=[1,0,0]) for c in [contact]])
                        #pause()
                        #handles = None
                        #self.env.Remove(tmpbody)
                        return False
            #self.env.Remove(tmpbody)
        
        #pause()
        return True
    
    def getTargetCenterPosSamples(self,Target_center_init_pos,Target_dir,trans_sample_delta,pair):
        pair_mins = pair.mins
        pair_maxs = pair.maxs
        dims = []
        for dim in xrange(3):
            if Target_dir[dim]==0:
                dims.append(dim)
                
        mins = [Target_center_init_pos[0],Target_center_init_pos[1],Target_center_init_pos[2]]
        maxs = [Target_center_init_pos[0],Target_center_init_pos[1],Target_center_init_pos[2]]
        for dim in dims:
            while mins[dim] > pair_mins[dim]:
                mins[dim]-=trans_sample_delta
            mins[dim]+=trans_sample_delta
            while maxs[dim] < pair_maxs[dim]:
                maxs[dim]+=trans_sample_delta
            maxs[dim]-=trans_sample_delta
        num_samples= multiply(subtract(maxs,mins),int(1/trans_sample_delta))
        #print 'num_samples',num_samples
        #print 'mins',mins
        #print 'input',Target_center_init_pos
        samples = []
        dim0 = mins[dims[0]]
        while dim0<=maxs[dims[0]]:
            dim1 = mins[dims[1]]
            while dim1<=maxs[dims[1]]:
                sample = [Target_center_init_pos[0],Target_center_init_pos[1],Target_center_init_pos[2]]
                sample[dims[0]]=dim0
                sample[dims[1]]=dim1
                samples.append(sample)
                dim1+=trans_sample_delta
            dim0+=trans_sample_delta
        #bodies=self.showFacePair(pair,False)
        #self.showPoints(samples)
        #[self.env.Remove(body) for body in bodies]
        return samples
            
    def getNonTipContactCounts(self):
        with FingertipVisibility(self.robot.GetActiveManipulator(),'both',True):
            report = CollisionReport()
            self.env.CheckCollision(self.robot, report)
            return report.contacts 

    def getContactCounts(self):
        with GripperVisibility(self.robot.GetActiveManipulator()):
            report = CollisionReport()
            self.env.CheckCollision(self.robot, report)
            return report.contacts             

    def getGraspFromSample(self,sample):
        Target_T_grasp = eye(4)
        Target_T_grasp[:3,3] = sample.position
        Target_T_grasp[:3,:3] = sample.gripper_direction[:3,:3]
        O_T_grasp = dot(self.target.GetTransform(),Target_T_grasp)
        return O_T_grasp
        
    def generateDistanceToGripperangle(self):
        distance_to_gripperangle = {}
        angle = 0
        LT_pos_contact = [.01,-.015,0.]
        LT_T_contact = eye(4)
        LT_T_contact[:3,3] = LT_pos_contact
        RT_pos_contact = [.01,.015,0.]
        RT_T_contact = eye(4)
        RT_T_contact[:3,3] = RT_pos_contact
        
        while angle <= .548:
            O_T_leftTip = self.robot.GetLink('l_gripper_l_finger_tip_link').GetTransform()
            O_T_leftTipContact = dot(O_T_leftTip,LT_T_contact)
            #handle = drawTransform(self.env,O_T_leftTip)
            #pause()
            O_T_rightTip = self.robot.GetLink('l_gripper_r_finger_tip_link').GetTransform()
            O_T_rightTipContact = dot(O_T_rightTip,RT_T_contact)
            #handle = drawTransform(self.env,O_T_rightTip)
            #pause()
            #handle = None
            xl,yl,zl = O_T_leftTipContact[:3,3]
            xr,yr,zr = O_T_rightTipContact[:3,3]
            distance_to_gripperangle['%.6f'%sqrt(sum([x*x for x in [xl-xr,yl-yr,zl-zr]]))] = angle  
            angle += .000001
            v = self.robot.GetActiveDOFValues()
            v[self.robot.GetJoint('l_gripper_l_finger_joint').GetDOFIndex()] = angle
            self.robot.SetActiveDOFValues(v)
        return distance_to_gripperangle    
        
    def getGripperAngleFromDistance(self,distance):
        if distance>.0969 or distance<0:
            return -1
        key = '%.6f'%distance
        while not self.distance_to_gripperangle.has_key(key):
            distance+=.000001
            key = '%.6f'%distance
        return self.distance_to_gripperangle[key]

    def getFacePairs(self,sorted_faces):
        face_pairs = []
        Target_sample_init_dir = [rotationMatrixFromAxisAngle([0,0,1],pi/2), eye(3),rotationMatrixFromAxisAngle([1,0,0],pi/2)]
        num_faces = [len(x) for x in sorted_faces]
        for i in xrange(3):
            faces1 = sorted_faces[i*2]
            faces2 = sorted_faces[i*2+1]
            for j in xrange(num_faces[i*2]):
                face1 = faces1[j]                    
                for k in xrange(num_faces[i*2+1]):
                    face2 = faces2[k]

                    faces = [face1,face2]
                    Target_T_centers = [face.Target_T_center for face in faces]
                    Target_dir = dot(face1.Target_T_center[:3,:3],face1.Center_dir)                    
                    dims = []
                    for dim in xrange(3):
                        if Target_dir[dim]==0:
                            dims.append(dim)
                        else:
                            distance = abs(Target_T_centers[0][dim][3]-Target_T_centers[1][dim][3])

                    # make sure two faces are at least .5cm apart
                    if distance < 0.005:
                        continue
                    
                    gripper_angle = self.getGripperAngleFromDistance(distance)
                    # make sure there exists ik solution for the gripper
                    if gripper_angle == -1:
                        continue
                    
                    Target_extents = [dot(face.Target_T_center[:3,:3],face.Center_extents) for face in faces]
                    Target_boundaries_sets = [[[center[dim,3]-extent[dim],center[dim,3]+extent[dim]] for dim in xrange(3)] for center,extent in izip(Target_T_centers,Target_extents)]
                    min_sets = [[boundaries_set[dim][0] for dim in xrange(3)] for boundaries_set in Target_boundaries_sets] 
                    max_sets = [[boundaries_set[dim][1] for dim in xrange(3)] for boundaries_set in Target_boundaries_sets]

                    min_greater_than_max = False
                    
                    for dim in dims:
                        if min_greater_than_max:
                            break
                        for i in xrange(2):
                            if min_greater_than_max:
                                break
                            min_value=min_sets[i][dim]
                            for j in xrange(2):
                                max_value=max_sets[j][dim]
                                if min_value+.001>=max_value:
                                    min_greater_than_max = True
                                    break
                    # make sure the projection of the faces overlap
                    if min_greater_than_max:
                        continue
                    
                    mins = [max([min_set[dim] for min_set in min_sets]) for dim in xrange(3)]
                    maxs = [min([max_set[dim] for max_set in max_sets]) for dim in xrange(3)]
                    
                    #if (abs(subtract(maxs,mins))<0.01).any():
                    #    continue
                    Target_center_pos = [0,0,0]
                    Target_init_rot = eye(3)
                    for dim in xrange(3):
                        if faces[0].Center_dir[dim]!= 0:
                            Target_center_pos[dim] = average([face.Target_T_center[dim,3] for face in faces])
                            Target_init_rot = Target_sample_init_dir[dim]
                        else:
                            Target_center_pos[dim] = average([mins[dim],maxs[dim]])
                    Target_T_center = eye(4)
                    Target_T_center[:3,:3] = Target_init_rot
                    Target_T_center[:3,3] = Target_center_pos

                    face_pair = FacePair(face1,face2)
                    face_pair.gripper_angle = gripper_angle
                    face_pair.mins = mins
                    face_pair.maxs = maxs
                    face_pair.Target_T_center = Target_T_center
                    face_pair.Target_dir = Target_dir
                    #self.showFacePair(face_pair)

                    face_pairs.append(face_pair)
                        
        return face_pairs
  
    def sortFaces(self,faces):
        sorted_faces = [[],[],[],[],[],[]]
        for face in faces:
            Target_dir = dot(face.Target_T_center[:3,:3],face.Center_dir)
            if (Target_dir==[1,0,0]).all():
                #self.showFace(face)
                sorted_faces[0].append(face)
            elif (Target_dir==[-1,0,0]).all():
                #self.showFace(face)
                sorted_faces[1].append(face)
            elif (Target_dir==[0,1,0]).all():
                #self.showFace(face)
                sorted_faces[2].append(face)
            elif (Target_dir==[0,-1,0]).all():
                #self.showFace(face)
                sorted_faces[3].append(face)
            elif (Target_dir==[0,0,1]).all():
                #self.showFace(face)
                sorted_faces[4].append(face)
            elif (Target_dir==[0,0,-1]).all():
                #self.showFace(face)
                sorted_faces[5].append(face)
        return sorted_faces
    
    def getFaces(self,target):
        faces = []
        #O_T_Target = self.target.GetTransform()
        Center_pos_faces = [None]*6
        Center_dir_faces = [None]*6
        Center_dir_faces[0] = [1,0,0]
        Center_dir_faces[1] = [-1,0,0]
        Center_dir_faces[2] = [0,1,0]
        Center_dir_faces[3] = [0,-1,0]
        Center_dir_faces[4] = [0,0,1]
        Center_dir_faces[5] = [0,0,-1]
        Center_extents_faces = [None]*6

        for box in target.GetLinks()[0].GetGeometries():
            Target_T_Center = box.GetTransform()
            x,y,z = box.GetBoxExtents()
            Center_pos_faces[0] = [x,0,0]
            Center_pos_faces[1] = [-x,0,0]
            Center_pos_faces[2] = [0,y,0]
            Center_pos_faces[3] = [0,-y,0]
            Center_pos_faces[4] = [0,0,z]
            Center_pos_faces[5] = [0,0,-z]
            Center_extents_faces[0] = [0,y,z]
            Center_extents_faces[1] = [0,y,z]
            Center_extents_faces[2] = [x,0,z]
            Center_extents_faces[3] = [x,0,z]
            Center_extents_faces[4] = [x,y,0]
            Center_extents_faces[5] = [x,y,0]
            Center_T_faces = [eye(4)]*6
            for Center_T_face,Center_pos_face,Center_dir_face,Center_extents_face in izip(Center_T_faces,Center_pos_faces,Center_dir_faces,Center_extents_faces):
                Center_T_face[:3,3] = Center_pos_face
                Target_T_face = dot(Target_T_Center,Center_T_face)
                #O_T_face = dot(O_T_Target,Target_T_face)
                faces.append(Face(Target_T_face,Center_dir_face,Center_extents_face))
        return faces

if __name__=='__main__':
    env = Environment()
    try:
        # sets collision checker to pqp for min distance calculation
        collision_checker = RaveCreateCollisionChecker(env,'pqp')
        collision_checker.SetCollisionOptions(CollisionOptions.Contacts)
        env.SetCollisionChecker(collision_checker)
        
        # init target
        target = env.ReadKinBodyXMLFile('bizzaro_objects/bizzaro_mug_small.kinbody.xml')
        #target = env.ReadKinBodyXMLFile('bizzaro_objects/bizzaro_block.kinbody.xml')
        O_T_Target = array([[1.,0.,0.,1.],
                            [0.,1.,0.,0.],
                            [0.,0.,1.,1.],
                            [0.,0.,0.,1.]])
        target.SetTransform(O_T_Target)
        env.AddKinBody(target)
        setObjectTransparency(target)
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
            if len(contacts)<50:
                continue
            #print len(contacts)
            #pause()
            newgrasp = Grasp(robot,target,gmodel.getGlobalGraspTransform(g),None,
                             finalconfig[0][robot.GetActiveManipulator().GetGripperJoints()[0]],target_mass,effort=10,coefficient_of_friction=.2)
            #handle = newgrasp.show(wait=False,moveback=False)
            if newgrasp.evaluateGrasp() < maxScore:
                or_grasps.append(newgrasp)

            #print grasp.getContactCounts()
            #time.sleep(.2)
            #pause()
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

        gg = BoxGraspGenerator(robot,target)
        
        saved_grasps_filename = 'target.'+target.GetKinematicsGeometryHash()+'.robot.'+robot.GetRobotStructureHash()+'.grasps'

        if os.path.isfile(saved_grasps_filename):
            print 'loading grasps from',saved_grasps_filename
            saved_grasps_file = open(saved_grasps_filename,'r')
            simple_grasps = pickle.load(saved_grasps_file)
            saved_grasps_file.close()
        else:
            print 'generating grasps and save to',saved_grasps_filename
            pause()
            simple_grasps = gg.generateGrasps(gg.face_pairs)
            saved_grasps_file = open(saved_grasps_filename,'w')
            pickle.dump(simple_grasps,saved_grasps_file)
            saved_grasps_file.close()
        print 'found %d good grasps'%len(simple_grasps)
        grasps = []
        i=0
        #env.SetViewer('')
        print 'evaluating grasps'

#        def evalGrasps(simple_grasps,good_grasps):
#            i=0
#            for grasp in simple_grasps:
#                i+=1
#                newgrasp = Grasp(robot,target,grasp.Tgrasp, grasp.face_pair,grasp.gripper_angle,target_mass,effort=.01,coefficient_of_friction=.2)
#                if newgrasp.evaluateGrasp()<=maxScore:
#                    good_grasps.append(i)
#                if i%50==0:
#                    print '.',
#            return good_grasps
#        
#        manager = Manager()
#        good_grasps = manager.list([])
#        num_processes = 2
#        processes = []
#        start=time.time()
#        step = len(simple_grasps)/num_processes
#        while num_processes*step<len(simple_grasps):
#            step+=1
#        for i in xrange(num_processes):
#            print i*step,(i+1)*step
#            processes.append(Process(target=evalGrasps,
#                                     args=(simple_grasps[i*step:(i+1)*step],good_grasps)))
#        for process in processes:
#            process.start()
#            
#        for process in processes:
#            process.join()
#        
#        print len(good_grasps)
#
#        pause()

        for grasp in simple_grasps:
            i+=1
            #print 'evaluating grasp %d/%d'%(i,len(simple_grasps))
            newgrasp = Grasp(robot,target,grasp.Tgrasp, grasp.face_pair,
                             grasp.gripper_angle,target_mass,effort=10,coefficient_of_friction=.2)

            if newgrasp.evaluateGrasp()<=maxScore:
                #print '%d/%d'%(i,len(simple_grasps))
                grasps.append(newgrasp)
                #handle = newgrasp.show(wait=False,moveback=False)
                #pause()
                #handle = None
                #newgrasp.showGrasp()
            else:
                #print 'showing bad grasp'
                #handle = newgrasp.show(wait=False,moveback=False)
                #pause()
                #handle = None
                pass
            if i%100==0:
                print '%d/%d'%(i,len(simple_grasps))



        print 'found %d better grasps'%len(grasps)
        i=0
        #env.SetViewer('qtcoin')
        grasps = sorted(grasps,key=lambda g: g.score)
        with GripperVisibility(robot.GetActiveManipulator()):
            for grasp in grasps:
                i+=1
                print 'showing grasp %d/%d with score %.2f'%(i,len(grasps),grasp.score)
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
