from openravepy import __build_doc__
if not __build_doc__:
    from openravepy import *
    from numpy import *
else:
    from openravepy import OpenRAVEModel
    from numpy import inf, array

from utilities import *
from quickhull import *
from plane_fitting import *
from bizzaro_grasp.com_estimation import *

class Grasp:
    def __init__(self,robot,target,Tgrasp,face_pair,gripper_angle_start=.548,target_mass = 1.0,coefficient_of_friction = .2, effort = .1):
        self.O_T_grasp = Tgrasp
        self.face_pair = face_pair
        self.robot = robot
        self.target = target
        self.env = robot.GetEnv()
        self.O_pos_leftContact = None
        self.O_pos_rightContact = None
        self.LT_dir_contactNormal = array([0.,-1.,0.])
        self.RT_dir_contactNormal = array([0.,1.,0.])
        self.effort = effort
        self.gripper_angle_start = gripper_angle_start
        self.gripper_angle_final = None
        #if gripper_angle_start == None:
            #self.closeUntilContact()
            #self.closeAndAlign()
        O_T_R = self.robot.GetTransform()
        self.transformGrasp(self.O_T_grasp, self.gripper_angle_start)
        self.gripper_angle_final = self.gripper_angle_start
        
        self.O_pos_leftContact = self.getFingerContactPosition('left')
        self.O_pos_rightContact = self.getFingerContactPosition('right')
        
        O_T_leftTip = self.robot.GetLink('l_gripper_l_finger_tip_link').GetTransform()
        self.O_T_leftContact = copy(O_T_leftTip)
        self.O_T_leftContact[:3,3]=self.O_pos_leftContact
        O_T_rightTip = self.robot.GetLink('l_gripper_r_finger_tip_link').GetTransform()
        self.O_T_rightContact = copy(O_T_rightTip)
        self.O_T_rightContact[:3,3]=self.O_pos_rightContact
        
        self.target_weight = target_mass * 9.8
        self.target_center_of_mass = self.getGlobalCenterOfMass(target)
        self.O_T_centerOfGravity = eye(4)
        self.O_T_centerOfGravity[:3,3] = self.target_center_of_mass
        self.coefficient_of_friction = coefficient_of_friction
        self.friction_cone_angle = arctan(coefficient_of_friction)
        #self.robot.SetTransform(O_T_R)
        self.score = 1000
        
    def getGlobalCenterOfMass(self,kinbody):
        # center of axis-aligned bounding box
        #return kinbody.ComputeAABB().pos()
        return self.target.GetTransform()[:3,3]+estimate_com(kinbody)

    def evaluateGrasp(self, verbose=False, maxEffort=10.0, coefficient_of_friction=.2):
        result = 0
        # make sure both fingertips have contacts
        if self.O_pos_leftContact==None or self.O_pos_rightContact==None or isnan(self.O_pos_leftContact[0]) or isnan(self.O_pos_rightContact[0]): 
            return 1000
        
#        # check distance between average contacts and center of fingertip
#        O_T_leftTip = self.robot.GetLink('l_gripper_l_finger_tip_link').GetTransform()
#        LT_T_leftContact = dot(linalg.inv(O_T_leftTip),self.O_T_leftContact)
#        O_T_rightTip = self.robot.GetLink('l_gripper_r_finger_tip_link').GetTransform()
#        RT_T_rightContact = dot(linalg.inv(O_T_rightTip),self.O_T_rightContact)
#        #result += 1000*(abs(LT_T_leftContact[0,3]-.01)+abs(LT_T_leftContact[2,3]))
#        #result += 1000*(abs(RT_T_rightContact[0,3]-.01)+abs(RT_T_rightContact[2,3]))
#        
#        # point positions
#        O_pos_G = self.O_T_centerOfGravity[:3,3]
#        O_pos_L = self.O_T_leftContact[:3,3]
#        O_pos_R = self.O_T_rightContact[:3,3]
#        
#        # torque arms
#        LG = O_pos_G-O_pos_L
#        RG = O_pos_G-O_pos_R
#        LR = O_pos_R-O_pos_L
#        RL = O_pos_L-O_pos_R
        
        # forces
        O_P_G = self.O_T_centerOfGravity[:3,3]
        G = array([0,0,-self.target_weight])
        O_P_GEnd = O_P_G+G
        O_T_Contact_L,O_T_forceEnd_L = self.getNormalForce('left',1.0)
        O_T_Contact_R,O_T_forceEnd_R = self.getNormalForce('right',1.0)
        O_P_L = O_T_Contact_L[:3,3]
        O_P_R = O_T_Contact_R[:3,3]
        O_P_LEnd = O_T_forceEnd_L[:3,3]
        O_P_REnd = O_T_forceEnd_R[:3,3]
        O_P_Center = (O_P_L+O_P_R)/2.0
        L = O_P_LEnd-O_P_L
        R = O_P_REnd-O_P_R
        perpendicular_force = dot(G,L)/dot(L,L)*L
        splitting_force = sqrt(dot(perpendicular_force,perpendicular_force))

        if splitting_force<.0001:
            parallel_moment_arm = O_P_Center-O_P_G
        else:
            parallel_moment_arm = dot(O_P_Center-O_P_G,perpendicular_force)/dot(perpendicular_force,perpendicular_force)*perpendicular_force

        parallel_force = G-perpendicular_force
        sliding_force = sqrt(dot(parallel_force,parallel_force))
        if sliding_force<.0001:
            perpendicular_moment_arm = O_P_Center-O_P_G
        else:
            perpendicular_moment_arm = dot(O_P_Center-O_P_G,parallel_force)/dot(parallel_force,parallel_force)*parallel_force
        splitting_moment_arm = sqrt(dot(perpendicular_moment_arm,perpendicular_moment_arm))
        splitting_torque = splitting_force*splitting_moment_arm
        sliding_moment_arm = sqrt(dot(parallel_moment_arm,parallel_moment_arm))
        sliding_torque = sliding_force*sliding_moment_arm
        
        split_counter_effort = splitting_force
        slide_counter_effort = sliding_force/self.coefficient_of_friction 
        
        counter_effort = max(split_counter_effort,slide_counter_effort)
        
        if verbose:
            #print 'G',G
            #print 'L',L
            #print 'R',R
            #print 'perpendicular_force',perpendicular_force
            #print 'parallel_force',parallel_force
            #print 'O_P_Center',O_P_Center
            #print 'O_P_G',O_P_G
            #print 'perpendicular_moment_arm',perpendicular_moment_arm
            #print 'parallel_moment_arm',parallel_moment_arm
            print '%.2f splitting force'% splitting_force
            print '%.2f splitting moment arm'% splitting_moment_arm
            print '%.2f splitting torque'% splitting_torque
            print '%.2f sliding force'% sliding_force
            print '%.2f sliding moment arm'% sliding_moment_arm
            print '%.2f sliding torque'%sliding_torque
            print '%.2f split_counter_effort'%split_counter_effort
            print '%.2f slide counter effort'%slide_counter_effort
        
        result += counter_effort+splitting_torque+sliding_torque
        
        # net torque around left contact
        # LG x G + LR x R
        # net torque around right contact
        # RG x G + RL x L

#        # torque around left contact
#        T_L = LR*Fz_R+GL*Fz_G
#        # torque around right contact
#        T_R = LR*Fz_L+GR*Fz_G
#        # calculate torques
#        # transforms
#        O_T_contactLeft,O_T_contactLeftNormalEnd, O_T_contactLeftFrictionEnd = self.getForces('left')
#        O_T_contactRight,O_T_contactRightNormalEnd, O_T_contactRightFrictionEnd = self.getForces('right')
#
#        # forces
#        Fz_G = -self.target_weight
#        Fz_leftNormal = O_T_contactLeftNormalEnd[2,3]-O_T_contactLeft[2,3]
#        Fz_leftFriction = O_T_contactLeftFrictionEnd[2,3]-O_T_contactLeft[2,3]
#        Fz_rightNormal = O_T_contactRightNormalEnd[2,3]-O_T_contactRight[2,3]
#        Fz_rightFriction = O_T_contactRightFrictionEnd[2,3]-O_T_contactRight[2,3]
#        Fz_L = Fz_leftNormal+Fz_leftFriction
#        Fz_R = Fz_rightNormal+Fz_rightFriction
        
        
        #print 'score: %.2f'%result,
        #print 'pos_delta: l %.2f %.2f r %.2f %.2f'%(1000*abs(LT_T_leftContact[0,3]-.01),1000*abs(LT_T_leftContact[2,3]),1000*abs(RT_T_rightContact[0,3]-.01),1000*abs(RT_T_rightContact[2,3]))
        #print 'torque: l %.2f r %.2f'%(T_L,T_R)
        self.score = result
        return result
        
    def getNormalForce(self,fingertip='left',effort=None):
        if effort == None:
            effort = self.effort
        if fingertip == 'left':
            LT_T_forceEnd = eye(4)
            LT_T_forceEnd[:3,3] = effort*self.LT_dir_contactNormal
            O_T_Contact = self.O_T_leftContact
            O_T_forceEnd = dot(O_T_Contact, LT_T_forceEnd)
        elif fingertip == 'right':
            RT_T_forceEnd = eye(4)
            RT_T_forceEnd[:3,3] = effort*self.RT_dir_contactNormal
            O_T_Contact = self.O_T_rightContact
            O_T_forceEnd = dot(O_T_Contact, RT_T_forceEnd)
        return O_T_Contact, O_T_forceEnd
     
    def getFrictionForce(self,fingertip='left'):
        if fingertip == 'left':
            O_T_Contact = self.O_T_leftContact
            
        elif fingertip == 'right':
            O_T_Contact = self.O_T_rightContact

    def getFingerContactPosition(self,fingertip='left'):
        with FingertipVisibility(self.robot.GetActiveManipulator(),fingertip):
            report = CollisionReport()
            self.env.CheckCollision(self.robot, report)
            if len(report.contacts)==0:
                return None
            #return self.getAverageContactPosition(report.contacts)
            return self.getContactSurfaceCentroid(report.contacts)
    
    def getContactSurfaceCentroid(self,contacts):
        contacts_pos = [tuple(c.pos) for c in contacts]
        contacts_pos = point_projection_on_plane_3d(contacts_pos)
        #handles = []
        #handles.append([self.env.drawarrow(p1=c,p2=array(c)-.002*contacts[0].norm, linewidth=.0001, color=[1,0,0]) for c in contacts_pos])
        verts,triangles = qhull3d(contacts_pos,precision=0.00000000001)
        #norm = report.contacts[0].norm
        #handles.append([self.env.drawarrow(p1=c,p2=c-.002*norm, linewidth=.0001, color=[1,0,0]) for c in verts])
        cx,cy,cz = centroid_3d(verts)
        #handles=[]
        #handles.append(self.env.drawtrimesh(points=array(verts),indices=array(triangles),colors=array(((1,0,0)))))
        #pause()
        #handles=None
        return [round(x,3) for x in [cx,cy,cz]]
    
    def transformGrasp(self,Tgrasp,angle=.548):
        """set the robot configuration so that robot.GetActiveManipulator().GetEndEffectorTransform()==Tgrasp
        
        :param Tgrasp: a row-major 4x4 matrix in numpy.array format
        :param angle: gripper angle
        """
        v = self.robot.GetActiveDOFValues()
        v[self.robot.GetJoint('l_gripper_l_finger_joint').GetDOFIndex()] = angle
        self.robot.SetActiveDOFValues(v)
        O_T_R = self.robot.GetTransform() # robot transform R in global frame O 
        O_T_G = self.robot.GetActiveManipulator().GetEndEffectorTransform() # grasping frame G in global frame O
        G_T_O = linalg.inv(O_T_G) # global frame O in grasping frame G
        G_T_R = dot(G_T_O, O_T_R) # robot frame R in grasping frame G
        O_T_G_goal = Tgrasp # final grasping frame G_goal in global frame O 
        O_T_R_goal = dot(O_T_G_goal,G_T_R) # final robot transform R_goal in global frame O                
        self.robot.SetTransform(O_T_R_goal)
    
    def drawContacts(self,contacts,conelength=0.03,transparency=0.5):
        angs = linspace(0,2*pi,10)
        conepoints = r_[[[0,0,0]],conelength*c_[self.coefficient_of_friction*cos(angs),self.coefficient_of_friction*sin(angs),ones(len(angs))]]
        #print conepoints
        triinds = array(c_[zeros(len(angs)),range(2,1+len(angs))+[1],range(1,1+len(angs))].flatten(),int)
        #print triinds
        allpoints = zeros((0,3))
        for c in contacts:
            #R = rotationMatrixFromQuat(quatRotateDirection(array((0,0,1)),c[3:6]))
            R = rotationMatrixFromQuat(quatRotateDirection(array((0,0,1)),c[3:6]))
            points = dot(conepoints,transpose(R)) + tile(c[0:3],(conepoints.shape[0],1))
            allpoints = r_[allpoints,points[triinds,:]]
        return self.env.drawtrimesh(points=allpoints,indices=None,colors=array((1,0.4,0.4,transparency)))
    
    def show(self,wait=True,moveback=True):
        #time.sleep(.1)
        handles = []
        with GripperVisibility(self.robot.GetActiveManipulator()):
            handles.append(self.env.drawarrow(p1=self.O_T_centerOfGravity[:3,3],
                                              p2=self.O_T_centerOfGravity[:3,3]+array([0,0,-.01])*self.target_weight,
                                              linewidth=.001,color=[0,0,0]))
            
            
            O_T_R = self.robot.GetTransform()
            v = self.robot.GetActiveDOFValues()
            v[self.robot.GetJoint('l_gripper_l_finger_joint').GetDOFIndex()] = self.gripper_angle_final
            self.robot.SetActiveDOFValues(v)
            self.transformGrasp(self.O_T_grasp, self.gripper_angle_final)
            O_T_contactLeft,O_T_contactLeftEnd = self.getNormalForce('left')
            O_T_contactRight,O_T_contactRightEnd = self.getNormalForce('right')
 
            contacts = [r_[self.O_T_leftContact[:3,3],self.O_T_leftContact[:3,3]-O_T_contactLeftEnd[:3,3]],
                        r_[self.O_T_rightContact[:3,3],self.O_T_rightContact[:3,3]-O_T_contactRightEnd[:3,3]]]
            handles.append(self.drawContacts(contacts,conelength=self.effort*.005))
            
            O_P_L = O_T_contactLeft[:3,3]
            O_P_R = O_T_contactRight[:3,3]
            O_P_C = .5*(O_P_L+O_P_R)
            O_P_G = self.O_T_centerOfGravity[:3,3]
            handles.append(self.env.drawarrow(p1=O_P_G, p2=O_P_C, linewidth=.001,color=[0,0,1]))
            
            #handles.append(self.env.drawarrow(p1=O_T_contactLeft[:3,3],p2=O_T_contactLeftEnd[:3,3],linewidth=.001,color=[0,0,0]))
            #handles.append(self.env.drawarrow(p1=O_T_contactRight[:3,3],p2=O_T_contactRightEnd[:3,3],linewidth=.001,color=[0,0,0]))
            
            for finger in ['left','right']:
                with FingertipVisibility(self.robot.GetActiveManipulator(),finger):
                    report = CollisionReport()
                    self.env.CheckCollision(self.robot, report)
                    if len(report.contacts)>0:
                        #handles.append([self.env.drawarrow(p1=c.pos,p2=c.pos-.002*c.norm, linewidth=.0001, color=[1,0,0]) for c in report.contacts])
                        #print report.contacts[0].norm,report.contacts[0].pos
                        contacts = [tuple(c.pos) for c in report.contacts]
                        contacts = point_projection_on_plane_3d(contacts)
                        verts,triangles = qhull3d(contacts,precision=0.00000000001)
                        #norm = report.contacts[0].norm
                        #handles.append([self.env.drawarrow(p1=c,p2=c-.002*norm, linewidth=.0001, color=[1,0,0]) for c in verts])
                        #cx,cy,cz = centroid_3d(verts)
                        #norm = report.contacts[0].norm
                        #c = array([cx,cy,cz])
                        #handles.append([self.env.drawarrow(p1=c,p2=c-.002*norm, linewidth=.0001, color=[1,0,0])])
                        handles.append(self.env.drawtrimesh(points=array(verts),indices=array(triangles),colors=array(((1,0,0)))))
                        
                        #O_T_contact,O_T_contactEnd = self.getNormalForce(finger)
                        #print O_T_contact[:3,3]
                        #pause()
            if wait:
                pause()
                handles = None
            if moveback:
                self.robot.SetTransform(O_T_R)
            if not wait:
                return handles
