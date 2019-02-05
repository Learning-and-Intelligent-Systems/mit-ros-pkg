#!/usr/bin/env python

from numpy import *
from openravepy import *
import time
from simulated_annealing import *

kmax = 100000
alpha = .99995
#alpha=.9999
#gripper_angle_std = .32
#gripper_angle_std = .2
#gripper_angle_std = .16
#gripper_angle_std = .1
#gripper_angle_std = .08
#gripper_angle_std = .04
#gripper_angle_std = .02
gripper_angle_std = .01
#rotation_z_std = pi/1.0
#rotation_z_std = pi/2.0
#rotation_z_std = pi/3.0
#rotation_z_std = pi/4.0
#rotation_z_std = pi/6.0
#rotation_z_std = pi/12.0
#rotation_z_std = pi/24.0
rotation_z_std = pi/48.0
#pos_x_std = .12
#pos_x_std = .1
#pos_x_std = .06
#pos_x_std = .05
#pos_x_std = .03
#pos_x_std = .015
#pos_x_std = .0075
pos_x_std = .00375
#pos_y_std = .12
#pos_y_std = .1
#pos_y_std = .06
#pos_y_std = .05
#pos_y_std = .03
#pos_y_std = .015
#pos_y_std = .0075
pos_y_std = .00375
#pos_z_std = .12
#pos_z_std = .1
#pos_z_std = .06
#pos_z_std = .05
#pos_z_std = .03
#pos_z_std = .015
#pos_z_std = .0075
pos_z_std = .00375
targetfile = 'data/mug2.kinbody.xml'
angle_k = .5
#targetfile = 'data/ketchup.kinbody.xml'
#targetfile = 'data/lego4.kinbody.xml'
emin = .04
O_T_Target = mat([[1,0,0,1],[0,1,0,0],[0,0,1,.674],[0,0,0,1]])

class SimpleGrasp:
    """A simple grasp defined as (x,y,z,theta, gripper_angle)
    theta is defined as rotation around z axis
    """
    def __init__(self,gripper_angle=.548,rotation_z=0.0,pos=[0.0,0.0,0.0]):
        self.gripper_angle = gripper_angle
        self.rotation_z = rotation_z
        self.pos = pos
        
    def getTransform(self):
        """returns grasp transform in global frame"""
        O_T_grasp = matrixFromAxisAngle([0.0,0.0,1.0],self.rotation_z)
        O_T_grasp[:3,3] = self.pos
        return array(O_T_grasp)
    
    def getRobotTransform(self,robot):
        """returns robot transform in global frame"""
        O_T_grasp = self.getTransform()
        O_T_R = mat(robot.GetTransform()) # robot transform R in global frame O 
        O_T_G = mat(robot.GetActiveManipulator().GetEndEffectorTransform()) # grasping frame G in global frame O
        G_T_O = linalg.inv(O_T_G) # global frame O in grasping frame G
        G_T_R = dot(G_T_O, O_T_R) # robot frame R in grasping frame G
        O_T_G_goal = mat(O_T_grasp) # final grasping frame G_goal in global frame O 
        O_T_R_goal = dot(O_T_G_goal,G_T_R) # final robot transform R_goal in global frame O
        return array(O_T_R_goal)

class GraspGenerator:
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

    def __init__(self,env,target):
        self.env = env
        self.robot = env.GetRobots()[0]
        self.target= target
        self.handles = []
        self.drawindex=0
        
        # sets collision checker to pqp for min distance calculation
        collision_checker = self.env.CreateCollisionChecker('pqp')
        collision_checker.SetCollisionOptions(CollisionOptions.Distance|CollisionOptions.Contacts)
        self.env.SetCollisionChecker(collision_checker)
        
        self.left_contact_body = self.env.ReadKinBodyXMLFile('box.kinbody.xml')
        self.left_contact_body.SetName('left_contact_body')
        self.right_contact_body = self.env.ReadKinBodyXMLFile('box.kinbody.xml')
        self.right_contact_body.SetName('right_contact_body')
        self.updateGoalContactsLocations()
        self.env.AddKinBody(self.left_contact_body)
        self.env.AddKinBody(self.right_contact_body)
        
        
        self.sampling_boundary_x_min = 0
        self.sampling_boundary_x_max = 0
        self.sampling_boundary_y_min = 0
        self.sampling_boundary_y_max = 0
        self.sampling_boundary_z_min = 0
        self.sampling_boundary_z_max = 0
        
        self.O_T_PL=self.O_T_PR=self.O_N_PL=self.O_N_PR=self.PL_N_PL=self.PR_N_PR=None
        
    def drawTransform(self,transform,description=None,keep=False):
        """visualizes coordinate frame
        input transform is a 4x4 mat in global frame
        """
        if description is not None:
            print description

        transform=array(transform)            
        phandles = self.handles
        penv = self.env
        start = transform[:3,3]
        dir = mat(transform[:3,:3])
        unit_x = mat([[1],
                      [0],
                      [0]])
        unit_y = mat([[0],
                      [1],
                      [0]])
        unit_z = mat([[0],
                      [0],
                      [1]])
        end_x = start+.02*transpose(dir*unit_x)
        end_y = start+.02*transpose(dir*unit_y)
        end_z = start+.02*transpose(dir*unit_z)
        
        start=start.tolist()
        end_x=array(end_x)[0].tolist()
        end_y=array(end_y)[0].tolist()
        end_z=array(end_z)[0].tolist()
        if len(phandles) <= self.drawindex:
            phandles.append(penv.drawarrow(p1=start,p2=end_x,linewidth=0.001,color=[1.0,0.0,0.0]))
            phandles.append(penv.drawarrow(p1=start,p2=end_y,linewidth=0.001,color=[0.0,1.0,0.0]))
            phandles.append(penv.drawarrow(p1=start,p2=end_z,linewidth=0.001,color=[0.0,0.0,1.0]))
        else:
            phandles[self.drawindex]=penv.drawarrow(p1=start,p2=end_x,linewidth=0.001,color=[1.0,0.0,0.0])
            phandles[self.drawindex+1]=penv.drawarrow(p1=start,p2=end_y,linewidth=0.001,color=[0.0,1.0,0.0])
            phandles[self.drawindex+2]=penv.drawarrow(p1=start,p2=end_z,linewidth=0.001,color=[0.0,0.0,1.0])
        if keep:
            self.drawindex +=3
            
    def showGrasp(self,TGrasp,angle=.548):
        """visualizes a grasp transform
        TGrasp is a 4x4 mat in global frame
        angle is between 0 and .548
        """
        probot = self.robot
        pmanip = probot.GetActiveManipulator()
        v = probot.GetActiveDOFValues()
        v[47] = angle
        probot.SetActiveDOFValues(v)
        with self.GripperVisibility(pmanip):
            O_T_R = mat(probot.GetTransform()) # robot transform R in global frame O 
            O_T_G = mat(pmanip.GetEndEffectorTransform()) # grasping frame G in global frame O
            G_T_O = linalg.inv(O_T_G) # global frame O in grasping frame G
            G_T_R = dot(G_T_O, O_T_R) # robot frame R in grasping frame G
            O_T_G_goal = mat(TGrasp) # final grasping frame G_goal in global frame O 
            O_T_R_goal = dot(O_T_G_goal,G_T_R) # final robot transform R_goal in global frame O
                            
            probot.SetTransform(array(O_T_R_goal))
            self.updateGoalContactsLocations()
            pause()
            probot.SetTransform(array(O_T_R))

    def getPointsOnFingers(self):
        """returns information about two goal contact points on the fingers
        return O_T_PL,O_T_PR,O_N_PL,O_N_PR,PL_N_PL,PR_N_PR
        O_T_PL, O_T_PR: 4x4 numpy mat, transform of left/right contact point on the finger in global frame
        O_N_PL, O_N_PR: list with length 3, global coordinates of the tip of the normal of the goal contact on the fingertip
        PL_N_PL, PR_N_PR: 4x1 numpy mat, normal of goal contact in fingertip contact frame
        
        notation:
        LT, RT: left/right fingertip
        PL, PR: left/right goal contact
        T: transform, 4x4 numpy mat
        NT: tip of the normal , 4x1 numpy mat
        N: normal, list with length 3
        """
        O_T_LT = mat(self.robot.GetLink('l_gripper_l_finger_tip_link').GetTransform())
        LT_T_PL = mat([[0.,1.,0.,.01],
                       [0.,0.,1.,-.015],
                       [1.,0.,0.,0.],
                       [0.,0.,0.,1.]])
        O_T_PL = dot(O_T_LT,LT_T_PL)
        PL_N_PL = mat([[0.],
                       [0.],
                       [1.],
                       [1.]])
        O_NT_PL = dot(O_T_PL, PL_N_PL)
        O_N_PL = transpose(O_NT_PL[:3]-O_T_PL[:3,3]).tolist()[0]
        O_T_RT = mat(self.robot.GetLink('l_gripper_r_finger_tip_link').GetTransform())
        RT_T_PR = mat([[1.,0.,0.,.01],
                       [0.,0.,-1.,.015],
                       [0.,1.,0.,0.],
                       [0.,0.,0.,1.]])
        O_T_PR = dot(O_T_RT,RT_T_PR)
        PR_N_PR = mat([[0.],
                       [0.],
                       [1.],
                       [1.]])
        O_NT_PR = dot(O_T_PR, PR_N_PR)
        O_N_PR = transpose(O_NT_PR[:3]-O_T_PR[:3,3]).tolist()[0]
        return O_T_PL,O_T_PR,O_N_PL,O_N_PR,PL_N_PL,PR_N_PR
        
    def updatePointsOnFingers(self):
        self.O_T_PL,self.O_T_PR,self.O_N_PL,self.O_N_PR,self.PL_N_PL,self.PR_N_PR = self.getPointsOnFingers()
        
    def updateGoalContactsLocations(self):
        """updates the locations of the goal contact points"""
        self.updatePointsOnFingers()
        self.left_contact_body.SetTransform(array(self.O_T_PL))
        self.right_contact_body.SetTransform(array(self.O_T_PR))

    def showGoalContactPointsOnFingers(self):
        """visualizes the goal contacts on the fingers"""
        O_T_PL,O_T_PR,O_N_PL,O_N_PR,PL_N_PL,PR_N_PR=self.getPointsOnFingers()
        start = transpose(O_T_PL[:3,3]).tolist()[0]
        PL_P_PL = PL_N_PL[:]
        PL_P_PL[:3] *= .01
        end = transpose(dot(O_T_PL, PL_P_PL))[:3].tolist()[0]
        self.handles.append(self.env.drawarrow(p1=start,p2=end,linewidth=0.001,color=[.0,.0,.0]))
        pause()
        start = transpose(O_T_PR[:3,3]).tolist()[0]
        PR_P_PR = PR_N_PR[:]
        PR_P_PR[:3] = PR_P_PR[:3]*.01
        end = transpose(dot(O_T_PR, PR_P_PR))[:3].tolist()[0]
        self.handles.append(self.env.drawarrow(p1=start,p2=end,linewidth=0.001,color=[.0,.0,.0]))
        pause()
       
    def getMetricFromFingertipToTarget(self,show=False):
        """returns a metric that measures the differences between the goal contacts on the fingers and the closest points on the object
        return minDists[0]+minDists[1],minAngularDists[0]+minAngularDists[1]
        """
        report = CollisionReport()
        self.robot.Enable(False)
        contacts = []
        minDists = []
        with self.env:
            self.left_contact_body.Enable(True)
            self.right_contact_body.Enable(False)
            check = self.env.CheckCollision(self.left_contact_body, report)
            minDists.append(report.minDistance)
            contacts += report.contacts
            self.left_contact_body.Enable(False)
            self.right_contact_body.Enable(True)
            check = self.env.CheckCollision(self.right_contact_body, report)
            minDists.append(report.minDistance)
            contacts += report.contacts
            
        self.O_N_CL = contacts[0].norm
        self.O_N_CR = contacts[1].norm

        minAngularDists=[normal(self.O_N_PL-self.O_N_CL),normal(self.O_N_PR-self.O_N_CR)]
        #print 'mindist between left_contact_body and target: ', minDists[0],minAngularDists[0]
        #print 'mindist between right_contact_body and target: ', minDists[1],minAngularDists[1]

        if show:
            handles = [self.env.drawlinestrip(points=array((c.pos,c.pos-c.depth*c.norm)), linewidth=3.0, colors=array((1,0,0,1))) for c in contacts]
            handles1 = [self.env.drawarrow(p1=c.pos,p2=c.pos+.01*c.norm,linewidth=.001,color=[0.,0.,0.]) for c in contacts]
            time.sleep(.1)
            #pause()
        
        return minDists[0]+minDists[1],minAngularDists[0]+minAngularDists[1]

    def getSamplingAABB(self):
        """returns an axis-aligned bounding box that defines the cartesian position sampling region
        the gripper is about .1m long, so the sampling region is the AABB of the target plus .1m in each dimension 
        """
        AABB_Target = self.target.ComputeAABB()
        O_P_Target = AABB_Target.pos()
        halfX,halfY,halfZ = AABB_Target.extents()+[.1,.1,.1]
        return AABB(O_P_Target,[halfX,halfY,halfZ])

    def showSamplingRegion(self,keep=False):
        """visualizes the sampling region"""
        AABB_SamplingRegion = self.getSamplingAABB()
        O_P_Target = AABB_SamplingRegion.pos()
        halfX,halfY,halfZ = AABB_SamplingRegion.extents()
        
        O_P_BTL = O_P_Target + [-halfX,-halfY, halfZ]
        O_P_BTR = O_P_Target + [-halfX, halfY, halfZ]
        O_P_BBL = O_P_Target + [-halfX,-halfY,-halfZ]
        O_P_BBR = O_P_Target + [-halfX, halfY,-halfZ]
        O_P_FTL = O_P_Target + [ halfX,-halfY, halfZ]
        O_P_FTR = O_P_Target + [ halfX, halfY, halfZ]
        O_P_FBL = O_P_Target + [ halfX,-halfY,-halfZ]
        O_P_FBR = O_P_Target + [ halfX, halfY,-halfZ]
        
        if keep:
            handles = self.handles
        else:
            handles = []
        handles.append(self.env.drawlinestrip(points=array((O_P_BTL,O_P_BTR)),linewidth = 3.0, colors = array((1,0,0,.5))))
        handles.append(self.env.drawlinestrip(points=array((O_P_BTL,O_P_BBL)),linewidth = 3.0, colors = array((1,0,0,.5))))
        handles.append(self.env.drawlinestrip(points=array((O_P_BTL,O_P_FTL)),linewidth = 3.0, colors = array((1,0,0,.5))))
        handles.append(self.env.drawlinestrip(points=array((O_P_BBR,O_P_BTR)),linewidth = 3.0, colors = array((1,0,0,.5))))
        handles.append(self.env.drawlinestrip(points=array((O_P_BBR,O_P_BBL)),linewidth = 3.0, colors = array((1,0,0,.5))))
        handles.append(self.env.drawlinestrip(points=array((O_P_BBR,O_P_FBR)),linewidth = 3.0, colors = array((1,0,0,.5))))
        handles.append(self.env.drawlinestrip(points=array((O_P_FBL,O_P_FTL)),linewidth = 3.0, colors = array((1,0,0,.5))))
        handles.append(self.env.drawlinestrip(points=array((O_P_FBL,O_P_BBL)),linewidth = 3.0, colors = array((1,0,0,.5))))
        handles.append(self.env.drawlinestrip(points=array((O_P_FBL,O_P_FBR)),linewidth = 3.0, colors = array((1,0,0,.5))))
        handles.append(self.env.drawlinestrip(points=array((O_P_FTR,O_P_FTL)),linewidth = 3.0, colors = array((1,0,0,.5))))
        handles.append(self.env.drawlinestrip(points=array((O_P_FTR,O_P_BTR)),linewidth = 3.0, colors = array((1,0,0,.5))))
        handles.append(self.env.drawlinestrip(points=array((O_P_FTR,O_P_FBR)),linewidth = 3.0, colors = array((1,0,0,.5))))
        
    def clearHandles(self):
        """clear stored drawing handles"""
        self.handles = [None for h in self.handles]


    def random_draw(self):
        """returns a random grasp around the target object"""
        # get random gripper angle
        gripper_angle = random.uniform(low=0.0,high=.548)
        
        # get random rotation along z axis
        rotation_z = random.uniform(low=-pi,high=pi)
        
        # get random grasping point
        AABB_SamplingRegion = self.getSamplingAABB()
        x,y,z = AABB_SamplingRegion.pos()
        halfX,halfY,halfZ = AABB_SamplingRegion.extents()
        self.sampling_boundary_x_min = x-halfX 
        self.sampling_boundary_x_max = x+halfX
        self.sampling_boundary_y_min = y-halfY
        self.sampling_boundary_y_max = y+halfY
        self.sampling_boundary_z_min = z-halfZ
        self.sampling_boundary_z_max = z+halfZ
        pos = [random.uniform(low=x-halfX,high=x+halfX),
               random.uniform(low=y-halfY,high=y+halfY),
               random.uniform(low=z-halfZ,high=z+halfZ)]
        
        # construct random grasp
        grasp = SimpleGrasp(gripper_angle=gripper_angle,rotation_z=rotation_z,pos=pos)
        return grasp
    
    def evaluate_grasp(self,grasp,show=False):
        """evaluates a grasp by summing the min distances and angular differences"""
        # set gripper angle
        v = self.robot.GetActiveDOFValues()
        v[47] = grasp.gripper_angle
        self.robot.SetActiveDOFValues(v)
        # set robot transform
        self.robot.SetTransform(grasp.getRobotTransform(self.robot))

        self.robot.Enable(True)
        childlinkids = [link.GetIndex() for link in self.robot.GetActiveManipulator().GetChildLinks()]
        for link in self.robot.GetLinks():
            if link.GetIndex() not in childlinkids:
                link.Enable(False) # disable collision checking
        r = CollisionReport()
        if self.env.CheckCollision(gg.target,r): return 1000

        
        # update contact points on fingertips
        self.updateGoalContactsLocations()
        # calculate distances
        minDists,minAngDiffs = self.getMetricFromFingertipToTarget(show)
        # calculate score
        score = minDists + angle_k*minAngDiffs
        return score
    
    def random_neighbor(self,grasp):
        """finds a neighbor grasp"""
        # gripper_angle, gaussian with stddev .1rad
        gripper_angle = bound(value=random.normal(grasp.gripper_angle,gripper_angle_std),min=0.0,max=.548)
        # rotation_z, gaussian with stddev pi/4.0 rad
        small_rotation = random.normal(0,rotation_z_std)
        Q_rotation_z_old = quatFromAxisAngle([0.0,0.0,1.0],grasp.rotation_z)
        Q_small_rotation = quatFromAxisAngle([0.0,0.0,1.0],small_rotation)
        Q_rotation_z_new = quatMult(Q_small_rotation, Q_rotation_z_old)
        rotation_z_new = axisAngleFromQuat(Q_rotation_z_new)[2]
        # pos, gaussian with stddev 5cm
        x_old, y_old, z_old = grasp.pos
        x_new = bound(value=random.normal(x_old,pos_x_std),min=self.sampling_boundary_x_min,max=self.sampling_boundary_x_max)
        y_new = bound(value=random.normal(y_old,pos_y_std),min=self.sampling_boundary_y_min,max=self.sampling_boundary_y_max)
        z_new = bound(value=random.normal(z_old,pos_z_std),min=self.sampling_boundary_z_min,max=self.sampling_boundary_z_max)
        pos = [x_new,y_new,z_new]
        # construct neighbor grasp
        neighbor = SimpleGrasp(gripper_angle=gripper_angle,rotation_z=rotation_z_new,pos=pos)
        return neighbor
    
    def findBestGrasp(self):
        sa = SA(random_draw_function=self.random_draw, 
                evaluation_function=self.evaluate_grasp, 
                random_neighbor_function=self.random_neighbor, 
                temperature_function=temperature_function_ecs,
                kmax=kmax,
                emin=emin)
        best_state,best_e,best_k,best_t = sa.run()
        print 'best_e',best_e,'best_k',best_k,'best_t',best_t
        return best_state

def pause():
    raw_input('press ENTER to continue...')
    
def normal(input):
    input = array(input)
    squaresum = reduce(lambda x,y: x+y, map(lambda x: x*x, input.tolist()))
    return sqrt(squaresum)

def bound(value,min,max):
    if value<min: return min
    if value>max: return max
    return value

if __name__ == '__main__':
    try:
        # init env
        env = Environment()
        #env.SetViewer('qtcoin')
        # my dual screen setup
        #env.GetViewer().Move(1920,0)
        #env.GetViewer().SetSize(1400,1050)
        env.Reset()
        robot = env.ReadRobotXMLFile('robots/pr2-beta-static.robot.xml')
        env.AddRobot(robot)
        target = env.ReadKinBodyXMLFile(targetfile)
        env.AddKinBody(target)
    
        # init target pose
        target.SetTransform(array(O_T_Target))
    
        # init robot pose
        v = robot.GetActiveDOFValues()
        v[35] = 3.14/2 # l shoulder pan 
        v[56] = -3.14/2 # r shoulder pan
        v[14] = .31 # torso
        v[47] = .548 # l gripper
        robot.SetActiveDOFValues(v)
    
        gg = GraspGenerator(env,target)
        
#        env.SetViewer('qtcoin')
        # .007
#        best_grasp =mat([[ -9.99994516e-01,   3.30581865e-03,   0.00000000e+00,   1.08248496e+00],
#                         [ -3.30581865e-03,  -9.99994516e-01,   0.00000000e+00,   8.04820447e-04],
#                         [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   1.12025630e+00],
#                         [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])
#        best_gripper_angle = 0.145510784268
        # .061
#        best_grasp =mat([[-0.99998522,  0.00542033,  0.,          1.09135306],
#         [-0.00542033, -0.99998522 , 0. ,         0.01209565],
#         [ 0.  ,        0.  ,        1.  ,        1.08326733],
#         [ 0.    ,      0.   ,       0. ,         1.        ]])
#        best_gripper_angle = 0.473813196913
        # .041
#        best_grasp = mat([[ -9.99897957e-01,  -1.42826075e-02,   0.00000000e+00 ,  1.06640327e+00],
#                          [  1.42826075e-02,  -9.99897957e-01,   0.00000000e+00,  -1.54313573e-04],
#                          [  0.00000000e+00 ,  0.00000000e+00 ,  1.00000000e+00 ,  1.12117219e+00],
#                          [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00  , 1.00000000e+00]])
#        best_gripper_angle = 0.276822079626
        # .086
#        best_grasp = mat([[ -9.96346951e-01,   8.53962526e-02,   0.00000000e+00,   1.08631957e+00],
# [ -8.53962526e-02,  -9.96346951e-01,   0.00000000e+00,   3.12724122e-04],
# [  0.00000000e+00 ,  0.00000000e+00,   1.00000000e+00 ,  1.08322275e+00],
# [  0.00000000e+00 ,  0.00000000e+00 ,  0.00000000e+00 ,  1.00000000e+00]])
#        best_gripper_angle = 0.548
        # .097
#        best_grasp = mat([[-0.99993467, -0.01144018,  0.    ,      1.06581223],
#                         [ 0.01144018, -0.99993467 , 0.   ,       0.0136578 ],
# [ 0.  ,        0.   ,       1.  ,        1.04817796],
# [ 0.  ,        0.   ,       0.   ,       1.        ]])
#        best_gripper_angle=0.548
        # .125
#        best_grasp = mat([[-0.99112093 ,-0.13296299,  0.,          1.08859098],
#                          [ 0.13296299, -0.99112093,  0.  ,        0.00502477],
#                          [ 0.        ,  0.    ,      1.   ,       1.08304656],
#                          [ 0.        ,  0.   ,       0.   ,       1.        ]])
#        best_gripper_angle=0.548
#        # .015
#        best_grasp = mat([[-0.99789608 ,   -0.0648331  ,    0.   ,          1.0962671 ] ,  
#                          [ 0.0648331  ,   -0.99789608  ,   0.    ,        -0.00377738] ,  
#                          [ 0.        ,     0.    ,         1.   ,          1.08621228] ,  
#                          [ 0.        ,     0.      ,       0.    ,         1.        ]] )
#        best_gripper_angle =0.177020318508
#
#        
#        gg.showGrasp(best_grasp, best_gripper_angle)
#        pause()
        with gg.GripperVisibility(gg.robot.GetActiveManipulator()):
            start = time.time()
            #gg.showSamplingRegion(keep=True)
            best=gg.findBestGrasp()
            print 'SA took',time.time()-start
            print best.getTransform()
            #env.SetViewer('qtcoin')
            # my dual screen setup
            #env.GetViewer().Move(1920,0)
            #env.GetViewer().SetSize(1400,1050)
            #print 'showing the best grasp'
            #gg.showGrasp(best.getTransform(), best.gripper_angle)

    except openrave_exception, e:
        print e
    finally:
        #gg.clearHandles()
        print 'asdf'
'''
from grasp_generation_test import *; env = Environment();env.SetViewer('qtcoin');env.Reset();robot = env.ReadRobotXMLFile('robots/pr2-beta-static.robot.xml');env.AddRobot(robot);target = env.ReadKinBodyXMLFile('data/mug2.kinbody.xml');env.AddKinBody(target);O_T_Target = mat([[1,0,0,1],[0,1,0,0],[0,0,1,1],[0,0,0,1]]);target.SetTransform(array(O_T_Target));v = robot.GetActiveDOFValues();v[35] = 3.14/2;v[56] = -3.14/2;v[14] = .31;v[47] = .54;robot.SetActiveDOFValues(v);gg = GraspGenerator(env,target);gg.getMetricFromFingertipToTarget()

gg.clearHandles();env.Reset();import grasp_generation_test;reload(grasp_generation_test); from grasp_generation_test import *;robot = env.ReadRobotXMLFile('robots/pr2-beta-static.robot.xml');env.AddRobot(robot);target = env.ReadKinBodyXMLFile('data/mug2.kinbody.xml');env.AddKinBody(target);O_T_Target = mat([[1,0,0,1],[0,1,0,0],[0,0,1,1],[0,0,0,1]]);target.SetTransform(array(O_T_Target));v = robot.GetActiveDOFValues();v[35] = 3.14/2;v[56] = -3.14/2;v[14] = .31;v[47] = .54;robot.SetActiveDOFValues(v);gg = GraspGenerator(env,target)
'''
