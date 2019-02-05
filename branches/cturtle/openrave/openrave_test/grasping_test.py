#!/usr/bin/env python

from openravepy import *
from numpy import *
from openravepy.databases import inversekinematics,grasping
from openravepy.interfaces import BaseManipulation, TaskManipulation
import time
#from transformations import *

def pause():
    raw_input('press ENTER to continue...')

class GraspPlanning:
    class MyGrasperOptions:
        def __init__(self):
            self.friction = .2
            self.preshapes = ['.548']
            self.graspingnoise = .02
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

    def __init__(self,env):
        self.env = env
        self.env.SetDebugLevel(DebugLevel.Verbose)
        self.collision_checker=self.env.CreateCollisionChecker('pqp')
        self.collision_checker.SetCollisionOptions(CollisionOptions.Distance|CollisionOptions.Contacts)
        self.env.SetCollisionChecker(self.collision_checker)
        self.robot = self.env.GetRobot('pr2')
        self.ikmodel = inversekinematics.InverseKinematicsModel(self.robot,iktype=IkParameterization.Type.Transform6D)
        if not self.ikmodel.load(): 
            self.ikmodel.autogenerate()
        self.basemanip = BaseManipulation(self.robot)
        self.taskmanip = None
        self.updir = array((0,0,1))
        #self.target = self.env.ReadKinBodyXMLFile('models/wg_database/18718.kinbody.xml')
        self.target = self.env.ReadKinBodyXMLFile('data/mug2.kinbody.xml')
        self.env.AddKinBody(self.target)
        #self.target.SetTransform(array(mat([[1,0,0,.7],[0,1,0,0],[0,0,1,.673],[0,0,0,1]])))
        self.target.SetTransform(array(mat([[1,0,0,.7],[0,1,0,0],[0,0,1,.674],[0,0,0,1]])))
        # init robot pose
        self.leftarm = self.robot.GetManipulator('leftarm')
        self.rightarm = self.robot.GetManipulator('rightarm')
        v = self.robot.GetActiveDOFValues()
        v[35] = 3.14/2 # l shoulder pan
        v[56] = -3.14/2 # r shoulder pan
        v[14] = .31 # torso
        v[47] = .54 # l gripper
        self.robot.SetActiveDOFValues(v)
        self.gmodel = grasping.GraspingModel(robot=self.robot,target=self.target)
        if not self.gmodel.load(): self.gmodel.autogenerate(options=self.MyGrasperOptions())
        self.taskmanip = TaskManipulation(self.robot,graspername=self.gmodel.grasper.plannername)
        self.display_handles=[]

    def test(self):
        #{'grasptrans_nocol': [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11], 
        #'igrasptrans': [21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32], 
        #'igrasproll': [33], 'igraspdir': [12, 13, 14], 'igraspstandoff': [15], 
        #'forceclosure': [16], 'igrasppos': [17, 18, 19], 'igrasppreshape': [20]}
        # init state, e
        g = [0]*34
        g[21:33] = [1,0,0,0,1,0,0,0,1,0,0,0.3]
        g[20] = .548
        c = 0
        gbest = copy(g)
        ebest = 0
        k=0
        kmax = 100
        emax = -100
        e = 10000000
        tmax = 1000.0
        tmin = .001
        tfactor = -log( float(tmax) / tmin )
        g=array(g)
        print transpose(reshape(g[21:33],(4,3)))
        self.gmodel.showgrasp(array(g))
        contactsbest=[]
        while k < kmax and e > emax:
            gnew = self.neighbor(g)
            contacts=[]
            mindist = 0
            volume = 0
            try:
                contacts,finalconfig,mindist,volume = self.gmodel.runGraspFromTrans(array(gnew))
            except planning_error:
                pass
            enew = self.energy(contacts,volume)
            t = tmax * exp( tfactor * k / kmax )
            if enew < ebest:
                #self.gmodel.showgrasp(array(g))
                gbest = copy(gnew)
                ebest = enew
                contactsbest=copy(contacts)
                print 'k',k,'e',ebest,mindist
            if enew<e or (exp((e-enew)/t)>random.uniform()):
                g=copy(gnew)
                e=enew
            k=k+1
        print 'k',k,'e',ebest, gbest[30:33]
        self.display_handles.append(self.gmodel.drawContacts(contactsbest))
        self.gmodel.showgrasp(gbest)
    
    def neighbor(self,g):
        '''
        gnew=copy(g)
        for i in [32]:
            t=g[i]+random.uniform(-.1,.1)
            if t<.5 and t> -.5: gnew[i]=t
        '''
        gnew=self.gmodel.grasps[random.randint(0,len(self.gmodel.grasps))]
        #gt = eye(4)
        #gt[0:3,0:4] = transpose(reshape(g[21:33],(4,3)))
        #print gt
        #angle, direction, point = rotation_from_matrix(gt[0:3,0:3])
        #angle = [x+random.uniform(-pi/20,pi/20) for x in angle]
        #r = rotation_matrix(angle,direction,point)
        #gt = dot(r,gt)
        return gnew
    
    def energy(self,contacts,volume):
        #e=-len(contacts)/100.0-1000000000.0*volume
        e=-volume
        return e
    
    def orderGrasps(self):
        print 'ordering grasps by number of contact points'
        contactdists = []
        ab = self.target.ComputeAABB()
        for grasp in self.gmodel.grasps:
            try:
                contacts, finalconfig, mindist, volume = self.gmodel.runGrasp(grasp=grasp, translate=False, forceclosure=False)
                if len(contacts) > 0:
                    contactdists.append(min(sum((contacts[:,0:3]-tile(ab.pos(),(len(contacts),1)))**2,1)))
                else:
                    contactdists.append(inf)
            except planning_error, e:
                contactdists.append(inf)
        order = argsort(array(contactdists))
        self.gmodel.grasps = self.gmodel.grasps[order]

    def openGripper(self,goal=.54,teleport=False):
        if teleport:
            v = self.robot.GetActiveDOFValues()
            v[47] = goal
            self.robot.SetActiveDOFValues(v)
        old_DOF = self.robot.GetActiveDOFIndices()
        new_DOF = self.robot.SetActiveDOFs([47])
        self.basemanip.MoveActiveJoints([goal])
        self.robot.SetActiveDOFs(old_DOF)

    def raiseTorso(self,goal=.31,teleport=False):
        if teleport:
            v = self.robot.GetActiveDOFValues()
            v[14] = goal
            self.robot.SetActiveDOFValues(v)
        old_DOF = self.robot.GetActiveDOFIndices()
        new_DOF = self.robot.SetActiveDOFs([14])
        self.basemanip.MoveActiveJoints([goal])
        self.robot.SetActiveDOFs(old_DOF)

    def waitForController(self):
        while not self.robot.GetController().IsDone():
            time.sleep(.001)
            
    def haha(self,grasps):
        for g in grasps:
            a,b,c,d = self.gmodel.runGrasp(g,forceclosure=True)
            if c > .0001:
                if self.display_handles == []:
                    self.display_handles.append(self.gmodel.drawContacts(a))
                else:
                    self.display_handles[0]=self.gmodel.drawContacts(a)
                self.gmodel.showgrasp(g)
                print len(a),c

if __name__ == '__main__':
    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    env.Load('data/csail-pr2-table.env.xml')
    ot = GraspPlanning(env)
    pause()

'''
from grasping_test import *;env = Environment();env.SetViewer('qtcoin')

import grasping_test;reload(grasping_test);from grasping_test import *;env.Reset();env.Load('data/csail-pr2-table.env.xml');ot = GraspPlanning(env)

import grasping_test;reload(grasping_test);from grasping_test import *;
env.Reset();env.Load('data/csail-pr2-table.env.xml');ot = GraspPlanning(env)

ot.MoveJoint(14,.31);ot.WaitForController()

model_id = 18781
body = env.CreateKinBody()
body.InitFromFile('models/wg_database/'+str(model_id)+'.kinbody.xml')
env.AddKinBody(body)
body.SetTransform(array(mat([[1,0,0,.7],[0,1,0,0],[0,0,1,.673],[0,0,0,1]])))

Tlgrasp = mat([[1,0,0,.5],[0,1,0,.2],[0,0,1,1],[0,0,0,1]])
Trgrasp = mat([[1,0,0,.5],[0,1,0,-.2],[0,0,1,1],[0,0,0,1]])
solutions = ot.dualsolver.findMultiIKSolution(Tgrasps=[array(Tlgrasp),array(Trgrasp)],filteroptions=IkFilterOptions.CheckEnvCollisions)
ot.MoveArmsToJointPosition(r_[solutions])
ot.WaitForController()



body=env.CreateKinBody()

body.InitFromFile('models/wg_database/18638.kinbody.xml')

env.AddKinBody(body)

body.SetTransform(array(mat([[1,0,0,.7],[0,1,0,0],[0,0,1,.673],[0,0,0,1]])))

gmodel=grasping.GraspingModel(robot,body)

taskmanip = TaskManipulation(robot,graspername=gmodel.grasper.plannername)

goals,graspindex,searchtime,trajdata = taskmanip.GraspPlanning(graspindices=gmodel.graspindices,grasps=gmodel.grasps,target=target,approachoffset=.02,maxiter=1000,randomgrasps=True,randomdests=True)

grasps=sorted(ot.gmodel.grasps,key=lambda grasp: grasp[ot.gmodel.graspindices.get('forceclosure')])

contacts,finalconfig,mindist,volume

goals,graspindex,searchtime,trajdata = ot.taskmanip.GraspPlanning(graspindices=ot.gmodel.graspindices,grasps=array([grasps[gi[2]]]),target=ot.target,approachoffset=.02,maxiter=1000,randomgrasps=True,randomdests=True)


'''
