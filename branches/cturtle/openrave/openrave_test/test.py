#!/usr/bin/env python

from openravepy import *
from numpy import *
from openravepy.databases import inversekinematics
from openravepy.databases import grasping
import time

def pause():
    print 'press ENTER to continue...'
    raw_input()

class ORTest:
    def __init__(self,env,problem):
        self.env = env
        self.robot = self.env.GetRobot('pr2')
        self.problem = problem
        args = 'pr2'
        self.env.LoadProblem(self.problem,args)
        self.leftarm = self.robot.GetManipulator('leftarm')
        self.rightarm = self.robot.GetManipulator('rightarm')
        self.dualsolver = MultiManipIKSolver([self.leftarm,self.rightarm])
        for manip in [self.leftarm, self.rightarm]:
            self.robot.SetActiveManipulator(manip)
            self.ikmodel = inversekinematics.InverseKinematicsModel(self.robot, iktype = IkParameterization.Type.Transform6D)
            if not self.ikmodel.load(): self.ikmodel.autogenerate()
        self.robot.SetActiveManipulator(self.leftarm)

        #self.rc = self.robot.GetController()
        #print self.leftarm.GetArmJoints()+self.rightarm.GetArmJoints()

        #self.vals = self.robot.GetDOFValues()

    def Serialize(self, T):
        return 'goal %s'%(' '.join(str(f) for f in T))

    def MoveArmsToJointPosition(self, T):
        """Moves the two arms to the given joint position T"""
        self.robot.SetActiveDOFs(r_[self.leftarm.GetArmJoints(),self.rightarm.GetArmJoints()])
        success = self.problem.SendCommand('movealljoints '+self.Serialize(T))
        return success is not None

    def MoveJoint(self, dofIndex, val):
        min_val = self.robot.GetDOFLimits()[0][dofIndex]
        max_val = self.robot.GetDOFLimits()[1][dofIndex]
        link_name = self.robot.GetJoints()[dofIndex]
        if val>=min_val and val<=max_val:
            print 'moving link',link_name
            self.robot.SetActiveDOFs([dofIndex])
            success = self.problem.SendCommand('movealljoints '+self.Serialize([val]))
            return success is not None
        else:
            print 'goal value out of bound',[min_val, max_val],'of',link_name
            return False

    def WaitForController(self):
        while not self.robot.GetController().IsDone():
            time.sleep(0.001)

    '''
    def zero_joints(self):
        left_joint_ind = self.robot.GetManipulator('leftarm').GetArmJoints()
        right_joint_ind = self.robot.GetManipulator('rightarm').GetArmJoints()
        self.zero_joint(left_joint_ind)
        self.zero_joint(right_joint_ind)
       
    def zero_joint(self, joint_indices):
        for ji in joint_indices:
            print self.robot.GetJoints()[ji].GetName()
            pause()
            self.vals[ji] = 0
            self.rc.SetDesired(self.vals)
    '''
        
if __name__ == '__main__':
    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    env.Load('data/csail-pr2-table.env.xml')
    problem=RaveCreateProblem(env,'dualmanipulation')
    ot = ORTest(env,problem)
    print 'lifting torso'
    ot.MoveJoint(env.GetRobot('pr2').GetJoint('torso_lift_joint').GetDOFIndex(),.31)
    ot.WaitForController()

    Tlgrasp = mat([[1,0,0,.5],[0,1,0,.2],[0,0,1,1],[0,0,0,1]])
    Trgrasp = mat([[1,0,0,.5],[0,1,0,-.2],[0,0,1,1],[0,0,0,1]])
    print 'finding ik solutions for both arms'
    solutions = ot.dualsolver.findMultiIKSolution(Tgrasps=[array(Tlgrasp),array(Trgrasp)],filteroptions=IkFilterOptions.CheckEnvCollisions)
    print 'moving both arms'
    ot.MoveArmsToJointPosition(r_[solutions])
    ot.WaitForController()
    pause()
    RaveDestroy()
    
'''
from test import *;env = Environment();env.SetViewer('qtcoin')

import test;reload(test);from test import *;env.Reset();env.Load('data/csail-pr2-table.env.xml');ot = ORTest(env)

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

env.Reset();env.Load('data/csail-pr2-table.env.xml')

robot = env.GetRobot('pr2')
manip = robot.GetManipulator('leftarm')
robot.SetActiveDOFs(manip.GetArmJoints())
manip.GetEndEffectorTransform()
basemanip = BaseManipulation(robot)
jvals = robot.GetDOFValues()[manip.GetArmJoints()]
jvals[0]+=.5
basemanip.MoveActiveJoints(jvals)


v=robot.GetDOFValues()
v[47]=.25
rc.SetDesired(v)
gmodel = grasping.GraspingModel(robot,body)
gmodel.autogenerate()

'''
