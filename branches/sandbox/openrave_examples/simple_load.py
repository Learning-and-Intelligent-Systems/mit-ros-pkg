from openravepy import *
from numpy import *



#To get started: 

#load environment
env=Environment()

#if you want to see stuff, load viewer:
env.SetViewer('qtcoin')

#paths can be:
# relative to OPENRAVE_DATA
# relative to CWD
# absolute


#to load files int openrave:
env.Load('robots/pr2-beta-static.robot.xml')


robot=env.GetRobots()[0]
#  --or--  
robot=env.GetRobot('pr2')

#some useful accessors:
#  robot.GetJoints()
#  robot.GetLinks()

#Load another object:
target = env.Load('data/mug2.kinbody.xml')

# to see what is in the environment:
env.GetBodies()

#Bodies are actually called 'kinbodies' so to get one by name:
mug=env.GetKinBody('mug2')

#you can move the object by SetTransform
#   by default, all 6D poses are given with quaternion,position
#   in an array: [ qx, qy, qz, qw, x, y, z]
mug.SetTransform([0, 0, 0, 1, .5,.5,.5])

#you can also set  transforms with matrices:
T1=array([[1,0,0,.5],[0,1,0,.2],[0,0,1,1],[0,0,0,1]])

p=robot.GetActiveManipulator().FindIKSolution(T1,0)

#robot.GetActiveManipulator().GetEndEffectorTransform()

#to teleport the joints:
robot.SetJointValues(p,robot.GetActiveManipulator().GetArmIndices())

