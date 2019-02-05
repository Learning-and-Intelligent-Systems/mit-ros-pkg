#!/usr/bin/env python
from numpy import *
from openravepy import *
import time
 
from openravepy.databases import inversereachability,grasping
 
class GraspReachability:
    def __init__(self,env):
        self.env = env
        self.robot = self.env.GetRobot('pr2')
        self.target = self.env.ReadKinBodyXMLFile('data/mug2.kinbody.xml')
        self.env.AddKinBody(self.target)
        self.target.SetTransform(array(mat([[1,0,0,.7],[0,1,0,0],[0,0,1,.673],[0,0,0,1]])))
        v = self.robot.GetActiveDOFValues()
        v[35] = 3.14/2 # l shoulder pan
        v[56] = -3.14/2 # r shoulder pan
        v[14] = 0 # torso
        v[47] = .54 # l gripper
        self.robot.SetActiveDOFValues(v)
 
        self.gmodel = grasping.GraspingModel(robot=self.robot,target=self.target)
        if not self.gmodel.load():
            print 'go to http://people.csail.mit.edu/liuhuan/pr2/openrave/.openrave/ to get the database for openrave r1741!'
 
        self.irmodel = inversereachability.InverseReachabilityModel(robot=self.robot)
        if not self.irmodel.load():
            print 'go to http://people.csail.mit.edu/liuhuan/pr2/openrave/.openrave/ to get the database for openrave r1741!'
 
    def showBaseDistribution(self,thresh=1.0,zoffset=0.5,**kwargs): # zoffset decides where to draw the point cloud
        starttime = time.time()
        validgrasps,validindices = self.gmodel.computeValidGrasps(checkik=False,backupdist=0.01)
        print 'validgrasps',len(validgrasps)
        print 'validindices',validindices
        def graspiter():
            for grasp,graspindex in izip(validgrasps,validindices):
                #self.gmodel.showgrasp(grasp)
                yield self.gmodel.getGlobalGraspTransform(grasp,collisionfree=True),(self.gmodel,graspindex)
        densityfn,samplerfn,bounds = self.irmodel.computeAggregateBaseDistribution(graspiter(),logllthresh=1.0,**kwargs)
        print 'densityfn',densityfn
        print 'bounds',bounds
        print 'time to build distribution: %fs'%(time.time()-starttime)
        result = inversereachability.InverseReachabilityModel.showBaseDistribution(self.env,densityfn,bounds,zoffset=zoffset,thresh=thresh)
        pause()
        return result
 
def pause():
    raw_input('press ENTER to continue...')
 
if __name__=='__main__':
    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    robot = env.ReadRobotXMLFile('robots/pr2-beta-static.robot.xml')
    env.AddRobot(robot)
 
    gr = GraspReachability(env)
    print gr.showBaseDistribution(zoffset=0)
    pause()