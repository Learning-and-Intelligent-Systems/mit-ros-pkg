#!/usr/bin/env python

from openravepy import *
from numpy import *
from openravepy.databases import inversekinematics
import time, threading
from itertools import izip

import roslib; roslib.load_manifest('openrave_pr2')
import rospy

import actionlib
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from orrosplanning.srv import MoveToHandPositionResponse

def pause(msg=''):
    raw_input(msg+'\npress ENTER to continue...')

def drawConstraintPlane(env,manip,xyzconstraints):
    Tplane = eye(4)
    Tplane[0:3,0:2] = Tplane[0:3,xyzconstraints]
    Tplane[0:3,2] = cross(Tplane[0:3,0],Tplane[0:3,1])
    Tplane[0:3,3] = manip.GetEndEffectorTransform()[0:3,3]
    hplane = env.drawplane(transform=Tplane,extents=[1.0,1.0],texture=reshape([1,1,0.5,0.5],(1,1,4)))
    return hplane

def drawTransform(env,T,length=0.1,linewidth=.01):
    """draws a set of arrows around a coordinate system
    """
    return [env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,0],linewidth=linewidth,color=[1.0,0.0,0.0]),
            env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,1],linewidth=linewidth,color=[0.0,1.0,0.0]),
            env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,2],linewidth=linewidth,color=[0.0,0.0,1.0])]

def angleDistance(angle1,angle2):
    """expects both angle1 and angle2 to be within [-pi,pi] 
    """
    d=angle1-angle2
    if d>pi:
        return d-2*pi
    elif d<-pi:
        return d+2*pi
    else:
        return d

if __name__ == '__main__':
    """ init ROS
    """
    rospy.init_node('or_pr2_pushing_test')
    whicharm = 'l'
    arm_joint_trajectory_action_name = ''+whicharm+'_arm_controller/joint_trajectory_action'
    arm_joint_action_client = actionlib.SimpleActionClient(arm_joint_trajectory_action_name, JointTrajectoryAction)
    print 'waiting for',arm_joint_trajectory_action_name
    print 'have you set ROS_IP?'
    arm_joint_action_client.wait_for_server()
    torso_joint_trajectory_action_name = 'torso_controller/joint_trajectory_action'
    torso_joint_action_client = actionlib.SimpleActionClient(torso_joint_trajectory_action_name, JointTrajectoryAction)
    print 'waiting for',torso_joint_trajectory_action_name
    torso_joint_action_client.wait_for_server()
    print 'done init ROS'

    """ init env
    """        
    env = Environment()
    
    """ viewer
    """
    env.SetViewer('qtcoin')
    Tcamera=matrixFromAxisAngle([0.564935, 0.581771, 0.585142],2.118629)
    Tcamera[:3,3]=[1.277192, 0.174227, 0.732486]
    env.GetViewer().SetCamera(Tcamera)

    """ set up environment
    """
    env.Reset()
    env.Load('data/csail-pr2-table.env.xml')
    #env.Load('robots/pr2-beta-static.zae')#
    #env.Load('robots/pr2mit.dae')
    
    """ get objects handles
    """
    robot=env.GetRobot('pr2')
    robot.SetActiveManipulator('leftarm_torso')
    manip=robot.GetActiveManipulator()
    
    """ set up robot
    """
    #robot.GetJoint('l_shoulder_lift_joint').SetVelocityLimits([1])
    #robot.GetJoint('l_shoulder_lift_joint').SetLimits([-.35],[1.29])
    #robot.GetJoint('l_shoulder_pan_joint').SetLimits([-.56],[2.28])
    
    """ set up manipulation interface
    """
    #basemanip = interfaces.BaseManipulation(robot,plannername='rrt*')
    basemanip = interfaces.BaseManipulation(robot)

    """ joint state
    """
    envlock = threading.Lock()
    def UpdateRobotJoints(msg):
        with envlock:
            with env:
                values = robot.GetDOFValues()
                for name,pos in izip(msg.name,msg.position):
                    j = robot.GetJoint(name)
                    if j is not None:
                        values[j.GetDOFIndex()] = pos
                robot.SetDOFValues(values)
                
    sub = rospy.Subscriber("/joint_states", JointState, UpdateRobotJoints,queue_size=1)

    """ push
    """

    def constrainedMove(Tstart,Tgoal):
        constraintfreedoms=array([0,1,1,1,0,0])
        constraintmatrix=array([[1,0,0,0],
                                [0,1,0,0],
                                [0,0,1,0],
                                [0,0,0,1]])
        constrainterrorthresh=.01
        steplength=0.005
        h=[drawConstraintPlane(env, manip, [0,1]),drawTransform(env,Tgoal)]
        trajs=[]
        trajdata=None
        try:
            trajdata=basemanip.MoveToHandPosition(matrices=[Tgoal],maxiter=10000,maxtries=1,seedik=1,
                                                  constraintfreedoms=constraintfreedoms,
                                                  constraintmatrix=constraintmatrix,
                                                  constrainterrorthresh=constrainterrorthresh,
                                                  steplength=steplength,
                                                  outputtraj=True)
            robot.WaitForController(0)
        except openravepy.planning_error, e:
            pass
        if trajdata is None:
            print 're-plan with a new way point'
            Twaypoint=copy(Tgoal)
            Twaypoint[0,3]=(Tstart[0,3]+Twaypoint[0,3])/2.0
            Twaypoint[1,3]=(Tstart[1,3]+Twaypoint[1,3])/2.0
            trajs=r_[constrainedMove(Tstart, Twaypoint),constrainedMove(Twaypoint,Tgoal)]
        else:
            trajs=[trajdata]
        return trajs
    
    def executeTraj(trajdata):
        #torsogoal = SingleJointPositionGoal()
        torso_goal = JointTrajectoryGoal()
        arm_goal = JointTrajectoryGoal()
        # parse trajectory data into the ROS structure
        tokens = trajdata.split()
        numpoints = int(tokens[0])
        dof = int(tokens[1])
        trajoptions = int(tokens[2])
        numvalues = dof
        offset = 0
        if trajoptions & 4: # timestamps
            numvalues += 1
            offset += 1
        if trajoptions & 8: # base transforms
            numvalues += 7
        if trajoptions & 16: # velocities
            numvalues += dof
        if trajoptions & 32: # torques
            numvalues += dof
        arm_res = MoveToHandPositionResponse()
        torso_res = MoveToHandPositionResponse()
        for i in range(numpoints):
            start = 3+numvalues*i
            torso_pt=JointTrajectoryPoint()
            arm_pt=JointTrajectoryPoint()
            k=0
            for j in robot.GetJoints(manip.GetArmIndices()):
                pos=float(tokens[start+offset+j.GetDOFIndex()])
                if j.GetName()=='torso_lift_joint':
                    if i>0:
                        prevpos=torso_res.traj.points[-1].positions[0]
                        newpos=prevpos+angleDistance(pos,prevpos)
                        torso_pt.positions.append(newpos)
                    else:
                        torso_pt.positions.append(pos)
                else:
                    if i>0:
                        prevpos=arm_res.traj.points[-1].positions[k]
                        newpos=prevpos+angleDistance(pos,prevpos)
                        arm_pt.positions.append(newpos)
                    else:
                        arm_pt.positions.append(pos)
                    k=k+1
            if trajoptions & 4:
                arm_pt.time_from_start = rospy.Duration(float(tokens[start]))
                torso_pt.time_from_start = rospy.Duration(float(tokens[start]))

            arm_res.traj.joint_names =  \
                filter(lambda jn: jn!='torso_lift_joint',
                       [j.GetName() for j in robot.GetJoints(manip.GetArmIndices())])
            arm_res.traj.points.append(arm_pt)

            torso_res.traj.joint_names = ['torso_lift_joint']
            torso_res.traj.points.append(torso_pt)
        arm_goal.trajectory = arm_res.traj
        print 'sending arm JointTrajectoryGoal'
        arm_joint_action_client.send_goal(arm_goal)
        torso_goal.trajectory = torso_res.traj
        print 'sending torso JointTrajectoryGoal'
        torso_joint_action_client.send_goal(torso_goal)
        arm_joint_action_client.wait_for_result()
        torso_joint_action_client.wait_for_result()
        return arm_goal,torso_goal

    def testTraj(trajdata):
        # parse trajectory data into the ROS structure
        tokens = trajdata.split()
        numpoints = int(tokens[0])
        dof = int(tokens[1])
        trajoptions = int(tokens[2])
        numvalues = dof
        offset = 0
        if trajoptions & 4: # timestamps
            numvalues += 1
            offset += 1
        if trajoptions & 8: # base transforms
            numvalues += 7
        if trajoptions & 16: # velocities
            numvalues += dof
        if trajoptions & 32: # torques
            numvalues += dof

        for i in range(numpoints):
            start = 3+numvalues*i
            pos=robot.GetDOFValues()
            for j in robot.GetJoints(manip.GetArmIndices()):
                pos[j.GetDOFIndex()]=float(tokens[start+offset+j.GetDOFIndex()])
            robot.SetDOFValues(pos)
            time.sleep(.1)

    def addRobotShadow():
        newrobot = env.ReadRobotXMLFile(robot.GetXMLFilename())
        newrobot.SetName(robot.GetName())
        for link in newrobot.GetLinks():
            for geom in link.GetGeometries():
                geom.SetTransparency(.8)
        env.AddRobot(newrobot,True)
        newrobot.SetTransform(robot.GetTransform())
        newrobot.SetDOFValues(robot.GetDOFValues())
        newrobot.Enable(False)
        
    BOX_WIDTH=.122
    BOX_HEIGHT=.226
    # bl
#    Tstart=array([[0,0,1,.417],
#                  [0,1,0,.43],
#                  [-1,0,0,.74],
#                  [0,0,0,1]])
    # tl
#    Tstart=array([[0,0,1,.417+BOX_HEIGHT],
#                  [0,1,0,.43],
#                  [-1,0,0,.74],
#                  [0,0,0,1]])
#    Tstart=array([[0,0,1,.6],
#                  [0,1,0,-.2],
#                  [-1,0,0,.73],
#                  [0,0,0,1]])

    traj_file=open('traj1')
    lines=traj_file.readlines()
    traj_file.close()


    O_P_tabletop=array([.43-BOX_WIDTH/2,.417+BOX_HEIGHT/2])
    traj=[[float(x) for x in x.split()] for x in lines[:-2]]
    
    first=True
    for u in traj:
        tabletop_P_contact=array([-u[0],u[1]])
        tabletop_P_contactEnd=tabletop_P_contact+[-u[2],u[3]]
        O_P_contact=O_P_tabletop+tabletop_P_contact
        O_P_contactEnd=O_P_tabletop+tabletop_P_contactEnd
        O_T_contact=array([[0,0,1,0],
                           [0,1,0,0],
                           [-1,0,0,.74],
                           [0,0,0,1]])
        O_T_contact[0,3]=O_P_contact[1]
        O_T_contact[1,3]=O_P_contact[0]
        O_T_contactEnd=array([[0,0,1,0],
                           [0,1,0,0],
                           [-1,0,0,.74],
                           [0,0,0,1]])

        O_T_contactEnd[0,3]=O_P_contactEnd[1]
        O_T_contactEnd[1,3]=O_P_contactEnd[0]
        h=[drawTransform(env,O_T_contact),drawTransform(env,O_T_contactEnd)]
        if first:
            with envlock:
                trajdata=basemanip.MoveToHandPosition([O_T_contact],outputtraj=True)
                robot.WaitForController(0)
            executeTraj(trajdata)
            first=False
            print O_T_contact
            pause()
        else:
            with envlock:
                trajs=constrainedMove(manip.GetEndEffectorTransform(), O_T_contact)
            for traj in trajs:
                executeTraj(traj)
                
        with envlock:
            trajs=constrainedMove(manip.GetEndEffectorTransform(), O_T_contactEnd)
        for traj in trajs:
            executeTraj(traj)
            
        pause()

    pause('plan motion to initial hand position?') # hopefully before press enter, robot pose got updated
    h=drawTransform(env, Tstart)
    with envlock:
        trajdata=basemanip.MoveToHandPosition([Tstart],outputtraj=True)
        robot.WaitForController(0)
        #addRobotShadow() # draw target pose
    #pause('run on robot?')
    executeTraj(trajdata)
    
    pause('plan motion to goal hand position?') # hopefully before press enter, robot pose got updated
    Tgoal=manip.GetEndEffectorTransform()
    
#    for i in xrange(6):
#        Tgoal[1,3]=Tgoal[1,3]-.1
#        with envlock:
#            trajs=constrainedMove(manip.GetEndEffectorTransform(),Tgoal)
#            addRobotShadow() # draw target pose
#        pause('run on robot?')
#        for traj in trajs:
#            pause('run the next segment?')
#            executeTraj(traj)
            
    for i in xrange(2):
        Tgoal[0,3]=Tgoal[0,3]+.1
        with envlock:
            trajs=constrainedMove(manip.GetEndEffectorTransform(),Tgoal)
            #addRobotShadow() # draw target pose
        #pause('run on robot?')
        for traj in trajs:
            #pause('run the next segment?')
            executeTraj(traj)            
    
    for i in xrange(6):
        Tgoal[1,3]=Tgoal[1,3]-.1
        with envlock:
            trajs=constrainedMove(manip.GetEndEffectorTransform(),Tgoal)
            #addRobotShadow() # draw target pose
        #pause('run on robot?')
        for traj in trajs:
            #pause('run the next segment?')
            executeTraj(traj)
    
    for i in xrange(2):
        Tgoal[0,3]=Tgoal[0,3]-.1
        with envlock:
            trajs=constrainedMove(manip.GetEndEffectorTransform(),Tgoal)
            #addRobotShadow() # draw target pose
        #pause('run on robot?')
        for traj in trajs:
            #pause('run the next segment?')
            executeTraj(traj)            
    
    for i in xrange(6):
        Tgoal[1,3]=Tgoal[1,3]+.1
        with envlock:
            trajs=constrainedMove(manip.GetEndEffectorTransform(),Tgoal)
            #addRobotShadow() # draw target pose
        #pause('run on robot?')
        for traj in trajs:
            #pause('run the next segment?')
            executeTraj(traj)

#    trajfile=open('shoulder.traj')
#    lines=trajfile.readlines()
#    trajfile.close()
#    lines=lines[5:]
#    trajdata=""
#    for line in lines:
#        trajdata=trajdata+line
#    arm_goal,torso_goal=executeTraj(trajdata)

#    arm_goal_file=open('shoulderproblem_arm_joint_trajectory_action_goal','w')
#    arm_goal_file.write(str(arm_goal))
#    arm_goal_file.close()
#    torso_goal_file=open('shoulderproblem_torso_joint_trajectory_action_goal','w')
#    torso_goal_file.write(str(torso_goal))
#    torso_goal_file.close()