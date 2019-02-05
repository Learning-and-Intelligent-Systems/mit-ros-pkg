#!/usr/bin/env python
import roslib; roslib.load_manifest('pushing_test')
import rospy

import openravepy
from openravepy import *
from numpy import *
from openravepy.databases import inversekinematics

import sys
import os
import signal
import threading
import time
import select
import socket

from itertools import izip

import actionlib
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, PointHeadAction, PointHeadGoal, Pr2GripperCommandAction, Pr2GripperCommandGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from orrosplanning.srv import MoveToHandPositionResponse
from furniture.msg import Table_Poses
from furniture.msg import Table_Polygons

class SocketServer(threading.Thread):
    def __init__(self, host, port, backlog):
        threading.Thread.__init__(self)
        self.host = host
        self.port = port
        self.backlog = backlog
        self.client = None
        self.address = None
        self.isSocketOpen = False
        self.server = None
        
        self.setDaemon(True)
        self.start()

    def run(self):
        running = 1
        while running:
            while not self.isSocketOpen:
                try:
                    #print 'trying to open socket'
                    self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.server.bind((self.host, self.port))
                    self.server.listen(backlog)
                    self.isSocketOpen = True
                    #print 'server inited'
                except socket.error, (value, message):
                    if self.server:
                        self.server.close()
                    print 'box state publisher socket server initialization caught exception', value, message
                    print 'retrying'
                    rospy.sleep(1.0)

            while not self.client:
                print 'box state publisher socket server waiting for client'
                inputs = [self.server, sys.stdin] # server.accept() blocks, use sys.stdin to break
                
                inputready, outputready, exceptionready = select.select(inputs, [], [])
                for s in inputready:
                    if s == self.server:
                        client, address = self.server.accept()
                        self.client = client
                        self.address = address
                        print 'box state publisher socket server accepted client', client, address
                    
class SocketClient(threading.Thread):
    def __init__(self, host, port,size):
        threading.Thread.__init__(self)
        self.host = host
        self.port = port
        self.size = size
        self.isSocketOpen = False
        self.setDaemon(True)
        self.start()

    def run(self):
        running = 1
        control_msg = ''
        while running:
            while not self.isSocketOpen:
                try:
                    #print 'trying to open socket'
                    self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.server.connect((self.host, self.port))
                    self.isSocketOpen = True
                    #print 'client inited'
                except socket.error, (value, message):
                    if self.server:
                        self.server.close()
                    if value!=111: # connection refused error
                        print 'robot controller socket client caught exception', value, message
                        print 'retrying'
                    rospy.sleep(1.0)
            try:
                piece = self.server.recv(self.size)
                while len(piece)>0:
                    if piece.find('\n')>-1:
                        control_msg += piece.split('\n')[0]
                        print 'received control message',control_msg.split()
                        decoded_control_msg = [float(x) for x in control_msg.split()]
                        u = decoded_control_msg[:4]
                        q = decoded_control_msg[4:]
                        pushBox(u,q)
                        control_msg = ''
                        piece = piece[piece.find('\n')+1:]
                    else:
                        control_msg += piece
                        piece = ''
                    if control_msg.find('end')>-1:
                        control_msg = ''
                        self.server.close()
                        self.isSocketOpen=False
            except socket.error, (value, message):
                if self.server:
                    self.server.close()
                    self.isSocketOpen = False
                print 'robot controller socket client caught exception', value, message

def angleDistance(angle1, angle2):
    d = angle1 - angle2
    if d > pi:
        return d - 2 * pi
    elif d < -pi:
        return d + 2 * pi
    else:
        return d

def constrainedMove(Tstart, Tgoal):
    constraintfreedoms = array([0,1,1,1,0,0]) # in manipulator frame, can translate in x,y,z, rotate around x
    constraintmatrix = array([[1,0,0,0],
                              [0,1,0,0],
                              [0,0,1,0],
                              [0,0,0,1]])
    constrainterrorthresh = .01
    h = [drawConstraintPlane([0,1])]#, drawTransform(Tgoal)]
    traj = []
    trajdata = None
    try:
        trajdata = basemanip.MoveToHandPosition(
            matrices = [Tgoal],
            #maxiter = 100,
            #maxtries = 1,
            #seedik = 1,
            constraintfreedoms = constraintfreedoms,
            constraintmatrix =  constraintmatrix,
            constrainterrorthresh = constrainterrorthresh,
            #steplength = .005,
            outputtraj = True)
        robot.WaitForController(0)
    except openravepy.planning_error, e:
        print e
    if trajdata is None:
        print 'replan with a new way point'
        Twaypoint = copy(Tgoal)
        Twaypoint[0,3] = (Tstart[0,3] + Twaypoint[0,3]) / 2.0
        Twaypoint[1,3] = (Tstart[1,3] + Twaypoint[1,3]) / 2.0
        trajs = r_[constrainedMove(Tstart, Twaypoint), 
                   constrainedMove(Twaypoint, Tgoal)]
    else:
        trajs = [trajdata]
    return trajs
                                        
def executeTraj(trajdata,whicharm='l'):
    raw_input('press enter to move the robot')
    if manip_name.find('torso')>-1:
        torso_goal = JointTrajectoryGoal()
    arm_goal = JointTrajectoryGoal()
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
    if manip_name.find('torso')>-1:
        torso_res = MoveToHandPositionResponse()
    for i in range(numpoints):
        start = 3+numvalues*i
        torso_pt = JointTrajectoryPoint()
        arm_pt = JointTrajectoryPoint()
        k = 0
        for j in robot.GetJoints(manip.GetArmIndices()):
            pos = float(tokens[start+offset+j.GetDOFIndex()])
            if j.GetName()=='torso_lift_joint':
                if i>0:
                    prevpos = torso_res.traj.points[-1].positions[0]
                    newpos = prevpos + angleDistance(pos, prevpos)
                    torso_pt.positions.append(newpos)
                else:
                    torso_pt.positions.append(pos)
            else:
                if i>0:
                    prevpos = arm_res.traj.points[-1].positions[k]
                    newpos = prevpos + angleDistance(pos, prevpos)
                    arm_pt.positions.append(newpos)
                else:
                    arm_pt.positions.append(pos)
                k = k+1
        if trajoptions & 4:
            arm_pt.time_from_start = rospy.Duration(float(tokens[start]))*2
            if manip_name.find('torso')>-1:
                torso_pt.time_from_start = rospy.Duration(float(tokens[start]))*2
        arm_res.traj.joint_names = \
            filter(lambda jn: jn!='torso_lift_joint',
                 [j.GetName() for j in robot.GetJoints(manip.GetArmIndices())])
        arm_res.traj.points.append(arm_pt)
        if manip_name.find('torso')>-1:
            torso_res.traj.joint_names = ['torso_lift_joint']
            torso_res.traj.points.append(torso_pt)
    arm_goal.trajectory = arm_res.traj
    if manip_name.find('torso')>-1:
        torso_goal.trajectory = torso_res.traj
        print 'sending torso JointTrajectoryGoal'
        torso_joint_action_client.send_goal(torso_goal)

    print 'sending arm JointTrajectoryGoal'
    if whicharm == 'l':
        l_arm_joint_action_client.send_goal(arm_goal)
        l_arm_joint_action_client.wait_for_result()
    else:
        r_arm_joint_action_client.send_goal(arm_goal)
        r_arm_joint_action_client.wait_for_result()
    if manip_name.find('torso')>-1:
        torso_joint_action_client.wait_for_result()

def moveGripper(whichgripper, position, effort):
    gripperGoal = Pr2GripperCommandGoal()
    gripperGoal.command.position = position
    gripperGoal.command.max_effort = effort
    print 'sending gripper command goal'
    if whichgripper=='l':
        l_gripper_action_client.send_goal(gripperGoal)
        l_gripper_action_client.wait_for_result()
    elif whichgripper=='r':
        r_gripper_action_client.send_goal(gripperGoal)
        r_gripper_action_client.wait_for_result()
    else:
        print whichgripper,'needs to be either l or r'

def pushBox(u,q):
    # get rid of old visulization
    for simbox in simulated_boxes:
        simbox.GetLinks()[0].GetGeometries()[0].SetDraw(False)

    # visualize simulated push result
    sim_dx = q[1] + .5
    sim_dy = -q[0] + .19
    sim_dtheta = q[2]
    sim_R = rotationMatrixFromAxisAngle([0,0,1],sim_dtheta)
    O_P_simbox = dot(O_T_basefootprint,
                      basefootprint_P_box+array([[sim_dx],[sim_dy],[0],[0]]))
    print O_P_simbox
    O_T_simbox = eye(4)
    O_T_simbox[:3,:3] = sim_R
    O_T_simbox[:3,3] = transpose(O_P_simbox[:3])[0]
    simulated_box = createBox('simulated_box-%d'%len(simulated_boxes),
                              O_T_simbox, False,[0,1,0],.7)
    simulated_boxes.append(simulated_box)


    h=[]
    nSteps = 20
    dt = .002

    # calculate O_T_M: matlab coordinate frame in openrave frame
    O_T_M = array([[0,1,0,.5],   # assuming two frames' axes are aligned
                   [-1,0,0,.19], # only different order
                   [0,0,1,.74],
                   [0,0,0,1]])

    # push initial contact point in matlab frame
    M_P_contact = array([[u[0]],
                         [u[1]],
                         [0],
                         [1]])
    # push final contact point in matlab frame
    M_P_contactEnd = array([[u[0]+nSteps*dt*u[2]],
                            [u[1]+nSteps*dt*u[3]],
                            [0],
                            [1]])
    
    # push init and final contact positions in openrave frame
    O_P_contact = dot(O_T_M, M_P_contact)
    O_P_contact = transpose(O_P_contact)[0,:3]
    O_P_contactEnd = dot(O_T_M, M_P_contactEnd)
    O_P_contactEnd = transpose(O_P_contactEnd)[0,:3]

    # hard coded contact offset in fingertip frame
    lGripperRFingerTipLink_P_pushpoint = [-.016,-.031,0]

    # calculate O_T_lArmEEGoal: manipulator transform making desired contact
    lGripperRFingerTipLink_T_pushpoint = eye(4)
    lGripperRFingerTipLink_T_pushpoint[:3,3] = \
        lGripperRFingerTipLink_P_pushpoint
    O_T_lGripperRFingerTipLink = \
        robot.GetLink('l_gripper_r_finger_tip_link').GetTransform()
    O_T_pushpoint = dot(O_T_lGripperRFingerTipLink,
                        lGripperRFingerTipLink_T_pushpoint)
    O_T_lArmEE = robot.GetManipulator(manip_name)\
        .GetEndEffectorTransform()
    lArmEE_T_pushpoint = dot(linalg.inv(O_T_lArmEE),O_T_pushpoint)
    O_T_pushpointGoal_z = [0,0,1] # assuming only pushing in xy plane
    opposite_pushing_dir = O_P_contact - O_P_contactEnd
    q = -pi+arctan(opposite_pushing_dir[0]/opposite_pushing_dir[1])+pi
    O_T_pushpointGoal_x = [cos(q), sin(q), 0] # opposing pushing dir
    O_T_pushpointGoal_y = [-sin(q), cos(q), 0]
    O_T_pushpointGoal = array([r_[O_T_pushpointGoal_x,0],
                               r_[O_T_pushpointGoal_y,0],
                               r_[O_T_pushpointGoal_z,0],
                               [0,0,0,1]])
    O_T_pushpointGoal[:3,3] = O_P_contact
    # h.append(drawTransform(array([[1,0,0,O_P_contact[0]],
    #                               [0,1,0,O_P_contact[1]],
    #                               [0,0,1,O_P_contact[2]+.13],
    #                               [0,0,0,1]]),.1, .01))
    #h.append(drawTransform(O_T_pushpointGoal,.1,.01))
    pushpoint_T_lArmEE = linalg.inv(lArmEE_T_pushpoint)
    O_T_lArmEEGoal = dot(O_T_pushpointGoal,pushpoint_T_lArmEE)

    O_T_pushpointEndGoal = copy(O_T_pushpointGoal)
    O_T_pushpointEndGoal[:3,3] = O_P_contactEnd
    O_T_lArmEEEndGoal = dot(O_T_pushpointEndGoal,pushpoint_T_lArmEE)

    # draw init push point and end push point
    h.append(env.drawarrow(p1=O_T_pushpointGoal[:3,3]+[0,0,.13],
                           p2=O_T_pushpointEndGoal[:3,3]+[0,0,.13],
                           linewidth=.005,color=[1.0,0.0,0.0]))

    box.Enable(True)
    num_tries = 0
    max_num_tries = 20
    no_collision = False
    h_pushcontact= []
    while num_tries<max_num_tries:
        num_tries = num_tries+1
        try:
            trajdata = basemanip.MoveToHandPosition([O_T_lArmEEGoal], 
                                                    outputtraj=True)
            no_collision = True
            break
        except openravepy.planning_error, e:
            print e
            time.sleep(1.0)
            print '#%d/%d tries failed'%(num_tries,max_num_tries)
            O_T_pushpointGoal[:3,3] = O_P_contact+\
                .1*num_tries*(O_P_contact-O_P_contactEnd)
            pushpoint_T_lArmEE = linalg.inv(lArmEE_T_pushpoint)
            O_T_lArmEEGoal = dot(O_T_pushpointGoal,pushpoint_T_lArmEE)
            
            h_pushcontact.append(drawTransform(O_T_pushpointGoal,.1,.005))
    if not no_collision:
        print 'failed to jitter, ignoring box collision'
        box.Enable(False)
        trajdata = basemanip.MoveToHandPosition([O_T_lArmEEGoal], 
                                                outputtraj=True)   
    robot.WaitForController(0)
    executeTraj(trajdata)

    # trajs = constrainedMove(manip.GetEndEffectorTransform(), O_T_contact)
    # for traj in trajs:
    #     executeTraj(traj)
    
    box.Enable(False)
    trajs = constrainedMove(manip.GetEndEffectorTransform(), 
                            O_T_lArmEEEndGoal)
    for traj in trajs:
        executeTraj(traj)

    # FIXME: once torso calibration is done
    # move the torso up so that the robot can track box accurately again
    robot.SetActiveDOFs([robot.GetJoint('torso_lift_joint').GetDOFIndex()])
    trajdata = basemanip.MoveActiveJoints([.305], outputtraj=True)
    executeTraj(trajdata,'l')
    robot.SetActiveDOFs(manip.GetArmIndices())

    time.sleep(10)

    O_T_realbox = box.GetTransform()
    O_T_realbox[2,3] = O_T_realbox[2,3]-.001
    real_box = createBox('real_box-%d'%len(real_boxes),
                      O_T_realbox,False,[1,0,0],.7)
    real_boxes.append(real_box)

    print 'completed push'


def drawConstraintPlane(xyzconstraints):
    Tplane = eye(4)
    Tplane[0:3,0:2] = Tplane[0:3,xyzconstraints]
    Tplane[0:3,2] = cross(Tplane[0:3,0],Tplane[0:3,1])
    Tplane[0:3,3] = manip.GetEndEffectorTransform()[0:3,3]
    hplane = env.drawplane(transform=Tplane,extents=[1.0,1.0],texture=reshape([1,1,0.5,0.5],(1,1,4)))
    return hplane

def drawTransform(T,length=0.1,linewidth=.01):
    """draws a set of arrows around a coordinate system                         
    """
    return [env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,0],linewidth=linewidth,color=[1.0,0.0,0.0]),
            env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,1],linewidth=linewidth,color=[0.0,1.0,0.0]),
            env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,2],linewidth=linewidth,color=[0.0,0.0,1.0])]

if __name__=='__main__':
    send_host = ''
    send_port = 54321
    backlog = 1
    socketServer = SocketServer(send_host, send_port, backlog)

    BOX_L = .216
    BOX_W = .195
    BOX_H = .143
    basefootprint_P_box = array([[0],[0],[.816-BOX_H/2],[1]])
    O_P_tabletop = array([.43-BOX_W/2, .417+BOX_L/2])

    def on_exit(sig, func=None):
        print 'ctrl+c handler triggered'

        print 'closing server'
        socketServer.server.close()
        print 'closing client'
        socketClient.server.close()
        
        print 'cleaning up openrave'
        RaveDestroy()

    signal.signal(signal.SIGINT, on_exit)

    threads = []
    threads.append(socketServer)

    rospy.init_node('boxpushing_node')
    l_arm_joint_trajectory_action_name = 'l_arm_controller/joint_trajectory_action'
    l_arm_joint_action_client = actionlib.SimpleActionClient(l_arm_joint_trajectory_action_name, JointTrajectoryAction)
    print 'waiting for',l_arm_joint_trajectory_action_name
    print 'have you set ROS_IP?'
    l_arm_joint_action_client.wait_for_server()

    r_arm_joint_trajectory_action_name = 'r_arm_controller/joint_trajectory_action'
    r_arm_joint_action_client = actionlib.SimpleActionClient(r_arm_joint_trajectory_action_name, JointTrajectoryAction)
    print 'waiting for',r_arm_joint_trajectory_action_name
    r_arm_joint_action_client.wait_for_server()

    l_gripper_action_client_name = 'l_gripper_controller/gripper_action'
    l_gripper_action_client = actionlib.SimpleActionClient(l_gripper_action_client_name, Pr2GripperCommandAction)
    print 'waiting for',l_gripper_action_client_name
    l_gripper_action_client.wait_for_server()

    r_gripper_action_client_name = 'r_gripper_controller/gripper_action'
    r_gripper_action_client = actionlib.SimpleActionClient(r_gripper_action_client_name, Pr2GripperCommandAction)
    print 'waiting for',r_gripper_action_client_name
    r_gripper_action_client.wait_for_server()

    torso_joint_trajectory_action_name = 'torso_controller/joint_trajectory_action'
    torso_joint_action_client=actionlib.SimpleActionClient(torso_joint_trajectory_action_name, JointTrajectoryAction)
    print 'waiting for', torso_joint_trajectory_action_name
    torso_joint_action_client.wait_for_server()

    point_head_action_name = 'head_traj_controller/point_head_action'
    point_head_action_client = actionlib.SimpleActionClient(point_head_action_name, PointHeadAction)
    print 'waiting for', point_head_action_name
    point_head_action_client.wait_for_server()
    print 'done init ROS'


    env = Environment()

    env.SetViewer('qtcoin')
    # front view
    #Tcamera = matrixFromAxisAngle([0.564935, 0.581771, 0.585142],2.118629)
    #Tcamera[:3,3] = [1.277192, 0.174227, 0.732486]
    # top view
    Tcamera = matrixFromAxisAngle([0.014018, -0.033070, -0.999355],1.565693)
    Tcamera[:3,3] = [0.486017, 0.071643, 1.875806]
    env.GetViewer().SetCamera(Tcamera)

    env.Reset()
    env.Load('data/csail-pr2-table-pushing.env.xml')

    robot = env.GetRobot('pr2')
    manip_name = 'leftarm_torso'
    robot.SetActiveManipulator(manip_name)
    manip=robot.GetActiveManipulator()
    basemanip = interfaces.BaseManipulation(robot)

    print 'resetting joint limits'
    joint_names = ['torso_lift_joint',
                   'l_shoulder_lift_joint',
                   'l_shoulder_pan_joint',
                   'l_upper_arm_roll_joint',
                   'l_elbow_flex_joint',
                   'l_wrist_flex_joint',
                   'l_gripper_joint',
                   'r_shoulder_lift_joint',
                   'r_shoulder_pan_joint',
                   'r_upper_arm_roll_joint',
                   'r_elbow_flex_joint',
                   'r_wrist_flex_joint',
                   'r_gripper_joint']

    new_limits = [[.0115, .31-.005],
                  [-.5236+.17, 1.3963-.1],
                  [pi/4-1.5+.15, pi/4+1.5-.15],
                  [1.55-2.35+.15, 1.55+2.35-.15],
                  [-2.3213+.2, -.15],
                  [-2.18+.18, 0-.1],
                  [0-.01, .09-.002],
                  [-.5236+.17, 1.3963-.1],
                  [-pi/4-1.5+.15, -pi/4+1.5-.15],
                  [-1.55-2.35+.15, -1.55+2.35-.15],
                  [-2.3213+.2, -.15],
                  [-2.18+.18, 0-.1],
                  [0-.01, .09-.002]]

    for jn,limits in izip(joint_names,new_limits):
        print jn
        print 'old limits:',robot.GetJoint(jn).GetLimits()
        print 'new limits:', limits
        robot.GetJoint(jn).SetLimits([limits[0]],[limits[1]])


    realrobot = env.ReadRobotXMLFile(robot.GetXMLFilename())
    realrobot.SetName('real_pr2')
    for link in realrobot.GetLinks():
        for geom in link.GetGeometries():
            geom.SetTransparency(.8)
    env.AddRobot(realrobot, True)
    realrobot.SetTransform(robot.GetTransform())
    realrobot.SetDOFValues(robot.GetDOFValues())
    realrobot.Enable(False)

    def updateRobotJoints(msg):
        values = realrobot.GetDOFValues()
        for name,pos in izip(msg.name, msg.position):
            j = realrobot.GetJoint(name)
            if j is not None:
                values[j.GetDOFIndex()] = pos
        realrobot.SetDOFValues(values)
    
    rospy.Subscriber('/joint_states',JointState,updateRobotJoints,queue_size=1)

    real_boxes = []
    simulated_boxes = []

    def createBox(name,O_T_box,enabled,color=[0,0,1],transparency=0):
        box = env.ReadKinBodyURI('data/harddrive-box.kinbody.xml')
        box.SetName(name)
        #boxExtents = [BOX_L/2,BOX_W/2,BOX_H/2]
        #box.InitFromBoxes(array([r_[[0,0,0],boxExtents]]),draw=True)
        box.SetTransform(O_T_box)
        env.AddKinBody(box)
        box.Enable(enabled)
        box.GetLinks()[0].GetGeometries()[0].SetTransparency(transparency)
        box.GetLinks()[0].GetGeometries()[0].SetDiffuseColor(color)
        return box

    #box = env.CreateKinBody()
    #box.SetName('box')
    O_T_basefootprint = robot.GetLink('base_footprint').GetTransform()
    O_P_box = dot(O_T_basefootprint,basefootprint_P_box)
    #boxExtents = [BOX_L/2,BOX_W/2,BOX_H/2]
    #box.InitFromBoxes(array([r_[[0,0,0],boxExtents]]),draw=True)
    O_T_box = eye(4)
    O_T_box[:3,3] = transpose(O_P_box[:3])[0]
    #box.SetTransform(O_T_box)
    #env.AddKinBody(box)
    #box.Enable(False)
    box = createBox('box',O_T_box,False)

    dx=0
    dy=0
    dtheta=0
    def updateBoxPose(data):
        if len(data.poses)>1:
            print 'received more than 1 pose'
            print 'ignoring all but the first one'
        
        dx = data.poses[0].x
        dy = data.poses[0].y
        dtheta = data.poses[0].theta
        msg = '%.2f '%dx + \
              '%.2f '%dy + \
              '%.2f '%dtheta
        #print 'received', msg

        R = rotationMatrixFromAxisAngle([0,0,1],dtheta)
        O_P_box = dot(O_T_basefootprint, \
                      basefootprint_P_box+array([[dx],[dy],[0],[0]]))
        #print O_P_box[:3]
        O_T_box = eye(4)
        O_T_box[:3,:3] = R
        O_T_box[:3,3] = transpose(O_P_box[:3])[0]
        box.SetTransform(O_T_box)

        try:
            if socketServer.client:
                # print 'sending pose in table_init frame'
                # socketServer.client.send(msg)
                print 'sending pose in matlab frame'
                y = dx - .5
                x = -(dy - .19)
                t = dtheta
                msg = '%.2f '%x + \
                      '%.2f '%y + \
                      '%.2f '%t
                socketServer.client.send(msg)
        except socket.error, (value, message):
            if value!=32:
                print 'failed to send message'
                print value, message
            else:
                print 'done sending state estimation to matlab'
            print 'closing client'
            socketServer.client.close()
            socketServer.client = None
        
        point_head_goal = PointHeadGoal()
        point_head_goal.target.header.frame_id = '/base_footprint'

        # point_head_goal.target.point.x = basefootprint_P_box[0]+dx-.15
        # y = basefootprint_P_box[1]+dy
        # if abs(y)<.25:
        #     k = 15
        # elif abs(y)<.3:
        #     k = 10
        # elif abs(y)<.35:
        #     k = 6.67
        # else:
        #     k = 4.44
        # point_head_goal.target.point.y = k*y**3
        point_head_goal.target.point.x = basefootprint_P_box[0]+dx
        point_head_goal.target.point.y = basefootprint_P_box[1]+dy
        point_head_goal.target.point.z = basefootprint_P_box[2]-1.5*BOX_H/2
        point_head_action_client.send_goal(point_head_goal)
        # h = drawTransform(array([[1,0,0,point_head_goal.target.point.x],
        #                          [0,1,0,point_head_goal.target.point.y],
        #                          [0,0,1,point_head_goal.target.point.z],
        #                          [0,0,0,1]]), .3)
        time.sleep(1.0)
        h = None

    rospy.Subscriber('/table_poses', Table_Poses, updateBoxPose)


    time.sleep(1.0)
    robot.SetDOFValues(realrobot.GetDOFValues())


    recv_host = ''
    recv_port = 12345
    size = 1024
    socketClient = SocketClient(recv_host, recv_port, size)
    threads.append(socketClient)

    time.sleep(10)
    O_T_realbox = box.GetTransform()
    O_T_realbox[2,3] = O_T_realbox[2,3]-.001
    real_box = createBox('real_box-%d'%len(real_boxes),
                      O_T_realbox,False,[1,0,0],.7)
    real_boxes.append(real_box)

    print 'done initializing python script'
    print 'please start matlab script'
    rospy.spin()
    print 'haha'
