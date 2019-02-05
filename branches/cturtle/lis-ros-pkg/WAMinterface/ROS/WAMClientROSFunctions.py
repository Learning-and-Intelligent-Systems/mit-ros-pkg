#!/usr/bin/env python
'''ROS client functions to use the Barrett WAM and Hand (sample code in testprograms)
author: Kaijen Hsiao (kaijenhsiao@gmail.com)'''

import roslib
import time
import math
import scipy
import pdb
roslib.load_manifest('WAMinterface')

from std_srvs.srv import *
import rospy
from WAMinterface.srv import *


#call a ROS service 'servicename' with service message 'type' 
#and request arguments 'args', return the response (None if error)
def call_ROS_service(servicename, type = Empty, args = []):
    rospy.wait_for_service(servicename)
    try:
        s = rospy.ServiceProxy(servicename, type)
        resp = s(*args)
    except rospy.ServiceException, e:
        print "error in %s: %s"%(servicename, e)
        return None
    return resp


#connect to the WAM socket client (need to start socketwamif)
def connect_WAM_client(port):
    resp = call_ROS_service('connect_WAM_client', SocketPort, [port])
    return resp != None


#shut down the arm/hand if still connected and close the connection to the WAM socket client
def close_WAM_client():
    resp = call_ROS_service('close_WAM_client')
    return resp != None


#pretty-print a double array
def ppdoublearray(array):
    strlist = ["%4.2f"%x for x in array]
    return " ".join(strlist)




#Arm services

#connect the arm (must be run before other arm commands!)
#connect the hand separately with hand_connect
def connect_arm():
    resp = call_ROS_service('connect_arm')
    return resp != None


#disconnect the arm (can reconnect with connect_arm)
def disconnect_arm():
    resp = call_ROS_service('disconnect_arm')
    return resp != None


#turn on gravity compensation
def turn_on_gravity_comp():
    resp = call_ROS_service('turn_on_gravity_comp')
    return resp != None


#run joint angle calibration for optical encoders (put arm in home position first)
#joint angle sequence for calibration should be in calibrationangles.txt in the directory where socketwamif is run
def joint_calibration_optical():
    resp = call_ROS_service('joint_calibration_optical')
    return resp != None


#wait for trajectory to finish
def trajectory_wait():
    resp = call_ROS_service('trajectory_wait')
    return resp != None


#check if trajectory is finished (non-blocking)
#returns 1 if done, 0 if not done, -1 if error
def trajectory_status():
    resp = call_ROS_service('trajectory_status', TrajectoryStatus)
    return resp.status


#shut down the WAM controllers (all but gravity comp)
def disable_controllers():
    resp = call_ROS_service('disable_controllers')
    return resp != None


#get the current WAM joint angles
def get_joint_angles():
    resp = call_ROS_service('get_joint_angles', EmptyJointAngles)
    return resp.jointangles


#get the current motor angles (radians from zero pos)
def get_motor_angles():
    resp = call_ROS_service('get_motor_angles', EmptyJointAngles)
    return resp.jointangles


#move to a set of joint angles
def move_to_joint(jointangles):
    resp = call_ROS_service('move_to_joint', JointAnglesEmpty, [jointangles])
    return resp != None


#send arm home
def arm_home():
    resp = call_ROS_service('arm_home')
    return resp != None


#set the torque limits for the WAM joints (enforced in WAMcallback)
#[6.31, 6.31, 6.31, 6.31, 1.826, 1.826, 0.613] are Barrett's stated peak values
#[7.75,7.75,7.75,7.75,2.5,2.5,2] gives you most of the possible torque while avoiding most torque faults
def set_torque_limits(torquelimits):
    resp = call_ROS_service('set_torque_limits', TorqueLimits, [torquelimits])
    return resp != None


#move to a Cartesian position/orientation (tool frame as defined in wam.conf)
#pos is [x,y,z] in meters
#rot is a 3x3 rotation matrix as a 9-vector [R11, R12, R13, R21... R33]
#(origin is the point where joint 0,1,2 axes meet, frame as defined in manual)
def move_to_cartesian(pos, rot):
    resp = call_ROS_service('move_to_cartesian', CartesianMove, [pos, rot])
    return resp != None


#get Cartesian position/orientation (as in move_to_cartesian, tool frame defined in wam.conf)
def get_cartesian_pos_and_rot():
    resp = call_ROS_service('get_cartesian_pos_and_rot', CartesianPosAndRot)
    return (resp.pos, resp.rot)

#Arm joint trajectory control

#move through a joint angle trajectory (list of double[7]s of length length)
#e.g: len = 2, traj = [0,1,2,3,4,5,6,0,1,2,3,4,5,6]
def move_joint_trajectory(len, traj):
    resp = call_ROS_service('move_joint_trajectory', JointTrajectoryMove, [len, traj])
    return resp != None


#run inverse kinematics (palmmat4 is center-of-palm tool frame)
#(doesn't require the robot or running connect_WAM_client)
def inverse_kinematics(palmmat4, currentangles):
    resp = call_ROS_service('inverse_kinematics', InverseKinematics, [palmmat4, currentangles])
    if resp.success == 'success':
        return resp.resultangles
    else:
        print "no analytical IK solution in inverse_kinematics!"
        return None


#run optimization inverse kinematics (palmmat4 is center-of-palm tool frame)
#(doesn't require the robot or running connect_WAM_client)
def optimization_inverse_kinematics(palmmat4, currentangles):
    resp = call_ROS_service('optimization_inverse_kinematics', InverseKinematics, [palmmat4, currentangles])
    return resp.resultangles


#run forward kinematics (palm center relative to joint 0,1,2 origin)
#(doesn't require the robot or running connect_WAM_client)
def forward_kinematics(angles):
    resp = call_ROS_service('forward_kinematics', ForwardKinematics, [angles])
    return resp.palmmat




#Hand services

#connect hand (must be run before all other hand commands!)
def hand_connect():
    resp = call_ROS_service('hand_connect')
    return resp != None


#disconnect hand (can be reconnected with hand_connect)
def hand_disconnect():
    resp = call_ROS_service('hand_disconnect')
    return resp != None


#convert finger angle to position 
def hand_angleToPosition(motor, angle):
    if motor == 0:
        return int(angle*17.5*180/math.pi)
    return int(angle*125*180/math.pi)
    

#convert finger position to angle
def hand_positionToAngle(motor, position):
    if motor == 0:
        return float(position)*math.pi/(17.5*180.)
    return float(position)*math.pi/(125.*180.)


#set the position (int, 0-18000) for one motor 
#motor is 0 for spread, 1-3 for finger bend
#waits for termination 
def hand_set_finger_position(motor, position):
    resp = call_ROS_service('hand_set_finger_position', FingerPosition, [motor, position])
    return resp != None


#set the positions for all four motors ([S F1 F2 F3]) one at a time
#waits for termination
def hand_set_all_finger_positions(positions):
    resp = call_ROS_service('hand_set_all_finger_positions', AllFingerPositionsEmpty, [positions])
    return resp != None


#set the angle (double, joint radians) for one motor 
#motor is 0 for spread, 1-3 for finger bend
#waits for termination
def hand_set_finger_angle(motor, angle):
    resp = call_ROS_service('hand_set_finger_angle', FingerAngle, [motor, angle])
    return resp != None


#set the angles for all four motors ([S F1 F2 F3]) one at a time
#waits for termination
def hand_set_all_finger_angles(angles):
    resp = call_ROS_service('hand_set_all_finger_angles', AllFingerAnglesEmpty, [angles])
    return resp != None


#get the positions (0-17800 for bend, 0-3150 for spread) for all four motors ([S F1 F2 F3])
def hand_get_finger_positions():
    resp = call_ROS_service('hand_get_finger_positions', EmptyAllFingerPositions)
    return resp.positions


#get the breakaway status and positions for the three fingers [F1 F2 F3]
def hand_get_breakaway_positions():
    resp = call_ROS_service('hand_get_breakaway_positions', BreakawayStatusAndPositions)
    return (resp.breakawaystatus, resp.breakawaypositions)


#get the joint angles for all seven joints ([S F1 F2 F3 F1tip F2tip F3tip])
def hand_get_finger_angles():
    resp = call_ROS_service('hand_get_finger_angles', EmptyAllFingerAngles)
    return resp.angles


#clip the magnitudes of the values in a scipy array (changes arr in-place and also returns it)
def clipArrayMag(arr, maxmag):
    arr[arr>maxmag] = maxmag
    arr[arr<-maxmag] = -maxmag
    return arr


#move the fingers towards the spread/bend goal angles 
#can limit the move for each joint to at most maxincrement
#don't move joints that are already within anglethreshold
#opens/closes fingers together whenever possible
#if you already have the current finger angles, input them to avoid querying
#returns 1 if a command was sent to the hand; 0 otherwise
def hand_move_incremental(goalangles, maxincrement = None, anglethreshold = .01, currentfingerangs = None):
    verbose = 0
    sentcommand = 0

    #track which motors to move individually
    moveindmotors = scipy.array([1,1,1,1])            
            
    #how much are we trying to move by?
    if currentfingerangs == None:
        currentfingerangs = hand_get_finger_angles()
    currentfingerangs = scipy.array(currentfingerangs[0:4])
    angdiffs = scipy.array(goalangles) - currentfingerangs

    if verbose:
        print "currentfingerangs:", ppdoublearray(currentfingerangs)
        print "angdiffs:", ppdoublearray(angdiffs)

    #don't move any motor by more than maxincrement in one step
    if maxincrement:
        posmove = hand_angleToPosition(1, maxincrement)        
        #if all fingers are trying to bend by at least maxincrement, move them all at once
        if all(angdiffs[1:4] > maxincrement):
            if verbose:
                print "moving all fingers in by maxincrement"
            cmd = "GIC " + str(posmove)
            hand_send_raw_cmd(cmd)
            sentcommand = 1
            moveindmotors[1:4] = [0,0,0]
        elif all(angdiffs[1:4] < -maxincrement):
            if verbose:
                print "moving all fingers out by maxincrement"
            cmd = "GIO " + str(posmove)
            hand_send_raw_cmd(cmd)
            sentcommand = 1
            moveindmotors[1:4] = [0,0,0]

        #clip moves to at most maxincrement
        clipArrayMag(angdiffs, maxincrement)
        goalangles = list(angdiffs + currentfingerangs)
        if verbose:
            print "clipped goalangles:", ppdoublearray(goalangles)

    #if no max increment, and all fingers are trying to bend to the same place, also move them all at once
    else:
        if math.fabs(angdiffs[1]) > anglethreshold and not any(scipy.fabs(scipy.array(goalangles) - goalangles[1])[1:4] > anglethreshold):
            posmove = hand_angleToPosition(1, goalangles[1])
            cmd = "GM " + str(posmove)
            hand_send_raw_cmd(cmd)
            sentcommand = 1
            moveindmotors[1:4] = [0,0,0]
            if verbose:
                print "moving all fingers to angle %.3f"%goalangles[1]

    #only move joints that aren't already close to the goal
    moveindmotors[scipy.fabs(angdiffs)<anglethreshold] = 0

    #if we're still moving all 4 motors, send all four angles as one command
    if all(moveindmotors):
        hand_set_all_finger_angles(goalangles)
        sentcommand = 1
        if verbose:
            print "moving to goalangles:", ppdoublearray(goalangles)

    #otherwise, move the relevant motors one at a time
    else:
        if verbose:
            print "moveindmotors:", moveindmotors
        for motor in range(4):
            if moveindmotors[motor]:
                hand_set_finger_angle(motor, goalangles[motor])
                sentcommand = 1
                if verbose:
                    print "moving motor", motor, "to angle %.3f"%goalangles[motor]

    return sentcommand

#get the strain gauge values for all three fingers ([F1 F2 F3])
def hand_get_strain_gauge():
    resp = call_ROS_service('hand_get_strain_gauge', StrainGaugeVals)
    return resp.strainvals


#send a raw hand command (supervisory mode), wait for termination 
#don't include the \r at the end (it gets added later)
def hand_send_raw_cmd(cmd):
    resp = call_ROS_service('hand_send_raw_cmd', StringString, [cmd])
    return resp != None


#start hand realtime mode
#takes in the parameter string and the loop string (leave empty to use defaults)
def hand_realtime_start(parameterstring = '0', loopstring = '0'):
    resp = call_ROS_service('hand_realtime_start', HandRealtimeStart, [parameterstring, loopstring])
    return resp != None


#terminate hand realtime mode (back to supervisory mode)
def hand_realtime_terminate():
    resp = call_ROS_service('hand_realtime_terminate')
    return resp != None


#bend the fingers in realtime mode (assumes default startup)
#takes in vels[3] and gains[3], both between -127 and 127
#returns strainvals[3] and fingerpositions[3]
def hand_realtime_bend(vels, gains):
    resp = call_ROS_service('hand_realtime_bend', HandRealtimeBend, [vels, gains])
    return (resp.strainvals, resp.fingerpositions)


#pause for enter
def keypause():
    print "press enter twice to continue"
    raw_input()
    raw_input()

