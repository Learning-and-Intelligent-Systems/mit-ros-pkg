#!/usr/bin/env python
'''ROS server to use the Barrett WAM and Hand 
Starts a socket server to use with WAMinterfacelib/socketwamif/socketwamif'''


import roslib
roslib.load_manifest('WAMinterface')

from std_srvs.srv import *
import rospy
from WAMinterface.srv import *
import socketservercmds
import sys
import wamik
wamik.init_wamik()

sock = 0
sockserver = 0


#connect to the WAM client (must be called before all things except inverse_kinematics and forward_kinematics!)
def connect_WAM_client(req):
    global sock
    global sockserver

    if sock:
        print "sock was already open, shutting it down"
        socketservercmds.shut_down(sockserver)

    print "starting the socket server to talk to the WAM"
    sockserver=socketservercmds.start_server(req.port)

    print "connecting to the WAM socket client"
    print "now start up the WAMinterfacelib/socketwamif client"
    sock=socketservercmds.connect_client(sockserver)

#     socketservercmds.connect_arm(sock)

#     #get the current joint angles, to make sure the connection is fine
#     jointangles = socketservercmds.get_joint(sock)
#     print "current joint angles:", ppdoublearray(jointangles)

    return SocketPortResponse()


#shut down hand/arm/WAM client and close the connection
def close_WAM_client(req):
    if not sock:
        print "sock not open!"
        return EmptyResponse()

    print "shutting down WAM client"
    socketservercmds.shutdown_all(sock)
    return EmptyResponse()


#pretty-print a double array
def ppdoublearray(array):
    strlist = ["%4.2f"%x for x in array]
    return " ".join(strlist)



#Arm services


#connect the arm (must be run before other arm commands!)
#connect the hand separately with hand_connect
def connect_arm(req):
    success = socketservercmds.connect_arm(sock)
    if not success:
        print "error while connecting arm!"
    return EmptyResponse()


#disconnect the arm (can reconnect with connect_arm)
def disconnect_arm(req):
    success = socketservercmds.disconnect_arm(sock)
    if not success:
        print "error while disconnecting arm!"
    return EmptyResponse()


#turn on gravity compensation
def turn_on_gravity_comp(req):
    print "turning on gravity comp"
    success = socketservercmds.set_gcomp(sock)
    if not success:
        print "error in turn_on_gravity_comp!"
    return EmptyResponse()


#run joint angle calibration for optical encoders (put arm in home position first)
#joint angle sequence for calibration should be in calibrationangles.txt in the directory where socketwamif is run
def joint_calibration_optical(req):
    print "running joint calibration"
    success = socketservercmds.run_joint_calibration(sock)
    if not success:
        print "error in joint_calibration!"
    return EmptyResponse()


#wait for trajectory to finish
def trajectory_wait(req):
    print "waiting for trajectory to finish"
    success = socketservercmds.wait_until_traj_done(sock)
    if not success:
        print "error in trajectory_wait!"
    return EmptyResponse()


#check if trajectory is finished (non-blocking)
def trajectory_status(req):
    status = socketservercmds.check_traj_done(sock)
    return TrajectoryStatusResponse(status)


#disable the WAM controllers (all but gravity comp)
def disable_controllers(req):
    print "disabling controllers"
    success = socketservercmds.stop_controllers(sock)
    if not success:
        print "error in disable_controllers!"
    return EmptyResponse()


#get the current WAM joint angles
def get_joint_angles(req):
    print "about to get joint angles"
    try:
        jointangles = socketservercmds.get_joint(sock)
        print "returning joint angles:", ppdoublearray(jointangles)
    except:
        print "error in calling get_joint"
    return EmptyJointAnglesResponse(jointangles)


#get the current motor angles (radians from zero pos)
def get_motor_angles(req):
    print "about to get motor angles"
    try:
        motorangles = socketservercmds.get_motor_angles(sock)
    except:
        print "error in get_motor_angles"
    return EmptyJointAnglesResponse(motorangles)

        
#move to a set of joint angles
def move_to_joint(req):
    print "moving to joint angles:", ppdoublearray(req.jointangles)
    success = socketservercmds.move_joint(sock, req.jointangles)
    if not success:
        print "error in move_to_joint!"
    return JointAnglesEmptyResponse()


#send the arm back home
def arm_home(req):
    print "sending arm home"
    success = socketservercmds.go_home(sock)
    if not success:
        print "error in arm_home!"
    return EmptyResponse()


#set the torque limits for the WAM joints (enforced in WAMcallback)
#[6.31, 6.31, 6.31, 6.31, 1.826, 1.826, 0.613] are Barrett's stated peak values
#[7.75,7.75,7.75,7.75,2.5,2.5,2] gives you most of the possible torque while avoiding most torque faults
def set_torque_limits(req):
    print "setting torque limits:", ppdoublearray(req.torquelimits)
    success = socketservercmds.set_torque_limits(sock, req.torquelimits)
    if not success:
        print "error in set_torque_limits!"
    return TorqueLimitsResponse()


#move to a Cartesian position/orientation
#pos is [x,y,z] in meters
#rot is a 3x3 rotation matrix as a 9-vector [R11, R12, R13, R21... R33]
#(origin is the point where joint 0,1,2 axes meet, frame as defined in manual)
def move_to_cartesian(req):
    print "moving to Cartesian position:", ppdoublearray(req.pos)
    print "rot:", ppdoublearray(req.rot)
    success = socketservercmds.move_cartesian(sock, req.pos, req.rot)
    if not success:
        print "error in move_to_cartesian!"
    return CartesianMoveResponse()


#get Cartesian position/orientation of tool center (as defined in wam.conf)
#pos is [x,y,z] in meters
#rot is a 3x3 rotation matrix as a 9-vector [R11, R12, R13, R21... R33]
#(origin is the point where joint 0,1,2 axes meet, frame as defined in manual)
def get_cartesian_pos_and_rot(req):
    print "getting Cartesian position and rotation"
    (pos, rot) = socketservercmds.get_cartesian(sock)
    return CartesianPosAndRotResponse(pos, rot)

#Arm joint trajectory control

#move through a joint angle trajectory (list of double[7]s of length length)
#e.g: req.len = 2, req.traj = [0,1,2,3,4,5,6,0,1,2,3,4,5,6]
def move_joint_trajectory(req):
    print "moving joint trajectory"
    success = socketservercmds.move_joint_trajectory(sock,req.len,req.traj)
    if len(req.traj) % req.len !=0:
        print "trajectory must contain multiple of len joint angles!", req.traj, req.len
        return False
    if not success:
        print "error in move_joint_trajectory!"
    return JointTrajectoryMoveResponse()


#run analytical inverse kinematics (palmmat4 is center-of-palm tool frame)
#(doesn't require the robot or running connect_WAM_client)
def inverse_kinematics(req):
    print "running inverse kinematics"
    resultangles = wamik.run_ik(req.palmmat4, req.currentangles)
    if resultangles == None:
        print "no IK solution!"
        return InverseKinematicsResponse([0]*7, 'failed')
    else:
        return InverseKinematicsResponse(resultangles, 'success')

    
#run optimization inverse kinematics (palmmat4 is center-of-palm tool frame)
#(doesn't require the robot or running connect_WAM_client)
def optimization_inverse_kinematics(req):
    print "running optimization inverse kinematics"
    resultangles = wamik.run_opt_ik(req.palmmat4, req.currentangles)
    return InverseKinematicsResponse(resultangles, 'foundsomething')


#run forward kinematics (palm center relative to joint 0,1,2 origin)
#(doesn't require the robot or running connect_WAM_client)
def forward_kinematics(req):
    print "running forward kinematics"
    palmmat = wamik.run_fk(req.angles)
    return ForwardKinematicsResponse(palmmat)




#Hand services


#connect hand (must be run before all other hand commands!)
def hand_connect(req):
    print "connecting to hand"
    success = socketservercmds.connect_hand(sock)
    if not success:
        print "error in connect_hand!"
    return EmptyResponse()


#disconnect hand (can be reconnected with hand_connect)
def hand_disconnect(req):
    print "disconnecting hand"
    success = socketservercmds.disconnect_hand(sock)
    if not success:
        print "error in disconnect_hand!"
    return EmptyResponse()


#set the position (int, 0-18000) for one motor 
#motor is 0 for spread, 1-3 for finger bend
#waits for termination 
def hand_set_finger_position(req):
    print "setting finger %d position to %d"%(req.motor, req.position)
    success = socketservercmds.set_finger_position(sock, req.motor, req.position)
    if not success:
        print "error in set_finger_position!"
    return FingerPositionResponse()


#set the positions for all four motors ([S F1 F2 F3])
#waits for termination
def hand_set_all_finger_positions(req):
    print "setting finger positions to", req.positions
    success = socketservercmds.set_all_finger_positions(sock, req.positions)
    if not success:
        print "error in set_finger_positions!"
    return AllFingerPositionsEmptyResponse()


#set the angle (double, joint radians) for one motor 
#motor is 0 for spread, 1-3 for finger bend
#waits for termination
def hand_set_finger_angle(req):
    print "setting finger %d angle to %f"%(req.motor, req.angle)
    success = socketservercmds.set_finger_angle(sock, req.motor, req.angle)
    if not success:
        print "error in set_finger_angle!"
    return FingerAngleResponse()


#set the angles for all four motors ([S F1 F2 F3])
#waits for termination
def hand_set_all_finger_angles(req):
    print "setting finger angles to", req.angles
    success = socketservercmds.set_all_finger_angles(sock, req.angles)
    if not success:
        print "error in set_all_finger_angles!"
    return AllFingerAnglesEmptyResponse()


#get the positions (0-17800 for bend, 0-3150 for spread) for all four motors ([S F1 F2 F3])
def hand_get_finger_positions(req):
    print "getting finger positions"
    positions = socketservercmds.get_finger_positions(sock)
    return EmptyAllFingerPositionsResponse(positions)


#get the breakaway status and positions for the three fingers [F1 F2 F3]
def hand_get_breakaway_positions(req):
    print "getting breakaway status and positions"
    (breakawaystatus, breakawaypositions) = socketservercmds.get_breakaway_positions(sock)
    return BreakawayStatusAndPositionsResponse(breakawaystatus, breakawaypositions)


#get the joint angles for all seven joints ([S F1 F2 F3 F1tip F2tip F3tip])
def hand_get_finger_angles(req):
    print "getting finger angles"
    angles = socketservercmds.get_finger_angles(sock)
    return EmptyAllFingerAnglesResponse(angles)


#get the strain gauge values for all three fingers ([F1 F2 F3])
def hand_get_strain_gauge(req):
    print "getting strain gauge vals"
    strainvals = socketservercmds.get_strain_gauge(sock)
    return StrainGaugeValsResponse(strainvals)


#send a raw hand command (supervisory mode), wait for termination 
#don't put the \r at the end (it gets added later)
def hand_send_raw_cmd(req):
    print "sending raw hand command:", req.str
    success = socketservercmds.hand_raw(sock, req.str)
    if not success:
        print "error in hand_send_raw_cmd!"
    return StringStringResponse("done")


#start hand realtime mode
#takes in the parameter string and the loop string ('0' in either to use defaults)
def hand_realtime_start(req):
    print "running hand realtime start"
    success = socketservercmds.start_hand_realtime(sock, req.parameterstring, req.loopstring)
    if not success:
        print "error in hand_realtime_start!"
    return HandRealtimeStartResponse()


#terminate hand realtime mode (back to supervisory mode)
def hand_realtime_terminate(req):
    print "terminating hand realtime mode"
    success = socketservercmds.terminate_hand_realtime(sock)
    if not success:
        print "error in hand_realtime_terminate!"
    return EmptyResponse()


#bend the fingers in realtime mode (assumes default startup)
#takes in vels[3] and gains[3], both between -127 and 127
#returns strainvals[3] and fingerpositions[3]
#if you want a different realtime command, add your own ROS service using socketservercmds.send_realtime_command (look at socketservercmds.default_realtime_demo)
def hand_realtime_bend(req):
    print "vels:", req.vels, "gains:", req.gains
    (strainvals, fingerpositions) = socketservercmds.hand_bend_realtime(sock, req.vels, req.gains)
    return HandRealtimeBendResponse(strainvals, fingerpositions)



#start both the WAM socket server and the ROS server
def run_server():
    
    #register the ROS services
    rospy.init_node('WAM_server')
    rospy.Service('connect_WAM_client', SocketPort, connect_WAM_client)
    rospy.Service('close_WAM_client', Empty, close_WAM_client)
    rospy.Service('connect_arm', Empty, connect_arm)
    rospy.Service('disconnect_arm', Empty, disconnect_arm)
    rospy.Service('turn_on_gravity_comp', Empty, turn_on_gravity_comp)
    rospy.Service('joint_calibration_optical', Empty, joint_calibration_optical)
    rospy.Service('trajectory_wait', Empty, trajectory_wait)
    rospy.Service('trajectory_status', TrajectoryStatus, trajectory_status)
    rospy.Service('disable_controllers', Empty, disable_controllers)
    rospy.Service('get_joint_angles', EmptyJointAngles, get_joint_angles)
    rospy.Service('get_motor_angles', EmptyJointAngles, get_motor_angles)
    rospy.Service('move_to_joint', JointAnglesEmpty, move_to_joint)
    rospy.Service('arm_home', Empty, arm_home)
    rospy.Service('set_torque_limits', TorqueLimits, set_torque_limits)
    rospy.Service('move_to_cartesian', CartesianMove, move_to_cartesian)
    rospy.Service('get_cartesian_pos_and_rot', CartesianPosAndRot, get_cartesian_pos_and_rot)
    rospy.Service('move_joint_trajectory', JointTrajectoryMove, move_joint_trajectory)
    rospy.Service('inverse_kinematics', InverseKinematics, inverse_kinematics)
    rospy.Service('optimization_inverse_kinematics', InverseKinematics, optimization_inverse_kinematics) 
    rospy.Service('forward_kinematics', ForwardKinematics, forward_kinematics)
    rospy.Service('hand_connect', Empty, hand_connect)
    rospy.Service('hand_disconnect', Empty, hand_disconnect)
    rospy.Service('hand_set_finger_position', FingerPosition, hand_set_finger_position)
    rospy.Service('hand_set_all_finger_positions', AllFingerPositionsEmpty, hand_set_all_finger_positions)
    rospy.Service('hand_set_finger_angle', FingerAngle, hand_set_finger_angle)
    rospy.Service('hand_set_all_finger_angles', AllFingerAnglesEmpty, hand_set_all_finger_angles)
    rospy.Service('hand_get_finger_positions', EmptyAllFingerPositions, hand_get_finger_positions)
    rospy.Service('hand_get_breakaway_positions', BreakawayStatusAndPositions, hand_get_breakaway_positions)
    rospy.Service('hand_get_finger_angles', EmptyAllFingerAngles, hand_get_finger_angles)
    rospy.Service('hand_get_strain_gauge', StrainGaugeVals, hand_get_strain_gauge)
    rospy.Service('hand_send_raw_cmd', StringString, hand_send_raw_cmd)
    rospy.Service('hand_realtime_start', HandRealtimeStart, hand_realtime_start)
    rospy.Service('hand_realtime_terminate', Empty, hand_realtime_terminate)
    rospy.Service('hand_realtime_bend', HandRealtimeBend, hand_realtime_bend)

    #wait for requests
    print "ready for ROS client requests"
    rospy.spin()
                       
    

if __name__ == '__main__':
    run_server()
