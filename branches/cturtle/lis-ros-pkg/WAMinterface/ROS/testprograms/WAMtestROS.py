'''move the arm and/or hand using the functions in WAMClientROSFunctions
author: Kaijen Hsiao (kaijenhsiao@gmail.com)'''

import roslib
import time
roslib.load_manifest('WAMinterface')

from WAMClientROSFunctions import *

movearm = 1
movehand = 0
pause = 1

print "starting the WAM client"
connect_WAM_client(4321)

if movearm:
    print "connecting to arm"
    connect_arm()

    torquelimits = [9.99,9.99,9.99,9.99,9.99,9.99,9.99]
    print "setting torque limits to", torquelimits
    set_torque_limits(torquelimits)

    print "turning on gravity compensation"
    turn_on_gravity_comp()
    if pause:
        keypause()

    print "moving to just above home (in joint mode)"
    justabovehome = [0, -1.99, 0, 2.4, 0, 0.25, 0]
    move_to_joint(justabovehome)

    print "waiting for trajectory to finish (blocking version)"
    trajectory_wait()

    print "getting current joint angles"
    jointangles = get_joint_angles()
    if jointangles != None:
        print "joint angles:", ppdoublearray(jointangles)
    if pause:
        keypause()            

    print "moving farther out (in Cartesian mode)"
    fartheroutpos = [.31, 0, .42]
    fartheroutrot = [0,0,1,	 
                     0,1,0, 
                     -1,0,0]
    move_to_cartesian(fartheroutpos, fartheroutrot)

    print "waiting for trajectory to finish (non-blocking version)"
    while not trajectory_status():
        time.sleep(1)

    print "getting current Cartesian position/orientation"
    (pos, rot) = get_cartesian_pos_and_rot()
    print "pos:", ppdoublearray(pos)
    print "rot:", ppdoublearray(rot)
    if pause:
        keypause()

    print "get the current joint angles"
    jointangles = get_joint_angles()
    if jointangles != None:
        print "joint angles:", ppdoublearray(jointangles)

    print "carrying out a trajectory"
    length = 4
    traj = [
        [0.009007, -2.005244, 0.041125, 3.049417, 0.081722, -0.131585, -0.035286],
        [-0.000298, -1.762723, 0.034756, 2.848247, 0.062907, -0.094783, 0.002270],
        [-0.017828, -0.875831, 0.030786, 2.565059, 0.145275, 0.031809, -0.054683],
        [-0.017418, -1.459543, 0.024555, 2.865306, 0.103445, 0.092045, -0.078104]
        ]
    t=[]
    for a in traj:
        t.extend(a)
    print "len:", length
    print "traj:", t
    success = move_joint_trajectory(length,t)
    print "traj move success?", success != None

    print "get the current joint angles"
    jointangles = get_joint_angles()
    if jointangles != None:
        print "joint angles:", ppdoublearray(jointangles)

    if pause:
        keypause()

if movehand:
    print "connecting to hand"
    hand_connect()

    print "close hand (supervisory mode)"
    hand_send_raw_cmd('GC')

    print "getting breakaway status and positions"
    (breakawaystatus, breakawaypositions) = hand_get_breakaway_positions()
    print "breakawaystatus:", breakawaystatus
    print "breakawaypositions:", breakawaypositions

    print "get strain gauge values"
    strainvals = hand_get_strain_gauge()
    print "strainvals:", strainvals
    if pause:
        keypause()

    print "open hand (supervisory mode)"
    hand_send_raw_cmd('GO')

    print "open spread (supervisory mode)"
    hand_send_raw_cmd('SO')
    if pause:
        keypause()

    print "moving just finger 3 to bend 5000"
    hand_set_finger_position(3, 5000)

    print "getting all finger positions"
    currentpositions = hand_get_finger_positions()
    print "currentpositions:", currentpositions
    if pause:
        keypause()

    print "moving just finger 3 to angle PI/2"
    hand_set_finger_angle(3, math.pi/2)

    print "getting all finger angles"
    currentangles = hand_get_finger_angles()
    print "currentangles:", ppdoublearray(currentangles)
    if pause:
        keypause()

    print "moving fingers to positions spread 0, bend 5000, 10000, 15000"
    positions = [0, 5000, 10000, 15000]
    hand_set_all_finger_positions(positions)

    print "getting all finger positions"
    currentpositions = hand_get_finger_positions()
    print "currentpositions:", currentpositions
    if pause:
        keypause()

    print "moving fingers to angles spread PI, bend PI/2 (1.57), PI/3 (1.0), PI/4 (.79)"
    angles = [math.pi, math.pi/2, math.pi/3, math.pi/4]
    hand_set_all_finger_angles(angles)

    print "getting all finger angles"
    currentangles = hand_get_finger_angles()
    print "currentangles:", ppdoublearray(currentangles)
    if pause:
        keypause()

    print "closing fingers"
    hand_send_raw_cmd('GC')

    print "starting realtime mode"
    hand_realtime_start()

    gains = [100,100,100]
    print "opening hand in realtime mode"
    for i in range(25):
        vels = [-55, -50, -50]
        (strainvals, currentpositions) = hand_realtime_bend(vels, gains)
        print "currentpositions:", currentpositions
        if currentpositions != None and  currentpositions[0] == 0 and currentpositions[1] == 0 and currentpositions[2] == 0:
            break

    print "closing hand in realtime mode"
    for i in range(25):
        vels = [35, 30, 30]
        (strainvals, currentpositions) =  hand_realtime_bend(vels, gains)
        print "currentpositions:", currentpositions
        if currentpositions != None and currentpositions[0]>16000 and currentpositions[1]>16000 and currentpositions[2]>16000:
            break

    print "switching back to hand supervisory mode"
    hand_realtime_terminate()

    print "disconnecting hand"
    hand_disconnect()

if movearm:
    print "sending arm home"
    arm_home()

    print "waiting for trajectory to finish"
    trajectory_wait()

    print "shutting down controllers"
    disable_controllers()

    print "disconnecting arm"
    disconnect_arm()

print "closing the WAM client connection"
close_WAM_client()


