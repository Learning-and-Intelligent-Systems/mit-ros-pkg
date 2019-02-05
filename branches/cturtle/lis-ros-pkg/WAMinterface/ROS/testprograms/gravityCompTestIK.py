'''test IK while moving the robot around in gravity compensation mode
First start roscore, then run WAMinterface/ROS/WAMServerROS.py, then run this script, then run WAMinterface/WAMinterfacelib/socketwamif (as root!!) when prompted to do so.'''

import roslib
import time
roslib.load_manifest('WAMinterface')

from WAMClientROSFunctions import *

#pretty-print a 4x4 matrix in list form
def ppmat4(mat):
    for i in range(4):
        print ' '.join(['%2.3f' % x for x in mat[i*4:(i+1)*4]])
    print '\n'

#pretty-print a list of doubles to a string
def pplisttostr(list):
    return ' '.join(['%2.3f' % x for x in list])

#pretty-print a vector in nx1 scipy matrix form
def ppscipyvecttostr(mat):
    return ' '.join(['%2.3f' % x for x in mat.transpose().tolist()[0]])

#create a 4x4 scipy matrix from list
def mat4(list):
    newmat = scipy.matrix(scipy.zeros([4,4],dtype=float))
    for i in range(4):
        for j in range(4):
            newmat[i,j] = list[i*4+j]
    return newmat

#convert from joint0 frame to base frame
joint0_to_base_mat = scipy.matrix([[1,0,0,.22],
                                   [0,1,0,.14],
                                   [0,0,1,.346],
                                   [0,0,0,1]], dtype=float)


print "starting the WAM client"
connect_WAM_client(4321)

print "connecting to the arm"
connect_arm()

print "turning on gravity compensation"
turn_on_gravity_comp()

while(1):
    print "press enter to compare FK and IK to current pos/angles, x to move arm home and exit"
    input = raw_input()
    if 'x' in input:
        break

    print "getting current joint angles"
    jointangles = get_joint_angles()
    if jointangles != None:
        print "joint angles:", ppdoublearray(jointangles)

    print "getting current Cartesian position/orientation"
    (pos, rot) = get_cartesian_pos_and_rot()
    actualrot4 = [0]*16
    for i in range(3):
        actualrot4[i*4+3] = pos[i]
        for j in range(3):
            actualrot4[i*4+j] = rot[i*3+j]
    ppmat4(actualrot4)

    print "forward kinematics thinks:"
    palmrot = forward_kinematics(jointangles)
    ppmat4(palmrot)

    scipypalmrot4 = mat4(palmrot)
    basepalmmat = joint0_to_base_mat * scipypalmrot4
    print "position relative to robot base:", ppscipyvecttostr(basepalmmat[0:3,3])

    print "analytical inverse kinematics came up with:"
    zeroangles = [0]*7
    resultangles = inverse_kinematics(actualrot4, jointangles)
    #resultangles = inverse_kinematics(palmrot, jointangles)
    if resultangles != None:
        print "resultangles:", pplisttostr(resultangles)
        resultpalmmat = forward_kinematics(resultangles)
        print "\nresultpalmmat:"
        ppmat4(resultpalmmat)
    else:
        print "no analytical IK solution found!"

    #optimization IK is much less accurate than analytical, but good for getting more or less to barely-out-of-reach goals that analytical IK won't find solutions for.  Or for adding additional constraints.  
    print "optimization inverse kinematics came up with:"
    optresultangles = optimization_inverse_kinematics(actualrot4, jointangles)
    print "optresultangles:", pplisttostr(optresultangles)
    optresultpalmmat = forward_kinematics(optresultangles)
    print "\noptresultpalmmat:"
    ppmat4(optresultpalmmat)

print "sending arm home"
arm_home()
trajectory_wait()
disable_controllers()

print "closing the WAM client connection: idle the robot and press enter in the socketwamif window"
close_WAM_client()



