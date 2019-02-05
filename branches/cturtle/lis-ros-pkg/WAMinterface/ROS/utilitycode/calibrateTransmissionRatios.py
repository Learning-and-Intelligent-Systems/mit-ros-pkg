'''Calibrate the transmission ratios for the arm cables
To take data, uncomment mode = "take_data", start roscore, then run WAMinterface/ROS/WAMServerROS.py, then run this script, then run WAMinterface/WAMinterfacelib/socketwamif (as root!!) when prompted to do so.
To compute transmission ratios, uncomment mode = "compute_ratios".
To test the accuracy of the calibration positions, uncomment mode = "test_accuracy"'''

#mode = "take_data"
#mode = "compute_ratios"
mode = "test_accuracy"

import roslib
roslib.load_manifest('WAMinterface')
from WAMClientROSFunctions import *
import scipy
import time
import cPickle as pickle
import pdb
import sys

#pretty-print a 4x4 scipy matrix
def ppmat4tostr(mat):
    printstr = ''
    for i in range(4):
        printstr += ' '.join(['%2.3f' % x for x in mat[i,:].tolist()[0]])+'\n'
    return printstr

#pretty-print a list of doubles to a string
def ppdoublelisttostr(list):
    return ' '.join(['%2.3f' % x for x in list])


#pretty-print a list of ints to a string
def ppintlisttostr(list):
    return ' '.join([str(x) for x in list])


#pretty-print a vector in nx1 scipy matrix form
def ppscipyvecttostr(mat):
    return ' '.join(['%2.3f' % x for x in mat.transpose().tolist()[0]])

#convert from joint0 frame to base frame
joint0_to_base_mat = scipy.matrix([[1,0,0,.22],
                                   [0,1,0,.14],
                                   [0,0,1,.346],
                                   [0,0,0,1]], dtype=float)

#create a 4x4 scipy matrix that rotates about z by angle (rad)
def zrotmat(angle):
    return scipy.matrix([[math.cos(angle), -math.sin(angle), 0, 0],
                         [math.sin(angle), math.cos(angle), 0, 0],
                         [0,0,1,0],
                         [0,0,0,1]])


#get the current palm pos/rot relative to the robot base frame
def getBaseRot():
    print "getting current supposed Cartesian position/orientation"
    (pos, rot) = get_cartesian_pos_and_rot()
    supposedrot4 = scipy.matrix(scipy.eye(4), dtype=float)
    for i in range(3):
        supposedrot4[i,3] = pos[i]
        for j in range(3):
            supposedrot4[i,j] = rot[i*3+j]
    #print ppmat4tostr(supposedrot4)
            
    baserot = joint0_to_base_mat*supposedrot4
    return baserot


#return J2MP, the matrix to go from joint angles to motor angles
def findJ2MP(N):
    J2MP = scipy.matrix(scipy.zeros([7,7]))
    for i in [0,3,6]:
        J2MP[i,i] = -N[i]
    J2MP[1,1] = N[1]
    J2MP[2,1] = -N[1]
    J2MP[1,2] = -N[1]/(N[2]/100.)
    J2MP[2,2] = -N[1]/(N[2]/100.)
    J2MP[4,4] = N[4]
    J2MP[5,4] = N[4]
    J2MP[4,5] = -N[4]/(N[5]/100.)
    J2MP[5,5] = N[4]/(N[5]/100.)
    return J2MP

#return M2JP, the matrix to go from motor angles to joint angles
def findM2JP(N):
    M2JP = scipy.matrix(scipy.zeros([7,7]))
    for i in [0,3,6]:
        M2JP[i,i] = -1./N[i]
    M2JP[1,1] = 1./N[1]/2.
    M2JP[1,2] = -M2JP[1,1]
    M2JP[2,1] = -1./(N[1]/(N[2]/100.))/2.
    M2JP[2,2] = M2JP[2,1]
    M2JP[4,4] = 1./N[4]/2.
    M2JP[4,5] = M2JP[4,4]
    M2JP[5,4] = -1./(N[4]/(N[5]/100.))/2.
    M2JP[5,5] = -M2JP[5,4]
    return M2JP

#print a transform matrix in wam.conf format
def printTransformMatrixToStr(mat):
    printstr = ''
    for i in range(7):
        printstr += "\t<" + ',\t'.join(['%.7f' % x for x in mat[i,:].tolist()[0]])+">,\n"
    printstr = printstr[:-2]
    printstr += '>\n'
    #change 0.0000000 to 0 (barrett's parsing is picky)
    printstr = '<' + printstr[1:].replace("0.0000000", "0")
    return printstr 

#motorangles is a 7x1 scipy matrix, N is a 7-list of transmission ratios
def convertMotorToJoint(motorangles, N):
    M2JP = findM2JP(N)
    return M2JP*motorangles

#jointangles is a 7x1 scipy matrix, N is a 7-list of transmission ratios
def convertJointToMotor(jointangles, N):
    J2MP = findJ2MP(N)
    return J2MP*jointangles


#toss all current data into calibdata.p
def pickle_data(filename):
    pickle.dump([calibrationpositions, calibrationmotorangles, barrettjointangles, barrettpositions], open(filename, "w"))




#calibration sheet origins
sheetorigins = [[.55, 0, .061], [.85, 0, .061], [.55, .28, .061], [.85, .28, .061]]
posespersheet = 9 #nubmer of poses per sheet


#take pose data with the robot
if mode == "take_data":

    #generate calibrationpositions (palm positions/rotations in robot base frame)
    calibrationpositions = []

    for sheetorigin in sheetorigins:
        #x away from thumb, z out of palm
        sidewaysmat = scipy.matrix([[1,0,0,0],
                                    [0,0,1,0],
                                    [0,-1,0,0],
                                    [0,0,0,1]], dtype=float)
        sidewaysorigin = sheetorigin[:]
        sidewaysorigin[2] += .077
        rotations = [0, math.pi/4, -math.pi/4]
        for rotation in rotations:
            calibrationpositions.append(zrotmat(rotation)*sidewaysmat)
            calibrationpositions[-1][0:3, 3] = scipy.matrix(sidewaysorigin).transpose()
        uprightorigin = sheetorigin[:]
        uprightorigin[1] += .107
        uprightorigin[2] += .185
        uprightmat = scipy.matrix([[-1,0,0,0],
                                   [0,1,0,0],
                                   [0,0,-1,0],
                                   [0,0,0,1]], dtype=float)
        rotations = [0, math.pi/4, -math.pi/4, math.pi, math.pi*5/4, math.pi*3/4]
        for rotation in rotations:
            calibrationpositions.append(zrotmat(rotation)*uprightmat)
            calibrationpositions[-1][0:3, 3] = scipy.matrix(uprightorigin).transpose()

    #print "calibrationpositions:"
    #for calibrationposition in calibrationpositions:
    #    print ppmat4tostr(calibrationposition)+"\n"


    #to be filled in
    calibrationmotorangles = [0]*len(calibrationpositions)
    barrettjointangles = [0]*len(calibrationpositions)
    barrettpositions = [0]*len(calibrationpositions)

    datafile = file("calibdata.txt", 'w')

    print "starting the WAM client"
    connect_WAM_client(4321)

    print "connecting to the arm"
    connect_arm()

    print "turning on gravity compensation"
    turn_on_gravity_comp()

    sheetindex = -1
    poseindex = -1

    while(1):
        print "last entered sheet:", sheetindex, "pose:", poseindex
        print "enter a sheet number to record joint angles/motor angles for a calibration location, x to pickle data, move arm home, and exit, s to just pickle current data"
        input = raw_input()
        if 'x' in input:
            break
        elif 's' in input:
            pickle_data("calibdata.p")
            continue
        try:
            sheetindex = int(input.strip())
        except:
            print "not a valid sheet number"
            continue

        if sheetindex < 0 or sheetindex > len(sheetorigins):
            print "sheet number too high"
            continue    
        print "enter a pose number:"
        input = raw_input()
        try:
            poseindex = int(input.strip())
        except:
            print "not a valid pose number"
            continue
        if poseindex < 0 or poseindex > posespersheet:
            print "pose number too high"
            continue
        index = sheetindex*posespersheet + poseindex

        datafile.write("\nsheet"+str(sheetindex)+"pose"+str(poseindex)+"\n")
        print "getting current joint angles"
        jointangles = get_joint_angles()
        if jointangles != None:
            print "joint angles:", ppdoublelisttostr(jointangles)
            barrettjointangles[index] = jointangles
        datafile.write("barrett-reported joint angles:\n")
        datafile.write(ppdoublelisttostr(jointangles)+'\n')

        print "getting current motor angles"
        motorangles = get_motor_angles()
        print "motorangles:", ppdoublelisttostr(motorangles)    
        calibrationmotorangles[index] = motorangles
        datafile.write("motor angles:\n");
        datafile.write(ppdoublelisttostr(motorangles)+'\n')

        baserot = getBaseRot()
        barrettpositions[index] = baserot
        print "barrett-reported rotmat relative to robot base:\n", ppmat4tostr(baserot)
        print "actual rotmat relative to robot base:\n", ppmat4tostr(calibrationpositions[index])
        print "barrett-reported position relative to robot base:", ppscipyvecttostr(baserot[0:3,3])
        print "actual position relative to robot base:          ", ppscipyvecttostr(calibrationpositions[index][0:3, 3])
        datafile.write("barrett-reported position/orientation:\n")
        datafile.write(ppmat4tostr(baserot)+'\n')
        datafile.write("actual position/orientation:\n")
        datafile.write(ppmat4tostr(calibrationpositions[index])+'\n')


    print "pickling data and sending arm home"
    pickle_data("calibdata.p")
    arm_home()
    trajectory_wait()
    disable_controllers()

    print "closing the WAM client connection: idle the robot and press enter in the socketwamif window"
    close_WAM_client()



#compute best-fit transmission ratios (assumes calibdata.p exists)
if mode == "compute_ratios":

    from scipy.optimize import *
    import wamik

    #optimization function
    def optfunc(N):
        penalty = 0.
        if verbose:
            rotdiffarray = []
            posdiffarray = []
        for index in range(len(calibrationpositions)):
            #if index%9<=2: #just upright
            #if index%9>2:  #just sideways
            #    continue
            motorangles = calibrationmotorangles[index]
            palmmat = calibrationpositions[index]
            
            if type(motorangles) != int:
                jointangles = convertMotorToJoint(scipy.matrix(motorangles).transpose(), N)
                jointangleslist = jointangles.transpose().tolist()[0]
                fkresultpalmmat = wamik.mat4(wamik.run_fk(jointangleslist))
                fkbasepalmmat = joint0_to_base_mat * fkresultpalmmat

            rotanglediff = wamik.rotangle(fkbasepalmmat[0:3,0:3], palmmat[0:3,0:3])
            if rotanglediff > .5: #clearly got the wrong pose
                continue
            posdiff = fkbasepalmmat[0:3,3]-palmmat[0:3,3]
            posdiffmagsq = (posdiff.transpose()*posdiff)[0,0]
            pospenalty = posfact * posdiffmagsq 
            rotpenalty = rotfact * rotanglediff**2
            penalty += pospenalty + rotpenalty

            if verbose:
                #print "fkbasepalmmat:\n", ppmat4tostr(fkbasepalmmat)
                #print "actualrot:\n", ppmat4tostr(palmmat)
                #print "rotanglediff:", rotanglediff
                #print "posdiff:", ppscipyvecttostr(posdiff)
                #print "posdiffmagsq:", posdiffmagsq
                #print "posdiffmag:", posdiffmagsq**.5
                #print "pospenalty:", pospenalty
                #print "rotpenatly:", rotpenalty
                posdiffarray.append(posdiffmagsq**.5)
                rotdiffarray.append(rotanglediff)
                #raw_input()
        #print "N:", ppdoublelisttostr(N)
        #print "N:", N
        #print "total penalty:", penalty
        #raw_input()
        if verbose:
            print "posdiffarray:", ppdoublelisttostr(posdiffarray)
            print "rotdiffarray:", ppdoublelisttostr(rotdiffarray)
            print "average pos diff:%.3f"%(sum(posdiffarray)/len(posdiffarray))
            print "average rot diff:%.3f"%(sum(rotdiffarray)/len(rotdiffarray))

        return penalty


    #run optimization (cobyla is a constraint-based optimizer, which is excessive given the lack of
    #constraints, but it happens to be one I'm used to using)
    def run_transmission_ratio_opt(startN):
        answer = fmin_cobyla(optfunc, startN, [], rhobeg = .1, rhoend = .0001, iprint = 1, maxfun = 100000)
        return answer

    print "initializing wamik library"
    wamik.init_wamik()

    print "unpickling data"
    #[calibrationpositions, calibrationmotorangles, barrettjointangles, barrettpositions] = pickle.load(open("calibdata.p", "r"))
    [calibrationpositions, calibrationmotorangles, barrettjointangles, barrettpositions] = pickle.load(open("correctedcalibdata.p", "r"))

    posfact = 10000   #factor to multiply position differences
    rotfact = 100   #factor to multiply rotation differences
    verbose = 0

    #starting estimate of transmission ratios 
    #(N and n from wam.conf: N[2] usually == N[1], so it's been replaced by 100*n[2] for conciseness; likewise with N[5] and 100*n[5]) 
    #(factor of 100 is so that step sizes for N[2] and N[5] step sizes are comparable to the rest)
    #startN = [42., 28.25, 1.68*100., 18., 9.4796, 1*100., 14.93]  #original defaults
    startN = [41.495, 27.933, 167.595, 17.951, 9.488, 99.641, 14.894]

    m2jp = findM2JP(startN)

    verbose = 1
    print "before optimization:"
    optfunc(startN)
    verbose = 0
    print "running transmission ratio optimization"
    newN = run_transmission_ratio_opt(startN)
    print "new transmission ratios:", ppdoublelisttostr(newN)

    verbose = 1
    print "after optimization:"
    optfunc(newN)

    m2jp = findM2JP(newN)
    print "copy these into your wam.conf (and comment out the old):"
    print "N = <%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f>"%(newN[0],newN[1],newN[1],newN[3],newN[4],newN[4],newN[6])
    print "n = <0, 0, %.3f, 0, 0, %.3f, 0>"%(newN[2]/100., newN[5]/100.)
    print "m2jp =", printTransformMatrixToStr(m2jp)
    j2mp = findJ2MP(newN)
    print "j2mp =", printTransformMatrixToStr(j2mp)


'''
my results:
without adjusting z positions:
just vertical poses:
new transmission ratios: 41.985 27.934 168.333 17.931 9.500 101.055 14.932

before optimization:
posdiffarray: 0.001 0.004 0.004 0.006 0.009 0.009 0.006 0.006 0.009 0.012 0.012 0.013 0.002 0.003 0.005 0.009 0.007 0.009 0.009 0.007 0.012 0.013 0.010 0.011
rotdiffarray: 0.031 0.036 0.048 0.045 0.053 0.049 0.037 0.045 0.043 0.033 0.037 0.043 0.050 0.051 0.026 0.044 0.047 0.040 0.035 0.047 0.040 0.049 0.047 0.042
average pos diff: 0.008
average rot diff: 0.042

after optimization:
posdiffarray: 0.004 0.004 0.002 0.003 0.006 0.004 0.004 0.001 0.002 0.006 0.006 0.008 0.005 0.006 0.002 0.006 0.005 0.005 0.004 0.005 0.006 0.006 0.003 0.004
rotdiffarray: 0.015 0.019 0.031 0.026 0.039 0.037 0.028 0.025 0.045 0.028 0.021 0.020 0.032 0.034 0.019 0.030 0.036 0.023 0.021 0.021 0.021 0.036 0.026 0.024
average pos diff: 0.004
average rot diff: 0.027


just sideways poses:
new transmission ratios: 41.309 27.929 167.139 17.984 9.451 98.525 14.742

before optimization:
posdiffarray: 0.006 0.008 0.006 0.011 0.017 0.003 0.003 0.014 0.011 0.016
rotdiffarray: 0.060 0.060 0.014 0.025 0.049 0.074 0.043 0.035 0.044 0.059
average pos diff:0.009
average rot diff:0.046

after optimization:
posdiffarray: 0.002 0.005 0.004 0.003 0.003 0.003 0.004 0.002 0.005 0.006
rotdiffarray: 0.050 0.076 0.027 0.034 0.036 0.036 0.043 0.025 0.054 0.048
average pos diff:0.004
average rot diff:0.043


#all poses:
new transmission ratios: 41.495 27.933 167.595 17.951 9.488 99.641 14.894

before optimization:
posdiffarray: 0.006 0.008 0.006 0.001 0.004 0.004 0.006 0.009 0.009 0.011 0.017 0.006 0.006 0.009 0.012 0.012 0.013 0.003 0.003 0.002 0.003 0.005 0.009 0.007 0.009 0.014 0.011 0.016 0.009 0.007 0.012 0.013 0.010 0.011
rotdiffarray: 0.060 0.060 0.014 0.031 0.036 0.048 0.045 0.053 0.049 0.025 0.049 0.037 0.045 0.043 0.033 0.037 0.043 0.074 0.043 0.050 0.051 0.026 0.044 0.047 0.040 0.035 0.044 0.059 0.035 0.047 0.040 0.049 0.047 0.042
average pos diff:0.008
average rot diff:0.044

after optimization:
posdiffarray: 0.006 0.008 0.004 0.005 0.005 0.003 0.004 0.006 0.004 0.004 0.005 0.002 0.004 0.003 0.007 0.008 0.010 0.003 0.006 0.006 0.006 0.003 0.004 0.003 0.003 0.003 0.006 0.006 0.006 0.004 0.008 0.006 0.004 0.005
rotdiffarray: 0.052 0.061 0.026 0.019 0.021 0.040 0.034 0.035 0.031 0.047 0.047 0.033 0.030 0.046 0.036 0.026 0.034 0.042 0.042 0.030 0.028 0.020 0.025 0.024 0.022 0.046 0.052 0.048 0.017 0.027 0.023 0.032 0.022 0.033
average pos diff:0.005
average rot diff:0.034

N = <41.495, 27.933, 27.933, 17.951, 9.488, 9.488, 14.894>
n = <0, 0, 1.676, 0, 0, 0.996, 0>
m2jp = <<-0.0240992,	0,	0,	0,	0,	0,	0>,	
<0,	0.0178997,	-0.0178997,	0,	0,	0,	0>,	
<0,	-0.0299990,	-0.0299990,	0,	0,	0,	0>,	
<0,	0,	0,	-0.0557062,	0,	0,	0>,	
<0,	0,	0,	0,	0.0526984,	0.0526984,	0>,	
<0,	0,	0,	0,	-0.0525091,	0.0525091,	0>,	
<0,	0,	0,	0,	0,	0,	-0.0671406>>

j2mp = <<-41.4950940,	0,	0,	0,	0,	0,	0>,	
<0,	27.9334928,	-16.6672372,	0,	0,	0,	0>,	
<0,	-27.9334928,	-16.6672372,	0,	0,	0,	0>,	
<0,	0,	0,	-17.9513154,	0,	0,	0>,	
<0,	0,	0,	0,	9.4879521,	-9.5221543,	0>,	
<0,	0,	0,	0,	9.4879521,	9.5221543,	0>,	
<0,	0,	0,	0,	0,	0,	-14.8941279>>



with adjusting z positions:
all poses:
before optimization:
posdiffarray: 0.009 0.010 0.014 0.011 0.010 0.011 0.011 0.011 0.011 0.015 0.017 0.011 0.012 0.013 0.013 0.015 0.014 0.006 0.011 0.011 0.011 0.013 0.011 0.011 0.011 0.014 0.019 0.010 0.009 0.012 0.010 0.013 0.012
rotdiffarray: 0.067 0.101 0.037 0.039 0.040 0.037 0.037 0.036 0.019 0.071 0.069 0.051 0.036 0.042 0.041 0.036 0.029 0.057 0.111 0.043 0.043 0.052 0.050 0.041 0.041 0.040 0.045 0.039 0.042 0.026 0.035 0.025 0.031
average pos diff:0.012
average rot diff:0.046

new transmission ratios: 41.215 27.805 167.328 17.805 9.498 99.722 14.868

after optimization:
posdiffarray: 0.013 0.010 0.017 0.005 0.004 0.002 0.007 0.007 0.005 0.012 0.012 0.007 0.008 0.010 0.008 0.011 0.009 0.011 0.013 0.006 0.007 0.007 0.007 0.007 0.007 0.008 0.011 0.008 0.007 0.009 0.006 0.009 0.008
rotdiffarray: 0.063 0.096 0.041 0.036 0.026 0.036 0.039 0.020 0.024 0.084 0.068 0.041 0.024 0.041 0.023 0.029 0.012 0.077 0.118 0.048 0.040 0.054 0.037 0.039 0.038 0.057 0.045 0.035 0.030 0.023 0.033 0.021 0.021
average pos diff:0.009
average rot diff:0.043


N = <41.215, 27.805, 27.805, 17.805, 9.498, 9.498, 14.868>
n = <0, 0, 1.673, 0, 0, 0.997, 0>
m2jp = <<-0.0242630,    0,      0,      0,      0,      0,      0>,
        <0,     0.0179822,      -0.0179822,     0,      0,      0,      0>,
        <0,     -0.0300893,     -0.0300893,     0,      0,      0,      0>,
        <0,     0,      0,      -0.0561636,     0,      0,      0>,
        <0,     0,      0,      0,      0.0526429,      0.0526429,      0>,
        <0,     0,      0,      0,      -0.0524968,     0.0524968,      0>,
        <0,     0,      0,      0,      0,      0,      -0.0672598>>

j2mp = <<-41.2149729,   0,      0,      0,      0,      0,      0>,
        <0,     27.8053291,     -16.6172263,    0,      0,      0,      0>,
        <0,     -27.8053291,    -16.6172263,    0,      0,      0,      0>,
        <0,     0,      0,      -17.8051185,    0,      0,      0>,
        <0,     0,      0,      0,      9.4979574,      -9.5243930,     0>,
        <0,     0,      0,      0,      9.4979574,      9.5243930,      0>,
        <0,     0,      0,      0,      0,      0,      -14.8677135>>



'''


#correct the z-positions of the recorded poses when asking the robot to hold angles (instead of gravity comp)
#assumes calibdata.p exists already 
#put the new transmission ratios in newN (in case the ratios have changed since the data was taken)
if mode == "test_accuracy":
    #newN = [41.495, 27.933, 167.595, 17.951, 9.488, 99.641, 14.894]
    newN = [41.215, 27.805, 167.328, 17.805, 9.498, 99.722, 14.868]

    print "unpickling data"
    #[calibrationpositions, calibrationmotorangles, barrettjointangles, barrettpositions] = pickle.load(open("calibdata.p", "r"))
    [calibrationpositions, calibrationmotorangles, barrettjointangles, barrettpositions] = pickle.load(open("correctedcalibdata.p", "r"))

    print "starting the WAM client"
    connect_WAM_client(4321)

    print "connecting to the arm"
    connect_arm()

    print "turning on gravity compensation"
    turn_on_gravity_comp()

    justabovehome = [0, -1.99, 0, 2.4, 0, 0.25, 0]
    print "moving to just above home"
    move_to_joint(justabovehome)
    trajectory_wait()

    sheetindex = -1
    poseindex = -1

    while(1):
        print "last entered sheet:", sheetindex, "pose:", poseindex
        print "enter a sheet number to move the robot, p to print the current robot position,  x to move the arm home and quit, z to correct the height of the last index, s to pickle the corrected data, or c to release the controllers and return to gravity comp mode:"
        input = raw_input()
        if 'x' in input:
            break
        if 'c' in input:
            disable_controllers()
            continue
        if 'p' in input:
            currentjointangles = get_joint_angles()
            print "jointangles: ", ppdoublelisttostr(currentjointangles)
            baserot = getBaseRot()
            print "barrett-reported rotmat relative to robot base:\n", ppmat4tostr(baserot)            
            continue
        if 's' in input:
            print "pickling data"
            pickle_data("correctedcalibdata.p")
            continue
        if 'z' in input:
            if index > len(calibrationpositions) or type(calibrationmotorangles[index]) == int:
                print "last index wasn't valid!"
                continue
            print "enter new z value (meters):"
            input = raw_input()
            try:
                newz = float(input.strip())
            except:
                print "not a valid z value!"
                continue
            print "old position:\n", ppmat4tostr(calibrationpositions[index])
            calibrationpositions[index][2,3] = newz
            print "new position:\n", ppmat4tostr(calibrationpositions[index])
            continue
        try:
            sheetindex = int(input.strip())
        except:
            print "not a valid sheet number"
            continue

        if sheetindex < 0 or sheetindex > len(sheetorigins):
            print "sheet number too high"
            continue    
        print "enter a pose number:"
        input = raw_input()
        try:
            poseindex = int(input.strip())
        except:
            print "not a valid pose number"
            continue
        if poseindex < 0 or poseindex > posespersheet:
            print "pose number too high"
            continue
        index = sheetindex*posespersheet + poseindex

        motorangles = calibrationmotorangles[index]
        palmmat = calibrationpositions[index]
        
        if type(motorangles) == int:
            print "pose not recorded!  Continuing."
            continue
        
        #translate the recorded motorangles to new joint angles using the new transmission ratios
        try:
            jointangles = convertMotorToJoint(scipy.matrix(motorangles).transpose(), newN)
        except:
            pdb.set_trace()
        jointangleslist = jointangles.transpose().tolist()[0]

        #send the robot to the joint angles
        move_to_joint(jointangleslist)
        trajectory_wait()

        #figure out where the robot thinks it is, as well as where it meant to go (you'll have to measure to find out where it actually is)
        print "recorded jointangles: ", ppdoublelisttostr(barrettjointangles[index])
        print "translated jointangles:", ppdoublelisttostr(jointangles)
        currentjointangles = get_joint_angles()
        print "actual jointangles: ", ppdoublelisttostr(currentjointangles)

        baserot = getBaseRot()
        print "barrett-reported rotmat relative to robot base:\n", ppmat4tostr(baserot)
        print "recorded rotmat relative to robot base:\n", ppmat4tostr(palmmat)
        print "barrett-reported position relative to robot base:", ppscipyvecttostr(baserot[0:3,3])
        print "recorded position relative to robot base:          ", ppscipyvecttostr(palmmat[0:3, 3])

    print "pickling final data"
    pickle_data("correctedcalibdata.p")

    justabovehome = [0, -1.99, 0, 2.4, 0, 0.25, 0]
    print "moving to just above home"
    move_to_joint(justabovehome)
    trajectory_wait()

    print "sending arm home"
    arm_home()
    trajectory_wait()
    disable_controllers()

    print "closing the WAM client connection: idle the robot and press enter in the socketwamif window"
    close_WAM_client()

