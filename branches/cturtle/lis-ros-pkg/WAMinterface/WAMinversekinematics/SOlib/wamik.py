'''ctypes bindings for the C analytical forward/inverse kinematics functions, and optimization IK using scipy's fmin_cobyla.  
author: Kaijen Hsiao (kaijenhsiao@gmail.com)'''

from ctypes import *
import scipy
import os
from scipy.optimize import *
import pdb
import math

#create a Ctypes array of type 'type' ('double' or 'int')
def createCtypesArr(type, size):
    if type == 'int':
        arraytype = c_int * size
    else:
        arraytype = c_double * size
    return arraytype()


#pretty-print a 4x4 matrix
def ppmat4(mat):
    for i in range(4):
        print ' '.join(['%2.3f' % x for x in mat[i*4:(i+1)*4]])
    print '\n'


#pretty-print a list of doubles to a string
def pplisttostr(list):
    return ' '.join(['%2.3f' % x for x in list])


#flatten a list of lists
def flatten(lists):
    listscombined = []
    for list in lists:
        listscombined.extend(list)
    return listscombined


#create a 4x4 scipy matrix from list
def mat4(list):
    newmat = scipy.matrix(scipy.zeros([4,4],dtype=float))
    for i in range(4):
        for j in range(4):
            newmat[i,j] = list[i*4+j]
    return newmat


#return a scipy matrix of dimension dims filled with 0s
def zeromat(dims):
    return scipy.matrix(scipy.zeros(dims, dtype=float))


#return an identity matrix of dimension sizexsize 
def eyemat(size):
    return scipy.matrix(scipy.eye(size, dtype=float))


#find the rotation angle between the two rotation matrices
def rotangle(mat1, mat2):
    mat1to2 = mat2 * mat1.transpose()
    #print "mat1to2:\n", mat1to2
    tr = mat1to2[0,0]+mat1to2[1,1]+mat1to2[2,2]
    if(tr>3.0):
        tr=3.0
    elif(tr<-3.0):
        tr=-3.0
    angle = math.acos((tr-1.0)/2.0)
    return angle




#analytical IK


#run this first to use run_wam_ik or run_wam_fk
def init_wamik():
    global wamiklib
    #load the dll (Windows) or so (linux) as wamik
    if os.sys.platform == 'win32':
        wamiklib = cdll.wamik
    else:
        cdll.LoadLibrary("libc.so.6")
        wamiklib = CDLL("./wamik.so.1.0")

    #create the ctypes arrays
    global ikrot4, ikcurrentangles, ikresultangles, theta, handbasemat, elbowmat, wristmat
    ikrot4 = createCtypesArr('double', 16)
    ikcurrentangles = createCtypesArr('double', 7)
    ikresultangles = createCtypesArr('double', 7)

    theta = createCtypesArr('double', 7)
    handbasemat = createCtypesArr('double', 16)
    elbowmat = createCtypesArr('double', 16)
    wristmat = createCtypesArr('double', 16)


#run inverse kinematics (first run init_wamik)
#palmrot4 is a 16-list (4x4 homogeneous transformation mat in row order)
#for the palm frame (position in meters, center of Barrett Hand palm) 
#currentangles are angles to try to stay close to
#returns the result angles or None if no IK solution was found 
def run_ik(palmrot4, currentangles):
    global ikrot4, ikcurrentangles, ikresultangles

    #translate palmrot4 to the hand base frame
    handbasetopalm = mat4([1,0,0,0,
                           0,1,0,0,
                           0,0,1,-.1,
                           0,0,0,1])
    palmrotscipy = mat4(palmrot4)
    handbaserot = palmrotscipy * handbasetopalm

    #initialize the ctypes arrays
    ikrot4[:] = flatten(handbaserot.tolist())
    ikcurrentangles[:] = [0]*7

    #run inverse kinematics
    result = wamiklib.run_wam_ik(ikrot4, ikcurrentangles, ikresultangles)
    if result:
        return list(ikresultangles)
        #print "currentangles:", pplisttostr(currentangles)
        #print "resultangles:", pplisttostr(resultangles)

    else:
        print "no IK solution found!"
        return None


#run forward kinematics (origin at joint0 origin, Barrett coordinates (z up, x forward, y left))
#palm matrix is centered at the center of the Barrett Hand palm
#handbasemat point is at the base of the hand (.1 m behind palm surface)
#set return_other_mats to 1 to return the handbase, elbow, and wrist matrices
#inputs angles
#outputs palm matrix as 16-list (4x4 mat in row order)
def run_fk(angles, return_other_mats = 0):
    global theta, handbasemat, elbowmat, wristmat

    #initialize the input ctypes arrays
    theta[:] = angles
    handbasemat[:] = [0]*16
    elbowmat[:] = [0]*16
    wristmat[:] = [0]*16

    #run forward kinematics to find an appropriate pos and rot for the hand base
    wamiklib.run_wam_fk(theta, handbasemat, elbowmat, wristmat)
 
    #translate to palm
    palmtohandbase = mat4([1,0,0,0,
                           0,1,0,0,
                           0,0,1,.1,
                           0,0,0,1])
    handbasescipymat = mat4(handbasemat)

    palmmat = handbasescipymat * palmtohandbase
    palmmatlist = flatten(palmmat.tolist())

    if return_other_mats:
        return (palmmatlist, list(handbasemat), list(elbowmat), list(wristmat))
    else:
        return palmmatlist



#optimization IK

posfact = 1.0   #factor to multiply position differences
rotfact = 1.0   #factor to multiply rotation differences
desiredpalmrot4 = eyemat(4) #desired 4x4 rotation matrix of palmmovepoint
palmmovepoint = [0,0,0]     #point relative to palm origin

armlostops = [-2.66, -1.942, -2.704, -0.842, -4.811, -1.633, -2.2]
armhistops = [ 2.66,  1.962,  2.904,  3.117,  1.261,  1.508,  2.2]

#x and startarmangs are 7-lists of arm angles
def IKfunc(x, startarmangs, verbose = 0):
    penalty = 0.
    
    palmrot = run_fk(x)
    palmrot = mat4(palmrot)
    pointglobal = palmrot*scipy.matrix([palmmovepoint[0], palmmovepoint[1], palmmovepoint[2], 1]).transpose()
    #palmrot = barm.forwardKinematics(x)
    #palmpos = vec4to3(palmrot.getColumn(3))
    #pointglobal = palmrot.getMat3() * vec3(palmmovepoint) + palmpos
    
    #penalty for position differences
    posdiff = pointglobal - desiredpalmrot4[0:4, 3]

    #penalty is magnitude of diff squared * posfact
    penalty += (posdiff.transpose()*posdiff)[0,0] * posfact
    #penalty += (abs(posdiff)*posfact)**2
    if verbose:
        print "\n\nx:", x
        print "pointglobal:", pointglobal.transpose()
        print "desiredpos:", desiredpalmrot4[0:4, 3].transpose()
        print "posdiff:", posdiff.transpose()
        print "penalty:", penalty
    
    #penalty for rotation differences
    anglediff = rotangle(desiredpalmrot4[0:3,0:3], palmrot[0:3,0:3])
    #(anglediff, axis) = findRelRotation(desiredrot, palmrot.getMat3())
    if verbose:
        print "desiredrot\n", desiredpalmrot4[0:3,0:3]
        print "palmrot\n", palmrot
        print "anglediff:", anglediff
    anglepenalty = anglediff**2 * rotfact
    penalty += anglepenalty
    #penalty += (anglediff*rotfact)**2

    
    #stay close to startarmangs, all else being equal
    angdiff = (scipy.array(x)-scipy.array(startarmangs)).sum()
    penalty += .001*angdiff
    if verbose:
        print "anglediff:", anglediff
        print "anglepenalty:", anglepenalty, "\n\n"
        raw_input()    
    return penalty


#constraints (joint limits)
def IKcons(x, args):
    cons = 0.
    for armjoint in range(7):
        if x[armjoint] < armlostops[armjoint]:
            cons -= armlostops[armjoint] - x[armjoint]
        elif x[armjoint] > armhistops[armjoint]:
            cons -= x[armjoint] - armhistops[armjoint]
    return cons


#run optimization IK 
#(gets stuck in local minima occasionally, but quite good if you start sort of nearby, especially for finding as-close-as-it-gets solutions to barely-out-of-reach goals)
#palmrot4 is a 16-list (4x4 homogeneous transformation mat in row order)
#for the palm frame (position in meters, center of Barrett Hand palm) 
#startarmangs is a 7-list of arm angles to stay close to
#relpointtopalm is the point relative to the palm origin that you want to be at pos (3x1 scipy matrix)
#set verbose to 1 to make cobyla print everything
#maxfun is the max number of iterates for cobyla
def run_opt_ik(palmrot4, startarmangs = [0]*7, relpointtopalm = [0,0,0], verbose = 0, maxfun = 100000):
    global desiredpalmrot4
    global palmmovepoint
    global posfact
    global rotfact
    posfact = 1.0
    rotfact = 1.0
    palmmovepoint = relpointtopalm
    desiredpalmrot4 = mat4(palmrot4)
    answer = fmin_cobyla(IKfunc, startarmangs, [IKcons], args = (startarmangs,), rhobeg = [.01]*7, \
                     rhoend = 0.0001, iprint = verbose, maxfun = maxfun)
    #print answer
    return answer
