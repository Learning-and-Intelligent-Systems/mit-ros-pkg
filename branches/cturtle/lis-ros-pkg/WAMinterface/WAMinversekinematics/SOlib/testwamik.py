'''test program for wamik.py (both analytical and optimziation IK)
author: Kaijen Hsiao (kaijenhsiao@gmail.com)'''

import wamik
import scipy

#init
wamik.init_wamik()

#run forward kinematics to find an appropriate pos and rot
#should be [.31, 0, .42]
theta = [0.00, -0.32, 0.01, 2.45, 0.06, -0.59, -0.05]
print "angles:", wamik.pplisttostr(theta)

palmmat = wamik.run_fk(theta)
print "\npalmmat:"
wamik.ppmat4(palmmat)

#run inverse kinematics
currentangles = [0]*7
#analytical IK
resultangles = wamik.run_ik(palmmat, currentangles)

#optimization (need to start nearby-ish)
#currentangles = (scipy.array(theta) + scipy.randn(7)*.1).tolist()
optresultangles = wamik.run_opt_ik(palmmat, currentangles)

#run forward kinematics again to check the resulting angles
if resultangles != None:
    print "resultangles: ", wamik.pplisttostr(resultangles)

    resultpalmmat = wamik.run_fk(resultangles)
    print "\nresultpalmmat:"
    wamik.ppmat4(resultpalmmat)

else:
    print "No analytical IK solution found!"

print "optresultangles: ", wamik.pplisttostr(optresultangles)
optresultpalmmat = wamik.run_fk(optresultangles)
print "\noptresultpalmmat:"
wamik.ppmat4(optresultpalmmat)
