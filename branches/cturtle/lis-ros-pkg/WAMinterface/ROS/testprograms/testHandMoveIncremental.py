'''test the functionality of hand_move_incremental'''

import roslib
import math
roslib.load_manifest('WAMinterface')

from WAMClientROSFunctions import *

connect_WAM_client(4321)
hand_connect()
connect_arm()
turn_on_gravity_comp()

justabovehome = [0, -1.99, 0, 2.4, 0, 0.25, 0]
print "moving to just above home"
move_to_joint(justabovehome)
trajectory_wait()

#open fingers and close spread to start
hand_send_raw_cmd('GO')
hand_send_raw_cmd('SC')

#angle corresponding to a position of 15600 (my preferred bend close target)
bendCTang = 2.17817
maxincrement = .05

#goals to test out the various features of hand_move_incremental
goalanglesandincrements = \
    [([math.pi, bendCTang, bendCTang, bendCTang], None, \
          "just close fingers all at once"),
     ([0, 0, 0, 0], None, \
          "open fingers all at once, then open spread"),
     ([0, math.pi/3, math.pi/2, math.pi/2], maxincrement, \
          "close all fingers together by increments of maxincrement until F1 reaches PI/3, then alternate moving F2 and F3 to PI/2 by maxincrement increments"),
     ([0, 0, 0, 0], maxincrement, \
          "open all fingers together until F1 is fully open, then move F2 and F3 alternately by increments"),
     ([0, math.pi/3, math.pi/2, math.pi/2.5], maxincrement, \
          "close all fingers together until F1 reaches PI/3, then move F2 and F3 alternately to PI/2 and PI/2.5 by increments"), 
     ([math.pi, math.pi/4, math.pi/4, math.pi/4], None, \
          "move all fingers to PI/4 at once, then close spread"), 
     ([math.pi, 0, 0, 0], None, "back to home position")]


#run!
for (goalangles, increment, msg) in goalanglesandincrements:
    print "about to", msg
    print "press enter twice to continue"
    for i in range(2):
        raw_input()

    #moving all at once
    if increment == None:
        hand_move_incremental(goalangles)
    else:

        #run for no more than 35 steps, until you're there
        steps = math.floor(math.pi/maxincrement)
        notthereyet = [True, True, True, True]
        print "goalangles:", ppdoublearray(goalangles)
        while any(notthereyet) and steps > 0:
            currentangs = hand_get_finger_angles()
            print "currentangs:", ppdoublearray(currentangs[0:4])
            notthereyet = [math.fabs(d-c)>.02 for (d,c) in zip(goalangles, currentangs)]
            print "notthereyet:", notthereyet
            steps -= 1
            hand_move_incremental(goalangles, maxincrement = increment, anglethreshold = .02, currentfingerangs = currentangs)

print "sending arm home"
arm_home()
trajectory_wait()

print "shutting down"
close_WAM_client()
