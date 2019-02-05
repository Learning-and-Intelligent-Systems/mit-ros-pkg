'''put the hand in home position and move the arm home, when the WAM is already running in another script but it's stopped due to some bug
or, if not running already, and you're in home position but not at the right 
hand position, run the script with 'python moveHome.py handhome' to
put the hand in the right position'''

import roslib
import time
import sys
roslib.load_manifest('WAMinterface')

from WAMClientROSFunctions import *

if len(sys.argv) >= 2:
    if sys.argv[1] == 'handhome':
        print "connecting arm and hand"
        connect_WAM_client(4321)
        connect_arm()
        hand_connect()
        turn_on_gravity_comp()

else:
    #if you're already connected and out there, open the fingers first
    print "opening hand"
    for i in range(2):
        hand_send_raw_cmd('GO')


torquelimits = [7.75,7.75,7.75,7.75,2.5,2.5,2]
print "setting torque limits to", torquelimits
set_torque_limits(torquelimits)

justabovehome = [0, -1.99, 0, 2.4, 0, 0.25, 0]
print "moving to just above home"
move_to_joint(justabovehome)
trajectory_wait()

for i in range(2):
    print "opening hand"
    hand_send_raw_cmd('GO')

    print "closing spread"
    hand_send_raw_cmd('SC')

print "sending arm home"
arm_home()
trajectory_wait()

print "shutting down"
close_WAM_client()
