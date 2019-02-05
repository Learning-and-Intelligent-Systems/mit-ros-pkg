#! /usr/bin/python
import roslib
roslib.load_manifest('mm_test')

import rospy
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *

POS_CLOSE = 0
POS_OPEN = .08

rospy.init_node('move_the_gripper', anonymous=True)

client_l = actionlib.SimpleActionClient(
    'l_gripper_controller/gripper_action', Pr2GripperCommandAction)
client_r = actionlib.SimpleActionClient(
    'r_gripper_controller/gripper_action', Pr2GripperCommandAction)

client_l.wait_for_server()
client_r.wait_for_server()

while True:
    print "press 1 to open left gripper"
    print "press 2 to close left gripper"
    print "press 3 to open right gripper"
    print "press 4 to close right gripper"
    print "press q to exit"

    input = raw_input()

    if input == '1':
        client_l.send_goal(Pr2GripperCommandGoal(
                Pr2GripperCommand(position = POS_OPEN, max_effort = -1)))
        client_l.wait_for_result()
    elif input == '2':
        client_l.send_goal(Pr2GripperCommandGoal(
                Pr2GripperCommand(position = POS_CLOSE, max_effort = -1)))
        client_l.wait_for_result()
    elif input == '3':
        client_r.send_goal(Pr2GripperCommandGoal(
                Pr2GripperCommand(position = POS_OPEN, max_effort = -1)))
        client_r.wait_for_result()
    elif input == '4':
        client_r.send_goal(Pr2GripperCommandGoal(
                Pr2GripperCommand(position = POS_CLOSE, max_effort = -1)))
        client_r.wait_for_result()
    elif input == 'q':
        break

    if input == '1' or '2':
        client = client_l
    else:
        client = client_r

    result = client.get_result()
    did = []
    if client.get_state() != GoalStatus.SUCCEEDED:
        did.append("failed")
    else:
        if result.stalled: did.append("stalled")
        if result.reached_goal: did.append("reached goal")
    print ' and '.join(did)
