#!/usr/bin/env python

import roslib; roslib.load_manifest('opendoors_executive')
import rospy
import actionlib
import opendoors_executive.msg

def main():
    open_cupboard_client = actionlib.SimpleActionClient('open_cupboard_action', opendoors_executive.msg.OpenCupboardAction)
    
    print "waiting for server"
    open_cupboard_client.wait_for_server()

    goal = opendoors_executive.msg.OpenCupboardGoal()
    goal.open_cupboard = True
    print "sending goal"
    open_cupboard_client.send_goal(goal)

    print "waiting for result"
    open_cupboard_client.wait_for_result()

    print "RESULT:",open_cupboard_client.get_result()

if __name__ == '__main__':
    rospy.init_node('test_open_cupboard_action')
    main()
