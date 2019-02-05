#!/usr/bin/env python
import roslib; roslib.load_manifest('opencupboard_action')
import rospy
import actionlib
import opencupboard_msgs.msg

def main():
    '''
    Runs the openCupboard action. 
    '''
    open_cupboard_client = actionlib.SimpleActionClient('open_cupboard_action', opencupboard_msgs.msg.OpenCupboardAction)
    
    print "waiting for server"
    open_cupboard_client.wait_for_server()

    goal = opencupboard_msgs.msg.OpenCupboardGoal()
    goal.open_cupboard = True
    print "sending goal"
    open_cupboard_client.send_goal(goal)

    print "waiting for result"
    open_cupboard_client.wait_for_result()

    print "RESULT:",open_cupboard_client.get_result()

if __name__ == '__main__':
    rospy.init_node('test_open_cupboard_action')
    main()
