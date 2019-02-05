#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
import rospy
import pr2_controllers_msgs.msg 
import actionlib

class TorsoClient():

    def __init__(self):
        self.action_client = actionlib.SimpleActionClient('/torso_controller/position_joint_action', pr2_controllers_msgs.msg.SingleJointPositionAction)
        print 'waiting for server'
        self.action_client.wait_for_server()
        print 'done waiting'

    def up(self):
        goal = pr2_controllers_msgs.msg.SingleJointPositionGoal()
        goal.position = 0.3
        self.action_client.send_goal_and_wait(goal)

if __name__ == '__main__':
    rospy.init_node('torso_lifter', anonymous=True)
    torso_lifter = TorsoClient()
    torso_lifter.up()
    print 'done'
