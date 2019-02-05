#!/usr/bin/env python

'''
A Python test program for the service.

This test program shows how to call the move_arm service and the interpolated 
IK service from Python.  It also has example comments for documenting Python
code with Epydoc.

Technically, this belongs in the scripts directory since it is not code
anyone using the package (rather than working on it) would need.  However,
I have put in the the src directory so that it could serve as example
documentation.
'''

import roslib; roslib.load_manifest('move_arm_service')
import rospy
import move_arm_service.srv
from geometry_msgs.msg import PoseStamped
import copy

class ArmMovements:
    '''
    A class that contains functions for moving the arm
    using move_arm and interpolated IK.  

    Probably a class
    is overkill for these simple functions, but I wanted
    to show how to comment a class.
    '''

    def __init__(self, arm_name):
        '''
        Constructor for ArmMovements
        
        @type arm_name: string
        @param arm_name: The name of the arm this
        class should move ('right_arm' or 'left_arm')
        '''
        self.arm_name = arm_name
        '''
        The name of the arm this class moves
        '''

    def move_arm(self, pose_stamped):
        '''
        A function to move the arm to a particular pose by calling the
        move_arm_service.
        
        @type pose_stamped: geometry_msgs.msg.PoseStamped
        @param pose_stamped: The pose to move the arm to
        @rtype: move_arm_service.srv.MoveArmResponse
        @returns: The response from calling the move_arm_service
        '''
        move_call = move_arm_service.srv.MoveArmRequest()
        move_call.pose_stamped = pose_stamped
        move_call.arm = self.arm_name
        print 'Calling move_arm'
        move_arm_srv = rospy.ServiceProxy('move_arm_to_pose', move_arm_service.srv.MoveArm)
        move_resp = move_arm_srv(move_call)    
        return move_resp

    def interpolated_ik(self, start_pose, end_pose):
        '''
        Function for moving from start_pose to end_pose using interpolated IK.

        @type start_pose: geometry_msgs.msg.PoseStamped
        @param start_pose: The pose the arm will start in
        @type end_pose: geometry_msgs.msg.PoseStamped
        @param end_pose: The pose the arm will end in
        @rtype: move_arm_service.srv.InterpolatedIKResponse
        @returns: The response from the interpolated IK service
        '''
        ik_call = move_arm_service.srv.InterpolatedIKRequest()
        ik_call.start_pose = start_pose
        ik_call.end_pose = end_pose
        ik_call.arm = self.arm_name
        print 'Calling ik'
        ik_srv = rospy.ServiceProxy('run_interpolated_ik', 
                                    move_arm_service.srv.InterpolatedIK)
        ik_resp = ik_srv(ik_call)
        return ik_resp


def main():
    '''
    The main function
    '''
    rospy.init_node('test_ik_node')
    mover = ArmMovements('left_arm')
    pose_stamped = PoseStamped()
    pose_stamped.pose.position.x = 0
    pose_stamped.pose.position.y = 0.7
    pose_stamped.pose.position.z = 0
    pose_stamped.pose.orientation.x = 0.707
    pose_stamped.pose.orientation.y = 0
    pose_stamped.pose.orientation.z = 0
    pose_stamped.pose.orientation.w = 0.707
    pose_stamped.header.frame_id = 'torso_lift_link'
    pose_stamped.header.stamp = rospy.Time.now()
    move_resp = mover.move_arm(pose_stamped)
    print 'move_arm returned with response', move_resp.error_code.val
    end_pose = copy.deepcopy(pose_stamped)
    end_pose.pose.position.x = 0.1
    ik_resp = mover.interpolated_ik(pose_stamped, end_pose)
    print 'ik returned with response', ik_resp.error_code.val

if __name__ == '__main__':
    main()
