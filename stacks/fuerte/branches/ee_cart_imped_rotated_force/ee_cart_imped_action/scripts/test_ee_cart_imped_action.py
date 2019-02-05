#!/usr/bin/env python

import roslib; roslib.load_manifest('ee_cart_imped_action')
import rospy
import geometry_msgs.msg
import ee_cart_imped_action

def main():
    rospy.loginfo('The ee_cart_imped_action does no collision checking.  Please place robot away from any obstacles and press enter to continue')
    ans = raw_input()
    
    right_arm = ee_cart_imped_action.EECartImpedClient('right_arm')
    left_arm = ee_cart_imped_action.EECartImpedClient('left_arm')

    rospy.loginfo('Moving to initial positions')
    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.header.frame_id = '/torso_lift_link'
    pose_stamped.pose.position.y = -0.4
    pose_stamped.pose.position.z = -0.2
    pose_stamped.pose.orientation.y = 0.707
    pose_stamped.pose.orientation.w = 0.707
    right_arm.moveToPoseStamped(pose_stamped, 4.0)
    pose_stamped.pose.position.y = 0.4
    left_arm.moveToPoseStamped(pose_stamped, 4.0)

    #do this a couple times to make sure
    #everything continues to work
    rospy.loginfo('Beginning trajectory tests')
    for i in range(2):
        right_arm.addTrajectoryPoint(0.6, 0, 1.0, 0.707, 0, 0, 0.707,
                                     8, 1000, 5, 100, 100, 100,
                                     True, False, True, False, False, False,
                                     4.0, frame_id='/base_link')
        right_arm.addTrajectoryPoint(0.6, 0, 1.0, 0.707, 0, 0, 0.707,
                                     -3, 1000, 5, 100, 100, 100,
                                     True, False, True, False, False, False,
                                     6.0, frame_id='/base_link')
        rospy.loginfo('Press enter for a trajectory of 2 force points on the right arm')
        ans = raw_input()
        right_arm.sendGoal()
        right_arm.resetGoal()
        right_arm.addTrajectoryPoint(0.6, 0, 2.0, 0.707, 0, 0, 0.707,
                                     1000, 1000, 1000, 1.0, 100, 100,
                                     False, False, False, True, False, False,
                                     3.0, frame_id='/base_link')
        right_arm.addTrajectoryPoint(0.6, 0, 1.0, 0.5, 0.5, 0.5, 0.5,
                                     1000, 1000, 1000, 100, 100, 100,
                                     False, False, False, False, False, False,
                                     5.0, frame_id='/base_link')
        right_arm.addTrajectoryPoint(0.2, -0.6, 1.0, -0.5, 0.5, 0.5, 0.5,
                                     1000, 1000, 1000, 100, 100, 100,
                                     False, False, False, False, False, False,
                                     9.0, frame_id='/base_link')
        rospy.loginfo('Press enter for a torque to stiffness trajectory on the right arm')
        ans = raw_input()
        right_arm.sendGoal()
        right_arm.resetGoal()
        left_arm.addTrajectoryPoint(0, 0.4, 0, 0.707, 0, 0, 0.707,
                                    1000, 1000, 1000, 100, 100, 100,
                                    False, False, False, False, False, False,
                                    2.0, frame_id='/torso_lift_link')
        left_arm.addTrajectoryPoint(0, 0.4, 0, 0.707, 0, 0, 0.707,
                                    -5, 1000, 1000, 100, 100, 100,
                                    True, False, False, False, False, False,
                                    4.0, frame_id='/torso_lift_link')
        left_arm.addTrajectoryPoint(0.1, 0.4, -0.2, 0.707, 0, 0, 0.707,
                                    1000, 1000, 1000, 100, 100, 100,
                                    False, False, False, False, False, False,
                                    6.0, frame_id='/torso_lift_link')
        rospy.loginfo('Press enter for a trajectory on the left arm')
        ans = raw_input()
        left_arm.sendGoal()
        left_arm.resetGoal()
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header.frame_id = '/base_link'
        pose_stamped.pose.position.x = 0.6
        pose_stamped.pose.position.y = 0
        pose_stamped.pose.position.z = 1.0
        pose_stamped.pose.orientation.w = 1.0
        rospy.loginfo('Press enter to test move to pose on the right arm')
        ans = raw_input()
        right_arm.moveToPoseStamped(pose_stamped, 2.0)
        rospy.loginfo('Moving to another pose')
        pose_stamped.pose.position.y = -0.2
        pose_stamped.pose.orientation.z = -0.707
        pose_stamped.pose.orientation.w = 0.707
        right_arm.moveToPoseStamped(pose_stamped, 4.0)

        right_arm.addTrajectoryPoint(0.6, 0, 1.0, 0.707, 0, 0, 0.707,
                                     1000, 1000, 1000, 100, 100, 100,
                                     False, False, False, False, False, False,
                                     2.0, frame_id='/base_link')
        right_arm.addTrajectoryPoint(0.6, 0, 0.0, 0.707, 0, 0, 0.707,
                                     1000, -10, 1000, 100, 100, 100,
                                     False, True, False, False, False, False,
                                     4.0, frame_id='/torso_lift_link')
        right_arm.addTrajectoryPoint(0.6, 0, 1.0, 0.707, 0, 0, 0.707,
                                     1000, 1000, -5, 100, 100, 100,
                                     False, False, True, False, False, False,
                                     6.0, frame_id='/base_link')
        right_arm.addTrajectoryPoint(0.1, -0.4, 1.0, 0, 0, 0, 1.0,
                                     1000, 1000, 1000, 100, 100, 100,
                                     False, False, False, False, False, False,
                                     8.0, frame_id='/base_link')
        rospy.loginfo('Press enter for a mixed trajectory on right arm')
        ans = raw_input()
        right_arm.sendGoal()
        right_arm.resetGoal()
        pose_stamped.pose.position.y = 0.1
        rospy.loginfo('Press enter to test move to pose stamped on left arm')
        ans = raw_input()
        left_arm.moveToPoseStamped(pose_stamped, 3.0)
        pose_stamped.pose.position.x = 0
        pose_stamped.pose.position.y = 0.4
        pose_stamped.pose.position.z = 0.0
        pose_stamped.header.frame_id = '/torso_lift_link'
        pose_stamped.pose.orientation.y = 0.707
        pose_stamped.pose.orientation.w = 0.707
        rospy.loginfo('Testing another move to pose stamped on left arm')
        left_arm.moveToPoseStamped(pose_stamped, 4.0)

    rospy.loginfo('Beginning canceling tests')
    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.header.frame_id = '/base_link'
    pose_stamped.pose.position.x = 0.6
    pose_stamped.pose.position.y = 0
    pose_stamped.pose.position.z = 1.0
    pose_stamped.pose.orientation.w = 1.0
    right_arm.moveToPoseStamped(pose_stamped, 4.0)
    right_arm.addTrajectoryPoint(0.6, 0, 1.0, 0.707, 0, 0, 0.707,
                                 1000, 1000, -5, 100, 100, 100,
                                 False, False, True, False, False, False,
                                 2.0, frame_id='/base_link')
    right_arm.addTrajectoryPoint(0.6, 0, 1.0, 0, 0, 0, 1.0,
                                 1000, 1000, 1000, 100, 100, 100,
                                 False, False, False, False, False, False,
                                 4.0, frame_id='/base_link')
    rospy.loginfo('Press enter to test canceling goal')
    ans = raw_input()
    right_arm.sendGoal(wait=False)
    right_arm.cancelGoal()
    rospy.loginfo('Press enter to test canceling goal on second trajectory point')
    ans = raw_input()
    right_arm.sendGoal(wait=False)
    rospy.sleep(2.5)
    right_arm.cancelGoal()
    rospy.loginfo('Returning to initial position')
    right_arm.sendGoal()
    rospy.loginfo('Press enter to test goal preempt')
    ans = raw_input()
    right_arm.sendGoal(wait=False)
    rospy.sleep(2.5)
    rospy.loginfo('Returning to intial position')
    right_arm.sendGoal()
    rospy.loginfo('Press enter to test quick succession of goals.')
    ans = raw_input()
    for i in range(50):
        right_arm.sendGoal(wait=False)
        rospy.sleep(0.1)
    rospy.loginfo('Returning to initial position')
    right_arm.sendGoal()
    right_arm.resetGoal()
    right_arm.addTrajectoryPoint(0, -0.4, -0.2, 0, 0.707, 0, 0.707,
                                 1000, 1000, 1000, 100, 100, 100,
                                 False, False, False, False, False, False,
                                 4.0, frame_id='/torso_lift_link')
    left_arm.addTrajectoryPoint(0, 0.4, -0.2, 0, 0.707, 0, 0.707,
                                1000, 1000, 1000, 100, 100, 100,
                                False, False, False, False, False, False,
                                4.0, frame_id='/torso_lift_link')
    rospy.loginfo('Press enter to test simultaneous arm movement')
    ans = raw_input()
    left_arm.sendGoal(wait=False)
    right_arm.sendGoal()
    rospy.loginfo('Tests completed')

if __name__ == '__main__':
    rospy.init_node('ee_cart_imped_action_python_testing_node')
    main()
