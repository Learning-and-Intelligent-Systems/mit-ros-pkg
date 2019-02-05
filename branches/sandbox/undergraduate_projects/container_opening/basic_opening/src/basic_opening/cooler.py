#!/usr/bin/env python
import roslib
roslib.load_manifest('basic_opening')
import sys
import rospy
from pr2_utils import arm_control
from pr2_pick_and_place_demos.pick_and_place_manager import PickAndPlaceManager
from ee_cart_imped_msgs.msg import EECartImpedGoal, EECartImpedAction, StiffPoint
from geometry_msgs.msg import Pose, PoseStamped

def move_to():
    # manually put gripper in correct position
    pass

def grasp():
    # manually grasp
    pass

def open(ppm):
    print "opening cooler"
    goal_pose = goemetry_msgs.msg.PoseStamped()
    goal_pose.header.frame_id = "torso_lift_link"
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.pose.position.x = 0.0
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = 0.707 # handle orientation of a top handle on cupboard
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 0.707
    time = 15 
    (goal_force, goal_forces, goal_torque) = make_point()
    force_control(ppm, goal_pose, goal_force, goal_forces, goal_torque, (False, False, False), time)

def force_control(ppm, goal_pose, force, is_force, torque, is_torque, time):
    my_arm_control = arm_control.ArmControl("right_arm")
    transformed_goal_pose = ppm.tf_listener.transformPoint("/torso_lift_link", goal_pose)
    goal = EECartImpedGoal()
    goal.header.frame_id = "torso_lift_link"
    goal.header.stamp = rospy.Time.now()
    my_arm_control.add_trajectory_point_to_force_control(transformed_goal_pose.position.x, 
                                                         transformed_goal_pose.position.y, 
                                                         transformed_goal_pose.position.z, 
                                                         transformed_goal_pose.orientation.x, 
                                                         transformed_goal_pose.orientation.y, 
                                                         transformed_goal_pose.orientation.z, 
                                                         transformed_goal_pose.orientation.w, 
                                                         force.x, force.y, force.z, 
                                                         torque.x, torque.y, torque.z, 
                                                         is_force[0], is_force[1], is_force[2], 
                                                         is_torque[0], is_torque[1], is_torque[2], 
                                                         time)
    print "sending goal"
    my_arm_control.executeForceControl()
    print "waiting for result"
    my_arm_control.stop_in_place()

def make_point():
    force = geometry_msgs.msg.Point()
    force.x = 1000 #stiffness
    force.y = -30
    force.z = 100

    forces = (False, False, True)

    torque = geometry_msgs.msg.Point()
    torque.x = 10
    torque.y = 50
    torque.z = 40

    return (force, forces, torque)

def main():
    ppm = PickAndPlaceManager()

    move_to()
    grasp()
    open(ppm)

if __nam__ == '__main__':
    rospy.init_node("basic_opening")
    main()
