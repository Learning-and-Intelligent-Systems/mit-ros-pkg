#!/usr/bin/env python

import roslib; 
roslib.load_manifest('sushi_tutorials')
import rospy
import cc_example
import kinematics_msgs
import arm_navigation_msgs

from pr2_python import arm_planner
from pr2_python.arm_mover import ArmMover
from geometry_msgs.msg import PoseStamped

bad_poses = []

def get_pose(block, world_x_min, world_x_max, world_y_min, world_y_max):
    (cc_pose, used) = cc_example.get_gripper(block, world_x_min, world_x_max, \
                                         world_y_min, world_y_max, bad_poses)
    bad_poses.append(used)
    pose = PoseStamped()
    pose.header.frame_id = 'torso_lift_link'
    pose.pose.position.x = cc_pose[0]
    pose.pose.position.y = cc_pose[1]
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0

    return pose

def get_ik(pose, world):
    arm = arm_planner.ArmPlanner('right_arm')
    pose_stamped = get_pose(pose, world[0], world[1], \
                                world[2], world[3])
    print "arm_pose="+str(pose_stamped)
    ik_sol = arm.get_ik(pose_stamped)
    # ik_sol.solution is a RobotState
    #rospy.loginfo('IK Solution: '+str(ik_sol.solution))
    
    #check that the solution is correct
    wrist_pose = arm.get_hand_frame_pose(robot_state=ik_sol.solution, \
                                             frame_id=pose_stamped.header.frame_id)
    rospy.loginfo('Wrist pose in IK solution is\n' + str(wrist_pose))
    return ik_sol

def my_arm_planner(block_pose, (world_x_min, world_x_max, \
                                 world_y_min, world_y_max)):
    ik_sol = get_ik(block_pose, (world_x_min, world_x_max, \
                                     world_y_min, world_y_max))

    # when ik good kinematics_msgs.srv.GetConstraintAwarePositionIKResponse
    # when ik bad kinematics_msgs.srv.GetPositionIKResponse
    while not isinstance(ik_sol, kinematics_msgs.srv.GetConstraintAwarePositionIKResponse):
        print "poop="+str(bad_poses)
        ik_sol = get_ik(block_pose, (world_x_min, world_x_max, \
                                         world_y_min, world_y_max))
    return ik_sol

def my_arm_mover(goal_pose):
    arm_mover = ArmMover()
    arm_name = 'right_arm'

    # generate desired arm pose           
    # goal_pose = PoseStamped()
    # goal_pose.pose.position.x = 0.65
    # goal_pose.pose.orientation.w = 1.0
    # goal_pose.header.frame_id = '/torso_lift_link'

    handle = arm_mover.move_to_goal(arm_name, goal_pose)
    if handle.reached_goal():
        print 'Reached the goal!'
    else:
        print handle.get_errors()


def main():
    rospy.init_node("test_gripper", anonymous=True)

    # The wrist pose
    # pose_stamped = PoseStamped() 
    # pose_stamped.header.frame_id = 'torso_lift_link'
    # pose_stamped.pose.position.x = 0.65
    # pose_stamped.pose.orientation.w = 1.0
    
    world_x_min = 0.4
    world_x_max = 0.8
    world_y_min = -0.6
    world_y_max = 0.0
    block_pose = (0.65, -0.1, 0.2)
    
    goal_pose = my_arm_planner(block_pose, (world_x_min, world_x_max, \
                                 world_y_min, world_y_max))
    my_arm_mover(goal_pose)

main()
