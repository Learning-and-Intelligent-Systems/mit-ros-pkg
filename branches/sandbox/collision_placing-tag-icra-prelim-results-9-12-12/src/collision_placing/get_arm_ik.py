#!/usr/bin/env python

import roslib; 
roslib.load_manifest('collision_placing')
import rospy
import cc_example

from pr2_python import arm_planner
from geometry_msgs.msg import PoseStamped


rospy.init_node("test_gripper", anonymous=True)
arm = arm_planner.ArmPlanner('right_arm')

cc_pose = cc_example.permute_gripper()

# The wrist pose
pose_stamped = PoseStamped() 
pose_stamped.header.frame_id = 'torso_lift_link'
pose_stamped.pose.position.x = 0.65
pose_stamped.pose.orientation.w = 1.0

ik_sol = arm.get_ik(pose_stamped)
# ik_sol.solution is a RobotState
rospy.loginfo('IK Solution: '+str(ik_sol.solution))

#check that the solution is correct
wrist_pose = arm.get_hand_frame_pose(robot_state=ik_sol.solution, frame_id=pose_stamped.header.frame_id)
rospy.loginfo('Wrist pose in IK solution is\n' + str(wrist_pose))
