#! /usr/bin/python
import roslib
roslib.load_manifest('simple_arm_controller')
import rospy

import actionlib
import time
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, Pr2GripperCommandGoal, Pr2GripperCommandAction
from trajectory_msgs.msg import JointTrajectoryPoint

#to control the left arm:
whicharm='l'


rospy.init_node('controller_manager')

joint_trajectory_action_name = whicharm+'_arm_controller/joint_trajectory_action'
joint_action_client = actionlib.SimpleActionClient(joint_trajectory_action_name, JointTrajectoryAction)
print "waiting for server..."
joint_action_client.wait_for_server()
print "got server"
joint_names = ["_shoulder_pan_joint",
		"_shoulder_lift_joint",
		"_upper_arm_roll_joint",
		"_elbow_flex_joint",
		"_forearm_roll_joint",
		"_wrist_flex_joint",
		"_wrist_roll_joint"]
joint_names1=[whicharm+x for x in joint_names]
goal = JointTrajectoryGoal()
goal.trajectory.header.stamp = rospy.get_rostime()
goal.trajectory.joint_names = joint_names1


jointangles=[0]*7
jointangles[0]=.7
#jointangles[1]=-.3
jointvelocities=[0]*7
jointaccelerations=[0]*7

time_for_motion=rospy.Duration(1.0)

point = JointTrajectoryPoint(jointangles, jointvelocities, jointaccelerations, time_for_motion)
goal.trajectory.points = [point,]
joint_action_client.send_goal(goal)
time.sleep(1.0)


point = JointTrajectoryPoint([0]*7, [0]*7, [0]*7, rospy.Duration(3.0))
goal.trajectory.points = [point,]
joint_action_client.send_goal(goal)