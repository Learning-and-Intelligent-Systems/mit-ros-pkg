#!/usr/bin/env python
'''
@author: Ulziibayar Otgonbaatar and Annie Holladay
SimpleActionServer for moving the specified arm of PR2 close the handle of the cabinet and then to a position that it can open the cabinet. Along the way, it opens the gripper and closes. 
'''
import roslib; roslib.load_manifest('opencupboard_action')
import rospy
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal
import geometry_msgs.msg 
import actionlib 
from geometry_msgs.msg import Pose , PoseStamped
import time
import sys
import pdb
import tf
import copy
import opencupboard_msgs.msg 
import math
import arm_navigation_msgs.msg
from pr2_utils import arm_control

class GraspHandleAction():
	def __init__(self, name):
		'''
		Initializes an instance of the class
		@type name: String
		@param name: String specifying the name of the instance
		'''		
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, opencupboard_msgs.msg.GraspHandleAction, execute_cb=self.main1)
		self._as.start()
		'''
		instantiates an SimpleActionServer with and starts it immediately. 
		The action server is named with the given string. It is specified 
		to run the main1() function when actionStarts
		'''		

	def openGripper (self,a):
		'''
		Opens the gripper using the given simpleActionClient
		(Either /l_gripper_controller/gripper_action or 
		/r_gripper_controller/gripper_action). Specified the size of the 
		operation to be, which is the distance between the two fingertips to 
		be 0.08(max is 0.99 and min 0.00). The max_effort to be -1.0. This 
		field places a limit on the amount of effort (force in N) to apply 
		while moving to that position. If 'max_effort' is negative, it is ignored. 
		
		@param a: simpleActionClient which must take Pr2GripperCommandGoal as a goal
		@todo : make it take the max_effort and position as parameter
		'''
		rospy.loginfo("Waiting for open gripper action")
		a.wait_for_server()
		goalGripper = Pr2GripperCommandGoal ()
				
		goalGripper.command.position = 0.08
		goalGripper.command.max_effort = -1.0
		rospy.loginfo("Opening gripper")
	 	a.send_goal_and_wait(goalGripper)
		return

	def close (self, a):
		'''
		Close the gripper using the given simpleActionClient
		(Either /l_gripper_controller/gripper_action or 
		/r_gripper_controller/gripper_action). Specified the size of the 
		operation to be, which is the distance between the two fingertips 
		to be 0.00(max is 0.99 and min 0.00). The max_effort set to be 			
		50.0. This field places a limit on the amount of effort (force in N) 
		to apply while moving to that position. 
		
		@param a: simpleActionClient which must take Pr2GripperCommandGoal as a goal
		@todo : make it take the max_effort and position as parameter
		'''
		rospy.loginfo("Waiting for close gripper action")
		a.wait_for_server()
		squeeze = Pr2GripperCommandGoal ()
	   	squeeze.command.position = 0.0
	    	squeeze.command.max_effort = 50.0
		rospy.loginfo("Closing gripper")
		a.send_goal_and_wait(squeeze)
		return 

	def interpolatedIK(self,start, end, arm, handle_box):
		'''
		Moves the specified hand from the start pose to the end pose using 
		move_arm_service/srv/InterpolatedIK service.
		
		@type start: PoseStamped
		@param start: PoseStamped start is used as start_pose field for 
		              InterpolatedIKRequest
		@type end: PoseStamped
		@param end : PoseStamped is used as field for  InterpolatedIKRequest
		@type arm : String
		@param arm : String specifying either "right_arm" or "left_arm"
		'''
		ops = arm_navigation_msgs.msg.OrderedCollisionOperations()
		op = arm_navigation_msgs.msg.CollisionOperation()
		op.object1 = arm[0] + '_end_effector'
		op.object2 = op.COLLISION_SET_ALL
		op.operation = op.DISABLE
		ops.collision_operations.append(op)
		my_arm_control = arm_control.ArmControl(arm)
		ik_call = my_arm_control.move_arm_interpolated_ik(end, ops)

		return ik_call
	
	def main1 (self, goal ):
		'''
		This function is called with the goal msg whenever request to 
		GraspHandleAction is made. It moves the specified arm in the goal 
		to a position where its gripper is 0.1 (10 cm) in front of the 
		specified pose, meaning its x position is 0.1 less. It sends 
		MoveArmRequest to a MoveArm service to move the arm.Note that 
		MoveArm service moves that wrist of the arm to the specified position. 
		Based on the specified arm it initializes SimpleActionClient 
		(either /l_gripper_controller/gripper_action or 
		/r_gripper_controller/gripper_action) and uses it to open and close 
		the gripper. Using InterPolatedIK service, it then moves the arm 
		straight in x axis by 0.1 to finally reach the goal pose and closes the 
		gripper.
		
			
		@type goal : GraspHandleGoal:
		             PoseStamped  GraspHandleGoal.pose_stamped
			     String GraspHandleGoal.arm
		@param goal : GraspHandleGoal whose arm and pose_stamped we use to move the arm 
		'''	
		pose = goal.pose_stamped
		arm = goal.arm
		start = copy.deepcopy(pose)
		start.pose.position.x= pose.pose.position.x - 0.28
		end = copy.deepcopy(pose)
		end.pose.position.x= pose.pose.position.x - 0.18

		if arm=="left_arm":
			gripper_client  = actionlib.SimpleActionClient('/l_gripper_controller/gripper_action',Pr2GripperCommandAction)
		else:
			gripper_client  = actionlib.SimpleActionClient('/r_gripper_controller/gripper_action',Pr2GripperCommandAction)		

		#Moves the arm to x,y,z position using collision free
		(move_result, move_state) = self.moveArm(start,arm)	
		#Continue only when moveArm returns success
		if move_result:	
			#To open the gripper
			self.openGripper(gripper_client)
			#use interpolated IK to move the left arm 	
			ik_resp = self.interpolatedIK(start, end, arm, goal.handle_box)
			# if ik_resp.error_code.val != ik_resp.error_code.SUCCESS:
			if ik_resp.val != ik_resp.SUCCESS:
				rospy.logerr("Aborting because interpolated ik returned error %d", 
					     ik_resp.val)
				self._as.set_aborted()
				return
			#To close
			self.close(gripper_client)
			self._as.set_succeeded()
		else:
			rospy.logerr("Aborting because move_arm returned an error")
			self._as.set_aborted()
	
	def moveArm(self, pose, arm):
		'''
		Moves the specified arm to the specified pose using MoveArm service 
		which uses IK. Initializes MoveArmService and a MoveArmRequest to the 
		service with specified arm and pose.
		
		@type pose: PoseStamped
		@param pose: PoseStamped pose
		@type arm : String
		@param arm: String arm (either "right_arm" or "left_arm")		
		@return: MoveArmActionResult Message which was returned from the MoveArm service
		'''
		print 'Moving arm'
		my_arm_control = arm_control.ArmControl(arm)
		move_resp = my_arm_control.move_arm_collision_free(pose)
		print 'Arm moved'
		return move_resp

if __name__ == '__main__':
	
	rospy.init_node('grasp_handle_action')
	GraspHandleAction(rospy.get_name())
 	print "Ready to start the action"
	rospy.spin()


