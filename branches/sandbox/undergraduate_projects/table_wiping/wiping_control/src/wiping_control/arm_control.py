#!/usr/bin/env python
import roslib; roslib.load_manifest('wiping_control')
import rospy
import tf
import control_switcher
import move_arm_service.srv
import wiping_control.ee_cart_imped_action
import random
import geometry_msgs.msg
import wiping_utils.conversions
import wiping_utils.utils
import move_arm_msgs.msg
import actionlib
import pr2_pick_and_place_demos.pick_and_place_manager

PLANNER_NAME='/ompl_planning/plan_kinematic_path'
PLANNER_ID='SBLkConfig1'

class ArmControl:
    """
    Wrapper around the ee_cart_imped_client class for the table_wiper module.
    """
    def __init__(self, arm_name, manager=None):
        """
        Creates a new arm_control objects
        """
        rospy.loginfo("Creating arm controller...")
        self.arm_name = arm_name
        self.switchToForce()
        """
        Simple action client to deal with the ee_cart_imped_controller.
        """
        if self.arm_name[0] == 'l':
            self.force_control =\
                wiping_control.ee_cart_imped_action.ee_cart_imped_client(True)
        else:
            self.force_control =\
                wiping_control.ee_cart_imped_action.ee_cart_imped_client(False)
        self.switchToCartesian()
        self.move_arm_client = actionlib.SimpleActionClient('/move_'+self.arm_name,
                                                            move_arm_msgs.msg.MoveArmAction)
        rospy.loginfo('Waiting for move arm client')
        self.move_arm_client.wait_for_server()
        rospy.loginfo("Arm controller created")
        self.manager = manager
        if not self.manager:
            self.manager = pr2_pick_and_place_demos.pick_and_place_manager.PickAndPlaceManager()
        """
        Transform listener that allows the arm control to get the location of the hand in the torso_lift_link frame.
        """
	self.tf_listener = self.manager.tf_listener

    def try_hard_for_pose(self, pose_stamped, max_tries=3):
        if self.arm_name[0] == 'l':
            whicharm = 1
        else:
            whicharm = 0
        return (self.manager.try_hard_to_move_pose(whicharm, pose_stamped,
                                                   max_tries=max_tries,
                                                   use_joint_open_loop=1,
                                                   use_cartesian=1) == 1)

    def move_arm_collision_free(self, pose_stamped):
        self.switchToCartesian()
        goal = move_arm_msgs.msg.MoveArmGoal()
        goal.planner_service_name=PLANNER_NAME
        (pconstraint, oconstraint) =\
            wiping_utils.conversions.\
            poseStampedToPositionOrientationConstraints(pose_stamped,
                                                        self.arm_name[0]+
                                                        '_wrist_roll_link')
        goal.motion_plan_request.goal_constraints.position_constraints.append(pconstraint)
        goal.motion_plan_request.goal_constraints.orientation_constraints.append(oconstraint)
        goal.motion_plan_request.start_state = wiping_utils.utils.currentRobotState()
        goal.motion_plan_request.planner_id = PLANNER_ID
        goal.motion_plan_request.group_name = self.arm_name
        goal.motion_plan_request.num_planning_attempts = 2
        goal.motion_plan_request.allowed_planning_time = rospy.Duration(45.0)
        self.move_arm_client.send_goal_and_wait(goal, rospy.Duration(60.0))
        result = self.move_arm_client.get_result()
        state = self.move_arm_client.get_state()
        rospy.loginfo('Move arm returned in state %s with result %s', 
                      str(state),
                      str(result))
        return (result, state)

        # move_call = move_arm_service.srv.MoveArmRequest()
        # move_call.pose_stamped = pose_stamped
        # move_call.arm = arm_name
        # rospy.loginfo('Moving %s to goal', arm_name)
        # move_srv = rospy.ServiceProxy('move_arm_to_pose', move_arm_service.srv.MoveArm)
        # move_resp = move_srv(move_call)
        # return move_resp

    def add_trajectory_point_to_force_control(self, x, y, z, ox, oy, oz, ow, 
                                              fx, fy, fz, tx, ty, tz, 
                                              isfx, isfy, isfz, istx, 
                                              isty, istz, time, frame_id=''):
        """
        Adds a new point to the current trajectory. See the relevant method in ee_cart_imped_client for in depth documentation. The only difference is that the time for this is that time this particular action should take, as opposed to the time after the trajectory starts that the arm should be in place.
        """
        self.force_control.addTrajectoryPoint(x, y, z, ox, oy, oz, ow, 
                                              fx, fy, fz, tx, ty, tz, 
                                              isfx, isfy, isfz, istx, isty, istz, 
                                              self.force_control.trajectoryTime().to_sec()+
                                              time, frame_id)
    def get_hand_pose(self, frame_id='/torso_lift_link'):
        """
        @returns: The current pose of the left gripper in the 
        torso_lift_link frame.
        """
	while True:
	    try:
		return self.tf_listener.lookupTransform(frame_id, 
                                                        '/'+self.arm_name[0]+
                                                        '_gripper_tool_frame', 
                                                        rospy.Time(0))
	    except:
		continue

    def stop_in_place(self):
        """
        Cancels the current arm movement and commands it to stay in 
        place with maximum stiffness.
        """
	(trans,rot) = self.get_hand_pose()
	self.force_control.cancelGoal()
	self.force_control.resetGoal()
	self.force_control.addTrajectoryPoint(trans[0], trans[1], trans[2], 
                                              rot[0],rot[1],rot[2],rot[3],
                                              1000,1000,1000,30,30,30,
                                              False,False,False,False,False,False,1)
	self.force_control.sendGoal()
	self.force_control.resetGoal()

    def executeForceControl(self, reset =  True, wait = True):
        """
        Commands the arm to actually execute the goal trajectory it has stored.

        @type reset: Boolean
        @param reset: If this is set to False, then this does not reset the 
        current goal of the arm_control object. Defaults to True.
        @type wait: Boolean
        @param wait: If this is set to True, then this method will block until 
        the action completes. Otherwise it will return and let the action 
        continue. This defaults to True.
        """
        self.switchToForce()
	self.force_control.sendGoal(wait)
	if reset:
	    self.force_control.resetGoal()

    def cancel(self):
        """
        Cancels the current action of the arm.
        """
	self.force_control.cancelGoal()        

    def resetGoal(self):
        self.force_control.resetGoal()

    def switchToCartesian(self):
        rospy.loginfo('Switching to cartesian control on arm %s', 
                      self.arm_name)
        if self.arm_name[0] == 'l':
            control_switcher.PR2CMClient.load_cartesian(False)
        else:
            control_switcher.PR2CMClient.load_cartesian(True)
    
    def switchToForce(self):
        rospy.loginfo('Switching to force control on arm %s', self.arm_name)
        if self.arm_name[0] == 'l':
            control_switcher.PR2CMClient.load_ee_cart_imped(False)
        else:
            control_switcher.PR2CMClient.load_ee_cart_imped(True)

if __name__ == "__main__":
    rospy.init_node('arm_control_test_node')
    c = ArmControl('left_arm')
    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.pose.position.x = 0.75
    pose_stamped.pose.position.y = 0.47
    pose_stamped.pose.position.z = 1.1
    pose_stamped.pose.orientation.x = -0.5
    pose_stamped.pose.orientation.y = 0.5
    pose_stamped.pose.orientation.z = 0.5
    pose_stamped.pose.orientation.w = 0.5
    pose_stamped.header.frame_id = 'base_link'
    res = c.try_hard_for_pose(pose_stamped, max_tries=0)
    print 'result of trying hard was', res
