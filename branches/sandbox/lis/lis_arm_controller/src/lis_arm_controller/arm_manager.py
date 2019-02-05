#! /usr/bin/python
# based on Kaijen Hsiao's controlelr_manager.py in pr2_gripper_reactive_approach package
import roslib
roslib.load_manifest('lis_arm_controller')
import rospy

# import msgs and srvs
from object_manipulation_msgs.msg import GraspableObject
from object_manipulation_msgs.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, Pr2GripperCommandGoal, Pr2GripperCommandAction
from slipgrip_controller.msg import GripperFindContactAction, GripperFindContactGoal, GripperSlipServoAction, GripperSlipServoGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from simple_Jtranspose_controller.srv import MoveToPoseRequest
from std_srvs.srv import EmptyRequest
from move_arm_msgs.msg import MoveArmAction, MoveArmGoal
from motion_planning_msgs.msg import JointConstraint, PositionConstraint, OrientationConstraint, ArmNavigationErrorCodes, AllowedContactSpecification
# import libraries
import actionlib
import tf
import scipy
import math
from math import *
import random
import time
import sys
import threading
import numpy
# import utitlies
from interpolated_ik_motion_planner import ik_utilities
from object_manipulator.convert_functions import *
from object_manipulator.draw_functions import *
import joint_states_listener
import controller_manager

class ArmManager():
    def __init__(self, whicharm, tf_listener = None, using_slip_controller = 0): #whicharm is 'r' or 'l'
        self.whicharm = whicharm
        self.using_slip_controller = using_slip_controller
        rospy.loginfo('initializing '+whicharm+' controlelr manager')
        rospy.loginfo('controller manager: using_slip_controller: '+str(using_slip_controller))
        # listeners
        # initialize a tf listener
        if tf_listener == None: self.tf_listener = tf.TransformListener()
        else: self.tf_listener = tf_listener
        # joint state listener
        self.joint_states_listener = joint_states_listener.LatestJointStates() # has its own thread
        # action clients
        # joint action clients
        joint_trajectory_action_name = whicharm+'_arm_controller/joint_trajectory_action'
        self.joint_action_client = actionlib.SimpleActionClient(joint_trajectory_action_name, JointTrajectoryAction)
        self.joint_action_client.wait_for_server()
        # gripper action clients
        if self.using_slip_controller:
            gripper_action_name = whicharm+'_gripper_fingersensor_controller/gripper_action'
            gripper_find_contact_action_name = whicharm+'_gripper_fingersensor_controller/find_contact'
            gripper_slip_servo_action_name = whicharm+'_gripper_fingersensor_controller/slip_servo'
            self.gripper_action_client = actionlib.SimpleActionClient(gripper_action_name, Pr2GripperCommandAction)
            self.gripper_find_contact_action_client = actionlib.SimpleActionClient(gripper_find_contact_action_name, GripperFindContactAction)
            self.gripper_slip_servo_action_client = actionlib.SimpleActionClient(gripper_slip_servo_action_name, GripperSlipServoAction)
        else:
            gripper_action_name = whicharm+'_gripper_controller/gripper_action'
            self.gripper_action_client = actionlib.SimpleActionClient(gripper_action_name, Pr2GripperCommandAction)
        self.gripper_action_client.wait_for_server()
        if self.using_slip_controller: gripper_find_contact_action_client.wait_for_server()
        # move arm client is filled in the first time it's called
        self.move_arm_client = None
        # controllers
        self.cm = controller_manager.ControllerManager(self.whicharm, self.using_slip_controller)
        self.gripper_controller_state = self.cm.gripper_controller_state
        self.joint_controller_state = self.cm.joint_controller_state
        self.cartesian_controllers_state = self.cm.cartesian_controllers_state
        self.cartesian_moving_service = self.cm.cartesian_moving_service
        self.cartesian_cmd_service = self.cm.cartesian_cmd_service
        self.cartesian_preempt_service = self.cm.cartesian_preempt_service
        self.check_controllers_ok = self.cm.check_controllers_ok
        self.check_controller_states = self.cm.check_controller_states
        self.load_cartesian_controllers = self.cm.load_cartesian_controllers
        self.unload_cartesian_controllers = self.cm.unload_cartesian_controllers
        self.load_joint_controllers = self.cm.load_joint_controllers
        self.unload_joint_controllers = self.cm.unload_joint_controllers
        self.reload_cartesian_controllers = self.cm.reload_cartesian_controllers
        self.reload_joint_controllers = self.cm.reload_joint_controllers
        self.switch_to_cartesian_mode = self.cm.switch_to_cartesian_mode
        self.switch_to_joint_mode = self.cm.switch_to_joint_mode
        self.start_joint_controllers = self.cm.start_joint_controllers
        self.start_cartesian_controllers = self.cm.start_cartesian_controllers
        self.start_gripper_controller = self.cm.start_gripper_controller
        self.stop_all_controllers = self.cm.stop_all_controllers
        self.stop_controllers = self.cm.stop_controllers
        self.cart_params = self.cm.cart_params
        self.joint_params = self.cm.joint_params

        # create an IKUtilities class object
        rospy.loginfo("creating IKUtilities class objects")
        if whicharm == 'r': self.ik_utilities = ik_utilities.IKUtilities('right', self.tf_listener)
        else: self.ik_utilities = ik_utilities.IKUtilities('left', self.tf_listener)
        rospy.loginfo("done creating IKUtilities class objects")
        # joint names for the arm
        joint_names = ["_shoulder_pan_joint",
                       "_shoulder_lift_joint",
                       "_upper_arm_roll_joint",
                       "_elbow_flex_joint",
                       "_forearm_roll_joint",
                       "_wrist_flex_joint",
                       "_wrist_roll_joint"]
        self.joint_names = [whicharm + x for x in joint_names]
        rospy.loginfo("done with controller_manager init for the %s arm"%whicharm)
        
    # tell the gripper to close until contact (on one or both finger pads)
    # contacts_desired is "both", "left", "right", or "either"
    def find_gripper_contact(self, contacts_desired, use_force_control = 0, zero_fingertips = 0, blocking = 1, timeout = 12.,close_speed = 0.02):
        contacts_desired_dict = {"both":0, "left":1, "right":2, "either":3}
        goal = GripperFindContactGoal()
        goal.command.contacts_desired = contacts_desired_dict[contacts_desired]
        goal.command.close_speed = close_speed
        goal.command.max_effort = -1
        goal.command.zero_forces = zero_fingertips
        # tell the gripper to hold position if we're only looking for one contact
        goal.command.force_control_at_goal = 0
        if contacts_desired == "both": goal.command.force_control_at_goal = use_force_control
        rospy.loginfo("controller manager: sending find contact goal")
        self.gripper_find_contact_action_client.send_goal(goal)
        # if blocking is requested, wait for the state to reach the desired state
        if blocking:
            finished_within_time = self.gripper_find_contact_action_client.wait_for_result(rospy.Duration(timeout))
            if not finished_within_time:
                self.gripper_find_contact_action_client.cancel_goal()
                rospy.logerr("Gripper didn't see the desired number of contacts while closing in time")
                return (0, 0, 0, 0)
            state = self.gripper_find_contact_action_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                result = self.gripper_find_contact_action_client.get_result()
                return (result.data.left_contact, result.data.right_contact, result.data.left_pad_contact_force, result.data.right_pad_contact_force)
            return (0,0,0,0)

    # get the state from find_gripper_contact
    def get_find_gripper_contact_state(self):
        return self.gripper_find_contact_action_client.get_state()

    # get the final result from find_gripper_contact
    def get_find_gripper_contact_result(self):
        result = self.gripper_find_contact_action_client.get_result()
        if result: return (result.data.left_contact, result.data.right_contact, result.data.left_pad_contact_force, result.data.right_pad_contact_force)
        else:
            rospy.logerr("gripper_find_contact_action_client.get_result returned None!")
            return (0, 0, 0, 0)

    # start up slip servoing to move the object without slipping
    # fingertip start force should typically come from the output of find_gripper_contact
    # slip servo doesn't generate a result unless the object is dropped, so the function returns after the goal is sent
    def start_slip_servo(self, fingertip_start_force, deformation_limit = .025, max_effort = -1.0, fingertip_force_limit = -1.0):
        goal = GripperSlipServoGoal()
        goal.command.deformation_limit = deformation_limit
        goal.command.fingertip_start_force = -1
        goal.command.max_effort = max_effort
        goal.command.fingertip_force_limit = fingertip_force_limit
        rospy.loginfo("starting slip servo mode")
        self.gripper_slip_servo_action_client.send_goal(goal)

    # create a basic goal message for move_arm
    def create_move_arm_goal(self):
        goal = MoveArmGoal()
        if self.whicharm == "r": goal.motion_plan_request.group_name = "right_arm"
        else: goal.motion_plan_request.group_name = "left_arm"
        goal.motion_plan_request.num_planning_attempts = 2;
        goal.motion_plan_request.allowed_planning_time = rospy.Duration(5.0);
        goal.motion_plan_request.planner_id = ""
        goal.planner_service_name = "ompl_planning/plan_kinematic_path"
        return goal

    # send a goal to move arm and wait for the result, modifying the goal if the start state is in collision
    def send_move_arm_goal(self, goal, blocking = 1):
        # create an action client, if this is the first call to move_arm
        if self.move_arm_client == None:
            if self.whicharm == "r": self.move_arm_client = actionlib.SimpleActionClient('move_right_arm', MoveArmAction)
            if self.whicharm == "l": self.move_arm_client = actionlib.SimpleActionClient('move_left_arm', MoveArmAction)
            rospy.loginfo("waiting for move arm server")
            self.move_arm_client.wait_for_server()
            rospy.loginfo("move arm server was there")
        # send the goal off
        self.move_arm_client.send_goal(goal)
        # wait for the move to finish and return the result
        if blocking:
            for add_contact_tries in range(10):
                finished_within_time = self.move_arm_client.wait_for_result(rospy.Duration(60))
                if not finished_within_time:
                    self.move_arm_client.cancel_goal()
                    rospy.logerr("Timed out when asking move_arm to go to a joint goal")
                    return 0
                state = self.move_arm_client.get_state()
                result = self.move_arm_client.get_result()
                if state == GoalStatus.SUCCEEDED:
                    if result.error_code.val == 1:
                        rospy.loginfo("move_arm succeeded")
                        break
                    # start state was in collision!  Try to add contact regions to ignore the start collisions
                    elif result.error_code.val == ArmNavigationErrorCodes.START_STATE_IN_COLLISION:
                        rospy.loginfo("move arm start state in collision!  Adding allowed contact regions and trying again, try number %d"%add_contact_tries)
                        self.modify_move_arm_goal(goal, result.contacts)
                        self.move_arm_client.send_goal(goal)
                        continue
                    else:
                        rospy.logerr("move_arm did not succeed, state %d, error code %d"%(state, result.error_code.val))
                        break
            return result.error_code.val
        else: return 1

    # use move_arm to get to a desired joint configuration in a collision-free way
    def move_arm_joint(self, joint_angles, blocking = 1):
        self.check_controllers_ok('joint')
        rospy.loginfo("asking move_arm to go to angles: %s"%self.pplist(joint_angles))
        goal = self.create_move_arm_goal()
        # goal.disable_collision_monitoring = 1
        # add the joint angles as a joint constraint
        for (joint_name, joint_angle) in zip(self.joint_names, joint_angles):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = joint_angle
            joint_constraint.tolerance_below = .1
            joint_constraint.tolerance_above = .1
            goal.motion_plan_request.goal_constraints.joint_constraints.append(joint_constraint)
        # send the goal off to move arm, blocking if desired and modifying the start state if it's in collision
        result = self.send_move_arm_goal(goal, blocking)
        return result

    # if move_arm_joint returns an error code of START_STATE_IN_COLLISION, we can try to add allowed contact regions to get it out of the start collisions (appears to already be done in C++)
    def modify_move_arm_goal(self, goal, contact_info):
        allowed_contacts = []
        allowed_penetration_depth = .5
        for (i, contact) in enumerate(contact_info):
            allowed_contact_tmp = AllowedContactSpecification()
            allowed_contact_tmp.shape.type = allowed_contact_tmp.shape.BOX
            allowed_contact_tmp.shape.dimensions = [allowed_penetration_depth]*3
            allowed_contact_tmp.pose_stamped.pose.position = contact.position
            set_xyzw(allowed_contact_tmp.pose_stamped.pose.orientation, [0,0,0,1])
            allowed_contact_tmp.pose_stamped.header.stamp = rospy.Time.now()
            allowed_contact_tmp.pose_stamped.header.frame_id = contact.header.frame_id
            allowed_contact_tmp.link_names.append(contact.contact_body_1)
            allowed_contact_tmp.penetration_depth = allowed_penetration_depth
            allowed_contacts.append(allowed_contact_tmp);
            rospy.loginfo("Added allowed contact region: %d"%i)
            rospy.loginfo("Bodies                      : %s and %s"%(contact.contact_body_1, contact.contact_body_2))
        goal.motion_plan_request.allowed_contacts.extend(allowed_contacts)

    # move the wrist to the desired location while keeping the current wrist orientation upright
    # start angles are the joint angle start point for IK searches
    # default start angles and location are for moving the arm to the side
    def move_arm_constrained(self, start_angles = None, location = None, blocking = 1):
        current_pose = self.get_current_wrist_pose_stamped('base_link')
        move_arm_goal = self.create_move_arm_goal()
        # default search-starting arm angles are arm-to-the-side
        if start_angles == None:
            if self.whicharm == 'l': start_angles = [2.135, 0.803, 1.732, -1.905, 2.369, -1.680, 1.398]
            else: start_angles = [-2.135, 0.803, -1.732, -1.905, -2.369, -1.680, 1.398]
        # default location is arm-to-the-side
        if location == None:
            if self.whicharm == 'l': location = [0.05, 0.576, 0.794]
            else: location = [0.05, -0.576, 0.794]
        # add a position constraint for the goal
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = current_pose.header.frame_id
        position_constraint.link_name = self.ik_utilities.link_name #r_ or l_wrist_roll_link
        set_xyz(position_constraint.position, location)
        position_constraint.constraint_region_shape.type = Shape.BOX
        position_constraint.constraint_region_shape.dimensions = [0.02]*3
        position_constraint.constraint_region_orientation.w = 1.0
        position_constraint.weight = 1.0
        move_arm_goal.motion_plan_request.goal_constraints.position_constraints.append(position_constraint)
        # search for a feasible goal pose that gets our wrist to location while keeping all but the (base-link) yaw fixed
        current_pose_mat = pose_to_mat(current_pose.pose)
        current_pose_mat = pose_to_mat(current_pose.pose)
        found_ik_solution = 0
        for angle in range(0, 180, 5):
            for dir in [1, -1]:
                rot_mat = tf.transformations.rotation_matrix(dir*angle/180.*math.pi, [0,0,1])
                rotated_pose_mat = rot_mat * current_pose_mat
                desired_pose = stamp_pose(mat_to_pose(rotated_pose_mat), 'base_link')
                desired_pose.pose.position = position_constraint.position
                # check if there's a collision-free IK solution at the goal location with that orientation
                (solution, error_code) = self.ik_utilities.run_ik(desired_pose, start_angles, self.ik_utilities.link_name, collision_aware = 1)
                if error_code == "SUCCESS":
                    found_ik_solution = 1
                    break
            if found_ik_solution: break
            else:
                rospy.loginfo("no IK solution found for goal, aborting")
                return 0
        # add an orientation constraint for the goal
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.stamp = rospy.Time.now()
        orientation_constraint.header.frame_id = current_pose.header.frame_id
        orientation_constraint.link_name = self.ik_utilities.link_name
        orientation_constraint.orientation = desired_pose.pose.orientation
        orientation_constraint.type = OrientationConstraint.HEADER_FRAME
        orientation_constraint.absolute_roll_tolerance = 0.2
        orientation_constraint.absolute_pitch_tolerance = 0.5
        orientation_constraint.absolute_yaw_tolerance = 2*math.pi
        orientation_constraint.weight = 1.0
        move_arm_goal.motion_plan_request.goal_constraints.orientation_constraints.append(orientation_constraint)

        # add an orientation constraint for the entire path to keep the object upright
        path_orientation_constraint = copy.deepcopy(orientation_constraint)
        # temporary, while deepcopy of rospy.Time is broken
        path_orientation_constraint.header.stamp = rospy.Time(orientation_constraint.header.stamp.secs)
        path_orientation_constraint.orientation = current_pose.pose.orientation
        move_arm_goal.motion_plan_request.path_constraints.orientation_constraints.append(path_orientation_constraint)
        # send the goal off to move arm, blocking if desired and modifying the start state if it's in collision
        result = self.send_move_arm_goal(move_arm_goal, blocking)
        return result

    # use move_arm to get to a desired pose in a collision-free way (really, run IK and then call move arm to move to the IK joint angles)
    def move_arm_pose(self, pose_stamped, start_angles = None, blocking = 1):
        if start_angles == None:
            start_angles = self.get_current_arm_angles()
        (solution, error_code) = self.ik_utilities.run_ik(pose_stamped, start_angles, self.ik_utilities.link_name)
        if not solution:
            rospy.logerr("no IK solution found for goal pose!")
            return 0
        else:
            result = self.move_arm_joint(solution, blocking)
            return result

    # normalize a trajectory (list of lists of joint angles), so that the desired angles
    # are the nearest ones for the continuous joints (5 and 7)
    def normalize_trajectory(self, trajectory):
        current_angles = self.get_current_arm_angles()
        trajectory_copy = [list(angles) for angles in trajectory]
        for angles in trajectory_copy:
            angles[4] = self.normalize_angle(angles[4], current_angles[4])
            angles[6] = self.normalize_angle(angles[6], current_angles[6])
        return trajectory_copy

    # normalize an angle for a continuous joint so that it's the closest version
    # of the angle to the current angle (not +-2*pi)
    def normalize_angle(self, angle, current_angle):
        while current_angle-angle > math.pi: angle += 2*math.pi
        while angle - current_angle > math.pi: angle -= 2*math.pi
        return angle

    # find and execute a joint-angle trajectory to perform a collision-free Cartesian movement
    # if start_pose is None, starts from the current pose (if not None, will go straight to the start pose using the joint trajectory action)
    def command_interpolated_ik(self, end_pose, start_pose = None, collision_aware = 1, step_size = .02, max_joint_vel = .05):
        self.check_controllers_ok('joint')
        # find the current joint angles
        current_angles = self.get_current_arm_angles()
        if start_pose == None:
            # use the current wrist pose as the start pose/angles
            (current_trans, current_rot) = self.return_cartesian_pose()
            # put the current pose into a poseStamped
            start_pose = create_pose_stamped(current_trans+current_rot)
        (trajectory, error_codes) = self.ik_utilities.check_cartesian_path(start_pose, end_pose, current_angles, step_size, .1, math.pi/4, collision_aware)
        # if some point is not possible, quit
        if any(error_codes): 
            rospy.loginfo("can't execute an interpolated IK trajectory to that pose")
            return 0
        self.command_joint_trajectory(trajectory, max_joint_vel)
        return 1

    # use the joint_states_listener to get the arm angles
    def get_current_arm_angles(self):
        (found, positions, vels, efforts) = self.joint_states_listener.return_joint_states(self.joint_names)
        return positions

    # return the current pose of the wrist as a PoseStamped
    def get_current_wrist_pose_stamped(self, frame = 'base_link'):
        (current_trans, current_rot) = self.return_cartesian_pose(frame)
        return create_pose_stamped(current_trans+current_rot, frame)

    # use the joint_states_listener to get the current gripper opening
    def get_current_gripper_opening(self):
        (found, positions, vels, efforts) = self.joint_states_listener.return_joint_states([self.whicharm+'_gripper_joint'])
        return positions[0]
        
    # send a single desired pose to the Cartesian controller
    # (pose is either a PoseStamped or a 7-list of position and quaternion orientation; if the latter, put the frame in frame_id)
    def command_cartesian(self, pose, frame_id = 'base_link'):
        self.check_controllers_ok('cartesian')
        if type(pose) == list: m = create_pose_stamped(pose, frame_id)
        else: m = pose
        req = MoveToPoseRequest()
        req.pose = m
        try:
            self.cartesian_preempt_service(EmptyRequest())
        except:
            #rospy.loginfo("preempt unnecessary, robot not moving")
            pass
        else:
            rospy.loginfo("Cartesian movement preempted before issuing new command")
        self.cartesian_cmd_service(req)
        # self.cartesian_cmd_pub.publish(m)

    # check if the Cartesian controller thinks it's gotten to its goal (it's pretty loose about goals)
    def check_cartesian_done(self):
        resp = self.cartesian_moving_service()
        return resp.ismoving

    # check if we're near to a desired Cartesian pose (either PoseStamped or 7-list of position and quaternion in 'base_link' frame) by pos_thres(m) and rot_thres (rad)
    def check_cartesian_near_pose(self, goal_pose, pos_thres, rot_thres):
        if type(goal_pose) == list:
            goal_pos = goal_pose[0:3]
            goal_rot = goal_pose[3:7]
        else: (goal_pos, goal_rot) = pose_stamped_to_lists(self.tf_listener, goal_pose, 'base_link')
        (current_pos, current_rot) = self.return_cartesian_pose()
        # compute how far the wrist is translated from the goal
        diff = [x-y for (x,y) in zip(goal_pos, current_pos)]
        pos_diff = self.ik_utilities.vect_norm(diff)
        # compute how far the wrist is rotated from the goal
        norm_goal_rot = self.ik_utilities.normalize_vect(goal_rot)
        norm_current_rot = self.ik_utilities.normalize_vect(current_rot)
        rot_diff = self.ik_utilities.quat_angle(norm_current_rot, norm_goal_rot)
        # rospy.loginfo("pos_diff: %5.3f, rot_diff: %5.3f"%(pos_diff, rot_diff))
        if pos_diff < pos_thres and rot_diff < rot_thres: return 1
        return 0

    # check if both the Cartesian controllers think we're done and if we're close to where we want to be
    def check_cartesian_really_done(self, goal_pose, pos_thres, rot_thres):
        if not self.check_cartesian_done(): return 0
        return self.check_cartesian_near_pose(goal_pose, pos_thres, rot_thres)

    # wait for either the Cartesian pose to get near enough to a goal pose or timeout (a ROS duration) is reached
    # or settling_time (a ROS duration) after the Cartesian controllers report that they're done
    def wait_cartesian_really_done(self, goal_pose, pos_thres = .02, rot_thres = .1, timeout = 0., settling_time = rospy.Duration(3.0)):
        start_time = rospy.get_rostime()
        done_time = None
        while(1):
            if self.check_cartesian_really_done(goal_pose, pos_thres, rot_thres):
                rospy.loginfo("got to the goal")
                return "success"
            if done_time == None and self.check_cartesian_done():
                #rospy.loginfo("Cartesian controllers think they're done")
                done_time = rospy.get_rostime()
            if done_time and rospy.get_rostime() - done_time > settling_time:
                rospy.loginfo("settling time ran out")
                return "failed"
            if timeout != 0. and rospy.get_rostime() - start_time > timeout:
                rospy.loginfo("timed out")
                return "timed out"

    # move in Cartesian space using the Cartesian controllers
    # if collision_aware, checks whether the path could be collision-free, but uses the Cartesian controllers to execute it
    # if blocking, waits for completion, otherwise returns after sending the goal
    # step_size only used if collision aware (size of steps to check for collisions)
    # pos_thres and rot_thres are thresholds within which the goal is declared reached
    # times out and quits after timeout, and waits settling_time after the controllers declare that they're done (which is usually way too early)
    def move_cartesian(self, goal_pose, collision_aware = 0, blocking = 1,
                       step_size = .005, pos_thres = .02, rot_thres = .1,
                       timeout = rospy.Duration(30.),
                       settling_time = rospy.Duration(1.)):
        # get the current wrist pose as the start pose
        (current_pos, current_rot) = self.return_cartesian_pose('base_link')
        start_pose = create_pose_stamped(current_pos+current_rot)
        # check to see that the path could possibly be collision-free
        if collision_aware:
            current_angles = self.get_current_arm_angles()
            (traj, error_codes) = self.ik_utilities.check_cartesian_path(start_pose, goal_pose, current_angles, .01, .1, math.pi/4, 1)
            if any(error_codes): return "no solution"
        # go to the goal pose using the Cartesian controllers
        self.command_cartesian(goal_pose)
        if not blocking: return "sent goal"
        #blocking: wait for the Cartesian controllers to get there or time out
        return self.wait_cartesian_really_done(goal_pose, pos_thres, rot_thres, timeout = timeout, settling_time = settling_time)        

    # move in Cartesian space using IK Cartesian interpolation
    # if collision_aware, quits if the path is not collision-free
    # if blocking, waits for completion, otherwise returns after sending the goal
    # step_size is the step size for interpolation
    # pos_thres and rot_thres are thresholds within which the goal is declared reached
    # times out and quits after timeout, and waits settling_time after the
    # controllers declare that they're done
    def move_cartesian_ik(self, goal_pose, collision_aware = 0, blocking = 1,
                          step_size = .005, pos_thres = .02, rot_thres = .1,
                          timeout = rospy.Duration(30.),
                          settling_time = rospy.Duration(0.25), vel = .15):
        # send the interpolated IK goal to the joint trajectory action
        result = self.command_interpolated_ik(goal_pose, collision_aware = collision_aware, \
                                                  step_size = step_size, max_joint_vel = vel)
        if not result: return "no solution"
        if not blocking: return "sent goal"
        # blocking: wait for the joint trajectory action to get there or time out
        joint_action_result = self.wait_joint_trajectory_done(timeout)
        if joint_action_result == "timed out": return "timed out"
        if self.check_cartesian_really_done(goal_pose, pos_thres, rot_thres): return "success"
        return "failed"

    # send a single joint configuration to the joint trajectory action (takes in a 7-list of angles, nonblocking)
    def command_joint(self, jointangles):
        self.check_controllers_ok('joint')
        goal = JointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.get_rostime()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint(jointangles, [0]*7, [0]*7, rospy.Duration(5.0))
        goal.trajectory.points = [point,]
        self.joint_action_client.send_goal(goal)

    # send an entire joint trajectory to the joint trajectory action (nonblocking)
    def command_joint_trajectory(self, trajectory, max_joint_vel = 0.2, current_angles = None, blocking = 0):
        self.check_controllers_ok('joint')
        goal = JointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        # normalize the trajectory so the continuous joints angles are close to the current angles
        trajectory = self.normalize_trajectory(trajectory)
        # start the trajectory 0.05 s from now
        goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.05)
        # get the current joint angles
        if current_angles == None:
            current_angles = list(self.get_current_arm_angles())
        # set the first trajectory point to the current position
        trajectory = [current_angles] + trajectory
        # find appropriate times and velocities for the trajectory
        (times, vels) = self.ik_utilities.trajectory_times_and_vels(trajectory, [max_joint_vel]*7)
#       print "trajectory:"
#       for point in trajectory:
#           print self.pplist(point)
#           print "times:", self.pplist(times)
#           print "vels:"
#           for vel in vels:
#               print self.pplist(vel)
        # fill in the trajectory message
        for ind in range(len(trajectory)):
            point = JointTrajectoryPoint(trajectory[ind], vels[ind], [0]*7, rospy.Duration(times[ind]))
            goal.trajectory.points.append(point)
        # send the goal
        self.joint_action_client.send_goal(goal)
        # if blocking, wait until it gets there
        if blocking:
            self.wait_joint_trajectory_done(timeout = rospy.Duration(times[-1]+5))

    # check the state of the joint trajectory action (returns 1 if done, 0 otherwise)
    def check_joint_trajectory_done(self):
        state = self.joint_action_client.get_state()
        return state > 1

    # wait for the joint trajectory action to finish (returns 1 if succeeded, 0 otherwise)
    # timeout is a ROS duration; default (0) is infinite timeout
    # returns 0 if timed out, 1 if succeeded
    def wait_joint_trajectory_done(self, timeout = rospy.Duration(0)):
        finished = self.joint_action_client.wait_for_result(timeout)
        if not finished:
            return "timed out"
        state = self.joint_action_client.get_state()
        if state == 3:
            return "succeeded"
        return "other"

    # tell the joint trajectory action to stop the arm where it is now
    def freeze_arm(self):
        self.joint_action_client.cancel_all_goals()
        current_angles = self.get_current_arm_angles()
        self.command_joint(current_angles)
        

    # send a command to the gripper action
    def command_gripper(self, position, max_effort, blocking = 0):
        if not self.gripper_controller_state == 'running':
            self.check_controller_states()
            if not self.gripper_controller_state == 'running':
                rospy.logwarn("gripper controller not running!  Attempting to start")
                self.start_gripper_controller()
        goal = Pr2GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        self.gripper_action_client.send_goal(goal)
        # if blocking, wait for the gripper to finish
        if blocking:
            self.gripper_action_client.wait_for_result(rospy.Duration(4.0))
            time.sleep(.5)

    # return the current Cartesian pose of the gripper
    def return_cartesian_pose(self, frame = 'base_link'):
        (trans, rot) = self.tf_listener.lookupTransform(frame, self.whicharm+'_wrist_roll_link', rospy.Time(0))
        return (list(trans), list(rot))

    # print the current Cartesian pose of the gripper
    def print_cartesian_pose(self):
        (trans, rot) = self.return_cartesian_pose()
        print "Cartesian pose trans:", self.pplist(trans), "rot:", self.pplist(rot)
        print "rotation matrix:\n", self.ppmat(tf.transformations.quaternion_matrix(rot))

    # pretty-print list to string
    def pplist(self, list):
        return ' '.join(['%5.3f'%x for x in list])

    # pretty-print numpy matrix to string
    def ppmat(self, mat):
        str = ''
        for i in range(mat.shape[0]):
            for j in range(mat.shape[1]):
                str += '%2.3f\t'%mat[i,j]
            str += '\n'
        return str

# sample test cases
if __name__ == '__main__':
    def keypause():
        print "press enter to continue"
        raw_input()
    rospy.init_node('controller_manager', anonymous=True)
    # points 0-2 and 6-7 have IK solutions; 3-5 don't (but the cartesian-plain controllers will try to get you there anyway)
    points = [
        [0.5, -0.5, 0.8, 0.5, 0.0, 0.0, 0.5],
        [0.6, -0.2, 0.4, 0.0, 0.0, 0.5, 0.5],
        [0.2, -0.8, 0.4, 0.0, 0.5, 0.0, 0.5],
        [0.5, -0.5, 1.2, 0.5, 0.0, 0.0, 0.5],
        [0.6, -0.2, 1.2, 0.0, 0.0, 0.5, 0.5],
        [0.2, -0.8, 1.2, 0.0, 0.5, 0.0, 0.5],
        [.62, -.05, .65, -0.5, 0.5, 0.5, 0.5],
        [.62, -.05, .56, -0.5, 0.5, 0.5, 0.5]
        ]
    zeros = [0]*7
    sideangles = [-0.447, -0.297, -2.229, -0.719, 0.734, -1.489, 1.355]
    
    cm = ArmManager('r')
    test_angles = [
        [-0.094, 0.711, -0.000, -1.413, -2.038, -1.172, -0.798],
        [0.126, 0.832, 0.000, -1.740, -2.977, -1.091, 3.043]]

    print "going through test_angles and waiting for the trajectory to finish"
    cm.command_joint_trajectory(test_angles)
    result = cm.wait_joint_trajectory_done(rospy.Duration(30.0))
    print result

    print "starting gripper controller"
    cm.start_gripper_controller()
    
    print "switching to cartesian controllers"
    cm.switch_to_cartesian_mode()
    
    print "moving to point 6 and waiting for it to get there"
    cm.command_cartesian(points[6])
    result = cm.wait_cartesian_really_done(points[6], .01, .1, rospy.Duration(30.0), rospy.Duration(10.0))
    print result
    keypause()                                                                                                                                                
    print "desired pose:", cm.pplist(points[6])
    cm.print_cartesian_pose()

    print "going to all 0 angles with joint controllers, testing contingency-switch"
    cm.command_joint(zeros)
    keypause()

    print "opening gripper"
    cm.command_gripper(0.08, -1.0)
    keypause()

    print "going to near-side-grasp angles"
    cm.command_joint(sideangles)
    keypause()

    print "commanding an interpolated IK trajectory to go from side approach to side grasp"
    start_angles = sideangles
    tiltangle = math.pi/18.
    sideapproachmat = numpy.array([[0., -1., 0., 0.],
                                   [math.cos(tiltangle), 0., math.sin(tiltangle), 0.],
                                   [-math.sin(tiltangle), 0., math.cos(tiltangle), 0.],
                                   [0., 0., 0., 1.]])
    sideapproachpos = [.62, -.3, .6]
    approachmat = sideapproachmat
    approachpos = sideapproachpos
    approachquat = list(tf.transformations.quaternion_from_matrix(approachmat))
    sidegrasppos = sideapproachpos[:]
    sidegrasppos[1] += .05
    grasppos = sidegrasppos
    graspquat = approachquat[:]

    start_pose = create_pose_stamped(approachpos+approachquat)
    end_pose = create_pose_stamped(grasppos+graspquat)

    cm.command_interpolated_ik(end_pose, start_pose)
    keypause()

    print "closing gripper (blocking)"
    cm.command_gripper(0.0, 50.0, 1)
    print "done"

    print "moving to point 0 in Cartesian mode, testing contingency switch"
    cm.command_cartesian(points[0])
    keypause()
    cm.print_cartesian_pose()

    print "reloading cartesian controllers with zero PID terms (should be unable to move)"
    cm.cart_params.pose_fb_trans_p = 0.
    cm.cart_params.pose_fb_trans_d = 0.
    cm.cart_params.pose_fb_rot_p = 0.
    cm.cart_params.pose_fb_rot_d = 0.
    cm.reload_cartesian_controllers()

    print "trying to move to point 2 (should fail)"
    cm.command_cartesian(points[2])
    keypause()
    
    print "reloading cartesian controllers with normal PID terms"
    cm.cart_params.pose_fb_trans_p=800.
    cm.cart_params.pose_fb_trans_d=12.
    cm.cart_params.pose_fb_rot_p=80.
    cm.cart_params.pose_fb_rot_d=0.0
    cm.reload_cartesian_controllers()

    print "trying again to move to point 2 (should succeed)"
    cm.command_cartesian(points[2])
    keypause()
    
    print "starting joint controllers and going to all 0 angles"
    cm.command_joint([0,0,0,0,0,0,0])
    keypause()
    
    print "stopping all controllers"
    cm.stop_all_controllers()
