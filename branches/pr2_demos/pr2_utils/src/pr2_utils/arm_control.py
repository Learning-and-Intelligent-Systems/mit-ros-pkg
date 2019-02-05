#!/usr/bin/env python


import roslib; roslib.load_manifest('pr2_utils')
import rospy
import random
import geometry_msgs.msg
import actionlib
import pr2_pick_and_place_demos.pick_and_place_manager
import kinematics_msgs.srv
import trajectory_msgs.msg
import pr2_controllers_msgs.msg
import math
import interpolated_ik_motion_planner.srv
import actionlib_msgs.msg
import arm_navigation_msgs.srv
import arm_navigation_msgs.msg
import copy
import ee_cart_imped_action 
import pr2_controllers_msgs.msg

import pr2_utils.conversions
import pr2_utils.utils
import ee_cart_imped_control.control_switcher

PLANNER_NAME='/ompl_planning/plan_kinematic_path'
R_PLANNER_ID='RRTkConfig1'
L_PLANNER_ID='RRTkConfig1'
INT_IK_MAX_STEP_SIZE = 0.01
INT_IK_CONSISTENT_ANGLE = math.pi/6.0
INT_IK_POS_SPACING = 0.01
INT_IK_ROT_SPACING = 0.1
INT_IK_COLLISION_CHECK_RESOLUTION = 2
SHORTEST_FILTERABLE_TRAJECTORY = 3
ROBOT_ROOT_FRAME = '/odom_combined'

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
        self.curr_controller = ''
        self.switchToForce()
        """
        Simple action client to deal with the ee_cart_imped_controller.
        """
        self.force_control = ee_cart_imped_action.EECartImpedClient(arm_name)
        self.switchToCartesian()
        self.move_arm_client =\
            actionlib.SimpleActionClient('/move_'+self.arm_name,\
                                             arm_navigation_msgs.msg.MoveArmAction)
        rospy.loginfo('Waiting for move arm client')
        self.move_arm_client.wait_for_server()
        self.move_arm_planner = rospy.ServiceProxy\
            (PLANNER_NAME, arm_navigation_msgs.srv.GetMotionPlan)
        rospy.loginfo('Waiting for move arm planner')
        self.move_arm_planner.wait_for_service()
        self.traj_client =\
            actionlib.SimpleActionClient\
            ('/'+self.arm_name[0]+'_arm_controller/joint_trajectory_action',\
                 pr2_controllers_msgs.msg.JointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action server')
        self.traj_client.wait_for_server()
        self.interpolated_ik_planning_service =\
            rospy.ServiceProxy('/'+self.arm_name[0]+\
                                   '_interpolated_ik_motion_plan',\
                                   arm_navigation_msgs.srv.GetMotionPlan)
        rospy.loginfo('Waiting for interpolated IK planning service')
        self.interpolated_ik_planning_service.wait_for_service()
        self.interpolated_ik_parameter_service =\
            rospy.ServiceProxy('/'+self.arm_name[0]+\
                                   '_interpolated_ik_motion_plan_set_params',\
                                   interpolated_ik_motion_planner.srv.\
                                   SetInterpolatedIKMotionPlanParams)
        rospy.loginfo('Waiting for interpolated IK parameter setting service')
        self.interpolated_ik_parameter_service.wait_for_service()
        self.filter_trajectory_with_constraints_service = rospy.ServiceProxy\
            ('/trajectory_filter_server/filter_trajectory_with_constraints',\
                 arm_navigation_msgs.srv.FilterJointTrajectoryWithConstraints)
        rospy.loginfo('Waiting for trajectory filter with constraints service')
        self.filter_trajectory_with_constraints_service.wait_for_service()
        self.kinematics_info_service = rospy.ServiceProxy\
            ('/pr2_'+self.arm_name+ '_kinematics/get_ik_solver_info',\
                 kinematics_msgs.srv.GetKinematicSolverInfo)
        rospy.loginfo('Waiting for kinematics solver info service')
        self.kinematics_info_service.wait_for_service()
        self.get_state_validity_service = rospy.ServiceProxy\
            ('/planning_scene_validity_server/get_state_validity',\
                 arm_navigation_msgs.srv.GetStateValidity)
        rospy.loginfo('Waiting for state validity service.')
        self.get_state_validity_service.wait_for_service()
        self.collision_aware_ik_service = rospy.ServiceProxy\
            ('/pr2_'+self.arm_name+'_kinematics/get_constraint_aware_ik',\
                 kinematics_msgs.srv.GetConstraintAwarePositionIK)
        rospy.loginfo('Waiting for collision aware IK service')
        self.collision_aware_ik_service.wait_for_service()
        self.ik_service = rospy.ServiceProxy\
            ('/pr2_'+self.arm_name+'_kinematics/get_ik',\
                 kinematics_msgs.srv.GetPositionIK)
        rospy.loginfo('Waiting for IK service')
        self.collision_aware_ik_service.wait_for_service()
        self.set_planning_scene_service = rospy.ServiceProxy\
            ('/environment_server/set_planning_scene_diff',\
                 arm_navigation_msgs.srv.SetPlanningSceneDiff)
        rospy.loginfo('Waiting for set planning scene service')
        self.set_planning_scene_service.wait_for_service()
        self.get_planning_scene_service = rospy.ServiceProxy\
            ('/environment_server/get_planning_scene',\
                 arm_navigation_msgs.srv.GetPlanningScene)
        rospy.loginfo('Waiting for get planning scene service')
        self.get_planning_scene_service.wait_for_service()
        self.gripper_client = actionlib.SimpleActionClient\
            (self.arm_name[0]+'_gripper_controller/gripper_action',\
                 pr2_controllers_msgs.msg.Pr2GripperCommandAction)
        rospy.loginfo('Waiting for gripper action')
        self.gripper_client.wait_for_server()
        self.manager = manager
        if not self.manager:
            self.manager =\
                pr2_pick_and_place_demos.pick_and_place_manager.\
                PickAndPlaceManager()
        """
        Transform listener that allows the arm control to get the location of the hand in the torso_lift_link frame.
        """
	self.tf_listener = self.manager.tf_listener
        rospy.loginfo("Arm controller created")

    def open_gripper(self):
        self.command_gripper(0.08)
    def close_gripper(self):
        self.command_gripper(0.0)

    def command_gripper(self, position, effort=-1):
        goal = pr2_controllers_msgs.msg.Pr2GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort=effort
        self.gripper_client.send_goal_and_wait(goal)

    def execute_joint_trajectory(self, trajectory):
        self.switchToCartesian()
        goal = pr2_controllers_msgs.msg.JointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)
        self.traj_client.send_goal_and_wait\
            (goal, rospy.Duration(4.0) +\
                 goal.trajectory.points[len(goal.trajectory.points)-1].\
                 time_from_start)
        return (self.traj_client.get_result(), self.traj_client.get_state())

    def getIKSolution(self, pose_stamped,
                      collision_aware=True, starting_state=None,
                      seed_state=None):
        pos_req = kinematics_msgs.msg.PositionIKRequest()
        pos_req.ik_link_name = self.arm_name[0]+'_wrist_roll_link'
        pos_req.pose_stamped = pose_stamped
        if not starting_state: 
            starting_state = pr2_utils.utils.currentRobotState()
        if not seed_state:
            seed_state = pr2_utils.utils.currentRobotArmState(self.arm_name)
        pos_req.ik_seed_state = seed_state
        pos_req.robot_state = starting_state
        if collision_aware:
            coll_req = kinematics_msgs.srv.GetConstraintAwarePositionIKRequest()
            coll_req.ik_request = pos_req
            coll_req.timeout = rospy.Duration(10.0)
            return self.collision_aware_ik_service(coll_req)
        coll_req = kinematics_msgs.srv.GetPositionIKRequest()
        coll_req.ik_request = pos_req
        coll_req.timeout = rospy.Duration(10.0)
        return self.ik_service(coll_req)

    def try_hard_for_pose(self, pose_stamped, ordered_colls=
                          arm_navigation_msgs.msg.OrderedCollisionOperations(),
                          bounds=(0.01,0.01,0.01,0.1,0.1,0.1),
                          revert_scene=True):
        self.move_into_joint_limits()
        #first try with move_arm
        (ma_r, ma_s) = self.move_arm_collision_free\
            (pose_stamped, ordered_colls=ordered_colls, bounds=bounds,
             revert_scene=revert_scene)
        if ma_r.error_code.val == ma_r.error_code.SUCCESS and\
                ma_s == actionlib_msgs.msg.GoalStatus.SUCCEEDED:
            return True
        #try ignoring all collisions
        ordered_colls = arm_navigation_msgs.msg.OrderedCollisionOperations()
        coll = arm_navigation_msgs.msg.CollisionOperation()
        coll.object1 = coll.COLLISION_SET_ALL
        coll.object2 = coll.COLLISION_SET_ALL
        coll.operation = coll.DISABLE
        ordered_colls.collision_operations.append(coll)
        (ma_r, ma_s) = self.move_arm_collision_free\
            (pose_stamped, ordered_colls=ordered_colls, bounds=bounds,
             revert_scene=revert_scene)
        if ma_r.error_code.val == ma_r.error_code.SUCCESS and\
                ma_s == actionlib_msgs.msg.GoalStatus.SUCCEEDED:
            return True

        if self.arm_name[0] == 'l':
            whicharm = 1
        else:
            whicharm = 0
        #try moving open loop
        if self.manager.try_hard_to_move_pose(whicharm, pose_stamped,
                                              max_tries=0,
                                              use_joint_open_loop=1,
                                              use_cartesian=0) == 1:
            return True
        #use the force controller (the Cartesian one has some major issues)
        #note: slightly problematic since this controls the gripper
        #instead of the wrist... oh well.
        self.move_arm_force_control(pose_stamped, 8.0)
        return False

    def move_arm_to_side(self, revert_scene=True, ordered_colls=
                         arm_navigation_msgs.msg.OrderedCollisionOperations()):
        side_pose = geometry_msgs.msg.PoseStamped()
        side_pose.pose.position.x = 0
        side_pose.pose.position.y = -0.6
        if self.arm_name[0] == 'l':
            side_pose.pose.position.y = 0.6
        side_pose.pose.position.z = -0.25
        side_pose.pose.orientation.x = -0.5
        side_pose.pose.orientation.y = 0.5
        side_pose.pose.orientation.z = 0.5
        side_pose.pose.orientation.w = 0.5
        side_pose.header.frame_id = '/torso_lift_link'
        bounds = []
        bounds.append(0.01) #it's important to get x right
        for i in range(5): bounds.append(0.1)
        self.try_hard_for_pose\
            (side_pose, ordered_colls=ordered_colls, bounds=bounds,
             revert_scene=revert_scene)

    def outside_joint_limits(self):
        kin_resp = self.kinematics_info_service()
        state = pr2_utils.utils.currentRobotArmState(self.arm_name)
        info = kin_resp.kinematic_solver_info
        joints = state.joint_state
        trajectory = trajectory_msgs.msg.JointTrajectory()
        trajectory.header.frame_id = joints.header.frame_id
        trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
        trajectory.points[0].time_from_start = rospy.Duration(2.0)
        trajectory_required = None
        for i in range(len(joints.name)):
            trajectory.joint_names.append(joints.name[i])
            added = False
            nc = 0
            for limit in info.limits:
                if info.joint_names[nc] == joints.name[i]:
                    if limit.has_position_limits:
                        if joints.position[i] < limit.min_position:
                            trajectory.points[0].positions.append\
                                (limit.min_position + 0.01)
                            trajectory_required = True
                            added = True
                        elif joints.position[i] > limit.max_position:
                            trajectory.points[0].positions.append\
                                (limit.max_position - 0.01)
                            trajectory_required = True
                            added = True
                    break
                nc += 1
            if not added:
                trajectory.points[0].positions.append\
                    (joints.position[i])
        if not trajectory_required:
            return None
        return trajectory
                        
    
    def move_into_joint_limits(self):
        trajectory = self.outside_joint_limits()
        if trajectory != None:
            rospy.loginfo('Moving into joint limits open-loop')
            return self.execute_joint_trajectory(trajectory)
        return None

    def move_arm_collision_free\
            (self, pose_stamped, ordered_colls =\
                 arm_navigation_msgs.msg.OrderedCollisionOperations(),\
                 timeout=rospy.Duration(60.0),\
                 bounds=(0.01, 0.01, 0.01, 0.1, 0.1, 0.1),\
                 do_filter=True, revert_scene=True):
        if revert_scene:
            self.revert_planning_scene()
        self.move_into_joint_limits()
        self.switchToCartesian()
        goal = arm_navigation_msgs.msg.MoveArmGoal()
        goal.planner_service_name=PLANNER_NAME
        (pconstraint, oconstraint) =\
            pr2_utils.conversions.\
            poseStampedToPositionOrientationConstraints\
            (pose_stamped, self.arm_name[0]+'_wrist_roll_link', bounds=bounds)
        goal.motion_plan_request.goal_constraints.position_constraints.append\
            (pconstraint)
        goal.motion_plan_request.goal_constraints.orientation_constraints.\
            append(oconstraint)
        goal.motion_plan_request.start_state =\
            pr2_utils.utils.currentRobotState()
        if self.arm_name[0] == 'l':
            goal.motion_plan_request.planner_id = L_PLANNER_ID
        else:
            goal.motion_plan_request.planner_id = R_PLANNER_ID
        goal.motion_plan_request.group_name = self.arm_name
        goal.motion_plan_request.num_planning_attempts = 2
        #should be 45!!
        goal.motion_plan_request.allowed_planning_time = rospy.Duration(30.0)
        goal.operations = ordered_colls
        if do_filter:
            self.move_arm_client.send_goal_and_wait(goal, timeout)
            result = self.move_arm_client.get_result()
            state = self.move_arm_client.get_state()
        else:
            #use our own planning so we can control it a little more
            plan = self.plan_to_move_arm_collision_free\
                (goal, revert_scene=revert_scene)
            if plan.error_code.val != plan.error_code.SUCCESS:
                state = actionlib_msgs.msg.GoalStatus.ABORTED
                return plan, state
            self.convert_path_to_trajectory(plan.trajectory.joint_trajectory)
            trajectory = plan.trajectory.joint_trajectory
            result, state = self.execute_joint_trajectory\
                (trajectory)
            return plan, state
        if result.error_code.val != result.error_code.SUCCESS:
            rospy.logerr('Move arm failed with error code %d',
                         result.error_code.val)
        rospy.logdebug('Move arm returned in state %s with result %s', 
                       str(state),
                       str(result))        
        return (result, state)

    

    def plan_to_move_arm_collision_free(self, goal, revert_scene=True):
        if revert_scene:
            self.revert_planning_scene()
        self.set_ordered_collisions(goal.operations)
        return self.move_arm_planner(goal.motion_plan_request)
        
    def convert_path_to_trajectory(self, trajectory):
        '''
        Take a trajectory returned by a planner and add
        a small amount of time for each step.  Also properly
        wrap angles so that the shortest path is taken.  Note:
        it is usually a better idea to use the full filter_trajectory
        function rather than this one.
        '''
        i = 1
        for p in trajectory.points:
            p.time_from_start = rospy.Duration(i*0.05)
            i+=1
        for j in range(1, len(trajectory.points)):
            newpositions = []
            for i in range(len(trajectory.joint_names)):
                curr = trajectory.points[j].positions[i]
                if trajectory.joint_names[i][1:] ==\
                        '_forearm_roll_joint' or\
                        trajectory.joint_names[i][1:] ==\
                        '_wrist_roll_joint':
                    #this really should check if the joints
                    #are revolute... not this silly thing
                    prev = trajectory.points[j-1].positions[i]
                    while prev - curr > math.pi:
                        curr += 2.0*math.pi
                    while curr - prev > math.pi:
                        curr -= 2.0*math.pi
                newpositions.append(curr)
            trajectory.points[j].positions = tuple(newpositions)
    

    def move_arm_force_control(self, pose_stamped, time):
        self.switchToForce()
        print 'MOVING TO', pose_stamped
        self.force_control.moveToPoseStamped(pose_stamped, time)
        self.force_control.resetGoal()

    def filter_trajectory(self, motion_req, trajectory):
        if len(trajectory.points) < SHORTEST_FILTERABLE_TRAJECTORY:
            rospy.logwarn('Not filtering trajectory of length %d.  Too small.',
                          len(trajectory.points))
            return trajectory
        req = arm_navigation_msgs.srv.\
            FilterJointTrajectoryWithConstraintsRequest()
        req.trajectory = trajectory
        #these have to be in the root frame of the robot
        #go figure
        req.path_constraints = pr2_utils.conversions.transform_constraints\
            (ROBOT_ROOT_FRAME, motion_req.path_constraints, self.tf_listener)
        req.goal_constraints = pr2_utils.conversions.transform_constraints\
            (ROBOT_ROOT_FRAME, motion_req.goal_constraints, self.tf_listener)
        req.start_state = motion_req.start_state
        req.group_name = motion_req.group_name
        req.allowed_time = rospy.Duration(1.5)
        goodcall = True
        traj_resp = arm_navigation_msgs.srv.FilterJointTrajectoryResponse()
        traj_resp.error_code.val = traj_resp.error_code.PLANNING_FAILED
        try:
            traj_resp = self.filter_trajectory_with_constraints_service(req)
        except rospy.ServiceException:
            goodcall = False
        if not goodcall or\
                traj_resp.error_code.val != traj_resp.error_code.SUCCESS:
            rospy.logerr('Unable to filter trajectory with constraints')
            return self.convert_path_to_trajectory(trajectory)
        return traj_resp.trajectory

    def plan_interpolated_ik\
            (self, pose_stamped, ordered_colls=\
                 arm_navigation_msgs.msg.OrderedCollisionOperations(),\
                 bounds = (0.01, 0.01, 0.01, 0.1, 0.1, 0.1),\
                 starting_pose=None, starting_state=None, min_dist=None,\
                 revert_scene=True):
        if revert_scene:
            self.revert_planning_scene()
        changed_planning_scene = False
        link_name = self.arm_name[0] + '_wrist_roll_link'
        goal = arm_navigation_msgs.msg.MotionPlanRequest()
        (pconstraint, oconstraint) =\
            pr2_utils.conversions.\
            poseStampedToPositionOrientationConstraints\
            (pose_stamped, link_name, bounds=bounds)
        goal.goal_constraints.position_constraints.append(pconstraint)
        goal.goal_constraints.orientation_constraints.append(oconstraint)
        goal.group_name = self.arm_name
        if not starting_pose:
            starting_pose = self.get_wrist_pose(frame_id=
                                                pose_stamped.header.frame_id)
        if not starting_state:
            init_state = pr2_utils.utils.currentRobotState()
        else:
            init_state = pr2_utils.utils.copyRobotState(starting_state)
        init_state.multi_dof_joint_state =\
            arm_navigation_msgs.msg.MultiDOFJointState()
        init_state.multi_dof_joint_state.frame_ids.append\
            (starting_pose.header.frame_id)
        init_state.multi_dof_joint_state.child_frame_ids.append(link_name)
        init_state.multi_dof_joint_state.poses.append(starting_pose.pose)
        init_state.multi_dof_joint_state.stamp = rospy.Time(0)
        goal.start_state = init_state
        accept_short_path = False
        if min_dist != None and starting_pose.header.frame_id ==\
                pose_stamped.header.frame_id:
            accept_short_path = True
            spos = starting_pose.pose.position
            epos = pose_stamped.pose.position
            desired_length = math.sqrt((spos.x - epos.x)*
                                       (spos.x - epos.x)+
                                       (spos.y - epos.y)*
                                       (spos.y - epos.y)+
                                       (spos.z - epos.z)*
                                       (spos.z - epos.z))
            max_step_size = 0.01
            num_steps = math.ceil(desired_length/max_step_size)
            actual_step_size = desired_length/num_steps
        else:
            num_steps = 0
        param_req = interpolated_ik_motion_planner.srv.\
            SetInterpolatedIKMotionPlanParamsRequest()
        param_req.num_steps = num_steps
        param_req.consistent_angle = INT_IK_CONSISTENT_ANGLE
        param_req.steps_before_abort = 0
        param_req.pos_spacing = INT_IK_POS_SPACING
        param_req.rot_spacing = INT_IK_ROT_SPACING
        param_req.collision_check_resolution =\
            INT_IK_COLLISION_CHECK_RESOLUTION
        param_req.collision_aware = True
        param_req.start_from_end = False
        self.interpolated_ik_parameter_service(param_req)
        if len(ordered_colls.collision_operations) > 0:
            changed_planning_scene = True
            self.set_ordered_collisions(ordered_colls)
        rospy.logdebug('Calling interpolated ik motion planning service with goal%s', str(goal))
        ik_resp = self.interpolated_ik_planning_service(goal)
        ik_traj = trajectory_msgs.msg.JointTrajectory()
        ik_traj.joint_names =\
            list(ik_resp.trajectory.joint_trajectory.joint_names)
        for i in range(len(ik_resp.trajectory_error_codes)):
            c = ik_resp.trajectory_error_codes[i]
            if c.val != c.SUCCESS:
                if accept_short_path and i*actual_step_size >= min_dist:
                    rospy.logwarn('Accepting interpolated ik path of only length %f rather than length %f because of error %d', i*actual_step_size, 
                                  desired_length, c.val)
                    break
                else:
                    rospy.logerr\
                        ('Interpolated IK failed with error %d on step %d',\
                             c.val, i)
                    if changed_planning_scene and revert_scene:
                        self.revert_planning_scene()
                    return (None, c)
            else:
                ik_traj.points.append\
                    (ik_resp.trajectory.joint_trajectory.points[i])
        traj = self.filter_trajectory(goal, ik_resp.trajectory.joint_trajectory)
        if changed_planning_scene and revert_scene:
            self.revert_planning_scene()
        return (traj, None)

    def move_arm_interpolated_ik\
            (self, pose_stamped, revert_scene=True,\
                 ordered_colls=\
                 arm_navigation_msgs.msg.OrderedCollisionOperations(),\
                 min_dist=None, bounds=(0.01, 0.01, 0.01, 0.1, 0.1, 0.1)):
        self.move_into_joint_limits()
        (ik_traj, error_code) =\
            self.plan_interpolated_ik(pose_stamped, revert_scene=revert_scene,\
                                          ordered_colls=ordered_colls,\
                                          min_dist=min_dist, bounds=bounds)
        if not ik_traj: 
            return error_code
        error_code = arm_navigation_msgs.msg.ArmNavigationErrorCodes()
        error_code.val = error_code.SUCCESS
        (jt_r, jt_s) = self.execute_joint_trajectory(ik_traj)
        if jt_s != actionlib_msgs.msg.GoalStatus.SUCCEEDED:
            error_code.val = error_code.INVALID_TRAJECTORY
        return error_code

    def set_ordered_collisions(self, ordered_colls):
        diff = arm_navigation_msgs.srv.GetPlanningSceneRequest()
        sdiff = arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()
        sdiff.planning_scene_diff =\
            self.get_planning_scene_service(diff).planning_scene
        sdiff.operations = ordered_colls
        self.set_planning_scene_service(sdiff)

    def revert_planning_scene(self):
        rospy.logwarn('Reverting planning scene!')
        diff = arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()
        self.set_planning_scene_service(diff)

    def add_trajectory_point_to_force_control(self, x, y, z, ox, oy, oz, ow, 
                                              fx, fy, fz, tx, ty, tz, 
                                              isfx, isfy, isfz, istx, 
                                              isty, istz, time, frame_id=''):
        """
        Adds a new point to the current trajectory. See the relevant method in 
        ee_cart_imped_client for in depth documentation. The only difference is
        that the time for this is that time this particular action should take,
        as opposed to the time after the trajectory starts that the arm should 
        be in place.
        """
        self.force_control.addTrajectoryPoint\
            (x, y, z, ox, oy, oz, ow, fx, fy, fz, tx, ty, tz,\
                 isfx, isfy, isfz, istx, isty, istz,\
                 self.force_control.trajectoryTime().to_sec()+time,\
                 frame_id)
    def get_hand_pose(self, frame_id='/torso_lift_link'):
        """
        @returns: The current pose of the left gripper in the 
        torso_lift_link frame.
        """
        (trans, rot) = pr2_utils.conversions.get_transform\
            (frame_id, '/'+self.arm_name[0]+ '_gripper_tool_frame',\
                 self.tf_listener)
        return pr2_utils.conversions.TRtoPoseStamped\
            (trans, rot, frame_id)

    def get_wrist_pose(self, frame_id='/torso_lift_link'):
        (trans, rot) = pr2_utils.conversions.get_transform\
            (frame_id, '/'+self.arm_name[0]+'_wrist_roll_link',\
                 self.tf_listener)
        return pr2_utils.conversions.TRtoPoseStamped\
            (trans, rot, frame_id)


    def stop_in_place(self):
        """
        Cancels the current arm movement and commands it to stay in 
        place with maximum stiffness.
        """
        print 'STOPPING IN PLACE!'
        self.cancel_all_goals()
        #self.switchToCartesian()
        if self.curr_controller == 'force':
            hold_pose = self.get_hand_pose(frame_id='/torso_lift_link')
            print 'hold pose is', hold_pose
            self.move_arm_force_control(hold_pose, 0.0)

    def executeForceControl(self, reset=True, wait=True):
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
        self.move_arm_client.cancel_goal()

    def cancel_all_goals(self):
        self.force_control.cancelAllGoals()
        self.move_arm_client.cancel_all_goals()

    def resetGoal(self):
        self.force_control.resetGoal()

    def switchToCartesian(self):
        if self.curr_controller == 'cartesian': return
        rospy.logdebug('Switching to cartesian control on arm %s', 
                       self.arm_name)
        ee_cart_imped_control.control_switcher.PR2CMClient.load_cartesian\
            (self.arm_name)
        self.curr_controller = 'cartesian'    

    def switchToForce(self):
        if self.curr_controller == 'force': return
        rospy.logdebug('Switching to force control on arm %s', self.arm_name)
        ee_cart_imped_control.control_switcher.PR2CMClient.load_ee_cart_imped\
            (self.arm_name)
        self.curr_controller = 'force'

def move_arms_to_side():
    cr = ArmControl('right_arm')
    cr.move_arm_to_side()
    cr = ArmControl('left_arm')
    cr.move_arm_to_side()


def test_move_arm():
    c = ArmControl('left_arm')
    c.move_arm_to_side()
    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.pose.position.x = 0.52
    pose_stamped.pose.position.y = 0.2
    pose_stamped.pose.position.z = 0.97
    pose_stamped.pose.orientation.x = 0.0
    pose_stamped.pose.orientation.y = 0.0
    pose_stamped.pose.orientation.z = 0.0
    pose_stamped.pose.orientation.w = 1.0

    # pose_stamped.pose.position.x = 0.37
    # pose_stamped.pose.position.y = -0.28
    # pose_stamped.pose.position.z = 0.87
    # pose_stamped.pose.orientation.x = 0.839
    # pose_stamped.pose.orientation.y = 0.441
    # pose_stamped.pose.orientation.z = -0.278
    # pose_stamped.pose.orientation.w = 0.155
    pose_stamped.header.frame_id = '/base_link'
    #c.move_arm_to_side()
    res = c.move_arm_collision_free(pose_stamped)

def test_trying_hard():
    c = ArmControl('right_arm')
    side_pose = geometry_msgs.msg.PoseStamped()
    side_pose.pose.position.x = 0
    side_pose.pose.position.y = -0.6
    side_pose.pose.position.z = 0
    side_pose.pose.orientation.x = 0.5
    side_pose.pose.orientation.y = 0.5
    side_pose.pose.orientation.z = 0.5
    side_pose.pose.orientation.w = 0.5
    side_pose.header.frame_id = '/torso_lift_link'
    res = c.try_hard_for_pose(side_pose)
    print 'Resulting of trying ahrd was', res

def test_move_arm_to_side():
    cr = ArmControl('right_arm')
    cr.move_arm_to_side()

def test_planning_scene():
    diff = arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()
    op = arm_navigation_msgs.msg.CollisionOperation()
    op.object1 = op.COLLISION_SET_ALL
    op.object2 = op.COLLISION_SET_ALL
    op.operation = op.ENABLE
    ops = arm_navigation_msgs.msg.OrderedCollisionOperations()
    ops.collision_operations.append(op)
    diff.operations = ops
    gclient = rospy.ServiceProxy('/environment_server/get_planning_scene',
                                 arm_navigation_msgs.srv.GetPlanningScene)
    print 'waiting for get planning scene'
    gclient.wait_for_service()
    g_res = gclient()
    print 'get planning scene returns', g_res.planning_scene.allowed_collision_matrix
    # client = rospy.ServiceProxy('/environment_server/set_planning_scene_diff',
    #                             arm_navigation_msgs.srv.SetPlanningSceneDiff)
    # print 'Waiting for set planning scene'
    # client.wait_for_service()
    # diff_res = client(diff)
    # print 'diff_res =', diff_res.planning_scene.allowed_collision_matrix
    # g_res = gclient()
    # print 'get planning scene returns', g_res.planning_scene.allowed_collision_matrix
    # diff.operations = arm_navigation_msgs.msg.OrderedCollisionOperations()
    # diff_res = client(diff)
    # print 'set planning scene on nothing returns', diff_res.planning_scene.allowed_collision_matrix

def test_interpolated_ik():
    c = ArmControl('right_arm')
    pose_stamped = c.get_wrist_pose()
    pose_stamped.pose.position.z += 0.15
    op = arm_navigation_msgs.msg.CollisionOperation()
    op.object1 = 'r_end_effector'
    op.object2 = op.COLLISION_SET_ALL
    op.operation = op.DISABLE
    ops = arm_navigation_msgs.msg.OrderedCollisionOperations()
    ops.collision_operations.append(op)
    e = c.move_arm_interpolated_ik(pose_stamped,
                               ordered_colls=ops)
    if e:
        print 'error code was', e
    else:
        print 'Successfully found trajectory'
    

def test_trajectory_filtering():
    c = ArmControl('right_arm')
    pose_stamped = c.get_wrist_pose()
    pose_stamped.pose.position.y -= 0.1
    error = c.move_arm_interpolated_ik(pose_stamped)
    print 'Moving returned', error

def test_controller_switching():
    c = ArmControl('right_arm')
    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.header.frame_id = '/base_link'
    pose_stamped.pose.position.x = 0.6
    pose_stamped.pose.position.z = 1.2
    pose_stamped.pose.orientation.w = 1.0
    c.move_arm_collision_free(pose_stamped)
    pose_stamped.pose.position.z = 0.5
    c.move_arm_force_control(pose_stamped, 2.0)
    c.move_arm_to_side()
    cl = ArmControl('left_arm', manager=c.manager)
    pose_stamped.pose.position.z = 0.7
    cl.move_arm_collision_free(pose_stamped)
    pose_stamped.pose.position.z = 1.2
    cl.move_arm_force_control(pose_stamped, 2.0)
    cl.move_arm_to_side()

def test_quaternions():
    #move_arms_to_side()
    c = ArmControl('right_arm')
    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.header.frame_id = '/torso_lift_link'
    pose_stamped.pose.position.x = 0.6
    pose_stamped.pose.position.y = 0.01
    pose_stamped.pose.position.z = 0.008825
    pose_stamped.pose.orientation.x = 0.5
    pose_stamped.pose.orientation.y = 0.5 
    pose_stamped.pose.orientation.z = -0.5
    pose_stamped.pose.orientation.w = 0.5
    c.move_arm_collision_free(pose_stamped)
    print 'This is angle 0'
    raw_input()
    q1 = geometry_msgs.msg.Quaternion()
    q2 = geometry_msgs.msg.Quaternion()
    q1.y = 0.707
    q1.w = 0.707
    q2.z = 0.707
    q2.w = 0.707
    pose_stamped.pose.orientation = q1
    c.move_arm_collision_free(pose_stamped)
    print 'This is 90 degrees around y'
    raw_input()
    pose_stamped.pose.orientation = q2
    c.move_arm_collision_free(pose_stamped)
    print 'This is 90 degrees around z'
    raw_input()
    pose_stamped.pose.orientation = pr2_utils.utils.multiply_quaternions(q1, q2)
    c.move_arm_collision_free(pose_stamped)
    print 'This is 90 degrees around z followed by 90 degrees around y'
    raw_input()
    pose_stamped.pose.orientation = pr2_utils.utils.multiply_quaternions(q2, q1)
    c.move_arm_collision_free(pose_stamped)
    print 'This is 90 degrees around y followed by 90 degrees around z'
    raw_input()


def main():
    test_quaternions()
    
if __name__ == "__main__":
    rospy.init_node('arm_control_test_node')
    main()
