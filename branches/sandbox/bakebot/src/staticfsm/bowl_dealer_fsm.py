##!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
import rospy
import tf
import actionlib
from clients.arm_client import * 
from clients.base_client import BaseClient
from utilities.bakebot_pick_and_place_manager import *
from utilities.broad_table_object_manager import *
from utilities.bowl_dealer_utilities import *
from utilities.joint_recorder_player import *
import math
import smach
import smach_ros
import time 

class InitializeState(smach.State):
    def __init__(self, debug = False):
        smach.State.__init__(self, outcomes=['done', 'skip to success'],
                             output_keys =['move_over_positions_exhausted', 'use_new_pose_manager']) #TODO put the keys here
        self.debug = debug

    def execute(self, userdata):
        rospy.loginfo('Executing state InitializeState')
        mlogger  = EventLoggerClient('milestone', annotation='grab and postgrab lift & rotate')
        StaticLoggerLocker.check_in_logger('GRAB LOGGER', mlogger) 
        logger = EventLoggerClient.startfsm('Initialize state (deal bowl fsm)')
        userdata.move_over_positions_exhausted = False
        userdata.use_new_pose_manager = True
        logger.stops()
        if self.debug:
            choice = raw_input('press any key to skip to success, or enter to continue...')
            if len(choice) == 0:
                return 'done'
            else:
                return 'skip to success'
        else:
            return 'done'

class RefineMixingBowlPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'failed'],
                                   input_keys=['mixing_bowl',
                                               'table_frame_x_offset',
                                               'table_frame_y_offset'],
                                   output_keys=['mixing_bowl'])

    def execute(self, userdata):
        logger = EventLoggerClient.startfsm('RefineMixingBowlPosition (mixing fsm)')
        rospy.loginfo('Executing state RefineMixingBowlPosition')
        x = userdata.mixing_bowl.pose.pose.position.x
        y = userdata.mixing_bowl.pose.pose.position.y
        z = userdata.mixing_bowl.pose.pose.position.z
        rospy.loginfo('\n\n\noriginal pose of mixing bowl: ' + str((x,y,z)) + '\n\n\n')
        rospy.loginfo('replacing the pose of userdata.mixing_bowl with the pose from the most recent detection')
        btom = BroadTabletopObjectManager.get_broad_tabletop_object_manager(PickAndPlaceManager.get_pick_and_place_manager())
        detected_mixing_bowl = btom.refine_and_and_get_graspable_object(userdata.mixing_bowl.collision_name, userdata.table_frame_x_offset, userdata.table_frame_y_offset)
        if detected_mixing_bowl == None:
            rospy.logerr('could not find the mixing bowl!')
            logger.stopf()
            return 'failed'
        userdata.mixing_bowl.pose = detected_mixing_bowl.pose
        x = userdata.mixing_bowl.pose.pose.position.x
        y = userdata.mixing_bowl.pose.pose.position.y
        z = userdata.mixing_bowl.pose.pose.position.z
        rospy.loginfo('\n\n\ncorrected pose of mixing bowl: ' + str((x,y,z)) + '\n\n\n')
        logger.stops()
        return 'done'


# the robot will always grab with the left arm, and mix with the right arm
class Grab(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail', 'fail with exhaustion'], 
                             input_keys=['bowl_to_deal', 'arms_to_try', 'is_right_arm', 'table_frame_x_offset', 'table_frame_y_offset', 'move_over_positions_exhausted'],
                             output_keys=['is_right_arm', 'grasp_pose', 'global_grasp_pose', 'delta_x', 'delta_y','bowl_radius'])
        self.pick_and_place_manager = PickAndPlaceManager.get_pick_and_place_manager()
        self.arm_client = ArmClient.get_arm_client()
        self.broad_object_manager = BroadTabletopObjectManager.get_broad_tabletop_object_manager(self.pick_and_place_manager)

    def execute(self, userdata):
        rospy.loginfo('Executing state Grab')
        print 'I want to grab the bowl at: ', userdata.bowl_to_deal.pose.pose.position.x, userdata.bowl_to_deal.pose.pose.position.y
        print 'x off: ', userdata.table_frame_x_offset
        print 'y off: ', userdata.table_frame_y_offset
        logger = EventLoggerClient.startfsm('grab (deal bowl fsm)')
        whicharm, result, grasp_pose = self.broad_object_manager.refine_and_grasp(userdata.bowl_to_deal.collision_name, userdata.arms_to_try, userdata.table_frame_x_offset, userdata.table_frame_y_offset)
        userdata.is_right_arm = (whicharm == 0)
        userdata.grasp_pose = grasp_pose
        trans, rot = self.arm_client.get_transform(userdata.is_right_arm)
        userdata.global_grasp_pose = (trans, rot)
        print 'global grasp trans: ', trans
        print 'global grasp rot: ', rot

        if result == 'succeeded':
            #x = userdata.bowl_to_deal.box_dims.x
            #y = userdata.bowl_to_deal.box_dims.y
            #z = userdata.bowl_to_deal.box_dims.z
            print '\n\n\n****************************************************************************************\n\n'
            framename = 'r_gripper_tool_frame' if userdata.is_right_arm else 'l_gripper_tool_frame'
            print GripperBowlPoseManager.store_barcode_pos(userdata.bowl_to_deal, framename)
            x = userdata.bowl_to_deal.box_dims[0]
            y = userdata.bowl_to_deal.box_dims[1]
            z = userdata.bowl_to_deal.box_dims[2]
            print x,y,z
            print 'type: ', userdata.bowl_to_deal.type
            r = (math.sqrt(x**2 + y**2) ) / 2.0
            rospy.loginfo('!!!!!!!!!!! RADIUS OF BOWL: ' + str(r))
            userdata.bowl_radius = r;
            print '\n\n\n****************************************************************************************\n\n'
            logger.stops()
            return 'success'
        else:
            rospy.logwarn('grasp client returned: ' + str(result))
            if userdata.move_over_positions_exhausted:
                rospy.logerror('no more move over positions (they are exhausted)')
                return 'fail with exhaustion'
            userdata.delta_y = .1;
            if userdata.bowl_to_deal.pose.pose.position.x < .5:
                print '999999999999999999999999999999999999999999999999999999999'
                print '\n'
                rospy.logwarn('the x position is: ' + str(userdata.bowl_to_deal.pose.pose.position.x))
                userdata.delta_x = .1
            logger.stopf()
            return 'fail'

class ChangeGripperObjHeight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['acceptable_height_above_table', 'table_height', 'is_right_arm', 'table_frame_x_offset', 'table_frame_y_offset','delta_x', 'delta_y'],
                             output_keys=['current_height_above_table', 'delta_y', 'delta_x'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener

    def execute(self, userdata):
        rospy.loginfo('Executing state ChangeGripperObjHeight')
        logger = EventLoggerClient.startfsm('lift (deal bowl fsm)')
        (trans, rot) = self.arm_client.get_transform(userdata.is_right_arm)
        desired_z = userdata.acceptable_height_above_table + userdata.table_height
        print '*************************************************\n\n\n'
        print desired_z
        print userdata.acceptable_height_above_table
        print '*************************************************\n\n\n'
        
        rospy.loginfo('lifting the arm to desired z: ' + str(desired_z))
        status = self.arm_client.move_to_pose_const_ee(userdata.is_right_arm, trans[0], trans[1], desired_z)
        if status == 0 or status == -2:
            # -2 is a timeout
            rospy.loginfo('setting delta_y to offset table_frame_y_offset')
            userdata.delta_y = abs(userdata.table_frame_y_offset)
            userdata.delta_x = abs(userdata.table_frame_x_offset)
            print abs(userdata.delta_y)
            logger.stops()
            tlogger  = EventLoggerClient('milestone', annotation='transit to mix area')
            StaticLoggerLocker.check_in_logger('TRANSIT LOGGER', tlogger) 
            mlogger = StaticLoggerLocker.check_out_logger('GRAB LOGGER')
            if mlogger is not None:
                mlogger.stops()
            return 'success'
        else:
            rospy.loginfo('lift failed once, trying again')
            status = self.arm_client.move_to_pose_const_ee(userdata.is_right_arm, trans[0], trans[1], desired_z)
            if status == 0 or status == -2:
                # -2 is a timeout
                rospy.loginfo('setting delta_y to offset table_frame_y_offset')
                userdata.delta_y = abs(userdata.table_frame_y_offset)
                userdata.delta_x = abs(userdata.table_frame_x_offset)
                print abs(userdata.delta_y)
                logger.stops()
                mlogger = StaticLoggerLocker.check_out_logger('GRAB LOGGER')
                if mlogger is not None:
                    mlogger.stops()
                tlogger  = EventLoggerClient('milestone', annotation='transit to mix area')
                StaticLoggerLocker.check_in_logger('TRANSIT LOGGER', tlogger) 
                return 'success'
            else:
                rospy.logwarn('lift failed again, trying to move forward while lifting')
                status = self.arm_client.move_to_pose_const_ee(userdata.is_right_arm, trans[0] + .2, trans[1], desired_z)
                if status == 0 or status == -2:
                    rospy.loginfo('setting delta_y to offset table_frame_y_offset')
                    userdata.delta_y = abs(userdata.table_frame_y_offset)
                    userdata.delta_x = abs(userdata.table_frame_x_offset)
                    print abs(userdata.delta_y)
                    logger.stops()
                    mlogger = StaticLoggerLocker.check_out_logger('GRAB LOGGER')
                    if mlogger is not None:
                        mlogger.stops()
                    tlogger  = EventLoggerClient('milestone', annotation='transit to mix area')
                    StaticLoggerLocker.check_in_logger('TRANSIT LOGGER', tlogger) 
                    return 'success'
                else:
                    logger.stopf()
                    mlogger = StaticLoggerLocker.check_out_logger('GRAB LOGGER')
                    if mlogger is not None:
                        mlogger.stopf()
                    tlogger  = EventLoggerClient('milestone', annotation='transit to mix area')
                    StaticLoggerLocker.check_in_logger('TRANSIT LOGGER', tlogger) 
                    return 'fail'

class MoveRobotOver(smach.State):
    def __init__(self, moveRight):
        smach.State.__init__(self, outcomes=['success', 'fail'], 
                                   input_keys=['max_left_movement', 'max_right_movement', 'table_frame_x_offset', 'delta_y', 'table_frame_y_offset', 'delta_x'],
                                   output_keys=['move_over_positions_exhausted', 'table_frame_y_offset', 'table_frame_x_offset'])
        self.base_client = BaseClient.get_base_client()
        self.moveRight = moveRight

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveRobotOver')
        logger = EventLoggerClient.startfsm('MoveRobotOver (deal bowl fsm)')
        if self.moveRight:
            rospy.logwarn('moving the robot to the right by delta_y')
            rospy.logwarn('delta y is: ' + str(userdata.delta_y))
            if userdata.table_frame_y_offset > userdata.max_right_movement:
                rospy.logwarn('have already mvoed too far to the right.  cannot move further to the right.')
                userdata.move_over_positions_exhausted = True
                logger.stopf('cannot move further in this direction')
                return 'fail'
            else:
                status = self.base_client.translate(False, -1 * abs(userdata.delta_y))
                if status:
                    userdata.table_frame_y_offset = userdata.table_frame_y_offset + abs(userdata.delta_y)
                    if userdata.delta_x is not 0:
                        rospy.logwarn('driving negative x: ' + str(userdata.delta_x))
                        self.base_client.translate(True, abs(userdata.delta_x))
                        userdata.table_frame_x_offset = userdata.table_frame_x_offset - abs(userdata.delta_x)
                    logger.stops()
                    return 'success'
                else:
                    rospy.logerr('base client translate returned false indicating failure')
                    logger.stopf()
                    return 'fail'
        else:
            rospy.logwarn('moving the robot to the left by delta_y')
            rospy.logwarn('delta y is: ' + str(userdata.delta_y))
            if userdata.table_frame_y_offset < -1 * userdata.max_left_movement:
                rospy.logwarn('have already moved too far to the left.  cannot move further to the left.')
                userdata.move_over_positions_exhausted = True
                logger.stopf('cannot move further in this direction')
                return 'fail'
            else:
                status = self.base_client.translate(False, userdata.delta_y)
                if status:
                    userdata.table_frame_y_offset = userdata.table_frame_y_offset - abs(userdata.delta_y)
                    if userdata.delta_x is not 0:
                        rospy.logwarn('driving negative x: ' + str(userdata.delta_x))
                        self.base_client.translate(True, -1 * abs(userdata.delta_x))
                        userdata.table_frame_x_offset = userdata.table_frame_x_offset + abs(userdata.delta_x)
                    logger.stops()
                    return 'success'
                else:
                    rospy.logerr('base client translate returned false indicating failure')
                    logger.stopf()
                    return 'fail'



class RotateToKnownPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail', 'fail with exhaustion'],
                             input_keys=['is_right_arm', 'grasp_pose', 'bowl_to_deal', 'use_new_pose_manager'],
                             output_keys=['current_known_pose_name', 'hand_poses_exhausted', 'use_new_pose_manager'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener
        self.pose_manager = None

    def execute(self, userdata):
        rospy.loginfo('Executing state RotateToKnownPose')
        logger = EventLoggerClient.startfsm('RotateToKnownPose (deal bowl fsm)')
        if self.pose_manager == None or userdata.use_new_pose_manager:
            print self.pose_manager
            print userdata.use_new_pose_manager
            userdata.use_new_pose_manager = False
            rospy.loginfo('instantiating a new pose manager *****************(7777777777777777777777777777777777777777777777777')
            self.pose_manager = GripperBowlPoseManager(userdata.bowl_to_deal, self.arm_client, userdata.is_right_arm)
            userdata.hand_poses_exhausted = False
            pose, name_of_pose = self.pose_manager.get_current_recommended_pose()
        else:
            rospy.loginfo('getting the next recommended pose')
            pose, name_of_pose = self.pose_manager.get_next_recommended_pose()

        rospy.loginfo('rotating end effector to known pose: ' + name_of_pose + ' ' + str(pose))
        status = self.arm_client.rotate_end_effector(userdata.is_right_arm, pose[0], pose[1], pose[2], pose[3])
        if status == 0:
            rospy.loginfo('success')
            userdata.current_known_pose_name = name_of_pose
            if self.pose_manager.is_exhausted:
                rospy.logwarn('succeeded in rotating but am now exhausted, no more options after this!')
                userdata.hand_poses_exhausted = True
                self.pose_manager = None
            logger.stops()
            return 'success'
        else:
            rospy.loginfo('trying one more time')
            status = self.arm_client.rotate_end_effector(userdata.is_right_arm, pose[0], pose[1], pose[2], pose[3])
            if status == 0:
                rospy.loginfo('success')
                userdata.current_known_pose_name = name_of_pose
                if self.pose_manager.is_exhausted:
                    rospy.logwarn('succeeded in rotating but am now exhausted, no more options after this!')
                    userdata.hand_poses_exhausted = True
                    self.pose_manager = None
                logger.stops()
                return 'success'
            rospy.loginfo('still failed!!!!!!!!!!!!!!!')
            if self.pose_manager.is_exhausted:
                rospy.logerr('pose manager is exhausted and rotation to original pose failed.')
                rospy.logerr('cannot recover from this failure, failing with exhaustion')
                self.pose_manager = None
                logger.stopf('fail with exhaustion')
                return 'fail with exhaustion'
            rospy.logwarn('rotating the end effector failed.  going to next preferred pose by recursing via FSM')
            userdata.current_known_pose_name = 'unknown'
            logger.stopf()
            return 'fail'

class TransitBowlToMixingArea(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail', 'fail with exhaustion'],
                             input_keys=['is_right_arm', 'hand_poses_exhausted', 'mixing_bowl', 'mixing_area_pose_x_tol', 'mixing_area_pose_y_tol', 'mixing_area_pose_z_tol', 'acceptable_height_above_table', 'use_new_pose_manager'],
                             output_keys=['hand_poses_exhausted', 'use_new_pose_manager'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener
        self.pick_and_place_manager = PickAndPlaceManager.get_pick_and_place_manager()

    def execute(self, userdata):
        rospy.loginfo('executing state TransitBowlToMixingArea')
        logger = EventLoggerClient.startfsm('TransitBowlToMixingArea (deal bowl fsm)')
        mixing_bowl_pose = userdata.mixing_bowl.pose.pose.position
        rospy.loginfo('setting to use a new pose manager')
        x_pos = mixing_bowl_pose.x
        y_pos = mixing_bowl_pose.y
        z_pos = mixing_bowl_pose.z
        mixing_area_pose = (x_pos, y_pos, z_pos + userdata.acceptable_height_above_table)
        x_pos, y_pos, z_pos = mixing_area_pose
        x_plus_minus = userdata.mixing_area_pose_x_tol
        y_plus_minus = userdata.mixing_area_pose_y_tol
        z_plus_minus = userdata.mixing_area_pose_z_tol
        rospy.loginfo('Executing state TransitBowlToMixingArea')
        #status = self.arm_client.move_to_pose_const_ee(userdata.is_right_arm, x_pos, y_pos, z_pos, 1, x_plus_minus, y_plus_minus, z_plus_minus, 1)
        pose_to_point_at = [x_pos, y_pos, 0.65]
        self.pick_and_place_manager.point_head(pose_to_point_at, 'base_link')
        status = self.arm_client.move_to_pose_const_ee(userdata.is_right_arm, x_pos, y_pos, z_pos, 0, x_plus_minus, y_plus_minus, z_plus_minus, 1)
        if status == 0:
            rospy.loginfo('succeeded')
            userdata.use_new_pose_manager = True
            logger.stops()
            return 'success'
        else:
            rospy.logwarn('failed to plan transit')
            if userdata.hand_poses_exhausted:
                rospy.logerr('there are no more positions that I can rotate to to replan from')
                rospy.logerr('I cannot recover from this failure, failing with exhaustion')
                logger.stopf('fail with exhaustion')
                return 'fail with exhaustion'
            else:
                rospy.logwarn('failed to plan transit from this rotation.  going to re-rotate and try again')
                logger.stopf()
                return 'fail'

class MoveToPrePourPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail', 'fail with exhaustion', 'unknown pre pour pose'],
                             input_keys=['is_right_arm', 'hand_poses_exhausted', 'mixing_bowl', 'pour_height_above_centroid', 'current_known_pose_name', 'bowl_radius'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener
        self.has_been_called_once = False

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveToPrePourPosition')
        logger = EventLoggerClient.startfsm('MoveToPrePourPosition (deal bowl fsm)')
        if not self.has_been_called_once:
            rospy.loginfo('setting hand poses exhausted flag to false, for the first execution only')
            userdata.hand_poses_exhausted = False
            self.has_been_called_once = True

        mixing_bowl_pose = userdata.mixing_bowl.pose.pose.position
        x_pos = mixing_bowl_pose.x
        y_pos = mixing_bowl_pose.y
        z_pos = mixing_bowl_pose.z
        z_pos = z_pos + userdata.pour_height_above_centroid
        # need to figure out where to move to for the pre pose based on the current pose
        if userdata.current_known_pose_name == 'unknown':
            rospy.logwarn('unknown current pose, need to rotate to a known position to calculate pre pour position')
            logger.stopf('unknown current pose')
            return 'unknown pre pour pose'
        else:
            try:
                dx, dy, dz = GripperBowlPoseManager.POUR_ADJUSTMENT_DIRS_FROM_HIGH_CENTROID[userdata.current_known_pose_name]
            except KeyError:
                rospy.logwarn('could not find a pour adjustment for ' + str(userdata.current_known_pose_name))
                rospy.logwarn('going to try to rotate to another pose')
                if userdata.hand_poses_exhausted:
                    rospy.logerr('there are no more positions that I can rotate to to replan from')
                    rospy.logerr('I cannot recover fromt hsi failure, failing with exhastion')
                    logger.stopf('fail with exhastion')
                    return 'fail with exhaustion'
                else:
                    logger.stopf()
                    return 'fail'
            rospy.loginfo('POUR INFO ********************************************************')
            print 'bowl radius: ', userdata.bowl_radius
            print 'deltas: ', dx, dy, dz
            print 'mbx_pos', x_pos
            print 'mby_pos', y_pos
            print 'mbz_pos', z_pos
            x_pos = x_pos + dx * userdata.bowl_radius*.75 # used to be /2.0
            y_pos = y_pos + dy * userdata.bowl_radius*.75
            z_pos = z_pos + dz * userdata.bowl_radius*.75
            print x_pos
            print y_pos
            print z_pos
            x_plus_minus = (0.05, 0.05)
            y_plus_minus = (0.05, 0.05)
            z_plus_minus = (0.1, 0.0)
            status = self.arm_client.move_to_pose_const_ee(userdata.is_right_arm, x_pos, y_pos, z_pos, 1, x_plus_minus, y_plus_minus, z_plus_minus, 1)
            if status == 0:
                rospy.loginfo('succeeded')
                logger.stops()
                return 'success'
            else:
                rospy.logwarn('failed to plan move to pre pour position')
                if userdata.hand_poses_exhausted:
                    rospy.logerr('there are no more positions that I can rotate to to replan from')
                    rospy.logerr('I cannot recover from this failure, failing with exhaustion')
                    logger.stopf('fail with exhaustion')
                    return 'fail with exhaustion'
                else:
                    rospy.logwarn('failed to plan transit from this rotation.  going to re-rotate and try again')
                    logger.stopf()
                    return 'fail'

class Pour(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail' ],
                             input_keys = ['is_right_arm', 'current_known_pose_name'],
                             output_keys = ['use_new_pose_manager'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener

    def execute(self, userdata):
        rospy.loginfo('Executing state Pour')
        logger = EventLoggerClient.startfsm('Pour (deal bowl fsm)')
        #userdata.use_new_pose_manager = True
        if userdata.current_known_pose_name == 'unknown':
            rospy.logwarn('unknown current pose, need to rotate to a known position before pouring')
            logger.stopf('unknown current pose')
            return 'fail'
        px, py, pz, pw = GripperBowlPoseManager.POUR_POSES[userdata.current_known_pose_name]
        dx, dy, dz = GripperBowlPoseManager.POUR_ADJUSTMENT_DIRS_FROM_HIGH_CENTROID[userdata.current_known_pose_name]
        (trans, rot) = self.arm_client.get_transform(userdata.is_right_arm)
        (x, y, z) = trans
        delta = 0.11 
        x = x + delta * dx
        y = y + delta * dy
        z = z + delta * dz
        status = self.arm_client.rotate_end_effector(userdata.is_right_arm, px, py, pz, pw, determination=0)
        status = self.arm_client.move_to_pose(userdata.is_right_arm, True, x, y, z, px, py, pz, pw)
        if status == 0:
            rospy.loginfo('pour succeeded')
            logger.stops()
            mlogger = StaticLoggerLocker.check_out_logger('TRANSIT LOGGER')
            if mlogger is not None:
                mlogger.stops()
            tlogger  = EventLoggerClient('milestone', annotation='successful pour to revert')
            StaticLoggerLocker.check_in_logger('POUR LOGGER', tlogger) 
            return 'success'
        else:
            rospy.logwarn('pour failed to rotate to position')
            logger.stopf()
            return 'fail'

class SteepPour(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail' ],
                             input_keys = ['is_right_arm', 'current_known_pose_name'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener

    def execute(self, userdata):
        rospy.loginfo('Executing state SteepPour')
        logger = EventLoggerClient.startfsm('SteepPour (deal bowl fsm)')
        if userdata.current_known_pose_name == 'unknown':
            rospy.logwarn('unknown current pose, need to rotate to a known position before pouring')
            logger.stopf('unkown pos')
            return 'fail'
        px, py, pz, pw = GripperBowlPoseManager.STEEP_POUR_POSES[userdata.current_known_pose_name]
        dx, dy, dz = GripperBowlPoseManager.POUR_ADJUSTMENT_DIRS_FROM_HIGH_CENTROID[userdata.current_known_pose_name]
        (trans, rot) = self.arm_client.get_transform(userdata.is_right_arm)
        (x, y, z) = trans
        #delta = 0.10
        delta = .04
        x = x + delta * dx
        y = y + delta * dy
        z = z + delta * dz
        print 'steep pour info..................'
        print 'delta: ', delta, 'dx,dy,dz:', dx, dy, dz
        print 'px, py, pz:', px, py, pz
        #status = self.arm_client.rotate_end_effector(userdata.is_right_arm, px, py, pz, pw, determination=0)
        status = self.arm_client.move_to_pose(userdata.is_right_arm, True, x, y, z, px, py, pz, pw)
        if status == 0:
            rospy.loginfo('steep pour succeeded')
            logger.stops()
            return 'success'
        else:
            rospy.logwarn('||||||||||||||||   steep pour failed to rotate to position')
            #status = self.arm_client.rotate_end_effector(userdata.is_right_arm, px, py, pz, pw, determination=0, use_interpolation = False)
            status = self.arm_client.move_to_pose(userdata.is_right_arm, False, x, y, z, px, py, pz, pw)
            if status == 0:
                rospy.loginfo('steep pour succeeded, but only after turning interpolation off')
                logger.stops('had to turn of interpolation')
                return 'success'
            else:
                rospy.logwarn('||||||||||||||||   steep pour failed to rotate to position, trying again with determination!')
                status = self.arm_client.move_to_pose_const_ee(userdata.is_right_arm, x, y, z, determination = 2)
                rospy.logwarn('status after shift: ' + str(status))
                status = self.arm_client.rotate_end_effector(userdata.is_right_arm, px, py, pz, pw, determination=2)
                if status == 0:
                    rospy.loginfo('steep pour succeeded, but only after trying with determination')
                    logger.stops('had to turn of interpolation and with determination')
                    return 'success'
                else:
                    rospy.logwarn('||||||||||||||||   steep pour failed to rotate to position FAIL FAIL FAIL!')
                    logger.stopf()
                    return 'fail'

class SuperSteepPour(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail' ],
                             input_keys = ['is_right_arm', 'current_known_pose_name'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener

    def execute(self, userdata):
        rospy.loginfo('Executing state SuperSteepPour')
        logger = EventLoggerClient.startfsm('SuperSteepPour (deal bowl fsm)')
        if userdata.current_known_pose_name == 'unknown':
            rospy.logwarn('unknown current pose, need to rotate to a known position before pouring')
            logger.stopf('unkown pos')
            return 'fail'
        px, py, pz, pw = GripperBowlPoseManager.SUPER_STEEP_POUR_POSES[userdata.current_known_pose_name]
        dx, dy, dz = GripperBowlPoseManager.POUR_ADJUSTMENT_DIRS_FROM_HIGH_CENTROID[userdata.current_known_pose_name]
        (trans, rot) = self.arm_client.get_transform(userdata.is_right_arm)
        (x, y, z) = trans
        delta = 0.0
        x = x + delta * dx
        y = y + delta * dy
        z = z + delta * dz
        print 'super steep pour info..................'
        print 'delta: ', delta, 'dx,dy,dz:', dx, dy, dz
        print 'px, py, pz:', px, py, pz
        #status = self.arm_client.rotate_end_effector(userdata.is_right_arm, px, py, pz, pw, determination=0)
        status = self.arm_client.move_to_pose(userdata.is_right_arm, True, x, y, z, px, py, pz, pw)
        if status == 0:
            rospy.loginfo('super steep pour succeeded')
            logger.stops()
            return 'success'
        else:
            rospy.logwarn('||||||||||||||||   super steep pour failed to rotate to position')
            #status = self.arm_client.rotate_end_effector(userdata.is_right_arm, px, py, pz, pw, determination=0, use_interpolation = False)
            status = self.arm_client.move_to_pose(userdata.is_right_arm, False, x, y, z, px, py, pz, pw)
            if status == 0:
                rospy.loginfo('super steep pour succeeded, but only after turning interpolation off')
                logger.stops('had to turn of interpolation')
                return 'success'
            else:
                rospy.logwarn('||||||||||||||||   super steep pour failed to rotate to position, trying again with determination!')
                status = self.arm_client.move_to_pose_const_ee(userdata.is_right_arm, x, y, z, determination = 2)
                rospy.logwarn('status after shift: ' + str(status))
                status = self.arm_client.rotate_end_effector(userdata.is_right_arm, px, py, pz, pw, determination=2)
                if status == 0:
                    rospy.loginfo('super steep pour succeeded, but only after trying with determination')
                    logger.stops('had to turn of interpolation and with determination')
                    return 'success'
                else:
                    rospy.logwarn('||||||||||||||||   steep pour failed to rotate to position FAIL FAIL FAIL!')
                    logger.stopf()
                    return 'fail'

class Shake(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'], 
                             input_keys = ['is_right_arm'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener
        self.papm = PickAndPlaceManager.get_pick_and_place_manager()

    def execute(self, userdata):
        rospy.loginfo('Executing state Shake')
        logger = EventLoggerClient.startfsm('Shake (deal bowl fsm)')
        whicharm = 0 if userdata.is_right_arm else 1
        bcm = self.arm_client.cms[whicharm]
        positions = bcm.get_current_arm_angles()
        newpositions_plus = list()
        newpositions_minus = list()
        for i, position in enumerate(positions):
            if i < 4:
                fluff = 0.04
            elif i == 5:
                fluff = 0.08
            else:
                fluff = 0.04
            rospy.loginfo('i is ' + str(i) + ' and adding fluff of ' + str(fluff))
            newpositions_plus.append(position + fluff)
            newpositions_minus.append(position - fluff)

        for i in range(0,3):
            rospy.loginfo('sending one shake pulse')
            self.papm.try_hard_to_move_joint(whicharm, [newpositions_minus, newpositions_plus])
            time.sleep(0.5)

        bcm.move_arm_joint(positions)
        logger.stops()
        return 'done'


class Revert(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys = ['is_right_arm', 'current_known_pose_name'],
                             output_keys = ['do_sloppy_drop'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener

    def execute(self, userdata):
        isRightArm = userdata.is_right_arm
        rospy.loginfo('Executing state Revert')
        logger = EventLoggerClient.startfsm('Revert (deal bowl fsm)')
        mlogger = StaticLoggerLocker.check_out_logger('POUR LOGGER')
        if userdata.current_known_pose_name == 'unknown':
            rospy.logwarn('unknown current pose name, cannot rotate back to it')
            userdata.do_sloppy_drop = True
            logger.stopf()
            if mlogger is not None:
                mlogger.stopf()
            return 'fail'
        px, py, pz, pw = GripperBowlPoseManager.ALL_POSES[userdata.current_known_pose_name]
        (trans, rot) = self.arm_client.get_transform(isRightArm)
        #status = self.arm_client.rotate_end_effector(userdata.is_right_arm, px, py, pz, pw, determination=0)
        status = self.arm_client.move_to_pose(isRightArm, False, trans[0], trans[1], trans[2], px, py, pz, pw)
        if status == 0:
            rospy.loginfo('revert succeeded')
            userdata.do_sloppy_drop = False
            logger.stops()
            if mlogger is not None:
                mlogger.stops()
            tlogger  = EventLoggerClient('milestone', annotation='return transit and place/dump')
            StaticLoggerLocker.check_in_logger('RT LOGGER', tlogger) 
            return 'success'
        else:
            rospy.logwarn('pour failed to revert to position')
            userdata.do_sloppy_drop = True
            logger.stopf()
            if mlogger is not None:
                mlogger.stopf()
            tlogger  = EventLoggerClient('milestone', annotation='return transit and place/dump')
            StaticLoggerLocker.check_in_logger('RT LOGGER', tlogger) 
            return 'fail'
        
class ReturnTransitBowl(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys = ['is_right_arm', 'bowl_to_deal', 'table_height', 'acceptable_height_above_table'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener

    def execute(self, userdata):
        rospy.loginfo('Executing state ReturnTransitBowl')
        logger = EventLoggerClient.startfsm('ReturnTransitBowl (deal bowl fsm)')
        trans, rot = self.arm_client.get_transform(userdata.is_right_arm)
        pose = userdata.bowl_to_deal.pose.pose
        #trans_orig = (pose.position.x, pose.position.y, pose.position.z)
        trans_orig = (.1, .6, pose.position.z)
        rot_orig = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        xo, yo, zo = trans_orig
        zo = userdata.table_height + userdata.acceptable_height_above_table
        xrang = (xo - .1, xo + .1)
        yrang = (yo - .1, yo + .1)
        zrang = (zo, zo)
        #quater_xyzw = rot_orig
        quater_xyzw = rot
        #status = self.arm_client.try_hard_to_move_to_pose(userdata.is_right_arm, False, xrang, yrang, zrang, quater_xyzw, False, 1)
        dz = .25
        status = self.arm_client.move_to_pose(userdata.is_right_arm, False, trans[0], trans[1], trans[2] + dz, rot[0], rot[1], rot[2], rot[3])
        #status = self.arm_client.move_to_pose(userdata.is_right_arm, False, xo, yo, zo, rot[0], rot[1], rot[2], rot[3])
        #status = self.arm_client.move_to_pose(userdata.is_right_arm, False, xo-.15, yo+.1, zo-.2, rot[0], rot[1], rot[2], rot[3])
        # TODO: implement the fail with exhaustion
        if status == 0:
            logger.stops()
            return 'success'
        else:
            logger.stopf()
            return 'fail'

class ReturnTransitBowl2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail', 'fail with exhaustion'],
                             input_keys = ['is_right_arm', 'delta_y', 'bowl_to_deal', 'table_height', 'acceptable_height_above_table'],
                             output_keys = ['delta_y'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener
        self.fail_count = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state ReturnTransitBowl2')
        logger = EventLoggerClient.startfsm('ReturnTransitBowl2 (deal bowl fsm)')
        trans, rot = self.arm_client.get_transform(userdata.is_right_arm)
        # TODO figure out the right position for this
        x = .5
        y = .93
        z = 1.13 
        dely = .15
        if self.fail_count == 0:
            rospy.loginfo('resetting the delta y to zero')
            userdata.delta_y = dely
        if self.fail_count > 3:
            self.fail_count = 0
            return 'fail with exhaustion'
        else: 
            y = y - dely * self.fail_count
            rospy.loginfo('cutting down the y so to account for the shimmy: ' + str(y))
        status = self.arm_client.move_to_pose(userdata.is_right_arm, False, x, y, z, rot[0], rot[1], rot[2], rot[3])
        if status == 0:
            logger.stops()
            self.fail_count = 0
            return 'success'
        else:
            #status = self.arm_client.move_to_pose(userdata.is_right_arm, False, x, y, z, rot[0], rot[1], rot[2], rot[3])
            # TODO this could be a problem
            status = -1
            if status == 0:
                logger.stops()
                self.fail_count = 0
                return 'success'
            else:
                self.fail_count = self.fail_count + 1
                userdata.delta_y = dely
                logger.stopf()
                return 'fail'

class PlaceBowl(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys = ['is_right_arm', 'bowl_to_deal', 'table_height', 'acceptable_height_above_table'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener
        self.pick_and_place_manager = PickAndPlaceManager.get_pick_and_place_manager()

    def execute(self, userdata):
        rospy.loginfo('Executing state PlaceBowl')
        rospy.logwarn('AUTOMATICALLY ABORTING')
        return 'fail'
        logger = EventLoggerClient.startfsm('PlaceBowl (deal bowl fsm)')

        whicharm = 0 if userdata.is_right_arm else 1
        status = self.pick_and_place_manager.place_object(whicharm, userdata.bowl_to_deal.pose, padding = 0.01)
        rospy.loginfo('PAPM place request returned: ' + str(status))
        if status == 0:
            logger.stops()
            mlogger = StaticLoggerLocker.check_out_logger('RT LOGGER')
            if mlogger is not None:
                mlogger.stops()
            return 'success'
        else:
            logger.stopf()
            return 'fail'

class ForcePlaceBowl(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail', 'fail with exhaustion'],
                             input_keys = ['is_right_arm', 'bowl_to_deal', 'table_height', 'acceptable_height_above_table'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener
        self.pick_and_place_manager = PickAndPlaceManager.get_pick_and_place_manager()
        self.count = 0
        self.limit = 7

    def execute(self, userdata):
        rospy.loginfo('Executing state ForcePlaceBowl')
        rospy.logwarn('AUTOMATICALLY ABORTING')
        return 'fail with exhaustion'
        logger = EventLoggerClient.startfsm('ForcePlaceBowl (deal bowl fsm)')
        whicharm = 0 if userdata.is_right_arm else 1
        use_joint_open_loop = 0 if self.count < 3 else 1
        if use_joint_open_loop == 1:
            self.pick_and_place_manager.reset_collision_map()
            rospy.logwarn('using joint open loop in force place')
        status = self.pick_and_place_manager.place_object_override(whicharm, userdata.bowl_to_deal.pose, .05, use_joint_open_loop)
        rospy.loginfo('PAPM place request returned: ' + str(status))
        self.count = self.count + 1
        if status == 1:
            self.arm_client.retreat_arm(userdata.is_right_arm)
            logger.stops()
            mlogger = StaticLoggerLocker.check_out_logger('RT LOGGER')
            if mlogger is not None:
                mlogger.stops()
            self.count = 0
            return 'success'
        else:
            rospy.loginfo('force place failed.  count = ' + str(self.count))
            if self.count > self.limit:
                rospy.logerr('was unable to place')
                logger.stopf('fail with exhaustion')
                self.count = 0
                return 'fail with exhaustion'
            logger.stopf()
            return 'fail'

class DumpBowl(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys = ['is_right_arm', 'bowl_to_deal', 'table_height', 'acceptable_height_above_table'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener
        self.pick_and_place_manager = PickAndPlaceManager.get_pick_and_place_manager()
        self.count = 0
        self.limit = 5

    def execute(self, userdata):
        rospy.loginfo('Executing state DumpBowl')
        logger = EventLoggerClient.startfsm('DumpBowl (deal bowl fsm)')
        whicharm = 0 if userdata.is_right_arm else 1
        self.arm_client.move_delta_const_ee(False, -.1, 'z')
        self.arm_client.move_delta_const_ee(False, -.1, 'z')
        self.arm_client.move_delta_const_ee(False, -.1, 'z')
        self.pick_and_place_manager.open_gripper(whicharm)
        self.pick_and_place_manager.detach_object(whicharm)
        self.pick_and_place_manager.reset_collision_map()
        status = self.pick_and_place_manager.move_arm_to_side_open_loop(whicharm)
        if status == 1:
            logger.stops()
            mlogger = StaticLoggerLocker.check_out_logger('RT LOGGER')
            if mlogger is not None:
                mlogger.stops()
            self.count = 0
            return 'success'
        else:
            rospy.logwarn('moving arm to side failed, trying open loop')
            status = self.pick_and_place_manager.move_arm_to_side_open_loop(whicharm)
            if status == 1:
                logger.stops()
                mlogger = StaticLoggerLocker.check_out_logger('RT LOGGER')
                if mlogger is not None:
                    mlogger.stops()
                self.count = 0
                return 'success'
            else:
                logger.stopf()
                mlogger = StaticLoggerLocker.check_out_logger('RT LOGGER')
                if mlogger is not None:
                    mlogger.stopf()
                return 'fail'

class ResetRobotBasePos(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'],
                                   input_keys=['table_frame_x_offset', 'table_frame_y_offset'],
                                   output_keys=['table_frame_x_offset', 'table_frame_y_offset'])

    def execute(self, userdata):
        ROBOT_FRONT_X = 0.2 # TODO figure out this number for real!
        TABLE_MARGIN = 0.04
        rospy.loginfo('Executing state ResetRobotBasePos')
        logger = EventLoggerClient.startfsm('ResetRobotBasePos (mixing fsm)')
        bc = BaseClient.get_base_client()
        drive_x = -userdata.table_frame_x_offset
        drive_y = -userdata.table_frame_y_offset
        rospy.loginfo('DESIRED DRIVE X: ' + str(drive_x))
        rospy.loginfo('DESIRED DRIVE Y: ' + str(drive_y))

        bc.translate(True, drive_x)
        bc.translate(False, drive_y)

        userdata.table_frame_x_offset = 0
        userdata.table_frame_y_offset = 0
        logger.stops()
        return 'done'

class UserKeyboardArmClient(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.arm_client = ArmClient.get_arm_client()

    def execute(self, userdata):
        rospy.loginfo('Executing state UserKeyboardArmClient')
        logger = EventLoggerClient.startfsm('UserKeyboardArmClient (deal bowl fsm)')
        self.arm_client.ui_loop()
        logger.stops()
        return 'done'

class UserStateSelection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['1', '2', '3', '4', '5', '6', '7', '8', '10', '11', 'recurse']) 

    def execute(self, userdata):
        rospy.loginfo('Executing state UserStateSelection')
        logger = EventLoggerClient.startfsm('UserStateSelection (deal bowl fsm)')
        print '\n\n))))))))))))))))))))))))))))))))))))))))))))))))))))))'
        print '1) pre transit rotate'
        print '2) transit to mixing area'
        print '3) post transit rotate'
        print '4) move to pre pour position'
        print '5) pour'
        print '6) return transit bowl'
        print '7) place bowl'
        print '8) force place bowl'
        print '10) complete success'
        print '11) complete failure'
        choice = input('choice: ')
        try:
            logger.stops()
            cint = int(choice)
        except Exception:
            print 'user input did not register'
            return 'recurse'
        if choice == 1:
            return '1'
        elif choice == 2:
            return '2'
        elif choice == 3:
            return '3'
        elif choice == 4:
            return '4'
        elif choice == 5:
            return '5'
        elif choice == 6:
            return '6'
        elif choice == 7:
            return '7'
        elif choice == 8:
            return '8'
        elif choice == 10:
            return '10'
        elif choice == 11:
            return '11'
        else:
            return 'recurse'


def assemble_bowl_dealer_fsm( arms_to_try=(1,1), mixing_bowl_centroid=(1,0,1.1), ingred_bowl_radius=0.10, table_height=0.65, pour_height_above_centroid = 0.4, acceptable_height_above_table = 0.45, DEBUG = False):

    # arms to try, 1 = left, 0 = right

    sm = smach.StateMachine(outcomes=['bowl_dealing_success', 'bowl_dealing_failure'],
                            input_keys=['bowl_to_deal', 'arms_to_try', 'acceptable_height_above_table', 
                                        'table_height', 'mixing_bowl', 'mixing_area_pose_x_tol',
                                        'mixing_area_pose_y_tol', 'mixing_area_pose_z_tol',
                                        'table_height', 'pour_height_above_centroid', 'ingred_bowl_radius',
                                        'mixing_area_pose_x_tol', 'mixing_area_pose_y_tol', 'mixing_area_pose_z_tol',
                                        'table_frame_y_offset', 'table_frame_y_offset', 'delta_y', 'max_left_movement', 'max_right_movement'],
                            output_keys=['bowl_to_deal', 'arms_to_try', 'acceptable_height_above_table', 
                                        'table_height', 'mixing_bowl', 'mixing_area_pose_x_tol',
                                        'mixing_area_pose_y_tol', 'mixing_area_pose_z_tol',
                                        'table_height', 'pour_height_above_centroid', 'ingred_bowl_radius',
                                        'mixing_area_pose_x_tol', 'mixing_area_pose_y_tol', 'mixing_area_pose_z_tol',
                                        'table_frame_y_offset', 'table_frame_y_offset', 'delta_y', 'max_left_movement', 'max_right_movement'])
    sm.userdata.arms_to_try = arms_to_try
    #sm.userdata.ingredient_fake_name = ingredient_fake_name
    #sm.userdata.ingredient_original_pose = ingredient_original_pose
    sm.userdata.acceptable_height_above_table = acceptable_height_above_table
    sm.userdata.table_height = table_height
    #sm.userdata.mixing_bowl_centroid = mixing_bowl_centroid
    sm.userdata.pour_height_above_centroid = pour_height_above_centroid
    sm.userdata.bowl_radius = ingred_bowl_radius 
    #sm.userdata.mixing_area_pose = (mixing_bowl_centroid[0], mixing_bowl_centroid[1], mixing_bowl_centroid[2] + acceptable_height_above_table)
    sm.userdata.mixing_area_pose_x_tol = (0.05, 0.05)
    sm.userdata.mixing_area_pose_y_tol = (0.05, 0.05)
    sm.userdata.mixing_area_pose_z_tol = (0.10, 0)
    sm.userdata.table_frame_x_offset = 0
    sm.userdata.table_frame_y_offset = 0
    sm.userdata.delta_x = 0
    sm.userdata.delta_y = 0.1
    sm.userdata.max_right_movement = 0.5
    sm.userdata.max_left_movement = 0.5


    if DEBUG:
        fail_state = 'USER POSE ADJUSTMENT'
    else:
        fail_state = 'bowl_dealing_failure'

    with sm:
        smach.StateMachine.add('INITIALIZE', InitializeState(True),
                                transitions={'done':'REFINE M-BOWL POS',
                                             'skip to success':'bowl_dealing_success'},
                                remapping={'bowl_to_deal':'bowl_to_deal',  #TODO: add more remaps here
                                            'arms_to_try':'arms_to_try',
                                            'is_right_arm':'is_right_arm',
                                            'global_grasp_pose':'global_grasp_pose',
                                            'table_frame_y_offset':'table_frame_y_offset',
                                            'grasp_pose':'grasp_pose'})
        smach.StateMachine.add('REFINE M-BOWL POS', RefineMixingBowlPosition(),
                                transitions={'done':'GRAB',
                                             'failed':fail_state},
                                remapping={'mixing_bowl':'mixing_bowl',
                                           'table_frame_x_offset':'table_frame_x_offset',
                                           'table_frame_y_offset':'table_frame_y_offset'})
        smach.StateMachine.add('GRAB', Grab(),
                                transitions={'success':'POST GRASP LIFT',
                                             'fail':'MOVE ROBOT LEFT',
                                             'fail with exhaustion':fail_state},
                                remapping={'bowl_to_deal':'bowl_to_deal',
                                            'arms_to_try':'arms_to_try',
                                            'is_right_arm':'is_right_arm',
                                            'global_grasp_pose':'global_grasp_pose',
                                            'table_frame_x_offset':'table_frame_x_offset',
                                            'table_frame_y_offset':'table_frame_y_offset',
                                            'grasp_pose':'grasp_pose'})
        smach.StateMachine.add('MOVE ROBOT LEFT', MoveRobotOver(False),
                                transitions={'success':'GRAB',
                                             'fail':'GRAB'},
                                remapping={'max_left_movement':'max_left_movement',
                                           'max_right_movement':'max_right_movement',
                                           'delta_y':'delta_y',
                                           'table_frame_x_offset':'table_frame_x_offset',
                                           'move_over_positions_exhausted':'move_over_positions_exhausted',
                                           'table_frame_y_offset':'table_frame_y_offset'})
        smach.StateMachine.add('POST GRASP LIFT', ChangeGripperObjHeight(),
                                transitions={'success':'MOVE ROBOT RIGHT',
                                             'fail':'MOVE ROBOT RIGHT'},
                                remapping={'acceptable_height_above_table':'acceptable_height_above_table',
                                            'table_height':'table_height',
                                            'current_height_above_table':'current_height_above_table',
                                            'delta_y':'delta_y',
                                            'is_right_arm':'is_right_arm'})
        smach.StateMachine.add('MOVE ROBOT RIGHT', MoveRobotOver(True),
                                transitions={'success':'PRE TRANSIT ROTATE',
                                             'fail':'GRAB'},
                                remapping={'max_left_movement':'max_left_movement',
                                           'max_right_movement':'max_right_movement',
                                           'delta_y':'delta_y',
                                           'table_frame_x_offset':'table_frame_x_offset',
                                           'move_over_positions_exhausted':'move_over_positions_exhausted',
                                           'table_frame_y_offset':'table_frame_y_offset'})
        smach.StateMachine.add('PRE TRANSIT ROTATE', RotateToKnownPose(),
                                transitions={'success':'TRANSIT TO MIXING AREA',
                                             'fail':'PRE TRANSIT ROTATE',
                                             'fail with exhaustion':fail_state},
                                remapping={'grasp_pose':'grasp_pose',
                                            'current_known_pose_name':'current_known_pose_name',
                                            'hand_poses_exhausted':'hand_poses_exhausted',
                                            'is_right_arm':'is_right_arm'})
        smach.StateMachine.add('TRANSIT TO MIXING AREA', TransitBowlToMixingArea(),
                                transitions={'success':'MOVE TO PRE POUR POSITION',
                                             'fail':'PRE TRANSIT ROTATE',
                                             'fail with exhaustion':fail_state},
                                remapping={'mixing_bowl':'mixing_bowl',
                                            'mixing_area_pose_x_tol':'mixing_area_pose_x_tol',
                                            'mixing_area_pose_y_tol':'mixing_area_pose_y_tol',
                                            'mixing_area_pose_z_tol':'mixing_area_pose_z_tol',
                                            'acceptable_height_above_table':'acceptable_height_above_table',
                                            'hand_poses_exhausted':'hand_poses_exhausted',
                                            'is_right_arm':'is_right_arm'})
        smach.StateMachine.add('MOVE TO PRE POUR POSITION', MoveToPrePourPosition(),
                                transitions={'success':'POUR',
                                             'fail':'POST TRANSIT ROTATE',
                                             'unknown pre pour pose':'POST TRANSIT ROTATE',
                                             'fail with exhaustion':fail_state},
                                remapping={'mixing_bowl_centroid':'mixing_bowl_centroid',
                                            'current_known_pose_name':'current_known_pose_name',
                                            'pour_height_above_centroid':'pour_height_above_centroid',
                                            'hand_poses_exhausted':'hand_poses_exhausted',
                                            'bowl_radius':'bowl_radius',
                                            'is_right_arm':'is_right_arm'})
        smach.StateMachine.add('POST TRANSIT ROTATE', RotateToKnownPose(),
                                transitions={'success':'MOVE TO PRE POUR POSITION',
                                             'fail':'POST TRANSIT ROTATE',
                                             'fail with exhaustion':fail_state},
                                remapping={'grasp_pose':'grasp_pose',
                                            'current_known_pose_name':'current_known_pose_name',
                                            'hand_poses_exhausted':'hand_poses_exhausted',
                                            'is_right_arm':'is_right_arm'})
        smach.StateMachine.add('POUR', Pour(),
                                transitions={'success':'STEEP POUR',
                                             'fail':'STEEP POUR'},
                                remapping={'current_known_pose_name':'current_known_pose_name',
                                            'is_right_arm':'is_right_arm'})
        smach.StateMachine.add('STEEP POUR', SteepPour(),
                                transitions={'success':'SUPER STEEP POUR',
                                             'fail':'SUPER STEEP POUR'},
                                remapping={'current_known_pose_name':'current_known_pose_name',
                                            'is_right_arm':'is_right_arm'})
        smach.StateMachine.add('SUPER STEEP POUR', SuperSteepPour(),
                                transitions={'success':'SHAKE',
                                             'fail':'SHAKE'},
                                remapping={'current_known_pose_name':'current_known_pose_name',
                                            'is_right_arm':'is_right_arm'})
        smach.StateMachine.add('SHAKE', Shake(),
                                transitions={'done':'REVERT'},
                                remapping={'is_right_arm':'is_right_arm'})
        smach.StateMachine.add('REVERT', Revert(),
                                transitions={'success':'RETURN TRANSIT BOWL',
                                             'fail':'RETURN TRANSIT BOWL'},
                                remapping={'current_known_pose_name':'current_known_pose_name',
                                            'is_right_arm':'is_right_arm',
                                            'do_sloppy_drop':'do_sloppy_drop'})
        smach.StateMachine.add('RETURN TRANSIT BOWL', ReturnTransitBowl(),
                                transitions={'success':'PLACE BOWL',
                                             'fail':'PLACE BOWL'},
                                remapping={'current_known_pose_name':'current_known_pose_name',
                                            'is_right_arm':'is_right_arm',
                                            'table_height':'table_height',
                                            'acceptable_height_above_table':'acceptable_height_above_table'})
        smach.StateMachine.add('PLACE BOWL', PlaceBowl(),
                                transitions={'success':'bowl_dealing_success',
                                             'fail':'FORCE PLACE BOWL'},
                                remapping={'current_known_pose_name':'current_known_pose_name',
                                            'is_right_arm':'is_right_arm',
                                            'table_height':'table_height',
                                            'acceptable_height_above_table':'acceptable_height_above_table'})
        smach.StateMachine.add('FORCE PLACE BOWL', ForcePlaceBowl(),
                                transitions={'success':'bowl_dealing_success',
                                             'fail':'FORCE PLACE BOWL',
                                             'fail with exhaustion':'RETURN TRANSIT BOWL AGAIN'},
                                remapping={'current_known_pose_name':'current_known_pose_name',
                                            'is_right_arm':'is_right_arm',
                                            'table_height':'table_height',
                                            'acceptable_height_above_table':'acceptable_height_above_table'})
        smach.StateMachine.add('RETURN TRANSIT BOWL AGAIN', ReturnTransitBowl2(),
                                transitions={'success':'DUMP BOWL',
                                             'fail':'MOVE ROBOT LEFT AGAIN',
                                             'fail with exhaustion':'DUMP BOWL'},
                                remapping={'current_known_pose_name':'current_known_pose_name',
                                            'is_right_arm':'is_right_arm',
                                            'table_height':'table_height',
                                            'acceptable_height_above_table':'acceptable_height_above_table'})
        smach.StateMachine.add('MOVE ROBOT LEFT AGAIN', MoveRobotOver(False),
                                transitions={'success':'RETURN TRANSIT BOWL AGAIN',
                                             'fail':'RETURN TRANSIT BOWL AGAIN'},
                                remapping={'max_left_movement':'max_left_movement',
                                           'max_right_movement':'max_right_movement',
                                           'delta_y':'delta_y',
                                           'table_frame_x_offset':'table_frame_x_offset',
                                           'move_over_positions_exhausted':'move_over_positions_exhausted',
                                           'table_frame_y_offset':'table_frame_y_offset'})
        smach.StateMachine.add('DUMP BOWL', DumpBowl(),
                                transitions={'success':'MOVE ROBOT RIGHT AGAIN',
                                             'fail':fail_state},
                                remapping={'current_known_pose_name':'current_known_pose_name',
                                            'is_right_arm':'is_right_arm',
                                            'table_height':'table_height',
                                            'acceptable_height_above_table':'acceptable_height_above_table'})
        smach.StateMachine.add('MOVE ROBOT RIGHT AGAIN', MoveRobotOver(True),
                                transitions={'success':'MOVE ROBOT BACK',
                                             'fail':fail_state},
                                remapping={'max_left_movement':'max_left_movement',
                                           'max_right_movement':'max_right_movement',
                                           'delta_y':'delta_y',
                                           'table_frame_x_offset':'table_frame_x_offset',
                                           'move_over_positions_exhausted':'move_over_positions_exhausted',
                                           'table_frame_y_offset':'table_frame_y_offset'})
        smach.StateMachine.add('MOVE ROBOT BACK', ResetRobotBasePos(),
                                transitions={'done':'bowl_dealing_success'},
                                remapping={'table_frame_x_offset':'table_frame_x_offset',
                                           'table_frame_y_offset':'table_frame_y_offset'})
        if DEBUG:
            smach.StateMachine.add('USER POSE ADJUSTMENT', UserKeyboardArmClient(),
                                    transitions={'done':'USER STATE SELECTION'})
            smach.StateMachine.add('USER STATE SELECTION', UserStateSelection(),
                                    transitions={'1':'PRE TRANSIT ROTATE',
                                                '2':'TRANSIT TO MIXING AREA',
                                                '3':'POST TRANSIT ROTATE',
                                                '4':'MOVE TO PRE POUR POSITION',
                                                '5':'POUR',
                                                '6':'RETURN TRANSIT BOWL',
                                                '7':'PLACE BOWL',
                                                '8':'FORCE PLACE BOWL',
                                                '10':'bowl_dealing_success',
                                                '11':'bowl_dealing_failure',
                                                'recurse':'USER STATE SELECTION'})
    # end with container
    return sm


def main():
    try:
        sm = assemble_bowl_dealer_fsm('fake_name', ((0,0,0), (0,0,0,0)))
        sis = smach_ros.IntrospectionServer('introspection_server', sm, '/DEAL_BOWL')
        sis.start()
        #rospy.spin()
        outcome = sm.execute()
    finally:
        sis.stop()

if __name__ == '__main__':
    rospy.init_node('bowl_dealer')
    main()
