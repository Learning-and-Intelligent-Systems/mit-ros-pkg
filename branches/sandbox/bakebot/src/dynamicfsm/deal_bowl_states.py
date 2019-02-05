#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
import smachforward
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


class DealGrab(smachforward.FState):
    def __init__(self, ingred_name):
        smachforward.FState.__init__(self, 
                             input_keys=['mixing_bowl', 'bowl_to_deal', 'table_frame_x_offset', 'table_frame_y_offset'],
                             output_keys=['mixing_bowl', 'bowl_to_deal', 'grasp_pose', 'global_grasp_pose', 'delta_x', 'delta_y','bowl_radius'])
        self.pick_and_place_manager = PickAndPlaceManager.get_pick_and_place_manager()
        self.arm_client = ArmClient.get_arm_client()
        self.broad_object_manager = BroadTabletopObjectManager.get_broad_tabletop_object_manager(self.pick_and_place_manager)
        self.ingred_name = ingred_name

    def display_and_cleanup_success(self, userdata):
        print '\n\n\n**********************************************************************************\n\n'
        framename = 'r_gripper_tool_frame' 
        print 'storing barcode pos:', GripperBowlPoseManager.store_barcode_pos(userdata.bowl_to_deal, framename)
        x = userdata.bowl_to_deal.box_dims[0]
        y = userdata.bowl_to_deal.box_dims[1]
        z = userdata.bowl_to_deal.box_dims[2]
        print x,y,z
        print 'type: ', userdata.bowl_to_deal.type
        r = (math.sqrt(x**2 + y**2) ) / 2.0
        rospy.loginfo('!!!!!!!!!!! RADIUS OF BOWL: ' + str(r))
        userdata.bowl_radius = r;
        print '\n\n\n**********************************************************************************\n\n'

    def display_and_do_grasp(self, userdata):
        print 'I want to grab the bowl at: ', userdata.bowl_to_deal.pose.pose.position.x, userdata.bowl_to_deal.pose.pose.position.y
        print 'x off: ', userdata.table_frame_x_offset
        print 'y off: ', userdata.table_frame_y_offset
        whicharm, result, grasp_pose = self.broad_object_manager.refine_and_grasp(userdata.bowl_to_deal.collision_name, (1,1), userdata.table_frame_x_offset, userdata.table_frame_y_offset)
        userdata.grasp_pose = grasp_pose
        trans, rot = self.arm_client.get_transform(False) # (left arm only)
        userdata.global_grasp_pose = (trans, rot)
        print 'global grasp trans: ', trans
        print 'global grasp rot: ', rot
        return result
        
    def execute(self, userdata):
        print '\n\nclearing static gripper bowl pose manager!'

        GripperBowlPoseManager.clear_static_gripper_bowl_pose_manager()
        logger = EventLoggerClient.startfsm('refine mixing bowl')
        rospy.loginfo('Executing state RefineMixingBowlPosition')
        if not self.broad_object_manager.filtered:
            self.broad_object_manager.filter_ingredients()
        userdata.bowl_to_deal = self.broad_object_manager.get_ingredient(self.ingred_name)
        userdata.mixing_bowl = self.broad_object_manager.mixing_bowl
        if userdata.mixing_bowl == None:
            rospy.loginfo('null mixing bowl this is a big problem')
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
            return 'failure'
        userdata.mixing_bowl.pose = detected_mixing_bowl.pose
        x = userdata.mixing_bowl.pose.pose.position.x
        y = userdata.mixing_bowl.pose.pose.position.y
        z = userdata.mixing_bowl.pose.pose.position.z
        rospy.loginfo('\n\n\ncorrected pose of mixing bowl: ' + str((x,y,z)) + '\n\n\n')
        logger.stops()

        rospy.loginfo('Executing state Grab')
        logger = EventLoggerClient.startfsm('grab (deal bowl fsm)')
        result = self.display_and_do_grasp(userdata)

        if result == 'succeeded':
            self.display_and_cleanup_success(userdata)
            logger.stops()
            return 'success'
        else:
            self.base_client = BaseClient.get_base_client()
            drivex = -.15
            status = self.base_client.translate(True,drivex)
            result = self.display_and_do_grasp(userdata)
            status = self.base_client.translate(True,-drivex)
            if result == 'succeeded':
                self.display_and_cleanup_success(userdata)
                logger.stops()
                return 'success'
            else:
                drivey = drivex
                status = self.base_client.translate(False,drivey)
                result = self.display_and_do_grasp(userdata)
                status = self.base_client.translate(False,-drivey)
                if result == 'succeeded':
                    self.display_and_cleanup_success(userdata)
                    logger.stops()
                    return 'success'
                else:
                    return 'failure'

    def get_name_str(self):
        return 'grab'

    def get_params_str(self):
        return '(?x ?y)'

    def get_precondition_str(self):
        return '(and (INGRED ?x) (GRIPPER ?y) (reset ?y) (free ?y))'

    def get_effect_str(self):
		return '(and (carry ?y ?x) (not (free ?y)) (not (reset ?y)))'

    def parametrize(self, ff_line):
        print 'parametrizing deal bowl based on ff_line:', ff_line
        return True
        


class DealLift(smachforward.FState):
    def __init__(self):
        smachforward.FState.__init__(self,
                             input_keys=['acceptable_height_above_table', 'table_height', 'table_frame_x_offset', 'table_frame_y_offset','delta_x', 'delta_y'],
                             output_keys=['delta_y', 'delta_x'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener

    def execute(self, userdata):
        rospy.loginfo('Executing state ChangeGripperObjHeight')
        is_right_arm = False
        logger = EventLoggerClient.startfsm('lift (deal bowl fsm)')
        (trans, rot) = self.arm_client.get_transform(False)
        desired_z = userdata.acceptable_height_above_table + userdata.table_height
        print '*************************************************\n\n\n'
        print desired_z
        print userdata.acceptable_height_above_table
        print '*************************************************\n\n\n'
        rospy.loginfo('lifting the arm to desired z: ' + str(desired_z))
        status = self.arm_client.move_to_pose_const_ee(is_right_arm, trans[0], trans[1], desired_z)
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
            print '\n\n\nwarning, deal lift failed!\n\n\n'
            status = self.arm_client.move_to_pose_const_ee(is_right_arm, trans[0], trans[1], desired_z)
            status = self.arm_client.move_to_pose_const_ee(is_right_arm, trans[0], trans[1], desired_z)
            status = self.arm_client.move_to_pose_const_ee(is_right_arm, trans[0], trans[1], desired_z, determination=1)
            return 'success'

    def get_name_str(self):
        return 'lift'

    def get_params_str(self):
        return '(?x ?y)'

    def get_precondition_str(self):
		return '(and (INGRED ?x) (GRIPPER ?y) (carry ?y ?x))'

    def get_effect_str(self):
		return '(and (in-transit ?x) (not (on-table ?x)) (not (cardinal ?x)) (randomp ?x))'

class DealRotateToKnown(smachforward.FState):
    def __init__(self):
        smachforward.FState.__init__(self, 
                             input_keys=['grasp_pose', 'bowl_to_deal'],
                             output_keys=['current_known_pose_name', 'hand_poses_exhausted', 'use_new_pose_manager'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener

    def execute(self, userdata):
        rospy.loginfo('Executing state RotateToKnownPose')
        logger = EventLoggerClient.startfsm('RotateToKnownPose (deal bowl fsm)')
        pose_manager = GripperBowlPoseManager.get_gripper_bowl_pose_manager(userdata.bowl_to_deal, self.arm_client, False)
        #pose, name_of_pose = self.pose_manager.get_current_recommended_pose()
        # I may be missing out on the first one if I don't call current here
        # if that's the case then just use the current here and call next right before returning failure
        rospy.loginfo('getting the next recommended pose')
        pose, name_of_pose = pose_manager.get_next_recommended_pose()

        rospy.loginfo('rotating end effector to known pose: ' + name_of_pose + ' ' + str(pose))
        status = self.arm_client.rotate_end_effector(False, pose[0], pose[1], pose[2], pose[3])
        if status == 0:
            rospy.loginfo('success')
            userdata.current_known_pose_name = name_of_pose
            logger.stops()
            return 'success'
        else:
            rospy.loginfo('trying one more time')
            status = self.arm_client.rotate_end_effector(False, pose[0], pose[1], pose[2], pose[3])
            if status == 0:
                rospy.loginfo('success')
                userdata.current_known_pose_name = name_of_pose
                logger.stops()
                return 'success'
            rospy.loginfo('still failed!!!!!!!!!!!!!!!')
            rospy.logwarn('rotating the end effector failed.  going to next preferred pose by recursing via FSM')
            userdata.current_known_pose_name = 'unknown'
            logger.stopf()
            return 'failure'

    def get_name_str(self):
        return 'rotate-to-known'

    def get_params_str(self):
        return '(?x ?y)'

    def get_precondition_str(self):
		return '(and (INGRED ?x) (GRIPPER ?y) (carry ?y ?x))'

    def get_effect_str(self):
		return '(and (not (randomp ?x)) (cardinal ?x))'

class DealTransit(smachforward.FState):
    def __init__(self):
        smachforward.FState.__init__(self, 
                             input_keys=['mixing_bowl', 'mixing_area_pose_x_tol', 'mixing_area_pose_y_tol', 'mixing_area_pose_z_tol', 'acceptable_height_above_table'],
                             output_keys=[])
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
        pose_to_point_at = [x_pos, y_pos, 0.65]
        self.pick_and_place_manager.point_head(pose_to_point_at, 'base_link')
        status = self.arm_client.move_to_pose_const_ee(False, x_pos, y_pos, z_pos, 0, x_plus_minus, y_plus_minus, z_plus_minus, 1)
        if status == 0:
            rospy.loginfo('succeeded')
            logger.stops()
            return 'success'
        else:
            rospy.logwarn('failed to plan transit from this rotation.  going to re-rotate and try again')
            logger.stopf()
            return 'failure'

    def get_name_str(self):
        return 'transit'

    def get_params_str(self):
        return '(?x ?y)'

    def get_precondition_str(self):
		return '(and (INGRED ?x) (GRIPPER ?y) (carry ?y ?x) (cardinal ?x) (in-transit ?x))'

    def get_effect_str(self):
		return '(and (over-mb ?x))'

class DealPour(smachforward.FState):
    def __init__(self):
        smachforward.FState.__init__(self, 
                             input_keys=['mixing_bowl', 'pour_height_above_centroid', 'current_known_pose_name', 'bowl_radius'],
                             output_keys=['current_known_pose_name'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener
        self.papm = PickAndPlaceManager.get_pick_and_place_manager()

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveToPrePourPosition')
        is_right_arm = False
        logger = EventLoggerClient.startfsm('MoveToPrePourPosition (deal bowl fsm)')
        mixing_bowl_pose = userdata.mixing_bowl.pose.pose.position
        x_pos = mixing_bowl_pose.x
        y_pos = mixing_bowl_pose.y
        z_pos = mixing_bowl_pose.z
        z_pos = z_pos + userdata.pour_height_above_centroid
        # need to figure out where to move to for the pre pose based on the current pose
        if userdata.current_known_pose_name == 'unknown':
            rospy.logwarn('unknown current pose, need to rotate to a known position to calculate pre pour position')
            # going to try to match here
            isRightArm = is_right_arm
            (trans, rot) = self.arm_client.get_transform(isRightArm)
            pose_iterator = GripperBowlPoseManager.ALL_POSES.iteritems()
            threshold = .1
            at_known_pose = False
            while True:
                try:
                    key_pose = pose_iterator.next()
                    pose = key_pose[1]
                    close_enough = True
                    print 'checking this keypose: ', key_pose, pose
                    for actual, desired in zip(rot, pose):
                        close_enough = close_enough and (math.fabs(actual-desired) < threshold)
                    if close_enough:
                        print 'I am at this pose:', pose, rot
                        at_known_pose = True
                        print 'and this pose is called', key_pose[0]
                        userdata.current_known_pose_name = key_pose[0]
                        break
                except StopIteration:
                    break
            if at_known_pose:
                print 'recursing with known pose'
                logger.stops('recursing')
                return self.execute(userdata)
            else:
                print 'could not determine pose'
                logger.stopf('unknown current pose')
                return 'failure'
        else:
            try:
                dx, dy, dz = GripperBowlPoseManager.POUR_ADJUSTMENT_DIRS_FROM_HIGH_CENTROID[userdata.current_known_pose_name]
            except KeyError:
                rospy.logwarn('could not find a pour adjustment for ' + str(userdata.current_known_pose_name))
                rospy.logwarn('going to try to rotate to another pose')
                return 'failure'
            rospy.loginfo('POUR INFO ********************************************************')
            print 'current known pose: ', userdata.current_known_pose_name
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
            status = self.arm_client.move_to_pose_const_ee(is_right_arm, x_pos, y_pos, z_pos, 1, x_plus_minus, y_plus_minus, z_plus_minus, 1)
            if status == 0:
                rospy.loginfo('succeeded')
                logger.stops()
                #return 'success'
            else:
                rospy.logwarn('failed to plan move to pre pour position')
                logger.stopf()
                return 'failure'
######### POUR
        rospy.loginfo('Executing state Pour')
        logger = EventLoggerClient.startfsm('Pour (deal bowl fsm)')
        if userdata.current_known_pose_name == 'unknown':
            rospy.logwarn('unknown current pose, need to rotate to a known position before pouring')
            logger.stopf('unknown current pose')
            return 'failure'
        px, py, pz, pw = GripperBowlPoseManager.POUR_POSES[userdata.current_known_pose_name]
        dx, dy, dz = GripperBowlPoseManager.POUR_ADJUSTMENT_DIRS_FROM_HIGH_CENTROID[userdata.current_known_pose_name]
        (trans, rot) = self.arm_client.get_transform(False)
        (x, y, z) = trans
        delta = 0.11 
        x = x + delta * dx
        y = y + delta * dy
        z = z + delta * dz
        status = self.arm_client.rotate_end_effector(False, px, py, pz, pw, determination=0)
        status = self.arm_client.move_to_pose(False, True, x, y, z, px, py, pz, pw)
        if status == 0:
            rospy.loginfo('pour succeeded')
            logger.stops()
            mlogger = StaticLoggerLocker.check_out_logger('TRANSIT LOGGER')
            if mlogger is not None:
                mlogger.stops()
            tlogger  = EventLoggerClient('milestone', annotation='successful pour to revert')
            StaticLoggerLocker.check_in_logger('POUR LOGGER', tlogger) 
            #return 'success'
        else:
            rospy.logwarn('pour failed to rotate to position')
            logger.stopf()
            #return 'failure'
######### STEEP POUR
        rospy.loginfo('Executing state SteepPour')
        logger = EventLoggerClient.startfsm('SteepPour (deal bowl fsm)')
        if userdata.current_known_pose_name == 'unknown':
            rospy.logwarn('unknown current pose, need to rotate to a known position before pouring')
            logger.stopf('unkown pos')
            return 'failure'
        px, py, pz, pw = GripperBowlPoseManager.STEEP_POUR_POSES[userdata.current_known_pose_name]
        dx, dy, dz = GripperBowlPoseManager.POUR_ADJUSTMENT_DIRS_FROM_HIGH_CENTROID[userdata.current_known_pose_name]
        (trans, rot) = self.arm_client.get_transform(False)
        (x, y, z) = trans
        #delta = 0.10
        #delta = .04
        delta = 0
        x = x + delta * dx
        y = y + delta * dy
        z = z + delta * dz
        print 'steep pour info..................'
        print 'delta: ', delta, 'dx,dy,dz:', dx, dy, dz
        print 'px, py, pz:', px, py, pz
        status = self.arm_client.move_to_pose(False, True, x, y, z, px, py, pz, pw)
        if status == 0:
            rospy.loginfo('steep pour succeeded')
            logger.stops()
            #return 'success'
        else:
            rospy.logwarn('||||||||||||||||   steep pour failed to rotate to position')
            #status = self.arm_client.rotate_end_effector(userdata.is_right_arm, px, py, pz, pw, determination=0, use_interpolation = False)
            status = self.arm_client.move_to_pose(False, False, x, y, z, px, py, pz, pw)
            if status == 0:
                rospy.loginfo('steep pour succeeded, but only after turning interpolation off')
                logger.stops('had to turn of interpolation')
                #return 'success'
            else:
                rospy.logwarn('||||||||||||||||   steep pour failed to rotate to position, trying again with determination!')
                #status = self.arm_client.move_to_pose_const_ee(False, x, y, z, determination = 2)
                #rospy.logwarn('status after shift: ' + str(status))
                #status = self.arm_client.rotate_end_effector(False, px, py, pz, pw, determination=2)
                status = self.arm_client.move_to_pose_const_ee(False, x, y, z)
                rospy.logwarn('status after shift: ' + str(status))
                status = self.arm_client.rotate_end_effector(False, px, py, pz, pw)
                if status == 0:
                    rospy.loginfo('steep pour succeeded, but only after trying with determination')
                    logger.stops('had to turn of interpolation and with determination')
                    #return 'success'
                else:
                    rospy.logwarn('||||||||||||||||   steep pour failed to rotate to position FAIL FAIL FAIL!')
                    logger.stopf()
                    #return 'fail'
######### SUPER STEEP POUR
        rospy.loginfo('Executing state SuperSteepPour')
        logger = EventLoggerClient.startfsm('SuperSteepPour (deal bowl fsm)')
        px, py, pz, pw = GripperBowlPoseManager.SUPER_STEEP_POUR_POSES[userdata.current_known_pose_name]
        dx, dy, dz = GripperBowlPoseManager.POUR_ADJUSTMENT_DIRS_FROM_HIGH_CENTROID[userdata.current_known_pose_name]
        (trans, rot) = self.arm_client.get_transform(False)
        (x, y, z) = trans
        delta = 0.0
        x = x + delta * dx
        y = y + delta * dy
        z = z + delta * dz
        print 'super steep pour info..................'
        print 'delta: ', delta, 'dx,dy,dz:', dx, dy, dz
        print 'px, py, pz:', px, py, pz
        status = self.arm_client.move_to_pose(False, True, x, y, z, px, py, pz, pw)
        if status == 0:
            rospy.loginfo('super steep pour succeeded')
            logger.stops()
            #return 'success'
        else:
            rospy.logwarn('||||||||||||||||   super steep pour failed to rotate to position')
            status = self.arm_client.move_to_pose(False, False, x, y, z, px, py, pz, pw)
            if status == 0:
                rospy.loginfo('super steep pour succeeded, but only after turning interpolation off')
                logger.stops('had to turn of interpolation')
                #return 'success'
            else:
                rospy.logwarn('||||||||||||||||   super steep pour failed to rotate to position, trying again with determination!')
                #status = self.arm_client.move_to_pose_const_ee(False, x, y, z, determination = 2) 
                #rospy.logwarn('status after shift: ' + str(status)) 
                #status = self.arm_client.rotate_end_effector(False, px, py, pz, pw, determination=2)
                status = self.arm_client.move_to_pose_const_ee(False, x, y, z)
                rospy.logwarn('status after shift: ' + str(status)) 
                status = self.arm_client.rotate_end_effector(False, px, py, pz, pw)
                if status == 0:
                    rospy.loginfo('super steep pour succeeded, but only after trying with determination')
                    logger.stops('had to turn of interpolation and with determination')
                    #return 'success'
                else:
                    rospy.logwarn('||||||||||||||||   steep pour failed to rotate to position FAIL FAIL FAIL!')
                    logger.stopf()
                    return 'failure' # TODO may want to just make this success
######### SHAKE
        rospy.loginfo('Executing state Shake')
        logger = EventLoggerClient.startfsm('Shake (deal bowl fsm)')
        whicharm = 1 
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
        return 'success'

    def get_name_str(self):
        return 'pour'

    def get_params_str(self):
        return '(?x ?y)'

    def get_precondition_str(self):
		return '(and (INGRED ?x) (GRIPPER ?y) (carry ?y ?x) (in-transit ?x) (over-mb ?x) (cardinal ?x))'

    def get_effect_str(self):
		return '(and (inverted ?x) (poured-out ?x))'

class DealRevert(smachforward.FState):
    def __init__(self):
        smachforward.FState.__init__(self, 
                             input_keys = ['current_known_pose_name'],
                             output_keys = ['do_sloppy_drop'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener

    def execute(self, userdata):
        isRightArm = False
        rospy.loginfo('Executing state Revert')
        logger = EventLoggerClient.startfsm('Revert (deal bowl fsm)')
        mlogger = StaticLoggerLocker.check_out_logger('POUR LOGGER')
        if userdata.current_known_pose_name == 'unknown':
            rospy.logwarn('unknown current pose name, cannot rotate back to it')
            userdata.do_sloppy_drop = True
            logger.stopf()
            if mlogger is not None:
                mlogger.stopf()
            return 'failure'
        px, py, pz, pw = GripperBowlPoseManager.ALL_POSES[userdata.current_known_pose_name]
        (trans, rot) = self.arm_client.get_transform(isRightArm)
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
            return 'success'
            #return 'failure'

    def get_name_str(self):
        return 'revert'

    def get_params_str(self):
        return '(?x ?y)'

    def get_precondition_str(self):
		return '(and (INGRED ?x) (GRIPPER ?y) (carry ?y ?x) (in-transit ?x))'

    def get_effect_str(self):
		return '(and (not (inverted ?x)) (cardinal ?x))'

class DealDump(smachforward.FState):
    def __init__(self):
        smachforward.FState.__init__(self, 
                             input_keys = ['bowl_to_deal', 'table_height', 'acceptable_height_above_table'])
        self.arm_client = ArmClient.get_arm_client()
        self.pick_and_place_manager = PickAndPlaceManager.get_pick_and_place_manager()
        self.tf_listener = self.arm_client.tf_listener
        self.fail_count = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state ReturnTransitBowl')
        logger = EventLoggerClient.startfsm('ReturnTransitBowl (deal bowl fsm)')
        trans, rot = self.arm_client.get_transform(False)
        pose = userdata.bowl_to_deal.pose.pose
        trans_orig = (.1, .6, pose.position.z)
        rot_orig = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        xo, yo, zo = trans_orig
        zo = userdata.table_height + userdata.acceptable_height_above_table
        xrang = (xo - .1, xo + .1)
        yrang = (yo - .1, yo + .1)
        zrang = (zo, zo)
        quater_xyzw = rot
        #dz = .25
        dz = 0
        status = self.arm_client.move_to_pose(False, False, trans[0], trans[1], trans[2] + dz, rot[0], rot[1], rot[2], rot[3])
        if status == 0:
            logger.stops()
            #return 'success'
        else:
            logger.stopf()
            #return 'fail'

        rospy.loginfo('Executing state ReturnTransitBowl2')
        logger = EventLoggerClient.startfsm('ReturnTransitBowl2 (deal bowl fsm)')
        trans, rot = self.arm_client.get_transform(False)
        # TODO figure out the right position for this
        x = .5
        y = .93
        z = 1.13 
        dely = .15
        y = y - dely 
        rospy.loginfo('cutting down the y so to account for the shimmy: ' + str(y))
        self.base_client = BaseClient.get_base_client()
        status = self.base_client.translate(False, 1 * abs(dely))
        status = self.arm_client.move_to_pose(False, False, x, y, z, rot[0], rot[1], rot[2], rot[3])
        if status == 0:
            logger.stops()
            #return 'success'
        else:
            logger.stopf()
            #return 'failure'

        rospy.loginfo('Executing state DumpBowl')
        logger = EventLoggerClient.startfsm('DumpBowl (deal bowl fsm)')
        whicharm = 1 
        self.arm_client.move_delta_const_ee(False, -.1, 'z')
        self.arm_client.move_delta_const_ee(False, -.1, 'z')
        self.arm_client.move_delta_const_ee(False, -.1, 'z')
        self.pick_and_place_manager.open_gripper(whicharm)
        self.pick_and_place_manager.detach_object(whicharm)
        self.pick_and_place_manager.reset_collision_map()
        status = self.pick_and_place_manager.move_arm_to_side_open_loop(whicharm)
        self.base_client.translate(False, -1 * abs(dely))
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
                return 'failure'

    def get_name_str(self):
        return 'dump'

    def get_params_str(self):
        return '(?x ?y)'

    def get_precondition_str(self):
		return '(and (INGRED ?x) (GRIPPER ?y) (carry ?y ?x) (not (inverted ?x)) (in-transit ?x))'

    def get_effect_str(self):
		return '(and (done ?x) (not (carry ?y ?x)) (not (in-transit ?x)) (free ?y))'

class DealResetArm(smachforward.FState):
#TODO: will want to do this with only one arm based on params
    def __init__(self):
        smachforward.FState.__init__(self)

    def execute(self, userdata):
        logger = EventLoggerClient.startfsm('ResetBothArms (deal bowl sffsm)')
        rospy.loginfo('Executing state ResetBothArms')
        papm = PickAndPlaceManager.get_pick_and_place_manager()

        rospy.loginfo('switching both arms to cartesian control')
        client = PR2CMClient.get_pr2cm_client()
        print client.load_cartesian(True)
        print client.load_cartesian(False)

        rospy.loginfo('moving the right arm to the side')
        papm.move_arm_to_side(0)
        rospy.loginfo('moving the left arm to the side')
        papm.move_arm_to_side(1)

        rospy.loginfo('closing both grippers')
        papm.close_gripper(0)
        papm.close_gripper(1)
        rospy.loginfo('done with robot hardware position initialization')
        logger.stops()
        return 'success'

    def get_name_str(self):
        return 'resetarm'

    def get_params_str(self):
        return '(?x)'

    def get_precondition_str(self):
		return '(and (GRIPPER ?x) (free ?x))'

    def get_effect_str(self):
		return '(reset ?x)'

if __name__ == '__main__':
    rospy.init_node('testingstates')
    substates = list()
    substates.append(DealGrab())
    substates.append(DealLift())
    substates.append(DealRotateToKnown())
    substates.append(DealTransit())
    substates.append(DealPour())
    substates.append(DealRevert())
    substates.append(DealDump())
    substates.append(DealResetArm())
    domain_str, predicate_str = ff_utils.generate_domain_str('test', substates)
    #problem_file = open('deal_bowl_problem.pddl', 'r')
    #problem_str = problem_file.read()
    pe = DealBowlPredicateEstimator(domain='test')
    problem_str = pe.generate_problem_str(None, predicate_str,'(:goal (and (poured-out ingred) (reset gripper)))')
    pv = PredicateViewer.get_predicate_viewer()
    pv.add_estimator(pe, predicate_str)
    pv.view_predicates(None)
    ff_output = ff_utils.execute_ff(domain_str, problem_str)
    print '\n\nprocessed ff output:'
    for out in ff_output:
        print out

