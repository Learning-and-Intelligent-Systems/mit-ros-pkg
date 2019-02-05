#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
from smachforward import *
import rospy
import tf
import sys
from clients.unified_tf_client import UnifiedTFClient
import actionlib
from clients.arm_client import ArmClient
from clients.base_client import BaseClient
from utilities.bakebot_pick_and_place_manager import *
from utilities.broad_table_object_manager import *
from utilities.joint_recorder_player import *
from clients.pr2cm_client import *
from clients.mixing_client import *
from utilities.bowl_dealer_utilities import *
from bakebot.srv import *
from clients.clean_spoon_client import *
import math
import smach
import smach_ros
import time
import pickle


class ScrapeGrab(FState):
    def __init__(self, debug = False):
        FState.__init__(self, 
                                   input_keys=['cookie_sheet', 'mixing_bowl', 'is_right_arm', 'table_frame_x_offset', 'table_frame_y_offset', 'table_frame_x_offset'],
                                   output_keys=['current_known_pose_name', 'cookie_sheet', 'is_right_arm', 'grasp_pose', 'global_grasp_pose'])
        self.pick_and_place_manager = PickAndPlaceManager.get_pick_and_place_manager()
        self.arm_client = ArmClient.get_arm_client()
        self.broad_object_manager = BroadTabletopObjectManager.get_broad_tabletop_object_manager(self.pick_and_place_manager)

    def execute(self, userdata):
        rospy.loginfo('Executing state InitializeForScraping')
        rospy.loginfo('moving the right arm to the side')
        self.pick_and_place_manager.move_arm_to_side(0)
        rospy.loginfo('moving the left arm to the side')
        self.pick_and_place_manager.move_arm_to_side(1)
        rospy.loginfo('closing both grippers')
        self.pick_and_place_manager.close_gripper(0)
        self.pick_and_place_manager.close_gripper(1)
        self.pick_and_place_manager.find_table()
        rospy.loginfo('done with robot hardware position initialization')

        rospy.loginfo('Executing state Grab')
        whicharm, result, grasp_pose = self.broad_object_manager.refine_and_grasp(userdata.mixing_bowl.collision_name, (1,1), userdata.table_frame_x_offset, userdata.table_frame_y_offset)
        userdata.is_right_arm = False
        userdata.grasp_pose = grasp_pose
        trans, rot = self.arm_client.get_transform(userdata.is_right_arm)
        userdata.global_grasp_pose = (trans, rot)
        if result == 'succeeded':
            return 'success'
        else:
            rospy.logwarn('grasp client returned: ' + str(result))
            bc = BaseClient.get_base_client()
            bc.translate(True, -.2)
            whicharm, result, grasp_pose = self.broad_object_manager.refine_and_grasp(userdata.mixing_bowl.collision_name, (1,1), userdata.table_frame_x_offset + .2, userdata.table_frame_y_offset)
            bc.translate(True, .2)
            if result == 'succeeded':
                return 'success'
            else:
                return 'failure'

    def get_name_str(self):
        return 'grab-mb'

    def get_params_str(self):
        return '(?x ?y)'

    def get_precondition_str(self):
		return '(and (MIXING-BOWL ?x) (GRIPPER ?y) (not (SPATULA-GRIPPER ?y)) (reset ?y) (free ?y))'

    def get_effect_str(self):
		return '(and (carry ?y ?x) (not (free ?y)) (not (reset ?y)) (not (reset-safe ?y)))'

class ScrapeLift(FState):
    def __init__(self):
        FState.__init__(self, 
                             input_keys=['acceptable_height_above_table', 'table_height'],
                             output_keys=['current_height_above_table'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener

    def execute(self, userdata):
        rospy.loginfo('Executing state ChangeGripperObjHeight')
        time.sleep(1)
        is_right_arm = False
        papm = PickAndPlaceManager.get_pick_and_place_manager()
        (trans, rot) = self.arm_client.get_transform(is_right_arm)
        desired_z = userdata.acceptable_height_above_table + papm.table_height
        rospy.loginfo('lifting the arm to desired z: ' + str(desired_z))
        status = self.arm_client.move_to_pose_const_ee(is_right_arm, trans[0], trans[1], desired_z)
        if status == 0 or status == -2:
            return 'success'
        else:
            return 'failure'

    def get_name_str(self):
        return 'lift'

    def get_params_str(self):
        return '(?x ?y)'

    def get_precondition_str(self):
		return '(and (MIXING-BOWL ?x) (GRIPPER ?y) (carry ?y ?x))'

    def get_effect_str(self):
		return '(and (in-transit ?x) (not (on-table ?x)) (randomp ?x))'

class ScrapeRotate(FState):
    def __init__(self):
        FState.__init__(self, 
                             input_keys=['grasp_pose', 'bowl_to_deal', 'mixing_bowl'],
                             output_keys=['current_known_pose_name', 'hand_poses_exhausted', 'use_new_pose_manager'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener

    def execute(self, userdata):
        rospy.loginfo('Executing state RotateToKnownPose')
        logger = EventLoggerClient.startfsm('RotateToKnownPose (deal bowl fsm)')
        self.pose_manager = GripperBowlPoseManager(userdata.mixing_bowl, self.arm_client, True, is_pre_scrape = True)
        #pose, name_of_pose = self.pose_manager.get_current_recommended_pose()
        # I may be missing out on the first one if I don't call current here
        # if that's the case then just use the current here and call next right before returning failure
        rospy.loginfo('getting the next recommended pose')
        #pose, name_of_pose = self.pose_manager.get_next_recommended_pose()
        pose, name_of_pose = self.pose_manager.get_current_recommended_pose()

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
            (trans, rot) = self.arm_client.get_transform(False)
            status = self.arm_client.move_to_pose(False, False, trans[0], trans[1], trans[2], pose[0], pose[1], pose[2], pose[3])
            print status
            if status == 0:
                rospy.loginfo('success')
                userdata.current_known_pose_name = name_of_pose
                logger.stops()
                return 'success'
            rospy.logwarn('rotating the end effector failed.  going to next preferred pose by recursing via FSM')
            userdata.current_known_pose_name = 'unknown'
            logger.stopf()
            return 'failure'

    def get_name_str(self):
        return 'rotate-to-known'

    def get_params_str(self):
        return '(?x ?y)'

    def get_precondition_str(self):
		return '(and (MIXING-BOWL ?x) (GRIPPER ?y) (in-transit ?x) (not (out-of-way ?x)) (carry ?y ?x))'

    def get_effect_str(self):
		return '(and (not (randomp ?x)) (cardinal ?x))'

class ScrapeServoToCS(FState):
    def __init__(self):
        FState.__init__(self, 
                                   input_keys=['cookie_sheet', 'table_frame_x_offset', 'table_frame_y_offset', 'nominal_cookie_sheet_x', 'nominal_cookie_sheet_y', 'table_front_edge_x'],
                                   output_keys=['cookie_sheet', 'table_frame_x_offset', 'table_frame_y_offset', 'table_front_edge_x'])

    def execute(self, userdata):
        rospy.loginfo('Executing state LocateCookieSheet')
        x = userdata.cookie_sheet.pose.pose.position.x
        y = userdata.cookie_sheet.pose.pose.position.y
        z = userdata.cookie_sheet.pose.pose.position.z
        rospy.loginfo('\n\npose of cookie_sheet bowl: ' + str((x,y,z)) + '\n\n')
        rospy.loginfo('replacing the pose of userdata.cookie_sheet with the pose from the most recent detection')
        btom = BroadTabletopObjectManager.get_broad_tabletop_object_manager(PickAndPlaceManager.get_pick_and_place_manager())
        detected_cookie_sheet = btom.refine_and_and_get_graspable_object(userdata.cookie_sheet.collision_name, userdata.table_frame_x_offset, userdata.table_frame_y_offset)
        if detected_cookie_sheet == None:
            rospy.logerr('could not find the cookie_sheet bowl!')
            return 'failure'
        userdata.cookie_sheet.pose = detected_cookie_sheet.pose
        x = userdata.cookie_sheet.pose.pose.position.x
        y = userdata.cookie_sheet.pose.pose.position.y
        z = userdata.cookie_sheet.pose.pose.position.z
        print '\n\n'
        rospy.loginfo('corrected pose of cookie_sheet bowl: ' + str((x,y,z)))
        print '\n\n'

        ROBOT_FRONT_X = 0.2 
        TABLE_MARGIN = 0.04
        rospy.loginfo('Executing state ScrapingMoveRobot')
        bc = BaseClient.get_base_client()
        actual_x = userdata.cookie_sheet.pose.pose.position.x
        actual_y = userdata.cookie_sheet.pose.pose.position.y
        print '\n\n*****************************************'
        print 'cookie sheet pose'
        print userdata.cookie_sheet.pose.pose.position.x
        print userdata.cookie_sheet.pose.pose.position.y
        print userdata.cookie_sheet.pose.pose.position.z
        print 'desired cookie sheet pose'
        print userdata.nominal_cookie_sheet_x
        print userdata.nominal_cookie_sheet_y
        print '*****************************************\n'
        desired_x = userdata.nominal_cookie_sheet_x
        desired_y = userdata.nominal_cookie_sheet_y

        drive_x = actual_x - desired_x
        drive_y = actual_y - desired_y

        rospy.loginfo('DESIRED DRIVE X: ' + str(drive_x))
        rospy.loginfo('DESIRED DRIVE Y: ' + str(drive_y))

        bc.translate(True, drive_x)
        bc.translate(False, drive_y)

        userdata.table_frame_x_offset = drive_x  #TODO need to reverse this
        userdata.table_frame_y_offset = drive_y
        print '********** xoff', userdata.table_frame_x_offset
        print '********** yoff', userdata.table_frame_y_offset

        rospy.loginfo('Executing state LocateCookieSheet')
        x = userdata.cookie_sheet.pose.pose.position.x
        y = userdata.cookie_sheet.pose.pose.position.y
        z = userdata.cookie_sheet.pose.pose.position.z
        rospy.loginfo('\n\npose of cookie_sheet bowl: ' + str((x,y,z)) + '\n\n')
        rospy.loginfo('replacing the pose of userdata.cookie_sheet with the pose from the most recent detection')
        btom = BroadTabletopObjectManager.get_broad_tabletop_object_manager(PickAndPlaceManager.get_pick_and_place_manager())
        detected_cookie_sheet = btom.refine_and_and_get_graspable_object(userdata.cookie_sheet.collision_name, userdata.table_frame_x_offset, userdata.table_frame_y_offset)
        if detected_cookie_sheet == None:
            rospy.logerr('could not find the cookie_sheet bowl!')
            return 'failure'
        userdata.cookie_sheet.pose = detected_cookie_sheet.pose
        x = userdata.cookie_sheet.pose.pose.position.x
        y = userdata.cookie_sheet.pose.pose.position.y
        z = userdata.cookie_sheet.pose.pose.position.z
        print '\n\n'
        rospy.loginfo('corrected pose of cookie_sheet bowl: ' + str((x,y,z)))
        print '\n\n'
        return 'success'

    def get_name_str(self):
        return 'servo-to-cs'

    def get_params_str(self):
        return '(?x ?y)'

    def get_precondition_str(self):
		return '(and (COOKIE-SHEET ?x) (MIXING-BOWL ?y) (not (cs-zone ?x)) (out-of-way ?y))'

    def get_effect_str(self):
		return '(cs-zone ?x)'

class ScrapeServoFromCS(FState):
    def __init__(self):
        FState.__init__(self, 
                                   input_keys=['cookie_sheet', 'table_frame_x_offset', 'table_frame_y_offset', 'nominal_cookie_sheet_x', 'nominal_cookie_sheet_y', 'table_front_edge_x'],
                                   output_keys=['cookie_sheet', 'table_frame_x_offset', 'table_frame_y_offset', 'table_front_edge_x'])

    def execute(self, userdata):
        ROBOT_FRONT_X = 0.2 # TODO figure out this number for real!
        TABLE_MARGIN = 0.04
        rospy.loginfo('Executing state ScrapingMoveRobot')
        bc = BaseClient.get_base_client()
        drive_x = -userdata.table_frame_x_offset
        drive_y = -userdata.table_frame_y_offset
        rospy.loginfo('DESIRED DRIVE X: ' + str(drive_x))
        rospy.loginfo('DESIRED DRIVE Y: ' + str(drive_y))

        bc.translate(True, drive_x)
        bc.translate(False, drive_y)

        userdata.table_frame_x_offset = 0
        userdata.table_frame_y_offset = 0
        return 'success'

    def get_name_str(self):
        return 'servo-from-cs'

    def get_params_str(self):
        return '(?x)'

    def get_precondition_str(self):
		return '(and (COOKIE-SHEET ?x) (cs-zone ?x))'

    def get_effect_str(self):
		return '(not (cs-zone ?x))'

class ScrapeTransitAside(FState):
    def __init__(self, interpolate = True):
        FState.__init__(self, 
                             input_keys=['cookie_sheet', 'pour_height_above_centroid', 'current_known_pose_name', 'mixing_bowl_radius'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener
        self.interpolate = interpolate

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveBowlOutofWay')
        (trans, rot) = self.arm_client.get_transform(False)
        x = .5
        y = .5
        z = trans[2]
        if self.interpolate:
            status = self.arm_client.move_to_pose_const_ee(False, x, y, z, determination = 1)
        else:
            status = self.arm_client.move_to_pose(False, False, x, y, z, rot[0], rot[1], rot[2], rot[3])
        print 'move bowl out of way status: ', status
        if status == 0:
            return 'success'
        else:
            rospy.logwarn('moving bowl out of way failed')
            if not self.interpolate:
                rospy.loginfo('trying again without interpolation')
                status = self.arm_client.move_to_pose(False, False, x, y, z, rot[0], rot[1], rot[2], rot[3])
                if status == 0:
                    return 'success'
            return 'failure'

    def get_name_str(self):
        return 'transit-aside'

    def get_params_str(self):
        return '(?x ?y)'

    def get_precondition_str(self):
		return '(and (MIXING-BOWL ?x) (GRIPPER ?y) (carry ?y ?x) (in-transit ?x))'

    def get_effect_str(self):
		return '(and (out-of-way ?x))'

class ScrapeTransit(FState):
    def __init__(self):
        FState.__init__(self, 
                             input_keys=['cookie_sheet', 'pour_height_above_centroid', 'current_known_pose_name', 'mixing_bowl_radius'],
                             output_keys=['current_known_pose_name'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveToPrePourPosition')

        cookie_sheet_pose = userdata.cookie_sheet.pose.pose.position
        x_pos = cookie_sheet_pose.x
        y_pos = cookie_sheet_pose.y
        z_pos = cookie_sheet_pose.z
        z_pos = z_pos + userdata.pour_height_above_centroid
        # need to figure out where to move to for the pre pose based on the current pose
        if userdata.current_known_pose_name == 'unknown':
            rospy.logwarn('unknown current pose, need to rotate to a known position to calculate pre pour position')
            # going to try to match here
            isRightArm = False
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
                return self.execute(userdata)
            else:
                print 'could not determine pose'
                return 'failure'
        else:
            try:
                dx, dy, dz = GripperBowlPoseManager.POUR_ADJUSTMENT_DIRS_FROM_HIGH_CENTROID[userdata.current_known_pose_name]
            except KeyError:
                rospy.logwarn('could not find a pour adjustment for ' + str(userdata.current_known_pose_name))
                rospy.logerr('there are no more positions that I can rotate to to replan from')
                rospy.logerr('I cannot recover fromt hsi failure, failing with exhastion')
                return 'failure'
            rospy.loginfo('\n\n\nPOUR INFO ********************************************************')
            print 'mixing_bowl radius: ', userdata.mixing_bowl_radius
            print 'deltas: ', dx, dy, dz
            print x_pos
            print y_pos
            print z_pos
            x_pos = x_pos + dx * userdata.mixing_bowl_radius
            y_pos = y_pos + dy * userdata.mixing_bowl_radius
            z_pos = z_pos + dz * userdata.mixing_bowl_radius
            print x_pos
            print y_pos
            print z_pos
            x_plus_minus = (0.10, 0.10)
            y_plus_minus = (0.10, 0.10)
            z_plus_minus = (0.1, 0.0)
            print x_plus_minus
            print y_plus_minus
            print z_plus_minus
            
            status = self.arm_client.move_to_pose_const_ee(False, x_pos, y_pos, z_pos, 1, x_plus_minus, y_plus_minus, z_plus_minus, 1)
            if status == 0:
                rospy.loginfo('succeeded')
                return 'success'
            else:
                rospy.logwarn('failed to plan move to pre pour position')
                rospy.logerr('failed to plan transit from this rotation.  cannot recover from this in scrape')
                return 'failure'


    def get_name_str(self):
        return 'transit'

    def get_params_str(self):
        return '(?x ?y ?z)'

    def get_precondition_str(self):
		return '(and (MIXING-BOWL ?x) (GRIPPER ?y) (COOKIE-SHEET ?z) (carry ?y ?x) (cs-zone ?z) (not (poured-out ?x)) (in-transit ?x))'

    def get_effect_str(self):
		return '(and (over-cs ?x))'

class ScrapePour(FState):
    def __init__(self):
        FState.__init__(self, 
                             input_keys = [ 'current_known_pose_name'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener
        self.failed_once = False

    def execute(self, userdata):
        rospy.loginfo('Executing state Pour')
        if userdata.current_known_pose_name == 'unknown':
            rospy.logwarn('unknown current pose, need to rotate to a known position before pouring')
            rospy.logwarn('unknown current pose, need to rotate to a known position to calculate pre pour position')
            # going to try to match here
            isRightArm = False
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
                return self.execute(userdata)
            else:
                print 'could not determine pose'
            return 'failure'
        px, py, pz, pw = GripperBowlPoseManager.POUR_POSES[userdata.current_known_pose_name]

        print '\n\n\n'
        print 'originally wanted: ', px, py, pz, pw
        npx = .6
        npy = .4
        npz = .5
        npw = .4
        print 'now I want: ', npx, npy, npz, npw
        print '\n\n\n'
        (trans, rot) = self.arm_client.get_transform(False)
        x = trans[0] - .04
        y = trans[1] - .02
        z = trans[2] - .05
        print '**********8 driving backwards to prevent spilling when scraping'
        bc = BaseClient.get_base_client()
        bc.translate(True, -.02)
        status = self.arm_client.move_to_pose(False, True, x, y, z - .06, npx, npy, npz, npw, override_cmpapm = True) # used to have this one then the other one (both enabled)
        status = self.arm_client.move_to_pose(False, True, x, y, z, npx, npy, npz, npw, override_cmpapm = True)
        print '**************status: '
        if status == 0:
            rospy.loginfo('pour succeeded')
            return 'success'
        else:
            rospy.logwarn('pour failed to rotate to position')
            rospy.logwarn('pour failed to rotate to position')
            rospy.logwarn('pour failed to rotate to position')
            rospy.logwarn('pour failed to rotate to position')
            rospy.logwarn('pour failed to rotate to position')
            rospy.logwarn('pour failed to rotate to position')
            status = self.arm_client.move_to_pose(False, False, x, y, z, npx, npy, npz, npw, override_cmpapm = True)
            if status == 0:
                return 'success'
            status = self.arm_client.move_to_pose(False, False, x, y, z, npx, npy, npz, npw, override_cmpapm = True)
            if status == 0:
                return 'success'
            time.sleep(1)
            status = self.arm_client.move_to_pose(False, False, x, y, z, npx, npy, npz, npw, override_cmpapm = True)
            if status == 0:
                return 'success'
            else:
                return 'failure'

    def get_name_str(self):
        return 'pour'

    def get_params_str(self):
        return '(?x ?y)'

    def get_precondition_str(self):
		return '(and (MIXING-BOWL ?x) (GRIPPER ?y) (carry ?y ?x) (in-transit ?x) (over-cs ?x) (cardinal ?x))'

    def get_effect_str(self):
		return '(and (inverted ?x) (poured-out ?x))'

class ScrapeTransitSpoon(FState):
    def __init__(self):
        FState.__init__(self, 
                                   input_keys=['mixing_bowl', 'cookie_sheet', 'pre_mix_height_above_table', 'skipped_debug_init'],
                                   output_keys=['cookie_sheet'])
        self.arm_client = ArmClient.get_arm_client()
        self.papm = PickAndPlaceManager.get_pick_and_place_manager()
        self.fail_count = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state AddSpoonToHandCollisionSpace')
        rospy.loginfo('waiting for bakebot attach/detach service')
        rospy.wait_for_service('bakebot_attach_detach_service')
        rospy.loginfo('done waiting')
        isRightArm = True
        spoon_length = 0.24 # worked with .2
        spoon_radius = 0.02
        try:
            attach_detach = rospy.ServiceProxy('bakebot_attach_detach_service', AttachDetach)
            response = attach_detach(isRightArm, 1, spoon_length, spoon_radius)
            rospy.loginfo('service responsed with: ' + str(response))

            print '\n\n\n\n\n', 'spoon attached'
            #raw_input('pres enter to continue.........................................................')
        except rospy.ServiceException, e:
            rospy.logerr('service call failed')
            print e
            return 'failure'

        x = userdata.cookie_sheet.pose.pose.position.x
        y = userdata.cookie_sheet.pose.pose.position.y - .4 # TODO: configure these parameters
        z = self.papm.table_height + userdata.pre_mix_height_above_table # will want to make this the same z as bowl centroid
        print 'old z: ', z 
        z = .9
        print 'new z: ', z
        rospy.loginfo('Advancing the right arm first')
        self.arm_client.advance_arm(True)
        path = os.environ['BAKEBOT_JRPLOG_PATH']
        f = path + '2011-07-25-prescrapejrp.log'
        jrp = JointRecorderPlayer()
        jrp.load_saved_states_from_file(f)
        jrp.playback_all_joints(automatic = True)
        status = self.arm_client.move_to_pose(True, False, x, y, z, .5, 0.5, .5, .5, collision_aware = 0, override_cmpapm=True)
        if status is not 0:
            rospy.loginfo('Executing state RemoveSpoonFromHandCollisionSpace')
            rospy.loginfo('waiting for bakebot attach/detach service')
            rospy.wait_for_service('bakebot_attach_detach_service')
            rospy.loginfo('done waiting')
            isRightArm = True
            spoon_length = 0.2
            spoon_radius = 0.01
            try:
                attach_detach = rospy.ServiceProxy('bakebot_attach_detach_service', AttachDetach)
                response = attach_detach(isRightArm, 0, spoon_length, spoon_radius)
                rospy.loginfo('service responsed with: ' + str(response))
            except rospy.ServiceException, e:
                rospy.logerr('service call failed')
                print e
                return 'failure'
            status = self.arm_client.move_to_pose(True, False, x, y, z, .5, 0.5, .5, .5, collision_aware = 0, override_cmpapm=True)
            if status is not 0:
                return 'failure'
            else:
                return 'success'
        return 'success'

    def get_name_str(self):
        return 'transit-spoon'

    def get_params_str(self):
        return '(?x ?y)'

    def get_precondition_str(self):
		return '(and (GRIPPER ?y) (MIXING-BOWL ?x) (SPATULA-GRIPPER ?y) (poured-out ?x))'

    def get_effect_str(self):
		return '(and (over-cs ?y) (not (reset ?y)) (not (reset-safe ?y)))'

class ScrapePlungeSpoon(FState):
    def __init__(self):
        FState.__init__(self, 
                                   input_keys=['mixing_bowl', 'cookie_sheet', 'pre_mix_height_above_table'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PlungeSpoon')
        x = userdata.cookie_sheet.pose.pose.position.x
        z = .90 #TODO: will want to change this to be less fragile
        tfl = UnifiedTFClient.get_unified_tf_client()
        try:
            (trans, rot) = tfl.lookupTransform('/base_link', '/torso_lift_link', rospy.Time(0)) 
        except Exception as e:
            print e
            print 'burning off a bad tf reading and waiting'
            time.sleep(1)
            (trans, rot) = tfl.lookupTransform('/base_link', '/torso_lift_link', rospy.Time(0)) 

        if rot is not (0.0,0.0,0.0,1.0):
            rospy.logerr('nonzero rotation between base link and torso lift link!')
            print rot
        print '\n\n*******************************************************************'
        print trans
        print x
        print z
        print rot
        x = x - trans[0]
        z = z - trans[2]
        print  'nx',x
        print  'nz',z
        print '*******************************************************************\n\n'
        mc = MixingClient.get_mixing_client()
        status = mc.plunge_spoon(bowl_x = x, bowl_z = z, ismix = False)
        return 'success' if status else 'failure'

    def get_name_str(self):
        return 'plunge-spoon'

    def get_params_str(self):
        return '(?y)'

    def get_precondition_str(self):
		return '(and (GRIPPER ?y) (SPATULA-GRIPPER ?y) (over-cs ?y))'

    def get_effect_str(self):
		return '(and (not (over-cs ?y)) (in-mb ?y))'

class ScrapeSpoon(FState):
    def __init__(self):
        FState.__init__(self, 
                                   input_keys=['cookie_sheet']) 
        self.num_mixes_done = 0
        self.MAX_MIXES = 4  #TODO change this

    def execute(self, userdata):
        rospy.loginfo('Executing state AssessScrape')
        tf = UnifiedTFClient.get_unified_tf_client()
        (trans, rot) = tf.lookupTransform('torso_lift_link', 'r_wrist_roll_link', rospy.Time(0))
        starty = trans[1]
        for i in range(self.MAX_MIXES):
            rospy.loginfo('Executing state Scrape')
            xoff = 0
            zstartoff1 = 0
            if i == 1:
                xoff = .08
            elif i == 2:
                xoff = -.08
            elif i == 3:
                zstartoff1 = .1
            (trans, rot) = tf.lookupTransform('torso_lift_link', 'r_wrist_roll_link', rospy.Time(0))
            yoffset = -1*(starty - trans[0])
            csc = CleaningClient.get_cleaning_client()
            #csc.scrape(xmove = xoff, zstartoff = zstartoff1, yoff=yoffset)
#TODO: something is very wrong here (1/26/11).  the hand backs out further and further for every scrape action
# when there is no spoon.  maybe have to fix this.
            csc.scrape(xmove = xoff, zstartoff = zstartoff1)
           #print 'now going to plunge spoon'
           #mc = MixingClient.get_mixing_client()
           #x = userdata.cookie_sheet.pose.pose.position.x
           #z = .90 #TODO: will want to change this to be less fragile
           #tfl = UnifiedTFClient.get_unified_tf_client()
           #try:
           #    (trans, rot) = tfl.lookupTransform('/base_link', '/torso_lift_link', rospy.Time(0)) 
           #except Exception as e:
           #    print e
           #    print 'burning off a bad tf reading and waiting'
           #    time.sleep(1)
           #    (trans, rot) = tfl.lookupTransform('/base_link', '/torso_lift_link', rospy.Time(0)) 
           #if rot is not (0.0,0.0,0.0,1.0):
           #    rospy.logerr('nonzero rotation between base link and torso lift link!')
           #    print rot
           #x = x - trans[0]
           #print  'nx',x
           #status = mc.plunge_spoon(bowl_x = x, bowl_z = z, ismix = False)
        return 'success'

    def get_name_str(self):
        return 'scrape'

    def get_params_str(self):
		return '(?left ?right ?mb)'

    def get_precondition_str(self):
		return '(and (GRIPPER ?left) (not (SPATULA-GRIPPER ?left)) (GRIPPER ?right) (SPATULA-GRIPPER ?right) (MIXING-BOWL ?mb) (carry ?left ?mb) (in-mb ?right))'

    def get_effect_str(self):
		return '(scraped ?mb)'

class ScrapeDeplungeSpoon(FState):
    def __init__(self):
        FState.__init__(self, 
                                   input_keys=['mixing_bowl', 'pre_mix_height_above_table']) 

    def execute(self, userdata):
        rospy.loginfo('executing state DeplungeSpoon')
        mc = MixingClient.get_mixing_client()
        tfl = tf.TransformListener()
        try:
            (trans, rot) = tfl.lookupTransform('/base_link', '/torso_lift_link', rospy.Time(0)) # TODO: this may be reversed
        except Exception as e:
            print e
            print 'burning off a bad tf reading and waiting'
            time.sleep(1)
            (trans, rot) = tfl.lookupTransform('/base_link', '/torso_lift_link', rospy.Time(0)) # TODO: this may be reversed

        if rot is not (0.0,0.0,0.0,1.0):
            rospy.logerr('nonzero rotation between base link and torso lift link!')
            print rot
        print trans
        print rot
        bx = userdata.mixing_bowl.pose.pose.position.x
        by = userdata.mixing_bowl.pose.pose.position.y
        bz = userdata.mixing_bowl.pose.pose.position.z

        x = bx - trans[0]  #TODO: check this
        y = by - trans[1]
        z = bz - trans[2]
        print 'position in tll: ', x, y, z
        
        bowl_xyz_tllf = (x,y,z)

        if mc.deplunge_spoon(bowl_xyz_tllf, userdata.pre_mix_height_above_table, ismix = False):
            return 'success'
        else:
            rospy.logerr('deplunge failure')
            return 'failure'

    def get_name_str(self):
        return 'deplunge-spoon'

    def get_params_str(self):
        return '(?y)'

    def get_precondition_str(self):
		return '(and (GRIPPER ?y) (SPATULA-GRIPPER ?y) (in-mb ?y))'

    def get_effect_str(self):
		return '(and (not (in-mb ?y)) (over-cs ?y))'

class ScrapeRetreatSpoon(FState):
    def __init__(self):
        FState.__init__(self)

    def execute(self, userdata):
        rospy.loginfo('Executing state DumpSpoon')
        arm_client = ArmClient.get_arm_client()
        arm_client.retreat_arm(True)
        return 'success'

    def get_name_str(self):
        return 'retreat-spoon'

    def get_params_str(self):
        return '(?y)'

    def get_precondition_str(self):
		return '(and (GRIPPER ?y) (SPATULA-GRIPPER ?y))'

    def get_effect_str(self):
		return '(and (not (over-cs ?y)) (free ?y) (reset-safe ?y))'

class ScrapeRevert(FState):
    def __init__(self):
        FState.__init__(self, 
                             input_keys = ['current_known_pose_name'],
                             output_keys =['current_known_pose_name'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener

    def execute(self, userdata):
        isRightArm = False
        rospy.loginfo('Executing state Revert')
        logger = EventLoggerClient.startfsm('Revert (scraping fsm)')
        if userdata.current_known_pose_name == 'unknown':
            rospy.logwarn('unknown current pose name, cannot rotate back to it')
            rospy.logwarn('unknown current pose, need to rotate to a known position to calculate pre pour position')
            # going to try to match here
            isRightArm = False
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
                return self.execute(userdata)
            else:
                print 'could not determine pose'
            logger.stopf()
            return 'success'
        px, py, pz, pw = GripperBowlPoseManager.ALL_POSES[userdata.current_known_pose_name]
        rospy.loginfo('first moving the bowl out of the way')
        status = self.arm_client.move_delta_const_ee(False, .4, 'y') 
        failed = False
        if status is not 0:
            failed = True
            rospy.logwarn('failed but going to try for less')
            status = self.arm_client.move_delta_const_ee(False, .2, 'y') 
        (trans, rot) = self.arm_client.get_transform(isRightArm)
        status = self.arm_client.move_to_pose(isRightArm, False, trans[0], trans[1], trans[2], px, py, pz, pw)
        if failed:
            status = self.arm_client.move_delta_const_ee(False, .2, 'y') 
            status = self.arm_client.move_delta_const_ee(False, .1, 'z') 
            status = self.arm_client.move_delta_const_ee(False, .2, 'y') 
            status = self.arm_client.move_delta_const_ee(False, .1, 'z') 
            status = self.arm_client.move_delta_const_ee(False, .1, 'y') 
        print '3333333333333333333333333333333333333'
        if status == 0:
            rospy.loginfo('revert succeeded')
            logger.stops()
            return 'success'
        else:
            rospy.logwarn('pour failed to revert to position')
            logger.stopf()
            #return 'failure'
            return 'success'

    def get_name_str(self):
        return 'revert'

    def get_params_str(self):
        return '(?x ?y ?z)'

    def get_precondition_str(self):
		return '(and (MIXING-BOWL ?x) (GRIPPER ?y) (SPATULA-GRIPPER ?z) (reset-safe ?z) (inverted ?x) (carry ?y ?x) (in-transit ?x))'

    def get_effect_str(self):
		return '(and (not (inverted ?x)) (cardinal ?x))'

class ScrapeDump(FState):
    def __init__(self):
        FState.__init__(self, 
                             input_keys = ['bowl_to_deal', 'table_height', 'acceptable_height_above_table'])
        self.arm_client = ArmClient.get_arm_client()
        self.pick_and_place_manager = PickAndPlaceManager.get_pick_and_place_manager()
        self.tf_listener = self.arm_client.tf_listener
        self.fail_count = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state ReturnTransitBowl')
        is_right_arm = False
        #logger = EventLoggerClient.startfsm('ReturnTransitBowl (deal bowl fsm)')
        #trans, rot = self.arm_client.get_transform(False)
        #pose = userdata.bowl_to_deal.pose.pose
        #trans_orig = (.1, .6, pose.position.z)
        #rot_orig = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        #xo, yo, zo = trans_orig
        #zo = userdata.table_height + userdata.acceptable_height_above_table
        #xrang = (xo - .1, xo + .1)
        #yrang = (yo - .1, yo + .1)
        #zrang = (zo, zo)
        #quater_xyzw = rot
        #dz = .25
        #status = self.arm_client.move_to_pose(False, False, trans[0], trans[1], trans[2] + dz, rot[0], rot[1], rot[2], rot[3])
        #if status == 0:
            #logger.stops()
            ##return 'success'
        #else:
            #logger.stopf()
            ##return 'fail'

        rospy.loginfo('Executing state ReturnTransitBowl2')
        logger = EventLoggerClient.startfsm('ReturnTransitBowl2 (deal bowl fsm)')
        trans, rot = self.arm_client.get_transform(is_right_arm)
        # TODO figure out the right position for this
        x = .5
        y = .93
        z = 1.13 
        dely = .15
        y = y - dely 
        rospy.loginfo('cutting down the y so to account for the shimmy: ' + str(y))
        self.base_client = BaseClient.get_base_client()
        status = self.base_client.translate(False, 1*abs(dely))
        status = self.arm_client.move_to_pose(is_right_arm, False, x, y, z, rot[0], rot[1], rot[2], rot[3])
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
        status = self.base_client.translate(False, -1 * abs(dely))
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
		return '(and (MIXING-BOWL ?x) (GRIPPER ?y) (not (inverted ?x)) (carry ?y ?x) (in-transit ?x))'

    def get_effect_str(self):
		return '(and (done ?x) (not (carry ?y ?x)) (not (in-transit ?x)) (reset-safe ?y) (free ?y))'

class ScrapeResetArm(FState):
    def __init__(self, name=None):
        FState.__init__(self)
        self.name = name
        self.isRightArm = None
        self.papm = PickAndPlaceManager.get_pick_and_place_manager()

    def execute(self, userdata):
        rospy.loginfo('Executing state ResetArm')
        logger = EventLoggerClient.startfsm('ResetArm (mixing fsm)')
        client = PR2CMClient.get_pr2cm_client()
        if self.isRightArm is None:
            rospy.logwarn('did not parametrize which arm')
            return 'failure'
        if self.isRightArm:
            rospy.loginfo('moving the RIGHT arm to the side')
            self.papm.move_arm_to_side(0)
            self.papm.close_gripper(0)
            print client.load_cartesian(True)
        else:
            rospy.loginfo('moving the LEFT arm to the side')
            self.papm.move_arm_to_side(1)
            self.papm.close_gripper(1)
            print client.load_cartesian(False)
        logger.stops()
        return 'success'

    def get_name_str(self):
        return 'resetarm'

    def get_params_str(self):
        return '(?x)'

    def get_precondition_str(self):
		return '(and (GRIPPER ?x) (free ?x) (reset-safe ?x))'

    def get_effect_str(self):
		return '(reset ?x)'

    def get_name_str(self):
        return 'reset-arm'

    def get_params_str(self):
        return '(?x)'

    def get_precondition_str(self):
		return '(and (GRIPPER ?x) (reset-safe ?x) (free ?x))'

    def get_effect_str(self):
		return '(reset ?x)'

    def parametrize(self, ff_line):
        self.isRightArm = (ff_line[0] == 'RARM')
        print 'parametrizing ', self.name,'based on: ', ff_line, 'isrightarm', self.isRightArm
        self.parametrized = True
        return True
