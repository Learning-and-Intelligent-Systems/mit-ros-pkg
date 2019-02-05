#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
import rospy
import tf
import sys
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
from utilities.joint_recorder_player import *

class InitializeForScraping(smach.State):
    def __init__(self, debug = False):
        self.debug = debug
        if self.debug:
            smach.State.__init__(self, outcomes=['done', 'debug done', 'debug bypass'],
                                       input_keys=['cookie_sheet'],
                                       output_keys=['skipped_debug_init', 'current_known_pose_name', 'cookie_sheet'])
        else:
            smach.State.__init__(self, outcomes=['done'],
                                       input_keys=['cookie_sheet'],
                                       output_keys=['skipped_debug_init', 'current_known_pose_name', 'cookie_sheet'])
        self.executed_once = False

    def execute(self, userdata):
        rospy.loginfo('Executing state InitializeForScraping')
        if not self.executed_once:
            scrapinglogger = EventLoggerClient('milestone', annotation='scraping')
            StaticLoggerLocker.check_in_logger('SCRAPING LOGGER', scrapinglogger) 
            self.executed_once = True
        if not self.debug:
            rospy.loginfo('moving the right arm to the side')
            papm = PickAndPlaceManager.get_pick_and_place_manager()
            papm.move_arm_to_side(0)
            rospy.loginfo('moving the left arm to the side')
            papm.move_arm_to_side(1)
            rospy.loginfo('closing both grippers')
            papm.close_gripper(0)
            papm.close_gripper(1)
            papm.find_table()
            rospy.loginfo('done with robot hardware position initialization')
            userdata.skipped_debug_init = True
            return 'done'
        else:
            rospy.loginfo('initializing for scraping in debug mode.')
            file  = raw_input('\n\n\nenter the filename to load joint states from (blank for scrapingjoints.log, (s)-do regular fsm, (b)ypass and go to hand switch): ')
            if len(file) == 0:
                file  = 'scrapingjoints.log'
                userdata.current_known_pose_name = 'nine_o_clock_bi'
                userdata.skipped_debug_init = False
            elif file  == 's':
                rospy.loginfo('you have indicated that you wish to skip the load.  recursing for regular init')
                self.debug = False
                userdata.skipped_debug_init = True
                return self.execute(userdata)
            elif file == 'b':
                rospy.loginfo('bypassing a bunch of code (since the robot has not moved')
                return 'debug bypass'

            print 'filename: ', file 
            jrp = JointRecorderPlayer()
            try:
                jrp.load_saved_states_from_file(filename = file)
            except Exception as e:
                print e
                rospy.logerr('somethign went wrong opening the joint states')
                return 'debug done'
            
            jrp.playback_all_joints(automatic = True)
            file  = raw_input('enter the filename to load cookie sheet information from (blank for cookiesheet.log): ')
            if len(file ) == 0:
                file  = 'cookiesheet.log'

            print userdata.cookie_sheet
            try:
                print 'trying to open: ', file
                myfile = open(file, 'rb')
                sys.setrecursionlimit(10000)
                cookie_sheet = pickle.load(myfile)
                myfile.close()
                userdata.cookie_sheet = cookie_sheet
            except Exception as e:
                print e
                rospy.logerr('somethign went wrong opening the cookie sheet')
                print userdata.cookie_sheet

            print 'userdata cookiesheet: ', userdata.cookie_sheet

            papm = PickAndPlaceManager.get_pick_and_place_manager()
            #papm.open_gripper(1)

            raw_input('**************** press enter to close the grippers')
            papm.close_gripper(1)
            papm.close_gripper(0)
            papm.held_objects[1] = userdata.cookie_sheet
            return 'debug done'

                
class CleanupLogging(smach.State):
    def __init__(self, is_success):
        smach.State.__init__(self, outcomes=['done'])
        self.is_success = is_success

    def execute(self, userdata):
        rospy.loginfo('Executing state CleanupLogging: ' + str(self.is_success))
        scrapinglogger = StaticLoggerLocker.check_out_logger('SCRAPING LOGGER')
        if self.is_success:
            scrapinglogger.stops()
        else:
            scrapinglogger.stopf()
        return 'done'

class UserKeyboardArmClient(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.arm_client = ArmClient.get_arm_client()

    def execute(self, userdata):
        rospy.loginfo('Executing state UserKeyboardArmClient')
        self.arm_client.ui_loop()
        return 'done'


# the robot will always grab with the left arm, and mix with the right arm
class GrabMixingBowl(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail', 'fail with exhaustion'], 
                             input_keys=['mixing_bowl', 'is_right_arm', 'table_frame_x_offset', 'table_frame_y_offset', 'table_frame_x_offset', 'move_over_positions_exhausted'],
                             output_keys=['is_right_arm', 'grasp_pose', 'global_grasp_pose'])
        self.pick_and_place_manager = PickAndPlaceManager.get_pick_and_place_manager()
        self.arm_client = ArmClient.get_arm_client()
        self.broad_object_manager = BroadTabletopObjectManager.get_broad_tabletop_object_manager(self.pick_and_place_manager)

    def execute(self, userdata):
        rospy.loginfo('Executing state Grab')
        whicharm, result, grasp_pose = self.broad_object_manager.refine_and_grasp(userdata.mixing_bowl.collision_name, (1,1), userdata.table_frame_x_offset, userdata.table_frame_y_offset)
        userdata.is_right_arm = (whicharm == 0)
        userdata.grasp_pose = grasp_pose
        trans, rot = self.arm_client.get_transform(userdata.is_right_arm)
        userdata.global_grasp_pose = (trans, rot)
        if result == 'succeeded':
            return 'success'
        else:
            rospy.logwarn('grasp client returned: ' + str(result))
            ##if userdata.move_over_positions_exhausted:
                #rospy.logerror('no more move over positions (they are exhausted)')
                #return 'fail with exhaustion'
            bc = BaseClient.get_base_client()
            bc.translate(True, -.2)
            whicharm, result, grasp_pose = self.broad_object_manager.refine_and_grasp(userdata.mixing_bowl.collision_name, (1,1), userdata.table_frame_x_offset + .2, userdata.table_frame_y_offset)
            bc.translate(True, .2)
            if result == 'succeeded':
                return 'success'
            else:
                return 'fail'

class ScrapeChangeGripperObjHeight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['acceptable_height_above_table', 'table_height', 'is_right_arm'],
                             output_keys=['current_height_above_table'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener

    def execute(self, userdata):
        rospy.loginfo('Executing state ChangeGripperObjHeight')
        time.sleep(1)
        papm = PickAndPlaceManager.get_pick_and_place_manager()
        (trans, rot) = self.arm_client.get_transform(userdata.is_right_arm)
        desired_z = userdata.acceptable_height_above_table + papm.table_height
        rospy.loginfo('lifting the arm to desired z: ' + str(desired_z))
        status = self.arm_client.move_to_pose_const_ee(userdata.is_right_arm, trans[0], trans[1], desired_z)
        if status == 0 or status == -2:
            return 'success'
        else:
            return 'fail'


class RotateToPrePourPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['mixing_bowl', 'grasp_pose', 'current_known_pose_name'],
                             output_keys=['grasp_pose', 'current_known_pose_name'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener
        self.pose_manager = None

    def execute(self, userdata):
        rospy.loginfo('Executing state RotateToPrePourPose')
        self.pose_manager = GripperBowlPoseManager(userdata.mixing_bowl, self.arm_client, True, is_pre_scrape = True)
        pose, name_of_pose = self.pose_manager.get_current_recommended_pose()
        status = self.arm_client.rotate_end_effector(False, pose[0], pose[1], pose[2], pose[3]) # will always be right arm
        if status == 0:
            rospy.loginfo('success')
            userdata.current_known_pose_name = name_of_pose
            return 'success'
        else:
            rospy.logwarn('rotating the end effector failed, trying with determination')
            status = self.arm_client.rotate_end_effector(False, pose[0], pose[1], pose[2], pose[3], determination=1) # will always be right arm
            if status == 0:
                rospy.loginfo('success')
                userdata.current_known_pose_name = name_of_pose
                return 'success'
            else:
                rospy.logerr('rotating the end effector failed again, ABORTING')
                userdata.current_known_pose_name = 'unknown'
                return 'fail'



class LocateCookieSheet(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fail', 'done'],
                             input_keys=['cookie_sheet',
                                         'table_frame_x_offset',
                                         'table_frame_y_offset'],
                             output_keys=['cookie_sheet'])

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
            return 'fail'
        userdata.cookie_sheet.pose = detected_cookie_sheet.pose
        x = userdata.cookie_sheet.pose.pose.position.x
        y = userdata.cookie_sheet.pose.pose.position.y
        z = userdata.cookie_sheet.pose.pose.position.z
        print '\n\n'
        rospy.loginfo('corrected pose of cookie_sheet bowl: ' + str((x,y,z)))
        print '\n\n'
        return 'done'


class ScrapingMoveRobot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'],
                                   input_keys=['cookie_sheet', 'table_frame_x_offset', 'table_frame_y_offset', 'nominal_cookie_sheet_x', 'nominal_cookie_sheet_y', 'table_front_edge_x'],
                                   output_keys=['cookie_sheet', 'table_frame_x_offset', 'table_frame_y_offset', 'table_front_edge_x'])

    def execute(self, userdata):
        ROBOT_FRONT_X = 0.2 # TODO figure out this number for real!
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

        #if drive_x > (userdata.table_front_edge_x - ROBOT_FRONT_X - TABLE_MARGIN):
            #rospy.logwarn('the desired drive_x would cause the robot to hit the front of the table ')
            #print 'actual x', actual_x
            #print 'desired x', desired_x
            #print 'drive x', drive_x
            #print 'table x', userdata.table_front_edge_x
            #print 'robot front x', ROBOT_FRONT_X
            #print 'safety margin', TABLE_MARGIN
            #rospy.logwarn('capping the drive_x value to table_front - robot_front - table_margin')
            #drive_x = userdata.table_front_edge_x - ROBOT_FRONT_X - TABLE_MARGIN

        bc.translate(True, drive_x)
        bc.translate(False, drive_y)

        userdata.table_frame_x_offset = drive_x  #TODO need to reverse this
        userdata.table_frame_y_offset = drive_y
        print '********** xoff', userdata.table_frame_x_offset
        print '********** yoff', userdata.table_frame_y_offset

        #userdata.table_front_edge_x = userdata.table_front_edge_x - drive_x
        return 'done'

class ResetRobotBasePos(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'],
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
        #userdata.table_front_edge_x = userdata.table_front_edge_x - drive_x #TODO check this...
        return 'done'

class MoveBowlOutofWay(smach.State):
    def __init__(self, interpolate = True):
        smach.State.__init__(self, outcomes=['success', 'fail'],
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
            return 'fail'

class Revert(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys = ['current_known_pose_name'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener

    def execute(self, userdata):
        isRightArm = False
        rospy.loginfo('Executing state Revert')
        logger = EventLoggerClient.startfsm('Revert (scraping fsm)')
        if userdata.current_known_pose_name == 'unknown':
            rospy.logwarn('unknown current pose name, cannot rotate back to it')
            logger.stopf()
            return 'fail'
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
            return 'fail'


class MoveToPrePourPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail', 'unknown pre pour pose'],
                             input_keys=['cookie_sheet', 'pour_height_above_centroid', 'current_known_pose_name', 'mixing_bowl_radius'])
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
            return 'unknown pre pour pose'
        else:
            try:
                dx, dy, dz = GripperBowlPoseManager.POUR_ADJUSTMENT_DIRS_FROM_HIGH_CENTROID[userdata.current_known_pose_name]
            except KeyError:
                rospy.logwarn('could not find a pour adjustment for ' + str(userdata.current_known_pose_name))
                rospy.logerr('there are no more positions that I can rotate to to replan from')
                rospy.logerr('I cannot recover fromt hsi failure, failing with exhastion')
                return 'fail'
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
                return 'fail'



class Pour(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail' ],
                             input_keys = [ 'current_known_pose_name'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener
        self.failed_once = False

    def execute(self, userdata):
        rospy.loginfo('Executing state Pour')
        if userdata.current_known_pose_name == 'unknown':
            rospy.logwarn('unknown current pose, need to rotate to a known position before pouring')
            return 'fail'
        px, py, pz, pw = GripperBowlPoseManager.POUR_POSES[userdata.current_known_pose_name]

        #px, py, pz, pw = GripperBowlPoseManager.MIXING_BOWL_STEEP_POUR_POSES[userdata.current_known_pose_name]
        #status = self.arm_client.rotate_end_effector(False, px, py, pz, pw, determination=1)
        print '\n\n\n'
        print 'originally wanted: ', px, py, pz, pw
        #print 'status: ', status
        npx = .6
        npy = .4
        npz = .5
        npw = .4
        print 'now I want: ', npx, npy, npz, npw
        print '\n\n\n'
        #status = self.arm_client.rotate_end_effector(False, npx, npy, npz, npw, determination=1)
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
            #status2 = self.arm_client.rotate_end_effector(False, npx, npy, npz, npw, determination=0, use_interpolation = False)
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
            if not self.failed_once:
                rospy.logwarn('failed but am trying one more time.')
                self.failed_once = True
                return self.execute(userdata)
            else:
                return 'fail'




class AddSpoonToHandCollisionSpace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'failed'])

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
            return 'failed'
        return 'done'

class MoveMixingHandToPreScrapePosition(smach.State):
    def __init__(self, debug = False):
        smach.State.__init__(self, outcomes=['done', 'fail', 'fail but retry', 'fail with exhaustion'], 
                                   input_keys=['mixing_bowl', 'cookie_sheet', 'pre_mix_height_above_table', 'skipped_debug_init'],
                                   output_keys=['cookie_sheet'])
        self.arm_client = ArmClient.get_arm_client()
        self.papm = PickAndPlaceManager.get_pick_and_place_manager()
        self.fail_count = 0
        self.debug = debug

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveMixingHandToPreScrapePosition')
        if (not self.debug) or userdata.skipped_debug_init:
            x = userdata.cookie_sheet.pose.pose.position.x
            y = userdata.cookie_sheet.pose.pose.position.y - .4 # TODO: configure these parameters
            z = self.papm.table_height + userdata.pre_mix_height_above_table # will want to make this the same z as bowl centroid
            print 'old z: ', z 
            z = .9
            print 'new z: ', z
            rospy.loginfo('Advancing the right arm first')
            if self.fail_count == 0:
                self.arm_client.advance_arm(True)
                path = os.environ['BAKEBOT_JRPLOG_PATH']
                f = path + '2011-07-25-prescrapejrp.log'
                jrp = JointRecorderPlayer()
                jrp.load_saved_states_from_file(f)
                jrp.playback_all_joints(automatic = True)
            status = self.arm_client.move_to_pose(True, False, x, y, z, .5, 0.5, .5, .5, collision_aware = 0, override_cmpapm=True)
            if status is not 0:
                self.fail_count = self.fail_count + 1
                if self.fail_count >= 3:
                    rospy.logerr('moving to the pose failed even when detached. failing with exhaustion!')
                    return 'fail with exhaustion'
                elif self.fail_count == 1:
                    rospy.logerr('moving to the pose failed... trying again')
                    return 'fail but retry'
                else:
                    rospy.logerr('moving to the pose failed... DETACHING and trying again')
                    return 'fail'
    
            rospy.loginfo('moving to the pre mix position succeeded')
            if self.debug:
                jrp = JointRecorderPlayer()
                jrp.save_arm_state(2)
                print('\n\n\n\n\n\n')
                file  = raw_input('enter the filename to save joint states to (blank for scrapingjoints.log): ')
                if len(file ) == 0:
                    file  = 'scrapingjoints.log'
                jrp.write_saved_states_to_file(filename = file)
                file  = raw_input('enter the filename to save cookie sheet state to (blank for cookiesheet.log): ')
                if len(file ) == 0:
                    file  = 'cookiesheet.log'
                try:
                    print '**************************************************************'
                    myfile = open(file, 'wb')
                    cookie_sheet = userdata.cookie_sheet
                    print cookie_sheet
                    pickle.dump(cookie_sheet, myfile)
                    rospy.loginfo('file written to: ' + str(file))
                    myfile.close()
                    print '**************************************************************'
                except Exception as e:
                    print e
                    rospy.logerr('something went wrong writing the file')

            return 'done'
        else:
            # this assumes that i'm in the right spot based on the init
            return 'done'


class MoveMixingHandToPrePreScrapePosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail', 'fail with exhaustion'])
        self.arm_client = ArmClient.get_arm_client()
        self.papm = PickAndPlaceManager.get_pick_and_place_manager()
        self.failed_once = False


    def execute(self, userdata):
        rospy.loginfo('Executing state MoveMixingHandToPre Pre ScrapePosition')
        path = os.environ['BAKEBOT_JRPLOG_PATH']
        f = path + '2011-07-25-prescrapejrp.log'
        jrp = JointRecorderPlayer()
        jrp.load_saved_states_from_file(f)
        jrp.playback_all_joints(automatic = True)
        return 'done'
#        x = .6
#        y = -.6
#        z = .93 
#        if not self.failed_once:
#            status = self.arm_client.move_to_pose(True, False, x, y, z, -.5, 0, -.5, .5, collision_aware = 1, override_cmpapm=True)  # p69 num 6
#        else:
#            tol = 0.1
#            min_max_x_pos = (x - tol, x + tol)
#            min_max_y_pos = (y - tol, y + tol)
#            min_max_z_pos = (z - tol, z + tol)
#            quater_xyzw = (-.5, 0, -.5, .5) # TODO: may need to change this
#            status = self.arm_client.try_hard_to_move_to_pose(True, False, min_max_x_pos, min_max_y_pos, min_max_z_pos, quater_xyzw, 0.02, True)
#        if status is not 0:
#            if self.failed_once:
#                rospy.logerr('moving to the pose failed even when detached. failing with exhaustion!')
#                return 'fail with exhaustion'
#            else:
#                rospy.logerr('moving to poase failed')
#                status = self.arm_client.move_to_pose(True, False, x, y, z, -.5, 0, -.5, .5, collision_aware = 1, override_cmpapm = True)  # p69 num 6
#                if status == 0:
#                    rospy.loginfo('moving to the pre mix position succeeded')
#                    return 'done'
#                rospy.logerr('moving to the the pose failed... detaching and trying again')
#                self.failed_once = True
#                return 'fail'
#        else:
#            rospy.loginfo('\n\n\nmoving to the pre mix position succeeded\n\n\n')
#            return 'done'

class RemoveSpoonFromHandCollisionSpace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'failed'])

    def execute(self, userdata):
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
            return 'failed'
        return 'done'


class SwitchHandController(smach.State):
    def __init__(self, switch_to_imped):
        smach.State.__init__(self, outcomes=['done', 'fail'])
        self.switch_to_imped = switch_to_imped

    def execute(self, userdata):
        rospy.loginfo('Executing state SwitchHandToImpedanceControl')
        client = PR2CMClient.get_pr2cm_client()
        if self.switch_to_imped:
            status = client.load_ee_cart_imped(True) 
        else:
            status = client.load_cartesian(True)
        return 'done' if status else 'fail'


class PlungeSpoon(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                                   input_keys=['mixing_bowl', 'cookie_sheet', 'pre_mix_height_above_table'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PlungeSpoon')
        x = userdata.cookie_sheet.pose.pose.position.x
        #z = .88 #TODO: will want to change this to be less fragile
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
        return 'done' if status else 'fail'


class AssessScrape(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'shake', 'scrape','fail'])
        self.num_mixes_done = 0
        self.MAX_MIXES = 4  #TODO change this

    def execute(self, userdata):
        rospy.loginfo('Executing state AssessScrape')
        if self.num_mixes_done >= self.MAX_MIXES:
            rospy.loginfo('done scrape!')
            return 'done'
        else:
            #if self.num_mixes_done % 2 is 1: 
                rospy.loginfo('doing my ' + str(self.num_mixes_done) + ' mix')
                self.num_mixes_done = self.num_mixes_done + 1
                return 'scrape'
            #else:
                #rospy.loginfo('time to shake')
                #return 'shake'


class Scrape(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','fail'], 
                                   input_keys=['cookie_sheet']) 
        self.count = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Scrape')
       #mc = MixingClient.get_mixing_client()
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
       #print trans
       #print rot
       #bx = userdata.cookie_sheet.pose.pose.position.x
       #by = userdata.cookie_sheet.pose.pose.position.y
       #bz = userdata.cookie_sheet.pose.pose.position.z  #TODO: will want to correct this z

       #x = bx - trans[0]  #TODO: check this
       #y = by - trans[1]  # this value will be ignored
       ##z = bz - trans[2]
       #z = .78 - trans[2] #TODO make this less fragile
       #print 'position in tll: ', x, y, z
       #
       #print 'bowl position:', x, y, z
       #bowl_xyz_tllf = (x,y,z)
       #status = mc.circle_mix(bowl_xyz_tllf, circular_laps = 1, ismix = False)
       #return 'done' if status else 'fail'
        xoff = 0
        zstartoff1 = 0
        if self.count == 1:
            xoff = .08
        elif self.count == 2:
            xoff = -.08
        elif self.count == 3:
            zstartoff1 = .1
        csc = CleaningClient.get_cleaning_client()
        csc.scrape(xmove = xoff, zstartoff = zstartoff1)
        self.count = self.count + 1
        return 'done'


class Shake(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener
        self.papm = PickAndPlaceManager.get_pick_and_place_manager()

    def execute(self, userdata):
        rospy.loginfo('Executing state Shake')
        is_right_arm = False
        whicharm = 0 if is_right_arm else 1
        bcm = self.arm_client.cms[whicharm]
        positions = bcm.get_current_arm_angles()
        newpositions_plus = list()
        newpositions_minus = list()
        for i, position in enumerate(positions):
            if i < 4:
                fluff = 0.01
            elif i == 5:
                fluff = 0.18
            else:
                fluff = 0.00
            rospy.loginfo('i is ' + str(i) + ' and adding fluff of ' + str(fluff))
            newpositions_plus.append(position + fluff)
            newpositions_minus.append(position - fluff)
        for i in range(0,3):
            rospy.loginfo('sending one shake pulse')
            self.papm.try_hard_to_move_joint(whicharm, [newpositions_minus, newpositions_plus])
            time.sleep(0.5)
        bcm.move_arm_joint(positions)
        return 'done'



# will eventually want ot do this with the cartesian controller
class DeplungeSpoon(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
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
            return 'done'
        else:
            rospy.logerr('deplunge failure')
            return 'fail'


class PlaceBowl(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys = ['is_right_arm', 'bowl_to_deal', 'table_height', 'acceptable_height_above_table'],
                            output_keys = ['is_right_arm'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener
        self.pick_and_place_manager = PickAndPlaceManager.get_pick_and_place_manager()

    def execute(self, userdata):
        rospy.loginfo('Executing state PlaceBowl')
        rospy.logwarn('************* AUTOMATICALLY ABORTING *********************')
        return 'fail'
        #userdata.is_right_arm = False
        #time.sleep(2)
        #whicharm = 0 if userdata.is_right_arm else 1
        #status = self.pick_and_place_manager.place_object(whicharm, userdata.bowl_to_deal.pose)
        #rospy.loginfo('PAPM place request returned: ' + str(status))
        #if status == 0:
            #return 'success'
        #else:
            #return 'fail'


class ForcePlaceBowl(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail', 'fail with exhaustion'],
                             input_keys = ['is_right_arm', 'bowl_to_deal', 'table_height', 'acceptable_height_above_table'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener
        self.pick_and_place_manager = PickAndPlaceManager.get_pick_and_place_manager()
        self.count = 0
        self.limit = 5

    def execute(self, userdata):
        rospy.loginfo('Executing state ForcePlaceBowl')
        rospy.logwarn('************* AUTOMATICALLY ABORTING *********************')
        return 'fail'
#        whicharm = 1
#        use_joint_open_loop = 0 if self.count < 3 else 1
#        if use_joint_open_loop == 1:
#            rospy.logwarn('using joint open loop in force place')
#        status = self.pick_and_place_manager.place_object_override(whicharm, userdata.bowl_to_deal.pose, use_joint_open_loop)
#        rospy.loginfo('PAPM place request returned: ' + str(status))
#        self.count = self.count + 1
#        if status == 1:
#            return 'success'
#        else:
#            rospy.loginfo('force place failed.  count = ' + str(self.count))
#            if self.count > self.limit:
#                rospy.logerr('was unable to place')
#                return 'fail with exhaustion'
#            return 'fail'

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
        logger = EventLoggerClient.startfsm('DumpBowl (scrape fsm)')
        whicharm = 1
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



class ResetBothArms(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                                   input_keys=['mixing_bowl', 'pre_mix_height_above_table'])
        self.arm_client = ArmClient.get_arm_client()
        self.papm = PickAndPlaceManager.get_pick_and_place_manager()

    def execute(self, userdata):
        rospy.loginfo('Executing state ResetBothArms')
        rospy.loginfo('moving the right arm to the side')
        self.papm.move_arm_to_side(0)
        rospy.loginfo('moving the left arm to the side')
        self.papm.move_arm_to_side(1)
        rospy.loginfo('closing both grippers')
        self.papm.close_gripper(0)
        self.papm.close_gripper(1)
        return 'done'


class DumpSpoon(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'])

    def execute(self, userdata):
        rospy.loginfo('Executing state DumpSpoon')
        arm_client = ArmClient.get_arm_client()
        arm_client.retreat_arm(True)
        return 'done'

class MoveRobotOver(smach.State):
    def __init__(self, moveRight):
        smach.State.__init__(self, outcomes=['success', 'fail'], 
                                   input_keys=['max_left_movement', 'max_right_movement', 'table_frame_x_offset', 'delta_y', 'table_frame_y_offset', 'delta_x'],
                                   output_keys=['move_over_positions_exhausted', 'table_frame_y_offset', 'table_frame_x_offset', 'delta_x'])
        self.base_client = BaseClient.get_base_client()
        self.moveRight = moveRight

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveRobotOver')
        #userdata.delta_x = 0
        logger = EventLoggerClient.startfsm('MoveRobotOver (deal bowl fsm)')
        if self.moveRight:
            rospy.logwarn('moving the robot to the right by delta_y')
            rospy.logwarn('delta y is: ' + str(userdata.delta_y))
            #if userdata.table_frame_y_offset > userdata.max_right_movement:
            if False:
                rospy.logwarn('have already mvoed too far to the right.  cannot move further to the right.')
                userdata.move_over_positions_exhausted = True
                logger.stopf('cannot move further in this direction')
                return 'fail'
            else:
                status = self.base_client.translate(False, -1 * abs(userdata.delta_y))
                if status:
                    userdata.table_frame_y_offset = userdata.table_frame_y_offset + abs(userdata.delta_y)
                    #if userdata.delta_x is not 0:
                        #rospy.logwarn('driving negative x: ' + str(userdata.delta_x))
                        #self.base_client.translate(True, abs(userdata.delta_x))
                        #userdata.table_frame_x_offset = userdata.table_frame_x_offset - abs(userdata.delta_x)
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
                    #if userdata.delta_x is not 0:
                    #    rospy.logwarn('driving negative x: ' + str(userdata.delta_x))
                    #    self.base_client.translate(True, -1 * abs(userdata.delta_x))
                    #    userdata.table_frame_x_offset = userdata.table_frame_x_offset + abs(userdata.delta_x)
                    logger.stops()
                    return 'success'
                else:
                    rospy.logerr('base client translate returned false indicating failure')
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
        trans, rot = self.arm_client.get_transform(False)
        # TODO figure out the right position for this
        x = .5
        y = .93
        z = 1.16 #1.13 
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
        status = self.arm_client.move_to_pose(False, False, x, y, z, rot[0], rot[1], rot[2], rot[3])
        if status == 0:
            logger.stops()
            self.fail_count = 0
            return 'success'
        else:
            status = self.arm_client.move_to_pose(False, False, x, y, z, rot[0], rot[1], rot[2], rot[3])
            if status == 0:
                logger.stops()
                self.fail_count = 0
                return 'success'
            else:
                self.fail_count = self.fail_count + 1
                userdata.delta_y = dely
                logger.stopf()
                return 'fail'

def assemble_scraping_fsm(DEBUG = False):
    sm = smach.StateMachine(outcomes=['scraping_success', 'scraping_failure'],
                            input_keys=['acceptable_height_above_table', 'table_height', 'mixing_bowl', 'arms_to_try', 'table_frame_x_offset', 'table_frame_y_offset', 'pour_height_above_centroid', 'cookie_sheet', 'pre_mix_height_above_table'])
    sm.userdata.mixing_bowl_radius = 0.13  # TODO: measure this value
    sm.userdata.nominal_cookie_sheet_x = .65 #TODO: this value has to be fixed
    sm.userdata.nominal_cookie_sheet_y = 0
    sm.userdata.max_left_movement = .4
    sm.userdata.move_over_positions_exhausted = False


    pre_success_state = 'CLEAN UP LOGGING SUCCESS'
    success_state = 'scraping_success'
    pre_fail_state = 'CLEAN UP LOGGING FAIL'
    fail_state = 'scraping_failure'

    with sm:
        if DEBUG:
            smach.StateMachine.add('INIT FOR SCRAPING', InitializeForScraping(DEBUG),
                                    transitions={'done':'GRAB MIXING BOWL',
                                                 'debug bypass':'SWITCH HAND TO IMPED CTRL',
                                                'debug done':'MOVE HAND TO PRE SCRAPE POS'})
        else:
            smach.StateMachine.add('INIT FOR SCRAPING', InitializeForScraping(DEBUG),
                                    transitions={'done':'GRAB MIXING BOWL'})


        smach.StateMachine.add('GRAB MIXING BOWL', GrabMixingBowl(),
                                transitions={'success':'LIFT MIXING BOWL',
                                             'fail':pre_fail_state,
                                             'fail with exhaustion':pre_fail_state},
                                remapping={'mixing_bowl':'mixing_bowl',
                                           'arms_to_try':'arms_to_try',
                                           'table_frame_y_offset':'table_frame_y_offset',
                                           'table_frame_x_offset':'table_frame_x_offset',
                                           'move_over_positions_exhausted':'move_over_positions_exhausted',
                                           'grasp_pose':'grasp_pose',
                                           'global_grasp_pose':'global_grasp_pose'})

        smach.StateMachine.add('LIFT MIXING BOWL', ScrapeChangeGripperObjHeight(),
                                transitions={'success':'ROTATE TO PRE POUR POSE',
                                             'fail':'ROTATE TO PRE POUR POSE'},
                                remapping={'acceptable_height_above_table':'acceptable_height_above_table',
                                           'table_height':'table_height',
                                           'is_right_arm':'is_right_arm',
                                           'current_height_above_table':'current_height_above_table'})

        smach.StateMachine.add('ROTATE TO PRE POUR POSE', RotateToPrePourPose(),
                                transitions={'success':'MOVE BOWL OUT OF WAY',
                                             'fail':pre_fail_state},
                                remapping={'grasp_pose':'grasp_pose',
                                           'current_known_pose_name':'current_known_pose_name'})

        smach.StateMachine.add('MOVE BOWL OUT OF WAY', MoveBowlOutofWay(),
                                transitions={'success':'LOCATE COOKIE SHEET',
                                             'fail':'MOVE TO PRE POUR POSITION'})

        smach.StateMachine.add('LOCATE COOKIE SHEET', LocateCookieSheet(),
                                transitions={'done':'MOVE ROBOT',
                                             'fail':pre_fail_state},
                                remapping={'mixing_bowl':'mixing_bowl',
                                           'cookie_sheet':'cookie_sheet'})

        smach.StateMachine.add('MOVE ROBOT', ScrapingMoveRobot(),
                                transitions={'done':'LOCATE CS POS AGAIN'},
                                remapping={'cookie_sheet':'cookie_sheet', 
                                           'table_frame_x_offset':'table_frame_x_offset',
                                           'table_frame_y_offset':'table_frame_y_offset',
                                           'nominal_cookie_sheet_x':'nominal_cookie_sheet_x',
                                           'nominal_cookie_sheet_y':'nominal_cookie_sheet_y',
                                           'table_front_edge_x':'table_front_edge_x'})

        smach.StateMachine.add('LOCATE CS POS AGAIN', LocateCookieSheet(),
                                transitions={'done':'MOVE TO PRE POUR POSITION',
                                             'fail':pre_fail_state},
                                remapping={'mixing_bowl':'mixing_bowl',
                                           'cookie_sheet':'cookie_sheet'})

        smach.StateMachine.add('MOVE TO PRE POUR POSITION', MoveToPrePourPosition(),
                                transitions={'success':'POUR',
                                             'fail':pre_fail_state,
                                             'unknown pre pour pose':pre_fail_state},
                                remapping={'cookie_sheet':'cookie_sheet',
                                            'current_known_pose_name':'current_known_pose_name',
                                            'mixing_bowl_radius':'mixing_bowl_radius',
                                            'pour_height_above_centroid':'pour_height_above_centroid'})

                                            
        pour_fail = pre_fail_state if not DEBUG else 'USER POSE ADJUSTMENT'
        smach.StateMachine.add('POUR', Pour(),
                                transitions={'success':'ATTACH SPOON TO COLL MAP',
                                             'fail':pour_fail},
                                remapping={'current_known_pose_name':'current_known_pose_name'})
        if DEBUG:
            smach.StateMachine.add('USER POSE ADJUSTMENT', UserKeyboardArmClient(),
                                    transitions={'done':'POUR'})


        smach.StateMachine.add('ATTACH SPOON TO COLL MAP', AddSpoonToHandCollisionSpace(),
                                transitions={'done':'MOVE HAND TO PRE SCRAPE POS',
                                             'failed':'MOVE HAND TO PRE PRE SCRAPE POS'})

        smach.StateMachine.add('MOVE HAND TO PRE SCRAPE POS', MoveMixingHandToPreScrapePosition(DEBUG),
                                transitions={'done':'SWITCH HAND TO IMPED CTRL',
                                             'fail':'REMOVE SPOON FROM COLL MAP',
                                             'fail but retry':'MOVE HAND TO PRE SCRAPE POS',
                                             'fail with exhaustion':pre_fail_state},
                                remapping={'mixing_bowl':'mixing_bowl',
                                           'cookie_sheet':'cookie_sheet',
                                           'pre_mix_height_above_table':'pre_mix_height_above_table'})

        smach.StateMachine.add('REMOVE SPOON FROM COLL MAP', RemoveSpoonFromHandCollisionSpace(),
                                transitions={'done':'MOVE HAND TO PRE PRE SCRAPE POS',
                                             'failed':pre_fail_state})

        smach.StateMachine.add('MOVE HAND TO PRE PRE SCRAPE POS', MoveMixingHandToPrePreScrapePosition(),
                                transitions={'done':'MOVE HAND TO PRE SCRAPE POS',
                                             'fail':'MOVE HAND TO PRE PRE SCRAPE POS',
                                             'fail with exhaustion':pre_fail_state})

        smach.StateMachine.add('SWITCH HAND TO IMPED CTRL', SwitchHandController(True),
                                transitions={'done':'PLUNGE SPOON',
                                             'fail':pre_fail_state})

        smach.StateMachine.add('PLUNGE SPOON', PlungeSpoon(),
                                transitions={'done':'ASSESS SCRAPE',
                                             'fail':pre_fail_state})

        smach.StateMachine.add('ASSESS SCRAPE', AssessScrape(),
                                transitions={'done':'DEPLUNGE SPOON',
                                             'scrape':'SCRAPE',
                                             'shake':'SHAKE',
                                             'fail':pre_fail_state})

        smach.StateMachine.add('SHAKE', Shake(),
                                transitions={'done':'ASSESS SCRAPE'})

        smach.StateMachine.add('SCRAPE', Scrape(),
                                transitions={'done':'PLUNGE SPOON',
                                             'fail':pre_fail_state})

       #smach.StateMachine.add('HAND TO CART', SwitchHandController(False),
       #                        transitions={'done':'MOVE HAND TO PRE SCRAPE POS',
       #                                     'fail':pre_fail_state})

        smach.StateMachine.add('DEPLUNGE SPOON', DeplungeSpoon(),
                                transitions={'done':'SWITCH HAND TO POSITION CTRL',
                                             'fail': pre_fail_state},
                                remapping={'mixing_bowl':'mixing_bowl',
                                           'pre_mix_height_above_table': 'pre_mix_height_above_table'})

        smach.StateMachine.add('SWITCH HAND TO POSITION CTRL', SwitchHandController(False),
                                transitions={'done':'DUMP SPOON',
                                             'fail':pre_fail_state})
        smach.StateMachine.add('DUMP SPOON', DumpSpoon(), 
                                transitions={'done':'REVERT',
                                             'fail': pre_fail_state})
        smach.StateMachine.add('REVERT', Revert(),
                                transitions={'success':'RETURN TRANSIT BOWL',
                                             'fail':'RETURN TRANSIT BOWL'},
                                remapping={'current_known_pose_name':'current_known_pose_name'})

        #smach.StateMachine.add('RETURN TRANSIT BOWL', MoveBowlOutofWay(False),
        smach.StateMachine.add('RETURN TRANSIT BOWL', ReturnTransitBowl2(),
                                transitions={'success':'PLACE BOWL',
                                             'fail':'PLACE BOWL',
                                             'fail with exhaustion':'PLACE BOWL'})

        smach.StateMachine.add('PLACE BOWL', PlaceBowl(),
                                transitions={'success':'RESET BOTH ARMS',
                                             'fail':'FORCE PLACE BOWL'},
                                remapping={'current_known_pose_name':'current_known_pose_name',
                                            'bowl_to_deal':'cookie_sheet',
                                            'is_right_arm':'is_right_arm',
                                            'table_height':'table_height',
                                            'acceptable_height_above_table':'acceptable_height_above_table'})
        smach.StateMachine.add('FORCE PLACE BOWL', ForcePlaceBowl(),
                                transitions={'success':'RESET BOTH ARMS',
                                             'fail':'RETURN TRANSIT BOWL AGAIN',
                                             'fail with exhaustion':'RETURN TRANSIT BOWL AGAIN'},
                                remapping={'current_known_pose_name':'current_known_pose_name',
                                            'is_right_arm':'is_right_arm',
                                            'bowl_to_deal':'cookie_sheet',
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
        smach.StateMachine.add('RESET BOTH ARMS', ResetBothArms(),
                                transitions={'done':'MOVE ROBOT BACK',
                                             'fail': pre_fail_state})

        smach.StateMachine.add('MOVE ROBOT BACK', ResetRobotBasePos(),
                                transitions={'done':pre_success_state},
                                remapping={'cookie_sheet':'cookie_sheet', 
                                           'table_frame_x_offset':'table_frame_x_offset',
                                           'table_frame_y_offset':'table_frame_y_offset',
                                           'nominal_cookie_sheet_x':'nominal_cookie_sheet_x',
                                           'nominal_cookie_sheet_y':'nominal_cookie_sheet_y',
                                           'table_front_edge_x':'table_front_edge_x'})

        smach.StateMachine.add('CLEAN UP LOGGING FAIL', CleanupLogging(False),
                                transitions={'done':fail_state})

        smach.StateMachine.add('CLEAN UP LOGGING SUCCESS', CleanupLogging(True),
                                transitions={'done':success_state})

    return sm

if __name__ == '__main__':
    rospy.init_node('bowl_scraper')
    try:
        sm = assemble_scraping_fsm()
        sis = smach_ros.IntrospectionServer('introspection_server', sm, '/SCRAPE')
        sis.start()
        #rospy.spin()
        outcome = sm.execute()
    finally:
        sis.stop()

