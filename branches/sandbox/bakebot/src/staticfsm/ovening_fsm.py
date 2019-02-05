#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
import rospy
import tf
import sys
import os
import actionlib
from clients.arm_client import ArmClient
from clients.base_client import BaseClient
from utilities.bakebot_pick_and_place_manager import *
from utilities.broad_table_object_manager import *
from services.bakebot_logging import *
from clients.oven_door_client import *
from grab_rim_fsm import *
from clients.pr2cm_client import *
from clients.mixing_client import *
from utilities.bowl_dealer_utilities import *
from bakebot.srv import *
from clients.torso_client import *
import math
import smach
import smach_ros
import time
import pickle
from utilities.joint_recorder_player import *

class InitForOvening(smach.State):
    def __init__(self, debug = False):
        smach.State.__init__(self, outcomes=['done', 'grab', 'lift'])
        self.debug = debug
    def execute(self, userdata):
        rospy.loginfo('Executing state InitForOvening')
        torso_client = TorsoClient()
        torso_client.up()
        oveninglogger = EventLoggerClient('milestone', annotation='ovening')
        StaticLoggerLocker.check_in_logger('OVENING LOGGER', oveninglogger) 
        if self.debug:
            choice = raw_input( '\n\n\nskip to (g)rab or (l)ift or enter to continue: ')
            if len(choice) == 0:
                return 'done'
            elif choice == 'g':
                return 'grab'
            elif choice == 'l':
                return 'lift'
            else:
                return 'done'
        else:
            return 'done'

class Retreat(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                             input_keys=['table_retreat_dist'])
    def execute(self, userdata):
        rospy.loginfo('Executing state Retreat')  
        logger = EventLoggerClient.startfsm('Oven (ovening fsm)')
        return 'done'
        #bc = BaseClient.get_base_client()
        #delta = -userdata.table_retreat_dist 
        #status = bc.translate(True, delta)
        #if status: 
        #    logger.stops()
        #    return 'done'
        #else:
        #    logger.stopf()
        #    return 'fail'

class DriveToOven(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                             input_keys=['oven_area_pos'])
    def execute(self, userdata):
        rospy.loginfo('Executing state DriveToOven')  
        logger = EventLoggerClient.startfsm('DriveToOven (ovening fsm)')
        bc = BaseClient.get_base_client()
        (x, y) = userdata.oven_area_pos
        status = bc.go_to_pos(x, y)
        if not status:
            logger.stopf()
            return 'fail'
        else:
            logger.stops()
            return 'done'

class AlignRobotWithOven(smach.State):
    def __init__(self, debug = False):
        smach.State.__init__(self, outcomes=['open', 'place', 'fail'])
        self.gone_once = False
        self.debug = debug
    def execute(self, userdata):
        rospy.loginfo('Executing state AlignRobotWithOven')  
        logger = EventLoggerClient.startfsm('AlignRobotWithOven (ovening fsm)')
        # TODO will want ot have this align
        if not self.debug:
            if not self.gone_once:
                self.gone_once = True
                logger.stops()
                return 'open'
            else:
                logger.stops()
                return 'place'
        else:
            choice = raw_input('(o)pen or (p)lace:')
            if choice == 'o':
                return 'open'
            else:
                return 'place'

class OpenOven(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['get cs', 'done', 'fail'])
        self.gone_once = False
    def execute(self, userdata):
        rospy.loginfo('Executing state OpenOven')  
        logger = EventLoggerClient.startfsm('OpenOven (ovening fsm)')
        oc = OvenDoorClient.get_oven_door_client()
        status = oc.open_door()
        print '\n\n\n\t\topen door status: ', status
        papm = PickAndPlaceManager.get_pick_and_place_manager()
        papm.open_gripper(1)
        time.sleep(3)
        ac = ArmClient.get_arm_client()
        print 'move arm back status: ', ac.move_delta_const_ee(False, -.10, 'x')
        print 'move arm back status: ', ac.move_delta_const_ee(False, -.10, 'x')
        bc = BaseClient.get_base_client()
        bc.translate(True, -.15) 
        if status:
            logger.stops()
            if not self.gone_once:
                self.gone_once = True
                return 'get cs'
            else:
    #           time.sleep(10)
    #           print 'waited 10'
    #           time.sleep(10)
    #           print 'waited 10'
    #           time.sleep(10)
    #           print 'waited 10'
    #           time.sleep(10)
    #           print 'waited 10'
    #           time.sleep(10)
    #           print 'waited 10'
    #           time.sleep(10)
    #           print '1waited 10'
    #           time.sleep(10)
    #           print '1waited 10'
    #           time.sleep(10)
    #           print '1waited 10'
    #           time.sleep(10)
    #           print '1waited 10'
    #           time.sleep(10)
    #           print '1waited 10'
    #           time.sleep(10)
    #           print 'done waiting'
                return 'done'
        else:
            self.gone_once = True
            logger.stopf()
            return 'fail'

class DriveToTable(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                             input_keys=['table_area_pos', 'table_retreat_dist'])
    def execute(self, userdata):
        rospy.loginfo('Executing state DriveToTable')  
        logger = EventLoggerClient.startfsm('DriveToTable (ovening fsm)')
        bc = BaseClient.get_base_client()
        (x, y) = userdata.table_area_pos
        print 'i want to go to: ', (x, y)
        status = bc.go_to_pos(x, y, x_first = False)
        #status = status and bc.translate(True, userdata.table_retreat_dist)
        if not status:
            logger.stopf()
            return 'fail'
        else:
            logger.stops()
            return 'done'


class LocateCookieSheet(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fail', 'grab', 'align', 'lost cookie sheet'],
                             input_keys=['cookie_sheet',
                                         'nominal_cookie_sheet_pos',
                                         'table_frame_x_offset',
                                         'table_frame_y_offset'],
                             output_keys=['cookie_sheet', 'table_frame_y_offset', 'table_frame_x_offset'])
        self.num_attempts = 0
        self.first_time = True
    def execute(self, userdata):
        rospy.loginfo('Executing state LocateCookieSheet')
        logger = EventLoggerClient.startfsm('LocateCookieSheet (ovening fsm)')
        x = userdata.cookie_sheet.pose.pose.position.x
        y = userdata.cookie_sheet.pose.pose.position.y
        z = userdata.cookie_sheet.pose.pose.position.z
        rospy.loginfo('pose of cookie_sheet bowl: ' + str((x,y,z)))
        rospy.loginfo('replacing the pose of userdata.cookie_sheet with the pose from the most recent detection')
        btom = BroadTabletopObjectManager.get_broad_tabletop_object_manager(PickAndPlaceManager.get_pick_and_place_manager())
        if self.first_time or True: #TODO get rid of the or True
            self.first_time = False
            detected_objs = btom.do_broad_tabletop_object_detection()
            if len(detected_objs) == 0:
                rospy.logerr('could not find the cookie_sheet bowl!')
                return 'fail'
            mindist = 1000000
            minindex = -1
            xdes, ydes = userdata.nominal_cookie_sheet_pos
            print detected_objs
            for i, obj in enumerate(detected_objs):
                x = obj.pose.pose.position.x 
                y = obj.pose.pose.position.y 
                dist = (xdes - x)**2 + (ydes - y)**2
                print obj
                print 'dist: ', dist
                if dist < mindist:
                    mindist = dist
                    minindex = i
                    print 'minindex: ', i, mindist
            detected_cookie_sheet = detected_objs[minindex]
        else:
            detected_cookie_sheet = btom.refine_and_and_get_graspable_object(userdata.cookie_sheet.collision_name, userdata.table_frame_x_offset, userdata.table_frame_y_offset)

        if detected_cookie_sheet == None:
            rospy.logerr('could not find the cookie_sheet bowl!')
            return 'fail'
        userdata.cookie_sheet = detected_cookie_sheet
        x = userdata.cookie_sheet.pose.pose.position.x
        y = userdata.cookie_sheet.pose.pose.position.y
        z = userdata.cookie_sheet.pose.pose.position.z
        rospy.loginfo('corrected pose of cookie_sheet bowl: ' + str((x,y,z)))
        rospy.loginfo('collision name: ' + userdata.cookie_sheet.collision_name)
        xdes, ydes = userdata.nominal_cookie_sheet_pos
        dx = x - xdes
        dy = y - ydes
        sum
        print 'dx: ', dx
        print 'dy: ', dy
        if abs(dx) < .05 and abs(dy) < .05:
            # we are good, can return grab
            logger.stops()
            userdata.table_frame_x_offset = 0
            userdata.table_frame_y_offset = 0
            return 'grab'
        elif self.num_attempts > 3:
            logger.stops()
            userdata.table_frame_x_offset = 0
            userdata.table_frame_y_offset = 0
            return 'grab'
        else:
            self.num_attempts = self.num_attempts + 1
            logger.stops()
            return 'align'

class AlignRobotWithCookieSheet(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                                    input_keys=['cookie_sheet',
                                                'nominal_cookie_sheet_pos',
                                                'table_frame_x_offset',
                                                'table_frame_y_offset'],
                                    output_keys=['table_frame_x_offset', 'table_frame_y_offset'])
    def execute(self, userdata):
        rospy.loginfo('Executing state AlignRobotWithCookieSheet')  
        logger = EventLoggerClient.startfsm('AlignRobotWithCookieSheet (ovening fsm)')
        bc = BaseClient.get_base_client()
        actual_x = userdata.cookie_sheet.pose.pose.position.x
        actual_y = userdata.cookie_sheet.pose.pose.position.y
        print '\n\n*****************************************'
        print 'cookie sheet pose'
        print userdata.cookie_sheet.pose.pose.position.x
        print userdata.cookie_sheet.pose.pose.position.y
        print userdata.cookie_sheet.pose.pose.position.z
        print 'desired cookie sheet pose'
        print userdata.nominal_cookie_sheet_pos
        print '*****************************************\n'
        desired_x, desired_y = userdata.nominal_cookie_sheet_pos
        drive_x = actual_x - desired_x
        drive_y = actual_y - desired_y
        rospy.loginfo('DESIRED DRIVE Y: ' + str(drive_y))
        rospy.loginfo('DESIRED DRIVE X: ' + str(drive_x))
        bc.translate(False, drive_y)
        bc.translate(True, drive_x)
        userdata.table_frame_x_offset = drive_x  #TODO need to reverse this
        userdata.table_frame_y_offset = drive_y
        print '********** xoff', userdata.table_frame_x_offset
        print '********** yoff', userdata.table_frame_y_offset
        logger.stops()
        return 'done'

# the grab is done by the grab-rim-fsm

class OvenChangeGripperObjHeight(smach.State):
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


class RotateToPreOvenPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['current_known_pose_name'],
                             output_keys=['current_known_pose_name'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener
        self.pose_manager = None
    def execute(self, userdata):
        rospy.loginfo('Executing state RotateToPreOven')
        rospy.logwarn('AUTOMATICALLY ABORTING')
        return 'success'
        self.pose_manager = GripperBowlPoseManager(None, self.arm_client, True, is_pre_oven = True)
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

class MoveArmBack(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'])
        self.arm_client = ArmClient.get_arm_client()
    def execute(self, userdata):
        rospy.loginfo('Executing state MoveArmBack')
        print 'trying one'
        status = self.arm_client.move_to_pose_const_ee(False, .34, -.06, 1.12)
        #status = self.arm_client.move_to_pose(False, True, .37, -.07, 1.05, -.6, .36, .4, .6)
        if status == 0:
            rospy.loginfo('success')
            return 'done'
        else:
            status = self.arm_client.move_to_pose_const_ee(False, .34, -.06, 1.12)
            #status = self.arm_client.move_to_pose(False, False, .37, -.07, 1.05, -.6, .36, .4, .6)
            if status == 0:
                rospy.loginfo('success')
                return 'done'
            else:
                status = self.arm_client.move_to_pose_const_ee(False, .34, -.06, 1.12)
                #status = self.arm_client.move_to_pose(False, False, .37, -.07, 1.05, -.6, .36, .4, .6)
                if status == 0:
                    rospy.loginfo('success')
                    return 'done'
                else:
                    return 'fail'

class MoveRobotToPreOven(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'])
    def execute(self, userdata):
        rospy.loginfo('Executing state MoveToPreOven')  
        logger = EventLoggerClient.startfsm('MoveToPreOven (ovening fsm)')
        bc = BaseClient.get_base_client()
        #bc.fake_rotate(1)
        #bc.translate(True, -.03)
        logger.stops()
        return 'done'

class MoveHandToPreOvenPos(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'])
    def execute(self, userdata):
        rospy.loginfo('Executing state MoveHandToPreOvenPos')  
        logger = EventLoggerClient.startfsm('MoveHandToPreOvenPos (ovening fsm)')
        rospy.logerr('Not yet implemented')
        self.arm_client = ArmClient.get_arm_client()
        #status = self.arm_client.move_to_pose_const_ee(False, .54, -.09, 1.05)
        status = self.arm_client.move_to_pose_const_ee(False, .55, -.06, 1.06)
        if status == 0:
            rospy.loginfo('done')
            logger.stops()
            return 'done'
        else:
            status = self.arm_client.move_to_pose_const_ee(False, .55, -.06, 1.06)
            if status == 0:
                rospy.loginfo('done')
                logger.stops()
                return 'done'
            else:
                logger.stopf()
                return 'fail'

class MoveSheetIntoOven(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                             input_keys=['insert_sheet_delta'])
    def execute(self, userdata):
        rospy.loginfo('Executing state MoveSheetIntoOven')  
        logger = EventLoggerClient.startfsm('MoveSheetIntoOven (ovening fsm)')
        #delta = userdata.insert_sheet_delta + .10
        ac = ArmClient.get_arm_client()
        #ac.move_delta_const_ee(False, delta, 'x')
        status = ac.move_to_pose_const_ee(False, .63, -.06, 1.)
        #status = ac.move_to_pose_const_ee(False, .71, -.10, .97)
        if status == 0:
            logger.stops()
            return 'done'
        else:
            status = ac.move_to_pose_const_ee(False, .63, -.06, 1.0)
            if status == 0:
                logger.stops()
                return 'done'
            else:
                return 'fail'

class MoveSheetIntoOvenWithBase(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                             input_keys=['nominal_retreat_arm_delta'])
    def execute(self, userdata):
        rospy.loginfo('Executing state MoveSheetIntoOvenWithBase')  
        logger = EventLoggerClient.startfsm('MoveSheetIntoOvenWithBase (ovening fsm)')
        #bc = BaseClient.get_base_client()
        #delta = userdata.insert_sheet_delta
        #rospy.loginfo('going to call the insert (with base) for a delta of: ' + str(delta))
        #status = bc.translate(True, delta)
        #if status:
            #logger.stops()
            #return 'done'
        #else:
            #logger.stopf()
            #return 'fail'
        rospy.logwarn('not yet implemented')
        logger.stops()
        return 'done'

class LowerSheet(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'])
    def execute(self, userdata):
        rospy.loginfo('Executing state LowerSheet')  
        logger = EventLoggerClient.startfsm('LowerSheet (ovening fsm)')
        ac = ArmClient.get_arm_client()
        trans, rot = ac.get_transform(False)
        delta = 1.06- trans[2] # TODO: need to measure this to get it into oven
        status = ac.move_delta_const_ee(False, delta, 'z')
        if status == 0:
            logger.stops()
            return 'done'
        else:
            rospy.logwarn('unable to lower sheet, trying again')
            status = ac.move_delta_const_ee(False, delta, 'z')
            if status == 0:
                logger.stops()
                return 'done'
            rospy.logwarn('unable to lower sheet, trying again with smaller delta')
            status = ac.move_delta_const_ee(False, delta + .01, 'z')
            if status == 0:
                logger.stops()
                return 'done'
            else:
                logger.stopf()
                return 'fail'

class LowerSheetWithTorso(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'])
    def execute(self, userdata):
        rospy.loginfo('Executing state LowerSheetWithTorso')  
        logger = EventLoggerClient.startfsm('LowerSheetWithTorso (ovening fsm)')
        rospy.logerr('not yet implemented')
        # TODO implement
        logger.stopf()
        return 'fail'

class OpenGripper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'])
    def execute(self, userdata):
        rospy.loginfo('Executing state OpenGripper')  
        logger = EventLoggerClient.startfsm('OpenGripper (ovening fsm)')
        papm = PickAndPlaceManager.get_pick_and_place_manager()
        papm.open_gripper(1)
        logger.stops()
        return 'done'

class LiftGripper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'])
    def execute(self, userdata):
        rospy.loginfo('Executing state LiftGripper')  
        logger = EventLoggerClient.startfsm('LiftGripper (ovening fsm)')
        arm_client = ArmClient.get_arm_client()
        (trans, rot) = arm_client.get_transform(False)
        (x, y, z) = trans
        zdes = 1.05
        status = arm_client.move_to_pose_const_ee(False, x, y, zdes)
        if status == 0:
            logger.stops()
            return 'done'
        else:
            rospy.logwarn('arm client failed to lift')
            status = arm_client.move_to_pose_const_ee(False, x, y, zdes)
            if status == 0:
                logger.stops()
                return 'done'
            status = arm_client.move_to_pose_const_ee(False, x, y, zdes)
            if status == 0:
                logger.stops()
                return 'done'
            return 'fail'


class RetreatArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                             input_keys=['nominal_retreat_arm_delta'])
    def execute(self, userdata):
        rospy.loginfo('Executing state RetreatArm')  
        logger = EventLoggerClient.startfsm('RetreatArm (ovening fsm)')
        ac = ArmClient.get_arm_client()
        status = ac.move_to_pose_const_ee(False, .39, .41, 1.03)
        if status == 0:
            logger.stops()
            return 'done'
        else:
            status = ac.move_to_pose_const_ee(False, .39, .41, 1.03)
            if status == 0:
                logger.stops()
                return 'done'
            else:
                logger.stopf()
                return 'fail'

class RetreatArmWithBase(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                             input_keys=['nominal_retreat_arm_delta'])
    def execute(self, userdata):
        rospy.loginfo('Executing state RetreatArmWithBase')  
        logger = EventLoggerClient.startfsm('RetreatArmWithBase (ovening fsm)')
#       bc = BaseClient.get_base_client()
#       delta = -.2
#       rospy.loginfo('going to call the retreat (with base) for a delta of: ' + str(delta))
#       status = bc.translate(True, delta)
#       if status:
#           logger.stops()
#           return 'done'
#       else:
#           logger.stopf()
#           return 'fail'
        rospy.logwarn('not yet implemented')
        logger.stops()
        return 'done'

class MoveRobotToPreClose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'])
    def execute(self, userdata):
        rospy.loginfo('Executing state MoveRobotToPreClose')  
        logger = EventLoggerClient.startfsm('MoveRobotToPreClose (ovening fsm)')
        rospy.logerr('Not yet implemented')
        # TODO implementOven
        logger.stops()
        return 'done'

class CloseOven(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'], 
                             output_keys = ['cook_start_time'])
    def execute(self, userdata):
        rospy.loginfo('Executing state CloseOven')  
        logger = EventLoggerClient.startfsm('Oven (ovening fsm)')
        # TODO implement
        path = os.environ['BAKEBOT_LOG_PATH']
        f = path + '20110630-closeovenjrp.log'
        jrp = JointRecorderPlayer()
        jrp.load_saved_states_from_file(f)
        jrp.playback_all_joints(automatic = True)
        userdata.cook_start_time = rospy.Time.now()
        logger.stops()
        return 'done'

class MoveRobotToPreOpen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['open', 'wait', 'fail'])
        self.first_pass = True
        self.num_fails = 0
    def execute(self, userdata):
        desired = .88
        rospy.loginfo('Executing state MoveRobotToPreOpen')  
        logger = EventLoggerClient.startfsm('MoveRobotToPreOpen (ovening fsm)')
        odc = OvenDoorClient.get_oven_door_client()
        bc = BaseClient.get_base_client()
        actual = odc.get_door_pos()
        if actual > 500:
            rospy.logwarn('could not find the oven')
            self.num_fails = self.num_fails + 1
            if self.num_fails > 5:
                return 'fail'
            else:
                rospy.logwarn('could not find the oven, recursing')
                rospy.logwarn('number of failures to find oven: ' + str(self.num_fails))
                rospy.logwarn('backing up to try again')
                bc.translate(True, -.15)
                return self.execute(userdata)
        drivex = actual
        status = bc.translate(True, drivex)
        if not status:
            self.first_pass = False
            logger.stopf()
            return 'fail'
        if self.first_pass:
            self.first_pass = False
            logger.stops()
            return 'open'
        else:
            logger.stops()
            return 'wait'

class Wait(smach.State):
    def __init__(self, debug = False):
        smach.State.__init__(self, outcomes=['done'], 
                            input_keys=['cook_start_time'])
        self.debug = debug
    def execute(self, userdata):
        rospy.loginfo('Executing state Wait')  
        logger = EventLoggerClient.startfsm('Wait (ovening fsm)')
        if self.debug:
            choice = raw_input('press any key to skip or enter to wait 20min')
            if len(choice) > 0:
                print 'skipping'
                logger.stops()
                return 'done'
        start = userdata.cook_start_time
        stop = start + rospy.Duration(20 * 60)
        print stop
        print start
        print rospy.Duration(20*60)
        try:
            while rospy.Time.now() < stop and (not rospy.is_shutdown()):
                remaining = stop - rospy.Time.now() 
                rospy.loginfo('I still have to wait ' + str(remaining/10.**9) + ' seconds until cookies')
                rospy.sleep(60)
            rospy.loginfo('done waiting!')
            logger.stops()
            return 'done'
        finally:
            return 'done'

class UserKeyboardArmClient(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.arm_client = ArmClient.get_arm_client()
    def execute(self, userdata):
        rospy.loginfo('Executing state UserKeyboardArmClient')
        self.arm_client.ui_loop()
        return 'done'

class ResetBothArms(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                                   input_keys=['mixing_bowl', 'pre_mix_height_above_table'])
        self.arm_client = ArmClient.get_arm_client()
        self.papm = PickAndPlaceManager.get_pick_and_place_manager()
    def execute(self, userdata):
        client = PR2CMClient.get_pr2cm_client()
        print client.load_cartesian(True)
        print client.load_cartesian(False)
        rospy.loginfo('Executing state ResetBothArms')
        rospy.loginfo('moving the right arm to the side')
        self.papm.move_arm_to_side(0)
        rospy.loginfo('moving the left arm to the side')
        self.papm.move_arm_to_side(1)
        rospy.loginfo('closing both grippers')
        self.papm.close_gripper(0)
        self.papm.close_gripper(1)
        return 'done'

class CleanupLogging(smach.State):
    def __init__(self, is_success):
        smach.State.__init__(self, outcomes=['done'])
        self.is_success = is_success
    def execute(self, userdata):
        rospy.loginfo('Executing state CleanupLogging: ' + str(self.is_success))
        oveninglogger = StaticLoggerLocker.check_out_logger('OVENING LOGGER')
        if self.is_success:
            oveninglogger.stops()
        else:
            oveninglogger.stopf()
        return 'done'


def assemble_ovening_fsm(DEBUG = False):
    sm = smach.StateMachine(outcomes=['ovening_success', 'ovening_failure'],
                            input_keys=['acceptable_height_above_table', 'table_height', 'table_frame_x_offset', 'table_frame_y_offset', 'cookie_sheet']) 
    sm.userdata.cookie_sheet_radius = 0.1225  
    sm.userdata.grasp_with_right_hand = False
    sm.userdata.nominal_retreat_arm_delta = -0.15 # TODO need ot measure this
    sm.userdata.insert_sheet_delta = 0.15
    sm.userdata.table_retreat_dist = 0.3#TODO need to figure out what this distance is
    sm.userdata.nominal_cookie_sheet_pos = (.65, 0)
    #sm.userdata.oven_area_pos = (-.48, 1.97) # used to be .95
    sm.userdata.oven_area_pos = (-.52, 1.97) # used to be .95
    sm.userdata.table_area_pos = (0, .20)
    sm.userdata.is_right_arm = False

    pre_fail_state = 'OVENING FAIL CLEANUP'
    pre_success_state = 'OVENING SUCCESS CLEANUP'
    fail_state = 'ovening_failure'
    # TODO add to this here!
    with sm:
        smach.StateMachine.add('INIT OVENING', InitForOvening(debug = DEBUG),
                                transitions={'done':'RESET BOTH ARMS',
                                             'grab':'GRAB CSHEET',
                                             'lift':'LIFT'})

        smach.StateMachine.add('RESET BOTH ARMS', ResetBothArms(),
                                transitions={'done':'RETREAT FROM TABLE',
                                             'fail': pre_fail_state})

        smach.StateMachine.add('RETREAT FROM TABLE', Retreat(),
                                transitions={'done':'DRIVE TO OVEN',
                                             'fail': pre_fail_state})

        smach.StateMachine.add('DRIVE TO OVEN', DriveToOven(),
                                transitions={'done':'ALIGN WITH OVEN',
                                             'fail': pre_fail_state})

        smach.StateMachine.add('ALIGN WITH OVEN', AlignRobotWithOven(debug = DEBUG),
                                transitions={'open':'MOVE TO PRE OPEN',
                                             'place':'MOVE ROBOT TO PRE OVEN',
                                             'fail': pre_fail_state})

        smach.StateMachine.add('MOVE ROBOT TO PRE OVEN', MoveRobotToPreOven(),
                                transitions={'done':'MOVE HAND TO PRE OVEN',
                                             'fail': pre_fail_state})

        smach.StateMachine.add('MOVE HAND TO PRE OVEN', MoveHandToPreOvenPos(),
                                transitions={'done':'LOWER SHEET',
                                             'fail': pre_fail_state})

        smach.StateMachine.add('MOVE SHEET INTO OVEN', MoveSheetIntoOven(),
                                transitions={'done':'OPEN GRIPPER',
                                             'fail': 'MOVE SHEET INTO OVEN WITH BASE'})

        smach.StateMachine.add('MOVE SHEET INTO OVEN WITH BASE', MoveSheetIntoOvenWithBase(),
                                transitions={'done':'OPEN GRIPPER',
                                             'fail': pre_fail_state})

        smach.StateMachine.add('LOWER SHEET', LowerSheet(),
                                transitions={'done':'MOVE SHEET INTO OVEN',
                                             'fail': 'LOWER SHEET WITH TORSO'})

        smach.StateMachine.add('LOWER SHEET WITH TORSO', LowerSheetWithTorso(),
                                transitions={'done':'MOVE SHEET INTO OVEN',
                                             'fail': pre_fail_state})

        smach.StateMachine.add('OPEN GRIPPER', OpenGripper(),
                                transitions={'done':'LIFT GRIPPER',
                                             'fail': pre_fail_state})

        smach.StateMachine.add('LIFT GRIPPER', LiftGripper(),
                                transitions={'done':'RETREAT ARM',
                                             'fail':'RETREAT ARM'}) #NOTE is this worth it?

        smach.StateMachine.add('RETREAT ARM', RetreatArm(),
                                transitions={'done':'RESET BOTH ARMS PRE CLOSE',
                                             'fail': 'RETREAT ARM WITH BASE'})

        smach.StateMachine.add('RETREAT ARM WITH BASE', RetreatArmWithBase(),
                                transitions={'done':'RESET BOTH ARMS PRE CLOSE',
                                             'fail': pre_fail_state})

        smach.StateMachine.add('RESET BOTH ARMS PRE CLOSE', ResetBothArms(),
                                transitions={'done':'MOVE TO PRE CLOSE',
                                             'fail':'MOVE TO PRE CLOSE'})

        smach.StateMachine.add('MOVE TO PRE CLOSE', MoveRobotToPreClose(),
                                transitions={'done':'CLOSE OVEN',
                                             'fail': pre_fail_state})

        smach.StateMachine.add('CLOSE OVEN', CloseOven(),
                                transitions={'done':'MOVE TO PRE OPEN',
                                             'fail': pre_fail_state})

        smach.StateMachine.add('MOVE TO PRE OPEN', MoveRobotToPreOpen(),
                                transitions={'open':'OPEN OVEN',
                                             'wait':'WAIT',
                                             'fail': pre_fail_state})

        smach.StateMachine.add('WAIT', Wait(True),
                                transitions={'done':'OPEN OVEN'})

        smach.StateMachine.add('OPEN OVEN', OpenOven(),
                                transitions={'done':'SUC RESET BOTH ARMS',
                                             'get cs':'POST OPEN RESET',
                                             'fail': pre_fail_state})

        smach.StateMachine.add('SUC RESET BOTH ARMS', ResetBothArms(),
                                transitions={'done':'OVENING SUCCESS CLEANUP',
                                             'fail': 'OVENING SUCCESS CLEANUP'})

        smach.StateMachine.add('POST OPEN RESET', ResetBothArms(),
                                transitions={'done':'DRIVE TO TABLE',
                                             'fail':'DRIVE TO TABLE'})

        #smach.StateMachine.add('RETREAT FROM OVEN', Retreat(),
                                #transitions={'done':'DRIVE TO TABLE',
                                             #'fail': pre_fail_state})

        smach.StateMachine.add('DRIVE TO TABLE', DriveToTable(),
                                transitions={'done':'LOCATE COOKIE SHEET',
                                             'fail': pre_fail_state})

        smach.StateMachine.add('LOCATE COOKIE SHEET', LocateCookieSheet(),
                                transitions={'grab':'GRAB CSHEET',
                                             'align':'ALIGN WITH CS',
                                             'lost cookie sheet': 'LOCATE COOKIE SHEET', # TODO make this a search
                                             'fail': 'LOCATE COOKIE SHEET'})

        smach.StateMachine.add('ALIGN WITH CS', AlignRobotWithCookieSheet(),
                                transitions={'done':'LOCATE COOKIE SHEET',
                                             'fail': 'LOCATE COOKIE SHEET'})

        grabbing_fsm = assemble_grab_rim_fsm(use_all_clockpos = True)
        smach.StateMachine.add('GRAB CSHEET', grabbing_fsm,
                                transitions={'grabbing_success':'LIFT',
                                             'grabbing_failure': pre_fail_state},
                                remapping={'object_of_desire':'cookie_sheet'})
        grabbing_fsm.userdata.object_of_desire_name = 'cookie_sheet'

        smach.StateMachine.add('LIFT', OvenChangeGripperObjHeight(),
                                transitions={'success':'ROTATE',
                                             'fail': pre_fail_state})

        smach.StateMachine.add('ROTATE', RotateToPreOvenPose(),
                                transitions={'success':'MOVE ARM BACK',
                                             'fail': 'MOVE ARM BACK'})

        smach.StateMachine.add('MOVE ARM BACK', MoveArmBack(),
                                transitions={'done':'RETREAT FROM TABLE',
                                             'fail': 'RETREAT FROM TABLE'})

        smach.StateMachine.add('OVENING FAIL CLEANUP', CleanupLogging(False),
                                transitions={'done':fail_state})

        smach.StateMachine.add('OVENING SUCCESS CLEANUP', CleanupLogging(True),
                                transitions={'done':'ovening_success'})
    return sm



if __name__ == '__main__':
    rospy.init_node('ovening_fsm')
    try:
        sm = assemble_ovening_fsm()
        sis = smach_ros.IntrospectionServer('introspection_server', sm, '/OVEN')
        sis.start()
        #rospy.spin()
        outcome = sm.execute()
    finally:
        sis.stop()

