#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
from smachforward import *
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
from staticfsm.grab_rim_fsm import *
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

class OvenDriveToOven(FState):
    def __init__(self):
        FState.__init__(self, 
                             input_keys=['oven_area_pos'])
    def execute(self, userdata):
        rospy.loginfo('Executing state DriveToOven')  
        logger = EventLoggerClient.startfsm('DriveToOven (ovening fsm)')
        bc = BaseClient.get_base_client()
        (x, y) = userdata.oven_area_pos
        status = bc.go_to_pos(x, y)
        if not status:
            logger.stopf()
            return 'failure'
        else:
            logger.stops()
            return 'success'

    def get_name_str(self):
        return 'drive-to-oven'

    def get_params_str(self):
        return '(?x)'

    def get_precondition_str(self):
        return '(ROBOT ?x)'

    def get_effect_str(self):
        return '(and (at-oven ?x) (not (at-table ?x)))'

class OvenAlign(FState):
    def __init__(self, debug = False):
        FState.__init__(self)

    def execute(self, userdata):
        rospy.loginfo('Executing state AlignRobotWithOven')  
        logger = EventLoggerClient.startfsm('AlignRobotWithOven (ovening fsm)')
        logger.stops()
        return 'success'

    def get_name_str(self):
        return 'align'

    def get_params_str(self):
        return '(?x ?y)'

    def get_precondition_str(self):
        return '(and (ROBOT ?x) (OVEN ?y) (at-oven ?x))'

    def get_effect_str(self):
        return '(and (aligned ?x ?y))'

class OvenOpenOven(FState):
    def __init__(self):
        FState.__init__(self)

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
                return 'failure'
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
            return 'failure'
        else:
            logger.stops()
            #return 'wait'

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
        papm.move_arm_to_side(0)
        papm.move_arm_to_side(1)
        if status:
            logger.stops()
            return 'success'
        else:
            logger.stopf()
            return 'fail'

    def get_name_str(self):
        return 'open-oven'

    def get_params_str(self):
        return '(?x ?y ?z)'

    def get_precondition_str(self):
        return '(and (ROBOT ?x) (OVEN ?y) (COOKIE-SHEET ?z) (aligned ?x ?y) (not (carry ?x ?z)))'

    def get_effect_str(self):
        return '(and (open ?y))'

class OvenDriveToTable(FState):
    def __init__(self):
        FState.__init__(self, 
                             input_keys=['table_area_pos', 'table_retreat_dist'])
    def execute(self, userdata):
        rospy.loginfo('Executing state DriveToTable')  
        logger = EventLoggerClient.startfsm('DriveToTable (ovening fsm)')
        bc = BaseClient.get_base_client()
        (x, y) = userdata.table_area_pos
        print 'i want to go to: ', (x, y)
        status = bc.go_to_pos(x, y, x_first = False)
        if not status:
            logger.stopf()
            return 'failure'
        else:
            logger.stops()
            return 'success'

    def get_name_str(self):
        return 'drive-to-table'

    def get_params_str(self):
        return '(?x ?y)'

    def get_precondition_str(self):
        return '(and (ROBOT ?x) (OVEN ?y))'

    def get_effect_str(self):
        return '(and (at-table ?x) (not (at-oven ?x)) (not (aligned ?x ?y)))'

class OvenOveningGrab(FState):
    def __init__(self):
        FState.__init__(self, 
                             input_keys=['cookie_sheet',
                                         'nominal_cookie_sheet_pos',
                                         'table_frame_x_offset',
                                         'acceptable_height_above_table',
                                         'table_height',
                                         'table_frame_y_offset'],
                             output_keys=['cookie_sheet', 'table_frame_y_offset', 'table_frame_x_offset'])
        self.num_attempts = 0
        self.first_time = True
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener

    def execute(self, userdata):
        aligned = False
        i = 0
        while not aligned:
            print 'align loop iteration', i
            i = i + 1
            locate_status = self.locate_cookie_sheet(userdata)
            if locate_status == 'fail':
                print 'failed to locate the cookie sheet'
                return 'failure'
            elif locate_status == 'grab':
                aligned = True
                break
            else:
                self.align_with_cookie_sheet(userdata)
        print '*********************************************************************************'
        print 'prepping grab rim fsm'
        print 'going to grab cookie sheet with collision name: ', userdata.cookie_sheet.collision_name
        grabbing_fsm = assemble_grab_rim_fsm(use_all_clockpos=True)
        print grabbing_fsm.userdata
        grabbing_fsm.userdata.object_of_desire = userdata.cookie_sheet
        grabbing_fsm.userdata.object_of_desire_name = 'cookie_sheet'
        grabbing_fsm.userdata.table_frame_x_offset = userdata.table_frame_x_offset
        grabbing_fsm.userdata.table_frame_y_offset = userdata.table_frame_y_offset
        grabbing_fsm.userdata.grasp_with_right_hand = False
        print grabbing_fsm.userdata
        print 'done setting up grabbing fsm, going to execute...'
        outcome = grabbing_fsm.execute()
# TODO mbollini 2/22/12: it looks like it uses the wrong object_of_desire when the program gets here through normal recipe execution.
# during a fresh restart, it links up fine.
# the wrong userdata.object_of_desire pops up during refineobjectdetection in the grab rim fsm, but looks correct from this side
# I suspect that it is the old object_of_desire from the mixing bowl grab
        if outcome == 'grabbing_success':
            #return 'success'
            pass
        else:
            print 'grabbing failed'
            return 'failure'
        print '*********************************************************************************'
        rospy.loginfo('Executing state ChangeGripperObjHeight')
        papm = PickAndPlaceManager.get_pick_and_place_manager()
        (trans, rot) = self.arm_client.get_transform(False)
        desired_z = userdata.acceptable_height_above_table + papm.table_height
        rospy.loginfo('lifting the arm to desired z: ' + str(desired_z))
        status = self.arm_client.move_to_pose_const_ee(False, trans[0], trans[1], desired_z)
        if status == 0 or status == -2:
            #return 'success'
            pass
        else:
            print 'lifting failed'
            return 'failure'
        rospy.loginfo('Executing state MoveArmBack')
        status = self.arm_client.move_to_pose_const_ee(False, .34, -.06, 1.12)
        if status == 0:
            rospy.loginfo('success')
            #return 'done'
        else:
            status = self.arm_client.move_to_pose_const_ee(False, .34, -.06, 1.12)
            if status == 0:
                rospy.loginfo('success')
                #return 'done'
            else:
                status = self.arm_client.move_to_pose_const_ee(False, .34, -.06, 1.12)
                if status == 0:
                    rospy.loginfo('success')
                    #return 'done'
                else:
                    print 'failed move arm back'
                    return 'failure'
        return 'success'

    def align_with_cookie_sheet(self, userdata):
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

    def locate_cookie_sheet(self, userdata):
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

    def get_name_str(self):
        return 'grab'

    def get_params_str(self):
        return '(?x ?y)'

    def get_precondition_str(self):
        return '(and (ROBOT ?x) (COOKIE-SHEET ?y) (at-table ?x))'

    def get_effect_str(self):
        return '(and (carry ?x ?y))'

class OvenPutInOven(FState):
    def __init__(self):
        FState.__init__(self)

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveToPreOven')  
        logger = EventLoggerClient.startfsm('MoveToPreOven (ovening fsm)')
        bc = BaseClient.get_base_client()
        #bc.fake_rotate(1)
        #bc.translate(True, -.03)
        logger.stops()

        rospy.loginfo('Executing state MoveHandToPreOvenPos')  
        logger = EventLoggerClient.startfsm('MoveHandToPreOvenPos (ovening fsm)')
        rospy.logerr('Not yet implemented')
        self.arm_client = ArmClient.get_arm_client()
        #status = self.arm_client.move_to_pose_const_ee(False, .54, -.09, 1.05)
        status = self.arm_client.move_to_pose_const_ee(False, .55, -.06, 1.06)
        if status == 0:
            rospy.loginfo('done')
            logger.stops()
            #return 'done'
        else:
            status = self.arm_client.move_to_pose_const_ee(False, .55, -.06, 1.06)
            if status == 0:
                rospy.loginfo('done')
                logger.stops()
                #return 'done'
            else:
                logger.stopf()
                return 'failure'
        #return 'done'

        rospy.loginfo('Executing state MoveSheetIntoOven')  
        logger = EventLoggerClient.startfsm('MoveSheetIntoOven (ovening fsm)')
        ac = ArmClient.get_arm_client()
        status = ac.move_to_pose_const_ee(False, .63, -.06, 1.)
        if status == 0:
            logger.stops()
            #return 'done'
        else:
            status = ac.move_to_pose_const_ee(False, .63, -.06, 1.0)
            if status == 0:
                logger.stops()
                #return 'done'
            else:
                return 'failure'

        rospy.loginfo('Executing state LowerSheet')  
        logger = EventLoggerClient.startfsm('LowerSheet (ovening fsm)')
        ac = ArmClient.get_arm_client()
        trans, rot = ac.get_transform(False)
        delta = 1.06- trans[2] # TODO: need to measure this to get it into oven
        print 'delta: ', delta
        status = ac.move_delta_const_ee(False, delta, 'z')
        if status == 0:
            logger.stops()
            #return 'done'
        else:
            rospy.logwarn('unable to lower sheet, trying again')
            status = ac.move_delta_const_ee(False, delta, 'z')
            if status == 0:
                logger.stops()
                #return 'done'
            rospy.logwarn('unable to lower sheet, trying again with smaller delta')
            status = ac.move_delta_const_ee(False, delta + .01, 'z')
            if status == 0:
                logger.stops()
                #return 'done'
            else:
                logger.stopf()
                return 'failure'

        rospy.loginfo('Executing state OpenGripper')  
        logger = EventLoggerClient.startfsm('OpenGripper (ovening fsm)')
        papm = PickAndPlaceManager.get_pick_and_place_manager()
        papm.open_gripper(1)
        logger.stops()
        #return 'done'

        rospy.loginfo('Executing state LiftGripper')  
        logger = EventLoggerClient.startfsm('LiftGripper (ovening fsm)')
        arm_client = ArmClient.get_arm_client()
        (trans, rot) = arm_client.get_transform(False)
        (x, y, z) = trans
        zdes = 1.05
        status = arm_client.move_to_pose_const_ee(False, x, y, zdes)
        if status == 0:
            logger.stops()
            #return 'done'
        else:
            rospy.logwarn('arm client failed to lift')
            status = arm_client.move_to_pose_const_ee(False, x, y, zdes)
            if status == 0:
                logger.stops()
                #return 'done'
            status = arm_client.move_to_pose_const_ee(False, x, y, zdes)
            if status == 0:
                logger.stops()
                #return 'done'
            return 'failure'

        rospy.loginfo('Executing state RetreatArm')  
        logger = EventLoggerClient.startfsm('RetreatArm (ovening fsm)')
        ac = ArmClient.get_arm_client()
        status = ac.move_to_pose_const_ee(False, .39, .41, 1.03)
        if status == 0:
            logger.stops()
            return 'success'
        else:
            status = ac.move_to_pose_const_ee(False, .39, .41, 1.03)
            if status == 0:
                logger.stops()
                return 'success'
            else:
                logger.stopf()
                return 'failure'

    def get_name_str(self):
        return 'put-in-oven'

    def get_params_str(self):
        return '(?x ?y ?z)'

    def get_precondition_str(self):
        return '(and (ROBOT ?x) (OVEN ?y) (COOKIE-SHEET ?z) (at-oven ?x) (open ?y) (carry ?x ?z))'

    def get_effect_str(self):
        return '(and (not (carry ?x ?z)) (in-oven ?z))'

class OvenCloseOven(FState):
    def __init__(self):
        FState.__init__(self, output_keys=['cook_start_time'])

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveRobotToPreClose')  
        logger = EventLoggerClient.startfsm('MoveRobotToPreClose (ovening fsm)')
        logger.stops()
        rospy.loginfo('Executing state CloseOven')  
        logger = EventLoggerClient.startfsm('Oven (ovening fsm)')
        path = os.environ['BAKEBOT_LOG_PATH']
        f = path + '20110630-closeovenjrp.log'
        jrp = JointRecorderPlayer()
        jrp.load_saved_states_from_file(f)
        jrp.playback_all_joints(automatic = True)
        userdata.cook_start_time = rospy.Time.now()
        logger.stops()
        return 'success'

    def get_name_str(self):
        return 'close-oven'

    def get_params_str(self):
        return '(?x ?y)'

    def get_precondition_str(self):
        return '(and (ROBOT ?x) (OVEN ?y) (at-oven ?x) (open ?y))'

    def get_effect_str(self):
        return '(and (not (open ?y)))'

class OvenWait(FState):
    def __init__(self, debug = True, duration_min=20):
        FState.__init__(self, 
                            input_keys=['cook_start_time'])
        self.debug = debug
        self.duration_min = duration_min
    def execute(self, userdata):
        rospy.loginfo('Executing state Wait')  
        logger = EventLoggerClient.startfsm('Wait (ovening fsm)')
        if self.debug:
            choice = raw_input('press any key to skip or enter to wait 20min')
            if len(choice) > 0:
                print 'skipping'
                logger.stops()
                return 'success'
        start = userdata.cook_start_time
        stop = start + rospy.Duration(self.duration_min * 60)
        print stop
        print start
        print rospy.Duration(self.duration_min*60)
        try:
            while rospy.Time.now() < stop and (not rospy.is_shutdown()):
                remaining = stop - rospy.Time.now() 
                rospy.loginfo('I still have to wait ' + str(remaining/10.**9) + ' seconds until cookies')
                rospy.sleep(60)
            rospy.loginfo('done waiting!')
            logger.stops()
            return 'success'
        finally:
            return 'success'

    def get_name_str(self):
        return 'wait'

    def get_params_str(self):
        return '(?x ?y ?z)'

    def get_precondition_str(self):
        return '(and (ROBOT ?x) (OVEN ?y) (COOKIE-SHEET ?z) (not (open ?y)) (in-oven ?z) (at-oven ?x))'

    def get_effect_str(self):
        return '(and (cooked ?z))'

