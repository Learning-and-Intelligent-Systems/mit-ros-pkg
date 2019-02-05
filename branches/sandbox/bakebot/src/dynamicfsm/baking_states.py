#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')

from smachforward import *
from deal_bowl_states import *
from ovening_states import *
from mix_states import *
from scraping_states import *
from deal_bowl_predicate_estimator import *
from mix_predicate_estimator import *
from scraping_predicate_estimator import *
from ovening_predicate_estimator import *

import rospy
import tf
import actionlib
from clients.arm_client import * 
from clients.base_client import BaseClient
from clients.speech_client import *
from utilities.bakebot_pick_and_place_manager import *
from utilities.broad_table_object_manager import *
from utilities.bowl_dealer_utilities import *
from utilities.joint_recorder_player import *
import math
import smach
import smach_ros
import time 

class RobotInitializeState(smach.State):
    def __init__(self, debug = True):
        smach.State.__init__(self, outcomes=['success', 'fail'])
        self.debug = debug

    def execute(self, userdata):
        logger = EventLoggerClient.startfsm('RobotInitialzeState (bakebot fsm)')
        rospy.loginfo('Executing state RobotInitializeState')
        papm = PickAndPlaceManager.get_pick_and_place_manager()
        rospy.logwarn('clearing collision map')
        papm.reset_collision_map()

        rospy.loginfo('**************** CHECK TERMINAL OUTPUT **********')
        print '\n\n\n\n\nWARNING: the arms will swing back to the side.  make sure they are moved so nothing is hit!\n\n\n\n\n'
        if self.debug:
            choice = raw_input('\n\npress enter when clear (or (s)kip): ')
            if len(choice) == 1:
                rospy.loginfo('skipping')
                logger.stops()
                return 'success'

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

class UserInitializeState(smach.State):
    def __init__(self, user_instructions = None, debug = False):
        smach.State.__init__(self, outcomes=['scan', 'load'],
                                   input_keys=['save_detected_objects_filename', 'load_detected_objects_filename'],
                                   output_keys=['save_detected_objects_filename', 'load_detected_objects_filename', 'mixing_laps', 'branch_outcome'])
        self.btom = BroadTabletopObjectManager.get_broad_tabletop_object_manager(PickAndPlaceManager.get_pick_and_place_manager())
        self.user_instructions = user_instructions
        self.debug = debug

    def execute(self, userdata):
        rospy.loginfo('Executing state UserInitializeState')
        logger = EventLoggerClient.startfsm('UserInitializeState (bakebot fsm)')
        print '\n\n\n\n'
        print 'Welcome to the BAKEBOT Finite State Machine\n'
        print '\n\n'

        if self.user_instructions is not None:
            print 'setup instructions:'
            for inst in user_instructions:
                print '\t'+inst
            raw_input('\npress enter to continue...')

        if not self.debug: 
            logger.stops()
            return 'scan'

        choice = raw_input('Would you like to (s)can, (l)oad: ')

        if choice == 's':
            print '\nscanning the table...\n\n\n'
            logger.stops()
            return 'scan'
        elif choice == 'l':
            print '\nloading the table...\n\n\n'
            logger.stops()
            return 'load'
        else:
            logger.stopf()
            return self.execute(userdata)


class UserOperationState(smach.State):
    def __init__(self, instruction_line):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.instruction_line = str(instruction_line)

    def execute(self, userdata):
        rospy.loginfo('Executing state UserOperationState')
        logger = EventLoggerClient.startfsm('UserOperationState (bakebot fsm)')
        speech_client = SpeechClient.get_speech_client()
        print '\n\n\n'
        print '********************************************************************'
        print ''
        message = 'Dear Human Overlord,'
        message += '\n\n'
        message += ('I need your help executing this instruction: '+self.instruction_line)
        print message
        speech_client.speak(message, suppress_text_output=True)
        rospy.sleep(3)
        response = raw_input('\nDid you successfully accomplish '+self.instruction_line+'? [Y/n]: ')
        if len(response) == 0:
            ret = 'success'
        elif response[0] == 'n' or 'N':
            ret = 'failure'
        else:
            ret = 'success'
        speech_client.speak(self.instruction_line, suppress_text_output=True)
        message = 'Thank you for your help!'
        print '\n' + message
        print ''
        print '********************************************************************'
        print '\n\n\n'
        speech_client.speak(message, suppress_text_output=True)
        rospy.sleep(5)
        return 'success'

    def get_name_str(self):
        return 'user_operation_state: '+self.instruction_line


class ScanTabletop(smach.State):
    def __init__(self, ingredient_filter = None):
        smach.State.__init__(self, outcomes=['success', 'fail'], 
                             output_keys = ['mixing_bowl', 'cookie_sheet'])
        self.btom = BroadTabletopObjectManager.get_broad_tabletop_object_manager(PickAndPlaceManager.get_pick_and_place_manager())
        self.ingredient_filter = ingredient_filter

    def execute(self, userdata):
        rospy.loginfo('Executing state ScanTabletop')
        logger = EventLoggerClient.startfsm('ScanTabletop (bakebot fsm)')

        filename = raw_input('enter the filename to save to (blank for detectedobj.log): ')
        if len(filename) == 0:
            filename = 'detectedobj.log'
        print 'filename: ', filename
        rospy.logwarn('saving the detected objects to: ' + filename)

        self.btom.do_broad_tabletop_object_detection()  # stores the result internally
        status = self.btom.save_detected_objects_to_file(filename)
        print '\n\n\n\n***************************************************************************************'
        raw_input('detected objects saved.  press enter to continue...')
        if not status:
            rospy.logwarn('saving detected objects failed')
        else:
            rospy.loginfo('not saving detected objects')
        logger.stops()

        self.btom.filter_ingredients(ingredient_filter = self.ingredient_filter)
        userdata.mixing_bowl = self.btom.mixing_bowl
        userdata.cookie_sheet = self.btom.cookie_sheet
        return 'success'

class LoadObjectsFromFile(smach.State):
    def __init__(self, ingredient_filter = None):
        smach.State.__init__(self, outcomes=['success', 'failure'],
                                   input_keys=['bowl_to_deal', 'nominal_mixing_bowl_x', 'nominal_mixing_bowl_y', 'nominal_cookie_sheet_x', 'nominal_cookie_sheet_y'],
                                   output_keys=['bowl_to_deal', 'cookie_sheet', 'mixing_bowl'])
        self.btom = BroadTabletopObjectManager.get_broad_tabletop_object_manager(PickAndPlaceManager.get_pick_and_place_manager())
        self.ingredient_filter = ingredient_filter

    def execute(self, userdata):
        rospy.loginfo('Executing state LoadObjectsFromFile')
        filename = raw_input('enter the filename to load from (blank for detectedobj.log): ')
        do_filter = False
        if len(filename) == 0:
            filename = 'detectedobj.log'
        elif len(filename) == 1:
            filename = 'detectedobj.log'
            #do_filter = True
            do_filter = True
        print 'filename: ', filename
        if self.btom.load_detected_objects_from_file(filename): 
            detected_objects = self.btom.detected_objects
            for i,obj in enumerate(detected_objects):
                print i, obj
            #if not self.btom.filtered and do_filter:
            self.btom.filter_ingredients(ingredient_filter = self.ingredient_filter)
            mix_area_x = userdata.nominal_mixing_bowl_x
            mix_area_y = userdata.nominal_mixing_bowl_y
            #self.btom.mixing_bowl.pose.pose.position.x = mix_area_x
            #self.btom.mixing_bowl.pose.pose.position.y = mix_area_y
            userdata.mixing_bowl = self.btom.mixing_bowl
            cs_area_x = userdata.nominal_cookie_sheet_x
            cs_area_y = userdata.nominal_cookie_sheet_y
            #self.btom.cookie_sheet.pose.pose.position.x = cs_area_x
            #self.btom.cookie_sheet.pose.pose.position.y = cs_area_y
            userdata.cookie_sheet = self.btom.cookie_sheet
            #choice = raw_input('which item: ')
            #userdata.bowl_to_deal = detected_objects[int(choice)]
            #print 'chose to deal: ', userdata.bowl_to_deal
            ##self.btom.filter_ingredients()
            #userdata.mixing_bowl = self.btom.mixing_bowl
            #userdata.cookie_sheet = self.btom.cookie_sheet
            return 'success'
        else:
            return 'fail'
