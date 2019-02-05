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
from baking_states import *

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

class BakingStateMachine():
#TODO: put flag for view_predicates in central location
    def __init__(self, ingredient_filter, debug = True):
        self.state_list = list()
        self.debug = debug
        self.ingredient_filter = ingredient_filter
        self.fail_state = 'failure'
        self.success_state = 'success'

    def execute(self, user_instructions = None):
        sm = self._get_parametrized_state_machine()
        if len(self.state_list) == 0:
            first_state_name = self.success_state
        else:
            first_state_name = self.state_list[0].get_name_str()
        state_list = list()
        with sm:
            smach.StateMachine.add('ROBOT INIT', RobotInitializeState(self.debug),
                                    transitions={'success':'USER INIT',
                                                 'fail':self.fail_state})
            state_list.append('robot init')
            smach.StateMachine.add('USER INIT', UserInitializeState(user_instructions, self.debug),
                                    transitions={'scan':'SCAN TABLETOP',
                                                 'load':'LOAD FROM FILE'})
            state_list.append('user init')
            smach.StateMachine.add('SCAN TABLETOP', ScanTabletop(self.ingredient_filter),
                                    transitions={'success':first_state_name,
                                                 'fail':self.fail_state})
            state_list.append('scan OR load')
            smach.StateMachine.add('LOAD FROM FILE', LoadObjectsFromFile(self.ingredient_filter),
                                    transitions={'success': first_state_name,
                                                 'failure': 'LOAD FROM FILE'})
            for i, fstate in enumerate(self.state_list):
                fstate_name = fstate.get_name_str()
                if i + 1 >= len(self.state_list):
                    success = self.success_state
                else:
                    success = self.state_list[i+1].get_name_str()
                state_list.append(fstate_name)
                smach.StateMachine.add(fstate_name, fstate,
                                       transitions={'success':success,
                                                    'failure':self.fail_state})
        print 'about to execute these states sequentially:'
        for state in state_list:
            print state
        print '\n\n'
        if self.debug:
            raw_input('press enter to confirm...')
        outcome = sm.execute()
        print 'state machine outcome:', outcome
        return outcome

    def _get_parametrized_state_machine(self):
        sm = smach.StateMachine(outcomes = [self.success_state, self.fail_state])
        sm.userdata.nominal_mixing_bowl_x = 0.40
        sm.userdata.nominal_mixing_bowl_y = -0.25
        sm.userdata.pre_mix_height_above_table = .6
        sm.userdata.table_frame_x_offset = 0
        sm.userdata.table_frame_y_offset = 0
        sm.userdata.acceptable_height_above_table = .5
        sm.userdata.pour_height_above_centroid = .4
        sm.userdata.mixing_laps = 3
        sm.userdata.arms_to_try = (1,1)
        sm.userdata.acceptable_height_above_table = 0.45
        sm.userdata.table_height = .65
        sm.userdata.pour_height_above_centroid = .4
        sm.userdata.bowl_radius = .1 
        sm.userdata.mixing_area_pose_x_tol = (0.05, 0.05)
        sm.userdata.mixing_area_pose_y_tol = (0.05, 0.05)
        sm.userdata.mixing_area_pose_z_tol = (0.10, 0)
        sm.userdata.table_frame_x_offset = 0
        sm.userdata.table_frame_y_offset = 0
        sm.userdata.delta_x = 0
        sm.userdata.delta_y = 0.1
        sm.userdata.max_right_movement = 0.5
        sm.userdata.max_left_movement = 0.5
        sm.userdata.current_known_pose_name = 'unknown'
        sm.userdata.mixing_bowl_radius = 0.13  
        sm.userdata.nominal_cookie_sheet_x = .65 
        sm.userdata.nominal_cookie_sheet_y = 0
        sm.userdata.max_left_movement = .4
        sm.userdata.cookie_sheet_radius = 0.1225  
        sm.userdata.grasp_with_right_hand = False
        sm.userdata.nominal_retreat_arm_delta = -0.15 # TODO need ot measure this
        sm.userdata.insert_sheet_delta = 0.15
        sm.userdata.table_retreat_dist = 0.3#TODO need to figure out what this distance is
        sm.userdata.nominal_cookie_sheet_pos = (.65, 0)
        sm.userdata.oven_area_pos = (-.42, 2.00) # used to be -.38, 1.95
        sm.userdata.table_area_pos = (0, .20)
        return sm

    def deal_ingredient(self, ingred_name):
        substates = list()
        substates.append(DealGrab(ingred_name))
        substates.append(DealLift())
        substates.append(DealRotateToKnown())
        substates.append(DealRotateToKnown())
        substates.append(DealRotateToKnown())
        substates.append(DealRotateToKnown())
        substates.append(DealRotateToKnown())
        substates.append(DealTransit())
        substates.append(DealPour())
        substates.append(DealRevert())
        substates.append(DealDump())
        substates.append(DealResetArm())
        goal = '(:goal (and (poured-out ingred) (reset gripper)))'
        i = len(self.state_list)
        name = str(i)+'deal'+ingred_name 
        predicate_estimator = DealBowlPredicateEstimator(domain=name)
        self.state_list.append(FStateMachine(name, substates, predicate_estimator, goal, max_recursion_depth=25, view_predicates=False))

    def mix(self):
        substates = list()
        substates.append(MixServoToMB())
        substates.append(MixServoFromMB())
        substates.append(MixGrabMB())
        substates.append(MixTransitSpoon())
        substates.append(MixPlungeSpoon())
        substates.append(Mix(debug=True))
        substates.append(MixDeplunge())
        substates.append(MixRetreatSpoon())
        substates.append(MixRetreatGrab())
        substates.append(MixResetArm('first'))
        substates.append(MixResetArm('second'))
        substates.append(MixResetArm('third'))
        substates.append(MixResetArm('fourth'))
        predicate_estimator = MixPredicateEstimator(domain='mix')
        i = len(self.state_list)
        name = str(i)+'mix'
        goal = '(:goal (and (mixed mb) (not (mix-zone mb)) (reset larm) (reset rarm)))'
        self.state_list.append(FStateMachine(name, substates, predicate_estimator, goal, view_predicates=True))

    def scrape(self):
        substates = list()
        substates.append(ScrapeGrab())
        substates.append(ScrapeLift())
        substates.append(ScrapeRotate())
        substates.append(ScrapeServoToCS())
        substates.append(ScrapeServoFromCS())
        substates.append(ScrapeTransitAside())
        substates.append(ScrapeTransit())
        substates.append(ScrapePour())
        substates.append(ScrapeTransitSpoon())
        substates.append(ScrapePlungeSpoon())
        substates.append(ScrapeSpoon())
        substates.append(ScrapeDeplungeSpoon())
        substates.append(ScrapeRetreatSpoon())
        substates.append(ScrapeRevert())
        substates.append(ScrapeDump())
        substates.append(ScrapeResetArm('first'))
        substates.append(ScrapeResetArm('second'))
        substates.append(ScrapeResetArm('third'))
        substates.append(ScrapeResetArm('fourth'))
        predicate_estimator = ScrapePredicateEstimator(domain='scrape')
        i = len(self.state_list)
        name = str(i)+'scrape'
        goal = '(:goal (and (scraped mb) (reset larm) (reset rarm)))'
        self.state_list.append(FStateMachine(name, substates, predicate_estimator, goal, view_predicates=False))

    def bake(self, duration_min):
        substates = list()
        substates.append(OvenDriveToOven())
        substates.append(OvenDriveToOven())
        substates.append(OvenAlign())
        substates.append(OvenAlign())
        substates.append(OvenOpenOven())
        substates.append(OvenOpenOven())
        substates.append(OvenDriveToTable())
        substates.append(OvenDriveToTable())
        substates.append(OvenOveningGrab())
        substates.append(OvenPutInOven())
        substates.append(OvenCloseOven())
        substates.append(OvenWait(self.debug, duration_min))
        predicate_estimator = OveningPredicateEstimator(domain='oven')
        i = len(self.state_list)
        name = str(i)+'oven'
        goal = '(:goal (and (cooked cs) (open oven)))'
        self.state_list.append(FStateMachine(name, substates, predicate_estimator, goal, view_predicates=False))

    def unsupported_operation(self, instruction_line):
        self.state_list.append(UserOperationState(instruction_line))
        
if __name__ == '__main__':
    rospy.init_node('testing')
    bsm = BakingStateMachine(ClockwiseIngredientFilter(), debug = True)
    #bsm.deal_ingredient('sugar')
    #bsm.mix()
    #bsm.deal_ingredient('flour')
    #bsm.deal_ingredient('cocoa')
    #bsm.deal_ingredient('krispies')
    #bsm.mix()
    #bsm.scrape()
    bsm.bake(20)
    bsm.execute()
