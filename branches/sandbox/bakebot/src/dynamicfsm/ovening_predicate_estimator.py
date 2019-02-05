#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
from smachforward import *
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

class OveningPredicateEstimator(PredicateEstimator):
    def __init__(self, domain='mix'):
        self.domain = domain
        self.in_keys = ['cookie_sheet', 'mixing_bowl', 'acceptable_height_above_table', 'table_height', 'nominal_mixing_bowl_x', 'nominal_mixing_bowl_y', 'nominal_cookie_sheet_x', 'nominal_mixing_bowl_y', 'oven_area_pos']
        self.papm = PickAndPlaceManager.get_pick_and_place_manager()
        self.btom = BroadTabletopObjectManager.get_broad_tabletop_object_manager(self.papm)
        
    def get_in_keys(self):
        return self.in_keys

    def generate_problem_str(self, userdata, predicates, goal_str):
        # based on how fstate_machine is setup I expect predicates to already be expanded and renamed
        def_str = 'define (problem oven)'
        dom_str = '(:domain ' + self.domain + ')'
        obj_str = '(:objects cs robot oven)'
        str_values = list()
        for predicate_str in predicates:
            # (example) predicate_str = (reset ?rarm)
            str_values.append((predicate_str, self.get_predicate_value(userdata, predicate_str)))
        init_str = '(:init '
        for str_value in str_values:
            mod_predicate_str = str_value[0]
            if str_value[1]:
                init_str = init_str + ' ' + mod_predicate_str
            else:
                init_str = init_str + ' (not ' + mod_predicate_str + ')'
        init_str = init_str + ')'
        problem_str = '(' + def_str + '\n\t' + dom_str + '\n\t' + obj_str + '\n\t' + init_str + '\n\t' + goal_str + ')'
        return problem_str

    def rename_and_expand_predicate_list(self, predicate_list):
        renamed_and_expanded = list()
        for predicate in predicate_list:
            tokens = predicate.split()
            fxn = tokens[0].strip('(')
            if fxn == 'ROBOT':
                renamed_and_expanded.append('(ROBOT robot)')
            elif fxn == 'OVEN':
                renamed_and_expanded.append('(OVEN oven)')
            elif fxn == 'COOKIE-SHEET':
                renamed_and_expanded.append('(COOKIE-SHEET cs)')
            elif fxn == 'aligned':
                renamed_and_expanded.append('(aligned robot oven)')
            elif fxn == 'at-oven':
                renamed_and_expanded.append('(at-oven robot)')
            elif fxn == 'at-table':
                renamed_and_expanded.append('(at-table robot)')
            elif fxn == 'open':
                renamed_and_expanded.append('(open oven)')
            elif fxn == 'in-oven':
                renamed_and_expanded.append('(in-oven cs)')
            elif fxn == 'carry':
                renamed_and_expanded.append('(carry robot cs)')
            elif fxn == 'cooked':
                renamed_and_expanded.append('(cooked cs)')
            else:
                raise KeyError('did not match this predicate: ', predicate)
        return renamed_and_expanded

    def get_predicate_value(self, userdata, predicate_str):
        try:
            temp = userdata.mixing_bowl
        except KeyError:
            print 'mixing bowl not yet set'
            if not self.btom.filtered:
                self.btom.filter_ingredients()
            userdata.mixing_bowl = self.btom.mixing_bowl
        try:
            temp = userdata.cookie_sheet
        except KeyError:
            print 'cookie_sheet not yet set'
            if not self.btom.filtered:
                self.btom.filter_ingredients()
            userdata.cookie_sheet = self.btom.cookie_sheet

        # now thanks to the renaming and expanding of the predicates
        # i can count on it being (reset ?rarm)
        arm_client = ArmClient.get_arm_client()
        base_client = BaseClient.get_base_client()
        tokens = predicate_str.split()
        predicate = tokens[0].strip('(')
        print '\n\n\nchecking', predicate_str, 'for', predicate, '\n'
        if predicate == 'aligned':
            return False
        elif predicate == 'at-oven':
            (trans, rot) = base_client.get_transform()
            x = trans[0]
            y = trans[1]
            xdes, ydes = userdata.oven_area_pos
            tol = .3
            print 'x, y', x, y, 'xdes, ydes', xdes, ydes, 'tol', tol
            return (math.fabs(x - xdes) < tol and math.fabs(y - ydes) < tol)
        elif predicate == 'at-table':
            (trans, rot) = base_client.get_transform()
            x = trans[0]
            y = trans[1]
            xdes, ydes = (0,0)
            tol = .3
            print 'x, y', x, y, 'xdes, ydes', xdes, ydes, 'tol', tol
            return (math.fabs(x - xdes) < tol and math.fabs(y - ydes) < tol)
        elif predicate == 'open':
            return False
        elif predicate == 'in-oven':
            return False
        elif predicate == 'carry':
            # check the width of the fingers here 
            jrp = JointRecorderPlayer()
            left = jrp.get_gripper_state(False)
            right = jrp.get_gripper_state(True)
            # I only care about the left
            joint = left['l_gripper_l_finger_joint']
            print 'joint', joint
            jmin = 0.01
            jmax = .4
            # if less than min, hand closed, open if greater than max
            return (not ((joint < jmin) or (joint > jmax))) and not self.get_predicate_value(userdata, 'reset')
        elif predicate == 'cooked':
            return False
        else:
            print 'in the else for', predicate
            return True

