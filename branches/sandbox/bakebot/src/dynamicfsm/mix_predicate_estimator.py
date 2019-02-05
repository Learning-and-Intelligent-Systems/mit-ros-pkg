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

class MixPredicateEstimator(PredicateEstimator):
    def __init__(self, domain='mix'):
        self.domain = domain
        self.in_keys = ['mixing_bowl', 'acceptable_height_above_table', 'table_height', 'nominal_mixing_bowl_x', 'nominal_mixing_bowl_y']
        self.papm = PickAndPlaceManager.get_pick_and_place_manager()
        self.btom = BroadTabletopObjectManager.get_broad_tabletop_object_manager(self.papm)
        
    def get_in_keys(self):
        return self.in_keys

    def generate_problem_str(self, userdata, predicates, goal_str):
        # based on how fstate_machine is setup I expect predicates to already be expanded and renamed
        def_str = 'define (problem mix)'
        dom_str = '(:domain ' + self.domain + ')'
        obj_str = '(:objects mb larm rarm)'
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
            if fxn == 'GRIPPER':
                renamed_and_expanded.append('(GRIPPER larm)')
                renamed_and_expanded.append('(GRIPPER rarm)')
            elif fxn == 'MIXING-BOWL':
                renamed_and_expanded.append('(MIXING-BOWL mb)')
            elif fxn == 'SPATULA-GRIPPER':
                renamed_and_expanded.append('(SPATULA-GRIPPER rarm)')
            elif fxn == 'reset':
                renamed_and_expanded.append('(reset larm)')
                renamed_and_expanded.append('(reset rarm)')
            elif fxn == 'carry':
                renamed_and_expanded.append('(carry larm mb)')
                renamed_and_expanded.append('(carry rarm mb)')
            elif fxn == 'free':
                renamed_and_expanded.append('(free larm)')
                renamed_and_expanded.append('(free rarm)')
            elif fxn == 'over-mb':
                renamed_and_expanded.append('(over-mb larm)')
                renamed_and_expanded.append('(over-mb rarm)')
            elif fxn == 'reset-safe':
                renamed_and_expanded.append('(reset-safe larm)')
                renamed_and_expanded.append('(reset-safe rarm)')
            elif fxn == 'in-mb':
                renamed_and_expanded.append('(in-mb larm)')
                renamed_and_expanded.append('(in-mb rarm)')
            elif fxn == 'mixed':
                renamed_and_expanded.append('(mixed mb)')
            elif fxn == 'mix-zone':
                renamed_and_expanded.append('(mix-zone mb)')
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

        # now thanks to the renaming and expanding of the predicates
        # i can count on it being (reset ?rarm)
        arm_client = ArmClient.get_arm_client()
        tokens = predicate_str.split()
        predicate = tokens[0].strip('(')
        print 'checking', predicate_str, 'for', predicate
        if predicate == 'reset':
            # check the pose of the arm here
            arm = tokens[1].strip(')')
            isRightArm = (arm == 'rarm')
            (trans, rot) = arm_client.get_transform(isRightArm)
            if not isRightArm:
                x_des, y_des, z_des = 0.06, .57, .8
            else:
                x_des, y_des, z_des = 0.08, -.56, .8
            x, y, z = trans
            dist = math.fabs((x_des-x)**2 + (y_des-y)**2 + (z_des-z)**2)
            thresh = .05
            return dist < thresh
        elif predicate == 'carry':
            arm = tokens[1].strip(')')
            # check the width of the fingers here 
            jrp = JointRecorderPlayer()
            left = jrp.get_gripper_state(False)
            right = jrp.get_gripper_state(True)
            if arm == 'larm':
                joint = left['l_gripper_l_finger_joint']
            else:
                joint = right['r_gripper_l_finger_joint']
            jmin = 0.01
            jmax = .4
            value =(not ((joint < jmin) or (joint > jmax)))  
            print 'joint', joint, 'min', jmin, 'max', jmax, value
            # if less than min, hand closed, open if greater than max
            return (not ((joint < jmin) or (joint > jmax))) and not self.get_predicate_value(userdata, ('reset '+arm))
        elif predicate == 'free':
            arm = tokens[1].strip(')')
            return not self.get_predicate_value(userdata, ('carry '+arm))
        elif predicate == 'over-mb':
            arm = tokens[1].strip(')')
            isRightArm = (arm == 'rarm')
            (trans, rot) = arm_client.get_transform(isRightArm)
            x_pos, y_pos, z_pos = trans
            mixing_bowl_pose = userdata.mixing_bowl.pose.pose.position
            rospy.loginfo('setting to use a new pose manager')
            x_area = mixing_bowl_pose.x
            y_area = mixing_bowl_pose.y
            z_area = userdata.acceptable_height_above_table + userdata.table_height
            #print 'z_area', z_area
            #z_area = .8
            print 'z_area', z_area
            dist = math.sqrt((x_area-x_pos)**2 + (y_area-y_pos)**2 + (z_area-z_pos)**2)
            thresh = .3 
            print 'distance from mixing area center [m]', dist
            print 'threshold dist:', thresh
            return dist < thresh
        elif predicate == 'reset-safe':
            arm = tokens[1].strip(')')
            isRightArm = (arm == 'rarm')
            in_mb = self.get_predicate_value(userdata, 'in-mb ' + arm)
            carry = self.get_predicate_value(userdata, 'carry ' + arm)
            return (not in_mb) and (not carry)
        elif predicate == 'in-mb':
            arm = tokens[1].strip(')')
            isRightArm = (arm == 'rarm')
            (trans, rot) = arm_client.get_transform(isRightArm)
            x_pos, y_pos, z_pos = trans
            mixing_bowl_pose = userdata.mixing_bowl.pose.pose.position
            rospy.loginfo('setting to use a new pose manager')
            x_area = mixing_bowl_pose.x
            y_area = mixing_bowl_pose.y
            z_area = 0.2 + userdata.table_height #TODO need to fix the distance
            #print 'z_area', z_area
            #z_area = .8
            print 'z_area', z_area
            dist = math.sqrt((x_area-x_pos)**2 + (y_area-y_pos)**2 + (z_area-z_pos)**2)
            thresh = .3 if not isRightArm else .15
            print 'distance from mixing area center [m]', dist
            print 'threshold dist:', thresh
            if isRightArm:
                return dist < thresh
            else:
                print 'doing height thresh for left arm'
                return dist < thresh and z_pos < z_area
        elif predicate == 'mix-zone':
            print 'mix zone'
            x = userdata.mixing_bowl.pose.pose.position.x
            y = userdata.mixing_bowl.pose.pose.position.y
            desired_x = userdata.nominal_mixing_bowl_x
            desired_y = userdata.nominal_mixing_bowl_y
            dist = math.sqrt((x-desired_x)**2 + (y-desired_y)**2)
            thresh = .15
            print 'desired:', desired_x, desired_y, 'actual:', x,y, 'dist: ', dist, 'thresh: ', thresh
            return dist < thresh
            #return False
        elif predicate == 'mixed':
            return False
        else:
            print 'in the else for', predicate
            return True

