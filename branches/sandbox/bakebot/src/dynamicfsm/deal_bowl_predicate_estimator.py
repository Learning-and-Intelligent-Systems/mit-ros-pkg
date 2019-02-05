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

class DealBowlPredicateEstimator(PredicateEstimator):
    def __init__(self, domain='deal_bowl'):
        self.domain = domain
        self.bowl_to_deal = None
        self.in_keys = ['mixing_bowl', 'acceptable_height_above_table', 'table_height']
        
    def get_in_keys(self):
        return self.in_keys

    def set_bowl_to_deal(self, bowl_to_deal):
        self.bowl_to_deal = bowl_to_deal

    def rename_variables(self, predicate_str, var_lst):
        tokens = predicate_str.split()
        if (len(tokens) - 1) > len(var_lst):
            print var_lst
            raise AttributeError('not enough variables in var_lst to satisfy the predicate')
        if len(tokens) > 2:
            new_predicate_str = tokens[0]
            for i, token in enumerate(tokens[1::]):
                new_predicate_str = new_predicate_str + ' '+var_lst[i]
            new_predicate_str = new_predicate_str + ')'
            #print 'converted ', predicate_str, 'to', new_predicate_str
            return new_predicate_str
        else:
            predicate = tokens[0].strip('(')
            gripper_predicate_lst = ['GRIPPER', 'reset', 'free']
            if predicate in gripper_predicate_lst:
                return '(' + predicate + ' ' + 'gripper)'
            else:
                return '(' + predicate + ' ' + 'ingred)'

    def generate_problem_str(self, userdata, predicates, goal_str):
        def_str = 'define (problem deal-1-bowl)'
        dom_str = '(:domain ' + self.domain + ')'
        # this is a nasty hack
        if self.bowl_to_deal == None:
            ingred = 'ingred'
        else:
            ingred = self.bowl_to_deal
        obj_str = '(:objects '+str(ingred)+' gripper)'
        str_values = list()
        for predicate_str in predicates:
            str_values.append((predicate_str, self.get_predicate_value(userdata, predicate_str)))
        init_str = '(:init '
        for str_value in str_values:
            predicate_str = str_value[0]
            var_list = ['gripper', 'ingred']
            mod_predicate_str = self.rename_variables(predicate_str, var_list)
            if str_value[1]:
                init_str = init_str + ' ' + mod_predicate_str
            else:
                init_str = init_str + ' (not ' + mod_predicate_str + ')'
        init_str = init_str + ')'
        problem_str = '(' + def_str + '\n\t' + dom_str + '\n\t' + obj_str + '\n\t' + init_str + '\n\t' + goal_str + ')'
        return problem_str

    def rename_and_expand_predicate_list(self, predicate_list):
        # for dealing the bowl there is not any ambiguity within the predicates,
        # for example, (reset ?a) can only refer to (reset ?gripper), therefore
        # renaming and expandig is not necessary
        return predicate_list

    def get_predicate_value(self, userdata, predicate_str):
        tokens = predicate_str.split()
        predicate = tokens[0].strip('(')
        print 'checking', predicate_str, 'for', predicate
        arm_client = ArmClient.get_arm_client()

        if predicate == 'reset':
            isRightArm = False #TODO: do string parsing on predicate str to see which arm
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
        elif predicate == 'free':
            # check to see what the pose of the hand is (and where it is)
            return not self.get_predicate_value(userdata, 'carry')
        elif predicate == 'on-table':
            # check to see if I can find the ingredient with the object detection
            return not (self.get_predicate_value(userdata, 'carry') and self.get_predicate_value(userdata, 'in-transit'))
        elif predicate == 'poured-out':
            # has the ingredient ever been inverted?
            return False
        elif predicate == 'in-transit':
            # free is false and I am within some region above the table
            isRightArm = False 
            (trans, rot) = arm_client.get_transform(isRightArm)
            z_pos = trans[2]
            des = userdata.table_height + userdata.acceptable_height_above_table
            #des = .8
            dist = math.fabs(des- z_pos)
            thresh = .05
            print 'height', z_pos, 'des', des, 'diff', dist, 'thresh', thresh
            return dist < thresh
        elif predicate == 'over-mb':
            isRightArm = False 
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
        elif predicate == 'cardinal':
            return False




            # check the pose of the hand and compare it to the arm client cardinal poses (and has bowl)
            isRightArm = False
            (trans, rot) = arm_client.get_transform(isRightArm)
            pose_iterator = GripperBowlPoseManager.ALL_POSES.itervalues()
            threshold = .1
            while True:
                try:
                    pose = pose_iterator.next()
                    close_enough = True
                    for actual, desired in zip(rot, pose):
                        close_enough = close_enough and (math.fabs(actual-desired) < threshold)
                    if close_enough:
                        print 'I am at this pose:', pose, rot
                        # True, now I just need to make sure I am not carrying anything
                        return self.get_predicate_value(userdata, 'carry')
                except StopIteration:
                    break
            return False
        elif predicate == 'randomp':
            return not self.get_predicate_value(userdata, 'cardinal')
        elif predicate == 'inverted':
            # check the pose of the hand
            return False
        elif predicate == 'done':
            # check to see if the ingredient has ever been dumped
            return False
        else:
            print 'in the else for', predicate
            return True
            #raise KeyError('there was no matching predicate in deal bowl predicate estimator: '+predicate_str)

