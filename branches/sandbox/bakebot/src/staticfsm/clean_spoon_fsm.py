#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
import rospy
import tf
import sys
import actionlib
from geometry_msgs.msg import Pose
from object_manipulation_msgs.msg import ManipulationResult, GraspableObject, Grasp
from clients.arm_client import ArmClient
from clients.base_client import BaseClient
from clients.clean_spoon_client import CleaningClient
from utilities.bakebot_pick_and_place_manager import *
from utilities.broad_table_object_manager import *
from clients.pr2cm_client import *
from services.bakebot_logging import *
from clients.mixing_client import *
from utilities.bowl_dealer_utilities import *
from bakebot.srv import *
import math
import smach
import smach_ros
import time
import pickle
from utilities.joint_recorder_player import *


class GetDesiredSpoonOrientation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'plunge'],
                                   output_keys=['barcode_up'])
        self.desired_orientations = list()
        self.desired_orientations.append(True)
        self.desired_orientations.append(False)
        self.iterator = iter(self.desired_orientations)

    def execute(self, userdata):
        rospy.loginfo('Executing state GetDesiredSpoonOrientation')
        try:
            userdata.barcode_up = self.iterator.next()
            return 'plunge'
        except StopIteration:
            print 'resetting iterator for next time'
            self.iterator = iter(self.desired_orientations)
            return 'done'


class MoveToPrePreClean(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                             input_keys=['barcode_up', 'mixing_bowl_radius', 'mixing_bowl_height', 'mixing_bowl'])
        self.ac = ArmClient.get_arm_client()

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveToPrePreClean')
        csc = CleaningClient(no_switch = True)
        mbx = userdata.mixing_bowl.pose.pose.position.x
        mby = userdata.mixing_bowl.pose.pose.position.y
        mbz = userdata.mixing_bowl.pose.pose.position.z
        print 'mixing bowl position xyz: ', mbx, mby, mbz
        mixing_bowl_pos_bl = [mbx, mby, mbz]
        mixing_bowl_pos_tllf = csc.get_bowl_pos_in_tllf(mixing_bowl_pos_bl)
        print 'mixing bowl pos tllf: ', mixing_bowl_pos_tllf
        initial_pose = csc.get_initial_pos(userdata.mixing_bowl_radius, userdata.mixing_bowl_height, 
                                           mixing_bowl_pos_tllf, userdata.barcode_up)
        print 'initial pose from csc: ', initial_pose
        # initial pos is in TLFF
        # want to be a bit higher
        x = mbx
        yoff = (initial_pose[1] - mixing_bowl_pos_tllf[1])
        y = mby + yoff
        print 'yoff', yoff
        print 'y', y
        zoff = (initial_pose[2] - mixing_bowl_pos_tllf[2]) + .03
        z = mbz + zoff
        print 'zoff', zoff
        print 'z', z
        qx = initial_pose[3]
        qy = initial_pose[4]
        qz = initial_pose[5]
        qw = initial_pose[6]
        status = self.ac.move_to_pose(True, False, x, y, z, qx, qy, qz, qw)
        if status is not 0:
            rospy.logwarn('move to pose failed in pre plunge')
            status = self.ac.move_to_pose(True, False, x, y, z, qx, qy, qz, qw)
            if status is not 0:
                rospy.logwarn('move to pose failed in pre plunge again')
                return 'fail'
        return 'done'


class MoveToPreClean(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                             input_keys=['barcode_up', 'mixing_bowl_radius', 'mixing_bowl_height', 'mixing_bowl'])
        self.ac = ArmClient.get_arm_client()

    def execute(self, userdata):
        rospy.loginfo('executing state MoveToPreClean')
        csc = CleaningClient(no_switch = True)
        mbx = userdata.mixing_bowl.pose.pose.position.x
        mby = userdata.mixing_bowl.pose.pose.position.y
        mbz = userdata.mixing_bowl.pose.pose.position.z
        mixing_bowl_pos_bl = [mbx, mby, mbz]
        mixing_bowl_pos_tllf = csc.get_bowl_pos_in_tllf(mixing_bowl_pos_bl)
        initial_pose = csc.get_initial_pos(userdata.mixing_bowl_radius, userdata.mixing_bowl_height, 
                                           mixing_bowl_pos_tllf, userdata.barcode_up)
        # want to be a bit higher
        x = mbx
        yoff = (initial_pose[1] - mixing_bowl_pos_tllf[1])
        y = mby + yoff
        print 'yoff', yoff
        print 'y', y
        zoff = (initial_pose[2] - mixing_bowl_pos_tllf[2]) - .1
        z = mbz + zoff
        print 'zoff', zoff
        print 'z', z
        qx = initial_pose[3]
        qy = initial_pose[4]
        qz = initial_pose[5]
        qw = initial_pose[6]
        status = self.ac.move_to_pose(True, False, x, y, z, qx, qy, qz, qw)
        if status is not 0:
            rospy.logwarn('move to pose failed in pre plunge')
            status = self.ac.move_to_pose(True, False, x, y, z, qx, qy, qz, qw)
            if status is not 0:
                rospy.logwarn('move to pose failed in pre plunge again')
                return 'fail'
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

class CleanSpoon(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                             input_keys=['mixing_bowl', 'barcode_up', 'mixing_bowl_radius', 'mixing_bowl_height'])
        self.mc = MixingClient.get_mixing_client()

    def execute(self, userdata):
        logger = EventLoggerClient.startfsm('CleanSpoon')
        rospy.loginfo('Executing state CleanSpoon')
        csc = CleaningClient.get_cleaning_client()
        mbx = userdata.mixing_bowl.pose.pose.position.x
        mby = userdata.mixing_bowl.pose.pose.position.y
        mbz = userdata.mixing_bowl.pose.pose.position.z
        mixing_bowl_pos_bl = [mbx, mby, mbz]
        mixing_bowl_pos_tllf = csc.get_bowl_pos_in_tllf(mixing_bowl_pos_bl)
        #raw_input('\n\n\n\n\npress enter to continue....\n\n\n\n')
        status = csc.compliant_clean_spoon(mixing_bowl_pos_tllf, userdata.mixing_bowl_radius, 
                                           userdata.mixing_bowl_height, userdata.barcode_up)
        if status:
            logger.stops()
            return 'done'
        else:
            logger.stopf()
            return 'fail'

class LiftSpoon(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'])

    def execute(self, userdata):
        logger = EventLoggerClient.startfsm('LiftSpoon')
        rospy.loginfo('Executing state LiftSpoon')
        ac = ArmClient.get_arm_client()
        (trans, rot) = ac.get_transform(True)
        dz = .2
        status = ac.move_to_pose(True, False, trans[0], trans[1], trans[2] + dz, rot[0], rot[1], rot[2], rot[3])
        if status:
            logger.stops()
            return 'done'
        else:
            status = ac.move_to_pose(True, False, trans[0], trans[1], trans[2] + dz/2, rot[0], rot[1], rot[2], rot[3])
            logger.stopf()
            return 'fail'


def assemble_clean_spoon_fsm(mixing_bowl_height = .15):
    sm = smach.StateMachine(outcomes=['clean_success', 'clean_failure'],
                      input_keys=['mixing_bowl'])
    sm.userdata.mixing_bowl_radius = .13 #TODO change
    #sm.userdata.mixing_bowl_height = .15 #TODO change
    sm.userdata.mixing_bowl_height = mixing_bowl_height 
    fail_state = 'clean_failure'
    with sm:
        smach.StateMachine.add('GET DESIRED SPOON ORIENTATION', GetDesiredSpoonOrientation(),
                                transitions={'plunge':'MOVE TO PRE PRE CLEAN',
                                             'done':'clean_success'})

        smach.StateMachine.add('MOVE TO PRE PRE CLEAN', MoveToPrePreClean(),
                                transitions={'done':'MOVE TO PRE CLEAN',
                                             'fail':fail_state})

        smach.StateMachine.add('MOVE TO PRE CLEAN', MoveToPreClean(),
                                transitions={'done':'SWITCH TO IMPED',
                                             'fail':fail_state})

        smach.StateMachine.add('SWITCH TO IMPED', SwitchHandController(True),
                                transitions={'done':'CLEAN SPOON',
                                             'fail':fail_state})

        smach.StateMachine.add('CLEAN SPOON', CleanSpoon(),
                                transitions={'done':'SWITCH TO CART',
                                             'fail':fail_state})

        smach.StateMachine.add('SWITCH TO CART', SwitchHandController(False),
                                transitions={'done':'LIFT SPOON',
                                             'fail':fail_state})

        smach.StateMachine.add('LIFT SPOON', LiftSpoon(),
                                transitions={'done':'GET DESIRED SPOON ORIENTATION',
                                             'fail':'GET DESIRED SPOON ORIENTATION'})
    return sm
    

if __name__ == '__main__':
    rospy.init_node('grab_rim')
    try:
        sm = assemble_clean_spoon_fsm()
        sis = smach_ros.IntrospectionServer('introspection_server', sm, '/CLEAN_SPOON')
        sis.start()
        outcome = sm.execute()
    finally:
        sis.stop()

