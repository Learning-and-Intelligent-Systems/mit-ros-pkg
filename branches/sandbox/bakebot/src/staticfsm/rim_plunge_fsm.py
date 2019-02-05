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

class GetPrePlungePos(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['switch', 'done'],
                             input_keys=['mixing_bowl'],
                             output_keys=['pre_plunge_pos'])
        self.gone_once = False
        self.pre_plunges = list()
        self.iterator = None

    def execute(self, userdata):
        if not self.gone_once:
            q3 = (-.7, 0, .7, 0)
            q12 = (-.5, .5, .5, .5)
            r = .122 #TODO will want ot make the mixing bowl radius
            mbx = userdata.mixing_bowl.pose.pose.position.x - .02 # to account for shift when grabbing
            mby = userdata.mixing_bowl.pose.pose.position.y + .02
            mbz = userdata.mixing_bowl.pose.pose.position.z
            zdes = mbz + .5
            p12 = ((mbx + r, mby, zdes), q12, 12)
            p6 = ((mbx - r, mby, zdes), q12, 6)
            p3 = ((mbx, mby - r, zdes), q3, 3)
            self.pre_plunges.append(p3)
            p3 = ((mbx + .03, mby - r + .02, zdes), q3, 3)
            self.pre_plunges.append(p3)
            p3 = ((mbx - .03, mby - r + .02, zdes), q3, 3)
            self.pre_plunges.append(p3)
            #p9 = ((mbx + 0.03, mby + r, zdes), q3, 9)
            #p9 = ((mbx - 0.03, mby + r, zdes), q3, 9)
            self.pre_plunges.append(p12)
            self.pre_plunges.append(p6)
            #self.pre_plunges.append(p9)
            self.gone_once = True
            self.iterator = iter(self.pre_plunges)
        try:
            userdata.pre_plunge_pos = self.iterator.next()
            return 'switch'
        except StopIteration:
            self.gone_once = False
            self.pre_plunges = list()
            self.iterator = None
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
        rospy.loginfo('waiting to finish...')
        time.sleep(3)
        rospy.loginfo('...done waiting')
        return 'done' if status else 'fail'


class MoveToPrePlunge(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                             input_keys=['pre_plunge_pos'])
        self.ac = ArmClient.get_arm_client()

    def execute(self, userdata):
        x = userdata.pre_plunge_pos[0][0]
        y = userdata.pre_plunge_pos[0][1]
        z = userdata.pre_plunge_pos[0][2]
        qx = userdata.pre_plunge_pos[1][0]
        qy = userdata.pre_plunge_pos[1][1]
        qz = userdata.pre_plunge_pos[1][2]
        qw = userdata.pre_plunge_pos[1][3]
        status = self.ac.move_to_pose(True, True, x, y, z, qx, qy, qz, qw)
        if status is not 0:
            rospy.logwarn('move to psoe fialed in pre plunge')
            status = self.ac.move_to_pose(True, False, x, y, z, qx, qy, qz, qw)
            if status is not 0:
                rospy.logwarn('move to psoe fialed in pre plunge again')
                return 'fail'
        return 'done'


class Plunge(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                             input_keys=['mixing_bowl', 'pre_plunge_pos'])
        self.mc = MixingClient.get_mixing_client()

    def execute(self, userdata):
        rospy.loginfo('executing state Plunge')
        blx = userdata.pre_plunge_pos[0][0]
        bly = userdata.pre_plunge_pos[0][1]
        blz = userdata.pre_plunge_pos[0][2]
        (x, y, z) = self.mc.get_bowl_pos_in_tllf((blx, bly, blz))
        nso = (userdata.pre_plunge_pos[2] == 12) or (userdata.pre_plunge_pos[2] == 6)
        status = self.mc.plunge_spoon(x, y, z, nso, True)
        if status:
            return 'done'
        else:
            return 'fail'

class FinalPlunge(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                             input_keys=['mixing_bowl', 'pre_plunge_pos'])
        self.mc = MixingClient.get_mixing_client()

    def execute(self, userdata):
        rospy.loginfo('executing state FinalPlunge')
        status = self.mc.plunge_spoon()
        if status:
            return 'done'
        else:
            return 'fail'


def assemble_rim_plunge_fsm():
    sm = smach.StateMachine(outcomes=['rp_success', 'rp_failure'],
                      input_keys=['mixing_bowl'])
    fail_state = 'rp_failure'
    with sm:
        smach.StateMachine.add('GET PRE PLUNGE POS', GetPrePlungePos(),
                                transitions={'switch':'SWITCH TO CART',
                                             'done':'SWITCH TO IMPED 2'})

        smach.StateMachine.add('SWITCH TO CART', SwitchHandController(False),
                                transitions={'done':'MOVE TO PRE PLUNGE',
                                             'fail': fail_state})

        smach.StateMachine.add('MOVE TO PRE PLUNGE', MoveToPrePlunge(),
                                transitions={'done':'SWITCH TO IMPED',
                                             'fail': 'GET PRE PLUNGE POS'})

        smach.StateMachine.add('SWITCH TO IMPED', SwitchHandController(True),
                                transitions={'done':'PLUNGE',
                                             'fail': fail_state})

        smach.StateMachine.add('PLUNGE', Plunge(),
                                transitions={'done':'GET PRE PLUNGE POS',
                                             'fail': fail_state})

        smach.StateMachine.add('SWITCH TO IMPED 2', SwitchHandController(True),
                                transitions={'done':'FINAL PLUNGE',
                                             'fail': fail_state})

        smach.StateMachine.add('FINAL PLUNGE', FinalPlunge(),
                                transitions={'done':'rp_success',
                                             'fail': fail_state})

    return sm
    

if __name__ == '__main__':
    rospy.init_node('grab_rim')
    try:
        sm = assemble_grab_rim_fsm(standalone = True)
        sis = smach_ros.IntrospectionServer('introspection_server', sm, '/GRAB_RIM')
        sis.start()
        outcome = sm.execute()
    finally:
        sis.stop()

