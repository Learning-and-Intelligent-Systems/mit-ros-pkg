#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
import rospy
import tf
import sys
import actionlib
from geometry_msgs.msg import Pose
from clients.base_client import BaseClient
from utilities.bakebot_pick_and_place_manager import *
from utilities.broad_table_object_manager import *
from clients.pr2cm_client import *
from clients.mixing_client import *
from utilities.bowl_dealer_utilities import *
from bakebot.srv import *
import math
import smach
import smach_ros
import time
import pickle
from utilities.joint_recorder_player import *

class RestorePosition(smach.State):
    def __init__(self, gotopos = False):
        smach.State.__init__(self, outcomes=['done', 'fail'])
        self.gotopos = gotopos

    def execute(self, userdata):
        rospy.loginfo('Executing state RestorePosition')
        if not self.gotopos:
            ac = ArmClient.get_arm_client()
            (trans, rot) = ac.get_transform(False)
            self.trans = trans
            self.rot = rot
            self.gotopos = True
            return 'done'
        else:
            ac = ArmClient.get_arm_client()
            x = self.trans[0]
            y = self.trans[1]
            z = self.trans[2]
            qx = self.rot[0]
            qy = self.rot[1]
            qz = self.rot[2]
            qw = self.rot[3]
            status = ac.move_to_pose(False, True, x, y, z, qx, qy, qz, qw)
            if status == 0:
                return 'done'
            else:
                return 'fail'
        
class OpenGripper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'],
                                   input_keys=['grasp_with_right_hand']) # should always be false
    def execute(self, userdata):
        rospy.loginfo('Executing state OpenGripper')
        papm = PickAndPlaceManager.get_pick_and_place_manager()
        whicharm = 0 if userdata.grasp_with_right_hand else 1
        papm.open_gripper(whicharm)
        return 'done'

class RaiseHand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                                   input_keys=['object_of_desire', 'table_frame_x_offset', 'table_frame_y_offset', 'grasp_with_right_hand', 'clock_position'])
    def execute(self, userdata):
        rospy.loginfo('Executing state RaiseHand')
        ac = ArmClient.get_arm_client()
        isRightArm = userdata.grasp_with_right_hand
        (trans, rot) = ac.get_transform(isRightArm)
        print trans, rot
        xdes = trans[0]
        ydes = trans[1]
        zdes = trans[2] + .15
        status = ac.move_to_pose(isRightArm, True, xdes, ydes, zdes, rot[0], rot[1], rot[2], rot[3], pos_thresh = 0.01)
        if status is 0:
            return 'done'
        else:
            rospy.logwarn('failed once, trying again')
            status = ac.move_to_pose(isRightArm, True, xdes, ydes, zdes, rot[0], rot[1], rot[2], rot[3], pos_thresh = 0.01)
            if status is 0:
                return 'done'
            else:
                return 'fail'

class RotateToKnownPose(smach.State):
    def __init__(self, goal_pos, is_right_arm = False):
        smach.State.__init__(self, outcomes=['success', 'fail'])
        self.arm_client = ArmClient.get_arm_client()
        self.tf_listener = self.arm_client.tf_listener
        self.goal_pos = goal_pos
        self.is_right_arm = is_right_arm
    def execute(self, userdata):
        rospy.loginfo('Executing state RotateToKnownPose')
        logger = EventLoggerClient.startfsm('RotateToKnownPose (switch sides fsm)')
        pose = GripperBowlPoseManager.ALL_POSES[self.goal_pos]
        status = self.arm_client.rotate_end_effector(self.is_right_arm, pose[0], pose[1], pose[2], pose[3])
        if status == 0:
            rospy.loginfo('success')
            logger.stops()
            return 'success'
        else:
            rospy.loginfo('trying one more time')
            status = self.arm_client.rotate_end_effector(self.is_right_arm, pose[0], pose[1], pose[2], pose[3])
            if status == 0:
                rospy.loginfo('success')
                logger.stops()
                return 'success'
            rospy.loginfo('still failed!!!!!!!!!!!!!!!')
            logger.stopf()
            return 'fail'

class MoveHandOver(smach.State):
    def __init__(self, is_right_arm = False, back = False):
        smach.State.__init__(self, outcomes=['done', 'fail'])
        self.is_right_arm = is_right_arm
        self.back = back
    def execute(self, userdata):
        # TODO may want ot change this so that we move to a specific pos, and not relative to previous pos
        rospy.loginfo('Executing state RaiseHand')
        ac = ArmClient.get_arm_client()
        (trans, rot) = ac.get_transform(self.is_right_arm)
        rbowl = .13 - .02
        sign = -1 if self.back else 1
        xdes = trans[0]
        ydes = trans[1] - sign * 2 * rbowl
        zdes = trans[2]
        status = ac.move_to_pose_const_ee(self.is_right_arm, xdes, ydes, zdes, determination = 2)
        if status is 0:
            return 'done'
        else:
            return 'fail'

class LowerHand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                                   input_keys=['object_of_desire', 'table_frame_x_offset', 'table_frame_y_offset', 'grasp_with_right_hand', 'clock_position'])

    def execute(self, userdata):
        rospy.loginfo('Executing state LowerHand')
        ac = ArmClient.get_arm_client()
        isRightArm = False
        (trans, rot) = ac.get_transform(isRightArm)
        print trans, rot
        xdes = trans[0]
        ydes = trans[1]
        zdes = trans[2]
        dz = .02
        zobj = userdata.object_of_desire.pose.pose.position.z
        zdim = userdata.object_of_desire.box_dims[2]
        status = 0
        zgoal = zobj + .10 + zdim #TODO should fix this
        print '***************************************'
        print 'zgoal: ', zgoal
        print 'zobj: ', zobj
        print 'zhard = .11 for mb and .04 for cs'
        print '***************************************'
        status = ac.move_to_pose(isRightArm, True, xdes, ydes, zgoal, rot[0], rot[1], rot[2], rot[3], pos_thresh = 0.01)
        if status is 0:
            return 'done'
        else:
            return 'fail'


class CloseGripper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'],
                                   input_keys=['grasp_with_right_hand']) # should always be false

    def execute(self, userdata):
        rospy.loginfo('Executing state CloseGripper')
        papm = PickAndPlaceManager.get_pick_and_place_manager()
        whicharm = 0 if userdata.grasp_with_right_hand else 1
        papm.close_gripper(whicharm)
        return 'done'


class ResetBothArms(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'])
        self.arm_client = ArmClient.get_arm_client()
        self.papm = PickAndPlaceManager.get_pick_and_place_manager()

    def execute(self, userdata):
        rospy.loginfo('Executing state ResetBothArms')
        client = PR2CMClient.get_pr2cm_client()
        print client.load_cartesian(True)
        print client.load_cartesian(False)
        rospy.loginfo('moving the right arm to the side')
        self.papm.move_arm_to_side(0)
        rospy.loginfo('moving the left arm to the side')
        self.papm.move_arm_to_side(1)
        rospy.loginfo('closing both grippers')
        self.papm.close_gripper(0)
        self.papm.close_gripper(1)
        return 'done'


class ResetOneArm(smach.State):
    def __init__(self, isRightArm, retreat_first = False):
        smach.State.__init__(self, outcomes=['done', 'fail'])
        self.arm_client = ArmClient.get_arm_client()
        self.papm = PickAndPlaceManager.get_pick_and_place_manager()
        self.isRightArm = isRightArm
        self.retreat_first = retreat_first

    def execute(self, userdata):
        rospy.loginfo('Executing state ResetOneArm: ' + str(self.isRightArm))
        client = PR2CMClient.get_pr2cm_client()
        print client.load_cartesian(self.isRightArm)
        if self.retreat_first:
            rospy.loginfo('retreating the right arm')
            self.arm_client.retreat_arm(self.isRightArm)
        aname = 'right' if self.isRightArm else 'left'
        whicharm = 0 if self.isRightArm else 1
        rospy.loginfo('moving the ' + aname + ' arm to the side')
        self.papm.move_arm_to_side(whicharm)

        rospy.loginfo('closing the ' + aname + ' gripper')
        self.papm.close_gripper(whicharm)
        return 'done'


def assemble_switch_sides_fsm(right_arm = False):
    sm = smach.StateMachine(outcomes=['switching_success', 'switching_failure'],
                            input_keys=['object_of_desire', 'table_frame_x_offset', 'table_frame_y_offset', 'grasp_with_right_hand'],
                            output_keys=['object_of_desire'])
    fail_state = 'switching_failure'
    sm.userdata.grasp_with_right_hand = right_arm
    with sm:
        restore_position_state = RestorePosition(gotopos = False)
        smach.StateMachine.add('RECORD POSITION', restore_position_state,
                                transitions={'done':'OPEN GRIPPER',
                                             'fail':fail_state})

        smach.StateMachine.add('OPEN GRIPPER', OpenGripper(),
                                transitions={'done':'LIFT GRIPPER'})

        smach.StateMachine.add('LIFT GRIPPER', RaiseHand(),
                                transitions={'done':'PRE TRANSIT ROTATE',
                                             'fail':fail_state})

        smach.StateMachine.add('PRE TRANSIT ROTATE', RotateToKnownPose('three_o_clock_bi'),
                                transitions={'success':'MOVE HAND OVER',
                                             'fail':'MOVE HAND OVER'})

        smach.StateMachine.add('MOVE HAND OVER', MoveHandOver(),
                                transitions={'done':'POST TRANSIT ROTATE',
                                             'fail':fail_state})

        smach.StateMachine.add('POST TRANSIT ROTATE', RotateToKnownPose('three_o_clock_bi'),
                                transitions={'success':'LOWER GRIPPER',
                                             'fail':fail_state})

        smach.StateMachine.add('LOWER GRIPPER', LowerHand(),
                                transitions={'done':'CLOSE GRIPPER',
                                             'fail':fail_state})

        smach.StateMachine.add('CLOSE GRIPPER', CloseGripper(),
                                transitions={'done':'LIFT BOWL'})

        smach.StateMachine.add('LIFT BOWL', RaiseHand(),
                                transitions={'done':'ROTATE TO SIX',
                                             'fail':fail_state})

        smach.StateMachine.add('ROTATE TO SIX', RotateToKnownPose('six_o_clock_bi'),
                                transitions={'success':'ROTATE TO NINE',
                                             'fail':'ROTATE TO NINE'})

        smach.StateMachine.add('ROTATE TO NINE', RotateToKnownPose('nine_o_clock_bi'),
                                transitions={'success':'MOVE HAND BACK',
                                             'fail':fail_state})

        smach.StateMachine.add('MOVE HAND BACK', MoveHandOver(back = True),
                                transitions={'done':'RESTORE POSITION',
                                             'fail':fail_state})

        smach.StateMachine.add('RESTORE POSITION', restore_position_state, 
                                transitions={'done':'switching_success',
                                             'fail':fail_state})
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

