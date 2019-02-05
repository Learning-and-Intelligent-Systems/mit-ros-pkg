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
from clients.mixing_client import *
from utilities.bowl_dealer_utilities import *
from bakebot.srv import *
import math
import smach
import smach_ros
import time
import pickle
from utilities.joint_recorder_player import *

class Detect(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             output_keys=['object_of_desire', 'table_frame_x_offset', 'table_frame_y_offset', 'grasp_with_right_hand', 'clock_position'])
        self.pick_and_place_manager = PickAndPlaceManager.get_pick_and_place_manager()
        print 'constructing detect object'
        self.arm_client = ArmClient.get_arm_client()
        self.btom = BroadTabletopObjectManager.get_broad_tabletop_object_manager(self.pick_and_place_manager)
        print 'done'

    def execute(self, userdata):
        rospy.loginfo('Executing state Detect')
        filename = raw_input('enter the filename to load from (blank for detectedobj.log): ')
        if len(filename) == 0:
            filename = 'detectedobj.log'
            self.btom.load_detected_objects_from_file('detectedobj.log')
        elif len(filename) == 1:
            print 'doing a fresh scan...'
            self.btom.do_broad_tabletop_object_detection()
            status = self.btom.save_detected_objects_to_file('detectedobj.log')
            print '\n\n\n\n***************************************************************************************'
            if not status:
                rospy.logwarn('saving detected objects failed')
            
        for i, obj in enumerate(self.btom.detected_objects):
            print i, obj.collision_name, obj.pose.pose.position.x, obj.pose.pose.position.y
        try: 
            choice = int(raw_input('\n\n\t\t\tobject choice: '))
            print choice
            arms = raw_input('(r)ight, (l)eft: ')

            okclockpos = [12, 3, 6, 9]
            clocknames = ['twelve_o_clock', 'three_o_clock', 'six_o_clock', 'nine_o_clock']
            while True:
                clock = int(raw_input('enter clock position to grab at (12, 3, 6, 9): '))
                if clock not in okclockpos:
                    print 'not in list', okclockpos
                else:
                    break

            userdata.grasp_with_right_hand = (arms == 'r')
            userdata.table_frame_x_offset = 0
            userdata.table_frame_y_offset = 0
            userdata.object_of_desire = self.btom.detected_objects[i]
            userdata.clock_position = clocknames[okclockpos.index(clock)]

        except Exception as e:
            print 'caught exception!'
            print str(e)
            self.execute(userdata) 
            return 'fail'
        return 'success'

class ChooseClockPos(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail with exhaustion'],
                                   output_keys=['clock_position'])
        self.clocknames = ['six_o_clock', 'nine_o_clock', 'twelve_o_clock', 'three_o_clock']
        self.iterator = iter(self.clocknames)

    def execute(self, userdata):
        rospy.loginfo('Executing state ChooseClockPos')
        try:
            retclock = self.iterator.next()
            rospy.loginfo('the next clock position is: ' + str(retclock))
            userdata.clock_position = retclock
            return 'done'
        except StopIteration:
            rospy.logwarn('ran out of clock positions')
            return 'fail with exhaustion'




class RefineObjectDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                                   input_keys=['object_of_desire', 'object_of_desire_name', 'table_frame_x_offset', 'table_frame_y_offset'],
                                   output_keys=['object_of_desire'])

    def execute(self, userdata):
        rospy.loginfo('Executing state RefineObjectDetection')
        x = userdata.object_of_desire.pose.pose.position.x
        y = userdata.object_of_desire.pose.pose.position.y
        z = userdata.object_of_desire.pose.pose.position.z
        rospy.loginfo('pose of object' + str((x,y,z)))
        rospy.loginfo('replacing the pose of userdata.object_of_desire with the pose from the most recent detection')
        print 'I want to grab the object with collision name: ', userdata.object_of_desire.collision_name
        print 'and that object has the object_of_desire_name:', userdata.object_of_desire_name
        btom = BroadTabletopObjectManager.get_broad_tabletop_object_manager(PickAndPlaceManager.get_pick_and_place_manager())
        detected_object = btom.refine_and_and_get_graspable_object(userdata.object_of_desire.collision_name, userdata.table_frame_x_offset, userdata.table_frame_y_offset)
        if detected_object == None:
            rospy.logerr('could not find the object_of_desire!')
            return 'fail'
        userdata.object_of_desire.box_dims = detected_object.box_dims
        userdata.object_of_desire.pose = detected_object.pose
        x = userdata.object_of_desire.pose.pose.position.x
        y = userdata.object_of_desire.pose.pose.position.y
        z = userdata.object_of_desire.pose.pose.position.z
        rospy.loginfo('corrected pose of object_of_desire: ' + str((x,y,z)))
        return 'done'

        
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


class MoveHandOverRim(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                                   input_keys=['object_of_desire', 'object_of_desire_name', 'table_frame_x_offset', 'table_frame_y_offset', 'grasp_with_right_hand', 'clock_position'])
        self.first_time = True

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveHandOverRim')
        #xdim = userdata.object_of_desire.box_dims.x
        #ydim = userdata.object_of_desire.box_dims.y
        xdim = userdata.object_of_desire.box_dims[0]
        ydim = userdata.object_of_desire.box_dims[1]
        zdim = userdata.object_of_desire.box_dims[2]
        r = (math.sqrt(xdim**2 + ydim**2)) / 2.0
        rospy.loginfo('calculated radius: ' + str(r))
        rcor = .02
        r = r - rcor
        rospy.loginfo('subtracting correction: ' + str(rcor))
        rhard = .123 if userdata.object_of_desire_name == 'cookie_sheet' else .14 #.13
        r = rhard
        rospy.logwarn('overriding radius with hardcodedd value: ' + str(rhard))
        rospy.loginfo('zdim: ' + str(zdim))

        h = .4 # TODO tune this
        #h = 0.2
        x = userdata.object_of_desire.pose.pose.position.x
        y = userdata.object_of_desire.pose.pose.position.y
        z = userdata.object_of_desire.pose.pose.position.z
        pose_to_point_at = [x, y, .65]
        pick_and_place_manager = PickAndPlaceManager.get_pick_and_place_manager()
        print 'point the head at xyz', pose_to_point_at
        pick_and_place_manager.point_head(pose_to_point_at, 'base_link')

        if userdata.clock_position is 'twelve_o_clock':
            xoff = r
            yoff = 0
        elif userdata.clock_position is 'three_o_clock':
            xoff = 0
            yoff = -r
        elif userdata.clock_position is 'six_o_clock':
            xoff = -r
            yoff = 0
        else:
            xoff = 0
            yoff = r

        xdes = x + xoff #+ userdata.table_frame_x_offset
        ydes = y + yoff #+ userdata.table_frame_y_offset
        zdes = z + h

        print '\n\n\n*******************************************'
        print 'x', x
        print 'y', y
        print 'r: ', r
        print 'xoff: ', xoff
        print 'yoff: ', yoff
        print 'desired: ', xdes, ydes, zdes
        print 'xdim', xdim
        print 'ydim', ydim
        print 'zdim', zdim
        print '*******************************************'

        posenamein = userdata.clock_position + '_bi'
        posenameout = userdata.clock_position + '_bo'
        rotin = GripperBowlPoseManager.ALL_POSES[posenamein]
        rotout = GripperBowlPoseManager.ALL_POSES[posenameout]
        ac = ArmClient.get_arm_client()
        rot = rotin
        if self.first_time:
            self.first_time = False
            ac.advance_arm(userdata.grasp_with_right_hand)
        status = ac.move_to_pose(userdata.grasp_with_right_hand, False, xdes, ydes, zdes, rot[0], rot[1], rot[2], rot[3], pos_thresh=.025, rot_thresh=.25)
        status = ac.move_to_pose(userdata.grasp_with_right_hand, False, xdes, ydes, zdes, rot[0], rot[1], rot[2], rot[3], pos_thresh=.035, rot_thresh=.25)

        #goalpose = Pose()
        #goalpose.position.x = xdes
        #goalpose.position.y = ydes
        #goalpose.position.z = zdes
        #goalpose.orientation.x = rot[0]
        #goalpose.orientation.y = rot[1]
        #goalpose.orientation.z = rot[2]
        #goalpose.orientation.w = rot[3]
        #papm = PickAndPlaceManager.get_pick_and_place_manager()
        #btom = BroadTabletopObjectManager.get_broad_tabletop_object_manager(papm)
        #armstotry = (0,0) if userdata.grasp_with_right_hand else (1,1)
        #(resultval, whicharm, grasp) = btom.refine_and_grasp(userdata.object_of_desire.collision_name, armstotry, 0, 0, goalpose)
        #print 'resutlval: ', resultval

        if status is not 0: 
            rot = rotout
            status = ac.move_to_pose(userdata.grasp_with_right_hand, True, xdes, ydes, zdes, rot[0], rot[1], rot[2], rot[3])
            if status is not 0: 
                return 'fail'
        return 'done'


# NOTE that this one switches fo4r the left hand, not the right hand like usual...
class SwitchHandController(smach.State):
    def __init__(self, switch_to_imped):
        smach.State.__init__(self, outcomes=['done', 'fail'])
        self.switch_to_imped = switch_to_imped

    def execute(self, userdata):
        rospy.loginfo('Executing state SwitchHandToImpedanceControl')
        client = PR2CMClient.get_pr2cm_client()
        if self.switch_to_imped:
            status = client.load_ee_cart_imped(False) 
        else:
            status = client.load_cartesian(False)
        return 'done' if status else 'fail'


class LowerHand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                                   input_keys=['object_of_desire', 'zgoal', 'object_of_desire_name', 'table_frame_x_offset', 'table_frame_y_offset', 'grasp_with_right_hand', 'clock_position'])

    def execute(self, userdata):
        rospy.loginfo('Executing state LowerHand')
        #x = userdata.object_of_desire.pose.pose.position.x
        #y = userdata.object_of_desire.pose.pose.position.y
        #xdes = x
        #ydes = y
        #mc = MixingClient.get_mixing_client(use_right_arm = False)
        #status = mc.plunge_spoon() # will probably want to modify mc to take x and y, but we'll wait and see
        ac = ArmClient.get_arm_client()
        isRightArm = userdata.grasp_with_right_hand

        (trans, rot) = ac.get_transform(isRightArm)
        print trans, rot
        xdes = trans[0]
        ydes = trans[1]
        zdes = trans[2]
        dz = .02
        zobj = userdata.object_of_desire.pose.pose.position.z
        zdim = userdata.object_of_desire.box_dims[2]

        status = 0
        #while status is 0: # TODO: add minimum z threshold here
        #while True: # TODO: add minimum z threshold here
            #zdes = zdes - dz
            #zgoal = zobj + .08 + zdim
            #print 'zdes: ', zdes, '\tzobj: ', zobj, '\tzdim: ', zdim, '\tzgoal: ', zgoal
            #if zdes < zgoal:
                #break
            #status = ac.move_to_pose(isRightArm, True, xdes, ydes, zdes, rot[0], rot[1], rot[2], rot[3], pos_thresh = 0.01)
            #time.sleep(1)
#
            #print status
        zgoal = zobj + .09 + zdim #TODO should fix this
        if userdata.object_of_desire_name == 'cookie_sheet':
            print 'hard setting z'
            zgoal = .74
        else: 
            print 'hard setting z for mixing bowl'
            zgoal = .86
            print 'zgoal:', zgoal, ' userdata.zgoal:', userdata.zgoal
            #TODO: change this for iros demo
        print '***************************************'
        print 'zgoal: ', zgoal
        print 'zobj: ', zobj
        print 'zobj+.09:', (zobj+.09)
        print '***************************************'
        status = ac.move_to_pose(isRightArm, True, xdes, ydes, zgoal, rot[0], rot[1], rot[2], rot[3], pos_thresh = 0.01)
        if status is 0:
            return 'done'
        else:
            return 'fail'


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
        zdes = trans[2] + .3
        status = ac.move_to_pose(isRightArm, True, xdes, ydes, zdes, rot[0], rot[1], rot[2], rot[3], pos_thresh = 0.01)
        if status is 0:
            return 'done'
        else:
            status = ac.move_to_pose(isRightArm, True, xdes, ydes, zdes, rot[0], rot[1], rot[2], rot[3], pos_thresh = 0.01)
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


def assemble_grab_rim_fsm(standalone=False, use_all_clockpos = False, right_arm = False, zgoal = 0):
    sm = smach.StateMachine(outcomes=['grabbing_success', 'grabbing_failure'],
                            input_keys=['object_of_desire', 'object_of_desire_name', 'table_frame_x_offset', 'table_frame_y_offset', 'grasp_with_right_hand', 'clock_position'],
                            output_keys=['object_of_desire'])
# TODO: mbollini 2/22/12 make sure that the object_of_desire gets reset appropriately

    fail_state = 'grabbing_failure'
    transit_fail_state = 'CHOOSE CLOCK POS' if use_all_clockpos else fail_state
    sm.userdata.zgoal = zgoal

    with sm:

        if standalone:
            smach.StateMachine.add('RESET BOTH ARMS', ResetBothArms(),
                                transitions={'done':'DETECT',
                                             'fail': fail_state})
            smach.StateMachine.add('DETECT', Detect(),
                                    transitions={'success':'REFINE OBJECT DETECTION',
                                                'fail':fail_state})
        else:
            smach.StateMachine.add('RESET ARM', ResetOneArm(right_arm),
                                transitions={'done':'REFINE OBJECT DETECTION',
                                             'fail': fail_state})

        smach.StateMachine.add('REFINE OBJECT DETECTION', RefineObjectDetection(),
                                transitions={'done':'OPEN GRIPPER',
                                             'fail':fail_state})

        postopen = 'CHOOSE CLOCK POS' if use_all_clockpos else 'MOVE OVER RIM'
        smach.StateMachine.add('OPEN GRIPPER', OpenGripper(),
                                transitions={'done':postopen})

        smach.StateMachine.add('CHOOSE CLOCK POS', ChooseClockPos(),
                                transitions={'done':'MOVE OVER RIM',
                                            'fail with exhaustion':fail_state})

        smach.StateMachine.add('MOVE OVER RIM', MoveHandOverRim(),
                                transitions={'done':'LOWER HAND',
                                             'fail':transit_fail_state})

        smach.StateMachine.add('LOWER HAND', LowerHand(),
                                transitions={'done':'CLOSE GRIPPER',
                                             'fail':'CLOSE GRIPPER'})

        smach.StateMachine.add('CLOSE GRIPPER', CloseGripper(),
                                transitions={'done':'grabbing_success'})

        #smach.StateMachine.add('SWITCH TO IMPED', SwitchHandController(True),
                                #transitions={'done':'LOWER HAND',
                                             #'fail':fail_state})

        #smach.StateMachine.add('LOWER HAND', LowerHand(),
        #                        transitions={'done':'CLOSE GRIPPER',
        #                                     'fail':fail_state})
#
#        smach.StateMachine.add('CLOSE GRIPPER', CloseGripper(),
#                                transitions={'done':'SWITCH TO CART'})
#
##        smach.StateMachine.add('SWITCH TO CART', SwitchHandController(False),
##                                transitions={'done':'grabbing_success',
#                                             'fail':fail_state})
    return sm

def assemble_retreat_fsm(right_arm = False):
    sm = smach.StateMachine(outcomes=['grabbing_retreat_success', 'grabbing_retreat_failure'],
                            input_keys=['object_of_desire', 'object_of_desire_name', 'table_frame_x_offset', 'table_frame_y_offset', 'grasp_with_right_hand', 'clock_position'],
                            output_keys=['object_of_desire'])

    fail_state = 'grabbing_retreat_failure'
    with sm:
        smach.StateMachine.add('OPEN GRIPPER', OpenGripper(),
                                transitions={'done':'RAISE HAND'})

        smach.StateMachine.add('RAISE HAND', RaiseHand(),
                                transitions={'done':'RESET ARM',
                                             'fail':'RESET ARM'})

        smach.StateMachine.add('RESET ARM', ResetOneArm(right_arm, True),
                            transitions={'done':'grabbing_retreat_success',
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

