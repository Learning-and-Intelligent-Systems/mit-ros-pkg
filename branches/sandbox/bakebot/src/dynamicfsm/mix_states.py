#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
from smachforward import *
import rospy
import tf
import actionlib
from clients.arm_client import ArmClient
from clients.base_client import BaseClient
from clients.pr2cm_client import PR2CMClient
from clients.mixing_client import MixingClient
from utilities.bakebot_pick_and_place_manager import *
from utilities.broad_table_object_manager import *
from utilities.joint_recorder_player import *
from clients.unified_tf_client import UnifiedTFClient
from staticfsm.rim_plunge_fsm import *
from staticfsm.clean_spoon_fsm import *
from staticfsm.grab_rim_fsm import *
from staticfsm.switch_bowl_sides_fsm import *
from bakebot.srv import *
import math
import smach
import smach_ros
import time


class MixServoToMB(FState):
    def __init__(self):
        FState.__init__(self, 
                                   input_keys=['mixing_bowl',
                                               'table_front_edge_x',
                                               'nominal_mixing_bowl_x',
                                               'nominal_mixing_bowl_y',
                                               'table_frame_x_offset',
                                               'table_frame_y_offset'],
                                   output_keys=['mixing_bowl', 
                                                'table_frame_x_offset',
                                                'table_frame_y_offset',
                                                'table_front_edge_x'])

    def execute(self, userdata):
        logger = EventLoggerClient.startfsm('RefineMixingBowlPosition (mixing fsm)')
        rospy.loginfo('Executing state RefineMixingBowlPosition')
        papm = PickAndPlaceManager.get_pick_and_place_manager()
        btom = BroadTabletopObjectManager.get_broad_tabletop_object_manager(papm)
        if not btom.filtered:
            btom.filter_ingredients()
        userdata.mixing_bowl = btom.mixing_bowl
        x = userdata.mixing_bowl.pose.pose.position.x
        y = userdata.mixing_bowl.pose.pose.position.y
        z = userdata.mixing_bowl.pose.pose.position.z
        rospy.loginfo('\n\n\noriginal pose of mixing bowl: ' + str((x,y,z)) + '\n\n\n')
        rospy.loginfo('replacing the pose of userdata.mixing_bowl with the pose from the most recent detection')
        btom = BroadTabletopObjectManager.get_broad_tabletop_object_manager(PickAndPlaceManager.get_pick_and_place_manager())
        detected_mixing_bowl = btom.refine_and_and_get_graspable_object(userdata.mixing_bowl.collision_name, userdata.table_frame_x_offset, userdata.table_frame_y_offset)
        if detected_mixing_bowl == None:
            rospy.logerr('could not find the mixing bowl!')
            logger.stopf()
            return 'failure'
        userdata.mixing_bowl.pose = detected_mixing_bowl.pose
        x = userdata.mixing_bowl.pose.pose.position.x
        y = userdata.mixing_bowl.pose.pose.position.y
        z = userdata.mixing_bowl.pose.pose.position.z
        rospy.loginfo('\n\n\ncorrected pose of mixing bowl: ' + str((x,y,z)) + '\n\n\n')
        logger.stops()

        rospy.loginfo('Executing state RefineTableFront')
        logger = EventLoggerClient.startfsm('RefineTableFront (mixing fsm)')
        papm = PickAndPlaceManager.get_pick_and_place_manager()
        rospy.loginfo('original table_front_edge_x: ' + str(papm.table_front_edge_x))
        papm.find_table()
        userdata.table_front_edge_x = papm.table_front_edge_x
        rospy.loginfo('updated table_front_edge_x: ' + str(papm.table_front_edge_x))
        logger.stops()

        rospy.loginfo('Executing state MixingMoveRobot')
        logger = EventLoggerClient.startfsm('MixingMoveRobot (mixing fsm)')
        ROBOT_FRONT_X = 0.2 # TODO figure out this number for real!
        TABLE_MARGIN = 0.04
        bc = BaseClient.get_base_client()
        actual_x = userdata.mixing_bowl.pose.pose.position.x
        actual_y = userdata.mixing_bowl.pose.pose.position.y
        desired_x = userdata.nominal_mixing_bowl_x
        desired_y = userdata.nominal_mixing_bowl_y
        
        drive_x = actual_x - desired_x
        drive_y = actual_y - desired_y
        print '\n\n\nactual x: ', actual_x, ' desired_x: ', desired_x, 'drive_x: ', drive_x
        print 'actual y: ', actual_y, ' desired_y: ', desired_y, 'drive_y: ', drive_y, '\n\n\n'

        rospy.loginfo('DESIRED DRIVE X: ' + str(drive_x))
        rospy.loginfo('DESIRED DRIVE Y: ' + str(drive_y))
        bc.translate(True, drive_x)
        bc.translate(False, drive_y)

        userdata.table_frame_x_offset = -drive_x  
        userdata.table_frame_y_offset = -drive_y
        print '********** xoff', userdata.table_frame_x_offset
        print '********** yoff', userdata.table_frame_y_offset
        userdata.table_front_edge_x = userdata.table_front_edge_x - drive_x
        logger.stops()

        logger = EventLoggerClient.startfsm('RefineMixingBowlPosition (mixing fsm)')
        rospy.loginfo('Executing state RefineMixingBowlPosition')
        x = userdata.mixing_bowl.pose.pose.position.x
        y = userdata.mixing_bowl.pose.pose.position.y
        z = userdata.mixing_bowl.pose.pose.position.z
        rospy.loginfo('\n\n\noriginal pose of mixing bowl: ' + str((x,y,z)) + '\n\n\n')
        rospy.loginfo('replacing the pose of userdata.mixing_bowl with the pose from the most recent detection')
        btom = BroadTabletopObjectManager.get_broad_tabletop_object_manager(PickAndPlaceManager.get_pick_and_place_manager())
        detected_mixing_bowl = btom.refine_and_and_get_graspable_object(userdata.mixing_bowl.collision_name, userdata.table_frame_x_offset, userdata.table_frame_y_offset)

        if detected_mixing_bowl == None:
            rospy.logerr('could not find the mixing bowl!')
            #logger.stopf()
            #return 'failure'
        else:
            userdata.mixing_bowl.pose = detected_mixing_bowl.pose
            x = userdata.mixing_bowl.pose.pose.position.x
            y = userdata.mixing_bowl.pose.pose.position.y
            z = userdata.mixing_bowl.pose.pose.position.z
            rospy.loginfo('\n\n\ncorrected pose of mixing bowl: ' + str((x,y,z)) + '\n\n\n')
        logger.stops()
        return 'success'

    def get_name_str(self):
        return 'servo-to-mb'

    def get_params_str(self):
        return '(?x)'

    def get_precondition_str(self):
		return '(not (mix-zone ?x))'

    def get_effect_str(self):
		return '(mix-zone ?x)'

class MixServoFromMB(FState):
    def __init__(self):
        FState.__init__(self, 
                                   input_keys=['mixing_bowl', 'table_frame_x_offset', 'table_frame_y_offset', 'nominal_mixing_bowl_x', 'nominal_mixing_bowl_y', 'table_front_edge_x'],
                                   output_keys=['mixing_bowl', 'table_frame_x_offset', 'table_frame_y_offset', 'table_front_edge_x'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ResetRobotBasePos')
        ROBOT_FRONT_X = 0.2 
        TABLE_MARGIN = 0.04
        logger = EventLoggerClient.startfsm('ResetRobotBasePos (mixing fsm)')
        bc = BaseClient.get_base_client()
        drive_x = userdata.table_frame_x_offset
        drive_y = userdata.table_frame_y_offset
        rospy.loginfo('DESIRED DRIVE X: ' + str(drive_x))
        rospy.loginfo('DESIRED DRIVE Y: ' + str(drive_y))
        bc.translate(True, drive_x)
        bc.translate(False, drive_y)
        userdata.table_frame_x_offset = 0
        userdata.table_frame_y_offset = 0
        userdata.table_front_edge_x = userdata.table_front_edge_x - drive_x 
        logger.stops()

        logger = EventLoggerClient.startfsm('RefineMixingBowlPosition (mixing fsm)')
        rospy.loginfo('Executing state RefineMixingBowlPosition')
        x = userdata.mixing_bowl.pose.pose.position.x
        y = userdata.mixing_bowl.pose.pose.position.y
        z = userdata.mixing_bowl.pose.pose.position.z
        rospy.loginfo('\n\n\noriginal pose of mixing bowl: ' + str((x,y,z)) + '\n\n\n')
        rospy.loginfo('replacing the pose of userdata.mixing_bowl with the pose from the most recent detection')
        btom = BroadTabletopObjectManager.get_broad_tabletop_object_manager(PickAndPlaceManager.get_pick_and_place_manager())
        detected_mixing_bowl = btom.refine_and_and_get_graspable_object(userdata.mixing_bowl.collision_name, userdata.table_frame_x_offset, .2) #TODO will need to change this around to make sure it detects ok
        if detected_mixing_bowl == None:
            rospy.logerr('could not find the mixing bowl!')
            logger.stopf()
            return 'failure'
        userdata.mixing_bowl.pose = detected_mixing_bowl.pose
        x = userdata.mixing_bowl.pose.pose.position.x
        y = userdata.mixing_bowl.pose.pose.position.y
        z = userdata.mixing_bowl.pose.pose.position.z
        rospy.loginfo('\n\n\ncorrected pose of mixing bowl: ' + str((x,y,z)) + '\n\n\n')
        logger.stops()
        return 'success'

    def get_name_str(self):
        return 'servo-from-mb'

    def get_params_str(self):
        return '(?x)'

    def get_precondition_str(self):
		return '(mix-zone ?x)'

    def get_effect_str(self):
		return '(not (mix-zone ?x))'


class MixGrabMB(FState):
    def __init__(self):
        FState.__init__(self, 
                                   input_keys=['mixing_bowl', 'table_frame_x_offset', 'table_frame_y_offset'],
                                   output_keys=['mixing_bowl'])

    def execute(self, userdata):
        grab_zgoal = .86 # for grab rim
        grabbing_fsm = assemble_grab_rim_fsm(zgoal=grab_zgoal)
        grabbing_fsm.userdata.object_of_desire = userdata.mixing_bowl
        grabbing_fsm.userdata.object_of_desire_name = 'mixing_bowl'
        grabbing_fsm.userdata.table_frame_x_offset = userdata.table_frame_x_offset
        grabbing_fsm.userdata.table_frame_y_offset = userdata.table_frame_y_offset
        grabbing_fsm.userdata.grasp_with_right_hand = False
        grabbing_fsm.userdata.clock_position = 'nine_o_clock'
        outcome = grabbing_fsm.execute()
        if outcome == 'grabbing_success':
            return 'success'
        else:
            return 'failure'

    def get_name_str(self):
        return 'grab-mb'

    def get_params_str(self):
        return '(?x ?y)'

    def get_precondition_str(self):
		return '(and (MIXING-BOWL ?x) (GRIPPER ?y) (not (SPATULA-GRIPPER ?y)) (reset ?y) (free ?y) (mix-zone ?x))'

    def get_effect_str(self):
		return '(and (carry ?y ?x) (not (free ?y)) (not (reset ?y)) (not (reset-safe ?y)))'


class MixTransitSpoon(FState):
    def __init__(self):
        FState.__init__(self, 
                                  input_keys=['mixing_bowl', 'pre_mix_height_above_table'])
        self.arm_client = ArmClient.get_arm_client()
        self.papm = PickAndPlaceManager.get_pick_and_place_manager()

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveMixingHandToPreMixPosition')
        x = userdata.mixing_bowl.pose.pose.position.x
        y = userdata.mixing_bowl.pose.pose.position.y
        z = self.papm.table_height + userdata.pre_mix_height_above_table
        rospy.loginfo('advancing the arm from the side position')
        self.logger = EventLoggerClient.startfsm('MoveMixingHandToPreMixPosition (mixing fsm)')
        self.arm_client.advance_arm(True)
        status = self.arm_client.move_to_pose(True, False, x, y, z, -.5, .5, .5, .5, collision_aware = 1)  # p 120, num 1. TODO  may need to change to better
        print status
        if status is not 0:
            status = self.arm_client.move_to_pose(True, False, x, y, z, -.5, .5, .5, .5, collision_aware = 1)  # p 120, num 1. TODO  may need to change to better
            print status
            if status is not 0:
                status = self.arm_client.move_to_pose(True, False, x, y, z, -.5, .5, .5, .5, collision_aware = 0)  # p 120, num 1. TODO  may need to change to better
                print status
                if status is not 0:
                    rospy.logerr('moving to the pose failed even when detached. failing with exhaustion!')
                    self.logger.stopf()
                    return 'failure'
        rospy.loginfo('moving to the pre mix position succeeded')
        self.logger.stops()
        return 'success'

    def get_name_str(self):
        return 'transit-spoon'

    def get_params_str(self):
        return '(?y ?x ?z)'

    def get_precondition_str(self):
		return '(and (GRIPPER ?y) (MIXING-BOWL ?x) (GRIPPER ?z) (carry ?z ?x) (mix-zone ?x) (SPATULA-GRIPPER ?y))'

    def get_effect_str(self):
		return '(and (over-mb ?y) (not (reset ?y)) (not (reset-safe ?y)))'


class MixPlungeSpoon(FState):
    def __init__(self):
        FState.__init__(self, 
                                   input_keys=['mixing_bowl'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PlungeSpoon')
        logger = EventLoggerClient.startfsm('PlungeSpoon (mixing fsm)')
        mc = MixingClient.get_mixing_client()
        x = userdata.mixing_bowl.pose.pose.position.x
        y = userdata.mixing_bowl.pose.pose.position.y
        z = userdata.mixing_bowl.pose.pose.position.z
        (bx, by, bz) = mc.get_bowl_pos_in_tllf((x, y, z))
        status = mc.plunge_spoon(bowl_x = bx, bowl_y = by, bowl_z = bz) # TODO don't hard code this
        status = mc.plunge_spoon(bowl_x = bx, bowl_y = by, bowl_z = bz) # TODO don't hard code this
        logger.stops()
        return 'success' if status else 'failure'

    def get_name_str(self):
        return 'plunge-spoon'

    def get_params_str(self):
        return '(?y ?x ?z)'

    def get_precondition_str(self):
		return '(and (GRIPPER ?y) (SPATULA-GRIPPER ?y) (GRIPPER ?x) (MIXING-BOWL ?z) (carry ?x ?z) (over-mb ?y))'

    def get_effect_str(self):
		return '(and (not (over-mb ?y)) (in-mb ?y))'


class Mix(FState):

    def __init__(self, debug=False):
        FState.__init__(self, 
                                   input_keys=['mixing_laps', 'mixing_bowl', 'table_frame_x_offset', 'table_frame_y_offset'],
                                   output_keys=[])
        self.num_mixes_done = 0
        self.debug = debug 
        self.first_pass = True
        self.mix_list = ['circular', 'linear', 'circular', 'switch sides', 'circular', 'linear']
        self.iterator = iter(self.mix_list)

    def execute(self, userdata):
        rospy.loginfo('Executing state SwitchHandToImpedanceControl')
        logger = EventLoggerClient.startfsm('SwitchHandToImpedanceControl (mixing fsm)')
        client = PR2CMClient.get_pr2cm_client()
        status = client.load_ee_cart_imped(True) 
        logger.stops()
        if not status:
            return 'failure'

        rospy.loginfo('Executing state AssessMix')

        mc = MixingClient.get_mixing_client()
        logger = EventLoggerClient.startfsm('AssessMix (mixing fsm)')
        while True:
            if not self.debug:
                try:
                    action = self.iterator.next()
                    rospy.loginfo('commanding a ' + str(action) + ' mix')
                    choice = action[0]
                except StopIteration:
                    rospy.loginfo('done mixing!')
                    logger.stops()
                    mlogger = StaticLoggerLocker.check_out_logger('MIX LOGGER')
                    if mlogger:
                        mlogger.stops()
                    self.first_pass = True
                    status = client.load_cartesian(True)
                    return 'success'
            else:
                choice = raw_input('\n (s)witch sides, (c)ircle mix, (l)inear mix, (r)im plunge, (a)ll, (d)one: ')
            if choice == 'c':
                logger.stops()
                rospy.loginfo('Executing state CircleMix')
                logger = EventLoggerClient.startfsm('CircleMix (mixing fsm)')
                mc = MixingClient.get_mixing_client()
                bx = userdata.mixing_bowl.pose.pose.position.x
                by = userdata.mixing_bowl.pose.pose.position.y
                bz = userdata.mixing_bowl.pose.pose.position.z
                bowl_xyz_tllf = mc.get_bowl_pos_in_tllf((bx, by, bz))
                status = mc.circle_mix(bowl_xyz_tllf)
                logger.stops()
                if not status:
                    status = client.load_cartesian(True)
                    return 'failure'
                else:
                    continue
            elif choice == 'l':
                logger.stops()
                rospy.loginfo('Executing state LinearMix')
                logger = EventLoggerClient.startfsm('LinearMix (mixing fsm)')
                mc = MixingClient.get_mixing_client()
                tfl = tf.TransformListener()
                try:
                    (trans, rot) = tfl.lookupTransform('/base_link', '/torso_lift_link', rospy.Time(0)) 
                except Exception as e:
                    print e
                    print 'burning off a bad tf reading and waiting'
                    time.sleep(1)
                    (trans, rot) = tfl.lookupTransform('/base_link', '/torso_lift_link', rospy.Time(0)) 
                if rot is not (0.0,0.0,0.0,1.0):
                    rospy.logerr('nonzero rotation between base link and torso lift link!')
                    print rot
                print trans
                print rot
                bx = userdata.mixing_bowl.pose.pose.position.x
                by = userdata.mixing_bowl.pose.pose.position.y
                bz = userdata.mixing_bowl.pose.pose.position.z
                x = bx - trans[0]  
                y = by - trans[1]
                z = bz - trans[2]
                print 'position in tll: ', x, y, z
                bowl_xyz_tllf = (x,y,z)
                status = mc.linear_mix(bowl_xyz_tllf)
                logger.stops()
                if not status:
                    status = client.load_cartesian(True)
                    return 'failure'
                else:
                    continue
            elif choice == 'r':
                print 'rim plunge no longer supported'
                continue
            elif choice == 's':
                print 'rim plunge no longer supported'
                continue
                logger.stops()
                switch_sides_fsm = assemble_switch_sides_fsm()
                switch_sides_fsm.userdata.object_of_desire = userdata.mixing_bowl
                switch_sides_fsm.userdata.table_frame_x_offset = userdata.table_frame_x_offset
                switch_sides_fsm.userdata.table_frame_y_offset = userdata.table_frame_y_offset
                switch_sides_fsm.userdata.grasp_with_right_hand = False
                outcome = switch_sides_fsm.execute()
                if outcome == 'switching_success':
                    continue
                else:
                    status = client.load_cartesian(True)
                    return 'failure'
            elif choice == 'd':
                logger.stops()
                mlogger = StaticLoggerLocker.check_out_logger('MIX LOGGER')
                if mlogger:
                    mlogger.stops()
                return 'success'
            elif choice == 'a':
                self.debug = False
                status = client.load_cartesian(True)
                return self.execute(userdata)
            else:
                print 'done (looping again now...)'
                continue

    def get_name_str(self):
        return 'mix'

    def get_params_str(self):
		return '(?left ?right ?mb)'

    def get_precondition_str(self):
		return '(and (GRIPPER ?left) (not (SPATULA-GRIPPER ?left)) (GRIPPER ?right) (SPATULA-GRIPPER ?right) (MIXING-BOWL ?mb) (carry ?left ?mb) (in-mb ?right))'

    def get_effect_str(self):
		return '(mixed ?mb)'




class MixDeplunge(FState):
    def __init__(self):
        FState.__init__(self, 
                                   input_keys=['mixing_bowl', 'pre_mix_height_above_table']) 

    def execute(self, userdata):
        rospy.loginfo('Executing state SwitchHandToImpedanceControl')
        logger = EventLoggerClient.startfsm('SwitchHandToImpedanceControl (mixing fsm)')
        client = PR2CMClient.get_pr2cm_client()
        status = client.load_ee_cart_imped(True) 
        logger.stops()
        if not status:
            return 'failure'

        rospy.loginfo('executing state LiftSpoon')
        logger = EventLoggerClient.startfsm('LiftSpoon (mixing fsm)')
        mc = MixingClient.get_mixing_client()
        tfl = tf.TransformListener()
        try:
            (trans, rot) = tfl.lookupTransform('/base_link', '/torso_lift_link', rospy.Time(0)) # TODO: this may be reversed
        except Exception as e:
            print e
            print 'burning off a bad tf reading and waiting'
            time.sleep(1)
            (trans, rot) = tfl.lookupTransform('/base_link', '/torso_lift_link', rospy.Time(0)) # TODO: this may be reversed

        if rot is not (0.0,0.0,0.0,1.0):
            rospy.logerr('nonzero rotation between base link and torso lift link!')
            print rot
        print trans
        print rot
        bx = userdata.mixing_bowl.pose.pose.position.x
        by = userdata.mixing_bowl.pose.pose.position.y
        bz = userdata.mixing_bowl.pose.pose.position.z
        x = bx - trans[0]  
        y = by - trans[1]
        z = bz - trans[2]
        print 'position in tll: ', x, y, z
        bowl_xyz_tllf = (x,y,z)
        print '\n\npremix height above table:', userdata.pre_mix_height_above_table
        if mc.deplunge_spoon(bowl_xyz_tllf, userdata.pre_mix_height_above_table - .4):
            status = client.load_cartesian(True) 
            logger.stops()
            return 'success'
        else:
            rospy.logerr('deplunge failure')
            status = client.load_cartesian(True) 
            logger.stopf()
            return 'failure'

    def get_name_str(self):
        return 'deplunge-spoon'

    def get_params_str(self):
		return '(?y)'

    def get_precondition_str(self):
		return '(and (GRIPPER ?y) (SPATULA-GRIPPER ?y) (in-mb ?y))'

    def get_effect_str(self):
		return '(and (not (in-mb ?y)) (over-mb ?y))'


class MixRetreatSpoon(FState):
    def __init__(self):
        FState.__init__(self, 
                                   input_keys=['mixing_bowl', 'pre_mix_height_above_table']) 
        self.arm_client = ArmClient.get_arm_client()
        self.papm = PickAndPlaceManager.get_pick_and_place_manager()

    def execute(self, userdata):
        rospy.loginfo('Executing state SwitchHandToImpedanceControl')
        logger = EventLoggerClient.startfsm('SwitchHandToImpedanceControl (mixing fsm)')
        client = PR2CMClient.get_pr2cm_client()
        status = client.load_ee_cart_imped(True) 
        logger.stops()
        if not status:
            return 'failure'

        rospy.loginfo('executing state LiftSpoon')
        logger = EventLoggerClient.startfsm('LiftSpoon (mixing fsm)')
        mc = MixingClient.get_mixing_client()
        tfl = tf.TransformListener()
        try:
            (trans, rot) = tfl.lookupTransform('/base_link', '/torso_lift_link', rospy.Time(0)) # TODO: this may be reversed
        except Exception as e:
            print e
            print 'burning off a bad tf reading and waiting'
            time.sleep(1)
            (trans, rot) = tfl.lookupTransform('/base_link', '/torso_lift_link', rospy.Time(0)) # TODO: this may be reversed

        if rot is not (0.0,0.0,0.0,1.0):
            rospy.logerr('nonzero rotation between base link and torso lift link!')
            print rot
        print trans
        print rot
        bx = userdata.mixing_bowl.pose.pose.position.x
        by = userdata.mixing_bowl.pose.pose.position.y
        bz = userdata.mixing_bowl.pose.pose.position.z
        x = bx - trans[0]  
        y = by - trans[1]
        z = bz - trans[2]
        print 'position in tll: ', x, y, z
        bowl_xyz_tllf = (x,y,z)
        print '\n\npremix height above table:', userdata.pre_mix_height_above_table
        if mc.deplunge_spoon(bowl_xyz_tllf, userdata.pre_mix_height_above_table - .4):
            status = client.load_cartesian(True) 
            logger.stops()
            #return 'success'
        else:
            rospy.logerr('deplunge failure')
            status = client.load_cartesian(True) 
            logger.stopf()
            return 'failure'


        rospy.loginfo('Executing state RetreatSpoon')
        mixing_bowl_height = .15  # for clean spoon client
        clean_spoon_fsm = assemble_clean_spoon_fsm(mixing_bowl_height)
        clean_spoon_fsm.userdata.mixing_bowl = userdata.mixing_bowl
        outcome = clean_spoon_fsm.execute()
        if outcome == 'clean_failure':
            print 'cleaning the spoon failed'
            return 'failure'

        isRightArm = True
        rospy.loginfo('Executing state ResetOneArm: ' + str(isRightArm))
        logger = EventLoggerClient.startfsm('ResetOneArm (mixing fsm)')
        client = PR2CMClient.get_pr2cm_client()
        print client.load_cartesian(isRightArm)

        rospy.loginfo('retreating the arm')
        self.arm_client.retreat_arm(isRightArm)

        aname = 'right' if isRightArm else 'left'
        whicharm = 0 if isRightArm else 1
        rospy.loginfo('moving the ' + aname + ' arm to the side')
        self.papm.move_arm_to_side(whicharm)

        rospy.loginfo('closing the ' + aname + ' gripper')
        self.papm.close_gripper(whicharm)
        logger.stops()
        return 'success'

    def get_name_str(self):
        return 'retreat-spoon'

    def get_params_str(self):
		return '(?y ?x ?z)'

    def get_precondition_str(self):
		return '(and (GRIPPER ?y) (GRIPPER ?x) (MIXING-BOWL ?z) (carry ?x ?z) (SPATULA-GRIPPER ?y))'

    def get_effect_str(self):
		return '(and (not (over-mb ?y)) (not (in-mb ?y)) (reset-safe ?y))'


class MixRetreatGrab(FState):
    def __init__(self):
        FState.__init__(self, input_keys=['mixing_bowl', 
                                          'table_frame_x_offset',
                                          'table_frame_y_offset',
                                          'pre_mix_height_above_table']) 

    def execute(self, userdata):
        rospy.loginfo('Executing state RetreatGrab')
        grabbing_retreat_fsm = assemble_retreat_fsm(right_arm = False)
        grabbing_retreat_fsm.userdata.object_of_desire = userdata.mixing_bowl
        grabbing_retreat_fsm.userdata.object_of_desire_name = 'mixing_bowl'
        grabbing_retreat_fsm.userdata.table_frame_x_offset = userdata.table_frame_x_offset
        grabbing_retreat_fsm.userdata.table_frame_y_offset = userdata.table_frame_y_offset
        grabbing_retreat_fsm.userdata.grasp_with_right_hand = False
        grabbing_retreat_fsm.userdata.clock_position = 'nine_o_clock'
        outcome = grabbing_retreat_fsm.execute()
        if outcome == 'grabbing_retreat_failure':
            return 'failure'
        else:
            return 'success'

    def get_name_str(self):
        return 'retreat-grab'

    def get_params_str(self):
		return '(?x ?y ?z)'

    def get_precondition_str(self):
		return '(and (GRIPPER ?y) (MIXING-BOWL ?x) (GRIPPER ?z) (SPATULA-GRIPPER ?z) (not (SPATULA-GRIPPER ?y)) (not (free ?y)))'

    def get_effect_str(self):
		return '(and (free ?y) (not (carry ?y ?x)) (reset-safe ?y))'


class MixResetArm(FState):
    def __init__(self, name=None):
        FState.__init__(self, input_keys=['mixing_bowl', 'pre_mix_height_above_table'])
        self.arm_client = ArmClient.get_arm_client()
        self.papm = PickAndPlaceManager.get_pick_and_place_manager()
        self.name = name
        self.isRightArm = None

    def execute(self, userdata):
        rospy.loginfo('Executing state ResetArm')
        print self.name, self.is_parametrized(), self.parametrized
        logger = EventLoggerClient.startfsm('ResetArm (mixing fsm)')
        if self.isRightArm is None:
            rospy.logwarn('did not parametrize which arm')
            return 'failure'
        if self.isRightArm:
            rospy.loginfo('moving the right arm to the side')
            self.papm.move_arm_to_side(0)
            self.papm.close_gripper(0)
        else:
            rospy.loginfo('moving the left arm to the side')
            self.papm.move_arm_to_side(1)
            self.papm.close_gripper(1)
        logger.stops()
        return 'success'

    def get_name_str(self):
        return 'reset-arm'

    def get_params_str(self):
		return '(?x)'

    def get_precondition_str(self):
		return '(and (GRIPPER ?x) (reset-safe ?x))'

    def get_effect_str(self):
		return '(and (reset ?x))'

    def parametrize(self, ff_line):
        self.isRightArm = (ff_line[0] == 'RARM')
        print 'parametrizing ', self.name,'based on: ', ff_line, 'isrightarm', self.isRightArm
        self.parametrized = True
        return True
