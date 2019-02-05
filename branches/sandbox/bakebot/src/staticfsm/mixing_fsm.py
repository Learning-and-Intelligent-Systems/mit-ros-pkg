#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
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
from rim_plunge_fsm import *
from clean_spoon_fsm import *
from grab_rim_fsm import *
from switch_bowl_sides_fsm import *
from bakebot.srv import *
import math
import smach
import smach_ros
import time

class InitializeForMixing(smach.State):
    def __init__(self, debug = False, standalone = False):
        smach.State.__init__(self, outcomes=['done', 'skip to attach', 'done with mixing'], output_keys=['load_detected_objects_filename'])
        self.debug = debug
        self.standalone = standalone

    def execute(self, userdata):
        rospy.loginfo('Executing state InitializeState')
        if True or self.debug:
            mlogger  = EventLoggerClient('milestone', annotation='refine mb pos and readjust base pos')
            StaticLoggerLocker.check_in_logger('MB LOGGER', mlogger) 
            choice = raw_input('\n\n\npress enter to continue...):')
            if self.standalone: 
                filename = raw_input('enter the filename to load from (blank for detectedobj.log): ')
                if len(filename) == 0:
                    filename = 'detectedobj.log'
                print 'filename: ', filename
                userdata.load_detected_objects_filename = filename
                client = PR2CMClient.get_pr2cm_client()
                print client.load_cartesian(True)
                print client.load_cartesian(False)
                papm = PickAndPlaceManager.get_pick_and_place_manager()
                rospy.loginfo('moving the right arm to the side')
                papm.move_arm_to_side(0)
                rospy.loginfo('moving the left arm to the side')
                papm.move_arm_to_side(1)
                rospy.loginfo('closing both grippers')
                papm.close_gripper(0)
                papm.close_gripper(1)
                rospy.loginfo('done with robot hardware position initialization')
            choice = raw_input('\n\n\npress enter to continue regular init (or q to finish):')
            if len(choice) == 0:
                return 'done'
            elif choice == 'q':
                mlogger.stops()
                return 'done with mixing'
            else:
                return self.execute(userdata)
        else:
            return 'done'

class LoadObjectsFromFile(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                                   input_keys=['load_detected_objects_filename'])
        self.btom = BroadTabletopObjectManager.get_broad_tabletop_object_manager(PickAndPlaceManager.get_pick_and_place_manager())

    def execute(self, userdata):
        rospy.loginfo('Executing state LoadObjectsFromFile')
        if self.btom.load_detected_objects_from_file(userdata.load_detected_objects_filename):
            return 'success'
        else:
            return 'fail'

class FilterIngredients(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'not enough detected objects'],
                                   input_keys=['ingredients_list', 'mixing_bowl', 'cookie_sheet'],
                                   output_keys=['ingredients_list', 'mixing_bowl', 'cookie_sheet'])
        self.btom = BroadTabletopObjectManager.get_broad_tabletop_object_manager(PickAndPlaceManager.get_pick_and_place_manager())

    def execute(self, userdata):
        rospy.loginfo('Executing state FilterIngredients')
        rospy.loginfo("pulling the mixing bowl out of the ingredients, and sorting from left to right")
        detected_objects = self.btom.detected_objects
        if len(detected_objects) < 2:
            rospy.logerr('less than two ingredients detected')
            rospy.logerr('detected_objects')
            return 'not enough detected objects'
        detected_ingredients = list()

        if len(detected_objects) > 6:
            rospy.logerr('too many objects detected.  discarding objects with y value greater than ymax')
            ymax = .65
            print ymax
            removelist = list()
            for obj in detected_objects:
                print 'obj ', obj, ' yval = ', obj.pose.pose.position.y
                if obj.pose.pose.position.y > ymax:
                    print 'going to remove this one!'
                    removelist.append(obj)
            for obj in removelist:
                detected_objects.remove(obj)
            if len(detected_objects) is not 4:
                rospy.logerr('the length of det obj is not four after removing bads: ', str(len(detected_objects)))
                return 'not enough detected objects'


        # first sort the list by y position, take the two lowest out.  they are the cookie sheet and mixing bowl
        sorted_by_y = sorted(detected_objects, key = lambda obj: obj.pose.pose.position.y)
        rightmost = [sorted_by_y[0], sorted_by_y[1]]
        # now sort those two by x
        sorted_by_x = sorted(rightmost, key = lambda obj: obj.pose.pose.position.x)
        mixing_bowl = sorted_by_x[0]
        cookie_sheet = sorted_by_x[1]
        
        sorted_by_y.remove(mixing_bowl)
        sorted_by_y.remove(cookie_sheet)

        if len(sorted_by_y) != 4:  # indicating not a full test table
            print 'not a full table...'
            for obj in sorted_by_y:
                print obj.pose.pose.position.x, obj.pose.pose.position.y
            detected_ingredients = sorted_by_y
        else:
            sorted_by_x = sorted(sorted_by_y, key = lambda obj: obj.pose.pose.position.x)
            # take the first two out (first row)
            first_row = [sorted_by_x[0], sorted_by_x[1]]
            for obj in first_row:
                sorted_by_x.remove(obj)
            second_row = sorted_by_x
            # we want the first row sorted right to left
            first_row = sorted(first_row, key = lambda obj: obj.pose.pose.position.y)
            # we want the second row sorted left to right
            second_row = sorted(second_row, key = lambda obj: -obj.pose.pose.position.y)
            detected_ingredients = list()
            detected_ingredients.extend(first_row)
            detected_ingredients.extend(second_row)
            for obj in detected_ingredients:
                print obj.pose.pose.position.x, obj.pose.pose.position.y
        # package results
        userdata.ingredients_list = detected_ingredients
        userdata.mixing_bowl = mixing_bowl
        userdata.cookie_sheet = cookie_sheet
        print 'mixing bowl at: ', userdata.mixing_bowl.pose.pose.position.x, userdata.mixing_bowl.pose.pose.position.y
        print 'cookie sheetat: ', userdata.cookie_sheet.pose.pose.position.x, userdata.cookie_sheet.pose.pose.position.y
        return 'done'


class RefineMixingBowlPosition(smach.State):
    mixing_bowl_stored = None
    def __init__(self, close_logger=False, store=False, load=False):
        smach.State.__init__(self, outcomes=['done', 'failed'],
                                   input_keys=['mixing_bowl',
                                               'table_frame_x_offset',
                                               'table_frame_y_offset'],
                                   output_keys=['mixing_bowl'])
        self.store = store
        self.load = load
        self.close_logger = close_logger

    def execute(self, userdata):
        logger = EventLoggerClient.startfsm('RefineMixingBowlPosition (mixing fsm)')
        rospy.loginfo('Executing state RefineMixingBowlPosition')
        if self.load:
            userdata.mixing_bowl = RefineMixingBowlPosition.mixing_bowl_stored
            rospy.loginfo('###############  loading the old mixing bowl position #######################\n\n\n')
        x = userdata.mixing_bowl.pose.pose.position.x
        y = userdata.mixing_bowl.pose.pose.position.y
        z = userdata.mixing_bowl.pose.pose.position.z
        if self.store:
            rospy.loginfo('###############  saving the old mixing bowl position #######################\n\n\n')
            RefineMixingBowlPosition.mixing_bowl_stored = userdata.mixing_bowl
        rospy.loginfo('\n\n\noriginal pose of mixing bowl: ' + str((x,y,z)) + '\n\n\n')
        rospy.loginfo('replacing the pose of userdata.mixing_bowl with the pose from the most recent detection')
        btom = BroadTabletopObjectManager.get_broad_tabletop_object_manager(PickAndPlaceManager.get_pick_and_place_manager())
        if not self.load:
            detected_mixing_bowl = btom.refine_and_and_get_graspable_object(userdata.mixing_bowl.collision_name, userdata.table_frame_x_offset, userdata.table_frame_y_offset)
        else:
            print '\n\ndddddddddddddddddddd adjusting y\n\n'
            detected_mixing_bowl = btom.refine_and_and_get_graspable_object(userdata.mixing_bowl.collision_name, userdata.table_frame_x_offset, .2)

        if detected_mixing_bowl == None:
            rospy.logerr('could not find the mixing bowl!')
            logger.stopf()
            return 'failed'
        userdata.mixing_bowl.pose = detected_mixing_bowl.pose
        x = userdata.mixing_bowl.pose.pose.position.x
        y = userdata.mixing_bowl.pose.pose.position.y
        z = userdata.mixing_bowl.pose.pose.position.z
        rospy.loginfo('\n\n\ncorrected pose of mixing bowl: ' + str((x,y,z)) + '\n\n\n')
        logger.stops()
        if self.close_logger:
            mlogger = StaticLoggerLocker.check_out_logger('MB LOGGER')
            if mlogger is not None:
                mlogger.stops()
        return 'done'


class RefineTableFront(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'],
                                   input_keys=['table_front_edge_x'],
                                   output_keys=['table_front_edge_x'])

    def execute(self, userdata):
        rospy.loginfo('Executing state RefineTableFront')
        logger = EventLoggerClient.startfsm('RefineTableFront (mixing fsm)')
        papm = PickAndPlaceManager.get_pick_and_place_manager()
        rospy.loginfo('original table_front_edge_x: ' + str(papm.table_front_edge_x))
        papm.find_table()
        userdata.table_front_edge_x = papm.table_front_edge_x
        rospy.loginfo('updated table_front_edge_x: ' + str(papm.table_front_edge_x))
        logger.stops()
        return 'done'


class MixingMoveRobot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'],
                                   input_keys=['mixing_bowl', 'table_frame_x_offset', 'table_frame_y_offset', 'nominal_mixing_bowl_x', 'nominal_mixing_bowl_y', 'table_front_edge_x'],
                                   output_keys=['mixing_bowl', 'table_frame_x_offset', 'table_frame_y_offset', 'table_front_edge_x'])

    def execute(self, userdata):
        ROBOT_FRONT_X = 0.2 # TODO figure out this number for real!
        TABLE_MARGIN = 0.04
        rospy.loginfo('Executing state MixingMoveRobot')
        logger = EventLoggerClient.startfsm('MixingMoveRobot (mixing fsm)')
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

        #if drive_x > (userdata.table_front_edge_x - ROBOT_FRONT_X - TABLE_MARGIN):
            #rospy.logwarn('the desired drive_x would cause the robot to hit the front of the table ')
            #print 'actual x', actual_x
            #print 'desired x', desired_x
            #print 'drive x', drive_x
            #print 'table x', userdata.table_front_edge_x
            #print 'robot front x', ROBOT_FRONT_X
            #print 'safety margin', TABLE_MARGIN
            #rospy.logwarn('capping the drive_x value to table_front - robot_front - table_margin')
            #drive_x = userdata.table_front_edge_x - ROBOT_FRONT_X - TABLE_MARGIN

        bc.translate(True, drive_x)
        bc.translate(False, drive_y)

        userdata.table_frame_x_offset = -drive_x  #TODO need to reverse this
        userdata.table_frame_y_offset = -drive_y
        print '********** xoff', userdata.table_frame_x_offset
        print '********** yoff', userdata.table_frame_y_offset

        userdata.table_front_edge_x = userdata.table_front_edge_x - drive_x
        logger.stops()
        return 'done'

class ResetRobotBasePos(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'],
                                   input_keys=['mixing_bowl', 'table_frame_x_offset', 'table_frame_y_offset', 'nominal_mixing_bowl_x', 'nominal_mixing_bowl_y', 'table_front_edge_x'],
                                   output_keys=['mixing_bowl', 'table_frame_x_offset', 'table_frame_y_offset', 'table_front_edge_x'])

    def execute(self, userdata):
        ROBOT_FRONT_X = 0.2 # TODO figure out this number for real!
        TABLE_MARGIN = 0.04
        rospy.loginfo('Executing state ResetRobotBasePos')
        logger = EventLoggerClient.startfsm('ResetRobotBasePos (mixing fsm)')
        bc = BaseClient.get_base_client()
        drive_x = userdata.table_frame_x_offset
        drive_y = userdata.table_frame_y_offset
        rospy.loginfo('DESIRED DRIVE X: ' + str(drive_x))
        rospy.loginfo('DESIRED DRIVE Y: ' + str(drive_y))

        #if drive_x > (userdata.table_front_edge_x - ROBOT_FRONT_X - TABLE_MARGIN):
            #rospy.logwarn('the desired drive_x would cause the robot to hit the front of the table ')
            #print 'actual x', actual_x
            #print 'desired x', desired_x
            #print 'drive x', drive_x
            #print 'table x', userdata.table_front_edge_x
            #print 'robot front x', ROBOT_FRONT_X
            #print 'safety margin', TABLE_MARGIN
            #rospy.logwarn('capping the drive_x value to table_front - robot_front - table_margin')
            #drive_x = userdata.table_front_edge_x - ROBOT_FRONT_X - TABLE_MARGIN

        bc.translate(True, drive_x)
        bc.translate(False, drive_y)

        userdata.table_frame_x_offset = 0
        userdata.table_frame_y_offset = 0
        userdata.table_front_edge_x = userdata.table_front_edge_x - drive_x #TODO check this...
        logger.stops()
        return 'done'

class UserBranch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grab', 'clean'])

    def execute(self, userdata):
        rospy.loginfo('Executing state UserBranch')

        choice = raw_input('(g)rab the mixing bowl or (c)lean the spoon: ')
        if choice == 'g':
            return 'grab'
        elif choice == 'c':
            return 'clean'
        else:
            print 'invalid input'
            return self.execute(userdata)
            
class AddSpoonToHandCollisionSpace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state AddSpoonToHandCollisionSpace')
        mlogger  = EventLoggerClient('milestone', annotation='attach spoon and move to post plunge')
        StaticLoggerLocker.check_in_logger('SPOON LOGGER', mlogger) 
        logger = EventLoggerClient.startfsm('AddSpoonToHandCollisionSpace (mixing fsm)')
        rospy.loginfo('waiting for bakebot attach/detach service')
        rospy.wait_for_service('bakebot_attach_detach_service')
        rospy.loginfo('done waiting')
        isRightArm = True
        spoon_length = 0.24 # worked with .2
        spoon_radius = 0.01
        try:
            attach_detach = rospy.ServiceProxy('bakebot_attach_detach_service', AttachDetach)
            response = attach_detach(isRightArm, 1, spoon_length, spoon_radius)
            rospy.loginfo('service responsed with: ' + str(response))

            print '\n\n\n\n\n', 'spoon attached'
            #raw_input('pres enter to continue.........................................................')
        except rospy.ServiceException, e:
            rospy.logerr('service call failed')
            print e
            logger.stopf()
            return 'failed'
        logger.stops()
        return 'done'


class MoveMixingHandToPreMixPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail', 'fail with exhaustion'], 
                                   input_keys=['mixing_bowl', 'pre_mix_height_above_table'])
        self.arm_client = ArmClient.get_arm_client()
        self.papm = PickAndPlaceManager.get_pick_and_place_manager()
        self.failed_once = False

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveMixingHandToPreMixPosition')
        x = userdata.mixing_bowl.pose.pose.position.x
        y = userdata.mixing_bowl.pose.pose.position.y
        z = self.papm.table_height + userdata.pre_mix_height_above_table
        rospy.loginfo('advancing the arm from the side position')
        if not self.failed_once:
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
                    if self.failed_once:
                        rospy.logerr('moving to the pose failed even when detached. failing with exhaustion!')
                        self.logger.stopf()
                        self.failed_once = False
                        return 'fail with exhaustion'
                    else:
                        rospy.logerr('moving to the the pose failed... detaching and trying again')
                        self.failed_once = True
                        return 'fail'
        rospy.loginfo('moving to the pre mix position succeeded')
        self.logger.stops()
        self.failed_once = False
        return 'done'


class ResetBothArms(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                                   input_keys=['mixing_bowl', 'pre_mix_height_above_table'])
        self.arm_client = ArmClient.get_arm_client()
        self.papm = PickAndPlaceManager.get_pick_and_place_manager()

    def execute(self, userdata):
        rospy.loginfo('Executing state ResetBothArms')
        logger = EventLoggerClient.startfsm('ResetBothArms (mixing fsm)')
        rospy.loginfo('moving the right arm to the side')
        self.papm.move_arm_to_side(0)
        rospy.loginfo('moving the left arm to the side')
        self.papm.move_arm_to_side(1)
        rospy.loginfo('closing both grippers')
        self.papm.close_gripper(0)
        self.papm.close_gripper(1)
        logger.stops()
        return 'done'

class RemoveSpoonFromHandCollisionSpace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state RemoveSpoonFromHandCollisionSpace')
        logger = EventLoggerClient.startfsm('RemoveSpoonFromHandCollisionSpace (mixing fsm)')
        rospy.loginfo('waiting for bakebot attach/detach service')
        rospy.wait_for_service('bakebot_attach_detach_service')
        rospy.loginfo('done waiting')
        isRightArm = True
        spoon_length = 0.2
        spoon_radius = 0.01
        try:
            attach_detach = rospy.ServiceProxy('bakebot_attach_detach_service', AttachDetach)
            response = attach_detach(isRightArm, 0, spoon_length, spoon_radius)
            rospy.loginfo('service responsed with: ' + str(response))
        except rospy.ServiceException, e:
            rospy.logerr('service call failed')
            print e
            logger.stopf()
            return 'failed'
        logger.stops()
        return 'done'

class MoveMixingHandToPrePreMixPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail', 'fail with exhaustion'])
        self.arm_client = ArmClient.get_arm_client()
        self.papm = PickAndPlaceManager.get_pick_and_place_manager()
        self.failed_once = False

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveMixingHandToPre Pre MixPosition')
        x =  .4
        y = -.25  #TODO: figure out what these need to be
        z = 1.2
        if not self.failed_once:
            self.logger = EventLoggerClient.startfsm('MoveMixingHandToPrePreMixPosition (mixing fsm)')
            status = self.arm_client.move_to_pose(True, True, x, y, z, -.5, .5, .5, .5)  # p 120, num 1. TODO  may need to change to better
        else:
            tol = 0.1
            min_max_x_pos = (x - tol, x + tol)
            min_max_y_pos = (y - tol, y + tol)
            min_max_z_pos = (z - tol, z + tol)
            quater_xyzw = (-.5, .5, .5, .5) # TODO: may need to change this
            status = self.arm_client.try_hard_to_move_to_pose(userdata.is_right_arm, True, min_max_x_pos, min_max_y_pos, min_max_z_pos, quater_xyzw, 0.02, (determination > 1), recursionsRemaining)
        if status is not 0:
            if self.failed_once:
                rospy.logerr('moving to the pose failed even when detached. failing with exhaustion!')
                self.logger.stopf()
                return 'fail with exhaustion'
            else:
                rospy.logerr('moving to the the pose failed... detaching and trying again')
                self.failed_once = True
                return 'fail'
        else:
            rospy.loginfo('moving to the pre mix position succeeded')
            self.logger.stops()
            return 'done'


class SwitchHandController(smach.State):
    def __init__(self, switch_to_imped):
        smach.State.__init__(self, outcomes=['done', 'fail'])
        self.switch_to_imped = switch_to_imped

    def execute(self, userdata):
        rospy.loginfo('Executing state SwitchHandToImpedanceControl')
        logger = EventLoggerClient.startfsm('SwitchHandToImpedanceControl (mixing fsm)')
        client = PR2CMClient.get_pr2cm_client()
        if self.switch_to_imped:
            status = client.load_ee_cart_imped(True) 
        else:
            status = client.load_cartesian(True)
        logger.stops()
        return 'done' if status else 'fail'


class PlungeSpoon(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
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
        logger.stops()
        return 'done' if status else 'fail'


class AssessMix(smach.State):
    def __init__(self, debug = True):
        smach.State.__init__(self, outcomes=['done', 'linear', 'circular', 'rim plunge', 'whisk', 'switch sides', 'fail'],
                                   input_keys=['post_reset_branch_result', 'post_switch_branch_result', 'mixing_laps', 'mixing_bowl'],
                                   output_keys=['post_reset_branch_result', 'post_switch_branch_result'])
        self.num_mixes_done = 0
        self.debug = debug
        self.first_pass = True
        self.mix_list = ['circular', 'linear', 'circular', 'switch sides', 'circular', 'linear']
        self.iterator = iter(self.mix_list)

    def execute(self, userdata):
        rospy.loginfo('Executing state AssessMix')

        # TODO: get rid of this for the real test
        mc = MixingClient.get_mixing_client()
        #mc.DRY_RUN = True

        if self.first_pass:
            rospy.loginfo('first pass at assess mix')
            self.iterator = iter(self.mix_list)
            mlogger = StaticLoggerLocker.check_out_logger('SPOON LOGGER')
            mlogger.stops()
            tlogger  = EventLoggerClient('milestone', annotation='mixing')
            StaticLoggerLocker.check_in_logger('MIX LOGGER', tlogger)
            self.first_pass = False
            self.num_mixes_done = 0
            #if userdata.mixing_laps < 100: # no flag set
                #self.mixing_laps = userdata.mixing_laps
            #else:
                #rospy.loginfo('caught flag in mixing laps, setting up to not switch sides')
                #self.mixing_laps = userdata.mixing_laps - 100
        logger = EventLoggerClient.startfsm('AssessMix (mixing fsm)')
        if not self.debug:
            try:
                action = self.iterator.next()
                rospy.loginfo('commanding a ' + str(action) + ' mix')
                if action == 'switch sides':
                    userdata.post_reset_branch_result = 'switch_sides'
                    userdata.post_switch_branch_result = 'attach_spoon'
                return action
            except StopIteration:
                rospy.loginfo('done mixing!')
                logger.stops()
                mlogger = StaticLoggerLocker.check_out_logger('MIX LOGGER')
                mlogger.stops()
                userdata.post_reset_branch_result = 'retreat_grab'
                userdata.post_switch_branch_result = 'retreat_grab'
                self.first_pass = True
                return 'done'
        else:
            while True:
                choice = raw_input('\n (s)witch sides, (c)ircle mix, (l)inear mix, (r)im plunge, (a)ll, (d)one: ')
                if choice == 'c':
                    logger.stops()
                    return 'circular'
                elif choice == 'l':
                    logger.stops()
                    return 'linear'
                elif choice == 'r':
                    logger.stops()
                    return 'rim plunge'
                elif choice == 's':
                    logger.stops()
                    userdata.post_reset_branch_result = 'switch_sides'
                    userdata.post_switch_branch_result = 'attach_spoon'
                    return 'switch sides'
                elif choice == 'd':
                    userdata.post_reset_branch_result = 'retreat_grab'
                    userdata.post_switch_branch_result = 'retreat_grab'
                    logger.stops()
                    mlogger = StaticLoggerLocker.check_out_logger('MIX LOGGER')
                    mlogger.stops()
                    return 'done'
                elif choice == 'a':
                    self.debug = False
                    return self.execute(userdata)
                else:
                    print 'done'
                    continue


class CircleMix(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                                   input_keys=['mixing_bowl']) 

    def execute(self, userdata):
        rospy.loginfo('Executing state CircleMix')
        logger = EventLoggerClient.startfsm('CircleMix (mixing fsm)')
        mc = MixingClient.get_mixing_client()
        bx = userdata.mixing_bowl.pose.pose.position.x
        by = userdata.mixing_bowl.pose.pose.position.y
        bz = userdata.mixing_bowl.pose.pose.position.z
        bowl_xyz_tllf = mc.get_bowl_pos_in_tllf((bx, by, bz))
        status = mc.circle_mix(bowl_xyz_tllf)
        logger.stops()
        return 'done' if status else 'fail'


class LinearMix(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                                   input_keys=['mixing_bowl']) 

    def execute(self, userdata):
        rospy.loginfo('Executing state LinearMix')
        logger = EventLoggerClient.startfsm('LinearMix (mixing fsm)')
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

        x = bx - trans[0]  #TODO: check this
        y = by - trans[1]
        z = bz - trans[2]
        print 'position in tll: ', x, y, z
        
        bowl_xyz_tllf = (x,y,z)

        status = mc.linear_mix(bowl_xyz_tllf)
        logger.stops()
        return 'done' if status else 'fail'


class WhiskMix(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                                   input_keys=['mixing_bowl']) 

    def execute(self, userdata):
        rospy.loginfo('Executing state WhiskMix')
        rospy.logwarn('not yet implemented!')
        return 'done'
        #mc = MixingClient.get_mixing_client()
        #pose = userdata.mixing_bowl.pose
        #status = mc.linear_mix(bowl_xyz_tllf)
        #return 'done' if status else 'fail'


# will want ot do this with the cartesian controller
class LiftSpoon(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'],
                                   input_keys=['mixing_bowl', 'pre_mix_height_above_table']) 

    def execute(self, userdata):
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

        x = bx - trans[0]  #TODO: check this
        y = by - trans[1]
        z = bz - trans[2]
        print 'position in tll: ', x, y, z
        
        bowl_xyz_tllf = (x,y,z)

        print '\n\npremix height above table:', userdata.pre_mix_height_above_table
        if mc.deplunge_spoon(bowl_xyz_tllf, userdata.pre_mix_height_above_table - .4):
            logger.stops()
            return 'done'
        else:
            rospy.logerr('deplunge failure')
            logger.stopf()
            return 'fail'



class Branch(smach.State):
    # the switching_key has to be one of the outcomes
    def __init__(self, outcome1, outcome2, switching_key):
        smach.State.__init__(self, outcomes=[outcome1, outcome2],
                                   input_keys=[switching_key]) 
        self.switching_key = switching_key
    def execute(self, userdata):
        rospy.loginfo('Executing state Branch')
        code = 'retval = userdata.' + self.switching_key
        try:
            exec code
        except Exception as e:
            print e
            raw_input('an exception occurred, prese enter to continue')
        return retval


class ResetOneArm(smach.State):
    def __init__(self, isRightArm, retreat=False):
        smach.State.__init__(self, outcomes=['done', 'fail'])
        self.arm_client = ArmClient.get_arm_client()
        self.papm = PickAndPlaceManager.get_pick_and_place_manager()
        self.isRightArm = isRightArm
        self.retreat_first = retreat

    def execute(self, userdata):
        rospy.loginfo('Executing state ResetOneArm: ' + str(self.isRightArm))
        logger = EventLoggerClient.startfsm('ResetOneArm (mixing fsm)')
        client = PR2CMClient.get_pr2cm_client()
        print client.load_cartesian(self.isRightArm)

        if self.retreat_first:
            rospy.loginfo('retreating the right arm')
            self.arm_client.retreat_arm(self.isRightArm)

        aname = 'right' if self.isRightArm else 'left'
        whicharm = 0 if self.isRightArm else 1
        rospy.loginfo('moving the ' + aname + ' arm to the side')
        if not self.isRightArm:
            self.papm.move_arm_to_side(whicharm)
        #TODO isn't there another move_arm thing that should go here? (for the left arm)

        rospy.loginfo('closing the ' + aname + ' gripper')
        self.papm.close_gripper(whicharm)
        logger.stops()
        return 'done'


def assemble_mixing_fsm(DEBUG = False, standalone = False):
    sm = smach.StateMachine(outcomes=['mixing_success', 'mixing_failure'],
                            input_keys=['nominal_mixing_bowl_x', 'nominal_mixing_bowl_y', 'mixing_bowl', 'pre_mix_height_above_table', 'table_frame_x_offset', 'table_frame_y_offset', 'mixing_laps'],
                            output_keys=['nominal_mixing_bowl_x', 'nominal_mixing_bowl_y', 'mixing_bowl', 'pre_mix_height_above_table', 'table_frame_x_offset', 'table_frame_y_offset'])


############################################## iros demo parameters
    mixing_bowl_height = .15  # for clean spoon client
    grab_zgoal = .86 # for grab rim
##############################################

    fail_state = 'mixing_failure'
    sm.userdata.grasp_with_right_hand = False
    sm.userdata.clock_position = 'nine_o_clock'

    with sm:
        post_init = 'LOAD FROM FILE' if standalone else 'REFINE M-BOWL POS'
        smach.StateMachine.add('INIT FOR MIXING', InitializeForMixing(DEBUG, standalone),
                                transitions={'done':post_init,
                                             'skip to attach':'ATTACH SPOON TO COLL MAP',
                                             'done with mixing':'mixing_success'})

        if standalone:
            sm.userdata.nominal_mixing_bowl_x = 0.40
            sm.userdata.nominal_mixing_bowl_y = -0.25
            sm.userdata.pre_mix_height_above_table = .6
            sm.userdata.table_frame_x_offset = 0
            sm.userdata.table_frame_y_offset = 0
            sm.userdata.acceptable_height_above_table = .5
            sm.userdata.pour_height_above_centroid = .4
            sm.userdata.mixing_laps = 3
            smach.StateMachine.add('LOAD FROM FILE', LoadObjectsFromFile(),
                                    transitions={'success':'FILTER INGREDIENTS',
                                                 'fail':'LOAD FROM FILE'},
                                    remapping={'load_detected_objects_filename':'load_detected_objects_filename'})


            smach.StateMachine.add('FILTER INGREDIENTS', FilterIngredients(),
                                    transitions={'done':'REFINE M-BOWL POS',
                                                 'not enough detected objects':fail_state},
                                    remapping={'ingredients_list':'ingredients_list',
                                               'cookie_sheet':'cookie_sheet',
                                               'mixing_bowl':'mixing_bowl'})


        smach.StateMachine.add('REFINE M-BOWL POS', RefineMixingBowlPosition(close_logger = True, store = True),
                                transitions={'done':'REFINE TABLE FRONT',
                                             'failed':fail_state},
                                remapping={'mixing_bowl':'mixing_bowl',
                                           'table_frame_x_offset':'table_frame_x_offset',
                                           'table_frame_y_offset':'table_frame_y_offset'})

        smach.StateMachine.add('REFINE TABLE FRONT', RefineTableFront(),
                                transitions={'done':'MOVE ROBOT'},
                                remapping={'table_front_edge_x':'table_front_edge_x'})

        smach.StateMachine.add('MOVE ROBOT', MixingMoveRobot(),
                                transitions={'done':'REFINE M-BOWL POS AGAIN'},
                                remapping={'mixing_bowl':'mixing_bowl', 
                                           'table_frame_x_offset':'table_frame_x_offset',
                                           'table_frame_y_offset':'table_frame_y_offset',
                                           'nominal_mixing_bowl_x':'nominal_mixing_bowl_x',
                                           'nominal_mixing_bowl_y':'nominal_mixing_bowl_y',
                                           'table_front_edge_x':'table_front_edge_x'})

        #post_refine = 'USER BRANCH' if DEBUG else 'GRAB MB'
        post_refine = 'GRAB MB'
       #smach.StateMachine.add('REFINE M-BOWL POS AGAIN', RefineMixingBowlPosition(),
       #                        transitions={'done':'ATTACH SPOON TO COLL MAP',
       #                                     'failed':fail_state},
       #                        remapping={'mixing_bowl':'mixing_bowl',
       #                                   'table_frame_x_offset':'table_frame_x_offset',
       #                                   'table_frame_y_offset':'table_frame_y_offset'})
        smach.StateMachine.add('REFINE M-BOWL POS AGAIN', RefineMixingBowlPosition(),
                                transitions={'done':post_refine,
                                             'failed':fail_state},
                                remapping={'mixing_bowl':'mixing_bowl',
                                           'table_frame_x_offset':'table_frame_x_offset',
                                           'table_frame_y_offset':'table_frame_y_offset'})

        smach.StateMachine.add('USER BRANCH', UserBranch(),
                                transitions={'grab':'GRAB MB',
                                             'clean':'PRE CLEAN ATTACH SPOON'},
                                remapping={'mixing_bowl':'mixing_bowl',
                                           'table_frame_x_offset':'table_frame_x_offset',
                                           'table_frame_y_offset':'table_frame_y_offset'})

        grabbing_fsm = assemble_grab_rim_fsm(zgoal=grab_zgoal)
        smach.StateMachine.add('GRAB MB', grabbing_fsm,
                                transitions={'grabbing_success':'ATTACH SPOON TO COLL MAP',
                                             'grabbing_failure': fail_state},
                                remapping={'object_of_desire':'mixing_bowl'})
        grabbing_fsm.userdata.object_of_desire_name = 'mixing_bowl'

        smach.StateMachine.add('ATTACH SPOON TO COLL MAP', AddSpoonToHandCollisionSpace(),
                                transitions={'done':'MOVE HAND TO PRE MIX POS',
                                             'failed':'MOVE HAND TO PRE PRE MIX POS'})

        smach.StateMachine.add('MOVE HAND TO PRE MIX POS', MoveMixingHandToPreMixPosition(),
                                transitions={'done':'SWITCH HAND TO IMPED CTRL',
                                             'fail':'REMOVE SPOON FROM COLL MAP',
                                             'fail with exhaustion':fail_state},
                                remapping={'mixing_bowl':'mixing_bowl',
                                           'pre_mix_height_above_table':'pre_mix_height_above_table'})

        smach.StateMachine.add('REMOVE SPOON FROM COLL MAP', RemoveSpoonFromHandCollisionSpace(),
                                transitions={'done':'MOVE HAND TO PRE PRE MIX POS',
                                             'failed':fail_state})

        smach.StateMachine.add('MOVE HAND TO PRE PRE MIX POS', MoveMixingHandToPrePreMixPosition(),
                                transitions={'done':'MOVE HAND TO PRE MIX POS',
                                             'fail':'MOVE HAND TO PRE PRE MIX POS',
                                             'fail with exhaustion':fail_state})

        smach.StateMachine.add('SWITCH HAND TO IMPED CTRL', SwitchHandController(True),
                                transitions={'done':'PLUNGE SPOON',
                                             'fail':fail_state})

        smach.StateMachine.add('PLUNGE SPOON', PlungeSpoon(),
                                transitions={'done':'ASSESS MIX',
                                             'fail':fail_state})

        smach.StateMachine.add('ASSESS MIX', AssessMix(DEBUG),
                                transitions={'done':'LIFT SPOON',
                                             'circular':'CIRCLE MIX',
                                             'linear':'LINEAR MIX',
                                             'switch sides':'LIFT SPOON',
                                             'whisk':'PRE WHISK SWITCH', #TODO will have to get rid of all whisk
                                             'rim plunge':'RIM PLUNGE',
                                             'fail': fail_state},
                                remapping={'mixing_bowl':'mixing_bowl',
                                           'mixing_laps':'mixing_laps'})

        rim_plunge_fsm = assemble_rim_plunge_fsm()
        smach.StateMachine.add('RIM PLUNGE', rim_plunge_fsm,
                                transitions={'rp_success':'ASSESS MIX',
                                             'rp_failure': fail_state},
                                remapping={'mixing_bowl':'mixing_bowl'})

        smach.StateMachine.add('CIRCLE MIX', CircleMix(),
                                transitions={'done':'ASSESS MIX',
                                             'fail': fail_state},
                                remapping={'mixing_bowl':'mixing_bowl'})

        smach.StateMachine.add('LINEAR MIX', LinearMix(),
                                transitions={'done':'ASSESS MIX',
                                             'fail': fail_state},
                                remapping={'mixing_bowl':'mixing_bowl'})

        smach.StateMachine.add('PRE WHISK SWITCH', SwitchHandController(False),
                                transitions={'done':'WHISK MIX',
                                             'fail': fail_state})

        smach.StateMachine.add('WHISK MIX', WhiskMix(),
                                transitions={'done':'POST WHISK SWITCH',
                                             'fail': fail_state},
                                remapping={'mixing_bowl':'mixing_bowl'})
        
        smach.StateMachine.add('POST WHISK SWITCH', SwitchHandController(True),
                                transitions={'done':'ASSESS MIX',
                                             'fail': fail_state})

        smach.StateMachine.add('LIFT SPOON', LiftSpoon(),
                                transitions={'done':'SWITCH HAND TO POSITION CTRL',
                                             'fail': fail_state},
                                remapping={'mixing_bowl':'mixing_bowl',
                                           'pre_mix_height_above_table': 'pre_mix_height_above_table'})

        smach.StateMachine.add('SWITCH HAND TO POSITION CTRL', SwitchHandController(False),
                                transitions={'done':'CLEAN SPOON',
                                             'fail': fail_state})

        smach.StateMachine.add('PRE CLEAN ATTACH SPOON', AddSpoonToHandCollisionSpace(),
                                transitions={'done':'CLEAN SPOON',
                                             'failed':'CLEAN SPOON'})

        clean_spoon_fsm = assemble_clean_spoon_fsm(mixing_bowl_height)
        smach.StateMachine.add('CLEAN SPOON', clean_spoon_fsm,
                                transitions={'clean_success':'RESET RIGHT ARM',
                                             'clean_failure': 'RESET RIGHT ARM'})

        smach.StateMachine.add('RESET RIGHT ARM', ResetOneArm(True, True),
                                transitions={'done':'POST RESET BRANCH',
                                             'fail': fail_state})

        smach.StateMachine.add('POST RESET BRANCH', Branch('switch_sides', 'retreat_grab', 'post_reset_branch_result'),
                                transitions={'switch_sides':'SWITCH SIDES',
                                             'retreat_grab': 'RETREAT GRAB'})

        switch_sides_fsm = assemble_switch_sides_fsm()
        smach.StateMachine.add('SWITCH SIDES', switch_sides_fsm,
                                transitions={'switching_success':'POST SWITCH BRANCH',
                                             'switching_failure': 'RESET BOTH ARMS'},
                                remapping={'object_of_desire':'mixing_bowl'})

        smach.StateMachine.add('POST SWITCH BRANCH', Branch('attach_spoon', 'retreat_grab', 'post_switch_branch_result'),
                                transitions={'attach_spoon':'ATTACH SPOON TO COLL MAP',
                                             'retreat_grab': 'RETREAT GRAB'})

        grabbing_retreat_fsm = assemble_retreat_fsm(right_arm = False)
        smach.StateMachine.add('RETREAT GRAB', grabbing_retreat_fsm,
                                transitions={'grabbing_retreat_success':'MOVE ROBOT BACK',
                                             'grabbing_retreat_failure': fail_state},
                                remapping={'object_of_desire':'mixing_bowl'})

        smach.StateMachine.add('MOVE ROBOT BACK', ResetRobotBasePos(),
                                transitions={'done':'RESET BOTH ARMS'},
                                remapping={'mixing_bowl':'mixing_bowl', 
                                           'table_frame_x_offset':'table_frame_x_offset',
                                           'table_frame_y_offset':'table_frame_y_offset',
                                           'nominal_mixing_bowl_x':'nominal_mixing_bowl_x',
                                           'nominal_mixing_bowl_y':'nominal_mixing_bowl_y',
                                           'table_front_edge_x':'table_front_edge_x'})

        smach.StateMachine.add('RESET BOTH ARMS', ResetBothArms(),
                                transitions={'done':'REFINE M-BOWL POS 3',
                                             'fail': fail_state})

        smach.StateMachine.add('REFINE M-BOWL POS 3', RefineMixingBowlPosition(load = True),
                                transitions={'done':'mixing_success',
                                             'failed':fail_state},
                                remapping={'mixing_bowl':'mixing_bowl',
                                           'table_frame_x_offset':'table_frame_x_offset',
                                           'table_frame_y_offset':'table_frame_y_offset'})

    return sm


def main():

    try:
        time.sleep(2)
        logger = EventLoggerClient.startlogger()
        sm = assemble_mixing_fsm(DEBUG = True, standalone = True)
        sis = smach_ros.IntrospectionServer('introspection_server', sm, '/MIX')
        sis.start()
        #print 'kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk'
        #rospy.spin()
        outcome = sm.execute()
    finally:
        sis.stop()

if __name__ == '__main__':
    rospy.init_node('mixing_fsm')
    main()

