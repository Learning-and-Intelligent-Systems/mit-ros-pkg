#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
import rospy
import tf
import actionlib
from clients.arm_client import ArmClient
from clients.base_client import BaseClient
from utilities.bakebot_pick_and_place_manager import *
from services.bakebot_logging import *
from utilities.broad_table_object_manager import *
from bowl_dealer_fsm import *
from utilities.bowl_dealer_utilities import *
from mixing_fsm import *
from ovening_fsm import *
from grab_rim_fsm import *
from scraping_fsm import *
import math
import smach
import smach_ros
import time

class RobotInitializeState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'])

    def execute(self, userdata):
        logger = EventLoggerClient.startfsm('RobotInitialzeState (bakebot fsm)')
        rospy.loginfo('Executing state RobotInitializeState')
        papm = PickAndPlaceManager.get_pick_and_place_manager()
        rospy.logwarn('clearing collision map')
        papm.reset_collision_map()

        rospy.loginfo('**************** CHECK TERMINAL OUTPUT **********')
        print '\n\n\n\n\nWARNING: the arms will swing back to the side.  make sure they are moved so nothing is hit!\n\n\n\n\n'
        choice = raw_input('\n\npress enter when clear (or (s)kip): ')
        if len(choice) == 1:
            rospy.loginfo('skipping')
            logger.stops()
            return 'success'

        rospy.loginfo('switching both arms to cartesian control')
        client = PR2CMClient.get_pr2cm_client()
        print client.load_cartesian(True)
        print client.load_cartesian(False)

        rospy.loginfo('moving the right arm to the side')
        papm.move_arm_to_side(0)
        rospy.loginfo('moving the left arm to the side')
        papm.move_arm_to_side(1)

        rospy.loginfo('closing both grippers')
        papm.close_gripper(0)
        papm.close_gripper(1)
        rospy.loginfo('done with robot hardware position initialization')
        logger.stops()
        return 'success'


class UserInitializeState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['scan', 'load', 'recurse'],
                                   input_keys=['save_detected_objects_filename', 'load_detected_objects_filename'],
                                   output_keys=['save_detected_objects_filename', 'load_detected_objects_filename', 'mixing_laps', 'branch_outcome'])
        self.btom = BroadTabletopObjectManager.get_broad_tabletop_object_manager(PickAndPlaceManager.get_pick_and_place_manager())

    def execute(self, userdata):
        rospy.loginfo('Executing state UserInitializeState')
        logger = EventLoggerClient.startfsm('UserInitializeState (bakebot fsm)')
        print '\n\n\n\n'
        print 'Welcome to the BAKEBOT Finite State Machine\n'
        print '\n\n'
        #'recurse', 'scan', 'load from file', 'skip to mixing', 'skip to scraping', 'skip to ovening'],
        choice = raw_input('Would you like to (s)can, (l)oad and deal, or load and (m)ix, or load and s(c)rape, or load and (o)ven: ')

        if choice == 's':
            filename = raw_input('enter the filename to save to (blank for detectedobj.log): ')
            if len(filename) == 0:
                filename = 'detectedobj.log'
            print 'filename: ', filename
            userdata.save_detected_objects_filename = filename

            print '\nscanning the table...\n\n\n'
            logger.stops()
            userdata.branch_outcome = 'deal'
            return 'scan'
        elif choice == 'l':
            filename = raw_input('enter the filename to load from (blank for detectedobj.log): ')
            if len(filename) == 0:
                filename = 'detectedobj.log'
            print 'filename: ', filename
            userdata.load_detected_objects_filename = filename

            print '\nloading the objects from a file (via pickle)'
            logger.stops()
            userdata.branch_outcome = 'skip to dealing'
            return 'load'
        elif choice == 'c':
            filename = raw_input('enter the filename to load from (blank for detectedobj.log): ')
            if len(filename) == 0:
                filename = 'detectedobj.log'
            print 'filename: ', filename
            userdata.load_detected_objects_filename = filename

            print '\nloading the objects from a file (via pickle)'
            print 'going right to the scraping'
            logger.stops()
            userdata.branch_outcome = 'skip to scraping'
            return 'load'
        elif choice == 'm':
            filename = raw_input('enter the filename to load from (blank for detectedobj.log): ')
            if len(filename) == 0:
                filename = 'detectedobj.log'
            print 'filename: ', filename
            userdata.load_detected_objects_filename = filename

            print '\nloading the objects from a file (via pickle)'
            print 'going right to the mixing'
            logger.stops()
            userdata.branch_outcome = 'skip to mixing'
            userdata.mixing_laps = 7 # TODO: increase this
            return 'load'
        elif choice == 'o':
            filename = raw_input('enter the filename to load from (blank for detectedobj.log): ')
            if len(filename) == 0:
                filename = 'detectedobj.log'
            print 'filename: ', filename
            userdata.load_detected_objects_filename = filename

            print '\nloading the objects from a file (via pickle)'
            print 'going right to the ovening'
            logger.stops()
            userdata.branch_outcome = 'skip to ovening'
            return 'load'
        else:
            logger.stopf()
            return 'recurse'

class Branch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['deal', 'skip to dealing', 'scan', 'skip to mixing', 'skip to scraping', 'skip to ovening'],
                                   input_keys=['branch_outcome'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Branch: ' + str(userdata.branch_outcome))
        return userdata.branch_outcome


class ScanTabletop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                                   input_keys=['save_detected_objects_filename'])
        self.btom = BroadTabletopObjectManager.get_broad_tabletop_object_manager(PickAndPlaceManager.get_pick_and_place_manager())

    def execute(self, userdata):
        rospy.loginfo('Executing state ScanTabletop')
        logger = EventLoggerClient.startfsm('ScanTabletop (bakebot fsm)')
        self.btom.do_broad_tabletop_object_detection()  # stores the result internally
        if userdata.save_detected_objects_filename is not None:
            rospy.logwarn('saving the detected objects to: ' + str(userdata.save_detected_objects_filename))
            status = self.btom.save_detected_objects_to_file(userdata.save_detected_objects_filename)
            userlogger = EventLoggerClient.bpstart('waiting for user input in scan tabletop')
            print '\n\n\n\n***************************************************************************************'
            raw_input('detected objects saved.  press enter to continue...')
            userlogger.stops()
            if not status:
                rospy.logwarn('saving detected objects failed')
        else:
            rospy.loginfo('not saving detected objects')
        logger.stops()
        return 'success'



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

        ## first sort the list by x position, the one with the smallest value is the big bowl
       #sorted_by_x = sorted(detected_objects, key=lambda obj: obj.pose.pose.position.x) 
       #for i, obj in enumerate(sorted_by_x):
       #    print i, obj.collision_name, obj.pose.pose.position.x
       #mixing_bowl = sorted_by_x[0]
       #sorted_by_x.remove(mixing_bowl)

       ## now remove the cookie sheet
       ##cookie_sheet = detected_ingredients[-1]
       ##detected_ingredients.remove(cookie_sheet)

       ## now that the mixing bowl is removed, all that is left is the ingredients and the cookie sheet
       #detected_ingredients = sorted_by_x

       ## the cookie sheet is the one that is left with the minium absolute value y position (closest to x axis)
       #sorted_by_xaxis = sorted(detected_ingredients, key=lambda obj: abs(obj.pose.pose.position.y))
       #cookie_sheet = sorted_by_xaxis[0]
       #sorted_by_xaxis.remove(cookie_sheet)

       ## sort the ingredients by reverse x
       ##sorted_by_y = sorted(detected_ingredients, key=lambda obj: obj.pose.pose.position.y)
       #sorted_by_y = sorted(sorted_by_xaxis, key=lambda obj: -obj.pose.pose.position.x)
       #detected_ingredients = sorted_by_y
    

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




class DealMaster(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['deal ingredient', 'mix', 'done'],
                                   input_keys=['ingredients_list'],
                                   output_keys=['bowl_to_deal', 'mixing_laps'])
        self.first_pass = True
        self.last_pass = False
        self.singledeallogger = None
        self.DEBUG_DEAL = False

    def execute(self, userdata):
        rospy.loginfo('Executing state DealMaster')
        if self.first_pass:
            self.logger = EventLoggerClient.startfsm('DealMaster firstpass (bakebot fsm)')
            self.first_pass = False
            self.remaining_ingredients_to_deal = userdata.ingredients_list
            self.remaining_ingredients_to_deal.reverse() # so that I can just pop off the back
            if len(userdata.ingredients_list) == 4 and not self.DEBUG_DEAL:
                # ideal order: sugar mix cocoa flower mix cornflakes  (last mix is guaranteed)
                # ingred_list: sugar cocoa flower cornflakes
                # reversed: cornflakes flower cocoa sugar
                # I want: cornflakes mix flower cocoa mix sugar


                self.remaining_ingredients_to_deal.insert(1, 'mix')
                #self.remaining_ingredients_to_deal.insert(4, 'mix')
                print 'firstpass bakebot dealmaster: ', self.remaining_ingredients_to_deal
            else:
                rospy.logwarn('THERE ARE NOT ENOUGH INGREDIENTS IN THE LIST TO DO A REAL DEAL')

        #if not self.DEBUG_DEAL:
        if True:
            if len(self.remaining_ingredients_to_deal) is 0:
                if self.last_pass:
                    rospy.loginfo('all done with deal master')
                    self.mixlogger.stops('should be going to scrape now')
                    # this is going to go to the scrape
                    return 'done'
                else:
                    rospy.loginfo('out of ingredients')
                    if self.singledeallogger is not None:
                        self.singledeallogger.stops()
                    self.logger.stops('done with deal master, doing one last mix')
                    rospy.loginfo(str(self.remaining_ingredients_to_deal))
                    self.last_pass = True
                    userdata.mixing_laps = 7 # TODO: increase this
                    self.mixlogger = EventLoggerClient.startfsm('MIX fsm')
                    return 'mix'
            else:
               if self.singledeallogger is not None:
                   self.singledeallogger.stops()
               bowl_to_deal = self.remaining_ingredients_to_deal.pop()
               if bowl_to_deal == 'mix':
                   rospy.loginfo('its time to do a mix******************************************')
                   userdata.mixing_laps = 13 
                   return 'mix'
               else: 
                   self.singledeallogger = EventLoggerClient.startfsm('dealing 1 ingredient')
                   userdata.bowl_to_deal = bowl_to_deal
                   return 'deal ingredient'
        else:
            pass
            #for i, ingred in enumerate(self.remaining_ingredients_to_deal):
                #if ingred == 'mix':
                    #print i, ingred
                #else:
                    #print i, 'ingred at: ', ingred.pose.pose.position.x, ingred.pose.pose.position.y 
            #print '100: done'
            #try:
                #choice = int(raw_input('enter choice to skip to: '))
                #if choice == 100:
            
#       rospy.loginfo('Executing state DealMaster')
#       if self.first_pass:
#           self.logger = EventLoggerClient.startfsm('DealMaster firstpass (bakebot fsm)')
#           self.first_pass = False
#           self.remaining_ingredients_to_deal = userdata.ingredients_list
#           self.remaining_ingredients_to_deal.reverse() # so that I can just pop off the back

#       if len(self.remaining_ingredients_to_deal) is not 0:
#           if len(self.remaining_ingredients_to_deal) is 3 and (not self.DEBUG_DEAL):
#               rospy.loginfo('\n\n ********************* already dealt one.  mixing the butter in ***************\n\n') 
#               userdata.mixing_laps = 4
#               return 'mix'
#           elif len(self.remaining_ingredients_to_deal) is 1 and (not self.DEBUG_DEAL):
#               rospy.loginfo('\n\n ********************* already dealt three.  mixing the flour and cocoa in ***************\n\n') 
#               userdata.mixing_laps = 4
#               return 'mix'
#           else:
#           if self.singledeallogger is not None:
#               self.singledeallogger.stops()
#           self.singledeallogger = EventLoggerClient.startfsm('dealing 1 ingredient')
#           userdata.bowl_to_deal = self.remaining_ingredients_to_deal.pop()
#           return 'deal ingredient'
#       else:
#           if self.last_pass:
#               rospy.loginfo('all done with deal master')
#               self.mixlogger.stops('should be going to scrape now')
#               # this is going to go to the scrape
#               return 'done'
#           rospy.loginfo('out of ingredients')
#           if self.singledeallogger is not None:
#               self.singledeallogger.stops()
#           self.logger.stops('done with deal master')
#           rospy.loginfo(str(self.remaining_ingredients_to_deal))
#           self.last_pass = True
#           userdata.mixing_laps = 5 # TODO: increase this
#           self.mixlogger = EventLoggerClient.startfsm('MIX fsm')
#           return 'mix'
    

def assemble_bakebot_fsm():
    fail_state = 'complete_failure'
    complete_fsm = smach.StateMachine(outcomes=['complete_success', 'complete_failure'])
    complete_fsm.userdata.nominal_mixing_bowl_x = 0.40
    complete_fsm.userdata.nominal_mixing_bowl_y = -0.25
    complete_fsm.userdata.pre_mix_height_above_table = .6
    complete_fsm.userdata.table_frame_x_offset = 0
    complete_fsm.userdata.table_frame_y_offset = 0
    complete_fsm.userdata.acceptable_height_above_table = .5
    complete_fsm.userdata.pour_height_above_centroid = .4
    complete_fsm.userdata.mixing_laps = 3

    with complete_fsm:

        smach.StateMachine.add('ROBOT INIT', RobotInitializeState(),
                                transitions={'success':'USER INIT',
                                             'fail':fail_state})
        
        smach.StateMachine.add('USER INIT', UserInitializeState(),
                                transitions={'scan':'SCAN TABLETOP',
                                             'recurse':'USER INIT',
                                             'load':'LOAD FROM FILE'} ,
                                remapping={'save_detected_objects_filename':'save_detected_objects_filename',
                                           'load_detected_objects_filename':'load_detected_objects_filename'})

        smach.StateMachine.add('LOAD FROM FILE', LoadObjectsFromFile(),
                                transitions={'success':'FILTER INGREDIENTS',
                                             'fail':'LOAD FROM FILE'},
                                remapping={'load_detected_objects_filename':'load_detected_objects_filename'})


        smach.StateMachine.add('FILTER INGREDIENTS', FilterIngredients(),
                                transitions={'done':'BRANCH',
                                             'not enough detected objects':fail_state},
                                remapping={'ingredients_list':'ingredients_list',
                                           'cookie_sheet':'cookie_sheet',
                                           'mixing_bowl':'mixing_bowl'})

        smach.StateMachine.add('BRANCH', Branch(),
                                transitions={'scan':'SCAN TABLETOP',
                                             'deal':'DEAL MASTER',
                                             'skip to dealing':'DEAL MASTER',
                                             'skip to mixing':'MIX',
                                             'skip to scraping':'SCRAPE AND POUR',
                                             'skip to ovening':'OVENING'} ,
                                remapping={'save_detected_objects_filename':'save_detected_objects_filename',
                                           'load_detected_objects_filename':'load_detected_objects_filename'})

        smach.StateMachine.add('SCAN TABLETOP', ScanTabletop(),
                                transitions={'success':'FILTER INGREDIENTS',
                                             'fail':fail_state},
                                remapping={'save_detected_objects_filename':'save_detected_objects_filename'})


        smach.StateMachine.add('DEAL MASTER', DealMaster(),
                                transitions={'done':'SCRAPE AND POUR',
                                             'mix':'MIX',
                                             'deal ingredient':'DEAL BOWL'},
                                remapping={'ingredients_list':'ingredients_list',
                                           'bowl_to_deal':'bowl_to_deal'})
            
        bowl_dealer_fsm = assemble_bowl_dealer_fsm(DEBUG = False)  # TODO: need to pass the right arguments
        smach.StateMachine.add('DEAL BOWL', bowl_dealer_fsm,
                                transitions={'bowl_dealing_success': 'DEAL MASTER',
                                             'bowl_dealing_failure': fail_state},
                                remapping={'bowl_to_deal':'bowl_to_deal',
                                           'arms_to_try':'arms_to_try',
                                           'acceptable_height_above_table':'acceptable_height_above_table',
                                           'table_height':'table_height',
                                           'mixing_bowl':'mixing_bowl',
                                           'mixing_area_pose_x_tol':'mixing_area_pose_x_tol',
                                           'mixing_area_pose_y_tol':'mixing_area_pose_y_tol',
                                           'mixing_area_pose_z_tol':'mixing_area_pose_z_tol',
                                           'pour_height_above_centroid':'pour_height_above_centroid',
                                           'ingred_bowl_radius':'ingred_bowl_radius',
                                           'table_frame_x_offset':'table_frame_x_offset',
                                           'table_frame_y_offset':'table_frame_y_offset',
                                           'delta_x':'delta_x',
                                           'max_right_movement':'max_right_movement',
                                           'max_left_movement':'max_left_movement'})  # TODO: complete mapping

        mixing_fsm = assemble_mixing_fsm()
        smach.StateMachine.add('MIX', mixing_fsm,
                                transitions={'mixing_success': 'DEAL MASTER',
                                             'mixing_failure': fail_state},
                                remapping={'nominal_mixing_bowl_x':'nominal_mixing_bowl_x',
                                           'nominal_mixing_bowl_y':'nominal_mixing_bowl_y',
                                           'table_frame_x_offset':'table_frame_x_offset',
                                           'table_frame_y_offset':'table_frame_y_offset',
                                           'mixing_bowl':'mixing_bowl',
                                           'mixing_laps':'mixing_laps',
                                           'pre_mix_height_above_table':'pre_mix_height_above_table'})

        scraping_fsm = assemble_scraping_fsm()
        smach.StateMachine.add('SCRAPE AND POUR', scraping_fsm,
                                transitions={'scraping_success': 'OVENING',
                                             'scraping_failure': fail_state},
                                remapping={'mixing_bowl':'mixing_bowl',
                                           'acceptable_height_above_table':'acceptable_height_above_table',
                                           'table_height':'table_height',
                                           'cookie_sheet':'cookie_sheet',
                                           'arms_to_try':'arms_to_try',
                                           'table_frame_x_offset':'table_frame_x_offset',
                                           'pour_height_above_centroid':'pour_height_above_centroid',
                                           'pre_mix_height_above_table':'pre_mix_height_above_table',
                                           'table_frame_y_offset':'table_frame_y_offset'})
        
        ovening_fsm = assemble_ovening_fsm()
        smach.StateMachine.add('OVENING', ovening_fsm,
                                transitions={'ovening_success': 'complete_success',
                                             'ovening_failure': fail_state},
                                remapping={'table_height':'table_height',
                                           'cookie_sheet':'cookie_sheet',
                                           'acceptable_height_above_table':'acceptable_height_above_table',
                                           'table_frame_x_offset':'table_frame_x_offset',
                                           'table_frame_y_offset':'table_frame_y_offset'})
    #end with container
    return complete_fsm




    
def main():
    try:
        time.sleep(2)
        logger = EventLoggerClient.startlogger()
        sm = assemble_bakebot_fsm()
        sis = smach_ros.IntrospectionServer('introspection_server', sm, '/BIGFSM')
        sis.start()
        outcome = sm.execute()
        EventLoggerClient.stoplogger('success')
    except Exception as e:
        print e
    finally:
        EventLoggerClient.stoplogger('failure')
        sis.stop()

if __name__ == '__main__':
    rospy.init_node('bakebot_fsm')
    main()
