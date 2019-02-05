#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
from object_manipulation_msgs.msg import ManipulationResult, GraspableObject, Grasp
from geometry_msgs.msg import Pose
from clients.arm_client import *
from dynamicfsm.ingredient_filter import *
import rospy
import tf
import math
import pickle
import sys
from services.bakebot_logging import *

class BroadTabletopObjectManager():
    
    btom_instance = None 

    def __init__(self, pick_and_place_manager):
        if pick_and_place_manager == None:
            raise Exception('need to have a pick and place manager!')
        self.pick_and_place_manager = pick_and_place_manager
        self.detected_objects = list()
        # these below are just from the filter
        self.detected_ingredients = None
        self.mixing_bowl = None
        self.cookie_sheet = None
        self.filtered = False

    @staticmethod
    def get_broad_tabletop_object_manager(pick_and_place_manager):
        if BroadTabletopObjectManager.btom_instance == None:
            rospy.loginfo('instantiating a new BroadTabletopObjectManager')
            BroadTabletopObjectManager.btom_instance = BroadTabletopObjectManager(pick_and_place_manager)
        else:
            rospy.loginfo('returning existing instance of BroadTabletopObjectManager')
        return BroadTabletopObjectManager.btom_instance

# these shouldn't really be here...
    def filter_ingredients(self, filter_cs = True, filter_mb = True, ingredient_filter = None):
       #if self.filtered:
       #    print 'already filtered, skipping'
       #    return
        print 'filtering ingredients in btom...'
        detected_objects = self.detected_objects
        if len(detected_objects) < 2:
            rospy.logerr('less than two ingredients detected')
        detected_ingredients = list()
        # first sort the list by y position, take the two lowest out.  they are the cookie sheet and mixing bowl
        sorted_by_y = sorted(detected_objects, key = lambda obj: obj.pose.pose.position.y)
        rightmost = [sorted_by_y[0], sorted_by_y[1]]
        # now sort those two by x
        sorted_by_x = sorted(rightmost, key = lambda obj: obj.pose.pose.position.x)
        if filter_mb:
            self.mixing_bowl = sorted_by_x[0]
            sorted_by_y.remove(self.mixing_bowl)
        if filter_cs:
            self.cookie_sheet = sorted_by_x[1]
            sorted_by_y.remove(self.cookie_sheet)
        if ingredient_filter == None:
            # do the regular clockwise bowl/ingredient filtering
            self.detected_ingredients = sorted_by_y
            self.ingredient_filter = ClockwiseIngredientFilter()
            self.ingredient_filter.filter_ingredients(sorted_by_y)
        else:
            self.detected_ingredients = sorted_by_y
            self.ingredient_filter = ingredient_filter
            self.ingredient_filter.filter_ingredients(sorted_by_y)
        self.filtered = True

    def get_ingredient(self, name):
        return self.ingredient_filter.get_ingredient(name)

    def get_ingredient_iterator(self):
        return self.ingredient_filter.get_ingredient_iterator()
# end complete-fucking-hack

    def do_broad_tabletop_object_detection(self, refine = True):
        # returns a list of detected objects on the table.
        # this list is filtered to eliminate duplicates
        # the COLLISION NAMES ARE FAKE.  these object names cannot be passed to the grasp program
        # you have to a refined detection of the object before trying to grasp
        logger = EventLoggerClient('broad ttop detection')
        self.pick_and_place_manager.find_table()
        if self.pick_and_place_manager.table_front_edge_x > 1.0:
            rospy.logwarn('table front edge is way too far away: ' + str(self.pick_and_place_manager.table_front_edge_x))
            self.pick_and_place_manager.table_front_edge_x = 1
            rospy.logwarn('table front edge corrected: ' + str(self.pick_and_place_manager.table_front_edge_x))
        all_detected_objects = list()
        point_head_locs = list()
        #y_positions_cm = range(-40,41,20)
        #x_positions_cm = [15, 25]
        y_positions_cm = range(-10,51,30) #THESE ARE GOOD
        x_positions_cm = [5, 20]
        rospy.loginfo('investigating these y positions: '+str(y_positions_cm))
        rospy.loginfo('investigating these x positions: '+str(x_positions_cm))
        oindex = 0
        broadlogger = EventLoggerClient('doing the wide detection sweep')
        for x_pos_cm in x_positions_cm:
            for y_pos_cm in y_positions_cm:
                detectionlogger = EventLoggerClient('single ttop detection call')
                pose_to_point_at = [self.pick_and_place_manager.table_front_edge_x + x_pos_cm/100.0, y_pos_cm/100.0, self.pick_and_place_manager.table_height]
                rospy.loginfo('pointing the head at pose: ' + str(pose_to_point_at))
                self.pick_and_place_manager.point_head(pose_to_point_at, 'base_link')
                (detected_objects, table) = self.pick_and_place_manager.call_tabletop_detection(take_static_collision_map = 1, update_table = 1, clear_attached_objects = 1, replace_table_in_collision_map = 1) #NOTE: must have replace_table_in_collision_map = 1 or it will crash the planner!
                detectionlogger.stops('detecteded ' + str(len(detected_objects)) + ' objects')
                for obj in detected_objects:
                    obj.collision_name = 'fake_collision_name_'+str(oindex) + obj.collision_name
                    oindex = oindex + 1
                rospy.loginfo('this pass detected ' + str(len(detected_objects)) + ' objects')
                self.add_detected_objects_if_new(all_detected_objects, detected_objects)
                self.pick_and_place_manager.detected_objects = all_detected_objects
                self.pick_and_place_manager.print_object_list()
        broadlogger.stops()
        if refine:
            refinelogger = EventLoggerClient('refining objecte detection')
            rospy.loginfo('************ REFINING detected objects')
            refined_obj = list()
            for obj in all_detected_objects:
                if obj.pose.pose.position.x > 1.5:
                    print 'its too far away: ', obj.pose.pose.position.x
                    continue
                else:
                    refined = self.pick_and_place_manager.refine_object_detection(obj.pose.pose.position.x, obj.pose.pose.position.y)
                if refined == None:
                    rospy.logwarn('turns out nothing was there at ' + str(obj.pose.pose.position.x) + ' ' + str(obj.pose.pose.position.y))
                else:
                    refined.collision_name = obj.collision_name
                    refined_obj.append(refined)
            all_detected_objects = refined_obj
            refinelogger.stops()
        self.detected_objects = all_detected_objects
        logger.stops('total detected ' + str(len(all_detected_objects)) + ' discrete objects')
        i = 0
        for obj in all_detected_objects:
            print '\tobj', i, 'xy:', obj.pose.pose.position.x, obj.pose.pose.position.y, obj.collision_name
            i = i + 1
        return all_detected_objects
    
    def add_detected_objects_if_new(self, all_detected_objects, new_detected_objects, minimum_dist = 0.20):
        # modifies all detected objects list
        rospy.loginfo('checking ' + str(len(new_detected_objects)) + ' objects to see if they are already found')
        rospy.loginfo('I already found ' + str(len(all_detected_objects)) + ' objects')
        add_these_objects = list()
        for new in new_detected_objects:
            already_in_list = False
            remove_these_objects = list()
            for old in all_detected_objects:
                nx = new.pose.pose.position.x
                ny = new.pose.pose.position.y
                ox = old.pose.pose.position.x
                oy = old.pose.pose.position.y
                name = new.collision_name 
                dist = math.sqrt((nx - ox)**2 + (ny - oy)**2)
                rospy.loginfo('the distance between this object and its neighbor is: '+str(dist))
                if  dist < minimum_dist:
                    rospy.loginfo('not yet adding object ' + str(new.collision_name) + '(' + str(dist) + 'm deviation)')
                    rospy.loginfo('type: ' + str(new.type))
                    if (old.type == 'cluster'):
                        if new.type != 'cluster':
                            rospy.loginfo('the new object is a database model in the same spot as the point cluster')
                            rospy.loginfo('going to replace the point cluster with the database mdoel')
                            remove_these_objects.append(old)
                        elif self.box_vol(old.box_dims) < self.box_vol(new.box_dims):
                            rospy.loginfo('the new object has a bigger box: ' + str(self.box_vol(new.box_dims)))
                            rospy.loginfo('than the old object: '+ str(self.box_vol(old.box_dims)))
                            remove_these_objects.append(old)
                        else:
                            already_in_list = True
                    else:
                        already_in_list = True
            for obj in remove_these_objects:
                all_detected_objects.remove(obj)
            if not already_in_list:
                rospy.loginfo('adding object ' + str(new.collision_name))
                rospy.loginfo('type: ' + str(new.type))
                add_these_objects.append(new)
        for new in add_these_objects:
            rospy.loginfo('adding: ' + str(new))
            if new != None:
                all_detected_objects.append(new)
            else:
                rospy.logwarn('a NoneType object got into the list!')

    def box_vol(self, box_dims):
        return box_dims[0] * box_dims[1] * box_dims[2]

    def refine_and_and_get_graspable_object(self, name, table_frame_x_offset = 0.0, table_frame_y_offset = 0.0):
        # refines the detection (resetting the collision name space) and returns the object
        logger = EventLoggerClient('refine and get graspable')
        object_to_grab = None
        for obj in self.detected_objects:
            if obj.collision_name == name:
                rospy.loginfo('found an object in detected object list with same name: '+str(name))
                object_to_grab = obj
        if object_to_grab == None:
            rospy.logwarn('could not find an object in detected object list with name: '+str(name))
            return None

        rospy.loginfo('clearing collision map')
        self.pick_and_place_manager.reset_collision_map();
        rospy.loginfo('refining the object detection')
        print 'origx', object_to_grab.pose.pose.position.x
        print 'origy', object_to_grab.pose.pose.position.y
        print 'origz', object_to_grab.pose.pose.position.z
        print 'tableframexoff', table_frame_x_offset
        print 'tableframeyoff', table_frame_y_offset
        print 'corrx', (object_to_grab.pose.pose.position.x+table_frame_x_offset)
        print 'corry', (object_to_grab.pose.pose.position.y+table_frame_y_offset)

        refined = self.pick_and_place_manager.refine_object_detection(object_to_grab.pose.pose.position.x + table_frame_x_offset, object_to_grab.pose.pose.position.y + table_frame_y_offset) # TODO: these might need to be -'s
        if refined is not None:
            logger.stops()
            print 'newx', refined.pose.pose.position.x
            print 'newy', refined.pose.pose.position.y
            print 'newz', refined.pose.pose.position.z
        else:
            logger.stopf()
        return refined
    
    def refine_and_grasp(self, name, arms_to_try, table_frame_x_offset = 0.0, table_frame_y_offset = 0.0, desired_grasp_pose = None):
        # table_frame_y_offset is positive if the robot moved to its right (meaning the table y position is greater in the robot's base frame)
        object_to_grab = self.refine_and_and_get_graspable_object(name, table_frame_x_offset, table_frame_y_offset)
        if object_to_grab == None:
            rospy.logwarn('no object with name was found in broad detection list: ' + str(name))
            return (0, -1, None)
        for whicharm in arms_to_try:
            self.pick_and_place_manager.check_preempted()
            if desired_grasp_pose is not None:
                desired_grasp = Grasp()
                desired_grasp.grasp_pose = desired_grasp_pose
                desired_grasps = [desired_grasp]
            else:
                desired_grasps = None
            grasplogger = EventLoggerClient('grasp')
            (result, arm_used, grasp_pose) = self.pick_and_place_manager.grasp_object_and_check_success(object_to_grab, whicharm, desired_grasps)
            if result == 'succeeded':
                grasplogger.stops()
                rospy.loginfo('BTO: grasp success.')
                return (whicharm, result, grasp_pose)
            grasplogger.stopf()
        rospy.logwarn('BTO: grasp failed')
        return (whicharm, result, grasp_pose)

    def save_detected_objects_to_file(self, filename):
        try:
            myfile = open(filename, 'wb')
            pickle.dump((self.detected_objects, self.pick_and_place_manager.table_height), myfile)
            myfile.close()
            rospy.loginfo('file written to: ' + str(filename))
            return True
        except Exception as e:
            print e
            rospy.logerr('somethign went wrong saving the file')
            return False


    def load_detected_objects_from_file(self, filename):
        try:
            myfile = open(filename, 'rb')
            (self.detected_objects, self.pick_and_place_manager.table_height) = pickle.load(myfile)
            rospy.loginfo('saved objects loaded from ' + str(filename))
            for obj in self.detected_objects:
                self.pick_and_place_manager.detected_objects = self.detected_objects
            return True
        except Exception as e:
            print e
            rospy.logerr('something went wrong loading the file')
            return False
