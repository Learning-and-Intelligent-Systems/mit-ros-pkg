#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
import rospy
import tf
import actionlib
from clients.unified_tf_client import UnifiedTFClient
from clients.arm_client import ArmClient
from utilities.bakebot_pick_and_place_manager import *
from utilities.broad_table_object_manager import *
import math

class GripperBowlPoseManager():
    ALL_POSES = {
        'init': (-98, -98, -98, -98),
        'six_o_clock_bi': (-.5, .5, .5, .5),
        'six_o_clock_bo': (-.5, -.5, .5, -.5),
        'nine_o_clock_bi': (0, .7, 0, .7),
        'nine_o_clock_bo': (.7, 0, -.7, 0),
        'twelve_o_clock_bi': (-.5, -.5, .5, -.5),
        'twelve_o_clock_bo': (-.5, .5, .5, .5),
        'three_o_clock_bi': (.7, 0, -.7, 0),
        'three_o_clock_bo': (0, .7, 0, .7),
        'unknown': (-99, -99, -99, -99)}

    POUR_ADJUSTMENT_DIRS_FROM_HIGH_CENTROID = {
        'init': (0, 0, 0),
        'six_o_clock_bi': (1, 0, 0),
        'six_o_clock_bo': (1, 0, 0),
        'twelve_o_clock_bi': (-1, 0, 0),
        'twelve_o_clock_bo': (-1, 0, 0),
        'nine_o_clock_bi': (0, -1, 0),
        'nine_o_clock_bo': (0, -1, 0),
        'three_o_clock_bi': (0, 1, 0),
        'three_o_clock_bo': (0, 1, 0),
        'unknown': (0,0,0)}

    POUR_POSES =  {
        'init': (-98, -98, -98, -98),
        'six_o_clock_bo': (.16, .7, -.7, .2),
        'six_o_clock_bi': (-.10, .7, .7, .2),
        'twelve_o_clock_bo': (-.7, -.2, -.1, .65),
        'twelve_o_clock_bi': (.7, .1, -.1, .7),
        'three_o_clock_bo': (-.4, .6, -.4, .6),
        'three_o_clock_bi': (.6, -.5, -.5, .43),
        'nine_o_clock_bo': (-.6, -.4, .6, .4),
        'nine_o_clock_bi': (.4, .6, .4, .6),
        'unknown': (-99, -99, -99, -99)}

    ##STEEP_POUR_POSES =  {
        #'init': (-98, -98, -98, -98),
        #'six_o_clock_bo': (0.167, 0.703, -0.654, 0.22),
        #'six_o_clock_bi': (-0.083, 0.669, 0.71, 0.18),
        #'twelve_o_clock_bo': (-.75, .05, .07, .65),
        #'twelve_o_clock_bi': (0.66, 0.1, -0.1, 0.735),
        #'three_o_clock_bo': (-0.409, 0.58, -0.398, 0.577),
        #'three_o_clock_bi': (0.5585, -0.467, -0.527, 0.437),
        #'nine_o_clock_bo': (-0.574, -0.388, 0.623, 0.36),
        #'nine_o_clock_bi': (0.405, 0.585, 0.34, 0.61),
        #'unknown': (-99, -99, -99, -99)}
    STEEP_POUR_POSES =  {
        'init': (-98, -98, -98, -98),
        'six_o_clock_bo': (.06, -.65, .675, .344),
        'six_o_clock_bi': (.34, .62, .70, -.089),
        'twelve_o_clock_bo': (-.239, -.35, .28, .85),
        'twelve_o_clock_bi': (.845, .33, .25, .33),
        'three_o_clock_bo': (-.68, .32, -.30, .60),
        'three_o_clock_bi': (.45, -.55, -.42, .6),
        'nine_o_clock_bo': (-.41, -.55, .25, .68),
        'nine_o_clock_bi': (.5, .5, .5, .50),
        'unknown': (-99, -99, -99, -99)}

    SUPER_STEEP_POUR_POSES =  {
        'init': (-98, -98, -98, -98),
        'six_o_clock_bo': (-.2, .7, -.7, .2),
        'six_o_clock_bi': (.16, .7, .7, -.25),
        'twelve_o_clock_bo': (-.6, -.3, -.3, .66),
        'twelve_o_clock_bi': (.6, -.4, .4, .6),
        'three_o_clock_bo': (.6, -.35, .6, -.35),
        'three_o_clock_bi': (.3, -.6, -.3, .65),
        'nine_o_clock_bo': (-.26, -.7, .15, .67),
        'nine_o_clock_bi': (.6, .3, .65, .35),
        'unknown': (-99, -99, -99, -99)}

    MIXING_BOWL_STEEP_POUR_POSES =  {  
        'init': (-98, -98, -98, -98),
        'nine_o_clock_bo': (-.45, -.51, .47, .46),
        'nine_o_clock_bi': (.60, .35, .55, .46),
        'unknown': (-99, -99, -99, -99)}

    object_name_to_barcode_in= dict()
    gbpm_instance = None

    @staticmethod
    def get_gripper_bowl_pose_manager(object_info, arm_client, isRightArm, 
                                        is_pre_scrape=False, is_pre_oven=False):
        if GripperBowlPoseManager.gbpm_instance == None:
            rospy.loginfo('instantiating a new gbpm')
            GripperBowlPoseManager.gbpm_instance = GripperBowlPoseManager(object_info, arm_client, isRightArm, is_pre_scrape, is_pre_oven)
        return GripperBowlPoseManager.gbpm_instance

    @staticmethod
    def clear_static_gripper_bowl_pose_manager():
        GripperBowlPoseManager.gbpm_instance = None

    def __init__(self, object_info, arm_client, isRightArm, is_pre_scrape = False, is_pre_oven = False):
        self.original_pose = arm_client.get_transform(isRightArm) 
        self.iterator = self.get_preferred_poses_iterator(object_info, is_pre_scrape = is_pre_scrape, is_pre_oven = is_pre_oven)
        self.current_preferred_pose = self.iterator.next()
        self.is_exhausted = False

    def get_preferred_poses_iterator(self, object_info, is_pre_scrape, is_pre_oven):
        if not is_pre_oven:
            self.barcode_in = self.is_barcode_in_bowl(object_info)
        else:
            self.barcode_in = True
        self.preferred_poses = list()
        bc = 'bi' if self.barcode_in else 'bo'
        if is_pre_scrape:
            rospy.loginfo('generating scraping pour pose iterator')
            self.preferred_poses.append('nine_o_clock_' + bc)
        elif is_pre_oven:
            rospy.loginfo('generating ovening pour pose iterator')
            self.preferred_poses.append('six_o_clock_' + bc)
        else:
            self.preferred_poses.append('three_o_clock_' + bc)
            self.preferred_poses.append('three_o_clock_' + bc)
            self.preferred_poses.append('three_o_clock_' + bc)
            self.preferred_poses.append('three_o_clock_' + bc)
            self.preferred_poses.append('six_o_clock_' + bc)
            self.preferred_poses.append('twelve_o_clock_' + bc)
            self.preferred_poses.append('twelve_o_clock_' + bc)
            self.preferred_poses.append('nine_o_clock_' + bc)
            self.preferred_poses.append('six_o_clock_' + bc)
            self.preferred_poses.append('three_o_clock_' + bc)
            self.preferred_poses.append('twelve_o_clock_' + bc)
            self.preferred_poses.append('nine_o_clock_' + bc)
            print 'preferred poses list: ', self.preferred_poses
        return iter(self.preferred_poses)

    def is_barcode_in_bowl(self, object_info):
        if object_info.id_num in GripperBowlPoseManager.object_name_to_barcode_in:
            rospy.loginfo('3333333333333333333333333333 I have one of these stored in my dict.' + str(object_info.id_num))
            print object_info.id_num
            barcode_in = GripperBowlPoseManager.object_name_to_barcode_in[object_info.id_num]
            print barcode_in
            return barcode_in
        else:
            in_bowl = (object_info.type == 'cluster')
            rospy.loginfo('\n\n******************** IN BOWL: ' + str(in_bowl) + '\n\n')
            in_check = raw_input('press enter to confirm that this is true, else enter any char...')
            if len(in_check) is not 0:
                rospy.loginfo('flipping the status of in_bowl...')
                in_bowl = not in_bowl
                rospy.loginfo('\n\n******************** IN BOWL: ' + str(in_bowl) + '\n\n')
            return in_bowl

    @staticmethod
    def store_barcode_pos(object_info, hand_link_name):
        tfc = UnifiedTFClient.get_unified_tf_client()
        print '3333333333333333333333 storing in bowl', object_info.id_num
        print object_info
        print 'hand link frame: ', hand_link_name
        datatype = tfc.transformPose(hand_link_name, object_info.pose)
        print 'datatype: ', datatype
        #raw_input('\n\n\n\npress enter to continue...')
        barcode_in = datatype.pose.position.y < 0
        print 'barcode in: ', barcode_in
        rospy.loginfo('\n\n******************** IN BOWL: ' + str(barcode_in) + '\n\n')
        in_check = raw_input('press enter to confirm that this is true, else enter any char...')
        if len(in_check) is not 0:
            rospy.loginfo('flipping the status of in_bowl...')
            barcode_in = not barcode_in
            rospy.loginfo('\n\n******************** IN BOWL: ' + str(barcode_in) + '\n\n')
        GripperBowlPoseManager.object_name_to_barcode_in[object_info.id_num] = barcode_in
        return barcode_in

    def get_current_recommended_pose(self):
        # want to return pose, name of pose
        return GripperBowlPoseManager.ALL_POSES[self.current_preferred_pose], self.current_preferred_pose

    def get_next_recommended_pose(self):
        rospy.loginfo('\n\ncurrent recommended pose: ' + str(self.get_current_recommended_pose()) + ', GETTING NEXT RECOMMENDED POSE \n\n')
        try:
            self.current_preferred_pose = self.iterator.next()
        except StopIteration:
            self.is_exhausted = True
            rospy.logwarn('GBPM: ran out of new positions to try, returning original pose')
            return self.original_pose[1], 'original_pose'
        return self.get_current_recommended_pose()

    def reset_exhaustion(self):
        self.is_exhausted = False
        self.iterator = iter(self.preferred_poses)
