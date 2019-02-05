#!/usr/bin/env python

import roslib; roslib.load_manifest('pr2_utils')
import rospy
import object_manipulation_msgs.msg
import actionlib
import arm_navigation_msgs.msg
import arm_navigation_msgs.srv
import geometry_msgs.msg
import utils
import arm_control
import actionlib_msgs.msg
import math

class PickPlace:
    def __init__(self, arm_name):
        self.arm_name = arm_name
        self.arm_control = arm_control.ArmControl(self.arm_name)
        self.pickup_client = actionlib.SimpleActionClient\
            ('/object_manipulator/object_manipulator_pickup',\
                 object_manipulation_msgs.msg.PickupAction)
        rospy.loginfo('Waiting for pickup action')
        self.pickup_client.wait_for_server()
        self.set_planning_scene_service = rospy.ServiceProxy\
            ('/environment_server/set_planning_scene_diff',\
                 arm_navigation_msgs.srv.SetPlanningSceneDiff)
        rospy.loginfo('Waiting for set planning scene service')
        self.set_planning_scene_service.wait_for_service()
        self.get_planning_scene_service = rospy.ServiceProxy\
            ('/environment_server/get_planning_scene',\
                 arm_navigation_msgs.srv.GetPlanningScene)
        rospy.loginfo('Waiting for get planning scene service')
        self.get_planning_scene_service.wait_for_service()
        self.collision_object_pub = rospy.Publisher\
            ('/collision_object', arm_navigation_msgs.msg.CollisionObject)
        self.attached_collision_pub = rospy.Publisher\
            ('/attached_collision_object',\
                 arm_navigation_msgs.msg.AttachedCollisionObject)
        rospy.loginfo('PickPlace created')

    def pickup_shape(self, shape, pose_stamped, 
                     pregrasp_pose, postgrasp_pose):
        '''
        shapes should be an arm_navigation_msgs/Shape
        '''
        coll_obj = arm_navigation_msgs.msg.CollisionObject()
        coll_obj.header.frame_id = pose_stamped.header.frame_id
        coll_obj.id = 'object_for_pickup'
        coll_obj.operation.operation =\
            coll_obj.operation.ADD
        coll_obj.shapes = [shape]
        coll_obj.poses = [pose_stamped.pose]
        self.collision_object_pub.publish(coll_obj)
        diff = arm_navigation_msgs.srv.GetPlanningSceneRequest()
        resp = self.get_planning_scene_service(diff)
        print 'collision objects are', resp.planning_scene.collision_objects
        grasp_pose = self.get_nearest_point(shape, pose_stamped)
        grasp_pose.pose.position.x -= 0.14
        grasp_pose.pose.position.y = pregrasp_pose.pose.position.y
        grasp_pose.pose.position.z = pregrasp_pose.pose.position.z
        grasp_pose.pose.orientation = pregrasp_pose.pose.orientation

        return self.pickup(pregrasp_pose, grasp_pose, postgrasp_pose,
                           name=coll_obj.id)


    def pickup(self, pregrasp_pose, grasp_pose, postgrasp_pose, name=None):
        #first move to the approach distance
        rospy.loginfo('pregrasp =\n'+str(pregrasp_pose)+'\ngrasp =\n'
                      +str(grasp_pose)+'\npostgrasp=\n'+str(postgrasp_pose))

        #self.arm_control.move_arm_to_side(revert_scene=False)
        mops = arm_navigation_msgs.msg.OrderedCollisionOperations()
        if name != None:
            op = arm_navigation_msgs.msg.CollisionOperation()
            op.object1 = op.COLLISION_SET_ALL
            op.object2 = name
            op.operation = op.DISABLE
            mops.collision_operations.append(op)
        result, state = self.arm_control.move_arm_collision_free\
            (pregrasp_pose, timeout=rospy.Duration(30.0), do_filter=False,\
                 revert_scene=False, ordered_colls=mops)
        rospy.loginfo('Moved to pregrasp')
        if result.error_code.val != result.error_code.SUCCESS or\
                (state != actionlib_msgs.msg.GoalStatus.SUCCEEDED):
            rospy.logerr('Move arm for pickup failed')
            return False
        #now open the gripper
        self.arm_control.open_gripper()
        #now approach using interpolated IK, ignoring all collisions
        coll = arm_navigation_msgs.msg.CollisionOperation()
        coll.object1 = coll.COLLISION_SET_ALL
        coll.object2 = coll.COLLISION_SET_ALL
        coll.operation = coll.DISABLE
        ops = arm_navigation_msgs.msg.OrderedCollisionOperations()
        ops.collision_operations.append(coll)
        error_code = self.arm_control.move_arm_interpolated_ik(grasp_pose,
                                                               ordered_colls=
                                                               ops,
                                                               revert_scene=
                                                               False)
        if error_code.val != error_code.SUCCESS:
            rospy.logerr('Grasp approach failed')
            return False
        rospy.loginfo('Grasped')
        self.arm_control.close_gripper()
        #add the thing to the gripper
        if name != None:
            diff = arm_navigation_msgs.srv.GetPlanningSceneRequest()
            resp = self.get_planning_scene_service(diff)
            print 'collision objects are', resp.planning_scene.collision_objects
            op = arm_navigation_msgs.msg.AttachedCollisionObject()
            op.link_name = self.arm_name[0]+'_gripper_r_finger_tip_link'
            op.touch_links = [self.arm_name[0]+'_gripper_r_finger_tip_link',
                              self.arm_name[0]+'_gripper_l_finger_tip_link',
                              self.arm_name[0]+'_gripper_r_finger_link',
                              self.arm_name[0]+'_gripper_l_finger_link',
                              self.arm_name[0]+'_gripper_palm_link']
            op.object.id = name
            op.object.operation.operation =\
                op.object.operation.ATTACH_AND_REMOVE_AS_OBJECT
            self.attached_collision_pub.publish(op)
            diff = arm_navigation_msgs.srv.GetPlanningSceneRequest()
            resp = self.get_planning_scene_service(diff)
            print 'collision objects are', resp.planning_scene.collision_objects
            # sdiff = arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()
            # sdiff.planning_scene_diff =\
            #     self.get_planning_scene_service(diff).planning_scene
            # sdiff.planning_scene_diff.attached_collision_objects.append(op)
            # self.set_planning_scene_service(sdiff)

        #retreat using interpolated IK, still ignoring all collisions
        error_code = self.arm_control.move_arm_interpolated_ik(postgrasp_pose,
                                                               ordered_colls=
                                                               ops,
                                                               revert_scene=
                                                               False)
        if error_code.val != error_code.SUCCESS:
            rospy.logerr('Grasp retreat failed')
            return False
        rospy.loginfo('Retreated')
        return True
        
    def pickup_named_object(self, name, grasps=[], lift_in_base_link=\
                                geometry_msgs.msg.Vector3(0,0,1)):
        goal = object_manipulation_msgs.msg.PickupGoal()
        goal.arm_name = self.arm_name
        goal.target.reference_frame_id = 'base_link'
        goal.lift.direction.header.frame_id = 'base_link'
        goal.lift.direction.vector = lift_in_base_link
        goal.lift.desired_distance = 0.2
        goal.lift.min_distance = 0.05
        goal.collision_object_name = name
        goal.collision_support_surface_name = ''
        goal.desired_grasps = grasps
        rospy.loginfo('Sending pickup goal')
        self.pickup_client.send_goal_and_wait(goal)
        
    def get_nearest_point(self, shape, pose_stamped):
        #right now we have an axis-oriented box
        #will fix this to be general late
        nearest_point = utils.copyPoseStamped(pose_stamped)
        if shape.type == shape.BOX:
            nearest_point.pose.position.x -= shape.dimensions[0]/2.0
        if shape.type == shape.CYLINDER:
            nearest_point.pose.position.x -= shape.dimensions[0]
        return nearest_point

    def reset_collision_map(self):
        #remove all objects
        self.arm_control.revert_planning_scene()
        reset_object = arm_navigation_msgs.msg.CollisionObject()
        reset_object.operation.operation =\
            arm_navigation_msgs.msg.CollisionObjectOperation.REMOVE
        reset_object.header.frame_id = "base_link"
        reset_object.header.stamp = rospy.Time.now()
        reset_object.id = "all"
        self.collision_object_pub.publish(reset_object)
        diff = arm_navigation_msgs.srv.GetPlanningSceneRequest()
        resp = self.get_planning_scene_service(diff)

        #and all attached objects
        reset_attached_objects =\
            arm_navigation_msgs.msg.AttachedCollisionObject()
        reset_attached_objects.link_name = "all"
        reset_attached_objects.object.header.frame_id = "base_link"
        reset_attached_objects.object.header.stamp = rospy.Time.now()
        reset_attached_objects.object = reset_object
        self.attached_collision_pub.publish(reset_attached_objects)
        diff = arm_navigation_msgs.srv.GetPlanningSceneRequest()
        resp = self.get_planning_scene_service(diff)
        print 'collision objects are', resp.planning_scene.collision_objects
