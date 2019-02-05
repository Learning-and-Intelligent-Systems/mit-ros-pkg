#!/usr/bin/env python

import roslib; 
roslib.load_manifest('imp_placing')
import rospy
import kinematics_msgs
import sys
import random
import time

from object import *
from modes import *
from degree_of_freedom import *

import numpy as np
from pr2_python import arm_planner, world_interface, arm_mover, gripper
import pr2_python.transform_listener as tf
import pr2_python.geometry_tools as gt
from arm_navigation_msgs.msg import Shape, CollisionObject, AttachedCollisionObject, OrderedCollisionOperations, CollisionOperation
#from physics import *
from pr2_tasks.pickplace import PickPlace
from pr2_tasks.pickplace_definitions import PlaceGoal, ObjectInfo, FrameFreeGrasp
from geometry_msgs.msg import PoseStamped, Point, Pose
from pr2_python.exceptions import ManipulationError
from visualization_msgs.msg import Marker
from pr2_python.exceptions import ArmNavError

from household_objects_database_msgs.srv import GetModelMesh  

def make_marker(pose, namespace="my_namespace", mtype=Marker.SPHERE, \
                        scale=(0.05,0.05,0.05), color=(1.0,0.0,1.0,0.8)):
    '''
    Publishes a marker in rviz.
    @type pose: PoseStamped
    @param pose: pose of the marker
    @type namespace: String
    @param namespace: name of the marker in rviz
    @type mtype: int
    @param mtype: type of marker
    @type scale: array
    @param scale: (x,y,z) size of the marker
    @type color: array
    @param color: [r,g,b,a] color of the marker
    '''
    pub = rospy.Publisher('robust_placing', Marker)
    
    count = 0
    while count < 10:
        marker = Marker()
        marker.header.frame_id = pose.header.frame_id #'/torso_lift_link'
        marker.ns = namespace
        marker.id = 0
        marker.type = mtype
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        pub.publish(marker)
        rospy.sleep(0.1)
        count = count+1

def get_table_height(my_world):
    '''
    returns the height of the table in '/torso_lift_link' frame
    
    @rtype float
    @returns table height
    '''
    height = 0.0
    objs = my_world.collision_objects()
    for obj in objs:
        #print "obj in tooals get_table_height="+str(obj.id)
        if obj.id == "table":
            table_pose = tf.transform_pose("/torso_lift_link", \
                                               obj.header.frame_id, \
                                               obj.poses[0])
            height = table_pose.position.z#+0.01
    return height

def get_obj(my_world, name):
    '''
    Returns the object with the given id if it is attached in the collision map
    
    @type name: String
    @param name: name of the object in the collision map
    
    @rtype arm_navigation_msgs.msg.attached_collision_object
    @returns object if one exists in the collision map
    '''
    objs = my_world.attached_collision_objects()
    for obj in objs:
        if obj.object.id == name:
            return obj
    rospy.loginfo("ERROR: OBJECT NAME DOES NOT EXIST")
    return 

def get_shape(obj):
    '''
    for a given object, returns its shape
    
    @type obj: arm_navigation_msgs.msg.attached_collision_object
    @param obj: detected object 
    
    @rtype arm_navigation_msgs.msg.shape
    @returns the first (should be only 1) shape in an obj
    '''
    if len(obj.object.shapes) != 1:
        rospy.loginfo("ERROR: TOO MANY SHAPES: %s",str(len(obj.object.shapes)))
    return obj.object.shapes[0]


def get_gripper(arm, obj, goal):
    '''
    finds the pose of the gripper for an object in its hand at the given goal pose
    
    @type obj: arm_navigation_msgs.msg.attached_collision_object
    @param obj: the object being held by the gripper
    @type goal: PoseStamped
    @param goal: goal pose for the object in torso_lift_link
    
    @rtype PoseStamped
    @returns pose of the gripper for the given object at the given goal 
    '''
#    print 'Getting the gripper.  Oh yeah'

    # first get the object's current pose
    obj_pose = tf.transform_pose("/torso_lift_link", obj.object.header.frame_id, \
                                     obj.object.poses[0])
    obj_pose_stamped = PoseStamped()
    obj_pose_stamped.header.frame_id = "/torso_lift_link"
    obj_pose_stamped.pose = obj_pose
#        self.make_marker(obj_pose_stamped, namespace='obj_pose', color=(1.0,1.0,1.0,0.8))

    # next get the gripper's current pose
    gripper_pose_stamped = arm.get_hand_frame_pose(\
        frame_id="/torso_lift_link") 
    gripper_pose = gripper_pose_stamped.pose
#        self.make_marker(obj_pose_stamped, namespace='gripper_pose', color=(1.0,0.0,0.0,0.8), mtype=Marker.ARROW)  

    # the position of the object when the wrist is the origin
    # grasp = gripper_pose^{-1}*obj_pose
    grasp = gt.inverse_transform_pose(obj_pose, gripper_pose)
    
    #we need the position of the gripper when the object is at goal
    #gripper_goal = goal*grasp^{-1}

    origin = Pose()
    origin.orientation.w = 1
    
    #this is grasp^{-1}
    grasp_inv = gt.inverse_transform_pose(origin, grasp)

    #this is goal*grasp_inv
    gripper_goal = gt.transform_pose(grasp_inv, goal.pose)
    gripper_ps = PoseStamped()
    gripper_ps.header.frame_id = goal.header.frame_id
    gripper_ps.pose = gripper_goal
    
    return gripper_ps

def make_pose(goal, h, orientation):
    '''
    @type pose: PoseStamped
    @param pose: original pose
    @type h: float
    @param h: additional height added in the z direction
    @type orientation: list
    @param orientation: orientation to be added to pose
    
    @rtype PoseStamped
    @returns a pose modified by h and orientation
    '''
    pose = PoseStamped()
    pose.header.frame_id = goal.header.frame_id
    pose.pose.position.x = goal.pose.position.x
    pose.pose.position.y = goal.pose.position.y
    pose.pose.position.z = goal.pose.position.z + h
    pose.pose.orientation = gt.multiply_quaternions(goal.pose.orientation, orientation)
    
    return pose

def make_gripper_obs():
    '''
    Creates a box obstacle from the dimensions of the gripper.
    '''
    obs = Shape()
    obs.type = 1
    obs.dimensions =[0.10,0.10,0.10]
    return obs

def get_collisions(my_world):
    '''
    creates list of collisions to be ignored.
    @rtype OrderedCollisionOperations
    @returns list of collisions to be ignored
    '''
    ignore = OrderedCollisionOperations()

    gripper = CollisionOperation()
    gripper.object1 = "l_end_effector"
    gripper.object2 = CollisionOperation.COLLISION_SET_ALL
    gripper.operation = CollisionOperation.DISABLE

    wrist = CollisionOperation()
    wrist.object1 = "l_wrist_roll_link"
    wrist.object2 = CollisionOperation.COLLISION_SET_ALL
    wrist.operation = CollisionOperation.DISABLE

    obj = CollisionOperation()
    obj.object1 = CollisionOperation.COLLISION_SET_ATTACHED_OBJECTS
    obj.object2 = "table"
    obj.operation = CollisionOperation.DISABLE

    map_obj = CollisionOperation()
    map_obj.object1 = CollisionOperation.COLLISION_SET_ATTACHED_OBJECTS
    map_obj.object2 = "collision_map"
    map_obj.operation = CollisionOperation.DISABLE

    lfing = CollisionOperation()
    lfing.object1 = "r_gripper_r_finger_tip_link"
    lfing.object2 = "table"
    lfing.operation = CollisionOperation.DISABLE

    rfingtip = CollisionOperation()
    rfingtip.object1 = "r_gripper_l_finger_tip_link" 
    rfingtip.object2 = "table"
    rfingtip.operation = CollisionOperation.DISABLE

    lfingmap = CollisionOperation()
    lfingmap.object1 = "r_gripper_r_finger_tip_link"
    lfingmap.object2 = CollisionOperation.COLLISION_SET_ALL
    lfingmap.operation = CollisionOperation.DISABLE

    rfingtipmap = CollisionOperation()
    rfingtipmap.object1 = "r_gripper_l_finger_tip_link" 
    rfingtipmap.object2 = CollisionOperation.COLLISION_SET_ALL
    rfingtipmap.operation = CollisionOperation.DISABLE

    rfing = CollisionOperation()
    rfing.object1 = "r_gripper_l_finger_link" 
    rfing.object2 = CollisionOperation.COLLISION_SET_ALL#"table"
    rfing.operation = CollisionOperation.DISABLE

    rfing1 = CollisionOperation()
    rfing1.object1 = "r_gripper_r_finger_link" 
    rfing1.object2 = CollisionOperation.COLLISION_SET_ALL#"table"
    rfing1.operation = CollisionOperation.DISABLE

    forearm = CollisionOperation()
    forearm.object1 = CollisionOperation.COLLISION_SET_ATTACHED_OBJECTS
    forearm.object2 = "l_forearm_link"
    forearm.operation = CollisionOperation.DISABLE

    flex = CollisionOperation()
    flex.object1 = CollisionOperation.COLLISION_SET_ATTACHED_OBJECTS
    flex.object2 = "l_wrist_flex_link"
    flex.operation = CollisionOperation.DISABLE
    
    ops = []
    op = CollisionOperation()
    op.operation = op.DISABLE
    op.object1 = op.COLLISION_SET_ATTACHED_OBJECTS
    for l in my_world.hands['right_arm'].hand_links:
        op.object2 = l
        ops.append(copy.deepcopy(op))
        
    ignore.collision_operations = [gripper,wrist,obj, map_obj, lfing, rfing, rfingtip, lfingmap, rfingtipmap, forearm, rfing1, flex]+ops
    return ignore


def reachable(arm, pose, collisions):
    '''
    determines whether the a pose is has an ik solution
    
    @type pose:
    @param pose:
    
    @rtype boolean
    @returns whether the pose is valid
    '''             
    ik_sol = arm.get_ik(pose, ordered_collisions=collisions)
    #rospy.loginfo("ERROR CODE=%s"+str(ik_sol.error_code.val))
    if ik_sol.error_code.val != 1:
        return False
    else:
        return True

def move(my_arm_mover, arm, pose, collisions, try_hard):
    '''
    @type pose: PoseStamped
    @param pose: desired pose for the arm 
    
    @rtype boolean
    @returns whether the move was successful
    '''
    arm_name = arm #'left_arm'
    handle = my_arm_mover.move_to_goal(arm_name, pose, \
                                                ordered_collisions=collisions, \
                                                try_hard=try_hard)
    rospy.loginfo("MOVE SUCCESS=%s",str(handle.reached_goal()))
    return handle.reached_goal()

def place(pickplace, arm, obj_name, pose):    
    '''
    places the given object with the given arm in the give pose.
    
    @type arm: String
    @param arm: 'left_arm' or 'right_arm'. arm to place
    @type obj_name: String
    @param obj_name: name of the object being placed from the collision map
    @type pose: PoseStamped
    @param pose: pose to place the object
        
    @rtype: boolean
    @returns whether the place was successful
    '''
    try:       
        if arm=="right_arm":
            place_goal = PlaceGoal('right_arm', [pose], \
                                       collision_support_surface_name='all', \
                                       collision_object_name=obj_name)
        else:
            place_goal = PlaceGoal('left_arm', [pose], \
                                       collision_support_surface_name='table', \
                                       collision_object_name=obj_name)
        place_result = pickplace.place(place_goal)
        rospy.loginfo("Place result=%s",str(place_result))
    except ManipulationError:
        rospy.loginfo("POSE FAILED")
        return False
    return True
    
