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
import degree_of_freedom as dof
import tools

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

class Placer:
    
    def __init__(self, goal, modes):
        '''
        @type goal: PoseStameped
        @param goal: goal pose of the object
        @type modes: list
        @param modes: list of priors for each of the modes [tipping, sliding]
        '''
        self.goal = goal
        self.modes = modes

        self.my_world = world_interface.WorldInterface()
        self.right_arm = arm_planner.ArmPlanner('right_arm')
        self.left_arm = arm_planner.ArmPlanner('left_arm')
        self.my_arm_mover = arm_mover.ArmMover()
        self.pickplace = PickPlace(search_place=[], place_on_table=False)
        self._get_model_mesh_srv = rospy.ServiceProxy('/objects_database_node/get_model_mesh',
                                                      GetModelMesh)
        rospy.loginfo('Waiting for model mesh service')
        self._get_model_mesh_srv.wait_for_service()

    def setup(self, obj_type, obj_name):
        '''
        @type obj_type: String 
        @param obj_type: type of object, "paper_plate","plastic_plate","box"
        @type obj_name: String
        @param obj_type: name of the object in the collision map, 
                         "graspable_object_****"

        @rtype list (arm_navigation_msgs.msg.attached_collision_object, 
                     imp_placing.Object, 
                     arm_navigation_msgs.msg.shape,
                     arm_navigation_msgs.msg.shape)
        @returns (the collision object in the map, the Object representation, 
                  the gripper as an obstacle, the object shape)
        '''
        # self.my_world.reset_collider_node() # takes a long time
#        print "len of objs in place"+str(len(self.my_world.collision_objects()))
        table_height = tools.get_table_height(self.my_world)
        print "TABLE_HEIGHT="+str(table_height)
        #self.goal.pose.position.z = table_height + 0.075 #+ 0.1#+ 0.095 # gripper height
        if table_height == 0.0:
            print "ERROR: Table not detected"
        else:
            self.goal.pose.position.z = table_height#-0.45+ 0.075

        rospy.loginfo("Place goal=%s",str(self.goal))       
        tools.make_marker(self.goal, "goal_pose", mtype=Marker.ARROW, \
                        color=[0.0,1.0,0.0,0.8])

        if obj_type == "paper_plate":
            self.make_plate(obj_name, 18979) # small plate
        elif obj_type == "plastic_plate":
            self.make_plate(obj_name, 18980) # big plate
        coll_obj = tools.get_obj(self.my_world, obj_name)
        shape = tools.get_shape(coll_obj)
        obs = tools.make_gripper_obs()
        obj = Object(shape, self.goal, self.modes)
        return (coll_obj, obj, obs, shape)

    def make_plate(self, obj_name, plate):
        '''
        Converts the given obj in the collision map into a plate mesh 
        in the database
        @type obj_name: String
        @param obj_name: name of the obj in the collision map 
                         ("graspable_object_****")
        @type plate: int
        @param plate: either the big (18980) or small (18979) plate
        '''
        mesh = self._get_model_mesh_srv(plate)

        co = self.my_world.attached_collision_object(obj_name)
        co.object.shapes[0] = mesh.mesh
        co.object.poses[0].orientation.x = 0.707
        co.object.poses[0].orientation.y = 0
        co.object.poses[0].orientation.z = 0
        co.object.poses[0].orientation.w = 0.707
        self.my_world.add_attached_object(co)

    def run(self, action, obj_type, right=True):
        '''
        @type action: String
        @param action: type of place, "dropping","passive","robust"
        @type obj_type: String
        @param obj_type: type of object, "paper_plate","plastic_plate","box"
        
        @rtype float
        @returns time for execution
        '''
#        action = sys.argv[1]
#        obj_type = sys.argv[2]
        obj_name = "graspable_object_1000"#sys.argv[1]

        (coll_obj, obj, obs, shape) = self.setup(obj_type, obj_name)
        if action == 'dropping':
            self.drop(coll_obj, obj, obj_type, right, drop_height=0.03)
            return time.time()
        elif action == "passive":
            success = self.passive(coll_obj, obj, obj_type, right)
            return time.time()
        elif action == "robust":
            success = self.robust(coll_obj, obj, obs, obj_type, shape, right)
            return time.time()
        else:
            rospy.loginfo("ERROR: %s not a valid action", str(action))
        return time.time()

##############################################################################
##
##  Placing implemented in 3 domains: dropping, passive, robust
##
##############################################################################

    def drop(self, coll_obj, obj, obj_type, right, drop_height=0.1):
        '''
        @type coll_obj:
        @param coll_obj: 

        @type obj: physics.Object
        @param obj: object being dropped
        @type drop_height: float
        @param drop_height: height to drop the object from (above the table)
        '''
        pose = PoseStamped()
        pose.header.frame_id = "/torso_lift_link"
        pose.pose.position.x = self.goal.pose.position.x
        pose.pose.position.y = self.goal.pose.position.y
        pose.pose.position.z = self.goal.pose.position.z + drop_height#.1#.05# + 0.02 #.1
        pose.pose.orientation = self.goal.pose.orientation

        ignore = tools.get_collisions(self.my_world)
        if right:
            gripper = tools.get_gripper(self.right_arm, coll_obj, pose)
            success = tools.move(self.my_arm_mover, "right_arm", gripper, ignore, True)
            # self.release("right_arm", gripper, ignore, obj_type)        

            tools.make_marker(gripper, "goal_pose_gripper", mtype=Marker.ARROW, \
                                  color=[0.0,1.0,0.8,0.8])

        else:
            gripper = tools.get_gripper(self.left_arm, coll_obj, pose)

            tools.make_marker(gripper, "goal_pose_gripper", mtype=Marker.ARROW, \
                                  color=[0.0,1.0,0.8,0.8])
            

        success = tools.move(self.my_arm_mover, "left_arm", gripper, ignore, True)
        self.release("left_arm", gripper, ignore, obj_type)        

    def passive(self, coll_obj, obj, obj_type, right):
        ignore = tools.get_collisions(self.my_world)
        (release_pose, wrist_release) = self.get_best_release(coll_obj, obj, ignore, right)

        tools.make_marker(release_pose, namespace="release_pose")
        print "release_pose="+str(release_pose)

        if right:
            success = tools.move(self.my_arm_mover, "right_arm", wrist_release, \
                                     ignore, True)
            # self.release("right_arm", wrist_release, ignore, obj_type)
            return success
        else:
            tools.make_marker(wrist_release, namespace="poop")
            success = tools.move(self.my_arm_mover, "left_arm", wrist_release, \
                                     ignore, True)

            self.release("left_arm", wrist_release, ignore, obj_type)
            return success
            

    def robust(self, coll_obj, obj, obs, obj_type, shape, right):
        ignore = tools.get_collisions(self.my_world)
        (release_pose, wrist_release) = self.get_best_release(coll_obj, obj, ignore, right)

        tools.make_marker(release_pose, namespace="release_pose")
        print "release_pose="+str(release_pose)

        success = False
        block_poses = self.block(obs, obj, release_pose)

        while not success: # not passive placing
            if len(block_poses) != 0.0:            
                next = block_poses.pop(0)
                #print "NEXT="+str(next)
                tools.make_marker(next, namespace="block", mtype=Marker.ARROW, \
                                      scale=(0.1,0.1,0.05), color=(0.0,1.0,0.0,0.8))
                if right:
                    success = tools.move(self.my_arm_mover, "left_arm", next, ignore, False)
                    rospy.loginfo("Block success=%s",str(success))
                else:
                    success = tools.move(self.my_arm_mover, "right_arm", next, ignore, False)
                    rospy.loginfo("Block success=%s",str(success))
            else:
                rospy.loginfo("Block failed.") 
                break

        if right:
            success = tools.move(self.my_arm_mover, "right_arm", wrist_release, \
                                     ignore, True)
            # self.release("right_arm", wrist_release, ignore, obj_type)
            return success
        else:
            success = tools.move(self.my_arm_mover, "left_arm", wrist_release, \
                                     ignore, True)
            self.release("left_arm", wrist_release, ignore, obj_type)
            return success
           


##############################################################################
##
##  Tools needed for placing: getting the best release pose, determing where
##  to place the other hand, and releasing the object
##
##############################################################################

    def get_best_release(self, coll_obj, obj, collisions, right):
        placements = obj.best_placements(self.goal)
        #placements.reverse()
        
        #(height, orientation) = placements.pop()
        #pose = self.make_pose(goal, height, orientation)
        pose = placements.pop(0)
        # pose = placements.pop(0)
        # pose = placements.pop(0)
        # pose = placements.pop(0)
        # pose = placements.pop(0)
#        pose = placements.pop(0)
#        pose = placements.pop(0)

        if right:
            arm = self.right_arm
        else:
            arm = self.left_arm

        gripper = tools.get_gripper(arm, coll_obj, pose)

        while not tools.reachable(arm, gripper, collisions):
            if len(placements) == 0:
                rospy.loginfo("ERROR: No reachable place pose.")
                break

            #(height, orientation) = placements.pop()
            #pose = self.make_pose(goal, height, orientation)
            pose = placements.pop(0)
            pose.pose.position.z = pose.pose.position.z# + .1
            print "all day. all mother fucking day.", len(placements)
            # m = Mode(obj, self.goal)
            # prob = m.fall_probability(pose)
            # print "prob="+str(prob)
            # print "orien="+str(pose.pose.orientation)
            gripper = tools.get_gripper(arm, coll_obj, pose)

        print "donezo"
        tools.make_marker(pose, namespace='place_pose', mtype=Marker.ARROW, \
                             color=(0.0,0.0,1.0,0.8))
        tools.make_marker(gripper, namespace="place_gripper_pose", \
                              mtype=Marker.ARROW, scale=(0.1,0.1,0.05))

        return (pose, gripper)

    def block(self, obs, obj, release_pose):
        buf = 0.03#0.1#.2#0.05
        
        # x = goal.pose.position.x
        # y = goal.pose.position.y
        # z = goal.pose.position.z
        x = release_pose.pose.position.x+.05
        y = release_pose.pose.position.y
        z = release_pose.pose.position.z-0.1

        if obj.shape.type == Shape.MESH:
            bb = gt.bounding_box_corners(obj.shape)
            dims = [(bb[7][0]-bb[0][0]),(bb[7][1]-bb[0][1]),(bb[7][2]-bb[0][2])]
            x_displacement = dims[0]/2 + obs.dimensions[0]/2 + buf
            y_displacement = dims[1]/2 + obs.dimensions[1]/2 + buf
            z_displacement = dims[2]/2 + obs.dimensions[2]/2 + buf
        else:
            x_displacement = obj.shape.dimensions[0]/2 + obs.dimensions[0]/2 + buf
            y_displacement = obj.shape.dimensions[1]/2 + obs.dimensions[1]/2 + buf
            z_displacement = obj.shape.dimensions[2]/2 + obs.dimensions[2]/2 + buf
        
        mode = Mode(obj, self.goal)
        dof = mode.get_degree_of_freedom(release_pose)
        ordered_dof = dof.index_order()

        block_poses = []
        print "ORDERED DOF=", dof.index_order()
        for i in dof.index_order():
            block_pose = PoseStamped()
            block_pose.header.frame_id = "/torso_lift_link"
            if i == 0:
                print "blocking x"
                block_pose.pose.position.x = x - x_displacement + 0.07 #+ buf
                block_pose.pose.position.y = y - 0.1
                block_pose.pose.position.z = z +0.2 #+ 0.12
                block_pose.pose.orientation.w = 0.707 
                block_pose.pose.orientation.x = 0.0#0.707#0.0
                block_pose.pose.orientation.y = 0.707#0.0#0.707
                block_pose.pose.orientation.z = 0.0
            elif i == 1:
                print "blocking y"
                block_pose.pose.position.x = x
                block_pose.pose.position.y = y - y_displacement
                block_pose.pose.position.z = z
                block_pose.pose.orientation.w = 1.0 
                block_pose.pose.orientation.x = 0.0
                block_pose.pose.orientation.y = 0.0
                block_pose.pose.orientation.z = 0.0
            elif i == 2:
                block_pose.pose.position.x = x
                block_pose.pose.position.y = y
                block_pose.pose.position.z = z - z_displacement 
                block_pose.pose.orientation.w = 0.707 
                block_pose.pose.orientation.x = 0.0
                block_pose.pose.orientation.y = 0.0
                block_pose.pose.orientation.z = 0.707
            block_poses.append(block_pose)

            name = "block"+str(i)
            # tools.make_marker(block_pose, namespace=name, mtype=Marker.ARROW, \
            #                       scale=(0.1,0.1,0.05), color=(0.0,1.0,0.0,0.8))

        return block_poses

    def release(self, arm, pose, collisions, obj_type):
        '''
        Releases the object: opens gripper and pulls it back out of the way.
        
        @type arm: String
        @param arm: 'left_arm' or 'right_arm'
        @type pose: PoseStamped
        @param pose: pose the obj is released from
        @type collisions: OrderedCollisionOperations
        @param collisions: list of collisions to ignore in move
        '''
        # open gripper
        #raw_input()
        rospy.loginfo("Releasing the object.")
        my_gripper = gripper.Gripper(arm)
        my_gripper.open()

        #raw_input()
        # pull hand back
        rospy.loginfo("Pulling the gripper out of the way.")
        back = PoseStamped()
        back.header.frame_id = "torso_lift_link"
        if obj_type == "box":
            back.pose.position.x = pose.pose.position.x
            if arm=="right_arm":
                back.pose.position.y = pose.pose.position.y - 0.15
            else:
                back.pose.position.y = pose.pose.position.y + 0.15
        else: 
            back.pose.position.x = pose.pose.position.x - 0.12
            back.pose.position.y = pose.pose.position.y
        back.pose.position.z = pose.pose.position.z
        back.pose.orientation = pose.pose.orientation
        tools.make_marker(back, namespace="pull_out", mtype=Marker.ARROW, \
                             scale=(0.1,0.1,0.05), color=(1.0,1.0,0.0,0.8))
        if arm=='right_arm':
            joint_trajectory = self.right_arm.plan_interpolated_ik(\
                back, collision_aware=False, consistent_angle=2.0*np.pi)
        else:
            joint_trajectory = self.left_arm.plan_interpolated_ik(\
                back, collision_aware=False, consistent_angle=2.0*np.pi)
        
        self.my_arm_mover.execute_joint_trajectory(arm, joint_trajectory)


rospy.init_node("test_motion_prims")        
obj_type = sys.argv[1]
action = sys.argv[2]
if (action=="dropping" or action=="passive" or action=="robust"):
    goal = PoseStamped()
    goal.header.frame_id = '/torso_lift_link'
    
    if obj_type=="paper_plate":
        goal.pose.position.x = 0.6
        goal.pose.position.y = 0.25
        goal.pose.position.z = -0.31
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        modes = [0.0, 1.0]
        
        my_placer = Placer(goal, modes)
        my_placer.run(action, obj_type, right=False)
        
    elif obj_type=="plastic_plate":
        goal.pose.position.x = 0.6
        goal.pose.position.y = 0.25
        goal.pose.position.z = -0.31
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        modes = [0.0, 1.0]
        
        my_placer = Placer(goal, modes)
        my_placer.run(action, obj_type, right=False)
        
    elif obj_type=="box":
        goal.pose.position.x = 0.6
        goal.pose.position.y = 0.25
        goal.pose.position.z = -0.17
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = -0.707
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 0.707
        modes = [1.0, 0.0]
        
        my_placer = Placer(goal, modes)
        my_placer.run(action, obj_type, right=False)
        
    else:
        print "ERROR: unsupported object type."
        
else:
    print "ERROR: unsupported action"



