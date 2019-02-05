#!/usr/bin/env python

import roslib; 
roslib.load_manifest('collision_placing')
import rospy
import kinematics_msgs
import sys
import random
import time

import numpy as np
from mobject import *
from motion_prim import *
from pr2_python import arm_planner, world_interface, arm_mover, gripper
import pr2_python.transform_listener as tf
import pr2_python.geometry_tools as gt
from arm_navigation_msgs.msg import Shape, CollisionObject, AttachedCollisionObject, OrderedCollisionOperations, CollisionOperation
from physics import *
from pr2_tasks.pickplace import PickPlace
from pr2_tasks.pickplace_definitions import PlaceGoal, ObjectInfo, FrameFreeGrasp
from geometry_msgs.msg import PoseStamped, Point, Pose
from pr2_python.exceptions import ManipulationError
from visualization_msgs.msg import Marker
from pr2_python.exceptions import ArmNavError

from household_objects_database_msgs.srv import GetModelMesh

class placer:
    def __init__(self, goal, prims):
        '''
        @type goal: PoseStameped
        @param goal: goal pose of the object
        @type prims: list
        @param prims: list of possible motion primitives for the given object
        '''
        self.goal = goal
        self.prims = prims

        self.my_world = world_interface.WorldInterface()
        self.right_arm = arm_planner.ArmPlanner('right_arm')
        self.left_arm = arm_planner.ArmPlanner('left_arm')
        self.my_arm_mover = arm_mover.ArmMover()
        self.pickplace = PickPlace(search_place=[], place_on_table=False)
        self._get_model_mesh_srv = rospy.ServiceProxy('/objects_database_node/get_model_mesh',
                                                      GetModelMesh)
        rospy.loginfo('Waiting for model mesh service')
        self._get_model_mesh_srv.wait_for_service()

    def get_obj(self, name):
        '''
        Returns the object with the given id if it is attached in the collision map
        
        @type name: String
        @param name: name of the object in the collision map
        
        @rtype arm_navigation_msgs.msg.attached_collision_object
        @returns object if one exists in the collision map
        '''
        objs = self.my_world.attached_collision_objects()
        for obj in objs:
            if obj.object.id == name:
                return obj
        rospy.loginfo("ERROR: OBJECT NAME DOES NOT EXIST")
        return 

    def get_shape(self, obj):
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

    def get_table_height(self):
        '''
        returns the height of the table in '/torso_lift_link' frame

        @rtype float
        @returns table height
        '''
        height = 0.0
        objs = self.my_world.collision_objects()
        for obj in objs:
            if obj.id == "table":
                table_pose = tf.transform_pose("/torso_lift_link", \
                                                   obj.header.frame_id, \
                                                   obj.poses[0])
                height = table_pose.position.z+0.01
        return height

    def get_gripper(self, obj, goal):
        '''
        finds the pose of the gripper for an object in its hand at the given goal pose
        
        @type obj: arm_navigation_msgs.msg.attached_collision_object
        @param obj: the object being held by the gripper
        @type goal: PoseStamped
        @param goal: goal pose for the object in torso_lift_link

        @rtype PoseStamped
        @returns pose of the gripper for the given object at the given goal 
        '''
        # first get the object's current pose
        obj_pose = tf.transform_pose("/torso_lift_link", obj.object.header.frame_id, \
                                         obj.object.poses[0])
        obj_pose_stamped = PoseStamped()
        obj_pose_stamped.header.frame_id = "/torso_lift_link"
        obj_pose_stamped.pose = obj_pose
#        self.make_marker(obj_pose_stamped, namespace='obj_pose', color=(1.0,1.0,1.0,0.8))

        # next get the gripper's current pose
        gripper_pose_stamped = self.right_arm.get_hand_frame_pose(\
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

    def move(self, arm, pose, collisions, try_hard):
        '''
        @type pose: PoseStamped
        @param pose: desired pose for the arm 
        
        @rtype boolean
        @returns whether the move was successful
        '''
        arm_name = arm #'left_arm'
        handle = self.my_arm_mover.move_to_goal(arm_name, pose, \
                                                    ordered_collisions=collisions, \
                                                    try_hard=try_hard)
        rospy.loginfo("MOVE SUCCESS=%s",str(handle.reached_goal()))
        return handle.reached_goal()

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
            back.pose.position.y = pose.pose.position.y - 0.15
        else: 
            back.pose.position.x = pose.pose.position.x - 0.15
            back.pose.position.y = pose.pose.position.y
        back.pose.position.z = pose.pose.position.z
        back.pose.orientation = pose.pose.orientation
        self.make_marker(back, namespace="pull_out", mtype=Marker.ARROW, \
                             scale=(0.1,0.1,0.05), color=(1.0,1.0,0.0,0.8))
        if arm=='right_arm':
            joint_trajectory = self.right_arm.plan_interpolated_ik(\
                back, collision_aware=False, consistent_angle=2.0*np.pi)
        else:
            joint_trajectory = self.left_arm.plan_interpolated_ik(\
                back, collision_aware=False, consistent_angle=2.0*np.pi)
        
        self.my_arm_mover.execute_joint_trajectory(arm, joint_trajectory)

    def make_marker(self, pose, namespace="my_namespace", mtype=Marker.SPHERE, \
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


    def block(self, obs, i, dist, shape, goal):
        '''
        for a given degree of freedom, will find the best placement for the obstacle to block
        that direction. moves the hand to that pose.
        
        @type obs: arm_navigation_msgs.msg.collision_object
        @param obs: obstacle (box representation of the other gripper)
        @type i: int
        @param i: index of the of the degree of freedom (x_trans,y_trans,z_trans,x_rot,y_rot,z_rot)
                  in the frame of the object
        @type dist: float
        @param dist: dist in the specified degree of freedom the obj will travel
        @type shape: arm_navigation_msgs.msg.shape
        @param shape: shape of obj
        @type goal: PoseStamped
        @param goal: goal pose of the object

        @type boolean
        @returns the success of the block move
        '''
        buf = 0.02#0.1#.2#0.05
        
        x = goal.pose.position.x
        y = goal.pose.position.y
        z = goal.pose.position.z

        if shape.type == Shape.MESH:
            bb = gt.bounding_box_corners(shape)
            dims = [(bb[7][0]-bb[0][0]),(bb[7][1]-bb[0][1]),(bb[7][2]-bb[0][2])]
            x_displacement = dims[0]/2 + obs.dimensions[0]/2 + buf
            y_displacement = dims[1]/2 + obs.dimensions[1]/2 + buf
            z_displacement = dims[2]/2 + obs.dimensions[2]/2 + buf
        else:
            x_displacement = shape.dimensions[0]/2 + obs.dimensions[0]/2 + buf
            y_displacement = shape.dimensions[1]/2 + obs.dimensions[1]/2 + buf
            z_displacement = shape.dimensions[2]/2 + obs.dimensions[2]/2 + buf

        #print "shape dims="+str(shape.dimensions[1]/2)+" obs dims="+str(obs.dimensions[1]/2)

        pose = PoseStamped()
        pose.header.frame_id = "/torso_lift_link"
        pose.pose.orientation.w = 0.707 
        pose.pose.orientation.x = 0.707
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0#-0.707

        if i==0 or i==3: # x axis
            if dist == 2.0: # stick
                pose.pose.orientation.w = 0.707
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.707
                pose.pose.orientation.z = 0.0
                
                x = x - x_displacement + .02
                y = y + 0.09
                z = z + 0.18
            elif dist>0: # tip
                x = x + x_displacement
            else:
                x = x - x_displacement
        if i==1 or i==4: # y axis
            if dist>0:
                y = y + y_displacement
            else:
                y = y - y_displacement

        if i==2 or i==5: # z axis
            if dist>0:
                z = z + z_displacement
            else:
                z = z - z_displacement

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z + 0.02
        self.make_marker(pose,namespace="block", mtype=Marker.ARROW, scale=(0.1,0.1,0.05), \
                             color=(0.0,1.0,0.0,0.8))
        success = self.move("left_arm", pose, None, False)
        rospy.loginfo("MOVE SUCCESS=%s",str(success))
        return success

    def reachable(self, pose, collisions):
        '''
        determines whether the a pose is has an ik solution

        @type pose:
        @param pose:

        @rtype boolean
        @returns whether the pose is valid
        '''             
        ik_sol = self.right_arm.get_ik(pose, ordered_collisions=collisions)
        #rospy.loginfo("ERROR CODE=%s"+str(ik_sol.error_code.val))
        if ik_sol.error_code.val != 1:
            return False
        else:
            return True

    def place(self, arm, obj_name, pose):    
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
            place_result = self.pickplace.place(place_goal)
            rospy.loginfo("Place result=%s",str(place_result))
        except ManipulationError:
            rospy.loginfo("POSE FAILED")
            return False
        return True

    def make_pose(self, pose, h, orientation):
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
        pose.header.frame_id = self.goal.header.frame_id
        pose.pose.position.x = self.goal.pose.position.x
        pose.pose.position.y = self.goal.pose.position.y
        pose.pose.position.z = self.goal.pose.position.z + h
        pose.pose.orientation = gt.multiply_quaternions(self.goal.pose.orientation, orientation)

        return pose

    def get_best_release(self, mobj, obj, goal, collisions):
        '''
        finds the best release for the given object at the given goal and returns
        the pose of the gripper for that release.

        @type mobj: Mobject
        @param mobj: Mobject of the object to be placed
        @type obj: arm_navigation_msgs.msg.collision_object
        @param obj: obj to be placed
        @type goal: PoseStamped
        @param goal: goal pose for the object
        
        @rtype PoseStamped
        @returns pose of the gripper for the object at the goal
        '''
        placements = mobj.object.best_placements(goal)
        placements.reverse()
        
        (height, orientation) = placements.pop()
        pose = self.make_pose(goal, height, orientation)
        gripper = self.get_gripper(obj, pose)

        while not self.reachable(gripper, collisions):
            if len(placements) == 0:
                rospy.loginfo("ERROR: No reachable place pose.")
                break

            (height, orientation) = placements.pop()
            pose = self.make_pose(goal, height, orientation)
            gripper = self.get_gripper(obj, pose)

        self.make_marker(pose, namespace='place_pose', mtype=Marker.ARROW, \
                             color=(0.0,0.0,1.0,0.8))
        self.make_marker(gripper, namespace="place_gripper_pose", mtype=Marker.ARROW, \
                             scale=(0.1,0.1,0.05))

        return (pose, gripper)

    def make_gripper_obs(self):
        '''
        Creates a box obstacle from the dimensions of the gripper.
        '''
        obs = Shape()
        obs.type = 1
        obs.dimensions =[0.10,0.10,0.10]
        return obs

    def make_plate(self, obj_name, plate):
        '''
        Converts the given obj in the collision map into a plate mesh in the database
        @type obj_name: String
        @param obj_name: name of the obj in the collision map ("graspable_object_****")
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

    def get_collisions(self):
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
        rfing = CollisionOperation()
        rfing.object1 = "r_gripper_l_finger_link" 
        rfing.object2 = "table"
        rfing.operation = CollisionOperation.DISABLE
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
        for l in self.my_world.hands['left_arm'].hand_links:
            op.object2 = l
            ops.append(copy.deepcopy(op))
        
        ignore.collision_operations = [gripper,wrist,obj, map_obj, lfing, rfing, rfingtip, forearm, flex]+ops
        return ignore

    def setup(self, obj_type, obj_name):
        '''
        @type obj_type: String
        @param obj_type: type of place, "dropping","passive","robust"
        @type obj_name: String 
        @param obj_name: type of object, "paper_plate","plastic_plate","box"
        '''
        # self.my_world.reset_collider_node() # takes a long time
        table_height = self.get_table_height()
        self.goal.pose.position.z = table_height + 0.1#+ 0.095 # height of the gripper

        rospy.loginfo("Place goal=%s",str(self.goal))       
        self.make_marker(self.goal, "goal_pose", mtype=Marker.ARROW, color=[0.0,1.0,0.0,0.8])

        if obj_type == "paper_plate":
            self.make_plate(obj_name, 18979) # small plate
        elif obj_type == "plastic_plate":
            self.make_plate(obj_name, 18980) # big plate
        obj = self.get_obj(obj_name)
        shape = self.get_shape(obj)
        mobj = Mobject(shape, self.prims)
        obs = self.make_gripper_obs()
        return (obj, obs, mobj, shape)

    def drop(self, obj, obj_type, drop_height=0.1):
        '''
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

        ignore = self.get_collisions()
        gripper = self.get_gripper(obj, pose)
        success = self.move("right_arm", gripper, ignore, True)
        self.release("right_arm", gripper, ignore, obj_type)        
    
    def passive(self, obj, mobj, obj_type):
        '''
        @type obj: physics.Object
        @param obj: object being placed
        @type mobj: mobject.Mobject
        @param mobj: object being placed

        @rtype boolean
        @returns success of passive place
        '''
        ignore = self.get_collisions()
        (release_pose, wrist_release) = self.get_best_release(mobj, obj, self.goal, ignore)

        self.make_marker(release_pose, namespace="release_pose")
        print "release_pose="+str(release_pose)

        success = self.move("right_arm", wrist_release, ignore, True)
        self.release("right_arm", wrist_release, ignore, obj_type)

        return success

    def robust(self, obj, mobj, obs, obj_type, shape):
        '''
        @type obj: physics.Object
        @param obj: object being placed
        @type mobj: mobject.Mobject
        @param obj: object being placed
        @type obs: Shape
        @param obs: the gripper as an obstacle
        
        @rtype boolean
        @returns succes of the robust place
        '''
        ignore = self.get_collisions()
        (release_pose, wrist_release) = self.get_best_release(mobj, obj, self.goal, ignore)

        self.make_marker(release_pose, namespace="release_pose")
        print "release_pose="+str(release_pose)

        vels = [0.0,0.0,0.0]
        dof = mobj.find_dof(self.goal, release_pose, vels)
        print "DOF="+str(dof.degrees)

        success = False
        order = dof.index_order()
        while not success: # not passive placing
            if len(order) != 0.0:            
                next = order.pop(0)
                #print "NEXT="+str(next)
                success = self.block(obs, next, dof[next], shape, self.goal)
                rospy.loginfo("Block success=%s",str(success))
            else:
                rospy.loginfo("Block failed.") 
                break

        success = self.move("right_arm", wrist_release, ignore, True)
        self.release("right_arm", wrist_release, ignore, obj_type)

        return success

    def run(self, action, obj_type, obj_name):
        '''
        @type action: String
        @param action: type of place, "dropping","passive","robust"
        @type obj_type: String
        @param obj_type: type of object, "paper_plate","plastic_plate","box"
        @type obj_name: String
        @param obj_name: name of the object in the collision map, "graspable_object_****"
        
        @rtype float
        @returns time for execution
        '''
        (obj, obs, mobj, shape) = self.setup(obj_type, obj_name)
        if action == 'dropping':
            self.drop(obj, obj_type)
            return time.time()
        elif action == "passive":
            success = self.passive(obj, mobj, obj_type)
            return time.time()
        elif action == "robust":
            success = self.robust(obj, mobj, obs, obj_type, shape)
            return time.time()
        else:
            rospy.loginfo("ERROR: %s not a valid action", str(action))
        return time.time()

