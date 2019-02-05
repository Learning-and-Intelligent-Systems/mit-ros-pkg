#!/usr/bin/env python

import roslib; 
roslib.load_manifest('collision_placing')
import rospy
import kinematics_msgs
import sys
import random
import time

from mobject import *
from motion_prim import *
from pr2_python import arm_planner, world_interface, arm_mover, gripper
import pr2_python.transform_listener as tf
import pr2_python.geometry_tools as gt
#from geometry_msgs.msg import PoseStamped
from arm_navigation_msgs.msg import Shape, CollisionObject, AttachedCollisionObject, OrderedCollisionOperations, CollisionOperation
from physics import *
from pr2_tasks.pickplace import PickPlace
from pr2_tasks.pickplace_definitions import PlaceGoal, ObjectInfo, FrameFreeGrasp
from geometry_msgs.msg import PoseStamped, Point, Pose
from pr2_python.exceptions import ManipulationError
from visualization_msgs.msg import Marker
from pr2_python.exceptions import ArmNavError

class placer:
    def __init__(self, goal, mass, prims):
        '''
        @type goal: PoseStameped
        @param goal: goal pose of the object
        @type mass: float
        @param mass: mass of the object
        @type prims: list
        @param prims: list of possible motion primitives for the given object
        '''
        self.goal = goal
        self.mass = mass
        self.prims = prims

        self.my_world = world_interface.WorldInterface()
        self.right_arm = arm_planner.ArmPlanner('right_arm')
        self.left_arm = arm_planner.ArmPlanner('left_arm')
        self.pickplace = PickPlace(search_place=[], place_on_table=False)

    def get_obj(self, name):
        '''
        Returns the object with the given id if it is attached in the collision map
        
        @type name: String
        @param name: name of the object in the collision map
        
        @rtype arm_navigation_msgs.msg.attached_collision_object
        @returns object if one exists in the collision map
        '''
#        other = self.my_world.

        objs = self.my_world.attached_collision_objects()
        for obj in objs:
            if obj.object.id == name:
                return obj
        print "ERROR: OBJECT NAME DOES NOT EXIST"
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
            print "ERROR: TOO MANY SHAPES: "+str(len(obj.object.shapes))
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
            print "obj id="+str(obj.id)
            if obj.id == "table":
                table_pose = tf.transform_pose("/torso_lift_link", obj.header.frame_id, obj.poses[0])
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
        obj_pose = tf.transform_pose("/torso_lift_link", obj.object.header.frame_id, obj.object.poses[0])
        obj_pose_stamped = PoseStamped()
        obj_pose_stamped.header.frame_id = "/torso_lift_link"
        obj_pose_stamped.pose = obj_pose
#        self.make_marker(obj_pose_stamped, namespace='obj_pose', color=(1.0,1.0,1.0,0.8))

        # next get the gripper's current pose
        gripper_pose_stamped = self.right_arm.get_hand_frame_pose(frame_id="/torso_lift_link") 
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

    def move(self, arm, pose, collisions):
        '''
        @type pose: PoseStamped
        @param pose: desired pose for the arm 
        
        @rtype boolean
        @returns whether the move was successful
        '''
        # goal = PoseStamped()
        # goal.header.frame_id = '/torso_lift_link'
        # goal.pose.position.x = 0.7
        # goal.pose.position.y = 0.2
        # goal.pose.position.z = -0.28
        # goal.pose.orientation.x = 0.707
        # goal.pose.orientation.y = 0.0
        # goal.pose.orientation.z = 0.0
        # goal.pose.orientation.w = 0.707
        # print "move pose="+str(goal)

        my_arm_mover = arm_mover.ArmMover()
        arm_name = arm #'left_arm'
        handle = my_arm_mover.move_to_goal(arm_name, pose, ordered_collisions=collisions)
        return handle.reached_goal()

    def release(self, arm):
        my_gripper = gripper.Gripper(arm)
        my_gripper.open()

    def make_marker(self, pose, namespace="my_namespace", mtype=Marker.SPHERE, \
                        scale=(0.05,0.05,0.05), color=(1.0,0.0,1.0,0.8)):
        pub = rospy.Publisher('doodie', Marker)

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
            marker.color.r = color[0]#1.0
            marker.color.g = color[1]#0.0
            marker.color.b = color[2]#1.0
            marker.color.a = color[3]#0.8
#            print "marker pose="+str(pose)

            #rospy.loginfo("MARKER msg="+str(marker))
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

        @type PoseStamped
        @returns the pose of the gripper positioned to block the given the degree of freedom
        '''
#        print "obs dim="+str(obs.dimensions)
        buf = 0.02#0.1#.2#0.05
        
        x = goal.pose.position.x
        y = goal.pose.position.y
        z = goal.pose.position.z

        x_displacement = shape.dimensions[0]/2 + obs.dimensions[0]/2 + buf
        y_displacement = shape.dimensions[1]/2 + obs.dimensions[1]/2 + buf
        z_displacement = shape.dimensions[2]/2 + obs.dimensions[2]/2 + buf

        #print "shape dims="+str(shape.dimensions[1]/2)+" obs dims="+str(obs.dimensions[1]/2)

        if i==0 or i==3: # x axis
            if dist>0:
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

        
        pose = PoseStamped()
        pose.header.frame_id = "/torso_lift_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z + 0.1
        pose.pose.orientation.w = 0.707 
        pose.pose.orientation.x = 0.707
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0#-0.707
        self.make_marker(pose,namespace="block", mtype=Marker.ARROW, scale=(0.1,0.1,0.05), color=(0.0,1.0,0.0,0.8))

        success = self.move("left_arm", pose, None)
        print "MOVE SUCCESS="+str(success)
        if not success:
            print "MOVE FAILED"
        return success
#        return pose

    def reachable(self, pose):
        '''
        determines whether the a pose is has an ik solution

        @type pose:
        @param pose:

        @rtype boolean
        @returns whether the pose is valid
        '''             
        ik_sol = self.right_arm.get_ik(pose)
#        rospy.loginfo("ERROR CODE="+str(ik_sol.error_code.val))
        if ik_sol.error_code.val != 1:
#            print "false. not reachable"
            return False
        else:
            print "true. reachable"
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
                place_goal = PlaceGoal('right_arm', [pose], collision_support_surface_name='all', collision_object_name=obj_name)
            else:
                place_goal = PlaceGoal('left_arm', [pose], collision_support_surface_name='table', collision_object_name=obj_name)
            print "ABOUT TO PLACE"
            doodie = self.pickplace.place(place_goal)
            print "doodie="+str(doodie)
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
        pose.header.frame_id = goal.header.frame_id
        pose.pose.position.x = goal.pose.position.x
        pose.pose.position.y = goal.pose.position.y
        pose.pose.position.z = goal.pose.position.z + h
        pose.pose.orientation = gt.multiply_quaternions(goal.pose.orientation, orientation)
        # pose.pose.orientation.x = #goal.pose.orientation.x + orientation.x#orientation[1]
        # pose.pose.orientation.y = #goal.pose.orientation.y + orientation.y#orientation[2]
        # pose.pose.orientation.z = #goal.pose.orientation.z + orientation.z#orientation[3]
        # pose.pose.orientation.w = #goal.pose.orientation.w + orientation.w#orientation[0]
        
        return pose

    def get_best_release(self, mobj, obj, goal):
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
        # height = 0.15
        # orientation = Quaternion()
        # orientation.w = 1.0
        # pose = self.make_pose(goal, height, orientation)
        # gripper = self.get_gripper(obj, pose)

#        orien = Orientation(0.0,0.0,0.0)
        placements = mobj.object.best_placements(goal)
        placements.reverse()
        
        (height, orientation) = placements.pop()
        pose = self.make_pose(goal, height, orientation)
    #    print "REACHABLE="+str(reachable(arm, pose))
        print "start height="+str(height)+" orien="+str(orientation)            
    
        gripper = self.get_gripper(obj, pose)

        while not self.reachable(gripper):
            if len(placements) == 0:
                print 'SHIT SNACKS'
                print "FUCK height="+str(height)+" orien="+str(orientation)                            
                break

#            print "len(placements)="+str(len(placements))
            (height, orientation) = placements.pop()
#            print "height="+str(height)+" orien="+str(orientation)            
            pose = self.make_pose(goal, height, orientation)
            gripper = self.get_gripper(obj, pose)

        print "height="+str(height)+" orien="+str(orientation)            
        self.make_marker(pose, namespace='farts', mtype=Marker.ARROW, color=(0.0,0.0,1.0,0.8))
        self.make_marker(gripper, namespace="poopie", mtype=Marker.ARROW, scale=(0.1,0.1,0.05))
        # print "pose="+str(pose)
        # print "gripper="+str(gripper)

        return (pose, gripper)

    def make_gripper_obs(self):
        obs = Shape()
        obs.type = 1
        obs.dimensions =[0.10,0.10,0.10]
        return obs

    def run(self): #(goal, mass, prims, obs):
        search_start = time.time()
        obj_name = sys.argv[1]

        table_height = self.get_table_height()
        self.goal.pose.position.z = table_height + 0.01
        print "goal="+str(self.goal)
        
        self.make_marker(self.goal, "goal_pose", mtype=Marker.ARROW, color=[0.0,1.0,0.0,0.8])

        obj = self.get_obj(obj_name)
        shape = self.get_shape(obj)

        obs = self.make_gripper_obs()
        
        mobj = Mobject(mass, shape, prims)
        
        pose = PoseStamped()
        pose.header.frame_id = "/torso_lift_link"
        pose.pose.position.x = goal.pose.position.x
        pose.pose.position.y = goal.pose.position.y
        pose.pose.position.z = goal.pose.position.z + 0.02 #.1
        pose.pose.orientation = goal.pose.orientation
        vels = [0.0,0.0,0.0]
        
#        self.place('right_arm', obj_name, pose)

        print "DIMS="+str(shape.dimensions)

        (release_pose, wrist_release) = self.get_best_release(mobj, obj, goal)
#        release_pose = pose
        self.make_marker(release_pose, namespace="release_pose")
        print "release_pose="+str(release_pose)
        search_end = time.time()

        # block_start = time.time()
        # dof = mobj.find_dof(goal, release_pose, vels)
        # print "DOF="+str(dof.degrees)

        # success = False
        # order = dof.index_order()
        # while not success: # not passive placing
        #     if len(order) != 0.0:            
        #         next = order.pop(0)
        #         print "NEXT="+str(next)
        #         success = self.block(obs, next, dof[next], shape, goal)
        #         print "SUCCESS="+str(success)
        #     else:
        #         print "BLOCK FAILED" 
        #         break

        # block_end = time.time()
        print "waiting for place:"
#        raw_input()

        ignore = OrderedCollisionOperations()
        gripper = CollisionOperation()
        gripper.object1 = CollisionOperation.COLLISION_SET_ALL#"l_end_effector"
        gripper.object2 = CollisionOperation.COLLISION_SET_ALL
        gripper.operation = CollisionOperation.DISABLE
        wrist = CollisionOperation()
        wrist.object1 = "l_wrist_roll_link"
        wrist.object2 = CollisionOperation.COLLISION_SET_ALL
        wrist.operation = CollisionOperation.DISABLE
        obj = CollisionOperation()
        obj.object1 = CollisionOperation.COLLISION_SET_ATTACHED_OBJECTS
        obj.object2 = "table"#CollisionOperation.COLLISION_SET_ALL
        obj.operation = CollisionOperation.DISABLE
        ignore.collision_operations = [gripper,wrist,obj]
        self.move("right_arm", wrist_release, ignore)
#        self.release("right_arm")
#        self.place('right_arm', obj_name, release_pose)

        search = search_end - search_start
        block = 0.0#block_end - block_start
        return (search, block)
    
goal = PoseStamped()
goal.header.frame_id = '/torso_lift_link'
goal.pose.position.x = 0.6
goal.pose.position.y = -0.25#0.5#0.2
goal.pose.position.z = -0.28
goal.pose.orientation.x = 0.0
goal.pose.orientation.y = 0.0
goal.pose.orientation.z = 0.707#0.0
goal.pose.orientation.w = 0.707#1.0
mass = 10 # grams?
prims = [MotionPrim.TIP]

rospy.init_node("test_motion_prims")

start = time.time()
my_placer = placer(goal, mass, prims)
(search_time, block_time) = my_placer.run()
end = time.time()
print "TIME="+str(end-start)
print "SEARCH TIME="+str(search_time)
print "BLOCK TIME="+str(block_time)

#my_placer.test_run()



#     def test_run(self):
#         obj_name = sys.argv[1]

#         table_height = 0.0
#         self.goal.pose.position.z = table_height
#         print "goal="+str(self.goal)

        

#         shape = Shape()
#         shape.type = Shape.BOX
#         shape.dimensions = [10,5,10]
#         pose = Pose()
#         pose = self.goal.pose
#         pose.position.z = self.goal.pose.position.z + 0.5
        
#         obj = AttachedCollisionObject()
#         obj.object.header.frame_id = "torso_lift_link"
#         obj.object.id = obj_name
#         shapes = [shape]
#         obj.object.shapes = shapes
#         poses = [pose]
#         obj.object.poses = poses
        
#         # obj = self.get_obj(obj_name)
#         # shape = self.get_shape(obj)

#         obs = self.make_gripper_obs()
        
#         mobj = Mobject(mass, shape, prims)
        
#         pose = PoseStamped()
#         pose.header.frame_id = "/torso_lift_link"
#         pose.pose.position.x = goal.pose.position.x
#         pose.pose.position.y = goal.pose.position.y
#         pose.pose.position.z = goal.pose.position.z + .1
#         pose.pose.orientation = goal.pose.orientation
#         vels = [0.0,0.0,0.0]
        
#         release_pose = self.get_best_release(mobj, obj, goal)
# #        self.make_marker(release_pose, "release_pose")
#         print "release_pose="+str(release_pose)
#         print "release angle="+str(geometry_tools.quaternion_to_euler(release_pose.pose.orientation))

#         dof = mobj.find_dof(goal, release_pose, vels)
#         print "DOF="+str(dof.degrees)

#         if not dof.isZero(): # not passive placing
#             print "SHOULDVE BLOCKED"
#             [i, dist] = dof.i_greatest()
#             obs_pose = self.block(obs, i, dist, shape, goal)
#             print "obs_pose="+str(obs_pose)
        
# #        self.place('right_arm', obj_name, release_pose)
