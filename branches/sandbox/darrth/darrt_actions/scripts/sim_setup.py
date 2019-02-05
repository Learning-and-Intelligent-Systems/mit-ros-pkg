from pr2_python.world_interface import WorldInterface
from pr2_python.torso import Torso
from pr2_python.gripper import Gripper
from pr2_tasks.arm_tasks import ArmTasks
from pr2_python.planning_scene_interface import get_planning_scene_interface
from pr2_python.conversions import pose_to_transform, transform_to_pose
from darrt_msgs.msg import ObjectType
import pr2_python.geometry_tools as gt

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseWithCovarianceStamped, Vector3, Transform
from actionlib import SimpleActionClient
from visualization_msgs.msg import MarkerArray

from arm_navigation_msgs.msg import CollisionObject, Shape
import rospy
import copy
import numpy as np

from darrt_msgs.msg import DARRTGoal, Primitive, RigidGrasp, Goal

FAR_TABLE_HEIGHT = 0.77
CENTER_TABLE_HEIGHT = 0.68
DOOR_TABLE_HEIGHT = 0.73
GRIPPER_LENGTH = 0.2

class World:
    def __init__(self, obj, obj_support_surface, place_surface='', ss_names=None):
        self.object = obj
        self.object_support_surface = obj_support_surface
        self.place_surface = place_surface
        self.support_surface_names = ss_names
        if not self.support_surface_names:
            self.support_surface_names = []

class SimSetup:
    
    def __init__(self, world, screenshot, fake_walls = False, box_plate=False, real=False):
        self.world = world
        self.screenshot = screenshot
        self.wi = WorldInterface()
        self.psi = get_planning_scene_interface()
        self.pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped)
        self.arms = ArmTasks()
        self.torso = Torso()
        self.gripper = {'right_arm':Gripper('right_arm'), 'left_arm':Gripper('left_arm')}
        self.box_plate=box_plate
        self.real = real
        self.fake_walls = fake_walls

    def setup(self, arm):
        self.wi.reset(repopulate=False)
        self.set_robot_position()
        goal = DARRTGoal()
        goal.support_surfaces = self.add_tables()
        if self.fake_walls:
            self.add_walls()
        #all the primitives
        arm_transit = Primitive(name='ArmTransit', parameters=[arm])
        base_transit = Primitive(name='BaseTransit', parameters=['pr2_base'])
        approach = Primitive(name="Approach", parameters=[arm])
        retreat = Primitive(name="Retreat", parameters=[arm])
        rigid_transfer = Primitive(name="RigidTransfer", parameters=[arm])
        push = Primitive(name="Push", parameters=[arm])
        pickup = Primitive(name="Pickup", parameters=[arm])
        uspat = Primitive(name="UseSpatula", parameters=[arm], 
                          numeric_parameters=[0.22, 0.2, 0.2])
        strans = Primitive(name="SpatulaTransfer", parameters=[arm])
        if self.world < 0:
            obj = self.add_little_object()
            spatula, grasps = self.add_spatula(arm)
            block = self.add_block()
            if self.world < -8: 
                self.add_barrier()
            goal.objects = [obj, spatula, block]
            rigid_transfer.parameters.append(spatula.collision_object.id)
            push.parameters += [obj.collision_object.id, arm[0]+'_end_effector']
            uspat.parameters += [spatula.collision_object.id, obj.collision_object.id, 
                                 block.collision_object.id]
            uspat.grasps = grasps
            lift = Vector3(z=1.0)
            uspat.directional_parameters.append(lift)
            strans.parameters += [spatula.collision_object.id, 
                                  obj.collision_object.id]
            goal.primitives = [arm_transit, base_transit, approach, retreat,
                               rigid_transfer, push, uspat, strans]
            goal.goals = [Goal(obj.collision_object.id, self.get_goal_poses(), False)]
            self.psi.reset()
            return goal
        else:
            rospy.loginfo('Old world set up is broken!')
            return None

        #this no longer works right
        if self.world < 14:
            obj = self.add_plate()
        else:
            obj = self.add_box()
        if self.world > 7:
            self.add_barrier()
        self.psi.reset()
        if self.world < 6 or self.world > 7:
            ss_surface = ss_names[2]
        #elif self.world == 12:
         #   ss_surface = ss_names[1]
        else:
            ss_surface = ss_names[0]
        return World(obj, ss_surface, ss_names[1])
        
    def set_robot_position(self):
        #right now worlds 6 and above don't move the robot
        if self.world > 15:
            return
        ps = PoseWithCovarianceStamped()
        ps.pose.covariance = [0]*36
        ps.header.frame_id = '/map'
        if abs(self.world) <= 2:
            #easier start pose
            ps.pose.pose.position.x = 2.7
            ps.pose.pose.position.y = -2.5
            ps.pose.pose.orientation.w = 1
        elif self.world < 0 or abs(self.world) < 6 or abs(self.world) == 14:
            ps.pose.pose.position.x = -1
            ps.pose.pose.position.y = 0
            ps.pose.pose.orientation.w = 1
        elif abs(self.world) == 12:
            ps.pose.pose.position.x = 0
            ps.pose.pose.position.y = 1.5
            ps.pose.pose.orientation.w = 1
        else:
            ps.pose.pose.position.x = 1
            ps.pose.pose.position.y = 1
            ps.pose.pose.orientation.z = -1.0/np.sqrt(2.0)
            ps.pose.pose.orientation.w = 1.0/np.sqrt(2.0)
            
        if not self.real:
            self.pose_pub.publish(ps)
        self.gripper['right_arm'].open()
        self.torso.move(0.3)

        self.arms.move_arm_to_side('left_arm')
        if abs(self.world) < 4:
            rospy.loginfo('Moving right arm to up position')
            self.arms._arm_mover.move_to_goal('right_arm', self.arms._arm_planners['right_arm'].joint_positions_to_joint_state(
                    self.arms.side_joint_trajectory['right_arm'][1]), try_hard=True)
        else:
            self.arms.move_arm_to_side('right_arm')

    def add_tables(self):
        if self.screenshot:
            return ['', '', '']
        table = CollisionObject()
        table.header.frame_id = self.wi.world_frame
        table.operation.operation = table.operation.ADD
        
        shape = Shape()
        shape.type = shape.MESH
        #center table
        shape.vertices = [Point(x=-0.2, y=-0.4,z=CENTER_TABLE_HEIGHT), Point(x=0.97, y=-0.6, z=CENTER_TABLE_HEIGHT), 
                          Point(x=1.0, y=0.25, z=CENTER_TABLE_HEIGHT), Point(x=-0.25, y=0.3, z=CENTER_TABLE_HEIGHT)]
        shape.triangles = [0, 1, 2, 2, 3, 0]
        
        pose = Pose()
        pose.orientation.w = 1.0
        poseb = copy.deepcopy(pose)
        poseb.position.z = -0.02
        poset = copy.deepcopy(pose)
        poset.position.z = 0.02
        table.shapes.append(shape)
        table.shapes.append(shape)
        table.shapes.append(shape)
        table.poses.append(poset)
        table.poses.append(pose)
        table.poses.append(poseb)
        table.id = 'center_table'
        self.wi.add_object(table)

        #table near the door
        table.id = 'door_table'
        shape.vertices = [Point(x=-2.4, y=1.84,z=DOOR_TABLE_HEIGHT), Point(x=-1.15, y=1.75, z=DOOR_TABLE_HEIGHT), 
                          Point(x=-1.15, y=2.5, z=DOOR_TABLE_HEIGHT), Point(x=-2.4, y=2.5, z=DOOR_TABLE_HEIGHT)]
        
        self.wi.add_object(table)
        
        #table in far corner
        table.id = 'far_corner'
        shape.vertices = [Point(x=3, y=-2.7,z=FAR_TABLE_HEIGHT), Point(x=2.4, y=-3.8, z=FAR_TABLE_HEIGHT), 
                          Point(x=3.2, y=-4.3, z=FAR_TABLE_HEIGHT), Point(x=3.8, y=-3.2, z=FAR_TABLE_HEIGHT)]
        self.wi.add_object(table)

        if self.fake_walls:
            #these are the table feet
            foot = CollisionObject()
            foot.header.frame_id = self.wi.world_frame
            foot.operation.operation = foot.operation.ADD
            foot.id = "far_corner_foot"
            
            shape = Shape()
            shape.type = shape.BOX
            shape.dimensions = [0.1, 0.5, FAR_TABLE_HEIGHT/2.0]
            
            pose = Pose()
            pose.position.x = 3
            pose.position.y = -3.4
            pose.position.z = shape.dimensions[2]/2.0
            angle = 0.5
            pose.orientation.z = np.cos(angle/2.0)
            pose.orientation.w = np.sin(angle/2.0)
            foot.shapes.append(shape)
            foot.poses.append(pose)
            self.wi.add_object(foot)

            foot.id = "center_table_foot1"
            shape.dimensions = [0.1, 0.75, 0.3]
            pose.position.x = 0.9
            pose.position.y = -0.1
            pose.position.z = shape.dimensions[2]/2.0
            angle = 0
            pose.orientation.z = np.cos(angle/2.0)
            pose.orientation.w = np.sin(angle/2.0)
            self.wi.add_object(foot)

            foot.id = "center_table_foot2"
            pose.position.x = -0.2
            self.wi.add_object(foot)
            
            foot.id = "door_table_foot"
            pose.position.x = -1.25
            pose.position.y = 2.1
            self.wi.add_object(foot)

        return ['center_table', 'door_table', 'far_corner']

    def add_walls(self):
        window_wall = CollisionObject()
        window_wall.header.frame_id = self.wi.world_frame
        window_wall.operation.operation = window_wall.operation.ADD
        window_wall.id = "window_wall"

        shape = Shape()
        shape.type = shape.BOX
        shape.dimensions = [8, 0.1, 2.5]
        pose = Pose()
        pose.position.x = 0
        pose.position.y = -2.63
        pose.position.z = shape.dimensions[2]/2.0-0.1
        angle = np.pi/6.25
        pose.orientation.z = np.cos(angle/2.0)
        pose.orientation.w = np.sin(angle/2.0)
        window_wall.shapes.append(shape)
        window_wall.poses.append(pose)
        self.wi.add_object(window_wall)

        shape.dimensions[0] = shape.dimensions[1]
        shape.dimensions[1] = 5
        pose.position.x = -2.45
        pose.position.y = 1
        angle = 0.05
        pose.orientation.z = np.cos(angle/2.0)
        pose.orientation.w = np.sin(angle/2.0)
        window_wall.id = "door_wall"
        self.wi.add_object(window_wall)
        
        shape.dimensions[1] = shape.dimensions[0]
        shape.dimensions[0] = 8
        pose.position.x = 0
        pose.position.y = 2.3
        angle = 0.1
        pose.orientation.z = np.cos(angle/2.0)
        pose.orientation.w = np.sin(angle/2.0)
        window_wall.id = "cabinet_wall"
        self.wi.add_object(window_wall)

        shape.dimensions[0] = shape.dimensions[1]
        shape.dimensions[1] = 3
        pose.position.x = 4
        pose.position.y = -3
        angle = 0.5
        pose.orientation.z = np.cos(angle/2.0)
        pose.orientation.w = np.sin(angle/2.0)
        window_wall.id = "banner_wall"
        self.wi.add_object(window_wall)

        shape.dimensions[1] = 4
        shape.dimensions[2] = 1
        pose.position.x = 3.25
        pose.position.y = 0.65
        pose.position.z = shape.dimensions[2]/2.0 - 0.05
        angle = 0
        pose.orientation.z = np.cos(angle/2.0)
        pose.orientation.w = np.sin(angle/2.0)
        window_wall.id = "computer_wall"
        self.wi.add_object(window_wall)

        shape.dimensions[1] = shape.dimensions[0]
        shape.dimensions[0] = 1.8
        pose.position.x = 3.9
        pose.position.y = -1.6
        angle = np.pi/5.5
        pose.orientation.z = np.cos(angle/2.0)
        pose.orientation.w = np.sin(angle/2.0)
        window_wall.id = "nook_wall"
        self.wi.add_object(window_wall)


    def add_little_object(self):
        little_obj = CollisionObject()
        little_obj.header.frame_id = self.wi.world_frame
        little_obj.operation.operation = little_obj.operation.ADD
        little_obj.id = "little_obj"
        shape = Shape()
        shape.type = shape.CYLINDER
        shape.dimensions = [0.05, 0.02]
        
        pose = Pose()
        pose.orientation.w = 1.0

        little_obj.shapes.append(shape)
        little_obj.poses.append(pose)
        
        little_obj_p = copy.deepcopy(little_obj)
        little_obj_p.poses[0].position.x = 3.1
        little_obj_p.poses[0].position.y = -3.2
        little_obj_p.poses[0].position.z = FAR_TABLE_HEIGHT + 0.02 + shape.dimensions[1]/2.0
        self.wi.add_object(little_obj_p)
        return ObjectType(type="RoundObject", collision_object=little_obj, parameters=['far_corner'])

    def add_block(self):
        block = CollisionObject()
        block.header.frame_id = self.wi.world_frame
        block.operation.operation = block.operation.ADD
        block.id = "block"
        shape = Shape()
        shape.type = shape.BOX
        shape.dimensions = [0.1, 0.1, 0.13]
        
        pose = Pose()
        pose.orientation.w = 1.0

        block.shapes.append(shape)
        block.poses.append(pose)
        
        block_p = copy.deepcopy(block)
        block_p.poses[0].position.x = 2.9
        block_p.poses[0].position.y = -3.5
        block_p.poses[0].position.z = FAR_TABLE_HEIGHT + 0.02 + shape.dimensions[2]/2.0
        self.wi.add_object(block_p)
        return ObjectType(type="FixedObject", collision_object=block, parameters=['far_table'])


    def add_spatula(self, arm):
        spatula = CollisionObject()
        spatula.id = "spatula";
        spatula.header.frame_id = self.wi.world_frame
        spatula.operation.operation = spatula.operation.ADD
        
        paddle = Shape()
        handle = Shape()
        paddle.type = paddle.BOX
        paddle.dimensions = [0.11, 0.12, 0.005]
        handle.type = handle.CYLINDER
        handle.dimensions = [0.02, 0.24]
        
        paddle_pose = Pose()
        handle_pose = Pose()
        paddle_pose.position.y = paddle.dimensions[1]/2.0
        paddle_pose.orientation.w = 1.0
        
        angle = np.pi/5.0
        handle_pose.position.y = -1.0*handle.dimensions[1]/2.0*np.sin(np.pi/2.0-angle)
        handle_pose.position.z = handle.dimensions[1]/2.0*np.cos(np.pi/2.0-angle)
        handle_pose.orientation.x = np.sin((np.pi/2.0-angle)/2.0)
        handle_pose.orientation.w = np.cos((np.pi/2.0-angle)/2.0)
        
        spatula.shapes = [paddle, handle]
        spatula.poses = [paddle_pose, handle_pose]

        #this is the grasp transformation
        pos_on_handle = handle.dimensions[1] - 0.1
        inv_grasp = Transform()
        grasp = RigidGrasp()
        #really should be calculating this...
        inv_grasp.translation.y = GRIPPER_LENGTH
        inv_grasp.translation.z = pos_on_handle/2.0
        #flip 90 degrees
        inv_grasp.rotation.z = np.sin(-1.0*np.pi/4.0)
        inv_grasp.rotation.w = np.cos(-1.0*np.pi/4.0)
        g = gt.transform_pose(transform_to_pose(inv_grasp), handle_pose)
        origin = Pose()
        origin.orientation.w = 1.0
        grasp.transform = pose_to_transform(gt.inverse_transform_pose(origin, g))
        grasp.touch_links = [arm[0]+'_end_effector']
        grasp.attach_link = arm[0]+'_gripper_r_finger_tip_link'
        grasp.min_approach_distance = 0
        grasp.desired_approach_distance = 0.15
        grasp.min_distance_from_surface = -1
        
        spat_p = copy.deepcopy(spatula)
        
        wtrans = Pose()
        wtrans.orientation.x = np.sin(angle/2.0)
        wtrans.orientation.w = np.cos(angle/2.0)
        if self.world == -1:
            wtrans.position.x = 3
            wtrans.position.y = -2.8
            wtrans.position.z = FAR_TABLE_HEIGHT + 0.02 + handle.dimensions[0]
            ss = ['far_corner']
        elif self.world == -2 or self.world == -7 or self.world == -9 or self.world == -5:
            wtrans.position.x = -1.7
            wtrans.position.y = 2
            wtrans.position.z = DOOR_TABLE_HEIGHT + 0.02 + handle.dimensions[0]
            ss = ['door_table']
        else:
            wtrans.position.x = 0.6
            wtrans.position.y = -0.3
            wtrans.position.z = CENTER_TABLE_HEIGHT + 0.02 + handle.dimensions[0]
            ss = ['center_table']
            if self.world == -4 or self.world == -5:
                wtrans.position.y = 0
                rot = Quaternion()
                rot.z = np.sin(np.pi/2.0)
                rot.w = np.cos(np.pi/2.0)
                wtrans.orientation = gt.multiply_quaternions(rot, wtrans.orientation)
                if self.world == -5:
                    wtrans.position.x = 0

        for i in range(len(spat_p.poses)):
            spat_p.poses[i] = gt.transform_pose(spat_p.poses[i], wtrans)
            
        self.wi.add_object(spat_p)
        return ObjectType(type="SpatulaObject", collision_object=spatula, parameters=ss,
                          numeric_parameters=paddle.dimensions+handle.dimensions+[angle]), [grasp]


    def add_plate(self):
        plate = CollisionObject()
        plate.header.frame_id = self.wi.world_frame
        plate.operation.operation = plate.operation.ADD
        plate.id = "plate"
        
        shape = Shape()
        shape.type = shape.CYLINDER
        shape.dimensions = [0.15, 0.04]
        if self.box_plate:
            shape.type = shape.BOX
            shape.dimensions = [0.3, 0.3, 0.04]
        
        pose = Pose()
        if self.world < 6 or self.world == 8 or self.world == 9 or self.world == 12:
            pose.position.x = 3.1
            pose.position.y = -3.2
            pose.position.z = FAR_TABLE_HEIGHT + 0.02 + shape.dimensions[1]/2.0
        elif self.world == 6:
            pose.position.x = 0.6#0.894692#0.5
            pose.position.y = -0.3#-0.468198#-0.2
            pose.position.z = CENTER_TABLE_HEIGHT + shape.dimensions[1]/2.0
        # elif self.world == 12:
        #     pose.position.x = -1.3
        #     pose.position.y = 2
        #     pose.position.z = DOOR_TABLE_HEIGHT + 0.02 + shape.dimensions[1]/2.0
        else:
            pose.position.x = 0.5#0.894692#0.5
            pose.position.y = 0.1
            pose.position.z = CENTER_TABLE_HEIGHT + 0.02 + shape.dimensions[1]/2.0

        pose.orientation.w = 1.0
        
        plate.shapes.append(shape)
        plate.poses.append(pose)

        self.wi.add_object(plate)

        return plate

    def add_box(self):
        box = CollisionObject()
        box.header.frame_id = self.wi.world_frame
        box.operation.operation = box.operation.ADD
        box.id = "box"
        
        shape = Shape()
        shape.type = shape.BOX
        shape.dimensions = [0.1, 0.05, 0.15]
        
        pose = Pose()
        pose.position.x = 3.1
        pose.position.y = -3.2
        pose.position.z = FAR_TABLE_HEIGHT + 0.02 + shape.dimensions[2]/2.0
        pose.orientation.w = 1.0
        
        box.shapes.append(shape)
        box.poses.append(pose)

        self.wi.add_object(box)

        return box

    def add_barrier(self):
        box = CollisionObject()
        box.header.frame_id = self.wi.world_frame
        box.operation.operation = box.operation.ADD
        box.id = "barrier1"
        
        shape = Shape()
        shape.type = shape.BOX
        shape.dimensions = [0.1, 2, 1]
        box.shapes.append(shape)
        pose = Pose()
        pose.position.x = 1
        pose.position.y = -1.5
        pose.position.z = 0
        pose.orientation.w = 1.0
        box.poses.append(pose)
        self.wi.add_object(box)

        # if self.world != 12:
        #     box.id = "barrier2"
        #     box.shapes[0].dimensions = [1, 0.1, 1]        
        #     box.poses[0].position.x = 1.5
        #     box.poses[0].position.y = 0
        #     self.wi.add_object(box)

    def get_goal_poses(self):
        place_pose_stamped = PoseStamped()
        place_pose_stamped.header.frame_id = self.wi.world_frame
        
        if abs(self.world) < 0 or (self.world == -5 or abs(self.world) %2 == 0 and 
                                   abs(self.world) != 12 and abs(self.world) != 4): 
            #easiest goal
            x = 1
            y = -2
            z = 0.85
        else:
            #slightly harder goal
            x = -1
            y = 1
            z = 0.85
            
        if abs(self.world) == 12:
            x = 0
            y = -1
            z - 0.85

        if abs(self.world) == 14:
            x = 0
            y = 1
            z = 0.85

        place_pose_stamped.pose.position.x = x
        place_pose_stamped.pose.position.y = y
        place_pose_stamped.pose.position.z = z
        place_pose_stamped.pose.orientation.w = 1.0
        return [place_pose_stamped]

        

    
