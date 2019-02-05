import roslib; roslib.load_manifest('darrt_actions')
from darrt_msgs.msg import DARRTAction, DARRTGoal
from pr2_tasks.tableware_detection import TablewareDetection, find_height_above_table
from pr2_tasks.pickplace_definitions import PlaceGoal
from actionlib import SimpleActionClient
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion
import pr2_python.visualization_tools as vt
import pr2_python.geometry_tools as gt
import copy
import rospy
import numpy as np
import object_manipulation_msgs.msg as om
import pr2_python.transform_listener as tl
from pr2_python.world_interface import WorldInterface
from pr2_python.planning_scene_interface import get_planning_scene_interface
from arm_navigation_msgs.msg import Shape, CollisionObject
from tabletop_object_detector.msg import Table
from pr2_tasks.pickplace_definitions import PickupGoal
import pickle
import sys

NPLATEGRASPS=200
GRASP_DIST=0.3

def add_map_tables(wi):
    (serving_table, dirty_table) = [add_map_table('serving_table', wi), add_map_table('dirty_table', wi)]
    #little_table = add_map_table('little_table', wi)
    st_base = CollisionObject()
    st_base.header = serving_table.pose.header
    st_base.id = "serving_table_base"
    st_base.poses.append(copy.deepcopy(serving_table.pose.pose))
    st_base.poses[0].position.z = 0.1
    st_shape = Shape()
    st_shape.type = Shape.CYLINDER
    st_shape.dimensions = [0.3, 0.1]
    st_base.shapes.append(st_shape)
    wi.add_object(st_base)
    #lt_base = CollisionObject()
    #lt_base.header = little_table.pose.header
    #lt_base.id = 'little_table_base'
    #lt_base.poses.append(copy.deepcopy(little_table.pose.pose))
    #lt_base.poses[0].position.z = 0.05
    #lt_shape = Shape()
    #lt_shape.type = Shape.CYLINDER
    #lt_shape.dimensions = [0.2, 0.1]
    #lt_base.shapes.append(lt_shape)
    #wi.add_object(lt_base)
    #return (serving_table, dirty_table, little_table)
    

def add_map_table(name, wi):
    table = rospy.get_param('wg-sushi-tables/'+name)
    upper_left = Point(x = table['upper_left']['x'], y = table['upper_left']['y'], 
                       z = table['upper_left']['z'])
    lower_left = Point(x = table['lower_left']['x'], y = table['lower_left']['y'], 
                       z = table['lower_left']['z'])
    upper_right = Point(x = table['upper_right']['x'], y = table['upper_right']['y'], 
                       z = table['upper_right']['z'])
    lower_right = Point(x = table['lower_right']['x'], y = table['lower_right']['y'], 
                       z = table['lower_right']['z'])
    table_object = CollisionObject()
    table_object.header.frame_id = wi.world_frame
    table_object.id = name
    table_pose = Pose()
    table_pose.position.x = (upper_left.x+upper_right.x+lower_left.x+lower_right.x)/4.0
    table_pose.position.y = (upper_left.y+upper_right.y+lower_left.y+lower_right.y)/4.0
    table_pose.position.z = (upper_left.z+upper_right.z+lower_left.z+lower_right.z)/4.0
    table_pose.orientation.w = 1.0
    table_shape = Shape()
    table_shape.type = Shape.MESH
    table_shape.vertices = [gt.inverse_transform_point(lower_left, table_pose), 
                                    gt.inverse_transform_point(upper_left, table_pose), 
                                    gt.inverse_transform_point(upper_right, table_pose), 
                                    gt.inverse_transform_point(lower_right, table_pose)]
    table_shape.triangles = [0, 1, 2, 2, 3, 0]
    table_object.shapes.append(table_shape)
    table_upper_pose = copy.deepcopy(table_pose)
    table_upper_pose.position.z += 0.02
    table_object.poses.append(table_pose)
    table_object.poses.append(table_upper_pose)
    table_object.shapes.append(table_shape)
    table_lower_pose = copy.deepcopy(table_pose)
    table_lower_pose.position.z -= 0.02
    table_object.poses.append(table_lower_pose)
    table_object.shapes.append(table_shape)
    if name == 'dirty_table':
        #kind of a hack =D
        for i in range(5):
            table_lower_pose = copy.deepcopy(table_lower_pose)
            table_lower_pose.position.z -= 0.02
            table_object.poses.append(table_lower_pose)
            table_object.shapes.append(table_shape)
    wi.add_object(table_object)
    table = Table()
    table.pose.header = table_object.header
    table.pose.pose = table_object.poses[0]
    table.convex_hull = table_object.shapes[0]
    return table
                        

def plate_grasps(plate_pose_stamped, grasp_reference_frame):
    plate_pose = tl.transform_pose(grasp_reference_frame, plate_pose_stamped.header.frame_id, plate_pose_stamped.pose)
    #this is the object in the frame of the gripper
    grasp_pose = Pose()
    grasps = []
    rospy.loginfo('plate pose = \n'+str(plate_pose))
    origin = Pose()
    origin.orientation.w = 1.0;
    grasp_pose.position.x = GRASP_DIST;
    for i in range(NPLATEGRASPS):
        alpha = 2.0*np.pi/NPLATEGRASPS*i
        grasp_pose.orientation = gt.multiply_quaternions(Quaternion(x=0.707, w=0.707),
                                                         Quaternion(z = np.sin(alpha/2.0), w = np.cos(alpha/2.0)))
        grasp = om.Grasp()

        grasp.grasp_pose = gt.transform_pose(gt.inverse_transform_pose(origin, grasp_pose), plate_pose)
        rospy.loginfo('grasp pose =\n'+str(grasp.grasp_pose))
        grasp.min_approach_distance = 0.05
        grasp.desired_approach_distance = 0.3
        grasps.append(grasp)

    return grasps
        

def get_plate(pickup_goals, table_pose, wi):
    goal = None
    for pg in pickup_goals:
        if pg.label == 'plate':
            goal = pg
            break
    if not goal:
        for pg in pickup_goals:
            if pg.label == 'graspable' and find_height_above_table(pg.object_pose_stamped, table_pose) < 0.03:
                pg.label = 'plate'
                goal = pg
                break
    if not goal: return None
    #add this as a cylinder instead
    rospy.loginfo('Plate is '+goal.collision_object_name)

    plate = wi.collision_object(goal.collision_object_name) 
    wi.remove_collision_object(goal.collision_object_name)
    plate.id = 'plate'
    plate.poses = [plate.poses[0]]
    plate.shapes = [plate.shapes[0]]
    plate.shapes[0].type = Shape.CYLINDER
    plate.shapes[0].dimensions = [(plate.shapes[0].dimensions[0]+plate.shapes[0].dimensions[1])/4.0, 
                                  plate.shapes[0].dimensions[2]]
    wi.add_object(plate)
    goal.collision_object_name = 'plate'
    return goal
    


def plate_main():
    pub = rospy.Publisher('darrt_trajectory', MarkerArray)
    rospy.loginfo('Waiting for action')
    rospy.loginfo('Doing detection')
    wi = WorldInterface()
    psi = get_planning_scene_interface()
    detector = TablewareDetection()
    plate = wi.collision_object('plate')
    good_detection = False
    goal = DARRTGoal()
    if plate:
        rospy.loginfo('Use last detection?')
        good_detection = (raw_input() == 'y')
        ops = PoseStamped()
        ops.header = copy.deepcopy(plate.header)
        ops.pose = copy.deepcopy(plate.poses[0])
        goal.pickup_goal = PickupGoal('right_arm', om.GraspableObject(reference_frame_id=ops.header.frame_id), 
                                      object_pose_stamped=ops, collision_object_name='plate',
                                      collision_support_surface_name='serving_table')
    while not good_detection:
        det = detector.detect_objects(add_table_to_map=False, 
                                      add_objects_as_mesh=False, table_name='serving_table')
        goal.pickup_goal = get_plate(det.pickup_goals, det.table.pose, wi)
        psi.reset()
        if not goal.pickup_goal:
            rospy.loginfo('Nothing to pick up!')
        rospy.loginfo('Good detection?')
        good_detection = (raw_input() == 'y')
    if not goal.pickup_goal:
        rospy.loginfo('Nothing to pick up!')
        return
    add_map_tables(wi)
    psi.reset()
    client = SimpleActionClient('/darrt_planning/darrt_action', DARRTAction)
    client.wait_for_server()

    goal.pickup_goal.arm_name = 'right_arm'
    goal.pickup_goal.desired_grasps = plate_grasps(goal.pickup_goal.object_pose_stamped, 
                                                   goal.pickup_goal.target.reference_frame_id)
    goal.pickup_goal.lift.desired_distance = 0.3
    place_pose_stamped = PoseStamped()
    place_pose_stamped.header.frame_id = wi.world_frame 
    # place_pose_stamped.pose.position.x = 1.3
    # place_pose_stamped.pose.position.y = -0.3
    # place_pose_stamped.pose.position.z = 1.01
    place_pose_stamped.pose.position.x = 0.0
    place_pose_stamped.pose.position.y = 0.0
    place_pose_stamped.pose.position.z = 0.76
    place_pose_stamped.pose.orientation.w = 1.0
    # place_pose_stamped.pose.orientation.x = 0.1
    # place_pose_stamped.pose.orientation.y = 0.1
    # place_pose_stamped.pose.orientation.w = np.sqrt(0.98)
    #place_pose_stamped = copy.deepcopy(goal.pickup_goal.object_pose_stamped)
    #place_pose_stamped.pose.position.x -= 0.2
    #place_pose_stamped.pose.position.y -= 0.2
    #place_pose_stamped.pose.position.z += 0.2
    goal.place_goal = PlaceGoal(goal.pickup_goal.arm_name, [place_pose_stamped],
                                collision_support_surface_name = 'dirty_table',
                                collision_object_name = goal.pickup_goal.collision_object_name)
    goal.place_goal.approach.direction.header.frame_id = wi.world_frame
    goal.place_goal.approach.direction.vector.x = np.sqrt(0.18)
    goal.place_goal.approach.direction.vector.y = -np.sqrt(0.18)
    goal.place_goal.approach.direction.vector.z = -0.8
    goal.place_goal.approach.desired_distance = 0.2
    goal.primitives = [goal.PICK, goal.PLACE, goal.PUSH, goal.BASE_TRANSIT]
    goal.min_grasp_distance_from_surface = 0.19
    goal.object_sampling_fraction = 0.7
    goal.retreat_distance = 0.3
    goal.debug_level = 2
    goal.do_pause = False
    goal.execute = False
    goal.planning_time = 6000
    goal.tries = 1
    goal.goal_bias = 0.2
    rospy.loginfo('Sending goal')
    client.send_goal_and_wait(goal)
    rospy.loginfo('Returned')
    result = client.get_result()

    marray = MarkerArray()
    if result.error_code == result.SUCCESS:
        #pickle everything!! great excitement
        filename = 'trajectory_and_objects.pck'
        rospy.loginfo('Pickling to '+filename)
        #the last bit allows us to recreate the planning
        pickle.dump([client.get_result(), goal, wi.collision_objects(), wi.get_robot_state()], open(filename, 'wb'))
        rospy.loginfo('Successful write')
        for t in result.primitive_trajectories:
            marray.markers += vt.trajectory_markers(t, ns='trajectory', resolution=3).markers
        for (i, m) in enumerate(marray.markers):
            m.id = i
            (m.color.r, m.color.g, m.color.b) = vt.hsv_to_rgb(i/float(len(marray.markers))*300.0, 1, 1)
        while not rospy.is_shutdown():
            pub.publish(marray)
            rospy.sleep(0.1)

NTRIALS = 1

def pickplace_main():

    rospy.loginfo('Waiting for action')
    rospy.loginfo('Doing detection')
    detector = TablewareDetection()
    det = detector.detect_objects(add_objects_as_mesh=False)
    if not det.pickup_goals:
        rospy.loginfo('Nothing to pick up!')
        return
    wi = WorldInterface()
    psi = get_planning_scene_interface()
    add_map_tables(wi)
    psi.reset()
    client = SimpleActionClient('/darrt_planning/darrt_action', DARRTAction)
    client.wait_for_server()
    
    goal = DARRTGoal()
    
    goal.pickup_goal = det.pickup_goals[0]
    goal.pickup_goal.lift_distance = 0.3
    goal.pickup_goal.arm_name = 'right_arm'
    
    place_pose_stamped = PoseStamped()
    place_pose_stamped.header.frame_id = wi.world_frame
    place_pose_stamped.pose.position.x = -1.3
    place_pose_stamped.pose.position.y = 0.7
    if 'graspable' in goal.pickup_goal.collision_object_name:
        place_pose_stamped.pose.position.z = 0.8
    else:
        place_pose_stamped.pose.position.z = 0.75
    place_pose_stamped.pose.orientation.w = 1.0
    goal.place_goal = PlaceGoal(goal.pickup_goal.arm_name, [place_pose_stamped],
                                collision_support_surface_name = 'serving_table',
                                collision_object_name = goal.pickup_goal.collision_object_name)
    goal.primitives = [goal.PICK, goal.PLACE, goal.BASE_TRANSIT]
    goal.object_sampling_fraction = 0.3
    goal.retreat_distance = 0.3
    goal.debug_level = 2
    goal.do_pause = False
    goal.execute = False
    goal.planning_time = 30
    goal.tries = 10
    goal.goal_bias = 0.2

    average_time = 0.0
    rospy.loginfo('First place goal is\n'+str(goal.place_goal.place_locations[0]))
    rospy.loginfo('Restart after '+str(goal.planning_time))
    for i in range(NTRIALS):
        rospy.loginfo('Sending goal')
        client.send_goal_and_wait(goal)
        rospy.loginfo('Returned')
        result = client.get_result()
        if result.error_code != result.SUCCESS:
            rospy.logerr('Something went wrong!  Error '+str(result.error_code)+'.  We had '+str(i)+
                         ' successful trials with an average time of '+str(average_time/float(i)))
            return
        average_time += result.planning_time
        rospy.loginfo('TRIAL '+str(i)+': '+str(result.planning_time))
    rospy.loginfo(str(NTRIALS)+' averaged '+str(average_time/float(NTRIALS))+' seconds to solve')


    # rospy.loginfo('Sending goal')
    # client.send_goal_and_wait(goal)
    # rospy.loginfo('Returned')
    # result = client.get_result()
    # if result.error_code == result.SUCCESS:
    #     #pickle everything!! great excitement
    #     filename = 'pickplace_and_objects.pck'
    #     rospy.loginfo('Pickling to '+filename)
    #     #the last bit allows us to recreate the planning
    #     pickle.dump([client.get_result(), goal, wi.collision_objects(), wi.get_robot_state()], open(filename, 'wb'))


def time_plate_main():
    
    rospy.loginfo('Reading file')
    [result, goal, collision_objects, robot_state] = pickle.load(open(sys.argv[1], 'r'))
    rospy.loginfo('Original starting state was\n'+str(robot_state))
    markers = vt.robot_marker(robot_state, ns='robot_starting_state', a = 0.5)
    pub = rospy.Publisher('/darrt_planning/robot_state_markers', MarkerArray)
    for i in range(10):
        rospy.loginfo('Publishing starting state!')
        pub.publish(markers)
        rospy.sleep(0.1)
    wi = WorldInterface()
    wi.reset(repopulate=False)
    psi = get_planning_scene_interface()
    psi.reset()
    plate = None
    for co in collision_objects:
        wi.add_object(co)
        if (co.id == goal.pickup_goal.collision_object_name):
            plate = co
    psi.reset()
    ops = PoseStamped()
    ops.header = copy.deepcopy(plate.header)
    ops.pose = copy.deepcopy(plate.poses[0])
    #the old pickup goal may have used base link to generate grasps
    #i don't know what we do with things for which we need a point cloud
    goal.pickup_goal = PickupGoal('right_arm', om.GraspableObject(reference_frame_id=ops.header.frame_id), 
                                  object_pose_stamped=ops, collision_object_name=goal.pickup_goal.collision_object_name,
                                  collision_support_surface_name=goal.pickup_goal.collision_support_surface_name,
                                  desired_grasps=plate_grasps(ops, ops.header.frame_id))
    rospy.loginfo('Teleop to initial state.  Ready to go?')
    raw_input()

    client = SimpleActionClient('/darrt_planning/darrt_action', DARRTAction)
    client.wait_for_server()
    goal.execute = False
    goal.planning_time = 200
    goal.tries = 10000
    goal.debug_level = 1
    goal.do_pause = False
    goal.goal_bias = 0.2
#    goal.place_goal.place_locations[0].pose.position.x = 1
 #   goal.place_goal.place_locations[0].pose.position.y = 1
  #  goal.place_goal.place_locations[0].pose.position.z = 0.86
    
    average_time = 0.0
    rospy.loginfo('First place goal is\n'+str(goal.place_goal.place_locations[0]))
    rospy.loginfo('File: '+sys.argv[1]+', restart after '+str(goal.planning_time))
    for i in range(NTRIALS):
        rospy.loginfo('Sending goal')
        client.send_goal_and_wait(goal)
        rospy.loginfo('Returned')
        result = client.get_result()
        if result.error_code != result.SUCCESS:
            rospy.logerr('Something went wrong!  Error '+str(result.error_code)+'.  We had '+str(i)+
                         ' successful trials with an average time of '+str(average_time/float(i)))
            return
        average_time += result.planning_time
        rospy.loginfo('TRIAL '+str(i)+': '+str(result.planning_time))
    rospy.loginfo(str(NTRIALS)+' averaged '+str(average_time/float(NTRIALS))+' seconds to solve')




rospy.init_node('darrt_test_node')
time_plate_main()
#pickplace_main()
