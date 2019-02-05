import roslib; roslib.load_manifest('darrt_actions')
from darrt_msgs.msg import DARRTAction, DARRTGoal
from pr2_tasks.tableware_detection import TablewareDetection, find_height_above_table
from pr2_tasks.pickplace_definitions import PlaceGoal
from actionlib import SimpleActionClient
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
import pr2_python.visualization_tools as vt
import pr2_python.geometry_tools as gt
import copy
import rospy
import numpy as np
import object_manipulation_msgs.msg as om
import pr2_python.transform_listener as tl
import pickle

def main():
    client = SimpleActionClient('/darrt_planning/darrt_action', DARRTAction)
    pub = rospy.Publisher('darrt_trajectory', MarkerArray)
    rospy.loginfo('Waiting for action')
    rospy.loginfo('Doing detection')
    detector = TablewareDetection()
    det = detector.detect_objects(add_objects_as_mesh=False)
    if not det.pickup_goals:
        rospy.loginfo('Nothing to pick up!')
        return
    client.wait_for_server()
    
    goal = DARRTGoal()
    
    goal.pickup_goal = det.pickup_goals[0]

    goal.pickup_goal.arm_name = 'right_arm'
    
    place_pose_stamped = PoseStamped()
    place_pose_stamped.header.frame_id = 'map'
    place_pose_stamped.pose.position.x -= 1.3
    place_pose_stamped.pose.position.y -= 0.3
    place_pose_stamped.pose.position.z  = 0.98
    goal.place_goal = PlaceGoal(goal.pickup_goal.arm_name, [place_pose_stamped],
                                collision_support_surface_name = det.table_name,
                                collision_object_name = goal.pickup_goal.collision_object_name)
    goal.primitives = [goal.PICK, goal.PLACE, goal.BASE_TRANSIT]
    goal.min_grasp_distance_from_surface = 0.17
    goal.object_sampling_fraction = 0.9
    goal.retreat_distance = 0.3
    goal.debug_level = 2
    goal.do_pause = False
    goal.planning_time = 6000
    goal.tries = 1
    goal.goal_bias = 0.2
    rospy.loginfo('Sending goal')
    client.send_goal_and_wait(goal)
    rospy.loginfo('Returned')
    result = client.get_result()
    if result.error_code == result.SUCCESS:
        #pickle everything!! great excitement
        filename = 'pickplace_and_objects.pck'
        rospy.loginfo('Pickling to '+filename)
        #the last bit allows us to recreate the planning
        pickle.dump([client.get_result(), goal, wi.collision_objects(), wi.get_robot_state()], open(filename, 'wb'))

    marray = MarkerArray()
    if result.error_code == result.SUCCESS:
        for t in result.primitive_trajectories:
            marray.markers += vt.trajectory_markers(t, ns='trajectory', resolution=3).markers
        for (i, m) in enumerate(marray.markers):
            m.id = i
            (m.color.r, m.color.g, m.color.b) = vt.hsv_to_rgb(i/float(len(marray.markers))*300.0, 1, 1)
        while not rospy.is_shutdown():
            pub.publish(marray)
            rospy.sleep(0.1)

rospy.init_node('pickplace_test_node')
main()
