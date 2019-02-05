#!/usr/bin/env/python
import roslib; roslib.load_manifest('darrt_actions')

from darrt_msgs.msg import DARRTAction, DARRTGoal
from darrt_actions.darrt_trajectories import get_trajectory_markers, Executor

from pr2_python.world_interface import WorldInterface
from pr2_python.planning_scene_interface import get_planning_scene_interface
from pr2_tasks.tableware_detection import TablewareDetection
from pr2_tasks.pickplace_definitions import PlaceGoal

from actionlib import SimpleActionClient
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
import rospy

def main():

    #Find an object to pick up
    psi = get_planning_scene_interface()
    psi.reset()
    rospy.loginfo('Doing detection')
    detector = TablewareDetection()
    det = detector.detect_objects(add_objects_as_mesh=False)
    if not det.pickup_goals:
        rospy.loginfo('Nothing to pick up!')
        return
    psi.reset()
    #DARRT action client
    client = SimpleActionClient('/darrt_planning/darrt_action', DARRTAction)
    rospy.loginfo('Waiting for DARRT action')
    client.wait_for_server()
    rospy.loginfo('Found DARRT action')

    #DARRTGoal
    goal = DARRTGoal()
    
    #pickup goal (from the tabletop detection)
    goal.pickup_goal = det.pickup_goals[0]
    goal.pickup_goal.arm_name = 'right_arm'
    
    #world interface to tell us whether we're using the 
    #map or odom_combined frame
    wi = WorldInterface() 

    #place goal (move far away)
    place_pose_stamped = PoseStamped()
    place_pose_stamped.header.frame_id = wi.world_frame
    place_pose_stamped.pose.position.x = -2
    place_pose_stamped.pose.position.y = 0
    place_pose_stamped.pose.position.z  = 0.85
    place_pose_stamped.pose.orientation.w = 1.0
    goal.place_goal = PlaceGoal(goal.pickup_goal.arm_name, 
                                [place_pose_stamped],
                                collision_support_surface_name = 
                                det.table_name,
                                collision_object_name = 
                                goal.pickup_goal.collision_object_name)

    goal.primitives = [goal.PICK, goal.PLACE, goal.BASE_TRANSIT]
    goal.planning_time = 30
    goal.tries = 3
    goal.debug_level = 1
    goal.object_sampling_fraction = 0.5

    #Send the goal to the action
    rospy.loginfo('Sending goal')
    client.send_goal_and_wait(goal)
    rospy.loginfo('Returned')

    #Check the result
    result = client.get_result()
    if result.error_code != result.SUCCESS:
        rospy.logerr('Planning failed.  Error code: '+str(result.error_code))
        return
    rospy.loginfo('Planning succeeded!')

    #at this point the planning is done.  
    #now we execute and visualize the plan

    #visualize trajectory
    pub = rospy.Publisher('darrt_trajectory', MarkerArray)
    marray = get_trajectory_markers(result.primitive_trajectories)
    for i in range(10):
        pub.publish(marray)
        rospy.sleep(0.05)


    #Executor is a Python class for executing DARRT plans
    executor = Executor()

    #execute trajectory
    rospy.loginfo('Press enter to execute')
    raw_input()
    executor.execute_trajectories(result)
    rospy.loginfo('Successfully executed!')

rospy.init_node('pickplace_test_node')
main()
