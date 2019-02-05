import roslib; roslib.load_manifest('darrt_actions')
import pickle
from pr2_python.world_interface import WorldInterface
from pr2_python.planning_scene_interface import get_planning_scene_interface
import sys
from actionlib import SimpleActionClient
from darrt_msgs.msg import DARRTAction
import rospy
from visualization_msgs.msg import MarkerArray
import pr2_python.visualization_tools as vt
import pr2_python.trajectory_tools as tt
from arm_navigation_msgs.msg import RobotState

NTRIALS = 10

def show_trajectory(forever=False):
    pub = rospy.Publisher('darrt_trajectory', MarkerArray)
    [result, goal, collision_objects, robot_state] = pickle.load(open(sys.argv[1], 'r'))
    marray = MarkerArray()
    for t in result.primitive_trajectories:
        marray.markers += vt.trajectory_markers(t, ns='trajectory', resolution=3).markers
    for (i, m) in enumerate(marray.markers):
        m.id = i
        (m.color.r, m.color.g, m.color.b) = vt.hsv_to_rgb(i/float(len(marray.markers))*300.0, 1, 1)
    for i in range(10):
        pub.publish(marray)
        rospy.sleep(0.1)
    if forever:
        while not rospy.is_shutdown():
            pub.publish(marray)
            rospy.sleep(0.1)


def modify_pickle_file():
    [result, goal, collision_objects, robot_state] = pickle.load(open(sys.argv[1], 'r'))
    robot_state = RobotState()
    robot_state.multi_dof_joint_state = tt.multi_dof_trajectory_point_to_multi_dof_state(
        result.primitive_trajectories[0].multi_dof_joint_trajectory.points[0],
        result.primitive_trajectories[0].multi_dof_joint_trajectory.joint_names,
        result.primitive_trajectories[0].multi_dof_joint_trajectory.frame_ids,
        result.primitive_trajectories[0].multi_dof_joint_trajectory.child_frame_ids,
        result.primitive_trajectories[0].multi_dof_joint_trajectory.stamp)
    robot_state.joint_state = tt.joint_trajectory_point_to_joint_state(
        result.primitive_trajectories[0].joint_trajectory.points[0], 
        result.primitive_trajectories[0].joint_trajectory.joint_names)
    pickle.dump([result, goal, collision_objects, robot_state], open(sys.argv[1], 'wb'))


def main():
    [result, goal, collision_objects, robot_state] = pickle.load(open(sys.argv[1], 'r'))
    wi = WorldInterface()
    #set the planning scene up correctly
    wi.reset(repopulate=False)
    for co in collision_objects:
        #for running easy version
        if ('table' in co.id or co.id == 'plate'):
            wi.add_object(co)
    psi = get_planning_scene_interface()
    psi.reset()
    client = SimpleActionClient('/darrt_planning/darrt_action', DARRTAction)
    client.wait_for_server()
    average_time = 0
    goal.planning_time = 30
    goal.tries = 10000
    goal.debug_level = 0
    goal.do_pause = False
    goal.goal_bias = 0.2
    #i think that this does not work
    #goal.robot_state = robot_state
    goal.execute = False

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

rospy.init_node('darrt_reader_and_timer')
#show_trajectory()
modify_pickle_file()
#main()

