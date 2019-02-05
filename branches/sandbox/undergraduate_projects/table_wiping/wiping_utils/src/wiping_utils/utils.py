import roslib; roslib.load_manifest('wiping_utils')
import planning_environment_msgs.srv
import rospy

def currentRobotState():
    state_srv = rospy.ServiceProxy('/environment_server/get_robot_state',
                                   planning_environment_msgs.srv.GetRobotState)
    state_resp = state_srv()
    return state_resp.robot_state
