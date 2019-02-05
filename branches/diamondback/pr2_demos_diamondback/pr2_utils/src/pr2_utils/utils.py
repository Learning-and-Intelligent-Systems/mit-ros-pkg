import roslib; roslib.load_manifest('pr2_utils')
import planning_environment_msgs.srv
import rospy
import motion_planning_msgs.msg
import kinematics_msgs.srv
import geometry_msgs.msg
import std_msgs.msg
import copy
import math


def currentRobotArmState(arm_name):
    '''
    @type arm_name: string
    @param arm_name: 'left_arm' or 'right_arm'
    @return: The current robot state of the arm with the
    links pr2_kinematics likes
    '''
    rospy.wait_for_service('/environment_server/get_robot_state')
    currstate = currentRobotState()
    info_srv = rospy.ServiceProxy('/pr2_'+arm_name+
                                  '_kinematics/get_ik_solver_info',
                                  kinematics_msgs.srv.GetKinematicSolverInfo)
    info_resp = info_srv()
    armstate = motion_planning_msgs.msg.RobotState()
    armstate.joint_state.header = copyHeader(currstate.joint_state.header)
    armstate.joint_state.name = info_resp.kinematic_solver_info.joint_names
    for name in info_resp.kinematic_solver_info.joint_names:
        ind = currstate.joint_state.name.index(name)
        armstate.joint_state.position.append\
            (currstate.joint_state.position[ind])
    return armstate


def currentRobotState():
    state_srv = rospy.ServiceProxy('/environment_server/get_robot_state',
                                   planning_environment_msgs.srv.GetRobotState)
    state_resp = state_srv()
    return state_resp.robot_state


def copyHeader(origheader):
    header = std_msgs.msg.Header()
    header.seq = origheader.seq
    header.stamp = copy.copy(origheader.stamp)
    header.frame_id = origheader.frame_id
    return header

def copyPose(origpose):
    pose = geometry_msgs.msg.Pose()
    pose.position = copy.copy(origpose.position)
    pose.orientation = copy.copy(origpose.orientation)
    return pose

def copyPoseStamped(origpose):
    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.header = copyHeader(origpose.header)
    pose_stamped.pose = copyPose(origpose.pose)
    return pose_stamped

def copyRobotState(state):
    new_state = motion_planning_msgs.msg.RobotState()
    new_state.joint_state.header = copyHeader(state.joint_state.header)
    new_state.joint_state.name = list(copy.copy(state.joint_state.name))
    new_state.joint_state.position = list(copy.copy(state.joint_state.position))
    new_state.joint_state.velocity = list(copy.copy(state.joint_state.velocity))
    new_state.joint_state.effort = list(copy.copy(state.joint_state.effort))
    new_state.multi_dof_joint_state.stamp =\
        state.multi_dof_joint_state.stamp
    new_state.multi_dof_joint_state.joint_names =\
        list(copy.copy(state.multi_dof_joint_state.joint_names))
    new_state.multi_dof_joint_state.frame_ids =\
        list(copy.copy(state.multi_dof_joint_state.frame_ids))
    new_state.multi_dof_joint_state.child_frame_ids =\
        list(copy.copy(state.multi_dof_joint_state.child_frame_ids)    )
    for p in state.multi_dof_joint_state.poses:
        newp = geometry_msgs.Pose()
        newp.position = copy.copy(p.position)
        newp.orientation = copy.copy(p.orientation)
        new_state.multi_dof_joint_state.poses.append(newp)
    return new_state

def transDist(pose_stamped1, pose_stamped2, listener=None):
    ps1 = pose_stamped1.pose.position
    ps2 = pose_stamped2.pose.position
    if pose_stamped1.header.frame_id !=\
            pose_stamped2.header.frame_id:
        if listener:
            tps = listener.transformPoseStamped\
                (pose_stamped2.header.frame_id, pose_stamped1)
            ps1 = tps.pose.position
        else:
            return None
    return math.sqrt((ps1.x - ps2.x)*
                     (ps1.x - ps2.x)+
                     (ps1.y - ps2.y)*
                     (ps1.y - ps2.y)+
                     (ps1.z - ps2.z)*
                     (ps1.z - ps2.z))
