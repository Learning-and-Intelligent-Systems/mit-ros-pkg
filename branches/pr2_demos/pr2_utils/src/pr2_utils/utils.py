import roslib; roslib.load_manifest('pr2_utils')
import rospy
import arm_navigation_msgs.msg
import arm_navigation_msgs.srv
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
    armstate = arm_navigation_msgs.msg.RobotState()
    armstate.joint_state.header = copyHeader(currstate.joint_state.header)
    armstate.joint_state.name = info_resp.kinematic_solver_info.joint_names
    for name in info_resp.kinematic_solver_info.joint_names:
        ind = currstate.joint_state.name.index(name)
        armstate.joint_state.position.append\
            (currstate.joint_state.position[ind])
    return armstate


def currentRobotState():
    state_srv = rospy.ServiceProxy('/environment_server/get_robot_state',
                                   arm_navigation_msgs.srv.GetRobotState)
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

def copyPointStamped(origpoint):
    point_stamped = geometry_msgs.msg.PointStamped()
    point_stamped.header = copyHeader(origpoint.header)
    point_stamped.point = copy.copy(origpoint.point)
    return point_stamped

def copyRobotState(state):
    new_state = arm_navigation_msgs.msg.RobotState()
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

def copyConstraints(constraints):
    cc = arm_navigation_msgs.msg.Constraints()
    for j in constraints.joint_constraints:
        cc.joint_constraints.append(copy.copy(j))
    for p in constraints.position_constraints:
        np = arm_navigation_msgs.msg.PositionConstraint()
        np.header = copyHeader(p.header)
        np.link_name = p.link_name
        np.target_point_offset = copy.copy(p.target_point_offset)
        np.position = copy.copy(p.position)
        np.constraint_region_shape.type = p.constraint_region_shape.type
        np.constraint_region_shape.dimensions =\
            copy.copy(p.constraint_region_shape.dimensions)
        np.constraint_region_shape.triangles =\
            copy.copy(p.constraint_region_shape.triangles)
        for v in p.constraint_region_shape.vertices:
            np.constraint_region_shape.vertices.append(copy.copy(v))
        np.constraint_region_orientation =\
            copy.copy(p.constraint_region_orientation)
        np.weight = p.weight
        cc.position_constraints.append(np)
    for o in constraints.orientation_constraints:
        no = arm_navigation_msgs.msg.OrientationConstraint()
        no.header = copyHeader(o.header)
        no.link_name = o.link_name
        no.type = o.type
        no.orientation = copy.copy(o.orientation)
        no.absolute_roll_tolerance = o.absolute_roll_tolerance
        no.absolute_pitch_tolerance = o.absolute_pitch_tolerance
        no.absolute_yaw_tolerance = o.absolute_yaw_tolerance
        no.weight = o.weight
        cc.orientation_constraints.append(no)
    for v in constraints.visibility_constraints:
        nv = arm_navigation_msgs.msg.VisiblityConstraint()
        nv.header = copyHeader(v.header)
        nv.target = copyPointStamped(v.target)
        nv.sensor_pose = copyPoseStamped(v.sensor_pose)
        nv.absolute_tolerance = v.absolute_tolerance
        cc.visbility_constraints.append(nv)
    return cc

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

def wrap_angle(angle):
    while angle > 2.0*math.pi:
        angle -= 2.0*math.pi
    while angle < 0:
        angle += 2.0*math.pi
    return angle

def wrap_angle_pi(angle):
    while angle > math.pi:
        angle -= 2.0*math.pi
    while angle < -math.pi:
        angle += 2.0*math.pi
    return angle

def shortest_angular_distance(a1, a2):
    a1 = wrap_angle(a1)
    a2 = wrap_angle(a2)
    da = wrap_angle(a1 - a2)
    if da > math.pi:
        da = wrap_angle(a2 - a1)
    return da

def multiply_quaternions(q1, q2):
    q = geometry_msgs.msg.Quaternion()
    q.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z
    q.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y
    q.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x
    q.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w
    return q

def conjugate(q):
    cq = geometry_msgs.msg.Quaternion()
    cq.x = -1.0*q.x
    cq.y = -1.0*q.y
    cq.z = -1.0*q.z
    cq.w = q.w
    return cq
