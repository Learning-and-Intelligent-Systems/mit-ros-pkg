import roslib; roslib.load_manifest('pr2_utils')
import geometry_msgs.msg
import motion_planning_msgs.msg

def pointToList(point):
    l = []
    l.append(point.x)
    l.append(point.y)
    l.append(point.z)
    return l

def listToPoint(l):
    point = geometry_msgs.Point()
    point.x = l[0]
    point.y = l[1]
    point.z = l[2]
    return point

def TRtoPose(trans, rot):
    pose = geometry_msgs.msg.Pose()
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    pose.orientation.x = rot[0]
    pose.orientation.y = rot[1]
    pose.orientation.z = rot[2]
    pose.orientation.w = rot[3]
    return pose

def TRtoPoseStamped(trans, rot, frame_id):
    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.header.frame_id = frame_id
    pose_stamped.pose = TRtoPose(trans, rot)
    return pose_stamped

def poseStampedToPositionOrientationConstraints\
        (pose_stamped, link_name, bounds=(0.01, 0.01, 0.01, 0.1, 0.1, 0.1)):
    pconstraint = motion_planning_msgs.msg.PositionConstraint()
    pconstraint.header = pose_stamped.header
    pconstraint.position = pose_stamped.pose.position
    pconstraint.link_name = link_name
    pconstraint.constraint_region_shape.type =\
        pconstraint.constraint_region_shape.BOX
    for i in range(0,3): 
        pconstraint.constraint_region_shape.dimensions.append(bounds[i])
    pconstraint.constraint_region_orientation.w = 1
    pconstraint.weight = 1.0
    oconstraint = motion_planning_msgs.msg.OrientationConstraint()
    oconstraint.header = pose_stamped.header
    oconstraint.link_name = link_name
    oconstraint.type = oconstraint.HEADER_FRAME
    oconstraint.orientation = pose_stamped.pose.orientation
    oconstraint.absolute_roll_tolerance = bounds[3]
    oconstraint.absolute_pitch_tolerance = bounds[4]
    oconstraint.absolute_yaw_tolerance = bounds[5]
    oconstraint.weight = 1.0
    return (pconstraint, oconstraint)

