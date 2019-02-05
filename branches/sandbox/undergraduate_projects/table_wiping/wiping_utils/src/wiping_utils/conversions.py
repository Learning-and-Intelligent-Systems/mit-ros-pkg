import roslib; roslib.load_manifest('wiping_utils')
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

def poseStampedToPositionOrientationConstraints(pose_stamped, link_name):
    pconstraint = motion_planning_msgs.msg.PositionConstraint()
    pconstraint.header = pose_stamped.header
    pconstraint.position = pose_stamped.pose.position
    pconstraint.link_name = link_name
    pconstraint.constraint_region_shape.type =\
        pconstraint.constraint_region_shape.BOX
    for i in range(0,3): pconstraint.constraint_region_shape.dimensions.append(0.01)
    pconstraint.constraint_region_orientation.w = 1
    pconstraint.weight = 1.0
    oconstraint = motion_planning_msgs.msg.OrientationConstraint()
    oconstraint.header = pose_stamped.header
    oconstraint.link_name = link_name
    oconstraint.type = oconstraint.HEADER_FRAME
    oconstraint.orientation = pose_stamped.pose.orientation
    oconstraint.absolute_roll_tolerance = 0.01
    oconstraint.absolute_pitch_tolerance = 0.01
    oconstraint.absolute_yaw_tolerance = 0.01
    oconstraint.weight = 1.0
    return (pconstraint, oconstraint)

