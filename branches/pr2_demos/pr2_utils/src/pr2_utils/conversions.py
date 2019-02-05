import roslib; roslib.load_manifest('pr2_utils')
import geometry_msgs.msg
import arm_navigation_msgs.msg
import pr2_utils.utils
import rospy

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
    pconstraint = arm_navigation_msgs.msg.PositionConstraint()
    pconstraint.header = pose_stamped.header
    pconstraint.position = pose_stamped.pose.position
    pconstraint.link_name = link_name
    pconstraint.constraint_region_shape.type =\
        pconstraint.constraint_region_shape.BOX
    for i in range(0,3): 
        pconstraint.constraint_region_shape.dimensions.append(bounds[i])
    pconstraint.constraint_region_orientation.w = 1
    pconstraint.weight = 1.0
    oconstraint = arm_navigation_msgs.msg.OrientationConstraint()
    oconstraint.header = pose_stamped.header
    oconstraint.link_name = link_name
    oconstraint.type = oconstraint.HEADER_FRAME
    oconstraint.orientation = pose_stamped.pose.orientation
    oconstraint.absolute_roll_tolerance = bounds[3]
    oconstraint.absolute_pitch_tolerance = bounds[4]
    oconstraint.absolute_yaw_tolerance = bounds[5]
    oconstraint.weight = 1.0
    return (pconstraint, oconstraint)

def transform_point(new_frame_id, old_frame_id, point, tf_listener):
    point_stamped = geometry_msgs.msg.PointStamped()
    point_stamped.header.stamp = rospy.Time(0)
    point_stamped.header.frame_id = old_frame_id
    point_stamped.point = point
    new_point = tf_listener.transformPoint(new_frame_id, point_stamped)
    return new_point.point

def transform_quaternion(new_frame_id, old_frame_id, quat, tf_listener):
    quat_stamped = geometry_msgs.msg.QuaternionStamped()
    quat_stamped.header.stamp = rospy.Time(0)
    quat_stamped.header.frame_id = old_frame_id
    quat_stamped.quaternion = quat
    new_quat = tf_listener.transformQuaternion(new_frame_id, quat_stamped)
    return new_quat.quaternion

def transform_constraints(frame_id, constraints, tf_listener):
    '''
    Taken from planning_environment::CollisionModels::convertConstraintsGivenNew WorldTransform
    '''
    transformed_constraints = pr2_utils.utils.copyConstraints(constraints)
    for c in transformed_constraints.position_constraints:
        c.position = transform_point(frame_id, c.header.frame_id,
                                     c.position, tf_listener)
        c.constraint_region_orientation =\
            transform_quaternion(frame_id, c.header.frame_id,\
                                     c.constraint_region_orientation,\
                                     tf_listener)
        c.header.frame_id = frame_id
    
    for oc in transformed_constraints.orientation_constraints:
        oc.orientation = transform_quaternion(frame_id, oc.header.frame_id,
                                              oc.orientation, tf_listener)
        oc.header.frame_id = frame_id
    for vc in transformed_constraints.visibility_constraints:
        vc.target = tf_listener.transformPoint(frame_id, vc.target)
    return transformed_constraints

def get_transform(from_frame, to_frame, tf_listener):
    looprate = rospy.Rate(10)
    ntries = 0
    while not rospy.is_shutdown() and ntries < 50:
        try:
            return tf_listener.lookupTransform(from_frame, to_frame,
                                               rospy.Time(0))
        except:
            if ntries % 10 == 0:
                rospy.logwarn('Unable to transform from %s to %s',
                              from_frame, to_frame)
        ntries += 1
        looprate.sleep()
    return None
