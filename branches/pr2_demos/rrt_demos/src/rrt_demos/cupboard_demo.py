#!/usr/bin/env python
import roslib
roslib.load_manifest('rrt_demos')
import rospy
import pr2_utils.pickplace_utils
import arm_navigation_msgs.msg
import geometry_msgs.msg
import actionlib_msgs.msg

def locate_soup():
    soup = arm_navigation_msgs.msg.Shape()
    soup.type = soup.CYLINDER
    soup.dimensions = [0.035, 0.1]
    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.pose.position.x = 0.85
    pose_stamped.pose.position.y = 0.15
    pose_stamped.pose.position.z = 0.95
    pose_stamped.pose.orientation.x = 0.0
    pose_stamped.pose.orientation.y = 0.0
    pose_stamped.pose.orientation.z = 0.0
    pose_stamped.pose.orientation.w = 1.0
    pose_stamped.header.frame_id = '/base_link'

    prepose = geometry_msgs.msg.PoseStamped()
    prepose.pose.position.x = 0.525
    prepose.pose.position.y = 0.15
    prepose.pose.position.z = 0.96
    prepose.pose.orientation.x = 0.0
    prepose.pose.orientation.y = 0.0
    prepose.pose.orientation.z = 0.0
    prepose.pose.orientation.w = 1.0
    prepose.header.frame_id = '/base_link'

    postpose = geometry_msgs.msg.PoseStamped()
    postpose.pose.position.x = 0.45
    postpose.pose.position.y = 0.15
    postpose.pose.position.z = 1.0
    postpose.pose.orientation.x = 0.0
    postpose.pose.orientation.y = 0.0
    postpose.pose.orientation.z = 0.0
    postpose.pose.orientation.w = 1.0
    postpose.header.frame_id = '/base_link'


    return soup,prepose,pose_stamped,postpose

def hand_box(pp):
    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.header.frame_id = 'torso_lift_link'
    pose_stamped.pose.position.y = 0.9
    pose_stamped.pose.position.z = 0.1
    pose_stamped.pose.orientation.z = 0.707
    pose_stamped.pose.orientation.w = 0.707
    result, status =\
        pp.arm_control.move_arm_collision_free(pose_stamped, revert_scene=False)
    if result.error_code.val == result.error_code.SUCCESS and\
            status == actionlib_msgs.msg.GoalStatus.SUCCEEDED:
        rospy.sleep(0.5)
        pp.arm_control.open_gripper()

def cupboard_demo():
    pp = pr2_utils.pickplace_utils.PickPlace('left_arm')
    pp.reset_collision_map()
    soup, prepose, pose_stamped, postpose = locate_soup()

    #pp.arm_control.move_arm_to_side()
    #hand_box(pp)
    #return
    picked = pp.pickup_shape(soup, pose_stamped, prepose, postpose)
    if picked:
        print 'pick succeeded!'
        hand_box(pp)
    else:
        print 'pick failed!'

if __name__ == '__main__':
    rospy.init_node('rrt_start_cupboard_demo')
    cupboard_demo()
