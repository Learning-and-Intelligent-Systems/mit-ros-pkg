#!/usr/bin/env python

import roslib; roslib.load_manifest('table_wiping_demo')
import rospy
import pr2_utils.arm_control
import head_control
import pr2_utils.pressure_listener
import copy
import geometry_msgs.msg
import random
import actionlib_msgs.msg
import std_msgs.msg
import math


X_REACH = 0.65
SECTOR_STRIPE_SIZE = 0.1
MAX_Z = 1.4
INIT_Z_FORCE = -2.0

class ArmWiper:
    def __init__(self, arm_name,
                 manager=None,
                 wipe_quat=geometry_msgs.msg.Quaternion(-0.5, 0.5, 0.5, 0.5)):
        self.arm_name = arm_name
        self.arm_control = pr2_utils.arm_control.ArmControl(self.arm_name,
                                                            manager=manager)
        self.pressure_listener =\
            pr2_utils.pressure_listener.PressureListener(self.arm_name)
        self.wipe_quat = wipe_quat
        
    def findTable(self, initial_wrist_pose, initial_gripper_pose, 
                  currently_touching):
        #first move the hand to above the table using collision aware
        move_arm_succeeded = False
        z = initial_wrist_pose.pose.position.z
        orig_z = z
        self.arm_control.move_arm_force_control(initial_gripper_pose, 4)
        zforce = INIT_Z_FORCE
        if not currently_touching:
            self.pressure_listener.pressure_threshold =\
                self.pressure_listener.avg_pressure + 200
            looprate = rospy.Rate(10)
            while self.pressure_listener.is_touching():
                looprate.sleep()
        rospy.loginfo('Using threshold %f for pressure listener',
                      self.pressure_listener.pressure_threshold)
        
        touching = False
        while not rospy.is_shutdown() and not touching:
            rospy.loginfo('Attempting to lower arm using force %f...', zforce)
            self.arm_control.resetGoal()
            self.arm_control.add_trajectory_point_to_force_control\
                (initial_gripper_pose.pose.position.x,\
                     initial_gripper_pose.pose.position.y,\
                     initial_gripper_pose.pose.position.z - 0.2,\
                     initial_gripper_pose.pose.orientation.x,\
                     initial_gripper_pose.pose.orientation.y,\
                     initial_gripper_pose.pose.orientation.z,\
                     initial_gripper_pose.pose.orientation.w,\
                     1000, 1000, zforce, 30, 30, 30,\
                     False, False, True, False, False, False,\
                     5, 'base_link')
            last_time = rospy.get_time()
            curr_pose = self.arm_control.get_hand_pose(frame_id='/base_link')
            last_z = curr_pose.pose.position.z
            self.arm_control.executeForceControl(wait=False)
            moved = True
            while not rospy.is_shutdown() and\
                    not touching and moved and\
                    curr_pose.pose.position.z > 0.6:
                touching = self.pressure_listener.is_touching()
                curr_time = rospy.get_time()
                curr_pose = self.arm_control.get_hand_pose\
                    (frame_id='/base_link')
                if curr_time - last_time > 0.25:
                    if last_z - curr_pose.pose.position.z < 0.005:
                        moved = False
                    rospy.loginfo('Current height: %f, last height %f', 
                                   curr_pose.pose.position.z, last_z)
                    last_z = curr_pose.pose.position.z
                    last_time = curr_time
            if touching:
                self.arm_control.stop_in_place()
            elif curr_pose.pose.position.z <= 0.6:
                self.arm_control.stop_in_place()
                break
            else:
                zforce -= 0.25
                #otherwise we send new goals too fast and bother the controller?
                rospy.sleep(0.25)
            #lowerthread.join()
        if not touching:
            rospy.logerr('Unable to find tabletop')
            return None
        return zforce
    

def main():
    rospy.init_node('left_arm_wiper_test_node')
    leftarm = ArmWiper('left_arm')
    first_gripper_pose = geometry_msgs.msg.PoseStamped()
    first_gripper_pose.header.frame_id = '/base_link'
    first_gripper_pose.pose.position.x = 0.292315184342
    first_gripper_pose.pose.position.y = 0.112984711883
    first_gripper_pose.pose.position.z = 0.724068450137
    first_gripper_pose.pose.orientation.x = -0.406092459444
    first_gripper_pose.pose.orientation.y = 0.658729066977
    first_gripper_pose.pose.orientation.z = 0.553274864268
    first_gripper_pose.pose.orientation.w = 0.308304809031
    leftarm.arm_control.move_arm_force_control(first_gripper_pose, 4)
    initial_wrist_pose = geometry_msgs.msg.PoseStamped()
    initial_wrist_pose.header.frame_id = '/base_link'
    initial_wrist_pose.pose.position.x = 0.65
    initial_wrist_pose.pose.position.y = 0.0109784746814
    initial_wrist_pose.pose.position.z = 0.9040684501317
    initial_wrist_pose.pose.orientation = leftarm.wipe_quat
    initial_gripper_pose =\
        pr2_utils.utils.copyPoseStamped(initial_wrist_pose)
    initial_gripper_pose.pose.position.z -= 0.18
    while not rospy.is_shutdown():
        zforce = leftarm.findTable(initial_wrist_pose,
                                   initial_gripper_pose, False)
        print 'zforce is', zforce

#attempt at stopping that killed the robot:
# header: 
#   seq: 0
#   stamp: 
#     secs: 0
#     nsecs: 0
#   frame_id: /torso_lift_link
# pose: 
#   position: 
#     x: 0.691478657094
#     y: 0.0166313259398
#     z: -0.419657911235
#   orientation: 
#     x: 0.525077513835
#     y: -0.503464333908
#     z: -0.501052026483
#     w: -0.468790076372


if __name__ == '__main__':
    main()
                       
