#!/usr/bin/env python

import roslib; roslib.load_manifest('wiping_demo')
import rospy
import wiping_control.arm_control
import wiping_sensing.head_control
import copy
import geometry_msgs.msg
import wiping_sensing.pressure_listener
import random
import actionlib_msgs.msg
import math

X_REACH = 0.65
SECTOR_STRIPE_SIZE = 0.1
MAX_Z = 1.4
INIT_Z_FORCE = -2.0

class LeftArmWiper:
    def __init__(self, manager=None,
                 wipe_quat=geometry_msgs.msg.Quaternion(-0.5, 0.5, 0.5, 0.5)):
        self.head_control = wiping_sensing.head_control.HeadController(manager=manager)
        self.manager = self.head_control.manager
        self.arm_control = wiping_control.arm_control.ArmControl('left_arm', self.manager)
        self.pressure_listener =\
            wiping_sensing.pressure_listener.PressureListener()
        self.wipe_quat = wipe_quat
        self.left_edge = None
        self.table = None
        leftinfo = self.head_control.findTableLeftEdge()
        if not leftinfo:
            rospy.logerr('Unable to find left table edge visually')
        else:
            (self.left_edge, self.table) = leftinfo
        
    def findTable(self, initial_wrist_pose, initial_gripper_pose, currently_touching):
        #first move the hand to above the table using collision aware
        move_arm_succeeded = False
        z = initial_wrist_pose.pose.position.z
        orig_z = z
        while not move_arm_succeeded and z < MAX_Z:
            initial_wrist_pose.pose.position.z = z
            rospy.logdebug('Attempting to move to position %s', str(initial_wrist_pose))
            (ma_r, ma_s) = self.arm_control.move_arm_collision_free(initial_wrist_pose)
            move_arm_succeeded = (ma_r.error_code.val == ma_r.error_code.SUCCESS and
                                  ma_s == actionlib_msgs.msg.GoalStatus.SUCCEEDED)
            if ma_r.error_code.val == ma_r.error_code.JOINT_LIMITS_VIOLATED or\
                    (ma_r.error_code.val == ma_r.error_code.SUCCESS and\
                         ma_s != actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                #the starting state caused an abort
                #this will never work
                break
            z += 0.05
        if not move_arm_succeeded:
            rospy.logerr('Unable to move arm collision free.  Using open loop control.')
            initial_wrist_pose.pose.position.z = orig_z
            if not self.arm_control.try_hard_for_pose(initial_wrist_pose, max_tries=0):
                #we may have actually moved using the Cartesian controller
                (trans, rot) = self.arm_control.get_hand_pose(frame_id='/base_link')
                dist = math.sqrt((trans[0] - initial_gripper_pose.pose.position.x)*
                                 (trans[0] - initial_gripper_pose.pose.position.x)+
                                 (trans[1] - initial_gripper_pose.pose.position.y)*
                                 (trans[1] - initial_gripper_pose.pose.position.y)+
                                 (trans[2] - initial_gripper_pose.pose.position.z)*
                                 (trans[2] - initial_gripper_pose.pose.position.z))
                if dist > 0.1:
                    rospy.logerr('Unable to move using open loop.  Using force control to move %f m.', dist)
                    self.arm_control.add_trajectory_point_to_force_control\
                        (initial_gripper_pose.pose.position.x,\
                             initial_gripper_pose.pose.position.y,\
                             initial_gripper_pose.pose.position.z,\
                             initial_gripper_pose.pose.orientation.x,\
                             initial_gripper_pose.pose.orientation.y,\
                             initial_gripper_pose.pose.orientation.z,\
                             initial_gripper_pose.pose.orientation.w,\
                             1000, 1000, 1000, 30, 30, 30,\
                             False, False, False, False, False, False,\
                             3, 'base_link')
                    self.arm_control.executeForceControl()
        zforce = INIT_Z_FORCE
        if not currently_touching and self.pressure_listener.is_touching():
            self.pressure_listener.pressure_threshold =\
                self.pressure_listener.avg_pressure + 200
        rospy.loginfo('Using threshold %f for pressure listener',
                      self.pressure_listener.pressure_threshold)
        touching = False
        while not rospy.is_shutdown() and not touching:
            rospy.loginfo('Attempting to lower arm using force %f...', zforce)
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
            (trans, rot) = self.arm_control.get_hand_pose(frame_id='/base_link')
            last_z = trans[2]
            self.arm_control.executeForceControl(True, False)
            moved = True
            while not rospy.is_shutdown() and\
                    not touching and moved and\
                    trans[2] > self.table.height - 0.1:
                touching = self.pressure_listener.is_touching() 
                curr_time = rospy.get_time()
                (trans, rot) = self.arm_control.get_hand_pose(frame_id='/base_link')
                if curr_time - last_time > 0.25:
                    if last_z - trans[2] < 0.005:
                        moved = False
                    rospy.loginfo('Current height: %f', trans[2])
                    last_z = trans[2]
                    last_time = curr_time
            if touching:
                self.arm_control.stop_in_place()
            elif trans[2] <= self.table.height - 0.1:
                self.arm_control.stop_in_place()
                break
            else:
                zforce -= 0.25
        if not touching:
            rospy.logerr('Unable to find tabletop')
            return None
        return zforce
    
    def wipeSector(self, ymin, ymax, currently_touching):
        y = ymax
        initial_wrist_pose = geometry_msgs.msg.PoseStamped()
        initial_wrist_pose.header.frame_id = 'base_link'
        initial_wrist_pose.header.stamp = rospy.Time(0)
        initial_wrist_pose.pose.position.x = X_REACH
        initial_wrist_pose.pose.position.y = y
        initial_wrist_pose.pose.position.z = self.table.height + 0.15 + 0.18
        initial_wrist_pose.pose.orientation = self.wipe_quat
        initial_gripper_pose = copy.deepcopy(initial_wrist_pose)
        initial_gripper_pose.pose.position.z -= 0.18
        while y > ymin:
            #first move the arm to above the table using collision free if possible
            zforce = None
            while not zforce:
                initial_wrist_pose.pose.position.y = y
                initial_gripper_pose.pose.position.y = y
                zforce = self.findTable(initial_wrist_pose, initial_gripper_pose,
                                        currently_touching)
                if not zforce: y -= 0.03
            #now wipe towards the robot
            at_front_edge = False
            rospy.loginfo('Wiping strip with y = %f', y)
            while not at_front_edge:
                self.arm_control.add_trajectory_point_to_force_control(0, y,
                                                                       self.table.height,
                                                                       self.wipe_quat.x,
                                                                       self.wipe_quat.y,
                                                                       self.wipe_quat.z,
                                                                       self.wipe_quat.w,
                                                                       1000, 1000, zforce,
                                                                       30, 30, 30,
                                                                       False, False, True,
                                                                       False, False, False, 10,
                                                                       '/base_link')
                start_time = rospy.get_time()
                self.arm_control.executeForceControl(True, False)
                curr_time = rospy.get_time()
                (trans, rot) = self.arm_control.get_hand_pose(frame_id='/base_link')
                while not rospy.is_shutdown() and\
                        self.pressure_listener.is_touching() and\
                        curr_time - start_time < 12 and\
                        trans[0] > self.table.xmin - 0.1 and\
                        trans[2] > self.table.height - 0.1:
                    curr_time = rospy.get_time()
                    (trans, rot) = self.arm_control.get_hand_pose(frame_id='/base_link')
                #are we somewhere reasonably near the right edge?
                if trans[0] < self.table.xmin + 0.07:
                    at_front_edge = True
                    #self.arm_control.stop_in_place()
                    #now move just a little towards the robot so that
                    #we have some chance of begin able to use collision
                    #free to move it back above the table
                    self.arm_control.add_trajectory_point_to_force_control(trans[0] - 0.05,
                                                                           trans[1], self.table.height + 0.15,
                                                                           rot[0], rot[1], rot[2],
                                                                           rot[3], 1000, 1000, 1000,
                                                                           30, 30, 30, False, False,
                                                                           False, False, False, False,
                                                                           2, 'base_link')
                    self.arm_control.executeForceControl()
                if trans[2] <= self.table.height - 0.09:
                    #redo this sector
                    rospy.logerr('We slipped off the table, re-doing sector')
                    y += SECTOR_STRIPE_SIZE
                    break
                else:
                    zforce -= 0.5
            currently_touching = False
            y -= SECTOR_STRIPE_SIZE

def main():
    rospy.init_node('left_arm_wiper_test_node')
    leftarm = LeftArmWiper()
    if leftarm.left_edge == None:
        rospy.logerr('Unable to locate table')
        return
    rospy.loginfo('Left edge is %f', leftarm.left_edge)
    leftarm.wipeSector(0, leftarm.left_edge+0.01, False)


if __name__ == '__main__':
    main()
                       
