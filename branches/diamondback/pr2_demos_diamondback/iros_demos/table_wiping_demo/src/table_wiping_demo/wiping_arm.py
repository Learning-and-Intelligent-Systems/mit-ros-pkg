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
MAX_Z_FORCE = 8

class ArmWiper:
    def __init__(self, arm_name, strip_topic_name='current_strip_wiped',
                 manager=None,
                 wipe_quat=geometry_msgs.msg.Quaternion(-0.5, 0.5, 0.5, 0.5)):
        self.arm_name = arm_name
        self.head_control = head_control.HeadController(manager=manager)
        self.manager = self.head_control.manager
        self.arm_control = pr2_utils.arm_control.ArmControl(self.arm_name,
                                                            self.manager)
        self.pressure_listener =\
            pr2_utils.pressure_listener.PressureListener(self.arm_name)
        self.wipe_quat = wipe_quat
        self.edge = None
        self.table = None
        info = self.head_control.findTableEdge(self.arm_name[0])
        if not info:
            rospy.logerr('Unable to find table edge visually')
        else:
            (self.edge, self.table) = info
        self.strip_topic_name = strip_topic_name
        self.strip_pub = rospy.Publisher(self.strip_topic_name, 
                                         std_msgs.msg.Float32)
        
    def findTableEdge(self):
        info = self.head_control.findTableEdge(self.arm_name[0])
        if not info:
            rospy.logerr('Unable to find table edge visually')
            self.edge = None
            self.table = None
            return False
        (self.edge, self.table) = info
        return True

    def findTable(self, initial_wrist_pose, initial_gripper_pose, 
                  currently_touching):
        #first move the hand to above the table using collision aware
        move_arm_succeeded = False
        z = initial_wrist_pose.pose.position.z
        orig_z = z
        while not move_arm_succeeded and z < MAX_Z:
            initial_wrist_pose.pose.position.z = z
            rospy.loginfo('Attempting to move to position %s', 
                           str(initial_wrist_pose))
            (ma_r, ma_s) = self.arm_control.move_arm_collision_free\
                (initial_wrist_pose)
            move_arm_succeeded = (ma_r.error_code.val == 
                                  ma_r.error_code.SUCCESS and
                                  ma_s == 
                                  actionlib_msgs.msg.GoalStatus.SUCCEEDED)
            z += 0.05
        if not move_arm_succeeded:
            rospy.logerr('Unable to move arm collision free.  Using open loop control.')
            initial_wrist_pose.pose.position.z = orig_z
            if not self.arm_control.try_hard_for_pose(initial_wrist_pose):
                #we may have actually moved using the force controller
                curr_pose = self.arm_control.get_hand_pose\
                    (frame_id=initial_gripper_pose.header.frame_id)
                dist = pr2_utils.utils.transDist(curr_pose,
                                                 initial_gripper_pose)
                if dist > 0.15:
                    rospy.logerr('Unable to move to initial position')
                    return None
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
                     1000, 1000, zforce, 100, 100, 100,\
                     False, False, True, False, False, False,\
                     4, 'base_link')
            last_time = rospy.get_time()
            curr_pose = self.arm_control.get_hand_pose(frame_id='/base_link')
            last_z = curr_pose.pose.position.z
            self.arm_control.executeForceControl(wait=False)
            moved = True
            while not rospy.is_shutdown() and\
                    not touching and moved and\
                    curr_pose.pose.position.z > self.table.height - 0.1:
                touching = self.pressure_listener.is_touching()
                curr_time = rospy.get_time()
                curr_pose = self.arm_control.get_hand_pose\
                    (frame_id='/base_link')
                if curr_time - last_time > 0.25:
                    if last_z - curr_pose.pose.position.z < 0.005:
                        moved = False
                    rospy.loginfo('Current height: %f', 
                                   curr_pose.pose.position.z)
                    last_z = curr_pose.pose.position.z
                    last_time = curr_time
            if touching:
                self.arm_control.stop_in_place()
            elif curr_pose.pose.position.z <= self.table.height - 0.1:
                self.arm_control.stop_in_place()
                break
            elif abs(zforce) < MAX_Z_FORCE:
                zforce -= 0.25
                #rospy.sleep(0.25) #otherwise robot mccrashy :(
            #lowerthread.join()
        if not touching:
            rospy.logerr('Unable to find tabletop')
            return None
        return zforce
    
    def wipeSector(self, ymin, ymax, currently_touching, 
                   max_wipes=float('inf')):
        if self.arm_name[0] == 'l':
            sign = -1.0
            y = ymax
        else:
            sign = 1.0
            y = ymin
        initial_wrist_pose = geometry_msgs.msg.PoseStamped()
        initial_wrist_pose.header.frame_id = 'base_link'
        initial_wrist_pose.header.stamp = rospy.Time(0)
        initial_wrist_pose.pose.position.x = X_REACH
        initial_wrist_pose.pose.position.y = y
        initial_wrist_pose.pose.position.z = self.table.height + 0.1 + 0.18
        initial_wrist_pose.pose.orientation = self.wipe_quat
        initial_gripper_pose =\
            pr2_utils.utils.copyPoseStamped(initial_wrist_pose)
        initial_gripper_pose.pose.position.z -= 0.18
        nwipes = 0
        while ((self.arm_name[0] == 'l' and y > ymin) or\
                   (self.arm_name[0] == 'r' and y < ymax)) and\
                   nwipes < max_wipes and not rospy.is_shutdown():
            zforce = None
            ntries = 0
            rospy.loginfo('Wiping strip with y = %f', y)
            self.strip_pub.publish(y)
            while not zforce and not rospy.is_shutdown() and\
                    ntries < 15:
                initial_wrist_pose.pose.position.y = y
                initial_gripper_pose.pose.position.y = y
                zforce = self.findTable(initial_wrist_pose, 
                                        initial_gripper_pose,
                                        currently_touching)
                if not zforce: 
                    y += sign*0.03
                ntries += 1
            if not zforce:
                rospy.logerr('Unable to wipe full sector')
                return
            #now wipe towards the robot
            at_front_edge = False
            zforce -= 1.0
            curr_table_height = None
            while not at_front_edge:
                self.arm_control.resetGoal()
                self.arm_control.add_trajectory_point_to_force_control\
                    (0, y, self.table.height, self.wipe_quat.x,\
                         self.wipe_quat.y, self.wipe_quat.z, self.wipe_quat.w,\
                         1000, 1000, zforce, 100, 100, 100, False, False, True,\
                         False, False, False, 5, '/base_link')
                start_time = rospy.get_time()
                self.arm_control.executeForceControl(reset=True, wait=False)
                curr_time = rospy.get_time()
                curr_pose =\
                    self.arm_control.get_hand_pose(frame_id='/base_link')
                if self.pressure_listener.is_touching():
                    curr_table_height = curr_pose.pose.position.z
                    rospy.loginfo('curr_table_height = %f', curr_table_height)
                while not rospy.is_shutdown() and\
                        self.pressure_listener.is_touching() and\
                        curr_time - start_time < 12 and\
                        curr_pose.pose.position.x > self.table.xmin - 0.1 and\
                        curr_pose.pose.position.z > self.table.height - 0.1 and\
                        (not curr_table_height or\
                             curr_pose.pose.position.z >=\
                             curr_table_height - 0.01):
                    curr_time = rospy.get_time()
                    curr_pose =\
                        self.arm_control.get_hand_pose(frame_id='/base_link')
                #are we somewhere reasonably near the front edge?
                if curr_pose.pose.position.x < self.table.xmin + 0.07:
                    at_front_edge = True
                    #self.arm_control.stop_in_place()
                    #now move just a little towards the robot so that
                    #we have some chance of begin able to use collision
                    #free to move it back above the table
                    self.arm_control.resetGoal()
                    newpose = pr2_utils.utils.copyPoseStamped(curr_pose)
                    newpose.pose.position.x -= 0.05
                    newpose.pose.position.z = self.table.height + 0.1
                    print 'TRYING TO MOVE UP!'
                    self.arm_control.move_arm_force_control(newpose, 2)
                elif curr_pose.pose.position.z <= self.table.height - 0.09:
                    #redo this sector
                    rospy.logerr('We slipped off the table, re-doing sector')
                    y -= sign*SECTOR_STRIPE_SIZE
                    break
                else:
                    if abs(zforce) < MAX_Z_FORCE: 
                        zforce -= 0.25
            currently_touching = False
            y += sign*SECTOR_STRIPE_SIZE
            nwipes += 1

def lowering_test():
    leftarm = ArmWiper('left_arm')
    initial_wrist_pose = geometry_msgs.msg.PoseStamped()
    initial_wrist_pose.header.frame_id = '/base_link'
    initial_wrist_pose.pose.position.x = X_REACH
    initial_wrist_pose.pose.position.y = leftarm.edge - 0.1
    initial_wrist_pose.pose.position.z = leftarm.table.height + 0.1 + 0.18
    initial_wrist_pose.pose.orientation = leftarm.wipe_quat
    initial_gripper_pose =\
        pr2_utils.utils.copyPoseStamped(initial_wrist_pose)
    initial_gripper_pose.pose.position.z -= 0.18
    zforce = leftarm.findTable(initial_wrist_pose,
                               initial_gripper_pose, False)
    print 'zforce is', zforce


def main():
    leftarm = ArmWiper('left_arm')
    if leftarm.edge == None:
        rospy.logerr('Unable to locate table')
        return
    rospy.loginfo('Left edge is %f', leftarm.edge)
    leftarm.wipeSector(-0.1, leftarm.edge-0.1, False)


if __name__ == '__main__':
    rospy.init_node('left_arm_wiper_test_node')
    main()
                       
