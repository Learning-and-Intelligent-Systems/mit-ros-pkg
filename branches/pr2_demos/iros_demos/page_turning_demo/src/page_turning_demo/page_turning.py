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
import math

class PageTurner:
    def __init__(self, arm_name, manager=None):
        rospy.loginfo("IN PAGETURNER INIT")
        self.arm_name = arm_name
        self.head_control = head_control.HeadController(manager=manager)
        self.manager = self.head_control.manager
        self.arm_control = pr2_utils.arm_control.ArmControl(self.arm_name,
                                                            self.manager)
        rospy.loginfo("CREATED ARM_CONTROL")
        self.pressure_listener =\
            pr2_utils.pressure_listener.PressureListener(self.arm_name)
#        self.wipe_quat = wipe_quat
        self.edge = None
        self.table = None
        rospy.loginfo("SET EDGE AND TABLE")

 #       info = self.head_control.findTableEdge(self.arm_name[0])
#        rospy.loginfo("SET INFO TO HEAD CONTROL")
#        if not info:
#            rospy.loginfo("COULDNT FIND EDGE")
#            rospy.logerr('Unable to find table edge visually')
#        else:
#            rospy.loginfo("FOUND EDGE FINE")
#            (self.edge, self.table) = info

        
    def findTable(self, initial_wrist_pose, 
                  initial_gripper_pose, currently_touching):
        print "in findTable"
        #first move the hand to above the table using collision aware
        move_arm_succeeded = False
        z = initial_wrist_pose.pose.position.z
        orig_z = z
        while not move_arm_succeeded and z < MAX_Z:
            initial_wrist_pose.pose.position.z = z
            rospy.logdebug('Attempting to move to position %s', 
                           str(initial_wrist_pose))
            (ma_r, ma_s) = self.arm_control.move_arm_collision_free\
                (initial_wrist_pose)
            move_arm_succeeded = (ma_r.error_code.val == 
                                  ma_r.error_code.SUCCESS and
                                  ma_s == 
                                  actionlib_msgs.msg.GoalStatus.SUCCEEDED)
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
            if not self.arm_control.try_hard_for_pose(initial_wrist_pose, 
                                                      max_tries=0):
                #we may have actually moved using the Cartesian controller
                (trans, rot) = self.arm_control.get_hand_pose\
                    (frame_id='/base_link')
                dist = ((trans[0] - initial_gripper_pose.pose.position.x)*
                        (trans[0] - initial_gripper_pose.pose.position.x)+
                        (trans[1] - initial_gripper_pose.pose.position.y)*
                        (trans[1] - initial_gripper_pose.pose.position.y)+
                        (trans[2] - initial_gripper_pose.pose.position.z)*
                        (trans[2] - initial_gripper_pose.pose.position.z))
                dist = math.sqrt(dist)
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
            self.arm_control.executeForceControl(wait=False)
            moved = True
            while not rospy.is_shutdown() and\
                    not touching and moved and\
                    trans[2] > self.table.height - 0.1:
                touching = self.pressure_listener.is_touching()
                curr_time = rospy.get_time()
                (trans, rot) = self.arm_control.get_hand_pose\
                    (frame_id='/base_link')
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
            #lowerthread.join()
        if not touching:
            rospy.logerr('Unable to find tabletop')
            return None
        return zforce

    def turn_page(self):
        arbitrary_x = 0.55
        print "in turn_page"
        initial_wrist_pose = geometry_msgs.msg.PoseStamped()
        initial_wrist_pose.header.frame_id = 'base_link'
        initial_wrist_pose.header.stamp = rospy.Time(0)
        initial_wrist_pose.pose.position.x = arbitrary_x
        initial_wrist_pose.pose.position.y = -1.0
        initial_wrist_pose.pose.position.z = self.table.height + 0.15 + 0.18
        initial_wrist_pose.pose.orientation = self.wipe_quat
        initial_gripper_pose = copy.deepcopy(initial_wrist_pose)
        initial_gripper_pose.pose.position.z -= 0.18
 
        currently_touching = False
        zforce = self.findTable(initial_wrist_pose,
                                initial_gripper_pose,
                                currently_touching)
        print "Done"


def main():
    rospy.init_node('page_turning_demo_node')
    rospy.loginfo("STARTING MAIN")

    turner = PageTurner('left_arm')

    rospy.loginfo("ABOUT TO START PAGE TURNER TOUCH!!!!!!!")
    
    # leftarm = ArmWiper('left_arm')
#    if turner.edge == None:
    #     rospy.loginfo("EDGE WAS NONE")
    #     rospy.logerr('Unable to locate table')
    #     return
    # rospy.loginfo("!!!!!!!!!!!!!!!!!!")
    # rospy.loginfo('Left edge is %f', turner.edge)
    turner.turn_page()
#    turner.wipeSector(-0.1, leftarm.edge-0.1, False)


if __name__ == '__main__':
    rospy.loginfo("SERIOUSLY COME ON ROBOT")
    main()
  
