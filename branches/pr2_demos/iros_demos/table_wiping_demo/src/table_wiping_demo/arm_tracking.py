#!/usr/bin/env python

import roslib; roslib.load_manifest('table_wiping_demo')
import rospy
import pr2_utils.arm_control
import math
import geometry_msgs.msg
import actionlib_msgs.msg
import arm_navigation_msgs.msg
import random
import copy

TOLERANCE=0.02
MIN_MOVE = None

def getDimension(pose_stamped, dimension):
    if dimension == 0:
        return pose_stamped.pose.position.x
    if dimension == 1:
        return pose_stamped.pose.position.y
    if dimension == 2:
        return pose_stamped.pose.position.z
    if dimension == 3:
        return pose_stamped.pose.orientation.x
    if dimension == 4:
        return pose_stamped.pose.orientation.y
    if dimension == 5:
        return pose_stamped.pose.orientation.z
    if dimension == 6:
        return pose_stamped.pose.orientation.w
    return None

def setDimension(pose_stamped, dimension, value):
    if dimension == 0:
        pose_stamped.pose.position.x = value
    elif dimension == 1:
        pose_stamped.pose.position.y = value
    elif dimension == 2:
        pose_stamped.pose.position.z = value
    elif dimension == 3:
        pose_stamped.pose.orientation.x = value
    elif dimension == 4:
        pose_stamped.pose.orientation.z = value
    elif dimension == 5:
        pose_stamped.pose.orientation.y = value
    elif dimension == 6:
        pose_stamped.pose.orientation.w = value

def getDimensionTR(trans, rot, dimension):
    if dimension < 3:
        return trans[dimension]
    if dimension < 7:
        return rot[dimension]
    return None

def jigglePose(origpose, jiggle):
    if jiggle == None:
        return pr2_utils.utils.copyPoseStamped(origpose)
    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.header = pr2_utils.utils.copyHeader(origpose.header)
    pose_stamped.pose.position.x =\
        origpose.pose.position.x +\
        random.uniform(-1.0*jiggle[0], jiggle[0])
    pose_stamped.pose.position.y =\
        origpose.pose.position.y +\
        random.uniform(-1.0*jiggle[1], jiggle[1])
    pose_stamped.pose.position.z =\
        origpose.pose.position.z +\
        random.uniform(-1.0*jiggle[2], jiggle[2])
    pose_stamped.pose.orientation.x =\
        origpose.pose.orientation.x +\
        random.uniform(-1.0*jiggle[3], jiggle[3])
    pose_stamped.pose.orientation.y =\
        origpose.pose.orientation.y +\
        random.uniform(-1.0*jiggle[4], jiggle[4])
    pose_stamped.pose.orientation.z =\
        origpose.pose.orientation.z +\
        random.uniform(-1.0*jiggle[5], jiggle[5])
    pose_stamped.pose.orientation.w =\
        origpose.pose.orientation.w +\
        random.uniform(-1.0*jiggle[6], jiggle[6])
    return pose_stamped



class HandTracker:
    def __init__(self, following_arm_name, monitored_arm_name, manager=None):
        self.following_arm_name = following_arm_name
        self.monitored_arm_name = monitored_arm_name
        self.following_arm_control = pr2_utils.arm_control.ArmControl\
            (following_arm_name, manager)
        self.monitored_arm = pr2_utils.arm_control.ArmControl\
            (monitored_arm_name, self.following_arm_control.manager)
        self.manager = self.following_arm_control.manager

    def track(self, pose_stamped, dimension, table_names, offset=0, 
              jiggle=None, max_lag_plus=float('inf'),
              max_lag_minus=float('inf'), tolerance=TOLERANCE):
        looprate = rospy.Rate(5)
        self.still_tracking = True
        origpose = pr2_utils.utils.copyPoseStamped(pose_stamped)
        monitored_arm_colls =\
            arm_navigation_msgs.msg.OrderedCollisionOperations()
        #the monitored hand can do whatever it likes and we
        #will ignore it
        #SHOULD READ PARAMETERS INSTEAD OF ASSUMING NAMES!
        for name in table_names:
            op = arm_navigation_msgs.msg.CollisionOperation()
            op.object1 = self.monitored_arm_name
            op.object2 = name
            op.operation = op.DISABLE
            monitored_arm_colls.collision_operations.append(op)
            oph = copy.copy(op)
            oph.object1 = self.monitored_arm_name[0]+'_end_effector'
            monitored_arm_colls.collision_operations.append(oph)
        while not rospy.is_shutdown() and self.still_tracking:
            rospy.loginfo('In following loop')
            (following_hand_trans, following_hand_rot) =\
                self.following_arm_control.get_hand_pose\
                (pose_stamped.header.frame_id)
            (monitored_hand_trans, monitored_hand_rot) =\
                self.monitored_arm.get_hand_pose(pose_stamped.header.frame_id)
            diff = getDimensionTR(following_hand_trans,\
                                      following_hand_rot, dimension) -\
                                      (getDimensionTR(monitored_hand_trans,\
                                                          monitored_hand_rot,\
                                                          dimension) + offset)
            if math.fabs(diff) > tolerance:
                setDimension(pose_stamped, dimension, 
                             getDimensionTR(monitored_hand_trans,
                                            monitored_hand_rot,
                                            dimension)+offset)
                rospy.loginfo('Moving following arm to pose %s.  Monitored arm is at trans: %s, rot: %s', str(pose_stamped), str(monitored_hand_trans), 
                              str(monitored_hand_rot))
                error_code = self.attemptMove(pose_stamped, jiggle=jiggle,
                                              ordered_colls=monitored_arm_colls)
                if error_code.val != error_code.SUCCESS:
                    print 'diff =', diff
                    if diff > max_lag_plus or (-1.0*diff) > max_lag_minus:
                        #get the arm out of there
                        rospy.logerr('Unable to keep up tracking.')
                        #DON'T use move_arm_to_side because that may reset the
                        #collision map!
                        side_pose = geometry_msgs.msg.PoseStamped()
                        side_pose.pose.position.x = 0
                        side_pose.pose.position.y = -0.6
                        side_pose.pose.position.z = -0.25
                        side_pose.pose.orientation.x = -0.5
                        side_pose.pose.orientation.y = 0.5
                        side_pose.pose.orientation.z = 0.5
                        side_pose.pose.orientation.w = 0.5
                        side_pose.header.frame_id = '/torso_lift_link'
                        moved_to_side = False
                        ntries = 0
                        go_to_pose = pr2_utils.utils.copyPoseStamped(side_pose)
                        while not moved_to_side and ntries < 5:
                            (ma_r, ma_s) =\
                                self.following_arm_control.\
                                move_arm_collision_free\
                                (go_to_pose, ordered_colls=monitored_arm_colls)
                            moved_to_side =\
                                (ma_r.error_code.val ==\
                                     ma_r.error_code.SUCCESS and\
                                     ma_s ==\
                                     actionlib_msgs.msg.GoalStatus.SUCCEEDED)
                            go_to_pose = jigglePose(side_pose,
                                                    (0.05,0.05,0.05, 0,0,0,0))
                            ntries += 1
                        if not moved_to_side:
                            self.following_arm_control.try_hard_for_pose\
                                (side_pose, max_tries=0)
                            pose_stamped =\
                                pr2_utils.utils.copyPoseStamped(origpose)
                    else:
                        pose_stamped = jigglePose(origpose, jiggle)
                        continue
                else:
                    pose_stamped = pr2_utils.utils.copyPoseStamped(origpose)
            else:
                rospy.loginfo('Following arm within tolerance of moving arm')
            looprate.sleep()
        
    def stopTracking(self):
        self.still_tracking = False

    def collisionWithFollowingArm(self, collisions):
        for op in collisions.collision_operations:
            if op.object1[0:2] == self.following_arm_name[0]+'_' or\
                    op.object2[0] == self.following_arm_name[0]+'_' or\
                    op.object1 == op.COLLISION_SET_ATTACHED_OBJECTS or\
                    op.object2 == op.COLLISION_SET_ATTACHED_OBJECTS:
                return True
        return False

    def broadenCollisionOps(self, collisions):
        origcoll = list(collisions.collision_operations)
        grippername = self.following_arm_name[0]+'_end_effector'
        gripperprefix = self.following_arm_name[0]+'_gripper'
        for op in origcoll:
            if op.object1[0:len(gripperprefix)] == gripperprefix:
                newop = arm_navigation_msgs.msg.CollisionOperation()
                newop.object1 = grippername
                newop.object2 = op.object2
                newop.operation = newop.DISABLE
                collisions.collision_operations.append(newop)
                aop = copy.copy(newop)
                aop.object1 = aop.COLLISION_SET_ATTACHED_OBJECTS
                collisions.collision_operations.append(aop)

    def makeSmallMove(self, direction=(-0.707, -0.707, 0)):
        (trans, rot) =\
            self.following_arm_control.get_wrist_pose\
            (frame_id='/base_link')
        pose_stamped = pr2_utils.conversions.TRtoPoseStamped\
            (trans, rot, '/base_link')
        pose_stamped.pose.position.x += 0.05*direction[0]
        pose_stamped.pose.position.y += 0.05*direction[1]
        pose_stamped.pose.position.z += 0.05*direction[2]
        #ignore all collisions during this move
        op = arm_navigation_msgs.msg.CollisionOperation()
        op.object1 = op.COLLISION_SET_ALL
        op.object2 = op.COLLISION_SET_ALL
        op.operation = op.DISABLE
        ordered_colls = arm_navigation_msgs.msg.OrderedCollisionOperations()
        ordered_colls.collision_operations.append(op)
        return self.attemptMove(pose_stamped, jiggle=None,
                                ordered_colls=ordered_colls,
                                move_for_collisions=False,
                                min_dist=0.03)
        

    def attemptToRemoveCollisions(self):
        ntries = 0
        ordered_colls = self.following_arm_control.in_collision()
        while self.collisionWithFollowingArm(ordered_colls) and\
                ntries < 3 and not rospy.is_shutdown() and\
                self.still_tracking:
            direction.append(random.uniform(-1,1))
            direction.append(random.uniform(-1,1))
            direction.append(random.uniform(-1,1))
            mag = math.sqrt(direction[0]*direction[0]+
                            direction[1]*direction[1]+
                            direction[2]*direction[2])
            direction[0] /= mag
            direction[1] /= mag
            direction[2] /= mag
            error_code = self.makeSmallMove(direction=direction)
            ordered_colls = self.following_arm_control.in_collision()
            ntries += 1
        if self.collisionWithFollowingArm(ordered_colls) > 0:
            return False
        return True

    def attemptMove(self, pose_stamped, jiggle=None, 
                    ordered_colls=
                    arm_navigation_msgs.msg.OrderedCollisionOperations(),
                    move_for_collisions=True,
                    min_dist=MIN_MOVE):
        # if move_for_collisions:
        #     if not self.attemptToRemoveCollisions():
        #         if self.collisionWithFollowingArm\
        #                 (self.following_arm_control.in_collision()):
        #             error_code =\
        #                 arm_navigation_msgs.msg.ArmNavigationErrorCodes()
        #             error_code.val = error_code.START_STATE_IN_COLLISION
        #             return error_code
        #first try moving using interpolated ik
        error_code =\
            self.following_arm_control.move_arm_interpolated_ik\
            (pose_stamped, ordered_colls=ordered_colls, min_dist=min_dist)
        if error_code.val == error_code.SUCCESS:
            return error_code
        if not move_for_collisions:
            error_code = arm_navigation_msgs.msg.ArmNavigationErrorCodes()
            error_code.val = error_code.PLANNING_FAILED
        #interpolated IK didn't work... try move_arm
        (ma_r, ma_s) = \
            self.following_arm_control.move_arm_collision_free\
            (pose_stamped, ordered_colls=ordered_colls,\
                 timeout=rospy.Duration(25.0))
        if ma_r.error_code.val == ma_r.error_code.SUCCESS and\
                ma_s != actionlib_msgs.msg.GoalStatus.SUCCEEDED:
            ma_r.error_code.val = ma_r.error_code.PLANNING_FAILED
        return ma_r.error_code
def main():
    rospy.init_node('right_arm_tracker_test_node')
    right_tracker = HandTracker('right_arm', 'left_arm')
    colls = right_tracker.following_arm_control.in_collision()
    print 'colls are', colls
    colls2 = right_tracker.following_arm_control.in_collision(ordered_colls=colls)
    print 'ignoring these collisions, collisions are', colls2
    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.header.frame_id = '/base_link'
    pose_stamped.pose.position.x = 0.2
    pose_stamped.pose.position.y = 0
    pose_stamped.pose.position.z = 0.6
    pose_stamped.pose.orientation.x = 0.0
    pose_stamped.pose.orientation.y = 0.707
    pose_stamped.pose.orientation.z = 0.0
    pose_stamped.pose.orientation.w = 0.707
    right_tracker.still_tracking = True
    op = arm_navigation_msgs.msg.CollisionOperation()
    op.object1 = op.COLLISION_SET_ALL
    op.object2 = op.COLLISION_SET_ALL
    op.operation = op.DISABLE
    ops = arm_navigation_msgs.msg.OrderedCollisionOperations()
    ops.collision_operations.append(op)
    #right_tracker.makeSmallMove(ops)
    right_tracker.attemptMove(pose_stamped)

if __name__ == '__main__':
    main()
                       
