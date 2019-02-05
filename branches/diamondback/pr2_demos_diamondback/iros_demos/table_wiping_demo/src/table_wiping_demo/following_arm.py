#!/usr/bin/env python

import roslib; roslib.load_manifest('table_wiping_demo')
import rospy
import pr2_utils.arm_control
import math
import geometry_msgs.msg
import actionlib_msgs.msg
import motion_planning_msgs.msg
import random
import copy
import threading
import std_msgs.msg

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
    def __init__(self, following_arm_name, monitored_arm_name, 
                 topic_name, dimension=1, offset=0, initial_pose=None, 
                 manager=None):
        self.following_arm_name = following_arm_name
        self.monitored_arm_name = monitored_arm_name
        self.following_arm_control = pr2_utils.arm_control.ArmControl\
            (following_arm_name, manager)
        self.monitored_arm = pr2_utils.arm_control.ArmControl\
            (monitored_arm_name, self.following_arm_control.manager)
        self.manager = self.following_arm_control.manager
        self.initial_pose = initial_pose
        if not self.initial_pose:
            self.initial_pose = self.following_arm_control.get_wrist_pose\
                (frame_id='/torso_lift_link')
        self.dimension = dimension
        self.goal = self.initial_pose
        self.newGoal = self.wristAtPose(self.goal)
        self.offset = offset
        self.sub_thread = threading.Thread\
            (target=self.goalCB,\
                 args=(std_msgs.msg.Float32\
                           (getDimension(self.goal, self.dimension)),))
        self.sub_thread.start()
        self.sub = rospy.Subscriber(topic_name, std_msgs.msg.Float32,
                                    self.goalCB)
        self.still_tracking = False


    def setInitialPose(self, initial_pose):
        self.initial_pose = initial_pose
        self.goal = initial_pose
        self.newGoal = self.wristAtPose(self.goal)

    def setOffset(self, offset):
        self.offset = offset
        
    def goalCB(self, pos):
        rospy.loginfo('Following arm: moving to %f', pos.data)
        self.goal = pr2_utils.utils.copyPoseStamped(self.initial_pose)
        setDimension(self.goal, self.dimension, pos.data+self.offset)
        self.newGoal = True

    def track(self, jiggle=None, max_lag_plus=float('inf'),
              max_lag_minus=float('inf')):
        #ignore any collisions of the other arm
        #with objects in the environment
        self.still_tracking = True
        ordered_colls = motion_planning_msgs.msg.OrderedCollisionOperations()
        coll = motion_planning_msgs.msg.CollisionOperation()
        coll.object1 = self.monitored_arm_name
        coll.object2 = coll.COLLISION_SET_OBJECTS
        coll.operation = coll.DISABLE
        ordered_colls.collision_operations.append(coll)
        collg = copy.copy(coll)
        collg.object1 = self.monitored_arm_name[0]+'_end_effector'
        ordered_colls.collision_operations.append(collg)
        loop_rate = rospy.Rate(4)
        bounds = []
        for i in range(3):
            if self.dimension == i or not jiggle:
                bounds.append(0.01)
            else:
                bounds.append(2.0*jiggle[i])
        for i in range(3):
            bounds.append(0.1)
        ntries = 0
        while not rospy.is_shutdown() and self.still_tracking:
            curr_pose = self.following_arm_control.get_wrist_pose\
                (frame_id = self.goal.header.frame_id)
            diff = getDimension(curr_pose, self.dimension) -\
                (getDimension(self.goal, self.dimension))
            if math.fabs(diff) < TOLERANCE:
                ntries = 0
                loop_rate.sleep()
                continue
            try_for_pose = self.goal
            if not self.newGoal:
                try_for_pose = jigglePose(self.goal, jiggle)
                ntries += 1
            else:
                self.newGoal = False
                ntries = 0
            rospy.loginfo('Attempting to move to %f', 
                          getDimension(self.goal, self.dimension))
            error_code = self.attemptMove(try_for_pose,
                                          bounds=bounds,
                                          ordered_colls=ordered_colls)
            rospy.loginfo('Move returned error code %d', error_code.val)
            if error_code != error_code.SUCCESS:
                curr_pose = self.following_arm_control.get_wrist_pose\
                    (frame_id = self.goal.header.frame_id)
                diff = getDimension(curr_pose, self.dimension) -\
                    (getDimension(try_for_pose, self.dimension))
                if (diff > max_lag_plus or (-1.0*diff) > max_lag_minus) and\
                        ntries > 5:
                    rospy.logerr('Unable to keep up tracking on goal %f after %d tries', getDimension(self.goal, self.dimension), ntries)
                    self.following_arm_control.move_arm_to_side(ordered_colls=
                                                                ordered_colls)
                    ntries = 0
            else:
                rospy.loginfo('Successfully moved')
            loop_rate.sleep()

    def wristAtPose(self, pose_stamped):
        curr_pose = self.following_arm_control.get_wrist_pose\
            (frame_id=pose_stamped.header.frame_id)
        return (pr2_utils.utils.transDist(curr_pose, pose_stamped) < TOLERANCE)
    

    def attemptMove(self, pose_stamped, 
                    bounds = (0.01, 0.01, 0.01, 0.1, 0.1, 0.1),
                    ordered_colls=
                    motion_planning_msgs.msg.OrderedCollisionOperations(),
                    min_dist=MIN_MOVE):
        error_code =\
            self.following_arm_control.move_arm_interpolated_ik\
            (pose_stamped, bounds=bounds,\
                 ordered_colls=ordered_colls, min_dist=min_dist)
        if error_code.val == error_code.SUCCESS:
            return error_code
        #interpolated IK didn't work... try move_arm
        (ma_r, ma_s) = \
            self.following_arm_control.move_arm_collision_free\
            (pose_stamped, ordered_colls=ordered_colls,\
                 timeout=rospy.Duration(25.0), bounds=bounds)
        if ma_r.error_code.val == ma_r.error_code.SUCCESS and\
                ma_s != actionlib_msgs.msg.GoalStatus.SUCCEEDED:
            ma_r.error_code.val = ma_r.error_code.PLANNING_FAILED
        return ma_r.error_code
        
    def stopTracking(self):
        self.still_tracking = False


def main():
    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.header.frame_id = '/base_link'
    pose_stamped.pose.position.x = 0.2
    pose_stamped.pose.position.y = 0
    pose_stamped.pose.position.z = 0.6
    pose_stamped.pose.orientation.x = 0.2
    pose_stamped.pose.orientation.y = 0.69
    pose_stamped.pose.orientation.z = 0.0
    pose_stamped.pose.orientation.w = 0.69
    right_tracker = HandTracker('right_arm', 'left_arm',
                                'publish_to_test_tracking',
                                initial_pose=pose_stamped,
                                offset=-.175)
    right_tracker.track(jiggle=(0.05, 0, 0.05, 0,0,0,0),\
                            max_lag_plus=0.1)

if __name__ == '__main__':
    rospy.init_node('right_arm_tracker_test_node')
    main()
                       
