#!/usr/bin/env python
import roslib; roslib.load_manifest('pressure_practice')
import rospy
from ee_cart_imped_action import *
from ee_cart_imped_control import *
from position_listener import *
import math
from geometry_msgs.msg import Vector3
import threading
import tf


class arm_control:
    """
    Wrapper around the ee_cart_imped_client class for the table_wiper module.
    """
    def __init__(self):
        """
        Creates a new arm_control objects
        """
        print "creating arm control"
	self.control = ee_cart_imped_client(True)
        print "created ee_cart_imped_client"
        """
        Simple action client to deal with the ee_cart_imped_controller.
        """
	self.tf_listener = tf.TransformListener()
        """
        Transform listener that allows the arm control to get the location of the hand in the torso_lift_link frame.
        """
	self.time = 0

    def add_trajectory_point(self, x, y, z, ox, oy, oz, ow, fx, fy, fz, tx, ty, tz, isfx, isfy, isfz, istx, isty, istz, time):
        """
        Adds a new point to the current trajectory. See the relevant method in ee_cart_imped_client for in depth documentation. The only difference is that the time for this is that time this particular action should take, as opposed to the time after the trajectory starts that the arm should be in place.
        """
	self.time += time
	self.control.addTrajectoryPoint(x, y, z, ox, oy, oz, ow, fx, fy, fz, tx, ty, tz, isfx, isfy, isfz, istx, isty, istz, self.time)

    def get_hand_pose(self):
        """
        @returns: The current pose of the left gripper in the torso_lift_link frame.
        """
	while True:
	    try:
		return self.tf_listener.lookupTransform('/torso_lift_link', '/l_gripper_tool_frame', rospy.Time(0))
	    except:
		continue

    def stop_in_place(self):
        """
        Cancels the current arm movement and commands it to stay in place with maximum stiffness.
        """
	(trans,rot) = self.get_hand_pose()
	self.control.cancelGoal()
	self.control.resetGoal()
	self.control.addTrajectoryPoint(trans[0], trans[1], trans[2], 
                                        rot[0],rot[1],rot[2],rot[3],
                                        1000,1000,1000,30,30,30,
                                        False,False,False,False,False,False,1)
	self.control.sendGoal()
	self.control.resetGoal()

    def execute(self, reset =  True, wait = True):
        """
        Commands the arm to actually execute the goal trajectory it has stored.

        @type reset: Boolean
        @param reset: If this is set to False, then this does not reset the current goal of the arm_control object. Defaults to True.
        @type wait: Boolean
        @param wait: If this is set to True, then this method will block until the action completes. Otherwise it will return and let the action continue. This defaults to True.f
        """
	self.control.sendGoal(wait)
	if reset:
	    self.control.resetGoal()
	    self.time = 0

    def cancel(self):
        """
        Cancels the current action of the arm.
        """
	self.control.cancelGoal()

if __name__ == "__main__":
    rospy.init_node('arm_control')
    c = arm_control()
    c.add_trajectory_point(.4, .5, 0, 0,0.707,-0.0,0.707,
                                     1000, 1000, 1000, 30, 30, 30,
                                     False, False, False, False, False,
                                     False, 5.0)
    c.add_trajectory_point(.4, 0, 0, 0,0.707,-0.0,0.707,
                                     1000, 1000, 1000, 30, 30, 30,
                                     False, False, False, False, False,
                                     False, 5.0)
# Arm should move to a point to the left, then to the center
# Arm should move part of the way to the left, but not the whole way
    c.execute(False)
    c.execute(True, False)
    print "Going to sleep"
    rospy.sleep(2.5)
    print "Woken up"
    c.stop_in_place()
    print "stopped in place"
    c.execute()
