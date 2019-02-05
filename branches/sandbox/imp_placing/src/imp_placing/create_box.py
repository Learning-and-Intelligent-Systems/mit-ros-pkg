#!/usr/bin/env python

import roslib; 
roslib.load_manifest('collision_placing')
import rospy
import sys
import math
from geometry_msgs.msg import PoseStamped, Quaternion
import pr2_python.transform_listener as tf
import pr2_python.geometry_tools as gt

from pr2_python.world_interface import WorldInterface

def create_plate(my_world):
    pose = PoseStamped()
    pose.header.frame_id = "/l_gripper_r_finger_link"
    pose.pose.position.x = 0.11
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.0
    pose.pose.orientation.w = 0.707
    pose.pose.orientation.x = 0.707
    pose.pose.orientation.y = 0.00
    pose.pose.orientation.z = 0.00

    box_dims = (0.25,0.25,0.03) 

    obj_id =  "graspable_object_1000"
    my_world.add_collision_box(pose, box_dims, obj_id)
    
    return obj_id
    
def create_box(my_world):
    pose = PoseStamped()
    pose.header.frame_id = "/l_gripper_r_finger_link"
    pose.pose.position.x = 0.05
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.15
    pose.pose.orientation.w = 0.5
    pose.pose.orientation.x = 0.5
    pose.pose.orientation.y = -0.5
    pose.pose.orientation.z = 0.5

    box_dims = (0.35,0.08,0.04) 

    obj_id =  "graspable_object_1000"
    my_world.add_collision_box(pose, box_dims, obj_id)
    
    return obj_id

def main():
    obj_type = sys.argv[1]
    my_world = WorldInterface()

#    print "quat="+str(gt.euler_to_quaternion(math.pi/2,math.pi/2,0))

    if obj_type=="plate" or obj_type=="paper_plate" or obj_type=="plastic_plate":
        obj = create_plate(my_world)
    elif obj_type=="box":
        obj = create_box(my_world)

    my_world.attach_object_to_gripper('left_arm', obj)

rospy.init_node('create_obj')
main()

#    box_pose = tf.transform_pose_stamped("/torso_lift_link", pose)

#    poop = gt.euler_to_quaternion(0.0,math.pi/2,math.pi/2)
#    print "poop="+str(poop)
    # fart = gt.quaternion_to_euler(Quaternion(0.0,0.0,0.707,0.707))
    # print "fart"+str(fart)
