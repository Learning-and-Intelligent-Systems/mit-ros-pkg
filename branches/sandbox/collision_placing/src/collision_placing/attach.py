#!/usr/bin/env python

import roslib; 
roslib.load_manifest('collision_placing')
import rospy
import sys

from pr2_python.world_interface import WorldInterface

def main():
    obj = sys.argv[1]
    my_world = WorldInterface()
    my_world.attach_object_to_gripper('right_arm', obj)

rospy.init_node('attach_obj')
main()
