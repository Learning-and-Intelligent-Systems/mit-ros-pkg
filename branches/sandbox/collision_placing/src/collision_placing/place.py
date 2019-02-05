#!/usr/bin/env python

import roslib; 
roslib.load_manifest('collision_placing')
import rospy
import sys

from pr2_python.world_interface import WorldInterface
from pr2_tasks.pickplace import PickPlace
from pr2_tasks.pickplace_definitions import PlaceGoal
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from pr2_python.exceptions import ManipulationError

def main():
    pub = rospy.Publisher('/fail_location', Marker)    

    pickplace = PickPlace(search_place=[])
    
    place_pose = PoseStamped()
    place_pose.header.frame_id = '/torso_lift_link'
    place_pose.pose.position.x = 0.7
    place_pose.pose.position.y = -0.2
    place_pose.pose.position.z = -0.28
    place_pose.pose.orientation.x = 0.707
    place_pose.pose.orientation.w = 0.707

    obj = sys.argv[1]
    
    try:

        place_goal = PlaceGoal('right_arm', [place_pose], \
                                   collision_support_surface_name='table', \
                                   collision_object_name=obj)
        pickplace.place(place_goal)

    except ManipulationError:        
        pass
        # my_world = WorldInterface()
        # fail_loc = my_world.collision_object(obj)
        # rospy.loginfo("FAIL LOC="+str(fail_loc))

    marker = Marker()
    marker.header.frame_id = '/torso_lift_link'
    marker.ns = ''
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = marker.ADD
    marker.header.frame_id = place_pose.header.frame_id
    marker.pose = place_pose.pose
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 0.8
    
    rospy.loginfo("marker msg="+str(marker))
    pub.publish(marker)
    rospy.sleep(15.0)
    
rospy.init_node('place_obj')
main()
