#!/usr/bin/env python

import roslib; 
roslib.load_manifest('collision_placing')
import rospy
import kinematics_msgs
import sys
#import toShape

from pr2_python import arm_planner
from pr2_python import world_interface
from geometry_msgs.msg import PoseStamped
from arm_navigation_msgs.msg import Shape, CollisionObject, AttachedCollisionObject
from physics import *
from pr2_tasks.pickplace import PickPlace
from pr2_tasks.pickplace_definitions import PlaceGoal
from geometry_msgs.msg import PoseStamped, Point, Pose
from pr2_python.exceptions import ManipulationError

from visualization_msgs.msg import Marker

def marker(pose):
    pub = rospy.Publisher('/centroid', Marker)    

    marker = Marker()
    marker.header.frame_id = '/torso_lift_link'
    marker.ns = ''
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = marker.ADD
    marker.pose.position.x = pose[0]
    marker.pose.position.y = pose[1]
    marker.pose.position.z = pose[2]
    marker.pose.orientation.w = 1
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

def good(pose):
    rospy.loginfo("pose="+str(pose))
    return False

    # ik_sol = arm.get_ik(pose)
    # if not isinstance (ik_sol, kinematics_msgs.srv.GetConstraintAwarePositionIKResponse):
    #     return False
    # else:
    #     return True

def acceptable(arm, pose):
        ik_sol = arm.get_ik(pose)
        if isinstance(ik_sol, kinematics_msgs.srv.GetConstraintAwarePositionIKResponse):
            print "Pose Acceptable"
            return True
        else:
            print "Pose Unacceptable"
            return False 

def place(pickplace, obj_name,  pose):
    # rospy.loginfo("pose"+str(pose))
    
    try:        
        place_goal = PlaceGoal('right_arm', [pose], collision_support_surface_name='table', collision_object_name=obj_name)
    #return False
        pickplace.place(place_goal)
    except ManipulationError:
        rospy.loginfo("POSE FAILED")
        return False
    return True

def make_pose(pose, h, orientation):
    pose = PoseStamped()
    pose.header.frame_id = goal.header.frame_id
    pose.pose.position.x = goal.pose.position.x
    pose.pose.position.y = goal.pose.position.y
    pose.pose.position.z = goal.pose.position.z + h 
    pose.pose.orientation.x = goal.pose.orientation.x + orientation[1]
    pose.pose.orientation.y = goal.pose.orientation.y + orientation[2]
    pose.pose.orientation.z = goal.pose.orientation.z + orientation[3]
    pose.pose.orientation.w = goal.pose.orientation.w + orientation[0]

    return pose

def main(goal, mass):
    rospy.init_node("test_placing")

    arm = arm_planner.ArmPlanner('right_arm')
    pickplace = PickPlace(search_place=[])

    obj_name = sys.argv[1]

    # shape = Shape()
    # shape.type = 1 # BOX
    # shape.dimensions = [5,5,10]
    # orien = Orientation(0.0, 0.0, 0.0)
    # obj = Object(shape, mass, orien)

    my_world = world_interface.WorldInterface()
    objs = my_world.attached_collision_objects()
#    objs = [toShape.get_obj()]
    rospy.loginfo("collision objects="+str(len(objs)))
    for obj in objs:
        rospy.loginfo("id="+str(obj.object.id))
        rospy.loginfo("name="+obj_name)
        print "id="+str(obj.object.id)+" name"+str(obj_name)
        if obj.object.id == obj_name:
            for i in range(len(obj.object.shapes)):
                orien = quat_to_orien(obj.object.poses[i].orientation)
                my_obj = Object(obj.object.shapes[i], mass, orien)                
                #marker(my_obj.mesh_centroid())

                placements = copy.deepcopy(my_obj.placements)
                placements.reverse()
                
                (height, orientation) = placements.pop()
                print "height="+str(height)+" orien="+str(orientation)
                pose = make_pose(goal, height, orientation)
                
#                while not good(pose):
                while not acceptable(arm, pose):
#                while not acceptable(pickplace, obj_name, pose):
                    (height, orientation) = placements.pop()
                    print "height="+str(height)+" orien="+str(orientation)
                    pose = make_pose(goal, height, orientation)
                    
                print "pose= "+str(pose)
                place(pickplace, obj_name, pose)


# poop = Shape()
# poop.type = 3
# poop.dimensions = []
# poop.triangles = [0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5, 0, 5, 6, 0, 6, 7, 0, 7, 8, 0, 8, 9, 0, 9, 10, 0, 10, 11, 0, 11, 12, 0, 12, 13, 0, 13, 14, 0, 14, 15, 0, 15, 16, 0, 16, 17, 0, 17, 18, 0, 18, 19, 0, 19, 20, 0, 20, 21, 0, 21, 22]
# poop.vertices = [Point(0.627733027503, -0.257368663369, -4.95490759533e-08), 
# Point(0.597918287709, 0.243853166108, -6.25516136665e-08), 
# Point(0.604025316528, 0.267788063058, -1.00909510081e-07), 
# Point(0.663117588687, 0.267671290818, -7.46618340486e-08), 
# Point(0.940314139347, 0.256665206252, -8.13168696823e-08), 
# Point(1.00624845221, 0.250569754545, -5.10732505177e-08), 
# Point(1.00749680359, 0.240829819432, -5.79549919166e-08), 
# Point(1.0170567927, -0.0461299142572, -7.64847243317e-08), 
# Point(1.02311593942, -0.373831719985, -5.78744625557e-08), 
# Point(1.02097378679, -0.422797108826, -6.74397568901e-08), 
# Point(1.01423100101, -0.503357117406, -1.3776132235e-08), 
# Point(1.01257487455, -0.52402443803, -4.15540917231e-08), 
# Point(1.01071954368, -0.534473891502, -3.31647349583e-08), 
# Point(1.0082483147, -0.542095778114, -8.51832879789e-08), 
# Point(0.99990845135, -0.55330035458, -5.38435216413e-08), 
# Point(0.93370217437, -0.568562881625, -4.05907343293e-08), 
# Point(0.722753407683, -0.582849481984, -3.88900538439e-08), 
# Point(0.715154602178, -0.582850889812, -4.5733788312e-08), 
# Point(0.692177442222, -0.581913720588, -4.19942516316e-08), 
# Point(0.675655124876, -0.573173065776, -1.03539164797e-07), 
# Point(0.67308040561, -0.571586308518, -1.14497078485e-08), 
# Point(0.669026979736, -0.565556486197, -8.98055461107e-08), 
# Point(0.659454441168, -0.524546375928, -2.19454356909e-08)] 


goal = PoseStamped()
goal.header.frame_id = '/torso_lift_link'
goal.pose.position.x = 0.7
goal.pose.position.y = -0.2
goal.pose.position.z = -0.28
goal.pose.orientation.x = 0.0
goal.pose.orientation.y = 0.0
goal.pose.orientation.z = 0.0
goal.pose.orientation.w = 1.0
mass = 10 # grams?

main(goal, mass)
