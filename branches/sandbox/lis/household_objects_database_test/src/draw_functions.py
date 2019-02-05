#!/usr/bin/python
# based on Kaijen Hsiao's draw_functions in object_manipulator

## @package draw_functions
#helper functions for drawing things in rviz

from __future__ import division
import roslib
roslib.load_manifest('object_manipulator')
import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import tf.transformations
import scipy
import math
import time
import pdb
from object_manipulator.convert_functions import *

##class to draw stuff to rviz
class DrawFunctions():
    
    def __init__(self, topic):
        self.marker_pub = rospy.Publisher(topic, Marker)

    ##clear the current set of points
    def clear_rviz_points(self, ns = 'points', id = 0):

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'points'
        marker.type = Marker.POINTS
        marker.action = Marker.DELETE
        marker.id = id
        self.marker_pub.publish(marker)


    ##fill in a Marker message
    def create_marker(self, type, dims, frame, ns, id, duration = 60., color = [1,0,0], opaque = 0.5, pos = [0.,0.,0.], quat = [0.,0.,0.,1.]):
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.type = type
        marker.action = Marker.ADD
        marker.scale.x = dims[0]
        marker.scale.y = dims[1]
        marker.scale.z = dims[2]
        marker.color.a = opaque
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.lifetime = rospy.Duration(duration)
        marker.id = id
        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = pos[2]
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]       
        return marker
 

    ##draw a set of points (3xn or 4xn scipy matrix) in rviz
    def draw_rviz_points(self, points, frame = 'narrow_stereo_optical_frame', size = .005, ns = 'points', id = 0, duration = 20., color = [0,0,1], opaque = 1.0):
        marker = self.create_marker(Marker.POINTS, [size, size, size], frame, ns, id, duration, color, opaque)

        for point_ind in range(scipy.shape(points)[1]):
            new_point = Point()
            new_point.x = points[0, point_ind]
            new_point.y = points[1, point_ind]
            new_point.z = points[2, point_ind]
            marker.points.append(new_point)

        self.marker_pub.publish(marker)
        rospy.loginfo("published points")


    ##draw a set of axes in rviz with arrows of varying lengths
    #pose is a 4x4 scipy matrix
    def draw_rviz_axes(self, pose_mat, frame, lengths = [.05, .01, .01], ns = 'axes', id = 0, duration = 300.):
        
        marker = self.create_marker(Marker.ARROW, [.01, .02, 0], frame, ns, id, duration)
        marker.color.a = 1.0

        #find the arrow endpoints
        start = pose_mat[0:3, 3]
        x_end = (pose_mat[:,0][0:3]*lengths[0] + start).T.tolist()[0]    
        y_end = (pose_mat[:,1][0:3]*lengths[1] + start).T.tolist()[0]
        z_end = (pose_mat[:,2][0:3]*lengths[2] + start).T.tolist()[0]
        start = start.T.tolist()[0]

    #         print "start:", start
    #         print "x_end:", x_end
    #         print "y_end:", y_end
    #         print "z_end:", z_end

        #draw the arrows (x=red, y=green, z=blue)
        marker.id = id
        marker.points = [Point(*start), Point(*x_end)]
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)
        marker.id = id+1
        marker.points = [Point(*start), Point(*y_end)]
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)
        marker.id = id+2
        marker.points = [Point(*start), Point(*z_end)]
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.marker_pub.publish(marker)


    ##draw a sphere in rviz at pose_mat (4x4 scipy matrix) with radius r
    def draw_rviz_sphere(self, pose_mat, r, frame = 'object_frame', ns = 'spheres', id = 0, duration = 60., color = [1,0,0], opaque = 0.5):
        
        (pos, quat) = mat_to_pos_and_quat(pose_mat)
        marker = self.create_marker(Marker.SPHERE, [r*2., r*2., r*2.], frame, ns, id, duration, color, opaque, pos, quat)
        self.marker_pub.publish(marker)


    ##draw a box in rviz at pose_mat (4x4 scipy matrix) defined by either:
    #    2-lists (min, max) of 3-lists (x,y,z) of corner coords
    #    or a 3-list of dimensions (x,y,z)
    #in frame_id frame (defaults to the object frame), id number id, and RGB color
    def draw_rviz_box(self, pose_mat, ranges, frame = 'object_frame', ns = 'boxes', id = 0, duration = 60., color = [1,0,0], opaque = 0.5):
        if len(ranges) == 2:
            dims = [upper-lower for (upper, lower) in list(zip(ranges[0], ranges[1]))]
            center = [(upper-lower)/2+lower for (upper, lower) in list(zip(ranges[0], ranges[1]))]

        elif len(ranges) == 3:
            dims = ranges
            center = [0., 0., 0.]

        #rotate the box center to frame
        center = scipy.matrix(center + [1])
        transformed_center = pose_mat * center.T
                                    
        quat = tf.transformations.quaternion_from_matrix(pose_mat)
            
        marker = self.create_marker(Marker.CUBE, dims, frame, ns, id, duration, color, opaque, transformed_center[0:3, 0], quat)
        self.marker_pub.publish(marker)


    ##draw a cylinder in rviz at pose_mat (4x4 scipy matrix, z-axis is cylinder axis) with radius r and length l
    def draw_rviz_cylinder(self, pose_mat, r, l, frame = 'object_frame', ns = 'cylinders', id = 0, duration = 60., color = [1,0,0], opaque = 0.5):
        
        (pos, quat) = mat_to_pos_and_quat(pose_mat)
        marker = self.create_marker(Marker.CYLINDER, [r*2., r*2., l], frame, ns, id, duration, color, opaque, pos, quat)
        self.marker_pub.publish(marker)

    def draw_mesh(self, pose_mat, mesh,scale=[1.,1.,1.], frame = 'base_link', ns = 'mesh', id = 0, duration = 60., color = [1.,0.,0.], opaque = .8):
        if pose_mat == None:
            pose_mat = scipy.mat([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        (pos, quat) = mat_to_pos_and_quat(pose_mat)
        marker = self.create_marker(Marker.TRIANGLE_LIST,scale,frame, ns, id, duration, color, opaque, pos, quat)
        marker.points = [mesh.vertices[x] for x in mesh.triangles]
        self.marker_pub.publish(marker)

    def clear_mesh(self, ns = 'mesh', num = 100):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.type = Marker.TRIANGLE_LIST
        marker.action = 2 # delete
        for i in range(num):
            marker.id = i
            self.marker_pub.publish(marker)


    ##clear all the currently drawn grasps
    def clear_grasps(self, ns = 'grasps', num = 150):

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.type = 0 #arrow
        marker.action = 2 #delete
        for i in range(num):
            marker.id = i
            self.marker_pub.publish(marker)


    ##draw a set of grasps (wrist Poses) as x and y-axis arrows in rviz, 
    #with the x-axis long compared to y
    def draw_grasps(self, grasps, frame, ns = 'grasps', pause = 0):

        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0)

        for tries in range(2):

            for (grasp_num, grasp) in enumerate(grasps):
                if grasp_num == 0:
                    marker.scale.x = 0.015
                    marker.scale.y = 0.025
                    length_fact = 1.5

                else:
                    marker.scale.x = 0.01
                    marker.scale.y = 0.015
                    length_fact = 1.0

                try:
                    orientation = grasp.orientation
                except:
                    pdb.set_trace()
                quat = [orientation.x, orientation.y, orientation.z, orientation.w]
                mat = tf.transformations.quaternion_matrix(quat)
                start = [grasp.position.x, grasp.position.y, grasp.position.z]
                x_end = list(mat[:,0][0:3]*.05*length_fact + scipy.array(start))    
                y_end = list(mat[:,1][0:3]*.02*length_fact + scipy.array(start))
                marker.id = grasp_num*3
                marker.points = [Point(*start), Point(*x_end)]
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                self.marker_pub.publish(marker)
                marker.id = grasp_num*3+1
                marker.points = [Point(*start), Point(*y_end)]
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                self.marker_pub.publish(marker)
                marker.id = grasp_num*3+2
                if pause:
                    print "press enter to continue"
                    raw_input()
            time.sleep(.5)


#test script
if __name__ == "__main__":
    rospy.init_node('draw_functions', anonymous=True)

    pose_mat = scipy.matrix(scipy.identity(4))
    pose_mat[2, 3] = -1.0

    draw_functions = DrawFunctions('visualization_marker')

    while(not rospy.is_shutdown()):

        points = scipy.matrix([[.8, 0., -1.0],
                               [.9, 0., -1.0],
                               [1.0, 0., -1.0],
                               [.9, .1, -1.0]]).T
        draw_functions.draw_rviz_points(points, frame = 'base_footprint')

        pose_mat[0, 3] = .3
        draw_functions.draw_rviz_box(pose_mat, [.1, .2, .3], frame = 'base_footprint')
        
        pose_mat[0, 3] = 0
        draw_functions.draw_rviz_cylinder(pose_mat, .1, .3, frame = 'base_footprint')
        
        pose_mat[0, 3] = -.3
        draw_functions.draw_rviz_sphere(pose_mat, .1, frame = 'base_footprint')

        pose_mat[0, 3] = .6
        pose = mat_to_pose(pose_mat)
        draw_functions.draw_grasps([pose], frame = 'base_footprint')
        
        print "drew shapes, press enter to re-draw"

        raw_input()
