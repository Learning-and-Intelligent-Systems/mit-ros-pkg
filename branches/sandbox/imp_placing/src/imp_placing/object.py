import math
import numpy
import copy

#from motion_prim import *
#import physics
import tools
import modes
import degree_of_freedom
from numpy.linalg import *
from numpy import array, matrix
from scipy.integrate import quad
from pr2_python import geometry_tools as gt
from geometry_msgs.msg import Quaternion, PoseStamped, Pose, Point
import pr2_python.transform_listener as tf

def frange(start, stop, step):
    r = start
    if step > 0:
        while r < stop:
            yield r
            r += step
    else:
        while r > stop:
            yield r
            r += step


class Object:

    def __init__(self, shape, pose, modes):
        self.shape = shape
        self.pose = pose
        self.modes = modes # probs of each mode given all configurations

    def volume(self):
        if self.shape.type == 0:
            # SPHERE:=type
            # radius:=dimensions[0]
            volume = math.pi * self.shape.dimensions[0]**2
        elif self.shape.type == 1:
            # BOX:=type
            # size_x:=dimensions[0]
            # size_y:=dimensions[1]
            # size_z:=dimensions[2]
            volume = self.shape.dimensions[0]*self.shape.dimensions[1]*self.shape.dimensions[2]
        elif self.shape.type == 2: 
            # CYLINDER:=type
            # radius:=dimensions[0]
            # length:=dimensions[1] (along Z axis)
            volume = (math.pi * self.shape.dimensions[0]**2) * self.shape.dimensions[1]
        else:
            # MESH:=type
            volume = self.mesh_volume()
        return volume

    def mesh_volume(self):
        vol = 0
        # triangles:= (p1.x, p2.x, p3.x, p1.y, p2.y, p3.y, p1.z, p2.z, p3.z)
        triangle_indices = []
        for i in range(len(self.shape.triangles)/3):
            triangle_indices.append([self.shape.triangles[3*i], \
                                         self.shape.triangles[3*i+1], \
                                         self.shape.triangles[3*i+2]])
        for tri in triangle_indices:
            p1 = self.shape.vertices[tri[0]]
            p2 = self.shape.vertices[tri[1]]
            p3 = self.shape.vertices[tri[2]]

            v321 = p3.x * p2.y * p1.z
            v231 = p2.x * p3.y * p1.z
            v312 = p3.x * p1.y * p2.z
            v132 = p1.x * p3.y * p2.z
            v213 = p2.x * p1.y * p3.z
            v123 = p1.x * p2.y * p3.z

            det = -v321 + v231 + v312 - v132 - v213 + v123
            vol += det * (1.0/6.0)
        return vol

    def centroid(self, pose):
        # ASSUME: centered at the origin
        if self.shape.type == 0:
            # SPHERE:=type (centered around (0,0,0))
            centroid = matrix([[0.0], [0.0], [0.0]])
        elif self.shape.type == 1:
            # BOX:=type
            # size_x:=dimensions[0]
            # size_y:=dimensions[1]
            # size_z:=dimensions[2]
            dim = [float(self.shape.dimensions[0]), \
                       float(self.shape.dimensions[1]), \
                       float(self.shape.dimensions[2])]
            centroid = matrix([[0.0], [0.0], [0.0]])
        elif self.shape.type == 2: 
            # CYLINDER:=type 
            # radius:=dimensions[0] (centered at (0,0))
            # length:=dimensions[1] (along Z axis)
            centroid = matrix([[0.0], [0.0], [self.shape.dimensions[1]/2]])
        else:
            # MESH:=type
            centroid = self.mesh_centroid()
        return centroid

    def mesh_centroid(self):
        vol = self.volume()
        cm = []
        triangle_indices = []
        for i in range(len(self.shape.triangles)/3):
            triangle_indices.append([self.shape.triangles[3*i], \
                                         self.shape.triangles[3*i+1], \
                                         self.shape.triangles[3*i+2]])
        for tri in triangle_indices:
            p1 = self.shape.vertices[tri[0]]
            p2 = self.shape.vertices[tri[1]]
            p3 = self.shape.vertices[tri[2]]

            centroid = ((p1.x+p2.x+p3.x)/3, (p1.y+p2.y+p3.y)/3, (p1.z+p2.z+p3.z)/3)
            cm.append(centroid)
        x_cm = math.fsum([cm_i[0] for cm_i in cm])/len(cm)
        y_cm = math.fsum([cm_i[1] for cm_i in cm])/len(cm)
        z_cm = math.fsum([cm_i[2] for cm_i in cm])/len(cm)
    
        return matrix([[x_cm], [y_cm], [z_cm]])

    def center_of_mass(self, pose):
        '''
        Assuming uniform mass distribution
        '''
        return self.centroid(pose)

    def translate(self, cm, pose):
        x = cm[0][0] + pose.pose.position.x
        y = cm[1][0] + pose.pose.position.y
        z = cm[2][0] + pose.pose.position.z
        return matrix([[x],[y],[z]])

    def inv_translate(self, cm, pose):
        x = cm[0][0] - pose.pose.position.x
        y = cm[1][0] - pose.pose.position.y
        z = cm[2][0] - pose.pose.position.z
        return matrix([[x],[y],[z]])

    def get_base(self, rotation):
        bb = gt.bounding_box_corners(self.shape) # in frame of obj
        transformed_bb = []
        
        # transform bounding box to frame of the robot
        for pt in bb:
            transformed_pt = gt.transform_list(pt, rotation)
            transformed_bb.append(transformed_pt)

        # find base in this frame
        transformed_bb.sort()
        (minxminyminz, minxminymaxz, minxmaxyminz, minxmaxymaxz, \
             maxxminyminz, maxxminymaxz, maxxmaxyminz, maxxmaxymaxz) = transformed_bb

        return [minxminyminz, minxmaxyminz, maxxminyminz, maxxmaxyminz]

    def max_angles(self, pose):
        '''
        @type pose: PoseStamped
        @param pose: the pose the object will be released in (in the frame of the robot)
        '''

        ## get the center of mass in the frame of the object, 
        ## then transform it into the robot frame
        cm = self.center_of_mass(pose).tolist() ## in the frame of the object
        tf_cm  = self.translate(cm, pose)
        
        ## get the corners of the base of the bounding box of the object 
        ## in robot frame
        base = self.get_base(pose.pose)

        ## get the euler angles of the pose (goal pose)
        (orig_phi, orig_theta, orig_psi) = gt.quaternion_to_euler(pose.pose.orientation)

        ## max rotation of pi/2
        max_phi = orig_phi + math.pi/2
        max_theta = orig_theta + math.pi/2

        pivot = Point()
        pivot.x = base[2][0]
        pivot.y = base[0][1]
        pivot.z = base[0][2]

#        print "base="+str(base)

        cm = Pose()
        cm.position.x = tf_cm[0][0]
        cm.position.y = tf_cm[1][0]
        cm.position.z = tf_cm[2][0]
        cm.orientation = pose.pose.orientation

        t = gt.inverse_transform_point(pivot, cm)

        tol=0.02
        ## find max angle around x-axis, phi
#        print "cm="+str(tf_cm.tolist())
        for phi in frange(0.0, math.pi/2, 0.1):
            transform = Pose()
            transform.orientation = gt.euler_to_quaternion(phi, 0.0, 0.0)
            
            cm_new = Pose()
            cm_new.position = cm.position
            cm_new.orientation = gt.transform_quaternion(cm.orientation, transform)

            p_new = gt.transform_point(t, cm_new)
            
            y_diff = abs(tf_cm[1][0]-p_new.y)
            z_diff = abs(tf_cm[2][0]-p_new.z)
           
            if y_diff < tol or z_diff < tol:
                max_phi = phi
                break 

        for theta in frange(0.0, math.pi/2, 0.1):
            transform = Pose()
            transform.orientation = gt.euler_to_quaternion(0.0, theta, 0.0)
            
            cm_new = Pose()
            cm_new.position = cm.position
            cm_new.orientation = gt.transform_quaternion(cm.orientation, transform)

            p_new = gt.transform_point(t, cm_new)

#            print "pivot="+str(p_new)
#            print "cm="+str(tf_cm)
            
            x_diff = abs(tf_cm[0][0]-p_new.x)
            z_diff = abs(tf_cm[2][0]-p_new.z)

#            print "theta=" + str(theta) + " x_diff="+str(x_diff) 
            
            if z_diff < tol or x_diff < tol:
                max_theta = theta
                break 

        return (max_phi, max_theta)

#     def best_placements(self, goal_pose):
#         placements = []
# #        max_angles = self.max_angles(pose)
#         max_angles = self.max_angles(goal_pose)
#         print "max angles="+str(max_angles)
#         max_height = 0.15
# #        tfp = tf.transform_pose_stamped("/base_link", goal_pose)
# #        pose_height = tfp.pose.position.z

#         # getting rid of x angle temporarily 
#         # for h in frange(0.0, max_height, 0.1):
#         #     for phi in frange(0.0, max_angles[0], 0.1):
#         #         for theta in frange(0.0, max_angles[1], 0.1):
#         #             placements.append((h, phi, theta, 0.0))
#         # for h in frange(0.0, max_height, 0.1):
#         #     for phi in frange(max_angles[0], math.pi/2, 0.1):
#         #         for theta in frange(max_angles[1], math.pi/2, 0.1):
#         #             placements.append((h, phi, theta, 0.0))

#         angle = -25. * (math.pi/180.)
#         h = 0.023
#         placements.append((h, 0.0,0.0,angle))
#         placements.append((h, 0.0,0.0,angle))
#         placements.append((h, 0.0,0.0,angle)) 
#         placements.append((h, 0.0,0.0,angle)) 
#         placements.append((h, 0.0,0.0,angle)) 

# #         for h in frange(0.0, max_height, 0.1):
# # #            for phi in frange(0.0, max_angles[0], 0.1):
# # #            for theta in frange(0.0, max_angles[1], 0.1):
# #             for psi in frange(0.0, -max_angles[1], -0.1):
# #                 placements.append((h, 0.0, 0.0, psi))
# #         for h in frange(0.0, max_height, 0.1):
# # #            for phi in frange(max_angles[0], math.pi/2, 0.1):
# # #            for theta in frange(max_angles[1], math.pi/2, 0.1):
# #             for psi in frange(-max_angles[1], -math.pi/2, -0.1):
# #                 placements.append((h, 0.0, 0.0, psi))


#         mode = modes.Mode(self, goal_pose)
#         best_releases = []
#         for place in placements:
#             best_releases.append(tools.make_pose(goal_pose, place[0], \
#                                                      gt.euler_to_quaternion(\
#                         place[1],place[2],place[3])))
#         return best_releases
        



    def real_best_placements(self, goal_pose):
        placements = []
#        max_angles = self.max_angles(pose)
        max_angles = self.max_angles(goal_pose)
        print "max angles="+str(max_angles)
        max_height = 0.15

        angle = -25. * (math.pi/180.)
        h = 0.023
        placements.append((h, 0.0,0.0,angle))
        placements.append((h, 0.0,0.0,angle))
        placements.append((h, 0.0,0.0,angle)) 
        placements.append((h, 0.0,0.0,angle)) 
        placements.append((h, 0.0,0.0,angle)) 
 
        if max_angles[1] < max_angles[0]:
            for h in frange(0.0, max_height, 0.1):
#            for phi in frange(0.0, max_angles[0], 0.1):
#            for theta in frange(0.0, max_angles[1], 0.1):
                for psi in frange(0.0, -max_angles[1], -0.1):
                    placements.append((h, 0.0, 0.0, psi))
            for h in frange(0.0, max_height, 0.1):
#            for phi in frange(max_angles[0], math.pi/2, 0.1):
#            for theta in frange(max_angles[1], math.pi/2, 0.1):
                for psi in frange(-max_angles[1], -math.pi/2, -0.1):
                    placements.append((h, 0.0, 0.0, psi))
        else:
            for h in frange(0.0, max_height, 0.1):
                # for phi in frange(0.0, max_angles[0], 0.1):
                for theta in frange(0.0, max_angles[0], 0.1):
                # for psi in frange(0.0, -max_angles[1], -0.1):
                    placements.append((h, 0.0, theta, 0.0))
            for h in frange(0.0, max_height, 0.1):
                # for phi in frange(max_angles[0], math.pi/2, 0.1):
                for theta in frange(max_angles[0], math.pi/2, 0.1):
                # for psi in frange(-max_angles[1], -math.pi/2, -0.1):
                    placements.append((h, 0.0, theta, 0.0))


        mode = modes.Mode(self, goal_pose)
        best_releases = []
        for place in placements:
            best_releases.append(tools.make_pose(goal_pose, place[0], \
                                                     gt.euler_to_quaternion(\
                        place[1],place[2],place[3])))
        
        best_releases = sorted(best_releases, key=lambda pose: mode.fall_probability(pose)) 
#        best_releases.reverse()

        for r in best_releases:
#            print "prob="+str(mode.fall_probability(r))
            angles = gt.quaternion_to_euler(r.pose.orientation)
            c = (180 / math.pi)
            print "angles="+str(angles[0]*c)+" "+str(angles[1]*c)+" "+str(angles[2]*c)
#            print "prob="+str(mode.fall_probability(r))+" "+str(r.pose.position.z+2.8)+" angles="+str((180/math.pi)*angles[0])+" "+str((180/math.pi)*angles[1])+" "+str((180/math.pi)*angles[2])

        return best_releases

    def best_placements(self, goal_pose):
        placements = []
#        max_angles = self.max_angles(pose)
        max_angles = self.max_angles(goal_pose)
        print "max angles="+str(max_angles)
        max_height = 0.15

        angle = -5. * (math.pi/180.)
        h = 0.025
        placements.append((h, 0.0,angle,0.0))
        placements.append((h, 0.0,angle,0.0))
        placements.append((h, 0.0,angle,0.0))
        placements.append((h, 0.0,angle,0.0))
        placements.append((h, 0.0,angle,0.0))
        # placements.append((h, 0.0,0.0,angle))
        # placements.append((h, 0.0,0.0,angle)) 
        # placements.append((h, 0.0,0.0,angle)) 
        # placements.append((h, 0.0,0.0,angle)) 


        mode = modes.Mode(self, goal_pose)
        best_releases = []
        for place in placements:
            best_releases.append(tools.make_pose(goal_pose, place[0], \
                                                     gt.euler_to_quaternion(\
                        place[1],place[2],place[3])))
        
        best_releases = sorted(best_releases, key=lambda pose: mode.fall_probability(pose)) 
#        best_releases.reverse()

        for r in best_releases:
#            print "prob="+str(mode.fall_probability(r))
            angles = gt.quaternion_to_euler(r.pose.orientation)
            c = (180 / math.pi)
            print "angles="+str(angles[0]*c)+" "+str(angles[1]*c)+" "+str(angles[2]*c)
#            print "prob="+str(mode.fall_probability(r))+" "+str(r.pose.position.z+2.8)+" angles="+str((180/math.pi)*angles[0])+" "+str((180/math.pi)*angles[1])+" "+str((180/math.pi)*angles[2])

        return best_releases
