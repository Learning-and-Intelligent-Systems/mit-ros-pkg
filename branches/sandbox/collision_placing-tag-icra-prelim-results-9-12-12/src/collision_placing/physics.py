import math
import numpy
import copy

from numpy.linalg import *
from numpy import array, matrix
from scipy.integrate import quad
from pr2_python import geometry_tools
from geometry_msgs.msg import Quaternion, PoseStamped, Pose, Point

g = 9.8

cons_density = lambda x,y,z,vol,mass : 1*(mass/vol)
linear_density = lambda x,y,z,vol,mass : (x+y+z)*(mass/vol)
quad_density = lambda x,y,z,vol,mass : (x**2 + y**2 + z**2)*(mass/vol)

def frange(start, stop, step):
    r = start
    while r < (stop-.000001):
        yield r
        r += step

class Orientation:
    def __init__(self, phi, theta, psi):
        self.phi = phi
        self.theta = theta
        self.psi = psi

    def to_quat(self):
        x = self.phi
        y = self.theta
        z = self.psi

        qw = math.cos(x/2)*math.cos(y/2)*math.cos(z/2) + math.sin(x/2)*math.sin(y/2)*math.sin(z/2)
        qx = math.sin(x/2)*math.cos(y/2)*math.cos(z/2) - math.cos(x/2)*math.sin(y/2)*math.sin(z/2)
        qy = math.cos(x/2)*math.sin(y/2)*math.cos(z/2) + math.sin(x/2)*math.cos(y/2)*math.sin(z/2)
        qz = math.cos(x/2)*math.cos(y/2)*math.sin(z/2) - math.sin(x/2)*math.sin(y/2)*math.cos(z/2)

        return (qw, qx, qy, qz)
    
    def string(self):
        cons = 180 / math.pi
        return str(self.phi*cons)+", "+str(self.theta*cons)+", "+str(self.psi*cons)
    
class DegreeOfFreedom:
    def __init__(self, degrees):
        '''
        datatype to hold information about the degrees of freedom
        of an object resulting from a certain release and type of 
        object. each dimension represents the distance the object
        will travel beyond the goal pose. 

        @type degrees: array
        @param degrees: list of 
        '''
        self.x_trans = degrees[0]
        self.y_trans = degrees[1]
        self.z_trans = degrees[2]
        self.x_rot = degrees[3]
        self.y_rot = degrees[4]
        self.z_rot = degrees[5]
        self.degrees = [self.x_trans, self.y_trans, self.z_trans, \
                            self.x_rot, self.y_rot, self.z_rot]

    def __getitem__(self,i):
        return self.degrees[i]

    def isZero(self):
        '''
        @rtype: Boolean
        @returns whether there is any non-zero degree of freedom 
        '''
        for degree in self.degrees:
            if degree != 0.0:
                return False
        return True
    
    def noRot(self):
        '''
        @rtype: Boolean
        @returns whether there is any non-zero rotational degree of freedom 
        '''
        if self.x_rot==0 and self.y_rot==0 and self.z_rot==0:
            return True
        return False

    def noTrans(self):
        '''
        @rtype: Boolean
        @returns whether there is any non-zero translational degree of freedom 
        '''
        if self.x_trans==0 and self.y_rot==0 and self.z_rot==0:
            return True
        return False

    def modify(self, dof):
        '''
        @type dof: DegreeOfFreedom
        @param dof: another dof to combine with self
        
        @rtype: DegreeOfFreedom
        @returns the maximum motion in each degree of freedom
        '''
        degs = []
        for i in range(len(self.degrees)):
            if self.degrees[i] != dof.degrees[i]:
                degs.append(max([self.degrees[i], dof.degrees[i]]))
            else:
                degs.append(self.degrees[i])
        return DegreeOfFreedom(degs)

    def index_order(self):
        order = sorted(range(len(self.degrees)), key=self.degrees.__getitem__)
        order = [item for item in order if self.degrees[item] != 0.0]
        return order

        # for i in order:
        #     if i == 0.0:
        #         order.remove(i)
        # return order

    def i_greatest(self):
        b = [0, 0.0]
        for i in range(len(self.degrees)):
            if self.degrees[i] > b[1]:
                b = [i, self.degrees[i]]
        return b
     
class Object:
    def __init__(self, shape, mass, orien):
        self.mass = mass
        self.thickness = 0.1 #thickness of triangles in mesh
        self.shape = shape
        self.orientation = orien
#        self.placements = self.best_placements()
  
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
            volume = self.shape.dimensions[0]*self.shape.dimensions[1]*self.shape.dimension[2]
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
            triangle_indices.append([self.shape.triangles[3*i], self.shape.triangles[3*i+1], self.shape.triangles[3*i+2]])
        for tri in triangle_indices:
           # print "TRIANGLE="+str(tri)
           # print "triangle vertices="+str((self.shape.vertices[tri[0]], self.shape.vertices[tri[1]], self.shape.vertices[tri[2]]))
            p1 = self.shape.vertices[tri[0]]
            p2 = self.shape.vertices[tri[1]]
            p3 = self.shape.vertices[tri[2]]
            # p1 = Vector(self.shape.vertices[tri[0]], self.shape.vertices[tri[1]], self.shape.vertices[tri[2]])
            # p2 = Vector(tri[1], tri[4], tri[7])
            # p3 = Vector(tri[2], tri[5], tri[8])

            v321 = p3.x * p2.y * p1.z
            v231 = p2.x * p3.y * p1.z
            v312 = p3.x * p1.y * p2.z
            v132 = p1.x * p3.y * p2.z
            v213 = p2.x * p1.y * p3.z
            v123 = p1.x * p2.y * p3.z

            det = -v321 + v231 + v312 - v132 - v213 + v123
            vol += det * (1.0/6.0)
        return vol

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
            dim = [float(self.shape.dimensions[0]), float(self.shape.dimensions[1]), float(self.shape.dimensions[2])]
            centroid = matrix([[0.0], [0.0], [0.0]])
            # centroid = matrix([[float(self.shape.dimensions[0])/2], \
            #                        [float(self.shape.dimensions[1])/2], \
            #                        [float(self.shape.dimensions[2])/2]])
        elif self.shape.type == 2: 
            # CYLINDER:=type 
            # radius:=dimensions[0] (centered at (0,0))
            # length:=dimensions[1] (along Z axis)
            centroid = matrix([[0.0], [0.0], [self.shape.dimensions[1]/2]])
        else:
            # MESH:=type
            centroid = self.mesh_centroid()
        #centroid = self.translate(centroid.tolist(), pose)
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
        # TODO
        return self.centroid(pose)

    def rotate_x(self, phi, pt):
        rotation_matrix = matrix([[1.0,0.0,0.0], \
                                      [0.0,math.cos(phi),-math.sin(phi)], \
                                      [0.0,math.sin(phi),math.cos(phi)]])
        pt_matrix = matrix([[pt[0]],[pt[1]],[pt[2]]])
        rotated = rotation_matrix*pt_matrix
        return rotated.tolist()
                                  
                                 

    def rotate_pt(self, angle, pt):
#        print "ANGLE="+str(angle.phi)+" "+str(angle.theta)+" "+str(angle.psi)
        rotation_matrix = matrix([[math.cos(angle.theta)*math.cos(angle.psi),
                            -math.cos(angle.phi)*math.sin(angle.psi)+\
                                math.sin(angle.phi)*math.sin(angle.theta)*\
                                       math.cos(angle.psi),
                            math.sin(angle.phi)*math.sin(angle.psi)+\
                                math.cos(angle.phi)*math.sin(angle.theta)*\
                                       math.cos(angle.psi)],
                           [math.cos(angle.theta)*math.sin(angle.psi), 
                            math.cos(angle.phi)*math.cos(angle.psi)+\
                                math.sin(angle.phi)*math.sin(angle.theta)*\
                                math.sin(angle.psi),
                            -math.sin(angle.phi)*math.cos(angle.psi)+\
                                math.cos(angle.phi)*math.sin(angle.theta)*\
                                math.sin(angle.psi)],
                           [-math.sin(angle.theta), 
                             math.sin(angle.phi)*math.cos(angle.theta),
                             math.cos(angle.phi)*math.cos(angle.theta)]])
#        print "rotation matrix="+str(rotation_matrix)
        #cm = self.center_of_mass()
        pt_matrix = matrix([[pt[0]], [pt[1]], [pt[2]]])
        rotated = rotation_matrix*pt_matrix
#        print "pt matrix="+str(pt_matrix)
#        print "rotated pt="+str(rotated)
        return rotated.tolist()

    def get_base(self, rotation):
        bb = geometry_tools.bounding_box_corners(self.shape) # in frame of obj
        transformed_bb = []
        
        # transform bounding box to frame of the robot
        #print "rotation="+str(rotation)
        for pt in bb:
            #print "pt="+str(pt)
            transformed_pt = geometry_tools.transform_list(pt, rotation)
            transformed_bb.append(transformed_pt)

        # find base in this frame
        transformed_bb.sort()
        (minxminyminz, minxminymaxz, minxmaxyminz, minxmaxymaxz, \
             maxxminyminz, maxxminymaxz, maxxmaxyminz, maxxmaxymaxz) = transformed_bb

        return [minxminyminz, minxmaxyminz, maxxminyminz, maxxmaxyminz]

#        return [bb[0][0],bb[4][0], bb[0][1],bb[2][1], bb[0][2],bb[1][2]]

    def max_angles(self, pose):
        '''
        @type pose: PoseStamped
        @param pose: the pose the object will be released in (in the frame of the robot)
        '''

        ## get the center of mass in the frame of the object, 
        ## then transform it into the robot frame
        cm = self.center_of_mass(pose).tolist() ## in the frame of the object
        tf_cm  = self.translate(cm, pose)
        
        ## get the corners of the base of the bounding box of the object in robot frame
        base = self.get_base(pose.pose)

#        print "base="+str([pivot.position.x, pivot.position.y, pivot.position.z])
        print "base="+str(base)
        ## corner of the base is the pivot pt, transform to obj frame
        # pivot_matrix = self.inv_translate([[base[3][0]],[base[3][1]],[base[3][2]]], pose).tolist()
        # pivot = [pivot_matrix[0][0], pivot_matrix[1][0], pivot_matrix[2][0]]

        ## get the euler angles of the pose (goal pose)
        (orig_phi, orig_theta, orig_psi) = geometry_tools.quaternion_to_euler(pose.pose.orientation)

        ## max rotation of pi/2
        max_phi = orig_phi + math.pi/2
        max_theta = orig_theta + math.pi/2

        pivot = Point()
        pivot.x = base[0][0]
        pivot.y = base[0][1]
        pivot.z = base[0][2]

        cm = Pose()
        cm.position.x = tf_cm[0][0]
        cm.position.y = tf_cm[1][0]
        cm.position.z = tf_cm[2][0]
        cm.orientation = pose.pose.orientation

        t = geometry_tools.inverse_transform_point(pivot, cm)

        tol=0.01
        ## find max angle around x-axis, phi
        print "cm="+str(tf_cm.tolist())
        for phi in frange(0.0, math.pi/2, 0.1):
            transform = Pose()
            transform.orientation = geometry_tools.euler_to_quaternion(phi, 0.0, 0.0)
            
            cm_new = Pose()
            cm_new.position = cm.position
            cm_new.orientation = geometry_tools.transform_quaternion(cm.orientation, transform)

            p_new = geometry_tools.transform_point(t, cm_new)
            
            y_diff = abs(tf_cm[1][0]-p_new.y)
            z_diff = abs(tf_cm[2][0]-p_new.z)
#            print "phi angle="+str(phi)+" pivot="+str([p_new.x, p_new.y, p_new.z])
           
            if y_diff < tol or z_diff < tol:
                max_phi = phi
                break 

        for theta in frange(0.0, math.pi/2, 0.1):
            transform = Pose()
            transform.orientation = geometry_tools.euler_to_quaternion(0.0, theta, 0.0)
            
            cm_new = Pose()
            cm_new.position = cm.position
            cm_new.orientation = geometry_tools.transform_quaternion(cm.orientation, transform)

            p_new = geometry_tools.transform_point(t, cm_new)
            
            x_diff = abs(tf_cm[0][0]-p_new.x)
            z_diff = abs(tf_cm[2][0]-p_new.z)
#            print "theta angle="+str(theta)+" pivot="+str([p_new.x, p_new.y, p_new.z])
            
            if z_diff < tol or x_diff < tol:
                max_theta = theta
                break 

        return (max_phi, max_theta)


        # #[minx,maxx,miny,maxy,minz,maxz]
        # base = self.get_base(Orientation(0.0,0.0,0.0)) 
        # #orien = self.orientation
        # orien = Orientation(0.0,0.0,0.0)
        
        # print "base="+str(base)
        # print "cm="+str(cm)
        # print "orientation="+orien.string()

        # max_phi = math.pi/2
        # max_theta = math.pi/2
        # max_psi = math.pi/2

        # for phi in frange(0, math.pi/2, 0.1):
        #     # new = self.rotate_pt(Orientation(orien.phi-phi, orien.theta, \
        #     #                                      orien.psi), \
        #     #                          [base[1],base[3],base[5]])
        #     new = self.rotate_x(orien.phi-phi, \
        #                             [base[1],base[3],base[5]])
        #     # print "FULL: cm="+str(cm)+" base="+str(new)
        #     # print "cm="+str(cm[2][0])+" base="+str(new[2][0])
        #     if cm[2][0] >= new[2][0]:
        #         max_phi = phi
        #         break
        # for theta in frange(0, math.pi/2, 0.1):
        #     new = self.rotate_pt(Orientation(orien.phi, orien.theta+theta, \
        #                                          orien.psi), \
        #                              [base[1],base[3],base[5]])
        #     # print "FULL: cm="+str(cm)+" base="+str(new)
        #     # print "cm="+str(cm[2][0])+" base="+str(new[2][0])
        #     if cm[2][0] >= new[2][0]:
        #         max_theta = theta
        #         break
        # for psi in frange(0, math.pi/2, 0.1):
        #     new = self.rotate_pt(Orientation(orien.phi, orien.theta, \
        #                                          orien.psi+psi), \
        #                              [base[1],base[3],base[5]])
        #     # print "FULL: cm="+str(cm)+" base="+str(new)
        #     # print "cm="+str(cm[1][0])+" base="+str(new[1][0])
        #     if cm[0][0] >= new[0][0]:
        #         max_psi = psi
        #         break
                    
        # return (max_phi, max_theta)

    def best_placements(self, pose):
        placements = []
#        max_angles = self.max_angles(pose)
        max_angles = self.max_angles(pose)
        print "max angles="+str(max_angles)
        max_height = 0.5

        for h in frange(0.0, max_height, 0.1):
            for phi in frange(0.0, math.pi/2, 0.1):
                for theta in frange(0.0, math.pi/2, 0.1):
#                    print "h="+str(h)+" phi="+str(phi)+" theta="+str(theta)
                    #print "QUAT="+str(Orientation(phi, theta, 0.0).to_quat())
                    placements.append((h, geometry_tools.euler_to_quaternion(phi,theta,0.0)))        
#                    placements.append((h, Orientation(phi, theta, 0.0).to_quat()))        
        return placements

        # only returns placements that are acceptable
#         for h in frange(0.0, max_height, 0.1):
#             for phi in frange(0.0, max_angles[0], 0.1):
#                 for theta in frange(0.0, max_angles[1], 0.1):
# #                    print "h="+str(h)+" phi="+str(phi)+" theta="+str(theta)
#                     #print "QUAT="+str(Orientation(phi, theta, 0.0).to_quat())
#                     placements.append((h, Orientation(phi, theta, 0.0).to_quat()))
#         return placements

