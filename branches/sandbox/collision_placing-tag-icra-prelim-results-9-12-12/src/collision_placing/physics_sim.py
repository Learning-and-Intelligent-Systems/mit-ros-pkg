import math
#import arm_navigation_msgs
import numpy
import copy

from numpy.linalg import *
from numpy import array, matrix
from scipy.integrate import quad

cor = {'wood': 0.603, "glass": 0.658, "steel": 0.597, "hollow plastic": 0.688}
g = 9.8

cons_density = lambda x,y,z,vol,mass : 1*(mass/vol)
linear_density = lambda x,y,z,vol,mass : (x+y+z)*(mass/vol)
quad_density = lambda x,y,z,vol,mass : (x**2 + y**2 + z**2)*(mass/vol)

mu_s = 0.35
mu_dynamic = 0.3
c_rr = 0.0002 # steel wheel on rail (couldnt find plastic)

class Object:
    def __init__(self, shape, mass, orien):
        self.mass = mass
        self.thickness = 0.1 # thickness of triangles in mesh
        self.shape = shape
        self.orientation = orien
        self.placements = self.best_placements()

    def volume(self):
        if self.shape.type == 1:
            volume = self.shape.dimensions[0]*self.shape.dimensions[1]*self.shape.dimension[2]
        else:
            volume = self.mesh_volume()
        return volume

    def mesh_volume(self):
        vol = 0
        for tri in self.shape.triangles:
            a = math.sqrt((tri[0]-tri[3])**2 + \
                              (tri[1]-tri[4])**2 + \
                              (tri[2]-tri[5])**2)
            b = math.sqrt((tri[3]-tri[6])**2 + \
                              (tri[4]-tri[7])**2 + \
                              (tri[5]-tri[8])**2)
            c = math.sqrt((tri[6]-tri[0])**2 + \
                              (tri[7]-tri[1])**2 + \
                              (tri[8]-tri[2])**2)
            tri_vol = (0.5 * (a+b+c)) * self.thickness
            vol += tri_vol
        return vol

    def centroid(self):
        if self.shape.type == 1:
            centroid = matrix([[float(self.shape.dimensions[0])/2], [float(self.shape.dimensions[1])/2], \
                                  [float(self.shape.dimensions[2])/2]])
        else:
            centroid = self.mesh_centroid()
        return centroid
            
    def mesh_centroid(self):
        vol = self.volume()
        cm = []
        for tri in self.shape.triangles:
            centroid = ((float(tri[0])+float(tri[3])+float(tri[6]))/3, (float(tri[1])+float(tri[4])+float(tri[7]))/3, \
                            (float(tri[2])+float(tri[5])+float(tri[8]))/3)
            cm.append(centroid)
        x_cm = math.fsum([cm_i[0] for cm_i in cm])/len(cm)
        y_cm = math.fsum([cm_i[1] for cm_i in cm])/len(cm)
        z_cm = math.fsum([cm_i[2] for cm_i in cm])/len(cm)
        return matrix([[x_cm], [y_cm], [z_cm]])

    def center_of_mass(self):
        # TODO
        return self.centroid()

    def rotate_cm(self, angle):
#        print "ANGLE="+str(angle.phi)+" "+str(angle.theta)+" "+str(angle.psi)

        rotation_matrix = matrix([[math.cos(angle.theta)*math.cos(angle.psi),
                            -math.cos(angle.phi)*math.sin(angle.psi)+\
                                math.sin(angle.phi)*math.sin(angle.theta)*math.cos(angle.psi),
                            math.sin(angle.phi)*math.sin(angle.psi)+\
                                math.cos(angle.phi)*math.sin(angle.theta)*math.cos(angle.psi)],
                           [math.cos(angle.theta)*math.sin(angle.psi), 
                            math.cos(angle.phi)*math.cos(angle.psi)+\
                                math.sin(angle.phi)*math.sin(angle.theta)*math.sin(angle.psi),
                            -math.sin(angle.phi)*math.cos(angle.psi)+\
                                math.cos(angle.phi)*math.sin(angle.theta)*math.sin(angle.psi)],
                           [-math.sin(angle.theta), 
                             math.sin(angle.phi)*math.cos(angle.theta),
                             math.cos(angle.phi)*math.cos(angle.theta)]])
        cm = self.center_of_mass()
        ans = rotation_matrix*cm
 #       print "rotated="+str(ans)
        #cm_matrix = matrix([[cm[0]],[cm[1]],[cm[2]]])
        return ans#rotation_matrix*cm

    def get_base(self):
        # assuming BOX. shape type=1
        cm = self.rotate_cm(self.orientation)
        base = (float(self.shape.dimensions[0]), float(self.shape.dimensions[1]), float(self.shape.dimensions[2]))
        # print "base="+str(base)
        # print "cm="+str(cm)
        if cm[0] >= base[0] or cm[0] <= 0:
            base = (base[0],-base[2],base[1])
        if cm[0] >= base[1] or cm[1]<= 0:
            base = (-base[2],base[1],base[0])
        return base

    def max_angles(self):
        cm = self.center_of_mass()
        base = self.get_base()
        orien = self.orientation
#       orien = self.quat_to_orien(self.orientation)

        max_phi = math.pi/2
        max_theta = math.pi/2

        for phi in frange(0, math.pi/2, 0.1):
            cm_new = self.rotate_cm(Orientation(orien.phi-phi, orien.theta, orien.psi))
            if abs(cm_new[1][0]) >= base[1]:
                max_phi = phi
                break
        for theta in frange(0, math.pi/2, 0.1):
            cm_new = self.rotate_cm(Orientation(orien.phi, orien.theta+theta, orien.psi))
            if abs(cm_new[0][0]) >= base[0]:
                max_theta = theta
                break
                                 
        return (max_phi, max_theta)

    def best_placements(self):
        placements = []
        max_angles = self.max_angles()
#        print "max_angles="+str(max_angles)

        for h in frange(0.0, 0.5, 0.1):
            for phi in frange(0.0, max_angles[0], 0.1):
                for theta in frange(0.0, max_angles[1], 0.1):
#                    print "h="+str(h)+" phi="+str(phi)+" theta="+str(theta)
                    placements.append((h, self.orien_to_quat(Orientation(phi, theta, 0.0))))
        return placements

    def quat_to_orien(self, quat):
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        ox = math.atan2(2*(w*x + y*z), 1-2(x**2 + y**2))
        oy = math.asin(2*(w*y - z*x))
        oz = math.atan2(2*(w*z + x*y), 1-2*(y**2 + z**2))

        return Orientation(ox, oy, oz)

    def orien_to_quat(self, orien):
        x = orien.phi
        y = orien.theta
        z = orien.psi

        qw = math.cos(x/2)*math.cos(y/2)*math.cos(z/2) + math.sin(x/2)*math.sin(y/2)*math.sin(z/2)
        qx = math.sin(x/2)*math.cos(y/2)*math.cos(z/2) - math.cos(x/2)*math.sin(y/2)*math.sin(z/2)
        qy = math.cos(x/2)*math.sin(y/2)*math.cos(z/2) + math.sin(x/2)*math.cos(y/2)*math.sin(z/2)
        qz = math.cos(x/2)*math.cos(y/2)*math.sin(z/2) - math.sin(x/2)*math.sin(y/2)*math.cos(z/2)

        return (qw, qx, qy, qz)



    # only need to worry about it sliding (so plate objects) check how the pr2 would release a book. too much friction? does it even slide? 
# 2 part release: set it on a corner then release the rest
    def degree_of_freedom(self, angle, velocity, contact):
        cm = self.center_of_mass()

        # define the forces on the body
        f_gravity = Vector(0.0, 0.0, -self.mass*9.8)

        f_friction = Vector(c_rr * f_gravity.magnitude(), 0.0, 0.0) # TODO direction of motion

        r = norm(cm-contact.matrix)
        tau = cross(r, f_friction.matrix) # torque from friction


        if self.stable(angle) and (velocity.magnitude()==0.0):
            return Vector(0.0, 0.0, 0.0)

        elif not self.stable(angle) and (velocity.magnitude() == 0.0):
            pass

        elif velocity.magnitude() != 0.0:
            return velocity.unit_vector()

    def place(self):
        # no angle or velocity
        orien_0 = Orientation(0.0, 0.0, 0.0)
        vel_0 = Velocity(0.0, Vector(0.0, 0.0, 0.0))
        print self.degree_of_freedom(orien_0, vel_0)

        # angle
        orien_1 = Orientation(math.pi/4, 0.0, 0.0)
        print self.degree_of_freedom(orien_1, vel_0)


class Orientation:
    def __init__(self, phi, theta, psi):
        self.phi = phi
        self.theta = theta
        self.psi = psi

class Vector:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.matrix = matrix([x, y, z])

    def get(self):
        return(self.x, self.y, self.z)

    def magnitude(self):
        return math.sqrt(x**2 + y**2 + z**2)

    def unit_vector(self):
        mag = self.magnitude()
        return Vector(self.x/mag, self.y/mag, self.z/mag)

    def opposite(self):
        return Vector(-self.x, -self.y, -self.z)
        

def center_of_mass(mass, density, x_min, x_max, y_min, y_max, z_min, z_max):
    inc = 0.1
    vol = (x_max-x_min)/inc * (y_max-y_min)/inc * (z_max-z_min)/inc

    x_mass = [math.fsum([density(x,y,z,vol,mass) \
                            for y in frange(y_min, y_max, inc) \
                            for z in frange(z_min, z_max, inc)]) \
                  for x in frange(x_min, x_max, inc)]
    y_mass = [math.fsum([density(x,y,z,vol,mass) \
                            for x in frange(x_min, x_max, inc) \
                            for z in frange(z_min, z_max, inc)]) \
                  for y in frange(y_min, y_max, inc)]
    z_mass = [math.fsum([density(x,y,z,vol,mass) \
                            for x in frange(x_min, x_max, inc) \
                            for y in frange(y_min, y_max, inc)]) \
                  for z in frange(z_min, z_max, inc)]

    x_ind = [x for x in frange(x_min, x_max, inc)]
    x_index = [(x_ind[i+1]+x_ind[i])/2 for i in range(len(x_ind)-1)] \
        + [(x_max+x_ind[-1])/2]
    y_ind = [y for y in frange(y_min, y_max, inc)]
    y_index = [(y_ind[i+1]+y_ind[i])/2 for i in range(len(y_ind)-1)] \
        + [(y_max+y_ind[-1])/2]
    z_ind = [z for z in frange(z_min, z_max, inc)]
    z_index = [(z_ind[i+1]+z_ind[i])/2 for i in range(len(z_ind)-1)] \
        + [(z_max+z_ind[-1])/2]

    x_cm = math.fsum([x_mass[i]*x_index[i] for i in range(len(x_index))])/mass
    y_cm = math.fsum([y_mass[i]*y_index[i] for i in range(len(y_index))])/mass
    z_cm = math.fsum([z_mass[i]*z_index[i] for i in range(len(z_index))])/mass

    return (x_cm,y_cm,z_cm)

def geometric_center(x, y, z):
    return (x/2, y/2, z/2)

def center_of_mass_cons(verts):
    norm = 1.0/len(verts)
    x = norm * math.fsum([verts[i][0] for i in range(len(verts))])
    y = norm * math.fsum([verts[i][1] for i in range(len(verts))])
    z = norm * math.fsum([verts[i][2] for i in range(len(verts))])
    return (x,y,z)

def stable(cm, pivot):
    if cm>pivot:
        return False
    return True
    # cm_r = math.sqrt(cm[0]**2 + cm[1]**2)
    # x_r = base_x[1]-base_x[0]
    # y_r = base_y[1]-base_y[0]
    # if (cm[0] > x_r) or (cm[1] > y_r):
    #     return False 
    # return True

def rotate(pt, phi, theta, psi):
    # z: psi, y: theta, x: phi
    new = geometry_msgs.msg.Point()
    new.x = pt.x*(math.cos(theta)*math.cos(psi)) + \
        pt.y*(-math.cos(phi)*math.sin(psi) + \
                   math.sin(phi)*math.sin(theta)*math.cos(psi)) + \
        pt.z*(math.sin(phi)*math.sin(psi) + \
                  math.cos(phi)*math.sin(theta)*math.cos(psi))
    new.y = pt.x*(math.cos(theta)*math.sin(psi)) + \
        pt.y*(math.cos(phi)*math.cos(psi) + \
                  math.sin(phi)*math.sin(theta)*math.sin(psi)) + \
        pt.z*(-math.sin(phi)*math.cos(psi) + \
                   math.cos(phi)*math.sin(theta)*maht.sin(psi))
    new.z = pt.x*(-math.sin(theta)) + \
        pt.y*(math.sin(phi)*math.cos(theta)) + \
        pt.z*(math.cos(phi)*math.cos(theta))
    return new

def get_pivots(mesh, phi, theta, psi):
    # rotation around 
    pivot = mesh.vertices[0]
    lowest = rotate(mesh.vertices[0], phi, theta, psi)
    for pt in mesh.vertices:
        new_pt = rotate(pt, phi, theta, psi)
        if new_pt.z < lowest:
            pivot = pt
        # elif new_pt.z == lowest:
        #     pivots.append(pt)
    return pivot

def max_angle(obj, axis):
    #cm = center_of_mass(obj) farts
#    print "object="+str(obj)
    cm = geometric_center(obj[7][0]-obj[0][0], obj[7][1]-obj[0][1], obj[7][2]-obj[7][2])
    print "center of mass="+str(cm)
    inc = 0.1

    if axis == "+x":
        r = math.sqrt(cm[0]**2 + cm[2]**2)
        theta = math.atan(cm[2]/cm[0])
        print str(r)+" "+str(theta)
        p = max([obj[i][0] for i in range(len(obj))])
        for phi in frange(theta, theta+(math.pi/2), inc):
            x = (r * math.cos(phi))
            if x<=0:
                #print phi
                return (phi-theta)
    elif axis == "+y":
        r = math.sqrt(cm[1]**2 + cm[2]**2)
        theta = math.atan(cm[2]/cm[1])
        p = max([obj[i][1] for i in range(len(obj))])
        for phi in frange(theta, theta+(math.pi/2), inc):
            y = r * math.cos(phi)      
            if y<=0:
                return (phi-theta)
    elif axis == "-x":
        r = math.sqrt(cm[0]**2 + cm[2]**2)
        theta = math.atan(cm[2]/cm[0])
        p = min([obj[i][0] for i in range(len(obj))])
        for phi in frange(theta, theta+(math.pi/2), inc):
            x = r * math.cos(phi)
            if x<=0:
                return (phi-theta)
    elif axis == "-y":
        r = math.sqrt(cm[1]**2 + cm[2]**2)
        theta = math.atan(cm[2]/cm[1])
        p = min([obj[i][1] for i in range(len(obj))])
        for phi in frange(theta, theta+(math.pi/2), inc):
            y = r * math.cos(phi)
            if y<=0:
                return (phi-theta)

def bounce(cor, theta_0, vel_0):
    theta_1 = 180 - theta_0
    vel_1 = cor * vel_0
    return (theta_1, vel_1)

def frange(start, stop, step):
    r = start
    while r < (stop-.000001):
        yield r
        r += step

def test():
    obj = [(0.0,0.0,0.0), (0.4,0.0,0.0), (0.0,0.4,0.0), (0.4,0.4,0.0), \
               (0.0,0.0,0.4),(0.4,0.0,0.4),(0.0,0.4,0.4),(0.4,0.4,0.4)]
    print "+x="+str( max_angle(obj, "+x"))
    print "+y="+str( max_angle(obj, "+y"))
    print "-x="+str( max_angle(obj, "-x"))
    print "-y="+str( max_angle(obj, "-y"))

#print center_of_mass(1, cons_density, 1.0, 2.0, 1.0, 2.0, 1.0, 2.0)
#test()
