import math
import numpy
import scipy
import pr2_python.geometry_tools as gt

#import trajectory 
from physics import *

def frange(start, stop, step):
    r = start
    while r < stop:
        yield r
        r += step
    
class MotionPrim:
    TIP = 0
    SLIDE = 1
    STICK = 2
    BOUNCE = 3

    def __init__(self, mobj, goal, pose, vels=[0.0,0.0,0.0]):
        '''
        super class for all motion primitives: tip, slide, stick, bounce

        @type mobj: Mobject
        @param mobj: describes the object being manipulated
        @type goal: PoseStamped
        @param goal: the final goal pose of the object (on a table, etc)
        @type pose: PoseStamped
        @param pose: the pose the object is being released from
        @type vels: array
        @param vels: the x,y,z velocity applied to the object as it is released.
                     assume velocities applied for t=1sec before release
        '''
        self.mobj = mobj
        self.goal = goal
        self.pose = pose
        self.vels = vels
        
    def execute(self):
        '''
        every MotionPrim will have an execute method

        @rtype: DegreeOfFreedom
        @returns the distance in each degree of freedom that the
                 object will move beyond the goal pose 
        '''
        print "EXECUTE NOT IMPLEMENTED"

class Tip(MotionPrim):
    def execute(self):
        '''
        Tip motion primitive.
        describes motion of an object if it is subject to tipping.
        resulting degrees of freedom will be rotational
        depends on angles of release.
        '''
        max_angles = self.mobj.object.max_angles(self.goal)
        orien = gt.quaternion_to_euler(self.pose.pose.orientation)
#        orien = quat_to_orien(self.pose.pose.orientation)
        #orien = Orientation(0.0, math.pi/2,0.0)

        x_tip = abs(orien[0] - max_angles[0])
        y_tip = abs(orien[1] - max_angles[1])
        print "max_angles="+str(max_angles)
        print "x_tip="+str(x_tip)+" y_tip="+str(y_tip)
        if (x_tip > 0) and (y_tip > 0):
            return DegreeOfFreedom([0.0,0.0,0.0,math.pi/2,math.pi/2,0.0])
        elif x_tip > 0:
            return DegreeOfFreedom([0.0,0.0,0.0,math.pi/2,0.0,0.0])
        elif y_tip > 0:
            return DegreeOfFreedom([0.0,0.0,0.0,0.0,math.pi/2,0.0])
        else:
            return DegreeOfFreedom([0.0,0.0,0.0,0.0,0.0,0.0])
        
class Slide(MotionPrim):
    def execute(self):
        '''
        Slide motion primitive.
        describes motion of object if it is subject to sliding.
        resulting degrees of freedom will be translation. 
        depends on forces of release. 
        '''
        # account for downward speed from dropping it 
        h = self.pose.pose.position.z - self.goal.pose.position.z
        v_z = self.vels[2]**2 + 2*9.8*h
        self.vels = [self.vels[0], self.vels[1], v_z]
        
        # assume if no forces, no sliding, center of mass travels in line
        mu = 0.3 # TODO
        normal = 9.8 * self.mobj.mass
        friction = mu * normal
        f_max = math.sqrt(normal**2 + friction**2)/1000.0 # in Newtons 
        v_max = f_max/self.mobj.mass
        angle_max = math.atan(friction/normal)
        print "max initial velocity="+str(v_max)
        print "vels="+str(self.vels)

        # if (self.vels[0] > f_max) and (self.vels[1] > f_max):
        #     angle = math.atan(self.vels[1]/self.vels[0])
        #     if angle > f_angle_max:
        #         d_x = (vels[0]**2)/(2*9.8*mu)
        #         d_y = (vels[1]**2)/(2*9.8*mu)
        #         return DegreeOfFreedom([d_x,d_y,0.0,0.0,0.0,0.0])
        #     else:
        #         return DegreeOfFreedom([0.0,0.0,0.0,0.0,0.0,0.0])
        dof = DegreeOfFreedom([0.0,0.0,0.0,0.0,0.0,0.0])
        if (self.vels[0] > v_max) or \
                (math.atan(self.vels[0]/self.vels[2]) > angle_max): 
            d_x = (self.vels[0]**2)/(2*9.8*mu)
            print "dx="+str(d_x)
            dof =dof.modify(DegreeOfFreedom([d_x,0.0,0.0,0.0,0.0,0.0]))
        if (self.vels[1] > v_max) or \
                (math.atan(self.vels[1]/self.vels[2]) > angle_max):
            d_y = (self.vels[1]**2)/(2*9.8*mu)
            dof = dof.modify(DegreeOfFreedom([0.0,d_y,0.0,0.0,0.0,0.0]))
        return dof

class Stick(MotionPrim):
    def execute(self):
        pass

class Bounce(MotionPrim):
    def execute(self):
        pass


# class MotionPrim:
    
#     def __init__(self, obj):
#         self.object = obj

#     def tip(self, goal):
#         max_height = 0.5
#         cm = self.object.center_of_mass()
#         base = self.object.get_base(physics.Orientation(0.0,0.0,0.0)) 
#         orien = physics.Orientation(0.0,0.0,0.0)
        
#         max_phi = math.pi/2
#         max_theta = math.pi/2

#         for phi in frange(0, math.pi/2, 0.1):
#             cm_new = self.rotate_cm(physics.Orientation(phi, 0.0, 0.0))
#             if abs(cm_new[1][0]) >= abs(base[3]):
#                 max_phi = phi
#                 break
#         for theta in frange(0, math.pi/2, 0.1):
#             cm_new = self.rotate_cm(physics.Orientation(0.0, theta, 0.0))
#             if abs(cm_new[0][0]) >= abs(base[1]):
#                 max_theta = theta
#                 break
        
#         traj_set = []
#         for h in frange(0.0, max_height, 0.1):
#             for phi in frange(0.0, max_phi, 0.1):
#                 for theta in frange(0.0, max_theta, 0.1):
                    
#                     orien = Orientation(phi, theta, 0.0).to_quat()
                    
#                     pose = PoseStamped()
#                     pose.header.frame_id == "\torso_lift_link"
#                     pose.position.x = goal.x
#                     pose.position.y = goal.y
#                     pose.position.z = goal.z + h
#                     pose.orientation.x = orien.x
#                     pose.orientation.y = orien.y
#                     pose.orientation.z = orien.z
#                     pose.orientation.w = orien.w

#                     forces = [0.0,0.0,0.0]
#                     traj_set.append(trajectory.Trajectory(pose, forces))
#         return traj_set

#     def slide(self, mu):
#         normal = 9.8 * self.obj.mass
#         friction = mu * normal
#         f_max = math.sqrt(normal**2 + friction**2)
#         f_angle_max = math.atan(friction/normal)

#         for h in frange(0.0, 0.5, 0.1):
#             for phi in frange(0.0, f_angle_max, 0.1):
#                 for theta in frange(0.0, f_angle_max, 0.1):
#                     for f in frange(0.0, f_max, 0.1):
                    
#                         orien = Orientation(phi, theta, 0.0).to_quat()
                    
#                         pose = PoseStamped()
#                         pose.header.frame_id == "\torso_lift_link"
#                         pose.position.x = goal.x
#                         pose.position.y = goal.y
#                         pose.position.z = goal.z + h
#                         pose.orientation.x = 0.0
#                         pose.orientation.y = 0.0
#                         pose.orientation.z = 0.0
#                         pose.orientation.w = 1.0

#                         forces = []
#                         traj_set.append(trajectory.Trajectory(pose, forces))
#         return traj_set

#     def stick(self, mu):
#         normal = 9.8 * self.obj.mass
#         friction = mu * normal
#         f_min = math.sqrt(normal**2 + friction**2)
#         f_angle_max

#     def bounce(self):
#         pass

