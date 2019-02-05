import math
import numpy
import scipy
import pr2_python.geometry_tools as gt
from degree_of_freedom import *

def frange(start, stop, step):
    r = start
    while r < stop:
        yield r
        r += step

class Mode:
    TIP = 0
    SLIDE = 1
    STICK = 2

    def __init__(self, obj, goal):
        self.obj = obj
        self.goal = goal
        #self.pose = pose

    def fall_probability(self, pose):
        tip_prior = self.obj.modes[0]
        stick_prior = self.obj.modes[1]

        (tip_prob, tip_dof) = self.tip_transition(tip_prior, pose)
        (stick_prob, stick_dof) = self.stick_transition(stick_prior, pose)
#        print "tip prob="+str(tip_prob)+" stick_prob="+str(stick_prob)
        total = (tip_prob[0]+tip_prob[1]+stick_prob[0]+stick_prob[1])
        prob = max(0,0, min(1.0, (1.0 - total), 1.0))
#        print "prob="+str(prob)
        return prob

    def get_degree_of_freedom(self, pose):
        tip_prior = self.obj.modes[0]
        stick_prior = self.obj.modes[1]

        (tip_prob, tip_dof) = self.tip_transition(tip_prior, pose)
        (stick_prob, stick_dof) = self.stick_transition(stick_prior, pose)
        return tip_dof.combine(stick_dof)

    def tip_transition(self, prior, pose):
        '''
        @type prob: float
        @param prob: probability of tipping over all configurations
        '''
        degrees = DegreeOfFreedom()
        tip_prob = 0.0
        if prior > 0:
            max_angles = self.obj.max_angles(self.goal)
            orien = gt.quaternion_to_euler(pose.pose.orientation)

            # height factor
            h = ((pose.pose.position.z-(-2.8)) / 2.8)
#            print "pose.z="+str(pose.pose.position.z)+" diff="+str(pose.pose.position.z+2.8)
#            print "h prob="+str(h)

            # assumes upright = 0 and only tips forward 
            # normalize angle. prob of tipping is the complement 
            if orien[0]-max_angles[0] < 0:
                x_tip = 0.0
            else:
                x_tip = ((orien[0] - max_angles[0]) / (math.pi/2 - max_angles[0]))
            if orien[1]-max_angles[1] < 0:
                y_tip = 0.0
            else:
                y_tip = ((orien[1] - max_angles[1]) / (math.pi/2 - max_angles[1]))
            
#            print "orien="+str(orien[0])+"max="+str(max_angles[0])
#            print "x_tip="+str(x_tip)+"y_tip="+str(y_tip)
            degrees.modify(0, x_tip)
            degrees.modify(1, y_tip)
            tip_prob = (x_tip+h, y_tip+h)        
            return (tip_prob, degrees) 
        else:
            return ((0.0, 0.0), degrees)

    def stick_transition(self, prior, pose):
        # sticking also proportional to angle
        degrees = DegreeOfFreedom()
        stick_prob = 0.0
        if prior > 0:
            max_angles = self.obj.max_angles(self.goal)
            orien = gt.quaternion_to_euler(pose.pose.orientation)

            # height factor
            h = (pose.pose.position.z-(-2.8)) / 2.8

            # probability of sticking is higher on low angles 
            # but sliding increases with high angles
            # for now with only stick, min sliding since we cant account
            # for it and just deal with sticking
            # assumes upright = 0 and only tips forward 
            # normalize angle. prob of tipping is the complement 


            if orien[0]-max_angles[0] < 0:
                x_stick = 1.0 # just a failure. but not really sticking
            else:
                x_stick = ((orien[0] - max_angles[0]) / (math.pi/2 - max_angles[0]))
            if orien[1]-max_angles[1] < 0:
                y_stick = 1.0
            else:
                y_stick = ((orien[1] - max_angles[1]) / (math.pi/2 - max_angles[1]))

            degrees.modify(0, y_stick)
            degrees.modify(1, x_stick)
            stick_prob = (x_stick+h, y_stick+h)        
            return (stick_prob, degrees)
        else:
            return ((0.0, 0.0), degrees)


# class Tip(MotionPrim):
#     def execute(self):
#         '''
#         Tip motion primitive.
#         describes motion of an object if it is subject to tipping.
#         resulting degrees of freedom will be rotational
#         depends on angles of release.
#         '''
#         max_angles = self.obj.max_angles(self.goal)
#         orien = gt.quaternion_to_euler(self.pose.pose.orientation)

#         x_tip = abs(orien[0] - max_angles[0])
#         y_tip = abs(orien[1] - max_angles[1])
#         print "max_angles="+str(max_angles)
#         print "x_tip="+str(x_tip)+" y_tip="+str(y_tip)
#         if (x_tip > 0) and (y_tip > 0):
#             return DegreeOfFreedom([0.0,0.0,0.0,math.pi/2,math.pi/2,0.0])
#         elif x_tip > 0:
#             return DegreeOfFreedom([0.0,0.0,0.0,math.pi/2,0.0,0.0])
#         elif y_tip > 0:
#             return DegreeOfFreedom([0.0,0.0,0.0,0.0,math.pi/2,0.0])
#         else:
#             return DegreeOfFreedom([0.0,0.0,0.0,0.0,0.0,0.0])
        
# class Slide(MotionPrim):
#     def execute(self):
#         '''
#         Slide motion primitive.
#         describes motion of object if it is subject to sliding.
#         resulting degrees of freedom will be translation. 
#         depends on forces of release. 
#         '''
#         # account for downward speed from dropping it 
#         h = self.pose.pose.position.z - self.goal.pose.position.z
#         v_z = self.vels[2]**2 + 2*9.8*h
#         self.vels = [self.vels[0], self.vels[1], v_z]
        
#         # assume if no forces, no sliding, center of mass travels in line
#         mu = 0.3 # TODO
#         normal = 9.8 * self.mobj.mass
#         friction = mu * normal
#         f_max = math.sqrt(normal**2 + friction**2)/1000.0 # in Newtons 
#         v_max = f_max/self.mobj.mass
#         angle_max = math.atan(friction/normal)
#         print "max initial velocity="+str(v_max)
#         print "vels="+str(self.vels)

#         # if (self.vels[0] > f_max) and (self.vels[1] > f_max):
#         #     angle = math.atan(self.vels[1]/self.vels[0])
#         #     if angle > f_angle_max:
#         #         d_x = (vels[0]**2)/(2*9.8*mu)
#         #         d_y = (vels[1]**2)/(2*9.8*mu)
#         #         return DegreeOfFreedom([d_x,d_y,0.0,0.0,0.0,0.0])
#         #     else:
#         #         return DegreeOfFreedom([0.0,0.0,0.0,0.0,0.0,0.0])
#         dof = DegreeOfFreedom([0.0,0.0,0.0,0.0,0.0,0.0])
#         if (self.vels[0] > v_max) or \
#                 (math.atan(self.vels[0]/self.vels[2]) > angle_max): 
#             d_x = (self.vels[0]**2)/(2*9.8*mu)
#             print "dx="+str(d_x)
#             dof =dof.modify(DegreeOfFreedom([d_x,0.0,0.0,0.0,0.0,0.0]))
#         if (self.vels[1] > v_max) or \
#                 (math.atan(self.vels[1]/self.vels[2]) > angle_max):
#             d_y = (self.vels[1]**2)/(2*9.8*mu)
#             dof = dof.modify(DegreeOfFreedom([0.0,d_y,0.0,0.0,0.0,0.0]))
#         return dof

# class Stick(MotionPrim):
#     def execute(self):
#         # self.mobj = mobj
#         # self.goal = goal
#         # self.pose = pose
#         # self.vels = vels
#         # inv = Quaternion()
#         # inv.x = -self.pose.pose.orientation.x
#         # inv.y = -self.pose.pose.orientation.y
#         # inv.z = -self.pose.pose.orientation.z
#         # inv.w = -self.pose.pose.orientation.w
#         [phi, theta, psi] = gt.quaternion_to_euler(self.pose.pose.orientation)
#         dof = DegreeOfFreedom([2.0,0.0,0.0,0.0,0.0,0.0])
#         return dof

