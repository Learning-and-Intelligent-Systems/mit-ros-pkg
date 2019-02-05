import math
import numpy
import copy

from motion_prim import *
import physics
from numpy.linalg import *
from numpy import array, matrix
from scipy.integrate import quad
from pr2_python import geometry_tools

def frange(start, stop, step):
    r = start
    while r < (stop-.000001):
        yield r
        r += step

class Mobject:

    def __init__(self, mass, shape, motion_prims):
        '''
        @type mass: float
        @param mass: mass of the object
        @type shape: arm_navigation_msgs.msg.Shape
        @param shape: either basic (box, cylinder,etc) or mesh
        @type motion_prims: array
        @param motion_prims: list of expected motion prims (an enum)
        '''
        self.mass = mass
        self.shape = shape
        self.motion_prims = motion_prims
        self.object = physics.Object(self.shape, self.mass, \
                                         physics.Orientation(0.0, 0.0, 0.0))

    def find_dof(self, goal, pose, vels=[0.0,0.0,0.0]):
        '''

        @type goal: PoseStamped
        @param goal: the final goal pose of the object (on a table, etc)
        @type pose: PoseStamped
        @param pose: the pose the object is being released from
        @type vels: array
        @param vels: the x,y,z velocity applied to the object as it is released
        
        @rtype DegreeOfFreedom
        @returns the x,y,z rotational and translation distance the object
                 will travel beyond the goal pose
        '''
        dof = DegreeOfFreedom([0.0,0.0,0.0,0.0,0.0,0.0])
        for prim in self.motion_prims:
            if prim == MotionPrim.TIP:
                tipper = Tip(self, goal, pose)
                dof = dof.modify(tipper.execute())
            if prim == MotionPrim.SLIDE:
                slider = Slide(self, goal, pose, vels)
                dof = dof.modify(slider.execute())
            if prim == MotionPrim.STICK:
                sticker = Stick(self, goal, pose)
                dof = dof.modify(sticker.execute())
            if prim == MotionPrim.BOUNCE:
                bouncer = Bounce(self, goal, pose)
                dof = dof.modify(bouncer.execute())
        return dof
