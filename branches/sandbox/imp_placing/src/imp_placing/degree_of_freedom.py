import math
import numpy
import copy

from numpy.linalg import *
from numpy import array, matrix
from scipy.integrate import quad
from pr2_python import geometry_tools
from geometry_msgs.msg import Quaternion, PoseStamped, Pose, Point

class DegreeOfFreedom:
    def __init__(self, degrees=[0.0,0.0,0.0]):
        '''
        Datatype representing the deterministic outcomes of modes. 
        Ranked list of the probabilites of error from the goal pose
        in the +/- x,y,z directions

        @type degrees: array
        @param degrees: list of errors in x,y,z 
        '''
        self.degrees = degrees

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

    def x(self):
        if max(self.degrees) == self.degrees[0]:
            return True
        else:
            return False

    def y(self):
        if max(self.degrees) == self.degrees[1]:
            return False
        else:
            return True

    def z(self):
        if max(self.degrees) == self.degrees[2]:
            return False
        else:
            return True

    def modify(self, i, val):
        self.degrees[i] = val

    def combine(self, dof):
        '''
        @type dof: DegreeOfFreedom
        @param dof: another dof to combine with self
        
        @rtype: DegreeOfFreedom
        @returns the maximum motion in each degree of freedom
        '''
        degs = [max(self.degrees[i],dof.degrees[i]) for i in range(len(self.degrees))]
        return DegreeOfFreedom(degs)

    def index_order(self):
        '''
        @rtype: list
        @returns ordered list of the indices of the nonzero degrees of freedom 
        '''
        order = sorted(range(len(self.degrees)), key=self.degrees.__getitem__)
        order = [item for item in order if self.degrees[item] != 0.0]
        return order

    def i_greatest(self):
        '''
        @rtype: list
        @returns list of index and value of largest degree of freedom
        '''
        i = self.index_order().pop(0)
        return (i, self.degrees[i])
