#!/usr/bin/env python

import roslib; 
roslib.load_manifest('imp_placing')
import rospy
from geometry_msgs.msg import PoseStamped
from modes import *
from place import *
from degree_of_freedom import *

class LearnProb:

    outcomes = {}

    def __init__(self, obj_type):
        self.obj = obj_type

    def add_outcome(self, pose, outcome, mode):
        self.outcomes[pose] = (outcome, mode)

    def get_outcomes(self):
        return self.outcomes

    def find_principle_outcome(self):
        dirs = [0,0,0]
        for o in outcomes:
            if o.x():
                dirs[0] += 1
            elif o.y():
                dirs[1] += 1
            elif o.z():
                dirs[2] += 1
#        principle = sorted(dirs.index(dirs.pop(max(dirs))))
        principle = dirs.index(max(dirs))
        return principle

    def psi(self, pose):
        self.find_closest
        return prob
    


#rospy.init_node("test_dropping")

