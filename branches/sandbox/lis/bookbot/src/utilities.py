#!/usr/bin/python
import roslib
roslib.load_manifest('pr2_pick_and_place_tutorial_python')
import rospy

# import libraries
import actionlib
import tf
import scipy
import math
from math import *
import random
import time
import sys
import threading

# import utitlies
from object_manipulator.convert_functions import *
from object_manipulator.draw_functions import *

def pause():
    print 'press enter to continue'
    raw_input()

def qx(t):
    return scipy.matrix([[1., 0., 0.,0.],
                         [0., cos(t), -sin(t),0.],
                         [0., sin(t), cos(t),0.],
                         [0.,0.,0.,1.]])

def qy(t):
    return scipy.matrix([[cos(t), 0., sin(t),0.],
                         [0., 1., 0.,0.],
                         [-sin(t), 0., cos(t),0.],
                         [0.,0.,0.,1.]])

def qz(t):
    return scipy.matrix([[cos(t), -sin(t), 0.,0.],
                         [sin(t), cos(t), 0.,0.],
                         [0., 0., 1.,0.],
                         [0.,0.,0.,1.]])

def draw_grasp(df,pose):
    for i in xrange(3):
        df.draw_grasps([pose],frame='base_link')
    print 'uncheck and recheck marker in rviz'
    pause()
    for i in xrange(3):
        df.draw_grasps([pose],frame='base_link')

def draw_bb(df,pose_mat, dims):
    for i in xrange(3):
        df.draw_rviz_box(pose_mat, [dims.x,dims.y,dims.z], frame = 'base_link')
    print 'uncheck and recheck marker in rviz'
    pause()
    for i in xrange(3):
        df.draw_rviz_box(pose_mat, [dims.x,dims.y, dims.z], frame = 'base_link')

def draw_pts(df,points):
    points_mat = scipy.matrix([[x.x,x.y,x.z] for x in points]).T
    for i in xrange(3):
        df.draw_rviz_points(points_mat, frame = 'base_link')
    print 'uncheck and recheck marker in rviz'
    pause()
    for i in xrange(3):
        df.draw_rviz_points(points_mat, frame = 'base_link')
