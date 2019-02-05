#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
import rospy
from bakebot.srv import *
import tf
import actionlib
import math
import sys
import time
from bakebot_controller_manager import *
from pr2_controller_manager import pr2_controller_manager_interface as pr2cm_interface

class PR2CMClient:
    '''
    This class switches between the ee_cart_imped controller and the standard move_arm
    suite of controllers on the PR2 arms.

    When invoked, the methods stop one controller and start the other.  They require that the
    controllers are already loaded (albeit stopped).
    '''
   
    pr2cm_client_instance = None

    @staticmethod
    def get_pr2cm_client():
        ''' 
        use this static factory method instead of the constructor to enforce singleton 
        '''
        if PR2CMClient.pr2cm_client_instance == None:
            rospy.loginfo('instantiating a new pr2 controller manager client')
            PR2CMClient.pr2cm_client_instance = PR2CMClient()
        else:
            rospy.loginfo('returning existing instance of client ' + str(PR2CMClient.pr2cm_client_instance))
        return PR2CMClient.pr2cm_client_instance

    def __init__(self):
        '''
        do not call this constructor, call the static factory method instead
        '''
        rospy.loginfo('creating PR2CMClient')
        #pr2cm_interface.list_controllers()
        rospy.loginfo('loading (but not starting) impedance controllers')
        status = pr2cm_interface.load_controller('r_arm_cart_imped_controller')
        status = pr2cm_interface.load_controller('l_arm_cart_imped_controller')
        pr2cm_interface.list_controllers()
        rospy.loginfo('done')


    @staticmethod
    def load_ee_cart_imped(isRightArm):
        '''
        loads the ee_cart_imped controller on the arm specified.
        stops the cartesian controller first.
        @param isRightArm: if true then the controller will be loaded on the right arm, else the left arm
        '''
        rospy.logwarn('starting ee_cart_imped controller on the '+ ('right' if isRightArm else 'left') + ' arm')
        if isRightArm:
            status = pr2cm_interface.stop_controller('r_arm_controller')
            #print 'stopping arm controller status:', status
            status = pr2cm_interface.stop_controller('r_arm_cartesian_trajectory_controller')
            #print 'stopping trajectory controller status:', status
            status = pr2cm_interface.stop_controller('r_arm_cartesian_pose_controller')
            #print 'stopping pose controller status:', status
            status = pr2cm_interface.start_controller('r_arm_cart_imped_controller')
            #print 'starting controller status:', status
        else:
            status = pr2cm_interface.stop_controller('l_arm_controller')
            #print 'stopping controller status:', status
            status = pr2cm_interface.stop_controller('l_arm_cartesian_trajectory_controller')
            #print 'stopping trajectory controller status:', status
            status = pr2cm_interface.stop_controller('l_arm_cartesian_pose_controller')
            #print 'stopping pose controller status:', status
            status = pr2cm_interface.start_controller('l_arm_cart_imped_controller')
            #print 'starting controller status:', status
        rospy.loginfo('done')
        return status
        

    @staticmethod
    def load_cartesian(isRightArm):
        '''
        loads the cartesian controller on the arm specified.
        stops the impedance controller first.
        @param isRightArm: if true then the controller will be loaded on the right arm, else the left arm
        '''
        rospy.logwarn('starting cartesian controller on the '+ ('right' if isRightArm else 'left') + ' arm')
        if isRightArm:
            status = pr2cm_interface.stop_controller('r_arm_cart_imped_controller')
            #print 'stopping controller status:', status
            status = pr2cm_interface.start_controller('r_arm_controller')
            #print 'starting controller status:', status
            status = pr2cm_interface.start_controller('r_arm_cartesian_trajectory_controller')
            #print 'starting trajectory controller status:', status
            status = pr2cm_interface.start_controller('r_arm_cartesian_pose_controller')
            #print 'starting pose controller status:', status
        else:
            status = pr2cm_interface.stop_controller('l_arm_cart_imped_controller')
            #print 'stopping controller status:', status
            status = pr2cm_interface.start_controller('l_arm_controller')
            #print 'starting controller status:', status
            status = pr2cm_interface.start_controller('l_arm_cartesian_trajectory_controller')
            #print 'starting trajectory controller status:', status
            status = pr2cm_interface.start_controller('l_arm_cartesian_pose_controller')
            #print 'starting pose controller status:', status
        rospy.loginfo('done')
        return status

if __name__ == '__main__':
    rospy.init_node('pr2cm_client_tester', anonymous=True)
    c = PR2CMClient.get_pr2cm_client()
    c.load_ee_cart_imped(False)
    time.sleep(3)
    c.load_cartesian(False)
