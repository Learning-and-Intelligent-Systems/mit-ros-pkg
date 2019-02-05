#!/usr/bin/env python
import roslib
roslib.load_manifest('pr2_utils')
import rospy
from pr2_controller_manager import pr2_controller_manager_interface as pr2cm_interface

class PR2CMClient:
    '''
    This class switches between the ee_cart_imped controller and the standard move_arm
    suite of controllers on the PR2 arms.

    When invoked, the methods stop one controller and start the other.  They require that the
    controllers are already loaded (albeit stopped).
    '''
   
    @staticmethod
    def load_ee_cart_imped(isRightArm):
        '''
        loads the ee_cart_imped controller on the arm specified.
        stops the cartesian controller first.
        @param isRightArm: if true then the controller will be loaded on the right arm, else the left arm
        '''
        rospy.logdebug('starting ee_cart_imped controller on the '+ ('right' if isRightArm else 'left') + ' arm')
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
        rospy.logdebug('done')
        return status
        
    @staticmethod
    def load_cartesian(isRightArm):
        '''
        loads the cartesian controller on the arm specified.
        stops the impedance controller first.
        @param isRightArm: if true then the controller will be loaded on the right arm, else the left arm
        '''
        rospy.logdebug('starting cartesian controller on the '+ ('right' if isRightArm else 'left') + ' arm')
        if isRightArm:
            status = pr2cm_interface.stop_controller('r_arm_cart_imped_controller')
            #print 'stopping controller status:', status
            status = pr2cm_interface.start_controller('r_arm_controller')
            #print 'starting controller status:', status
            status = pr2cm_interface.stop_controller('r_arm_cartesian_trajectory_controller')
            #print 'stopping trajectory controller status:', status
            status = pr2cm_interface.stop_controller('r_arm_cartesian_pose_controller')
            #print 'stopping pose controller status:', status
        else:
            status = pr2cm_interface.stop_controller('l_arm_cart_imped_controller')
            #print 'stopping controller status:', status
            status = pr2cm_interface.start_controller('l_arm_controller')
            #print 'starting controller status:', status
            status = pr2cm_interface.stop_controller('l_arm_cartesian_trajectory_controller')
            #print 'stopping trajectory controller status:', status
            status = pr2cm_interface.stop_controller('l_arm_cartesian_pose_controller')
            #print 'stopping pose controller status:', status
        rospy.logdebug('done')
        return status
