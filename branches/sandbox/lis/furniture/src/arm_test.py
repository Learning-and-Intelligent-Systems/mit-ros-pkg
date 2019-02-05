#!/usr/bin/python
import roslib
roslib.load_manifest('furniture')
import rospy
import tf
import actionlib
import scipy
import time
import math
import pr2_gripper_reactive_approach.controller_manager as controller_manager
from object_manipulator.convert_functions import *



class ArmTestManager():

    def __init__(self):
        self.LEFT = 1
        self.RIGHT = 0
        self.arm = self.RIGHT  # default is right arm
        self.use_slip_detection = 0
        self.tf_listener = tf.TransformListener()
        self.cms = [controller_manager.ControllerManager('r', self.tf_listener, self.use_slip_detection), 
                    controller_manager.ControllerManager('l', self.tf_listener, self.use_slip_detection)]
        self.left_actions = []
        self.right_actions = []

        
    def move_joints(self, arm, joints):
        arm = self.LEFT if arm == 'left' else self.RIGHT
        self.cms[arm].command_joint_trajectory([joints], blocking=1)


    def record_joints(self):
        (found, positions, vels, efforts) = self.cms[self.arm].joint_states_listener.return_joint_states(self.cms[self.arm].joint_names)
        if self.arm == self.LEFT:
            self.left_actions.append(positions)
        else:
            self.right_actions.append(positions)
            
    
    def keyboard_interface(self):
        while not rospy.is_shutdown():
            print "---------------------------------"
            print "enter l or r to record the left or right arm position"
            print "enter open or close to set the gripper position"
            print "enter q to quit"
            input = raw_input()

            if input == 'l':
                self.arm = self.LEFT
                self.record_joints()
                
            elif input == 'r':
                arm = self.RIGHT
                self.record_joints()
                
            elif input == 'p':
                actions = self.left_actions if self.arm == self.LEFT else self.right_actions
                for joints in actions:
                    self.move_joints(self.arm, joints)




if __name__ == '__main__':
    rospy.init_node('furniture_arm_test', anonymous=True)
    arm_test_manager = ArmTestManager()
    arm_test_manager.keyboard_interface()
