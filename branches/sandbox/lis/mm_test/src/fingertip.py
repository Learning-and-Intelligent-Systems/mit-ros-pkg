#!/usr/bin/python
import roslib
roslib.load_manifest('mm_test')
import rospy

# import messages and services
from fingertip_pressure.msg import PressureInfo, PressureInfoElement
from pr2_msgs.msg import PressureState


# import libraries
import tf
import sys
import threading
import math
from math import *

# import arm controller
import pr2_gripper_reactive_approach.controller_manager as controller_manager

# import utilities
from object_manipulator.convert_functions import *

class GripperTestManager():
    def __init__(self):
        self.LEFT = 1
        self.RIGHT = 0
        self.use_slip_detection = 0
        self.tf_listener = tf.TransformListener()
        self.cms = controller_manager.ControllerManager('l',self.tf_listener, self.use_slip_detection)
        rospy.Subscriber('/pressure/l_gripper_motor',PressureState, self.state_callback)
        #rospy.Subscriber('/pressure/l_gripper_motor_info',PressureInfo, self.info_callback)
        self.display_state = -1
        self.init_values = [[[0.0]*22],[[0.0]*22]]
        self.cur_values = [[[0.0]*22],[[0.0]*22]]
        self.initialized = False
        self.left_hit = False
        self.right_hit = False

    def state_callback(self, pressure_state):
        if self.display_state == self.LEFT:
            print 'left fingertip:\n',pressure_state.l_finger_tip
        elif self.display_state == self.RIGHT:
            print 'right fingertip:\n',pressure_state.r_finger_tip
        self.cur_values[self.LEFT] = pressure_state.l_finger_tip
        self.cur_values[self.RIGHT] = pressure_state.r_finger_tip
        if self.initialized:
            diff = lambda x,y: x-y if x-y>100 else 0
            sum = lambda x,y: x+y
            f = lambda whichtip: reduce(sum, map(diff, self.cur_values[whichtip], self.init_values[whichtip]))
            diff_left = f(self.LEFT)
            diff_right = f(self.RIGHT)
            if diff_left>diff_right: print 'left is stronger',
            elif diff_right>diff_left: print 'right is stronger',
            if diff_left > 0 and diff_right > 0: print 'both tips have contacts (any sensor value 100 higher than initial value)'
            elif diff_left > 0: print 'left tip has contact (any sensor value 100 higher than initial value)'
            elif diff_right > 0: print 'right tip has contact (any sensor value 100 higher than initial value)'
            self.left_hit = True if diff_left>0 else False
            self.right_hit = True if diff_right>0 else False

    def info_callback(self, pressure_info):
        print dir(pressure_info)
        print pressure_info.sensor

    def move_joints(self,joints,max_vel=.2):
        self.cms.command_joint_trajectory([joints], max_joint_vel = max_vel,blocking=1)

    def keyboard_interface(self):
        while not rospy.is_shutdown():
            print '-----------------------------------------'
            print 'enter c to initialize'
            print 'enter l or r to check the status of the left or right finger tip of the left gripper'
            print 'enter p to play'
            print 'enter q to quit'
            print 'enter z to haha'
            input = raw_input()
            if input == 'c':
                self.init_values = [x for x in self.cur_values]
                self.initialized = True
                print 'now you can touch the finger tip to test contacts'
            elif input == 'l' or input == 'r':
                self.display_state = self.LEFT if input == 'l' else self.RIGHT
            elif input == 'p':
                if not self.initialized:
                    print 'enter c to initialize'
                    continue
                while not rospy.is_shutdown():
                    if self.left_hit:
                        arm_angles = self.cms.get_current_arm_angles()
                        arm_angles[-1] += .1
                        self.move_joints(arm_angles,1.0)
                    elif self.right_hit:
                        arm_angles = self.cms.get_current_arm_angles()
                        arm_angles[-1] -= .1
                        self.move_joints(arm_angles,1.0)                    
            elif input == 'q':
                return
            elif input == 'z':
                print 'haha'
            else:
                continue

if __name__ == '__main__':
    rospy.init_node('mm_gripper_test', anonymous=True)

    gripper_test_manager = GripperTestManager()
    gripper_test_manager.keyboard_interface()
