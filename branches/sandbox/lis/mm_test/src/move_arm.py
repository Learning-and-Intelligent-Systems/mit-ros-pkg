#!/usr/bin/python
import roslib
roslib.load_manifest('mm_test')
import rospy
import tf
import actionlib
import scipy
import time
import math
import pr2_gripper_reactive_approach.controller_manager as controller_manager
from object_manipulator.convert_functions import *


def print_float_list(input):
    for item in input:
        print '%8.2f '%float(item),
    print

class ArmTestManager():
    def __init__(self):
        self.LEFT = 1
        self.RIGHT = 0

        self.arm_above_and_to_side_angles = [[-0.968, 0.729, -0.554, -1.891, -1.786, -1.127, 0.501],
                                             [0.968, 0.729, 0.554, -1.891, 1.786, -1.127, 0.501]]
        self.arm_to_side_angles = [[-2.135, 0.803, -1.732, -1.905, -2.369, -1.680, 1.398],
                                   [2.135, 0.803, 1.732, -1.905, 2.369, -1.680, 1.398]]
        self.traj_from_above_to_side = lambda whicharm: [self.arm_above_and_to_side_angles[whicharm], self.arm_to_side_angles[whicharm]]
        self.use_slip_detection = 0

        self.tf_listener = tf.TransformListener()
        self.cms = [controller_manager.ControllerManager('r', self.tf_listener, self.use_slip_detection), 
                    controller_manager.ControllerManager('l', self.tf_listener, self.use_slip_detection)]
    
    def keyboard_interface(self):
        while not rospy.is_shutdown():
            print "---------------------------------"
            print "enter 1 or 2 to move left or right arm to the side"
            print "enter ml or mr to specify goal joint angles and move the left or right arm"
            print "enter cml or cmr to specify goal pose and move the left or right arm"
            print "enter ol or or to open left or right gripper"
            print "enter cl or cr to close left or right gripper"
            print "enter l or r to check the left or right arm status"
            print "enter q to quit"
            print "enter z to haha"
            input = raw_input()
            if input == '1' or input == '2':
                whicharm = self.LEFT if input == '1' else self.RIGHT
                self.cms[whicharm].command_joint_trajectory(self.traj_from_above_to_side(whicharm), max_joint_vel = 0.5 ,blocking=1)
            elif input == 'ml' or input == 'mr':
                if input == 'ml':
                    whicharm = self.LEFT
                    print "----- LEFT", 
                else:
                    whicharm = self.RIGHT
                    print "----- RIGHT",
                print "ARM JOINT STATUS -----\nJOINT NAMES"
                for jn in self.cms[whicharm].joint_names: print repr(jn).rjust(10),
                print "\nJOINT POSITIONS"
                pos_current = self.cms[whicharm].get_current_arm_angles()
                for p in pos_current: print '%10.2f'%p,
                print "\n---------------------------------\nenter desired joint positions, 7 numbers separated by space:"
                pos = raw_input()
                pos = [float(x) for x in pos.split()]
                print "are you sure to have the arm move to the following joint angles?"
                print_float_list(pos)
                print "[Y/n]:",
                answer = raw_input()
                if answer == '' or answer == 'y' or answer == 'Y':
                    print "moving arm..."
                    self.cms[whicharm].command_joint_trajectory([pos_current, pos], max_joint_vel = 0.5, blocking=1)
                else:
                    continue
            elif input == 'cml' or input == 'cmr':
                print "TRANSFORMATION TO",
                if input == 'cml':
                    whicharm = self.LEFT
                    print '/left',
                else:
                    whicharm = self.RIGHT
                    print '/right',
                print '_wrist_roll_link FROM base_link'
                cs = self.cms[whicharm].return_cartesian_pose()
                print 'TRANSLATION'
                print cs[0]
                print 'QUATERNION'
                print cs[1]
                print 'enter desired translation, 3 numbers separated by space, enter directly to keep the current values:'
                trans = raw_input()
                if trans == '':
                    trans = cs[0]
                else:
                    trans = [float(x) for x in trans.split()]
                print 'are you sure about the following translation values?'
                print_float_list(trans)
                print '[Y/n]:',
                answer = raw_input()
                if answer == '' or answer == 'y' or answer == 'Y':
                    print 'enter desired rotation quaternion, 4 numbers separated by space, enter directly to keep the current values:'
                    rot = raw_input()
                    if rot == '':
                        rot = cs[1]
                    else:
                        rot = [float(x) for x in rot.split()]
                    print 'are you sure about the following rotation quaternion?'
                    print_float_list(rot)
                    print '[Y/n]:',
                    answer = raw_input()
                    if answer == '' or answer == 'y' or answer == 'Y':
                        print 'moving arm...'
                        self.cms[whicharm].move_cartesian(create_pose_stamped(trans+rot), settling_time = rospy.Duration(10))
                    else:
                        continue
                else:
                    continue
            elif input == 'ol' or input == 'or':
                whicharm = self.LEFT if input == 'ol' else self.RIGHT
                self.cms[whicharm].command_gripper(0.08, -1.0)
            elif input == 'cl' or input == 'cr':
                whicharm = self.LEFT if input == 'cl' else self.RIGHT
                self.cms[whicharm].command_gripper(0.0, -1.0)
            elif input == 'l' or input == 'r':
                whicharm = self.LEFT if input == 'l' else self.RIGHT
                while True:
                    (found, positions, vels, efforts) = self.cms[whicharm].joint_states_listener.return_joint_states(self.cms[whicharm].joint_names)
                    print "ARM JOINT STATUS -----\nJOINT NAMES"
                    for jn in self.cms[whicharm].joint_names: print repr(jn).rjust(10),
                    print "\nJOINT POSITIONS"
                    for p in positions: print '%10.2f'%p,
                    print "\nJOINT VELOCITIES"
                    for v in vels: print '%10.2f'%v,
                    print '\nJOINT EFFORTS'
                    for e in efforts: print '%10.2f'%e,
                    print ''
                    rospy.sleep(.1)
            elif input == 'z':
                print 'haha'

            elif input == 'q':
                return
            else:
                continue

if __name__ == '__main__':
    rospy.init_node('mm_move_arm_test', anonymous=True)
    arm_test_manager = ArmTestManager()
    arm_test_manager.keyboard_interface()
