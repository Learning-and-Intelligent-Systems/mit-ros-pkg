#!/usr/bin/env python
# @author mbollini@mit.edu

import roslib
roslib.load_manifest('bakebot')
import rospy
import sys
import tf
import time
import pickle
from bakebot_pick_and_place_manager import *
from clients.pr2cm_client import *
from sensor_msgs.msg import JointState
#import pr2_gripper_reactive_approach.controller_manager as controller_manager
from bakebot_controller_manager import *
from clients.unified_tf_client import *

class JointRecorderPlayer: 
    def __init__(self):
        # right arm at index 0, left at index 1
        self.tf_listener = UnifiedTFClient.get_unified_tf_client()
        use_slip_detection = False
        self.cms = [ControllerManager.get_controller_manager('r', self.tf_listener, use_slip_detection), ControllerManager.get_controller_manager('l', self.tf_listener, use_slip_detection)]
        self.latest_state = None
        self.saved_states = list()
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        rospy.loginfo('switching both arms to cartesian control')
        client = PR2CMClient.get_pr2cm_client()
        print client.load_cartesian(True)
        print client.load_cartesian(False)

    def joint_state_callback(self, data):
        latest_state = dict()
        for name, position in zip(data.name, data.position):
            latest_state[name] = position
        #latest_state['time_secs'] = data.header.stamp.secs
        self.latest_state = latest_state

    def save_arm_state(self, whichArms=2):
        if whichArms == 0:
            data = self.get_arm_state(True),
        elif whichArms == 1:
            data = self.get_arm_state(False),
        else:
            data = (self.get_arm_state(True), self.get_arm_state(False))
        print 'saving arm state data arm=' + str(whichArms)
        self.saved_states.append(data)

    def write_saved_states_to_file(self, filename = None):
        if filename == None:
            filename = raw_input('enter filename: ')
        myfile = open(filename, 'wb')
        pickle.dump(self.saved_states, myfile)
        myfile.close()
        print 'file written to ' + filename

    def clear_saved_states(self):
        self.saved_states = list()

    def load_saved_states_from_file(self, filename = None):
        if filename == None:
            filename = raw_input('enter filename: ')
        myfile = open(filename, 'rb')
        self.clear_saved_states()
        self.saved_states = pickle.load(myfile)
        myfile.close()
        print 'saved states loaded from ' + filename

    def get_arm_state(self, isRightArm):
        count = 0
        while self.latest_state == None:
            rospy.logwarn('NO STATE YET')
            count = count + 1
            print count
            time.sleep(1)
        names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'upper_arm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
        prefix = 'r_' if isRightArm else 'l_'
        for index, name in enumerate(names):
            names[index] = prefix + name
        ret = dict()
        for name in names:
            ret[name] = self.latest_state.get(name)
        #ret['time_secs'] = self.latest_state.get('time_secs')
        return ret

    def get_gripper_state(self, isRightArm):
        print 'gripper state'
        count = 0
        while self.latest_state == None:
            rospy.logwarn('NO STATE YET')
            count = count + 1
            print count
            time.sleep(1)
        names = ['gripper_joint', 'gripper_r_finger_joint', 'gripper_l_finger_joint', 'gripper_r_finger_tip_joint', 'gripper_l_finger_tip_joint']
        prefix = 'r_' if isRightArm else 'l_'
        for index, name in enumerate(names):
            names[index] = prefix + name
        ret = dict()
        for name in names:
            ret[name] = self.latest_state.get(name)
            print name, '\t', ret[name]
        #ret['time_secs'] = self.latest_state.get('time_secs')
        return ret

    def move_arm_joints(self, isRightArm, joint_angles):
        cms = self.cms[0] if isRightArm else self.cms[1]
        result = cms.move_arm_joint(joint_angles, accept_invalid = 1, blocking = 1)
        if result == 1:
            rospy.loginfo('move arm reported success')
        else:
            rospy.logerr('move arm reported disaster')
            papm = PickAndPlaceManager.get_pick_and_place_manager()
            whicharm = 0 if isRightArm else 1
            papm.try_hard_to_move_joint(whicharm, [joint_angles,], 2, use_open_loop = 1)
        return result
    

    def go_to_arm_state(self, key_value_state):
        rnames = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
        lnames = ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']
        if len(key_value_state) == 1:
            state, = key_value_state
            names = rnames if state.has_key(rnames[0]) else lnames
            joint_angles = list()
            for name in names:
                joint_angles.append(state.get(name))
            self.move_arm_joints(state.has_key(rnames[0]), joint_angles)
        else:
            a, b = key_value_state
            r_joint_angles = list()
            l_joint_angles = list()
            if a.has_key(rnames[0]):
                for rname, lname in zip(rnames, lnames):
                    r_joint_angles.append(a.get(rname))
                    l_joint_angles.append(b.get(lname))
            else:
                for rname, lname in zip(rnames, lnames):
                    r_joint_angles.append(b.get(rname))
                    l_joint_angles.append(a.get(lname))
            self.move_arm_joints(True, r_joint_angles)
            self.move_arm_joints(False, l_joint_angles)


    def joint_recorder_ui(self):
        print 'JOINT RECORDER MODE (type \'q\' to return to the main menu)'
        print 'to record (r)ight (l)eft or (b)oth arm states type the command letter and hit ENTER'
        while True:
            command = raw_input('\t command: ')
            if command == 'r':
                self.save_arm_state(0)
            elif command == 'l':
                self.save_arm_state(1)
            elif command == 'b':
                self.save_arm_state(2)
            else:
                return

    def show_joints(self, states=None):
        i = 0
        rnames = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
        lnames = ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']
        saved_states = self.saved_states if states == None else states
        for saved in saved_states:
            print '\n==========================================================================================' + str(i)
            i = i + 1
            if len(saved) == 1:
                s,  = saved
                names = rnames if s.has_key(rnames[0]) else lnames
                for name in names:
                    print str(name) + ': \t\t%.5f' % s.get(name)
            else:
                a, b = saved
                anames = lnames if a.has_key(lnames[0]) else rnames
                bnames = lnames if b.has_key(lnames[0]) else rnames
                index = 0
                for aname, bname in zip(anames, bnames):
                    space = ' ' if index == 2 else ''
                    print str(aname) + ': ' + str(space) + '\t%.5f \t\t ' % a.get(aname), 
                    print str(bname) + ': \t%.5f' % b.get(bname) 
                    index = index + 1

    def joint_player_ui(self):
        print 'JOINT PLAYER MODE (type \'q\' to return to the main menu)'
        print 'enter the number of the recorded state to playback'
        while True:
            command = raw_input('\t state: ')
            if command == 'q':
                return
            else:
                try:
                    command = int(command)
                    self.go_to_arm_state(self.saved_states[command])
                except Exception as e:
                    print 'invalid input (or something else bad happened)' + str(e)

    def go_to_ui(self):
        rnames = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
        lnames = ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']
        print 'GOTO CUSTOM JOINT STATE MODE (type \'q\' to return to the main menu)'
        while True:
            joints_to_show = list()
            right_state = self.get_arm_state(True)
            left_state = self.get_arm_state(False)
            joints_to_show.append((right_state, left_state))
            self.show_joints(joints_to_show)
            arm = raw_input('\n\nenter (r)ight or (l)eft arm: ')
            if arm != 'r' and arm != 'l':
                return
            else:
                names = rnames if arm == 'r' else lnames
                state = right_state if arm == 'r' else left_state
                print '\nthe joint order is: ' + str(names)
                print '\nenter a space separated list of the desired angles (in radians)'
                print '(mistyped commands will be ignored, seven tokens are expected)'
                print 'enter an \'x\' to command that joint to remain in its current position'
                try:
                    lst = raw_input('\n\nangles: ')
                    tokens = lst.split(' ')
                    if len(tokens) != 7:
                        print 'invalid number of tokens (expect 7): ' + str(len(tokens))
                        continue
                    else:
                        #angles = map(float, tokens)
                        angles = list()
                        index = 0
                        for token in tokens:
                            if token == 'x':
                                angles.append(state.get(names[index]))
                            else:
                                angles.append(float(token))
                            index = index + 1
                        self.move_arm_joints(arm == 'r', angles)
                except Exception as e:
                    print 'something bad happened: ' + str(e)
    
    def playback_all_joints(self, automatic = False, reverse = False):
        print 'JOINT PLAYBACK SERIES MODE (type \'q\' to return to main menu)'
        print 'this will playback ALL recorded states, starting with the oldest'
        
        try:
            if not automatic:
                pause_time_sec = int(raw_input('enter the pause time between recorded states: '))
            else:
                pause_time_sec = 0
            if not reverse:
                playlist = self.saved_states
            else:
                playlist = list()
                for state in self.saved_states:
                    playlist.append(state)
                playlist.reverse()
            for state in playlist:
                self.go_to_arm_state(state)
                time.sleep(pause_time_sec)
        except Exception as e:
            print 'an error occurred: ' + str(e)
            return
        return
        
    def ui(self):
        print '\n\nWelcome to the joint_recorder_player!'
        while True:
            print '\n\nMain menu: '
            print '\t (r)ecord joint states'
            print '\t (p)lay joint states back'
            print '\t (P)lay ALL joint states back in a series'
            print '\t (g)o to a custom joint state'
            print '\t (v)iew joint states in buffer'
            print '\t (s)ave buffer to file'
            print '\t (l)oad file into buffer (clearing buffer)'
            print '\t (c)lear buffer'
            print '\t (q)uit'
            choice = raw_input('\n\tchoice: ')
            if choice == 'r':
                self.joint_recorder_ui()
            elif choice == 'p':
                self.joint_player_ui()
            elif choice == 'P':
                self.playback_all_joints()
            elif choice == 'g':
                self.go_to_ui()
            elif choice == 'G':
                self.get_gripper_state(False)
                self.get_gripper_state(True)
            elif choice == 'v':
                self.show_joints()
            elif choice == 's':
                self.write_saved_states_to_file()
            elif choice == 'l':
                self.load_saved_states_from_file()
            elif choice == 'c':
                self.clear_saved_states()
            elif choice == 'q':
                return
            else:
                print 'invalid input'


if __name__ == '__main__':
    rospy.init_node('joint_recorder_player')
    jrp = JointRecorderPlayer()
    jrp.ui()
