#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
import rospy
from bakebot.srv import *
from services.bakebot_logging import *
from unified_tf_client import UnifiedTFClient
from utilities.joint_recorder_player import *
from utilities.bakebot_controller_manager import *
import tf
import actionlib
import itertools
import math
import os
import sys
import time

class ArmClient:
    
    arm_client_instance = None
    USE_CM_ARM_CLIENT = False

    @staticmethod
    def get_arm_client():
        if ArmClient.arm_client_instance == None:
            rospy.loginfo('instantiating a new arm client')
            ArmClient.arm_client_instance = ArmClient(UnifiedTFClient.get_unified_tf_client()) #TODO: may have to fix this
        else:
            rospy.loginfo('returning existing instance of arm client ' + str(ArmClient.arm_client_instance))
        return ArmClient.arm_client_instance

    def __init__(self, transformListener):
        self.r_tip_frame = 'r_wrist_roll_link'
        self.l_tip_frame = 'l_wrist_roll_link'
        self.base_frame = 'base_link'
        self.tf_listener = transformListener
        #rospy.loginfo('waiting for bakebot_move_ee service')
        #rospy.wait_for_service('bakebot_move_ee')
        rospy.loginfo('burning off a bad tf reading and waiting')
        self.get_transform(True)
        time.sleep(1)
        self.get_transform(True)
        use_slip_detection = 0
        self.cms = [ControllerManager.get_controller_manager('r', transformListener, use_slip_detection), ControllerManager.get_controller_manager('l', transformListener, use_slip_detection)]
        rospy.loginfo('done with CMArmClient init')
        if not ArmClient.USE_CM_ARM_CLIENT:
            rospy.logwarn('not using cmarmclient')
            self.papm = PickAndPlaceManager.get_pick_and_place_manager()
        rospy.loginfo('done with arm client init')

    
    def move_to_pose_const_ee(self, isRightArm, x_pos, y_pos, z_pos, determination = 0, x_plus_minus = (0.03, 0.03), y_plus_minus = (0.03, 0.03), z_plus_minus = (0.03, 0.03), recursionsRemaining = 1):
        print('moving to the pose with const ee: ' + str(isRightArm))
        (trans, rot) = self.get_transform(isRightArm)
        if determination == 0:
            status = self.move_to_pose(isRightArm, True, x_pos, y_pos, z_pos, rot[0], rot[1], rot[2], rot[3]);
        else:
            min_max_x_pos = (x_pos + x_plus_minus[0], x_pos - x_plus_minus[1])
            min_max_y_pos = (y_pos + y_plus_minus[0], y_pos - y_plus_minus[1])
            min_max_z_pos = (z_pos + z_plus_minus[0], z_pos - z_plus_minus[1])
            quater_xyzw = (rot[0], rot[1], rot[2], rot[3])
            print quater_xyzw
            status = self.try_hard_to_move_to_pose(isRightArm, True, min_max_x_pos, min_max_y_pos, min_max_z_pos, quater_xyzw, 0.02, (determination > 1), recursionsRemaining)
        return status

    def move_delta_const_ee(self, isRightArm, delta, axis, determination = 0):
        print 'moving delta with const ee'
        (trans, rot) = self.get_transform(isRightArm)
        x = trans[0]
        y = trans[1]
        z = trans[2]
        command = str(axis) + ' = ' + str(axis) + ' + ' + str(delta)
        print 'about to execute: ', command
        exec command
        status = self.move_to_pose_const_ee(isRightArm, x, y, z, determination = determination)
        return status
        
    def interpolate_range(self, start, end, delta):
        delta = abs(delta)
        ret = list()
        reverse = start > end 
        minimum = min((start,end))
        maximum = max((start,end))
        ret.append(minimum)
        while ret[len(ret)-1] < maximum:
            ret.append(ret[len(ret)-1] + delta)
        if reverse:
            ret.reverse()
        return ret

    def try_hard_to_move_to_pose(self, isRightArm, interpolate, min_max_x_pos, min_max_y_pos, min_max_z_pos, quater_xyzw, d_interp = 0.03, flexEEO = False, recursionsRemaining = 0):
        rospy.loginfo('trying hard to move to a pose (recursions remaining = ' + str(recursionsRemaining) +')')
        rospy.loginfo('first trying to move to the pose')
        x = (min_max_x_pos[1] + min_max_x_pos[0])/2
        y = (min_max_y_pos[1] + min_max_y_pos[0])/2
        z = (min_max_z_pos[1] + min_max_z_pos[0])/2
        qx, qy, qz, qw = quater_xyzw
        status = self.move_to_pose(isRightArm, interpolate, x, y, z, qx, qy, qz, qw, override_cmpapm=True)
        if status == 0:
            rospy.loginfo('moving to pose was successful, my job here is done')
            return status
        else:
            rospy.logwarn('moving to pose failed, trying harder...')

        rospy.loginfo('trying to move with increased step size')
        status = self.move_to_pose(isRightArm, interpolate, x, y, z, qx, qy, qz, qw, interpolate_step_size = 0.015, collision_step_size = 0.015, override_cmpapm=True)
        if status == 0:
            rospy.loginfo('moving to pose was successful, my job here is done')
            return status
        else:
            rospy.logwarn('moving to pose failed, trying harder...')

        rospy.loginfo('trying to move with EVEN MORE increased step size')
        status = self.move_to_pose(isRightArm, interpolate, x, y, z, qx, qy, qz, qw, interpolate_step_size = 0.03, collision_step_size = 0.03, override_cmpapm=True)
        if status == 0:
            rospy.loginfo('moving to pose was successful, my job here is done')
            return status
        else:
            rospy.logwarn('moving to pose failed, trying harder...')
        rospy.loginfo('trying iteration from minimum to maximum of the goal cartesian positions')
        (xmin, xmax) = min_max_x_pos
        (ymin, ymax) = min_max_y_pos
        (zmin, zmax) = min_max_z_pos
        print 'min -> max'
        print 'x:', xmin, xmax
        print 'y:', ymin, ymax
        print 'z:', zmin, zmax
        print 'step size: ', d_interp
        xlst = self.interpolate_range(xmin, xmax, d_interp)
        ylst = self.interpolate_range(ymin, ymax, d_interp)
        zlst = self.interpolate_range(zmin, zmax, d_interp)
        print 'xinterp: ',  xlst
        print 'yinterp: ',  ylst
        print 'zinterp: ',  zlst
        for try_z in zlst:
            for try_y in ylst:
                for try_x in xlst:
                    rospy.loginfo('trying to move to (x,y,z) = ' + str((try_x, try_y, try_z)))
                    rospy.loginfo('(qx,qy,qz,qw) = ' + str((qx,qy,qz,qw)))
                    status = self.move_to_pose(isRightArm, interpolate, try_x, try_y, try_z, qx, qy, qz, qw, override_cmpapm=True)
                    if status == 0:
                        rospy.loginfo('moving to pose was successful')
                        rospy.loginfo('number of recursions remaining: ' + str(recursionsRemaining))
                        if recursionsRemaining == 0:
                            rospy.loginfo('out of recursions, quitting')
                            return status
                        else:
                            rospy.loginfo('recursing to improve overall result')
                            return self.try_hard_to_move_to_pose(isRightArm, interpolate, min_max_x_pos, min_max_y_pos, min_max_z_pos, quater_xyzw, d_interp, flexEEO, recursionsRemaining - 1)
                    else:
                        rospy.logwarn('moving to pose failed, more brute force might do it...')
        rospy.logwarn('brute force iteration over the allowable stepping points did not work')
        if flexEEO and interpolate:
            rospy.logwarn('disabling end effector pose constraint and iterpolation (implicitly recursing)')
            return self.try_hard_to_move_to_pose(isRightArm, False, min_max_x_pos, min_max_y_pos, min_max_z_pos, quater_xyzw, d_interp, flexEEO, recursionsRemaining)
        else:
            rospy.logerr('out of options for moving the end effector to the desired pose')
            return -9999

    def rotate_end_effector(self, isRightArm, x_orien, y_orien, z_orien, w_orien, determination = 0, x_plus_minus = (0.02, 0.02), y_plus_minus = (0.02, 0.02), z_plus_minus = (0.02, 0.02), recursionsRemaining = 1, use_interpolation = True):
        print('rotate end effector')
        (trans, rot) = self.get_transform(isRightArm)
        if determination == 0:
            status = self.move_to_pose(isRightArm, use_interpolation, trans[0], trans[1], trans[2], x_orien, y_orien, z_orien, w_orien)
        else:
            min_max_x_pos = (trans[0] + x_plus_minus[0], trans[0] - x_plus_minus[1])
            min_max_y_pos = (trans[1] + y_plus_minus[0], trans[1] - y_plus_minus[1])
            min_max_z_pos = (trans[2] + z_plus_minus[0], trans[2] - z_plus_minus[1])
            quater_xyzw = (x_orien, y_orien, z_orien, w_orien)

            status = self.try_hard_to_move_to_pose(isRightArm, False, min_max_x_pos, min_max_y_pos, min_max_z_pos, quater_xyzw, 0.01, (determination > 1), recursionsRemaining)
        return status

    def get_transform(self, isRightArm):
        tip_frame = self.r_tip_frame if isRightArm else self.l_tip_frame
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.base_frame, tip_frame, rospy.Time(0))
            record = 'base frame: '+ str(self.base_frame) +'\ntrans: ' + str(trans) + '\nrot: ' + str(rot) + '\n'
            print record
            return (trans, rot)
        except (tf.LookupException, tf.ConnectivityException):
            rospy.loginfo('tf is all messed up in the arm client')
            return ((0,0,0), (0,0,0,0))

    def move_to_pose(self, isRightArm, interpolate, x_pos, y_pos, z_pos, x_orien, y_orien, z_orien, w_orien, interpolate_step_size = 0.005, collision_step_size = 0.005, collision_aware = 0, pos_thresh = 0.03, rot_thresh = 0.1, override_cmpapm = False):
        if interpolate:
            logger = EventLoggerClient(EventLogger.event_type_move_arm, 'moving to pose')
        else:
            logger = EventLoggerClient(EventLogger.event_type_move_arm, 'moving to pose with interpolation')
        rospy.loginfo('CMArmClient: moving to pose in ' + str(self.base_frame) + ' frame')
        cms = self.cms[0] if isRightArm else self.cms[1]
        goal_pose_lst = [x_pos, y_pos, z_pos, x_orien, y_orien, z_orien, w_orien]
        goal_pose = create_pose_stamped(goal_pose_lst, self.base_frame)
        rospy.loginfo('CMArmClient: moving to this pose: ' + str(goal_pose_lst))
        interpolate_step_size = 0.005
        collision_step_size = 0.005
        #collision_aware = 0
        blocking = 1
        #pos_thresh = 0.03
        #rot_thresh = 0.1
        timeout = rospy.Duration(60.0)
        settling_time = rospy.Duration(10.0)
        #timeout = rospy.Duration(30.0)
        rospy.loginfo('is right arm? ' + str(isRightArm))
        if ArmClient.USE_CM_ARM_CLIENT or override_cmpapm:
            if interpolate:
                rospy.loginfo('CMArmClient moving arm with INTERPOLATION')
                status = cms.move_cartesian_ik(goal_pose, collision_aware, blocking, collision_step_size, 
                                        pos_thresh, rot_thresh, timeout, settling_time)
            else:
                rospy.loginfo('CMArmClient moving arm')
                status = cms.move_cartesian(goal_pose, collision_aware, blocking, collision_step_size, 
                                        pos_thresh, rot_thresh, timeout, settling_time)
            rospy.loginfo('BCM returned with Status: ' + str(status))
            tip_frame = self.r_tip_frame if isRightArm else self.l_tip_frame
            (trans, rot) = self.tf_listener.lookupTransform(self.base_frame, tip_frame, rospy.Time(0))
            if status == 'failed' or status == 'no solution':
                rospy.logwarn('BCM status failed')
                rospy.loginfo('desired: ' + str(goal_pose_lst))
                rospy.loginfo('actual trans: ' + str(trans))
                rospy.loginfo('actual rot: ' + str(rot))
                des_trans = (x_pos, y_pos, z_pos)
                des_rot = (x_orien, y_orien, z_orien, w_orien)
                if self.is_close_enough(des_trans, des_rot, trans, rot):
                    rospy.loginfo('determined to be close enough')
                    logger.stops('close enough (original status = ' + status + ')')
                    return 0
                logger.stopf('not close enough: ' + status)
                return -1
            elif status == 'timed out' or status == 'settling timed out':
                rospy.logwarn('time out occurred')
                rospy.loginfo('time ran out, going to sleep for 15 seconds to see how it turns out')
                rospy.loginfo('desired: ' + str(goal_pose_lst))
                rospy.loginfo('actual trans: ' + str(trans))
                rospy.loginfo('actual rot: ' + str(rot))
                time.sleep(15.0)
                des_trans = (x_pos, y_pos, z_pos)
                des_rot = (x_orien, y_orien, z_orien, w_orien)
                if self.is_close_enough(des_trans, des_rot, trans, rot):
                    rospy.loginfo('determined to be close enough')
                    logger.stops('close enough (original status = ' + status + ')')
                    return 0
                logger.stopf('timeout: ' + status)
                return -2
            else:
                logger.stops('finished ok')
                return 0
        else:
            # this means that we use the papm move arm thing
            whicharm = 0 if isRightArm else 1
            try_const = 1 if interpolate else 0
            status = self.papm.try_hard_to_move_pose(whicharm, goal_pose, use_cartesian = 0, try_constrained = try_const)
            if status == 1:
                des_trans = (x_pos, y_pos, z_pos)
                des_rot = (x_orien, y_orien, z_orien, w_orien)
                tip_frame = self.r_tip_frame if isRightArm else self.l_tip_frame
                (trans, rot) = self.tf_listener.lookupTransform(self.base_frame, tip_frame, rospy.Time(0))
                if self.is_close_enough(des_trans, des_rot, trans, rot):
                    logger.stops('finished ok')
                    rospy.loginfo('success')
                    return 0
                else:
                    return self.move_to_pose(isRightArm, interpolate, x_pos, y_pos, z_pos, x_orien, y_orien, z_orien, w_orien, interpolate_step_size, collision_step_size, collision_aware, pos_thresh, rot_thresh, True)
            else:
                logger.stopf('failure')
                return self.move_to_pose(isRightArm, interpolate, x_pos, y_pos, z_pos, x_orien, y_orien, z_orien, w_orien, interpolate_step_size, collision_step_size, collision_aware, pos_thresh, rot_thresh, True)


    
    def is_close_enough(self, t_des, r_des, t_act, r_act):
        t_tol = 0.045
        r_tol = 0.10
        for desired, actual in zip(t_des, t_act):
            if (abs(desired - actual)) > t_tol:
                rospy.logwarn('translation not close enough.  desired='+str(desired)+' actual='+str(actual) +' tol=' + str(t_tol) + ' delta='+str(abs(desired-actual)))
                return False
        for desired, actual in zip(r_des, r_act):
            if (abs(desired - actual)) > r_tol:
                rospy.logwarn('IGNORING FOR ABS rotation not close enough.  desired='+str(desired)+' actual='+str(actual) + ' tol=' + str(r_tol) + ' delta='+str(abs(desired-actual)))
            if (abs(desired) - abs(actual)) > r_tol:
                rospy.logwarn('ABS(DES) - ABS(ACT) rotation not close enough.  desired='+str(desired)+' actual='+str(actual) + ' tol=' + str(r_tol) + ' delta='+str(abs(desired-actual)))
                return False
        return True


    def jrp_helper(self, is_right_arm, reverse_jrp):
        path = os.environ['BAKEBOT_JRPLOG_PATH']
        arm = 'r' if is_right_arm else 'l'
        f = path + '20110614-'+arm+'armspoon_jrp.log'
        jrp = JointRecorderPlayer()
        jrp.load_saved_states_from_file(f)
        jrp.playback_all_joints(automatic = True, reverse = reverse_jrp)

    def advance_arm(self, is_right_arm):
        self.jrp_helper(is_right_arm, False)

    def retreat_arm(self, is_right_arm):
        self.jrp_helper(is_right_arm, True)

    def ui_loop(self):
        acui = ArmClientUI(self)
        acui.ui_loop()
        
class ArmClientUI:

    def __init__(self, arm_client=None):
        if arm_client == None:
            arm_client = ArmClient(tf.TransformListener())
        self.arm_client = arm_client
        self.cmd_history = list()
        self.populate_known_poses()

    def populate_known_poses(self):
        self.known_poses = list()
        self.known_poses.append(str('x x x -0.5 0.5 0.5 0.5 (12oc-barcode-out)'))
        self.known_poses.append(str('x x x -0.5 -0.5 0.5 -0.5 (12oc-barcode-in)'))
        self.known_poses.append(str('x x x 0.0 0.7 0.0 0.7 (3oc-barcode-out)'))
        self.known_poses.append(str('x x x 0.7 0.0 -0.7 0.0 (3oc-barcode-in)'))
        self.known_poses.append(str('x x x -0.5 -0.5 0.5 -0.5 (6oc-barcode-out)'))
        self.known_poses.append(str('x x x -0.5 0.5 0.5 0.5 (6oc-barcode-in)'))
        self.known_poses.append(str('x x x 0.7 0.0 -0.7 0.0 (9oc-barcode-out)'))
        self.known_poses.append(str('x x x 0.0 0.7 0.0 0.7 (9oc-barcode-in)'))

        
    def const_ee_o_ui(self):
        ac = self.arm_client
        delta = .05
        print 'arm end effector movement with CONSTANT ORIENTATION'
        print '\ndelta set to %f meters' % delta
        choice = raw_input('(r)ight arm, (l)eft arm: ')
        isRightArm = (choice == 'r')
        if isRightArm:
            print 'controlling the RIGHT arm'
        else:
            print 'controlling the LEFT arm'
        exit = False
        while exit == False:
            (trans, rot) = ac.get_transform(isRightArm)
            key = raw_input('(f)orward, (l)eft, (r)ight, (b)ackward, (u)p, (d)own, (s)et delta, (q)uit: ')
            if key == 'f':
                #print ac.move_to_pose_const_ee(isRightArm, trans[0] + delta, trans[1], trans[2])
                print ac.move_delta_const_ee(isRightArm, delta, 'x')
            elif key == 'l':
                print ac.move_to_pose_const_ee(isRightArm, trans[0], trans[1] + delta, trans[2])
            elif key == 'r':
                print ac.move_to_pose_const_ee(isRightArm, trans[0], trans[1] - delta, trans[2])
            elif key == 'b':
                print ac.move_to_pose_const_ee(isRightArm, trans[0] - delta, trans[1], trans[2])
            elif key == 'u':
                print ac.move_to_pose_const_ee(isRightArm, trans[0], trans[1], trans[2] + delta)
            elif key == 'd':
                print ac.move_to_pose_const_ee(isRightArm, trans[0], trans[1], trans[2] - delta)
            if key == 'F':
                print ac.move_to_pose_const_ee(isRightArm, trans[0] + delta, trans[1], trans[2], determination = 2)
            elif key == 'L':
                print ac.move_to_pose_const_ee(isRightArm, trans[0], trans[1] + delta, trans[2], determination = 2)
            elif key == 'R':
                print ac.move_to_pose_const_ee(isRightArm, trans[0], trans[1] - delta, trans[2], determination = 2)
            elif key == 'B':
                print ac.move_to_pose_const_ee(isRightArm, trans[0] - delta, trans[1], trans[2], determination = 2)
            elif key == 'U':
                print ac.move_to_pose_const_ee(isRightArm, trans[0], trans[1], trans[2] + delta, determination = 2)
            elif key == 'D':
                print ac.move_to_pose_const_ee(isRightArm, trans[0], trans[1], trans[2] - delta, determination = 2)
            elif key == 's':
                delta = float(raw_input('enter delta (m): '))
                print 'delta set to %f meters' % delta
            elif key == 'q':
                exit = True
            else:
                continue

    def const_ee_pos_ui(self):
        ac = self.arm_client
        delta = .05
        print 'arm end effector movement with CONSTANT POSITION'
        print '\ndelta set to %f radians' % delta
        choice = raw_input('(r)ight arm, (l)eft arm: ')
        isRightArm = (choice == 'r')
        if isRightArm:
            print 'controlling the RIGHT arm'
        else:
            print 'controlling the LEFT arm'
        exit = False
        while exit == False:
            (trans, rot) = ac.get_transform(isRightArm)
            key = raw_input('(X) inc x, (x) dec x, (Y) inc y, (y) dec y, (Z) inc z, (z) dec z, (W) inc w, (w) dec w, (s)et delta, (q)uit: ')
            if key == 'X':
                print ac.rotate_end_effector(isRightArm, rot[0] + delta, rot[1], rot[2], rot[3])
            elif key == 'x':
                print ac.rotate_end_effector(isRightArm, rot[0] - delta, rot[1], rot[2], rot[3])
            elif key == 'Y':
                print ac.rotate_end_effector(isRightArm, rot[0], rot[1] + delta, rot[2], rot[3])
            elif key == 'y':
                print ac.rotate_end_effector(isRightArm, rot[0], rot[1] - delta, rot[2], rot[3])
            elif key == 'Z':
                print ac.rotate_end_effector(isRightArm, rot[0], rot[1], rot[2] + delta, rot[3])
            elif key == 'z':                                                           
                print ac.rotate_end_effector(isRightArm, rot[0], rot[1], rot[2] - delta, rot[3])
            elif key == 'W':
                print ac.rotate_end_effector(isRightArm, rot[0], rot[1], rot[2], rot[3] + delta)
            elif key == 'w':                                                                   
                print ac.rotate_end_effector(isRightArm, rot[0], rot[1], rot[2], rot[3] - delta)
            elif key == 's':
                delta = float(raw_input('enter delta (rad): '))
                print 'delta set to %f radians' % delta
            elif key == 'q':
                exit = True
            else:
                continue

    def display_known_poses(self):
        lenhist = len(self.cmd_history)
        lenknown = len(self.known_poses)
        minlenlst = self.cmd_history if lenknown > lenhist else self.known_poses
        maxlenlst = self.cmd_history if minlenlst == self.known_poses else self.known_poses
        line = 'command history\t\t\t\tknown poses' if minlenlst == self.known_poses else 'known poses\t\t\t\tcommand history'
        iterator = itertools.izip_longest(maxlenlst, minlenlst, fillvalue='~')
        print line
        for i, row in enumerate(iterator):
            line = str(i) + ') ' + str(row[0]) + '\t'
            if row[1] != '~':
                line = line + str(i + 300) + ') ' + str(row[1])
            print line

    def get_pose_from_known(self, choice):
        lenhist = len(self.cmd_history)
        lenknown = len(self.known_poses)
        minlenlst = self.cmd_history if lenknown > lenhist else self.known_poses
        maxlenlst = self.cmd_history if minlenlst == self.known_poses else self.known_poses
        if choice < len(maxlenlst):
            print 'choice is from the max list'
            print maxlenlst[choice]
            return maxlenlst[choice]
        else:
            print 'choice is from the min list'
            print minlenlst[choice - 300]
            return minlenlst[choice- 300]

    def go_to_pose_ui(self):
        print 'arm end effector movement'
        choice = raw_input('(r)ight arm, (l)eft arm: ')
        isRightArm = (choice == 'r')
        choice = raw_input('interpolate inverse kinematics?? (y)es, (n)o: ')
        interpolate = (choice == 'y')
        if isRightArm:
            print 'controlling the RIGHT arm'
        else:
            print 'controlling the LEFT arm'
        while True:
            (trans, rot) = self.arm_client.get_transform(isRightArm)
            self.display_known_poses()
            print '\nthe pose order is: x y z qx qy qz qw'
            print '(mistyped commands will be ignored, seven tokens are expected)'
            print 'enter a \'x\' to command that component of the pose to remain constant'
            try:
                lst = raw_input('pose: ')
                tokens = lst.split(' ')
                if len(tokens) != 7 and len(tokens) != 1:
                    print 'invalid number of tokens (expect 7): ' + str(len(tokens))
                    return
                elif len(tokens) == 1 and tokens[0] == 'q':
                    return
                else:
                    cur_pose = [trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]]
                    poses = list()
                    if len(tokens) == 1 and tokens[0] != 'q':
                        lst = self.get_pose_from_known(int(tokens[0]))
                        tokens = lst.split(' ')
                    for index in range(0,7):
                        if tokens[index] == 'x':
                            poses.append(cur_pose[index])
                        elif tokens[index] == 'q':
                            return
                        else:
                            poses.append(float(tokens[index]))
                    self.cmd_history.append(lst)
                    print 'going to poses: ' + str(poses)
                    self.arm_client.move_to_pose(isRightArm, interpolate, poses[0], poses[1], poses[2], poses[3], poses[4], poses[5], poses[6])
                    print 'done'
            except Exception as e:
                print 'something bad happened: ' + str(e)

    def ui_loop(self):
        while True:
            print '\n'
            print 'change (p)osition with const orientation'
            print 'change (o)rientation with const position'
            print '(g)o to arbitrary pose'
            print '(c)lear command history (of arbitrary poses)'
            print '(q)uit'
            choice = raw_input('choice: ')
    
            if choice == 'p':
                self.const_ee_o_ui()
            elif choice == 'o':
                self.const_ee_pos_ui()
            elif choice == 'g':
                self.go_to_pose_ui()
            elif choice == 'c':
                self.cmd_history = list()
            elif choice == 'q':
                return
            else:
                continue

if __name__ == '__main__':
    rospy.init_node('arm_client_ui', anonymous=True)
    acui = ArmClientUI()
    acui.ui_loop()

