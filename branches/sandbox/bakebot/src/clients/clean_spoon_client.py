#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
import rospy
from bakebot.srv import *
from pr2cm_client import *
import tf
from unified_tf_client import UnifiedTFClient
import actionlib
import math
import sys
import time
from utilities.bakebot_controller_manager import *

class CleaningClient:
   
    cleaning_client_instance = None

    def __init__(self, use_right_arm = True, no_switch = False):
        rospy.loginfo('waiting for mixing service')
        rospy.wait_for_service('bakebot_clean_spoon_service')
        rospy.loginfo('done waiting')
        if not no_switch:
            client = PR2CMClient.get_pr2cm_client()
            print client.load_ee_cart_imped(use_right_arm)
            print client.load_cartesian(not use_right_arm)
        self.use_right_arm = use_right_arm
        self.DRY_RUN = False

    @staticmethod
    def get_cleaning_client(use_right_arm = True, no_switch = False):
        if CleaningClient.cleaning_client_instance is None:
            rospy.loginfo('instantiating a new cleaning client')
            CleaningClient.cleaning_client_instance = CleaningClient(use_right_arm, no_switch)
        else:
            rospy.loginfo('returing existing instance of cleaning client')
        return CleaningClient.cleaning_client_instance
    
    def get_bowl_pos_in_tllf(self, bowl_pos_bl):
        tfl = UnifiedTFClient.get_unified_tf_client()
        try:
            (trans, rot) = tfl.lookupTransform('/base_link', '/torso_lift_link', rospy.Time(0)) # TODO: this may be reversed
        except Exception as e:
            print e
            print 'burning off a bad tf reading and waiting'
            time.sleep(1)
            (trans, rot) = tfl.lookupTransform('/base_link', '/torso_lift_link', rospy.Time(0)) # TODO: this may be reversed
        if rot is not (0.0,0.0,0.0,1.0):
            rospy.logerr('nonzero rotation between base link and torso lift link!')
            print rot
        print trans
        print rot
        bx = bowl_pos_bl[0]
        by = bowl_pos_bl[1] 
        bz = bowl_pos_bl[2]
        x = bx - trans[0]  #TODO: check this
        y = by - trans[1]
        z = bz - trans[2]
        print 'position in tll: ', x, y, z
        return (x, y, z)

    def call_service(self, is_right_arm, 
                           initial_position_tllf, initial_fk, initial_is_forces,
                           final_position_tllf, final_fk, final_is_forces,
                           duration):
        if self.DRY_RUN:
            rospy.logwarn('*** Not commanding the clean because this is a DRY_RUN ***')
            return True
        try:
            cleaner = rospy.ServiceProxy('bakebot_clean_spoon_service', CleanSpoon)
            x0 = initial_position_tllf[0]
            y0 = initial_position_tllf[1]
            z0 = initial_position_tllf[2]
            qx0 = initial_position_tllf[3]
            qy0 = initial_position_tllf[4]
            qz0 = initial_position_tllf[5]
            qw0 = initial_position_tllf[6]
            kx0 = initial_fk[0]
            ky0 = initial_fk[1]
            kz0 = initial_fk[2]
            kth0 = initial_fk[3]
            isfx0 = 1 if initial_is_forces[0] else 0
            isfy0 = 1 if initial_is_forces[1] else 0
            isfz0 = 1 if initial_is_forces[2] else 0
            isfth0 = 1 if initial_is_forces[3] else 0
            x1 = final_position_tllf[0]
            y1 = final_position_tllf[1]
            z1 = final_position_tllf[2]
            qx1 = final_position_tllf[3]
            qy1 = final_position_tllf[4]
            qz1 = final_position_tllf[5]
            qw1 = final_position_tllf[6]
            kx1 = final_fk[0]
            ky1 = final_fk[1]
            kz1 = final_fk[2]
            kth1 = final_fk[3]
            isfx1 = 1 if final_is_forces[0] else 0
            isfy1 = 1 if final_is_forces[1] else 0
            isfz1 = 1 if final_is_forces[2] else 0
            isfth1 = 1 if final_is_forces[3] else 0
            ira = True
            response = cleaner(ira, x0, y0, z0, qx0, qy0, qz0, qw0,
                                    kx0, ky0, kz0, kth0,
                                    isfx0, isfy0, isfz0, isfth0,
                                    x1, y1, z1, qx1, qy1, qz1, qw1,
                                    kx1, ky1, kz1, kth1,
                                    isfx1, isfy1, isfz1, isfth1,
                                    duration)
            if response.status is not 0:
                rospy.logerr('clean spoon response indicates failure: ' + str(response))
                return False
            else:
                rospy.loginfo('successful clean spoon action')
                return True
        except rospy.ServiceException, e:
            rospy.logerr('the service call failed: ' + str(e))
            print e
            return False

    def get_initial_pos(self, mixing_bowl_radius, mixing_bowl_height, mixing_bowl_pos_tll, is_barcode_up):
        if is_barcode_up:
            qdes = [.5, .5, .5, .5]
        else:
            qdes = [.5, .5, -.5, -.5]
        xoff = 0
        yoff = -.25 - mixing_bowl_radius
        zoff = mixing_bowl_height - .02
        print 'mixing_bowl_pos_tll:', mixing_bowl_pos_tll
        print 'mixing bowl radius: ', mixing_bowl_radius
        print 'mixing bowl height: ', mixing_bowl_height
        print 'offsets:', (xoff,yoff,zoff)
        initial_pos = [mixing_bowl_pos_tll[0] + xoff,
                       mixing_bowl_pos_tll[1] + yoff,
                       mixing_bowl_pos_tll[2] + zoff]
        initial_pos.extend(qdes)
        return initial_pos
        

    def compliant_clean_spoon(self, mixing_bowl_pos_tll, mixing_bowl_radius, mixing_bowl_height, is_barcode_up,
                              stiff = 1000, rotstiff = 50, zforcemag = 5, duration = 10):
        xmove = 0
        ymove = -.10
        zmove = -0.1
        initial_pos = self.get_initial_pos(mixing_bowl_radius, mixing_bowl_height, mixing_bowl_pos_tll, is_barcode_up)
        qdes = initial_pos[3:7]
        print initial_pos
        print qdes
        final_pos =   [initial_pos[0] + xmove,
                       initial_pos[1] + ymove,
                       initial_pos[2] + zmove]
        final_pos.extend(qdes)
        #initial_fk = [stiff, stiff, -1*zforcemag, rotstiff]
        initial_fk = [stiff, stiff, 300, rotstiff]
        final_fk = initial_fk
        initial_is_forces = [False, False, False, False]
        final_is_forces = initial_is_forces
        print 'cleaning spoon from initial ', initial_pos, ' to final ', final_pos, ' with duration ', duration
        return self.call_service(True, initial_pos, initial_fk, initial_is_forces,
                                       final_pos, final_fk, final_is_forces, duration)

    def scrape(self, negy_move = -.02, negz_move = .10, xmove = 0, duration = 4, stiff = 1000, dstiff = 1000, rotstiff = 50, zstartoff = 0, yoff =0):
# neg y used to be -.05
# neg z move used ot be .15
        # starts from the plunged position and ends outside
        tf = UnifiedTFClient.get_unified_tf_client()
        (trans, rot) = tf.lookupTransform('torso_lift_link', 'r_wrist_roll_link', rospy.Time(0))
        #trans[2] = trans[2] + zstartoff
        trans = (trans[0], trans[1]+yoff, trans[2] + zstartoff)
        p1 = list()
        p1.extend(trans)
        p1.extend(rot)
        print 'p1: ', p1
        fk1 = [stiff, stiff, dstiff, rotstiff]
        isf1 = [False, False, False, False]
        # this is the movement down and out
        p2 = list()
        for o in p1:
            p2.append(o)
        p2[0] = p2[0] + xmove
        p2[1] = p2[1] - negy_move
        p2[2] = p2[2] - negz_move
        #p2[3] = .5
        #2[4] = .5
        #2[5] = .5
        #2[6] = .5
        #fk2 = fk1
        #fk2[3] = 100
        fk2 = [stiff, stiff, stiff, 10]
        isf2 = isf1
        print 'p2: ', p2
        # this moves it to the pre plunge position
        p3 = list()
        for o in p1:
            p3.append(o)
        #p3[1] = p3[1] - negy_move
        fk3 = [stiff, stiff, stiff, rotstiff]
        isf3 = isf1
        print 'p3: ', p3
        rospy.loginfo('calling service to go from p1 to p2 (from in bowl to post scrape position)')
        self.call_service(self.use_right_arm, p1, fk1, isf1, p2, fk2, isf2, duration)
        (trans, rot) = tf.lookupTransform('torso_lift_link', 'r_wrist_roll_link', rospy.Time(0))
        rospy.loginfo('calling service to go from p2 to p3 (from post scrape to pre plunge')
        self.call_service(self.use_right_arm, p2, fk2, isf2, p3, fk3, isf3, duration)



if __name__ == '__main__':
    csc = get_cleaning_client(use_right_arm = True)
    while True:
        raw_input('press enter to do scrape action (from plunged position)')
        csc.scrape()

