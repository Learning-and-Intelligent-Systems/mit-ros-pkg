#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
import rospy
from bakebot.srv import *
from pr2cm_client import *
from services.bakebot_logging import *
import tf
import actionlib
import math
import sys
import time
from utilities.bakebot_controller_manager import *

class MixingClient:
# this class also does SCRAPING
   
    left_mixing_client_instance = None
    right_mixing_client_instance = None
    correct_z = -0.25

    def __init__(self, use_right_arm):
        rospy.loginfo('waiting for mixing service')
        rospy.wait_for_service('bakebot_mixing_service')
        rospy.loginfo('done waiting')
        client = PR2CMClient.get_pr2cm_client()
        print client.load_ee_cart_imped(use_right_arm)
        print client.load_cartesian(not use_right_arm)
        self.use_right_arm = use_right_arm
        self.DRY_RUN = False

    @staticmethod
    def get_mixing_client(use_right_arm = True):
        if use_right_arm:
            rospy.loginfo('getting mixing client for RIGHT arm !!!!!!!!!!!!!!!!!!!!!!!!!')
            if MixingClient.right_mixing_client_instance == None:
                rospy.loginfo('instantiating a new mixing client')
                MixingClient.right_mixing_client_instance = MixingClient(use_right_arm)
            else:
                rospy.loginfo('returning existing mixing client')
            return MixingClient.right_mixing_client_instance
        else:
            rospy.loginfo('getting mixing client for LEFT arm !!!!!!!!!!!!!!!!!!!!!!!!!')
            if MixingClient.left_mixing_client_instance == None:
                rospy.loginfo('instantiating a new mixing client')
                MixingClient.left_mixing_client_instance = MixingClient(use_right_arm)
            else:
                rospy.loginfo('returning existing mixing client')
            return MixingClient.left_mixing_client_instance
    
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

    def call_service(self, bowl_xyz_tllf = (.4,-.25,-.15), bowl_radius = 0.15, pre_mix_height_above_bz = .4,
                           is_mix = True,
                           lower_spoon = False, do_circular_mix = False, do_linear_mix = False, 
                           do_whisk_mix = False, raise_spoon = False,
                           circular_laps = 1, linear_laps = 1, whisk_laps = 0, #TODO: may want to change to 1, affects orientation of the plunge spoon when ismix
                           circular_period_sec = 7, linear_period_sec = 5, whisk_period_sec = 10, 
                           whisk_ang_vel = 3, angular_force_mag = 7, down_force_mag = 10, 
                           linear_force_mag = 10, radial_stiffness = 500, rotational_stiffness = 50, 
                           radial_pad = .04, is_angular_force = False, is_down_force = True):
        if self.DRY_RUN:
            rospy.logwarn('*** Not commanding the mix because this is a DRY_RUN ***')
            return True

        isRightArm = self.use_right_arm
        print 'is mix?', is_mix
        if not is_mix:
            rospy.loginfo('sending a service request for a SCRAPING action')

        try:
            mixer = rospy.ServiceProxy('bakebot_mixing_service', MixBowl)
            ira = 1 if isRightArm else 0
            ismix = 1 if is_mix else 0
            bx = bowl_xyz_tllf[0]
            by = bowl_xyz_tllf[1]
            bz = bowl_xyz_tllf[2]
            pmh = pre_mix_height_above_bz
            ls = 1 if lower_spoon else 0
            dcm = 1 if do_circular_mix else 0
            dlm = 1 if do_linear_mix else 0
            dwm = 1 if do_whisk_mix else 0
            rs = 1 if raise_spoon else 0

            radial_stiff = radial_stiffness
            rot_stiff = rotational_stiffness
            rad_pad = radial_pad
            isangf = is_angular_force
            isdownf = is_down_force
            response = mixer(ira, bx, by, bz, bowl_radius, pmh, ismix, ls, dcm, dlm, dwm, rs,
                             circular_laps, linear_laps, whisk_laps, circular_period_sec,
                             linear_period_sec, whisk_period_sec, whisk_ang_vel, 
                             angular_force_mag, down_force_mag, linear_force_mag,
                             radial_stiff, rot_stiff, rad_pad, isangf, isdownf)

            if response.status is not 0:
                rospy.logerr('mixing response indicates failure: ' + str(response))
                return False
            else:
                rospy.loginfo('successfull mixing action')
                return True
        except rospy.ServiceException, e:
            rospy.logerr('the service call failed: ' + str(e))
            print e
            return False

    def plunge_spoon(self, bowl_x = .4, bowl_y = -.25, bowl_z = 0, noonsixorientation = True, ismix = True):
        logger = EventLoggerClient(EventLogger.event_type_mixing, 'plunge spoon')
        print 'bowl x', bowl_x
        print 'bowl y', bowl_y
        print 'bowl z', bowl_z
        rospy.loginfo('commanding a spoon plunge')
        wl = 1 if noonsixorientation else 0
        status = self.call_service(bowl_xyz_tllf = (bowl_x, bowl_y, bowl_z), whisk_laps = wl, lower_spoon = True, is_mix = ismix)
        if status:
            logger.stops()
        else:
            logger.stopf()
        return status

    def deplunge_spoon(self, bowl_pos, pre_mix_height_above_bz, ismix = True):
        logger = EventLoggerClient(EventLogger.event_type_mixing, 'deplunge spoon')
        rospy.loginfo('commanding a spoon deplunge')
        status =self.call_service(raise_spoon  = True, bowl_xyz_tllf = bowl_pos, pre_mix_height_above_bz = pre_mix_height_above_bz, is_mix = ismix)
        if status:
            logger.stops()
        else:
            logger.stopf()
        return status

    #def circle_mix(self, bowl_pos, circular_laps = 3, circular_period_sec = 6, angular_force_mag = 8, down_force_mag = 5, ismix = True):
    def circle_mix(self, bowl_pos, circular_laps = 5, circular_period_sec = 7, angular_force_mag = 8, down_force_mag = 10, ismix = True):
        logger = EventLoggerClient(EventLogger.event_type_mixing, 'circle mix, laps='+str(circular_laps)+' period='+ str(circular_period_sec))
        rospy.loginfo('commanding a circle mix')
        claps = circular_laps
        cperd = circular_period_sec
        afm = angular_force_mag
        dfm = down_force_mag
        if not ismix:
            radstiff = 400
            rotstiff = 100
            idf = True
        else:
            print '8888888888888888888888888888888888888888888888888888888888888888888888'
            print 'is mix = True TRUE TRUE TRUE TRUE'
            print 'position tllf: ',  bowl_pos
            print 'setting the bowl position z to ',MixingClient.correct_z
            #bowl_pos = (bowl_pos[0], bowl_pos[1], -.25)
            bowl_pos = (bowl_pos[0], bowl_pos[1], MixingClient.correct_z)
            print 'corrected pose: ', bowl_pos
            print '8888888888888888888888888888888888888888888888888888888888888888888888'
            radstiff = 500
            rotstiff = 50 
            idf = False
        status = self.call_service(bowl_xyz_tllf = bowl_pos, is_mix = ismix, do_circular_mix = True, circular_laps = claps, circular_period_sec = cperd, angular_force_mag = afm, down_force_mag = dfm, radial_stiffness = radstiff, rotational_stiffness = rotstiff, is_down_force = idf)
        if status:
            logger.stops()
        else:
            logger.stopf()
        return status

    def linear_mix(self, bowl_pos, is_mix = True, linear_laps = 3, linear_period_sec = 1, linear_force_mag = 10, down_force_mag = 10):
        logger = EventLoggerClient(EventLogger.event_type_mixing, 'linear mix, laps='+str(linear_laps)+' period='+ str(linear_period_sec))
        rospy.loginfo('commanding a linear mix')
        if not is_mix:
            rospy.logerr('linear mix is unspecified for scraping action (is_mix is false)')
            return False
        laps = linear_laps
        perd = linear_period_sec
        lfm = linear_force_mag
        dfm = down_force_mag
        bowl_pos = (bowl_pos[0], bowl_pos[1], MixingClient.correct_z)
        status = self.call_service(bowl_xyz_tllf = bowl_pos, is_mix = is_mix, do_linear_mix = True, linear_laps = laps, linear_period_sec = perd, linear_force_mag = lfm, down_force_mag = dfm, is_down_force = False)
        if status:
            logger.stops()
        else:
            logger.stopf()
        return status

    def whisk_mix(self, bowl_pos, is_mix = True, whisk_laps = 1, whisk_period_sec = 10, whisk_ang_vel = 3, angular_force_mag = 7, down_force_mag = 10):
        rospy.loginfo('commanding a whisk mix')
        if not is_mix:
            rospy.logerr('whisk mix is unspecified for scraping action (is_mix is false)')
            return False
        laps = whisk_laps
        perd = whisk_period_sec
        afm = angular_force_mag
        dfm = down_force_mag
        wav = whisk_ang_vel
        return self.call_service(bowl_xyz_tlff = bowl_pos, do_whisk_mix = True, whisk_laps = laps, whisk_period_sec = perd, whisk_ang_vel = wav, angular_force_mag = afm, down_force_mag = dfm)

    def interface_loop(self):
        x = .4
        y = -.25
        #z = -.168
        z = -.25
        while True:
            #Jlaps = 1
            #Jperiod = 10
            #Jangular = 7
            #Jlinear = 10
            #Jdown = 3
            ISMIX = True
            choice = raw_input('\n(c)ircle mix, (l)inear mix, (w)hisk mix, (p)lunge, (a)ll, (r)aise, (q)uit: ')
            if choice == 's':
               #J(laps, period, angular, linear, down) = self.set_param_ui() 
               pass
            elif choice == 'c':
                self.circle_mix((x, y, z), ismix = ISMIX)
            elif choice == 'l':
                self.linear_mix((x, y, z))
            elif choice == 'w':
                self.whisk_mix((x, y, z))
            elif choice == 'p':
                self.plunge_spoon(bowl_x = x, bowl_y = y, bowl_z = z, ismix = ISMIX)
            elif choice == 'd':
                self.deplunge_spoon()
            elif choice == 'q':
                return
            elif choice == 'a':
                self.circle_mix((x, y, z), circular_laps = 1)
                self.linear_mix((x, y, z), linear_laps = 1)
                self.circle_mix((x, y, z), circular_laps = 1)
                self.linear_mix((x, y, z), linear_laps = 1)
            elif choice == 'C':
                print 'scrape'
                self.circle_mix((x, y, z), is_mix = ISMIX)
            elif choice == 'P':
                self.plunge_spoon(bowl_x = x, bowl_y = y, bowl_z = z, ismix = ISMIX, noonsixorientation=False)
            elif choice == 'D':
                print 'scrape'
                self.deplunge_spoon(is_mix = ISMIX)
            else:
                print 'invalid input'

    #Jdef set_param_ui(self):
        #Jprint '1) laps:', laps
        #Jprint '2) period:', period
        #Jprint '3) angular: ', angular
        #Jprint '4) linear: ', linear
        #Jprint '5) down: ', down
#J
        #Jchoice = int(raw_input('choice: '))
        

if __name__ == '__main__':
    rospy.init_node('mixing_client_tester', anonymous=True)
    mc = MixingClient(True)
    mc.interface_loop()
