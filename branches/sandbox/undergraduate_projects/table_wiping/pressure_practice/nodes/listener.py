#!/usr/bin/env python
import roslib; roslib.load_manifest('pressure_practice')
import rospy
from ee_cart_imped_action import *
from position_listener import *
import math
from ee_cart_imped_control import *
from fingertip_pressure.msg import PressureInfo
from geometry_msgs.msg import Vector3
from pr2_msgs.msg import PressureState
from sound_play.msg import SoundRequest
import threading
import tf
import position_listener
import sys
from drive_base import drive_base_client
from drive_base import msg
import table_detector
pressure_threshold = 3400
wait_threshold = .1
min_y = -.25
max_y = .51
class pressure_listener:
    def __init__(self):
        self.thread = threading.Thread(target = callback)
        self.thread.start()
        rospy.init_node('listener', anonymous=True)
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber("/pressure/l_gripper_motor", PressureState, self.callback)
        self.wait = 0
        self.state = 'not_touching'
        self.pub = rospy.Publisher('/robotsound', SoundRequest)
        self.arm_control = ee_cart_imped_client(True)
        self.arm_control.addTrajectoryPoint(.4, .5, 0, 0,0.707,-0.0,0.707,
                                     1000, 1000, 1000, 30, 30, 30,
                                     False, False, False, False, False,
                                     False, 5.0)
        self.arm_control.sendGoal()
        self.arm_control.resetGoal()
	self.base_control = drive_base_client.DriveBaseClient()
    def callback(self,data):
        frontSensors = data.l_finger_tip[3:5]
        frontSensors = frontSensors + data.r_finger_tip[3:5]
        s = sum(frontSensors)/4.0
        if (self.state == 'not_touching' or self.state == 'waiting') and s > pressure_threshold:
            self.wait = 0
            self.state = 'touching'
        if self.state == 'touching' and s < pressure_threshold:
            self.state = 'waiting'
            self.wait = rospy.get_time()
        if self.state == 'waiting' and s < pressure_threshold and rospy.get_time()-self.wait > wait_threshold:
            self.state = 'not_touching'

    def getHandPose(self):
        while True:
            try:
                return self.tf_listener.lookupTransform('/torso_lift_link', '/l_gripper_tool_frame', rospy.Time(0))
            except:
                continue 
    def hitTable(self, x, y, z, t):
        self.arm_control.addTrajectoryPoint(x, y, z+.05, 0,0.707,-0.0,0.707,
                                     1000, 1000, 1000, 30, 30, 30,
                                     False, False, False, False, False,
                                     False, t/2.0)
        self.arm_control.addTrajectoryPoint(x, y, z,  0.0 ,0.707,-0.0,0.707,
                                     1000, 1000, -8, 30, 30, 30,
                                     False, False, True, False, False,
                                     False, t)
        self.arm_control.sendGoal()
    def wipeTable(self):
        time = 3
        self.arm_control.resetGoal()
	self.arm_control.addTrajectoryPoint(0.0, .1, 0,  0.566,0.493,-0.466,0.468,
                                         1000, 1000, -7, 30, 30, 30,
                                         False, False, True, False, False,
                                         False, time);
        self.arm_control.sendGoal(False)
        print 'yay'
        rospy.sleep(1.0)
#loop to check that we are still touching
#if we are not, cancel the action and command the hand
# to stay where is currently is
        while True: 
            if self.state == "not_touching":
                self.arm_control.cancelGoal()
                self.arm_control.resetGoal()
                (trans, rot) = self.getHandPose()
                print "trans = \t" + str(trans)
                print "rot = \t" + str(rot)
                self.arm_control.addTrajectoryPoint(trans[0], trans[1], trans[2], 
                                                    rot[0], rot[1], rot[2], rot[3],
                                                    1000,1000,1000,30,30,30,
                                                    False, False, False, False, False, False, 1);
                self.arm_control.sendGoal()
		print "Goal Cancelled"
                break
 #move the hand straight up by .12 m
    def straight_up(self):
        self.arm_control.resetGoal()
        (trans, rot) = self.getHandPose()

#        self.arm_control.moveToPoint((trans[0],0,trans[2]),rot,1)
        trans=(trans[0],trans[1],trans[2]+.12)
        self.arm_control.moveToPoint(trans,rot,1.5)
        self.arm_control.sendGoal()
# wipe the table in a straight line towards the robot at y coordinate
#y, stop when you reach the end of the table. Y coordinate is in torso lift link frame
    def wipe(self,y):
        self.arm_control.resetGoal()
        (trans,rot)=self.getHandPose()
        self.arm_control.addTrajectoryPoint(0.75, y, trans[2],  0.566,0.493,-0.466,0.468,
                                          1000, 1000, 1000, 30, 30, 30,
                                          False, False, False, False, False,
                                          False, 1.5)
       
        self.arm_control.addTrajectoryPoint(0.75, y, 0,  0.566,0.493,-0.466,0.468,
                                        1000, 1000, -5, 30, 30, 30,
                                        False, False, True, False, False,
                                        False, 3)
        self.arm_control.sendGoal()
        self.arm_control.resetGoal()
        self.arm_control.addTrajectoryPoint(0.0, y, 0,  0.566,0.493,-0.466,0.468,
                                          1000, 1000, -7, 30, 30, 30,
                                          False, False, True, False, False,
                                          False, 3);
        self.arm_control.sendGoal(False)
        rospy.sleep(1.0)
        while True:
            if self.state == "not_touching":
                self.arm_control.cancelGoal()
                self.arm_control.resetGoal()
                (trans, rot) = self.getHandPose()
#                print "trans = \t" + str(trans)
#                print "rot = \t" + str(rot)
                self.arm_control.addTrajectoryPoint(trans[0], trans[1], trans[2], 
                                                    rot[0], rot[1], rot[2], rot[3],
                                                    1000,1000,1000,30,30,30,
                                                    False, False, False, False, False, False, 1);
                self.arm_control.sendGoal()
#		print "Goal Cancelled"
                break
#wipe the "wipeable" area of the table that is in front of the robot
    def wipe(self):
        current_wipe = min_y
        while current_wipe < y_max:
            self.straight_up()
            self.wipe(current_wipe)
            current_wipe += .1

#use the base to move until the gripper leave the edge of the table, finds
#stops when it reaches the edge.
#assunmes that the gripper is already touching the table
    def find_edge(self):
        while self.state == "touching":
            self.move_base(0, -0.01, 0.1)
        cancel_move()
        (trans,rot) = self.getHandPose()
        self.arm_control.cancelGoal()
        self.arm_control.resetGoal()
        self.arm_control.addTrajectoryPoint(trans[0], trans[1], trans[2], 
                                            rot[0],rot[1],rot[2],rot[3],
                                            1000,1000,1000,30,30,30,
                                            False,False,False,False,False,False,1)
        self.arm_control.sendGoal()
# move the base to the specified position at the specified velocity
    def move_base(self, x, y, vel):
        self.base_control.Move(x, y, vel)

    def turn_base(self, theta, clockwise):
        self.base_control.Turn(theta, clockwise)

def cancel_move():
    moveClient = actionlib.SimpleActionClient("move_base_action", msg.moveBaseAction)
    moveClient.cancel_all_goals()
def cancel_turn():
    turnClient = actionlib.SimpleActionClient("turn_base_action", msg.moveBaseAction)
    moveClient.cancel_all_goals() 
#move the right hand for that it will "follow" the specified x and y coordinates
def follow(right,x, y):
    right.resetGoal()
    right.moveToPoint((x-.06, y - .11, -.45), (0,.707,0,.707), 2)
    right.sendGoal(False)
if __name__ == '__main__':
    l = pressure_listener()
    r = ee_cart_imped_client(False)

################################################
##This is our table wiping demo from the video##
################################################
#    r.moveToPoint((0.320, -.2, -.45),(0,.707,0,.707),3)
#    r.sendGoal()
#    start = -.1
#    for i in range(6):
#        l.straight_up()
#        (trans, rot) = l.getHandPose()
#        follow(r,trans[0],start)
#        l.wipe(start)
#        start+=.1
#get the position of the table
    (x,y,z) = table_detector.get_detection_msg(l.tf_listener)
    print "x=\t" + str(x)
    print "y=\t" + str(y)
    print "z=\t" + str(z)
    print "\n"
#put the gripper on the center of the table
    l.hitTable(x,y,z,10)
    (trans,rot) = l.getHandPose()
    print "x=\t" + str(trans[0])
    print "y=\t" + str(trans[1])
    print "z=\t" + str(trans[2])
#find the edge of the table
    l.find_edge()
    rospy.spin()
