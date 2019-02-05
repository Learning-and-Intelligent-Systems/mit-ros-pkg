#!/usr/bin/env python
import roslib
#load in all the paths to the packages listed in 'manifest.xml'
roslib.load_manifest('kinect_joy')
#basic ros python commands:
import rospy
#import message types:
from body_msgs.msg import Skeletons

#//this is how to transform the data to be in a form the pr2 end effector will like:
def myCallback(skels):
    if len(skels.skeletons) == 0:
        return
    skel=skels.skeletons[0]
    ry=skel.right_hand.position.x-skel.torso.position.x
    rz=-skel.right_hand.position.y+skel.torso.position.y + .5
    rx=-(skel.right_hand.position.z-skel.torso.position.z)+.15
    
    ly=skel.left_hand.position.x-skel.torso.position.x
    lz=-skel.left_hand.position.y+skel.torso.position.y + .5
    lx=-(skel.left_hand.position.z-skel.torso.position.z)+.15
    print "right: %.03f %.03f %.03f   left:  %.03f %.03f %.03f "%(rx,ry,rz,lx,ly,lz)
 
#this command registers this process with the ros master
rospy.init_node('getskels')
sub = rospy.Subscriber("/skeletons", Skeletons, myCallback)

rospy.spin()

