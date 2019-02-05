#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
import rospy
from bakebot.srv import *
import tf
import actionlib
import math
import sys
import time

class UnifiedTFClient(tf.TransformListener):
    
    utf_client_instance = None

    @staticmethod
    def get_unified_tf_client():
        if UnifiedTFClient.utf_client_instance == None:
            rospy.loginfo('instantiating a new unified tf client')
            UnifiedTFClient.utf_client_instance = UnifiedTFClient(tf.TransformListener())
            #TODO: put while loop here to iterate until it stops throwing exceptions
        else:
            rospy.loginfo('returning existing instance of unified tf client ' + str(UnifiedTFClient.utf_client_instance))
        return UnifiedTFClient.utf_client_instance

    def __init__(self, transformListener):
        tf.TransformListener.__init__(self)
        rospy.loginfo('done with unified tf client init')

if __name__ == '__main__':
    rospy.init_node('test_tf', anonymous=True)
    utf = UnifiedTFClient.get_unified_tf_client()
    while(not rospy.is_shutdown()):
        try:
            (trans, rot) = utf.lookupTransform('base_link', 'r_wrist_roll_link', rospy.Time(0))
            print trans, rot
        except Exception as e:
            rospy.loginfo('tf is all messed up in the arm client')
