#!/usr/bin/python
import roslib
roslib.load_manifest('mm_test')
import rospy
import math
import tf

if __name__ == '__main__':
    rospy.init_node('mm_test_transforms')
    listener =  tf.TransformListener()

    rate = rospy.Rate(1.0)
    
    link_from = '/base_link'
    link_to = '/r_gripper_tool_frame' 
    
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(link_from, link_to, rospy.Time(0))
            print 'from',link_from,' to',link_to
            print 'trans',trans
            print 'rot',rot
        except (tf.LookupException, tf.ConnectivityException):
            continue
        rate.sleep()

