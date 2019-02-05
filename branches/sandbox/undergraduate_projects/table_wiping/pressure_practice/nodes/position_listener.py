#!/usr/bin/env python  
import roslib
roslib.load_manifest('pressure_practice')
import rospy
import math
import tf


if __name__ == '__main__':

    rospy.init_node('tf_listener')
    listener = tf.TransformListener()

    while True:
        try:
            (trans, rot) = listener.lookupTransform('torso_lift_link', 'l_gripper_palm_link', rospy.Time(0))
        except:
            continue
        print (trans, rot)
