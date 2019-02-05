#!/usr/bin/env python  
# @author mbollini@mit.edu

import roslib
roslib.load_manifest('bakebot')
import rospy
import math
from datetime import datetime, date, time
import tf

if __name__ == '__main__':
    rospy.init_node('tf_ee_logger')
    listener = tf.TransformListener()

    baseframe = '/base_link'
    otherframe = '/r_wrist_roll_link'

    filename = raw_input('enter filename: ')
    myfile = open(filename, 'a+')
    t = datetime.now()
    myfile.write('\n************************* ' + t.isoformat() + '\n')
    myfile.write('recording transformations from ' + baseframe + ' to ' + otherframe + '\n')
    count = 0

    while not rospy.is_shutdown():
        raw_input('press ENTER to record current position...')
        try:
            (trans, rot)  = listener.lookupTransform(baseframe, otherframe, rospy.Time(0))
            record = '%dth data point:\n' % count
            record = record + 'trans: ' + str(trans) + '\nrot: ' + str(rot) + '\n\n'
            print 'just wrote (to ' + filename + ') ' + record
            myfile.write(record)
            count = count + 1
        except (tf.LookupException, tf.ConnectivityException):
            continue
