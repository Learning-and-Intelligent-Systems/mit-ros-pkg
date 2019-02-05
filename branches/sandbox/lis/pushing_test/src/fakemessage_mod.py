#!/usr/bin/env python
import roslib; roslib.load_manifest('furniture')
import rospy
from geometry_msgs.msg import Pose2D
from furniture.msg import Table_Poses

import numpy
import sys

if __name__=='__main__':

    x = .5
    y = .19
    t = 0
    #x_stdev = .005
    #y_stdev = .005
    #t_stdev = .045
    x_stdev = 0
    y_stdev = 0
    t_stdev = 0
    
    if len(sys.argv)>2:
        x = sys.argv[1]
        y = sys.argv[2]
        t = sys.argv[3]
    if len(sys.argv)==6:
        x_stdev = sys.argv[4]
        y_stdev = sys.argv[5]
        t_stdev = sys.argv[6]
    
    rospy.init_node('box_pose_fake_publisher')
    try: 
        pub = rospy.Publisher('table_poses', Table_Poses)
        while not rospy.is_shutdown():
            tablePose = Pose2D()
            tablePose.x = float(y)+.5
            tablePose.y = -float(x)+.2
            tablePose.theta = float(t)
            tablePoses = Table_Poses()
            tablePoses.ids = [0]
            tablePoses.poses = [tablePose]
            pub.publish(tablePoses)
            print 'publishing'
            print tablePoses
            rospy.sleep(1.0)
    except rospy.ROSInterruptException, e:
        print e

        
