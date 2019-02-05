#!/usr/bin/env python
import roslib; roslib.load_manifest('target_nav')

from target_nav.srv import *
from geometry_msgs.msg import Pose,PoseStamped
import rospy
import tf
import time

position=(0,0,0)
orientation=(0,0,0,1) #quaternion





def localize(req):
    #would change position, orientation here
    #TODO: add actual checkerboard finder call
    print 'localize called'
    #in the meanwhile, return the pose of the robot in the world frame
    ps=PoseStamped()
    ps.header.frame_id="/base_link"
    ps.header.stamp=rospy.Time(0)
    psout = listener.transformPose("/world",ps)
    return UpdateEstimateResponse(True,psout)

if __name__ == "__main__":
    rospy.init_node('Localization_server')
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    s = rospy.Service('/checkerboard_localize', UpdateEstimate, localize)
    print "Ready to localize."
    r = rospy.Rate(100) # 30hz
    while not rospy.is_shutdown():
        r.sleep()
        br.sendTransform(position,orientation, rospy.Time.now(), "/odom_combined", "/world")
        

