#!/usr/bin/env python
import roslib
roslib.load_manifest('object_survey')
import rospy
import rosbag
import object_survey.srv
from object_survey.msg import CameraScan
import geometry_msgs.msg

import image_geometry.cameramodels
import cv_bridge
import cv

if __name__ == '__main__':
    service_name = "/get_camera_scan"
    print "waiting for service '"+service_name+"'..."
    rospy.wait_for_service(service_name)
    print "done."
    client = rospy.ServiceProxy(service_name,object_survey.srv.GetCameraScan)
    
    focus_point = geometry_msgs.msg.PointStamped()
    focus_point.point.x = 1.0
    focus_point.point.y = 0.0
    focus_point.point.z = 0.0
    focus_point.header.frame_id = 'torso_lift_link'

    result = client(focus_point)
    print result.scan.highdef_image.header

    bridge = cv_bridge.CvBridge()
    im_msg = result.scan.highdef_image#narrow_stereo_right_image
    print "converting to encoding: '%s'"%(im_msg.encoding)
    im = bridge.imgmsg_to_cv(im_msg, im_msg.encoding)
    resize = .4
    height = int(im.rows*resize)
    width = int(im.cols*resize)
    small = cv.CreateMat(height,width,cv.CV_8UC3)
    cv.Resize(im,small)
    window_name = "captured image"
    cv.ShowImage(window_name,small)
    cv.WaitKey()


    print dir(result)
    print result.scan.wide_stereo_left_caminfo
