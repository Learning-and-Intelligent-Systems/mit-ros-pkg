#!/usr/bin/env python
import roslib; roslib.load_manifest('furniture')
import rospy
import cv
import os
import sys
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

image_num = 0
got_disparity = False
got_left = False
got_right = False
bridge = CvBridge()


def disparityCallback(msg):
    global image_num, got_disparity, bridge
    if got_disparity==True:
        return
    print "DisparityImage:"
    print "  f = " + str(msg.f)
    print "  T = " + str(msg.T)
    print "  min_disparity = " + str(msg.min_disparity)
    print "  max_disparity = " + str(msg.max_disparity)
    print "  delta_d = " + str(msg.delta_d)
    print ""
    try:
        cv_image = bridge.imgmsg_to_cv(msg.image, "passthrough")
        cv.SaveImage("disp" + str(image_num) + ".png", cv_image)
        got_disparity = True
    except CvBridgeError, e:
        print e
    

def leftImageCallback(msg):
    global image_num, got_left, bridge
    if got_left:
        return
    try:
        cv_image = bridge.imgmsg_to_cv(msg, "passthrough")
        cv.SaveImage("left" + str(image_num) + ".png", cv_image)
        got_left = True
    except CvBridgeError, e:
        print e
    
def rightImageCallback(msg):
    global image_num, got_right, bridge
    if got_right:
        return
    try:
        cv_image = bridge.imgmsg_to_cv(msg, "passthrough")
        cv.SaveImage("right" + str(image_num) + ".png", cv_image)
        got_right = True
    except CvBridgeError, e:
        print e
    

def getImageNum():
    global image_num
    for f in filter(lambda f: f.endswith(".png"), os.listdir('.')):
        x_str = filter(lambda c: c.isdigit(), f.split('.')[0])
        if len(x_str) > 0:
            x = int(x_str)
            if x >= image_num:
                image_num = x+1
    print "image_num = " + str(image_num)

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    getImageNum()
    stereo_base = "/narrow_stereo_textured"
    if len(sys.argv) > 1:
        stereo_base = sys.argv[1]
    rospy.Subscriber(stereo_base + "/disparity", DisparityImage, disparityCallback)
    rospy.Subscriber(stereo_base + "/left/image_rect", Image, leftImageCallback)
    rospy.Subscriber(stereo_base + "/right/image_rect", Image, rightImageCallback)
    r = rospy.Rate(10)  # 10hz
    while not(got_disparity) or not(got_left) or not(got_right):
        if rospy.is_shutdown():
            break
        r.sleep()
