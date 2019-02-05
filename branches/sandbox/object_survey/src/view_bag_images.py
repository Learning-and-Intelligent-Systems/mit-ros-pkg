#!/usr/bin/env python
import roslib
roslib.load_manifest('object_survey')
import rospy
import rosbag
from object_survey.msg import SurveyScan
#from furniture_ops.srv import AlignClouds
#import poisson_surface_reconstruction.srv
import image_geometry.cameramodels
import cv_bridge
import cv
import sys
from tf import transformations
from tf.transformations import inverse_matrix
import numpy
import geometry_msgs.msg

def transform_matrix(pos,rot):
  T = numpy.dot(transformations.translation_matrix(pos), transformations.quaternion_matrix(rot))
  return T

def transform_from_matrix(T):
  r = transformations.quaternion_from_matrix(T)
  o = (T[0][3],T[1][3],T[2][3])
  return o,r

def geometry_msg_to_matrix(m):
    p = (m.transform.translation.x,m.transform.translation.y,m.transform.translation.z)
    r = (m.transform.rotation.x,m.transform.rotation.y,m.transform.rotation.z,m.transform.rotation.w)
    return transform_matrix(p,r)

def matrix_to_geometry_msg(T):
    (p,r) = transform_from_matrix(T)
    m = geometry_msgs.msg.Transform()
    m.translation.x = p[0]
    m.translation.y = p[1]
    m.translation.z = p[2]
    m.rotation.x = r[0]
    m.rotation.y = r[1]
    m.rotation.z = r[2]
    m.rotation.w = r[3]
    return m

if __name__ == '__main__':
    if len(sys.argv)<2:
        print "USAGE: view_bag_images.py bagfile.bag"
        print "  bagfile.bag should have a survey_results topic"
        exit()

    bagfile = sys.argv[1]

    bag = rosbag.Bag(bagfile)
    msg_index = 0
    clouds = []
    transforms = []
    #for topic, msg, t in bag.read_messages(topics=['analyzed_scans']):#'survey_results']):
    print "about to look for survey_results in "+bagfile
    print bag
    print bag.read_messages()


    for topic, msg, t in bag.read_messages():#topics=['survey_results']):
        print "topic:",topic
        #print "dir(msg):",dir(msg)
        #print "dir(msg.camera_scans[0]):",dir(msg.camera_scans[0])
        print "msg.camera_scans[0].narrow_stereo_right_caminfo:", msg.camera_scans[0].narrow_stereo_right_caminfo
        continue
        for j in range(len(msg.camera_scans[0].narrow_stereo_cloud.fields)):
          print msg.camera_scans[0].narrow_stereo_cloud.fields[j].name
        print "t:",t
        
        bridge = cv_bridge.CvBridge()

        im_msg = msg.camera_scans[0].narrow_stereo_left_image#wide_stereo_right_image
        if im_msg.height==0 or im_msg.height==0:
          print "problem: empty image"
          continue
        
        im = bridge.imgmsg_to_cv(im_msg, im_msg.encoding)
        resize = .4
        height = int(im.rows*resize)
        width = int(im.cols*resize)
        if im_msg.encoding=="mono8":
          encoding = cv.CV_8U
        elif im_msg.encoding=="bgr8":
          encoding = cv.CV_8UC3
        else:
          print "problem: unknown encoding: "+im_msg.encoding
          continue

        small = cv.CreateMat(height,width,encoding)
        cv.Resize(im,small)
        window_name = "captured image"
        cv.ShowImage(window_name,small)
        cv.WaitKey()


        continue
        msg_index += 1
        transforms += [msg.laser_transform]
        print transforms[-1]
        clouds += [msg.normal_cloud_filtered]
        continue
        r = msg.camera_transform.transform.rotation
        rot = (r.x,r.y,r.z,r.w)
        t = msg.camera_transform.transform.translation
        trans = (t.x,t.y,t.z)
        frame = msg.camera_transform.header.frame_id
        print 'transform:',rot,trans,'in the',frame,'frame'
        continue
        for j in range(len(msg.images)):
            im = msg.images[j]
            resize = .4

            bridge = cv_bridge.CvBridge()
            #cam_model = image_geometry.cameramodels.PinholeCameraModel()
            #print "camera frame:",info.header.frame_id
            #cam_model.fromCameraInfo(info)
            
            im = bridge.imgmsg_to_cv(im, im.encoding)
            height = int(im.rows*resize)
            width = int(im.cols*resize)
            small = cv.CreateMat(height,width,cv.CV_8UC3)
            cv.Resize(im,small)

            fname = 'survey_%03d_%03d.png'%(msg_index,j)
            print fname
            cv.SaveImage(fname,im)
            window_name = "captured image"#"captured image %s"%(fname)
            cv.ShowImage(window_name,small)
            cv.WaitKey(10)
            #don't destroy the window, just keep outputting images to the same window
            #cv.DestroyWindow(window_name)
        


    exit()
    print "waiting for service 'cloud2_to_ply'..."
    rospy.wait_for_service("cloud2_to_ply")
    print "done."
    cloud_to_mesh = rospy.ServiceProxy("cloud2_to_ply",poisson_surface_reconstruction.srv.Cloud2ToPLY)
    print cloud_to_mesh(clouds[0],'test.ply')
    


"""
    print "waiting for service 'align_clouds2'..."
    rospy.wait_for_service("align_clouds2")
    print "done."
    align_clouds = rospy.ServiceProxy("align_clouds2",furniture_ops.srv.AlignClouds2)
    wT1 = geometry_msg_to_matrix(transforms[0])
    wT2 = geometry_msg_to_matrix(transforms[1])
    Tmsg = matrix_to_geometry_msg(numpy.dot(inverse_matrix(wT1),wT2))
    print align_clouds(clouds[0],clouds[1],Tmsg)
    
"""
