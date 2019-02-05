#!/usr/bin/env python
import roslib
roslib.load_manifest('kinect_test')
import rospy
import cv

#load image and depth
im = cv.LoadImage('../objrec/tmp/background/0001.png')
depth = cv.Load('../objrec/tmp/background/0001.yml')

#turn it to grayscale
gray = cv.CreateMat(im.rows, im.cols, cv.CV_8UC1)
cv.CvtColor(im,gray,cv.CV_BGR2GRAY)

#run an edge detector
edges = cv.CreateMat(image.rows,image.cols,cv.CV_8UC1)

threshold1 = 100
threshold2 = 50
cv.Canny(gray,edges,threshold1,threshold2)

cv.ShowImage('im',im)
cv.ShowImage('gray',gray)
cv.ShowImage('edges',edges)
cv.WaitKey(0)
