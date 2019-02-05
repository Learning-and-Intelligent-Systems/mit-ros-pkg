#!/usr/bin/env python
import pc
from pc import cloud_source, transform_matrix, ik, execute, points, cloud_to_image
import cv

#make a new window using OpenCV
window_name = 'test'
cv.NamedWindow(window_name)

#for each new image from the kinect, show it on the window
for cloud in cloud_source():
    image = cv.CreateMat(cloud.height,cloud.width,cv.CV_8UC3)
    for (u,v,x,y,z,b,g,r) in points(cloud):
        image[v,u] = (b,g,r)

    cv.ShowImage(window_name,image)

    if cv.WaitKey(100)!=-1:
        break
