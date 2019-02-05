#!/usr/bin/env python
import pc
import cv

#for opencv user interface functions, see:
#http://opencv.willowgarage.com/documentation/python/highgui_user_interface.html

window_name = 'test'
cv.NamedWindow(window_name)
def mouse_handler(event,u,v,flags,cloud):
    if event==cv.CV_EVENT_LBUTTONUP:
        #cloud[u,v] gets the position and color information for the pixel u,v
        (x,y,z,b,g,r) = cloud[u,v]
        print 'That point is at (x,y,z)='+str((x,y,z))

for cloud in pc.cloud_source():
    cv.SetMouseCallback(window_name,mouse_handler,cloud)
    image = pc.cloud_to_image(cloud)
    cv.ShowImage(window_name,image)
    if cv.WaitKey(100)!=-1:
        break
