#!/usr/bin/env python
import cv, sys, os.path

directory = "."
if __name__=="__main__":
    if len(sys.argv)<2:
        print 'No directory specified.'
    else:
        directory = sys.argv[1]
print 'Reading images from '+directory

class mouse_handler:
    first_corner = None
    second_corner = None
    drawing = False
    def __init__(self,image,window_name):
        self.image = image
        self.window_name = window_name
        cv.SetMouseCallback(self.window_name,self.mouse_callback,image)
    def mouse_callback(self,event,u,v,flags,params):
        if event==cv.CV_EVENT_LBUTTONDOWN:
            self.first_corner = (u,v)
            self.drawing = True
        elif event==cv.CV_EVENT_LBUTTONUP:
            self.second_corner = (u,v)
            self.drawing = False
        elif event==cv.CV_EVENT_MOUSEMOVE and self.drawing:
            image2 = cv.CreateMat(self.image.height,self.image.width,cv.CV_8UC3)
            cv.Copy(self.image,image2)
            (x,y) = self.first_corner
            cv.PolyLine(image2,[[(x,y),(x,v),(u,v),(u,y)]],1,(255,255,255))
            cv.ShowImage(self.window_name,image2)
    def is_box_drawn(self):
        return self.first_corner is not None and self.drawing!=True

file_number = 0
last_file_number = 0

while True:
    file_number = file_number + 1
    image_name = "%s/%04d.png" % (directory,file_number)
    bbox_name = "%s/%04d.bbox" % (directory,file_number)
    if os.path.isfile(bbox_name):
        print 'Bounding box file '+bbox_name+' already exists, skipping.'
        continue
    if os.path.isfile(image_name):
        last_file_number = file_number
        image = cv.LoadImage(image_name)
        cv.ShowImage('image to crop',image)
        mh = mouse_handler(image,'image to crop')
        while not mh.is_box_drawn():
            key = cv.WaitKey(0)
        min_x = min(mh.first_corner[0],mh.second_corner[0])
        min_y = min(mh.first_corner[1],mh.second_corner[1])
        max_x = max(mh.first_corner[0],mh.second_corner[0])
        max_y = max(mh.first_corner[1],mh.second_corner[1])
        with open(bbox_name,'w') as f:
            f.write("#<min x> <max x> <min y> <max y>\n")
            f.write("%d %d %d %d\n" % (min_x, max_x, min_y, max_y))
        print 'Labeled bounding box in file '+bbox_name+'.'
    if file_number-last_file_number>1000:
        break
