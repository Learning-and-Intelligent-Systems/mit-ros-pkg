#!/usr/bin/env python
import pc, cv, sys, os.path

def new_file_names(directory,file_number):
    image_name = "%s/%04d.png" % (directory,file_number)
    depth_name = "%s/%04d.yml" % (directory,file_number)
    while os.path.isfile(image_name) or os.path.isfile(depth_name):
        file_number += 1
        image_name = "%s/%04d.png" % (directory,file_number)
        depth_name = "%s/%04d.yml" % (directory,file_number)
    return (image_name,depth_name,file_number)

(space_key, enter_key1, enter_key2) = (1048608,1048586,1113997)
(q_key, esc_key) = (1048689,1048603)

directory = '.'
if __name__=="__main__":
    if len(sys.argv)<2:
        print 'No directory specified.'
    else:
        directory = sys.argv[1]
print 'Writing images to '+directory
    
print "With focus on the image window, press space or enter to capture an image, or 'q' or esc to quit."
file_number = 1
for cloud in pc.cloud_source():
    image = cv.CreateMat(cloud.height,cloud.width,cv.CV_8UC3)
    for (u,v,x,y,z,b,g,r) in pc.points(cloud):
        image[v,u] = (b,g,r)
    cv.ShowImage('live kinect',image)
    key = cv.WaitKey(100)
    if key in (space_key, enter_key1, enter_key2):
        (image_name,depth_name,file_number) = new_file_names(directory,file_number)
        depth = cv.CreateMat(cloud.height,cloud.width,cv.CV_32FC3)
        for (u,v,x,y,z,b,g,r) in pc.points(cloud):
            depth[v,u] = (x,y,z)
        cv.SaveImage(image_name,image)
        cv.Save(depth_name,depth)
        print 'Saved '+image_name+' and '+depth_name+'.'
    elif key in (q_key, esc_key):
        break;
    elif key!=-1:
        print 'Unrecognized key pressed:',key
