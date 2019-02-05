#!/usr/bin/env python
import os

class pose:
    def __init__(this,strings):
        (this.xp,this.yp,this.depth,this.rx,this.ry,this.rz) = [float(x) for x in strings]
    def xyz(this):
        f = 525
        x = this.depth*this.xp/(f*f+this.xp**2)**.5
        y = this.depth*this.yp/(f*f+this.yp**2)**.5
        z = (this.depth**2-x*x)**.5
        return (x,y,z)
    def dist(p1,p2):
        (x1,y1,z1) = p1.xyz()
        (x2,y2,z2) = p2.xyz()
        return ((x1-x2)**2+(y1-y2)**2+(z1-z2)**2)**.5


def deviation(obj,im):
    detection_dir = '/var/tmp/hold-out/'+obj+'/0/pos_detections/'
    ls = os.listdir(detection_dir)
    for f in ls:
        if f[-8:-4]==im:
            detection_file = detection_dir+f
            break
    with open(detection_file) as f:
        detection_line = f.read()

    label_file = '/var/tmp/data/hold-out/'+obj+'/'+im+'.pose'
    with open(label_file) as f:
        label_line = f.read()

    p1 = pose(detection_line.split()[5:11])
    p2 = pose(label_line.split()[:6])
    print str(p1.dist(p2))+','+str(p1.rx-p2.rx)+','+str(p1.ry-p2.ry)+','+str(p1.rz-p2.rz)+','+obj+','+im

print 'dist,drx,dry,drz,obj,im'
dir = '/var/tmp/data/hold-out/'
ls = os.listdir(dir)
for obj in ls:
    obj_ls = os.listdir(dir+obj+'/')
    for f in obj_ls:
        if f[-5:]=='.pose':
            deviation(obj,f[-9:-5])

