#!/usr/bin/env python
import os
import os.path

dirpath = '/var/tmp/experiment/1/pos_detections/'
files = [f for f in os.listdir(dirpath) if os.path.isfile(dirpath+'/'+f)]
for f in files:
    img_path = f[f.find('__')+1:].replace('_','/')
    with open(dirpath+'/'+f) as detection:
        p = detection.read().split()[5:11]
    with open(img_path[:-4]+'.pose','w') as pose:
        pose.write(' '.join(p))
        
