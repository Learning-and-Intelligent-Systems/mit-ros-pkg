#!/usr/bin/env python
import evaldet
import os.path

objects = ['tiki-glass','sugar-jar','sardine16','coffee-jar',
                       'printed-tiki-glass']#,'jiffy','cascade','downy-bottle']
#objects = ['jiffy']
objrec_dir = '~/ros/objrec/experiment/52/all_detections'
l = objects[0]
for obj in objects[1:]:
    l = l + ',' + obj
print l
args = []
for j in [20,40]:
    dpm_dir = '~/ros/objrec/voc-release5/mixture-detections/'+str(j)
    l = ''
##    with open(os.path.expanduser(dpm_dir)+'/args.txt') as f:
##        args.append(f.read())
    first = True
    print 'angle_bin_width: '+str(j)
    for obj in objects:
        ap_dpm    = evaldet.average_precision(evaldet.detections(obj+'_',dpm_dir))
        if first:
            first = False
            l = l + str(ap_dpm)
        else:
            l = l + ',' + str(ap_dpm)
        print obj+','+str(ap_dpm)
        #ap_objrec = evaldet.average_precision(evaldet.detections(obj,objrec_dir))
        #print obj+','+str(ap_objrec)+','+str(ap_dpm)
    print l


for a in args:
    print a
