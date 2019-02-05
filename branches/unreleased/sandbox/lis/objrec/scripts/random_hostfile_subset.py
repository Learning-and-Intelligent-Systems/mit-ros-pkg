#!/usr/bin/env python
import random
import os
import time

with open('/var/tmp/objrec/hostfile') as f:
    hosts = f.read().splitlines()

first = hosts[0]
rest = hosts[1:]

with open('/tmp/times.txt','w') as f:
    f.write('')

for n in range(6,0,-1):
  with open('/tmp/times.txt','a') as f:
      f.write(str(n)+'\t')
  for trial in range(2):
    random.shuffle(rest)

    with open('/tmp/hostfile_subset','w') as f:
        f.write(first+'\n')
        for j in range(n):
            f.write(rest[j]+'\n')

    cmd = 'mpirun -hostfile /tmp/hostfile_subset /var/tmp/objrec/bin/detect -object_file /tmp/cascade-untextured.off.txt -image_file /var/tmp/objrec/data/cascade/0061.png -output_file /var/tmp/experiment/4/pos_detections/cascade-untextured.off_image__var_tmp_objrec_data_cascade_0061.png -verbose 1 -CPU_productivity_log_file /tmp/CPU_productivity.txt -n_best_detections 1 -bounding_box_overlap 0.5'
    print 'n='+str(n)+', trial '+str(trial+1)
    start = time.time()
    os.system(cmd)
    with open('/tmp/times.txt','a') as f:
      f.write(str(time.time()-start)+'\t')
  with open('/tmp/times.txt','a') as f:
      f.write('\n')
