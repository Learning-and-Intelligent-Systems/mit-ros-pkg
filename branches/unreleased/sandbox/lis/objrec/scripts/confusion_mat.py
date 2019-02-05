#!/usr/bin/env python
import learn_object
import cloud
import os

object_list = [
      ('downy-untextured.off',1,'downy'),
      ('cascade-untextured.off',1,'cascade'),
      ('frenchs-untextured.off',2,'frenchs'),
      ('jiffy-untextured.off',2,'jiffy'),
      ('cereal.off',2,'cereal'),
      ('printed-tiki-glass.off',float('inf'),'printed-tiki'),
      ('tiki-glass.off',float('inf'),'tiki'),
      ('campbells-untextured.off',float('inf'),'campbells'),
      ('domino-untextured.off',float('inf'),'sugar-jar'),
      ('coffee-jar.off',float('inf'),'coffee-jar'),
      ('quaker-untextured.off',float('inf'),'quaker'),
      ('sardine16.off',float('inf'),'sardine'),
      ('spool.off',float('inf'),'spool'),
##      ('clear-glass.off',float('inf'),'clear_glass'),
##      ('red-glass.off',float('inf'),'red_glass'),
##      ('vase.off',float('inf'),'vase'),
      ]

data_dir = '/var/tmp/data/test/'
hostfile = '/var/tmp/objrec/hostfile-test2'#'/var/tmp/objrec/hostfile-hold-out'
object_output_dir = '/tmp/hold-out'
full_image = True
experiment_dir = '/var/tmp/confusion-mat-test-full-image'
jobs = []
for (mesh,s,_o) in object_list:
    ls_objs = os.listdir(data_dir)
    for obj_dir in ls_objs:
        if obj_dir=='background':
            continue
        if not os.path.isdir(data_dir+obj_dir):
            continue
        obj = learn_object.object('/var/tmp/objrec/meshes/'+mesh,s,
                                  positive_image_dir=data_dir+obj_dir,
                                  object_output_dir = object_output_dir,
                                  hostfile=hostfile,
                                  minoverlap=0.00001)
        obj.set_experiment_dir(experiment_dir)
        ls_images = os.listdir(data_dir+obj_dir)
        for img in ls_images:
            if img[-3:]=='png' and os.path.exists(data_dir+obj_dir+'/'+img[:-4]+'.bbox'):
                image_file = data_dir+obj_dir+'/'+img
                cmd = obj.testing_image_job(image_file)
                if cmd is not None: # if it is not already completed
                    if full_image:
                        cmd += ' -ignore_bounding_box 1'
                    jobs.append(cmd)
hosts = cloud.connect(hostfile=hostfile)

for j in range(len(jobs)):
    print 'detection '+str(j)+' of '+str(len(jobs))
    learn_object.run_command(jobs[j])
