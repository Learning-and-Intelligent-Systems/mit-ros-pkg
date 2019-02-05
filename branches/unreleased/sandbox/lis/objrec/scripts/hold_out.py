#!/usr/bin/env python
import learn_object
import cloud
import default_parameters

object_list = [
      ('downy-untextured.off',1,'downy',{'camera_height_tolerance':.005}),
      ('cascade-untextured.off',1,'cascade',{'camera_height_tolerance':.005}),
      ('frenchs-untextured.off',2,'frenchs',{'camera_height_tolerance':.005}),
      ('jiffy-untextured.off',2,'jiffy',{'camera_height_tolerance':.005}),
      ('cereal.off',2,'cereal',{'n_depth_features':50}),
      ('printed-tiki-glass.off',float('inf'),'printed-tiki',{'camera_height_tolerance':.005}),
      ('tiki-glass.off',float('inf'),'tiki',{'angle_bin_width':30}),
      ('campbells-untextured.off',float('inf'),'campbells',{'camera_height_tolerance':.005}),
      ('domino-untextured.off',float('inf'),'sugar-jar',{'camera_height_tolerance':.005}),
      ('coffee-jar.off',float('inf'),'coffee-jar',{'edge_receptive_field_radius':10}),
      ('quaker-untextured.off',float('inf'),'quaker',{'camera_height_tolerance':.005}),
      ('sardine16.off',float('inf'),'sardine',{'edge_receptive_field_radius':10}),
      ('spool.off',float('inf'),'spool',{'camera_height_tolerance':.005}),
##      ('clear-glass.off',float('inf'),'clear_glass'),
##      ('red-glass.off',float('inf'),'red_glass'),
##      ('vase.off',float('inf'),'vase'),
      ]

hostfile = '/var/tmp/objrec/hostfile-hold-out'
hosts = cloud.connect(hostfile=hostfile)
print str(len(hosts))+' hosts'
print "cleaning up any old jobs that are still running."
learn_object.clean_old_jobs(hosts)
data_dir = '/var/tmp/data/hold-out/'
for (f,s,n,p) in object_list:
    objects = [learn_object.object('/var/tmp/objrec/meshes/'+f,s,
                                   positive_image_dir=data_dir+n,
                                   background_image_dir=data_dir+'background',
                                   object_output_dir = '/tmp/hold-out',
                                   hostfile=hostfile)]
    params_list = [dict(default_parameters.params.copy().items()+p.items())]
    learn_object.train_and_test_list(objects,hosts,params_list,'/var/tmp/hold-out/'+n)

