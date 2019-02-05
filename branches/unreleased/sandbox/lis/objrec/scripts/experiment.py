#!/usr/bin/env python
import learn_object
import cloud
import default_parameters
import traceback

params_list = [] #a list of dictionaries of parameters

params = default_parameters.params.copy()
def vary_parameter(name,vals):
     for x in vals:
          p = params.copy()
          p[name] = x
          params_list.append(p)

vary_parameter('n_training_images',[50,100,200,1000])
vary_parameter('depth_receptive_field_radius',[0.002,0.004,0.006,0.008])

vary_parameter('camera_height_tolerance',[0.005,0.010,0.015,0.020])
vary_parameter('camera_pitch_angle_tolerance',[10,15,20,25])

vary_parameter('n_edge_features',[0,50,100,150])
vary_parameter('n_depth_features',[0,50,100,150])


for x in [50,100,150,200]:
     p = params.copy()
     p['n_depth_features'] = x
     p['n_edge_features'] = x
     params_list.append(p)

vary_parameter('angle_bin_width',[10,20,30,40])
vary_parameter('edge_receptive_field_radius',[4,6,8,10])
vary_parameter('max_edge_position_variance',[200,250,300,350])
for x in [.2,.3,.4,.5]:
     p = params.copy()
     p['edge_low_threshold'] = x
     p['edge_high_threshold'] = x
     params_list.append(p)

#vary_parameter('depth_receptive_field_radius',[0.01,0.02,0.03,0.04])
vary_parameter('max_depth_feature_variance',[0.01,0.02,0.03,0.04])


"""
for angle_bin_width in [20]:#40,37.5,35,32.5,30,27.5,25,22.5,20,17.5,15,12.5,10,7.5,5]:
for max_edge_position_variance in [175,300,325]:
 for edge_receptive_field_radius in [1,3,5,9]:
  for edge_bin_overlap in [0.85,0.9,0.95]:
    for ambient_light in [0.25,0.5,1.0]:
     for edge_high_threshold in [.3,.35,.4,.45,.5, .55, .6, .65]:#[1600,1800,2000,2200,2400,2600,2800,3000,800,1000,1200,1400,50,100,200,400,600]:
      for n_edge_features in [100,200,300,400]:
         params = {
             'n_texton_features':0,
             'n_depth_features':100
             }
         params['n_edge_features'] = n_edge_features
         params['angle_bin_width'] = angle_bin_width
         params['random_seed'] = random_seed
         params['max_edge_position_variance'] = max_edge_position_variance
         params['edge_receptive_field_radius'] = edge_receptive_field_radius
         params['edge_bin_overlap'] = edge_bin_overlap
         params['ambient_light'] = ambient_light
         params['edge_high_threshold'] = edge_high_threshold
         params['edge_low_threshold'] = edge_high_threshold
         params['high_threshold_log_probability_shift'] = high_threshold_log_probability_shift
         params_list.append(params)
"""

"""
texture_params_list = [] #a list of dictionaries of parameters
for textons_file in ['/var/tmp/objrec/textons32.txt',
                     '/var/tmp/objrec/textons.txt',
                     '/var/tmp/objrec/textons32-steerable.txt']:
 for texton_receptive_field_radius in [1,2,3,4,5]:
  for max_texton_position_variance in [200,225,250,275,300]:
   for ambient_light in [0.25,0.5,1.0]:
         params = {
             'angle_bin_width':40,
             'n_edge_features':0,
             'n_texton_features':100,
             'n_depth_features':0
             }
         params['textons_file'] = textons_file
         params['texton_receptive_field_radius'] = texton_receptive_field_radius
         params['max_texton_position_variance'] = max_texton_position_variance
         params['ambient_light'] = ambient_light
         texture_params_list.append(params)
print str(len(texture_params_list))+' texture parameter settings'
"""

print str(len(params_list))+' parameter settings'

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
objects = []
for (n,s,i) in object_list:
     objects.append(learn_object.object('/var/tmp/objrec/meshes/'+n,
                                        s,
                                        object_output_dir = '/tmp/experiment',
                                        positive_image_dir=data_dir+i,
                                        background_image_dir=data_dir+'background'))

print [o.name for o in objects]


def experiment():
    hosts = cloud.connect()
    print str(len(hosts))+' hosts'
    print "cleaning up any old jobs that are still running."
    learn_object.clean_old_jobs(hosts)
    experiment = '/var/tmp/experiment'
    learn_object.train_and_test_list(objects,hosts,params_list,experiment)


n_tries = 0
success = False
while not success:
      try:
            n_tries = n_tries + 1
            experiment()
            success = True
      except KeyboardInterrupt:
            raise
      except:
            print 'failed to run experiment after '+str(n_tries)+' tries'
            traceback.print_exc()


#symmetric_obj_names = ['tiki-glass.off','domino.off','sardine16.off',
#                       'coffee-jar.off','printed-tiki-glass.off',
#                       'campbells-untextured.off','domino-untextured.off',
#                       'quaker-untextured.off',
#                       'campbells.obj','domino.obj','quaker.obj']
#symmetric_obj_names = []
#asymmetric_objects = [('jiffy.off',2),('cascade.off',1),
#                      ('downy-bottle.off',1),
#                      ('jiffy-untextured.off',2),('frenchs-untextured.off',2),
#                      ('cascade-untextured.off',1),('downy-untextured.off',1),
#                      ('jiffy.obj',1),('frenchs.obj',1),
#                      ('cascade.obj',1),('downy.obj',1)]
#asymmetric_objects_list = [[('downy-untextured-wbg.off',2)]]#,[('jiffy-untextured.off',2)],[('cascade-untextured.off',1)],[('frenchs-untextured.off',2)]]

#for asymmetric_objects in asymmetric_objects_list:
#      objects = [learn_object.object(n) for (n) in symmetric_obj_names]+\
#          [learn_object.object(n,s) for (n,s) in asymmetric_objects]
#objects = [learn_object.object('tiki-glass.off')]#############################
#      print [o.name for o in objects]
#
#      edges_experiment = '/mnt/experiment/'+asymmetric_objects[0][0]
#      learn_object.train_and_test_list(objects,hosts,edge_params_list,edges_experiment)

#texture_experiment = '/mnt/experiment/texture'
#learn_object.train_and_test_list(objects,hosts,texture_params_list,texture_experiment)
#
#edges_texture_experiment = '/mnt/experiment/edges_texture'
#learn_object.combine_experiments(objects,hosts,edges_experiment,texture_experiment,50,edges_texture_experiment)


#edges_edges_experiment = '/mnt/experiment/edges_edges'
#learn_object.combine_experiments(objects,hosts,edges_experiment,edges_experiment,10,edges_edges_experiment)
