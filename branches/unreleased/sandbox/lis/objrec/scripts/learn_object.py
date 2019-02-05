import distribute
import evaldet

import os
import os.path
import sys
import math
import time
import subprocess
import re
import shutil
import traceback
import gc
#import processify

class object:
    X_server_n = 99
    def __init__(this,mesh_file,rz_symmetry_order=float("inf"),
                 positive_image_dir=None,
                 background_image_dir='/var/tmp/objrec/data/background',
                 objrec_dir='/var/tmp/objrec',
                 object_output_dir='/tmp',
                 hostfile='/var/tmp/objrec/hostfile-experiment',
                 minoverlap = 0.5):
        this.rz_symmetry_order = rz_symmetry_order
        this.name = os.path.basename(mesh_file)
        this.objrec_dir = objrec_dir
        this.object_output_dir = object_output_dir
        if not os.path.exists(object_output_dir):
            os.makedirs(object_output_dir)
        this.mesh_file = mesh_file
        this.hostfile = hostfile
        if positive_image_dir is None:
            this.positive_image_dir = '/var/tmp/objrec/data/'+this.name
        else:
            this.positive_image_dir = positive_image_dir
        this.background_image_dir = background_image_dir
        this.minoverlap = minoverlap
    def set_experiment_dir(this,experiment_dir):
        this.experiment_dir = experiment_dir
        this.object_file = this.object_output_dir+'/'+this.name+'.txt'
        this.pos_detections_dir = this.experiment_dir+'/pos_detections'
        this.all_detections_dir = this.experiment_dir+'/all_detections'
#        this.views_dir = this.experiment_dir+'/views'
    def testing_image_job(this,image_file,lower_bound = float("-inf"), \
                    find_worst_positive=True,view_step=-1):

        if find_worst_positive:
            output_filename = this.pos_detections_dir
        else:
            output_filename = this.all_detections_dir
        
        output_filename += '/'+this.name+'_image_'+re.sub(r'[\/\~]','_',image_file)
        cmd = ''
        cmd += ' mpirun -hostfile '+this.hostfile
        cmd += ' '+this.objrec_dir+'/bin/detect'
        cmd += ' -object_file '+this.object_file
        cmd += ' -image_file '+image_file
        cmd += ' -output_file '+output_filename
        cmd += ' -verbose 1'
        #cmd += ' -ignore_edges_file 1'
        cmd += ' -CPU_productivity_log_file /tmp/CPU_productivity.txt'
        if not os.path.exists(image_file[:-4]+'.scene'):
            cmd += ' -n_chunks 10'
        #cmd += ' -visualize_search 1'
        if find_worst_positive:
            cmd += ' -n_best_detections 1'
            cmd += ' -bounding_box_overlap '+str(this.minoverlap)
        else:
            cmd += ' -ignore_bounding_box 1'
        if lower_bound!=float("-inf"):
            cmd += ' -lower_bound '+str(lower_bound)
        if os.path.exists(output_filename):
            #print output_filename+' exists, skipping'
            print cmd
            return None
        else:
            return cmd
    def testing_jobs(this,image_dir,lower_bound = float("-inf"), \
                find_worst_positive=True,view_step=-1):
        images = [f for f in os.listdir(os.path.expanduser(image_dir)) \
                  if f[-4:]=='.png']
        jobs = []
        for im in images:
            cmd = this.testing_image_job(image_dir+'/'+im,lower_bound,\
                                         find_worst_positive,view_step)
            if cmd is not None:
                jobs.append(cmd)
        return jobs
    def positive_testing_jobs(this,lower_bound):
        return this.testing_jobs(this.positive_image_dir,lower_bound,True)
    def all_testing_jobs(this,lower_bound,view_step=-1):
        neg_images = this.testing_jobs(this.background_image_dir,
                                       lower_bound,False,view_step)
        pos_images = this.testing_jobs(this.positive_image_dir,
                                       lower_bound,False,view_step)
        return neg_images+pos_images
        
def clean_old_jobs(hosts):
    jobs = []
    for h in hosts:
        j = distribute.job('killall -9 detect learn')
        j.start(h)
        jobs.append(j)
    while len([1 for j in jobs if not j.isdone(verbose=False)])>0:
        time.sleep(1)

def set_experiment_dir(objects,experiment_dir,delete=True):
    if delete:
        shutil.rmtree(os.path.expanduser(experiment_dir),True)
    for o in objects:
        o.set_experiment_dir(experiment_dir)
    try:
        os.mkdir(os.path.expanduser(objects[0].experiment_dir))
    except:
        pass
    try:
        os.mkdir(os.path.expanduser(objects[0].pos_detections_dir))
    except:
        pass
    try:
        os.mkdir(os.path.expanduser(objects[0].all_detections_dir))
    except:
        pass


def learn(objects,hosts,args=''):
    for o in objects:
        if os.path.exists(o.experiment_dir+'/'+o.name+'.txt'):
            print "object " + o.experiment_dir+'/'+o.name+'.txt' + " already exists, skipping."
        else:
            cmd = "mpirun"
            cmd += " -hostfile "+o.hostfile
            cmd += " -x DISPLAY=:99"
            cmd += " /var/tmp/objrec/bin/learn"
            cmd += " -mesh_file_name "+o.mesh_file
            cmd += " "+args+" "
            if o.rz_symmetry_order!=float("inf"):
                cmd += " -rz_symmetry_order "+\
                       str(o.rz_symmetry_order)
            cmd += " > " + o.object_file
            print cmd
            clean_old_jobs(hosts)
            start = time.time()
            os.system(cmd)
            elapsed = time.time()-start
            print 'learning took '+str(elapsed)+' seconds.'
            shutil.copy(o.object_file,o.experiment_dir+'/'+o.name+'.txt')
    objects_string = ''
    for o in objects:
        objects_string += ' ' + o.object_file
    distribute.copy(hosts,objects_string,o.object_output_dir)

def run_command(cmd):
    start = time.time()
    exit_code = -1
    while exit_code!=0:
        print "about to run:"
        print cmd
        exit_code = os.system(cmd)
        if exit_code != 0:
            print 'exit code: '+str(exit_code)
    elapsed = time.time()-start
    print 'took '+str(elapsed)+' seconds to run:'
    print cmd    

def positive_testing(objects,lower_bound=float('-inf'),args=''):
    for o in objects:
        jobs = o.positive_testing_jobs(lower_bound)
        for cmd in jobs:
            run_command(cmd+' '+args)

def lower_bound(obj):
    detections = evaldet.detections(obj.name,obj.pos_detections_dir)
    if len(detections)==0:
        print 'no positive detections for object '+obj.name
    confidences = [d.confidence for d in detections]
    worst_positive = min(confidences)
    best_positive = max(confidences)
    lb = worst_positive - (best_positive-worst_positive)*.2
    #print obj.name+': worst positive: '+str(worst_positive)+' best positive: '+str(best_positive)+' lower_bound: '+str(lb)
    return lb

#lower_bound = processify.processify(lower_bound)

def all_testing(objects,args=''):
    for o in objects:
        jobs = o.all_testing_jobs(lower_bound(o))
        for cmd in jobs:
            run_command(cmd+' '+args)

def results(objects,min_overlap=0.5):
    average_precisions = {}
    for obj in objects:
        detections = evaldet.detections(obj.name, obj.all_detections_dir,
                                        lambda f: 'background' not in f)
        ap = evaldet.average_precision(detections,min_overlap=min_overlap)
        average_precisions[obj.name] = ap
        print obj.name+\
              " average precision: "+str(ap)+\
              " margin: "+str(evaldet.margin(detections,min_overlap=min_overlap))
        detections = []
        gc.collect() #try to free memory
    return average_precisions

#results = processify.processify(results)

def separate_params(params):
    detection_params = {}
    learning_params = {}
    for p in params:
        if p=='camera_height_tolerance' or\
           p=='camera_pitch_angle_tolerance':
            detection_params[p] = params[p]
        else:
            learning_params[p] = params[p]
    return (detection_params, learning_params)

def train_and_test(objects,hosts,params,experiment_dir):
    set_experiment_dir(objects,experiment_dir,delete=False)
    (detection_params,learning_params) = separate_params(params)
    learning_args = ''.join([' -'+p+' '+str(learning_params[p]) for p in learning_params])
    detection_args = ''.join([' -'+p+' '+str(detection_params[p]) for p in detection_params])
    with open(os.path.expanduser(experiment_dir+'/args.txt'),'w') as f:
        f.write(learning_args+'\n')
        f.write(detection_args)
    print learning_args
    print detection_args
    print 'learning:'
    learn(objects,hosts,args=learning_args)
    print 'finding the worst positive example:'
    positive_testing(objects,args=detection_args)#,-800)
    print 'testing all examples:'
    all_testing(objects,detection_args)
    return results(objects)



#train_and_test = processify.processify(train_and_test)

def run_experiment(objects,
                   experiments_dir,current_experiment,experiments_list,
                   experiment_function,experiment_description,results_file):
    experiment_dir = experiments_dir+'/'+str(current_experiment)
    start_time = time.time()

    experiment = experiments_list[current_experiment]
    print 'experiment '+str(current_experiment)+' of '+str(len(experiments_list))
    average_precisions = experiment_function(objects,experiment,experiment_dir)    
    with open(results_file,'a') as f:
        f.write(experiment_description(experiment))
        for o in objects:
            f.write(str(average_precisions[o.name])+',')
        f.write(str(time.time()-start_time)+'\n')

#run_experiment = processify.processify(run_experiment)

def experiment_for_each(objects,hosts,experiments_list,experiments_dir,
                        experiment_function,headings_description,experiment_description):
    if not os.path.exists(os.path.expanduser(experiments_dir)):
        os.mkdir(os.path.expanduser(experiments_dir))
    results_file = os.path.expanduser(experiments_dir+'/results.txt')
    if not os.path.exists(results_file):
        with open(results_file,'w') as f: #write the header row
            f.write(headings_description(experiments_list[0]))
            for o in objects:
                f.write(o.name+',')
            f.write('time\n')

    with open(results_file) as f:
        n_experiments = len(f.readlines())-1
    if n_experiments==len(experiments_list):
        print 'already have all '+str(n_experiments)+' experiments in results file '+results_file
        return
    if n_experiments > 0:
        print 'already have results for experiments 0 through '+str(n_experiments-1)+' of '+str(len(experiments_list))+' in results file '+results_file+', skipping these experiments'
    else:
        print 'results file '+results_file+' has none of the '+str(len(experiments_list))+' experiments in it yet, starting with the first experiment'

#    for current_experiment in range(n_experiments,len(experiments_list)):
#        run_experiment(objects,hosts,
#                   experiments_dir,current_experiment,experiments_list,
#                   experiment_function,experiment_description,results_file)

    for current_experiment in range(n_experiments,len(experiments_list)):
        experiment_dir = experiments_dir+'/'+str(current_experiment)
        start_time = time.time()

        n_tries = 0
        success = False
        while not success:
            try:
                experiment = experiments_list[current_experiment]
                print 'experiment '+str(current_experiment)+' of '+str(len(experiments_list))
                n_tries = n_tries + 1
                average_precisions = experiment_function(objects,hosts,experiment,experiment_dir)
                success = True
            except KeyboardInterrupt:
                raise
            except:
                print 'failed to run experiment '+str(current_experiment)+\
                    ' after '+str(n_tries)+' tries'
                traceback.print_exc()
        
        with open(results_file,'a') as f:
            f.write(experiment_description(experiment))
            for o in objects:
                f.write(str(average_precisions[o.name])+',')
            f.write(str(time.time()-start_time)+'\n')

def train_and_test_list(objects,hosts,params_list,experiments_dir):
    def headings_description(params):
        s = ''
        for p in params:
            s += p + ','
        return s
    def experiment_description(params):
        s = ''
        for p in params:
            s += str(params[p])+','
        return s
    experiment_for_each(objects,hosts,params_list,experiments_dir,
                        train_and_test,headings_description,experiment_description)

def test_list(objects,hosts,params_list,experiments_dir,min_overlap=0.5):
    def headings_description(params):
        s = ''
        for p in params:
            s += p + ','
        return s
    def experiment_description(params):
        s = ''
        for p in params:
            s += str(params[p])+','
        return s
    def test(objects,hosts,params,experiment_dir):
        print 'experiment_dir='+experiment_dir
        set_experiment_dir(objects,experiment_dir,delete=False)
        return results(objects,min_overlap=min_overlap)
    experiment_for_each(objects,hosts,params_list,experiments_dir,
                        test,headings_description,experiment_description)


def load_results_file(filename):
    with open(filename) as f:
        lines = f.readlines()
    headings = lines[0].split(',')
    columns = {}
    for j in range(len(headings)):
        column = [l.split(',')[j] for l in lines[1:]]
        columns[headings[j]] = column
    return columns

class combination:
    def __init__(this,index1,index2,value1,value2):
        this.pair = (index1,index2)
        this.value1 = value1
        this.value2 = value2
    def combined_value(this):
        return this.value1+this.value2
    def file1(this):
        dir1 = os.path.expanduser(experiments_dir1+'/'+str(this.pair[0]))
        return dir1+'/'+obj.name+'.txt'
    def file2(this):
        dir2 = os.path.expanduser(experiments_dir2+'/'+str(this.pair[1]))
        return dir2+'/'+obj.name+'.txt'
    def __repr__(this):
        return 'combination'+str((this.pair[0],this.pair[1],this.value1,this.value2))
    

def combine_experiments(objects,experiments_dir1,experiments_dir2,
                        n_combinations,combined_dir):
    results1 = load_results_file(os.path.expanduser(experiments_dir1+'/results.txt'))
    results2 = load_results_file(os.path.expanduser(experiments_dir2+'/results.txt'))
    object_combinations = {}
    for obj in objects:
    #calculate sum of average precision values for all pairs from experiments_dir1,experiments_dir2
    #sort descending by sum of results1 and results2
        r1 = [float(x) for x in results1[obj.name]]
        r2 = [float(x) for x in results2[obj.name]]
        combinations = []
        for index1 in range(len(r1)):
            for index2 in range(len(r2)):
                combinations.append(combination(index1,index2,r1[index1],r2[index2]))
        combinations.sort(key=combination.combined_value,reverse=True)
        object_combinations[obj.name] = combinations[:n_combinations]
    combinations = [] #free up the memory
    all_combinations = []
    for j in range(n_combinations):
        combinations_j = {obj.name:object_combinations[obj.name][j] for obj in objects}
        all_combinations.append(combinations_j)

    def headings_description(combinations):
        s = 'experiments_dir1,experiments_dir2,'
        for obj in combinations:
            s += obj+'1,'+obj+'2,'
        return s
    def experiment_description(combinations):
        s = experiments_dir1+','+experiments_dir2+','
        for obj in combinations:
            c = combinations[obj]
            s += str(c.pair[0])+','+str(c.pair[1])+','
        return s
    def combine_experiment_function(objects,hosts,comb,experiment_dir):
        #combine_objects instead of learn(), followed by positive_testing and all_testing
        set_experiment_dir(objects,experiment_dir,delete=False)
        args = ''
        for obj in objects:
            c = comb[obj.name]
            args += '('+obj.name+' '+str(c.pair[0])+' '+str(c.pair[1])+')'
        with open(os.path.expanduser(experiment_dir+'/args.txt'),'w') as f:
            f.write(args)
        #print args
        for obj in objects:
            c = comb[obj.name]
            dir1 = os.path.expanduser(experiments_dir1+'/'+str(c.pair[0]))
            file1 = dir1+'/'+obj.name+'.txt'
            dir2 = os.path.expanduser(experiments_dir2+'/'+str(c.pair[1]))
            file2 = dir2+'/'+obj.name+'.txt'
            cmd = ''
            cmd += ' '+obj.objrec_dir+'/bin/combine_objects'
            cmd += ' '+file1
            cmd += ' '+file2
            cmd += ' '+obj.object_file
            try:
                err = subprocess.check_output(cmd,stderr=subprocess.STDOUT,shell=True)
            except subprocess.CalledProcessError, e:
                err = e.output
            except:
                import debug
                debug.keyboard()
            sys.stdout.write(err)
            sys.stdout.flush()
            #distribute.copy(hosts,obj.object_file,obj.object_file)
            shutil.copy(obj.object_file,obj.experiment_dir+'/'+obj.name+'.txt')
        print 'finding the worst positive example:'
        positive_testing(objects)#,-800)
        print 'testing all examples:'
        all_testing(objects)
        return results(objects)
    
    print all_combinations
    #experiment_function = processify.processify(combine_experiment_function)
    experiment_for_each(objects,hosts,all_combinations,combined_dir,
                        experiment_function,headings_description,experiment_description)
