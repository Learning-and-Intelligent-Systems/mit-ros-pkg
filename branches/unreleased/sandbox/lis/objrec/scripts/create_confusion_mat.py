#!/usr/bin/env python
import os,sys
fixed_width = False
step = 1
log_priors_full_image = [
    483, # 0 downy
    475, # 1 cascade
    379, # 2 frenchs
    440, # 3 jiffy
      674, # 4 cereal
      529, # 5 printed-tiki
      510, # 6 tiki
      462, # 7 campbells
      578, # 8 sugar-jar
      648, # 9 coffee-jar
      517, # 10 quaker
      522, # 11 sardine
      598, # 12 spool
      ]

log_priors_cropped = [
      618, # 0 downy
      538, # 1 cascade
      379, # 2 frenchs
      440, # 3 jiffy
      718, # 4 cereal
      529, # 5 printed-tiki
      510, # 6 tiki
      462, # 7 campbells
      558, # 8 sugar-jar
      678, # 9 coffee-jar
      527, # 10 quaker
      535, # 11 sardine
      598, # 12 spool
    ]
dir = '/var/tmp/confusion-mat-hold-out/pos_detections/'
search_priors = False
cropped = True
if cropped:
    similar_pairs = [(6,5),(5,5),(5,6),(6,6),(9,8),(8,9),(9,9),(8,8)]
    log_priors = log_priors_cropped
else:
    similar_pairs = [(6,5),(5,5),(5,6),(6,6),(9,8),(8,9),(9,9),(8,8)]
    log_priors = log_priors_full_image

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

def predicted_class(detection):
    for j in range(len(object_list)):
        obj = object_list[j]
        if detection[:len(obj[0])]==obj[0]:
            return j

def actual_class(detection):
    for j in range(len(object_list)):
        obj = object_list[j]
        if detection[-(9+len(obj[2])):-9]==obj[2]:
            return j

def image_name(detection):
    return detection[-8:-4]

log_probabilities = []
for ac in range(len(object_list)):
    log_probabilities.append({})

ls = os.listdir(dir)
for detection in ls:
    ac = actual_class(detection)
    if ac is not None:
        n = image_name(detection)
        log_probabilities[ac][n] = [float('-inf')]*len(object_list)

for detection in ls:
    pc = predicted_class(detection)
    ac = actual_class(detection)
    n = image_name(detection)
    if ac is not None and pc is not None:
        with open(dir+detection) as f:
            d = f.read().split()
        if len(d)==0:
            print dir+detection+' seems empty'
            d = ['-inf']
        log_probability = float(d[0])
        log_probabilities[ac][n][pc] = log_probability


def confusion_mat(log_priors):
    C = []
    for ac in range(len(object_list)):
        C.append([0]*len(object_list))
        lp = log_probabilities[ac]
        pc_counts = [0.0]*len(object_list)
        for n in lp:
            best_log_probability = float('-inf')
            for pc in range(len(object_list)):
                if lp[n][pc]+log_priors[pc]>best_log_probability:
                    best_log_probability = lp[n][pc]+log_priors[pc]
                    predicted_class = pc
            pc_counts[predicted_class] += 1
        for pc in range(len(object_list)):
            C[ac][pc] = pc_counts[pc]/len(lp)
    return C

def print_confusion_mat(C,fixed_width=True):
    C = confusion_mat(log_priors)
    if fixed_width:
        sys.stdout.write(" "*10)
    for (m,s,c) in object_list:
        if fixed_width:
            sys.stdout.write(","+c[:min(4,len(c))])
        else:
            sys.stdout.write(","+c)
    print ''

    for ac in range(len(object_list)):
        if fixed_width:
            sys.stdout.write(object_list[ac][2].ljust(10)[:10])
        else:
            sys.stdout.write(object_list[ac][2])
        for pc in range(len(object_list)):
            sys.stdout.write(","+"{:.2f}".format(C[ac][pc]))
            if ac==pc:
                score = 1.0 - C[ac][pc]
            else:
                score = C[ac][pc]
        print ''
    
def find_worst(log_priors):
    C = confusion_mat(log_priors)
    worst_score = float('-inf')
    for ac in range(len(object_list)):
        for pc in range(len(object_list)):
            if ac==pc:
                score = 1.0 - C[ac][pc]
            else:
                score = C[ac][pc]
            if score>worst_score and (ac,pc) not in similar_pairs: #confusion between printed-tiki and tiki is ok
                worst_pair = (ac,pc)
                worst_score = score
    return (worst_pair, worst_score)

def change_log_priors(log_priors):
    ((ac,pc),worst_score) = find_worst(log_priors)

    ac_initial = ac
    pc_initial = pc
    if ac==pc:
        while ac==pc and ac==ac_initial:
            log_priors[ac] += step
            ((ac,pc),worst_score) = find_worst(log_priors)
        change_index = ac_initial
        change_to = log_priors[ac_initial]
        #print_confusion_mat(log_priors)
        #print "new log prior for "+object_list[ac][2]+": "+str(log_priors[ac])
    else:
        while ac==ac_initial and pc==pc_initial:
            log_priors[pc] -= step
            ((ac,pc),worst_score) = find_worst(log_priors)
        change_index = pc_initial
        change_to = log_priors[pc_initial]
        #print_confusion_mat(log_priors)
        #print "new log prior for "+object_list[pc][2]+": "+str(log_priors[pc])
    return (change_index, change_to)

if search_priors:
    last_change_index = -1
    change_index = -2
    while last_change_index!=change_index:
        last_change_index = change_index
        (change_index, change_to) = change_log_priors(list(log_priors))
        print 'change '+object_list[change_index][2]+' to '+str(change_to)
        log_priors[change_index] = change_to

print_confusion_mat(log_priors,fixed_width=fixed_width)
((ac,pc),worst_score) = find_worst(log_priors)

if ac==pc:
    print 'worst is on diagonal: '+object_list[ac][2]+' '+str(1.0-worst_score)
else:
    print 'worst is off diagonal: actual: '+object_list[ac][2]+' predicted: '+object_list[pc][2]+' '+str(worst_score)

for j in range(len(object_list)):
    print '      '+str(log_priors[j])+', # '+str(j)+' '+object_list[j][2]
