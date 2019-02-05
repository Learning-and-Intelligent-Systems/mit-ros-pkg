#!/usr/bin/env python

independent = range(9)
dependent = range(9,30)
file_name = '/mnt/old_experiment/edges/original_results.txt'
#independent = range(11)
#dependent = range(11,32)
#file_name = '/mnt/experiment/edges/results.txt'


with open(file_name) as f:
    lines = f.readlines()

headings = lines[0].split(',')
experiments = []
for line in lines[1:]:
    experiments.append(line.split(','))

independent_values = {}
for v in independent:
    values = set([experiments[e][v] for e in range(len(experiments))])
    independent_values[v] = values
    print headings[v] + ' values: ' + str(values)

best_dependent = {}
for v in dependent:
    best_dependent[v]=max([float(experiments[e][v])
                           for e in range(len(experiments))])
    print headings[v] + ' best: '+str(best_dependent[v])

def valid_experiment(e,independent_values):
    for ivar in independent:
        if experiments[e][ivar] not in independent_values[ivar]:
            return False
    return True

def parameter_value_to_remove(independent_values):
    value_loss = []
    for ivar in independent:
        ivals = independent_values[ivar]
        if len(ivals)>1:
            for ival in ivals:
                worst_dvars = []
                worst_loss = float('-inf')
                remaining_ivals = dict(independent_values)
                remaining_ivals[ivar] = set(independent_values[ivar])
                remaining_ivals[ivar].remove(ival)

                for dvar in dependent:
                    best_without =\
                                 max([float(experiments[e][dvar])
                                      for e in range(len(experiments))
                                      if valid_experiment(e,remaining_ivals)])
                    best_with = \
                                 max([float(experiments[e][dvar])
                                      for e in range(len(experiments))
                                      if valid_experiment(e,independent_values)])
                    loss = best_with-best_without
                    triple = (headings[dvar],best_with,best_without)
                    if loss==worst_loss:
                        worst_dvars.append(triple)
                    elif loss>worst_loss:
                        worst_loss = loss
                        worst_dvars = [triple]
                value_loss.append((worst_loss,ivar,ival,worst_dvars))
    if len(value_loss)==0:
        return None
    s = sorted(value_loss,key=lambda v: v[0])
    return s[0]

remaining_ivals = independent_values
while True:
    v = parameter_value_to_remove(remaining_ivals)
    if v is None:
        break
    print 'removing '+headings[v[1]]+'='+str(v[2])+' decreses '+v[3][0][0]\
          +' by '+str(v[0])+' (from '+str(v[3][0][1])+' to '+\
          str(v[3][0][2])+')'
    if len(v[3])>1:
        print '*** and '+str(len(v[3])-1)+' more.***'
    remaining_ivals[v[1]] = set(remaining_ivals[v[1]])
    remaining_ivals[v[1]].remove(v[2])

print 'final remaining independent values:'
for ivar in independent:
    print headings[ivar]+':'+str(remaining_ivals[ivar])
