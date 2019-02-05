#!/usr/bin/env python
import numpy
import scipy
import scipy.stats

file_name = '/tmp/results.txt'
independent = range(12)
dependent = range(12,15)
n_samples = 10

def mean_confidence_interval(data,confidence=0.95):
    a = 1.0*numpy.array(data)
    n = len(a)
    m, se = numpy.mean(a), scipy.stats.sem(a)
    h = se * scipy.stats.t._ppf((1.0+confidence)/2., n-1)
    return m-h,m+h,m

with open(file_name) as f:
    lines = f.readlines()

headings = lines[0].split(',')
experiments = []
for line in lines[1:]:
    experiments.append(line.split(','))

first_output_line = ','.join([headings[j] for j in independent])
for j in dependent:
    first_output_line += ','+headings[j]+'_lo'
    first_output_line += ','+headings[j]+'_hi'
    first_output_line += ','+headings[j]

output_lines = [first_output_line]
current_experiment = 0
while current_experiment+n_samples<=len(experiments):
    current_output_line = ','.join([experiments[current_experiment][j]\
                                    for j in independent])
    
    for v in dependent:
        data = []
        for j in range(n_samples):
            data.append(float(experiments[j+current_experiment][v]))
        (lo,hi,m) = mean_confidence_interval(data)
        current_output_line += ','+str(lo)
        current_output_line += ','+str(hi)
        current_output_line += ','+str(m)
        
    output_lines.append(current_output_line)
    current_experiment += n_samples

for line in output_lines:
    print line
