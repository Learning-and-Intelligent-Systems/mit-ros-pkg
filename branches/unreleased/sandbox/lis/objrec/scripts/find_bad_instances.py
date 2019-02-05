#!/usr/bin/env python
import subprocess
import time
import sys
import math

if __name__=="__main__" and len(sys.argv) > 1:
    timeout = float(sys.argv[1])
else:
    timeout = 60

with open('../hostfile') as f:
    lines = f.readlines()
instances = [l.strip()[:-8].strip() for l in lines if l.strip().endswith('slots=24')]

print str(len(instances))+' instances'
procs = []
start_time = time.time()
for h in instances:
    procs.append(subprocess.Popen('ssh '+h+' echo Jesus is LORD',shell=True,stdout=subprocess.PIPE))

def time_string(t):
    minutes = math.floor(t/60.0)
    seconds = t%60
    return '%02d:%02d' % (minutes,seconds)

n_completed = 0
while n_completed<len(instances) and time.time()-start_time<timeout:
    time.sleep(1)
    elapsed_time = time.time() - start_time
    n_completed = len([1 for p in procs if p.poll() is not None])
    print "\rWaiting for "+str(len(instances)-n_completed)+" of "+str(len(instances))+" instances. "+time_string(elapsed_time)+" of "+time_string(timeout),
    sys.stdout.flush()
print ''

if n_completed == len(instances):
    print 'All instances completed.'
else:
    print 'These hosts did not complete in '+time_string(timeout)+':'
    for j in range(len(instances)):
        if procs[j].poll() is None:
            print instances[j]
            procs[j].kill()
