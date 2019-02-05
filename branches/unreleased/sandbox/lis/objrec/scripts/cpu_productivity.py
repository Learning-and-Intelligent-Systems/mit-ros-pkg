#!/usr/bin/env python

with open('/var/tmp/objrec/hostfile') as f:
    lines = f.readlines()
hosts = [l.strip()[:-8].strip() for l in lines if l.strip().endswith('slots=24')]

with open('/tmp/CPU_productivity.txt') as f:
    lines = f.readlines()

class productivity:
    def __init__(this,n_hypotheses,hypotheses_per_sec):
        this.n_hypotheses = n_hypotheses
        this.hypotheses_per_sec = hypotheses_per_sec

cpu_productivities = [productivity(float(l.split()[0]),float(l.split()[1])) for l in lines]


instance_productivities = []
for j in range(len(hosts)):
    mean_n_hypotheses = sum([p.n_hypotheses for p in cpu_productivities[j*24:(j+1)*24]])/24
    mean_hypotheses_per_sec = sum([p.hypotheses_per_sec for p in cpu_productivities[j*24:(j+1)*24]])/24
    p = productivity(mean_n_hypotheses,mean_hypotheses_per_sec)
    p.hostname = hosts[j]
    instance_productivities.append(p)

instance_productivities.sort(key=lambda x: x.n_hypotheses)

for p in instance_productivities:
    print '{:8.2f} {:8.2f} {}'.format(p.n_hypotheses,p.hypotheses_per_sec,p.hostname)
        
