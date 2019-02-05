#!/usr/bin/env python
import evaldet
import sys

object_list = [
      ('downy-untextured.off',1,'downy'),
      ('cascade-untextured.off',1,'cascade'),
      ('frenchs-untextured.off',2,'frenchs'),
      ('jiffy-untextured.off',2,'jiffy'),
      ('cereal.off',2,'cereal'),
      ('printed-tiki-glass.off',float('inf'),'printed-tiki'),
      ('tiki-glass.off',float('inf'),'tiki'),
      ('campbells-untextured.off',float('inf'),'campbells'),
      ('domino-untextured.off',float('inf'),'domino'),
      ('coffee-jar.off',float('inf'),'coffee-jar'),
      ('quaker-untextured.off',float('inf'),'quaker'),
      ('sardine16.off',float('inf'),'sardine'),
      ('spool.off',float('inf'),'spool'),
##      ('clear-glass.off',float('inf'),'clear_glass'),
##      ('red-glass.off',float('inf'),'red_glass'),
##      ('vase.off',float('inf'),'vase'),
      ]


def mean(l):
    if len(l)==0:
        return float('NaN')
    return float(sum(l))/len(l)
def var(l):
    m = mean(l)
    return 1.0/(len(l)+1)*sum([(x-m)**2 for x in l])

for (m,s,obj) in object_list:
    sys.stdout.write(obj+'_m,'+obj+'_v,')
    sys.stdout.flush()
print ''

for j in range(48):
    for (m,s,obj) in object_list:
        detections = evaldet.detections(obj,'/var/tmp/experiment/'+str(j)+'/all_detections')
        average_precisions = evaldet.leave_one_out_average_precisions(detections)
        m = mean(average_precisions)
        v = var(average_precisions)
        sys.stdout.write(str(m)+','+str(v)+',')
        sys.stdout.flush()
    print ''
