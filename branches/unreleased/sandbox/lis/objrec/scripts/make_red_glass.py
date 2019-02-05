#!/usr/bin/env python
import math
def circle_points(radius, z):
    for n in range(ngon):
        x = radius*math.cos(n*2.0*math.pi/float(ngon))
        y = radius*math.sin(n*2.0*math.pi/float(ngon))
        print(str(x)+' '+str(y)+' '+str(z))

ngon = 16
print('OFF')
print(str(ngon*2)+' '+str(ngon*4+(ngon-2)*2)+' 0')
circle_points(6.7/2,-11.4/2) #0 to (ngon-1)
circle_points(9.5/2,11.4/2) #ngon to (2*ngon-1)
#in case we want a better mesh: cup material is 4.7 mm thick

#bottom surface
for j in range(2,ngon):
    print '3 0 '+str(j-1)+' '+str(j)
    print '3 0 '+str(j)+' '+str(j-1) #two-sided

#sides
for j in range(ngon):
    print '3 '+str(j)+' '+str(ngon+j)+' '+str(ngon+(j+1)%ngon)
    print '3 '+str(j)+' '+str(ngon+(j+1)%ngon)+' '+str(ngon+j)
    print '3 '+str(j)+' '+str((j+1)%ngon)+' '+str(ngon+(j+1)%ngon)
    print '3 '+str(j)+' '+str(ngon+(j+1)%ngon)+' '+str((j+1)%ngon)
