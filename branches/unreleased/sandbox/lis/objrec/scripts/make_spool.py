#!/usr/bin/env python
import math
def circle_points(radius, z):
    for n in range(ngon):
        x = radius*math.cos(n*2.0*math.pi/float(ngon))
        y = radius*math.sin(n*2.0*math.pi/float(ngon))
        print(str(x)+' '+str(y)+' '+str(z))
def tri((c0,p0),(c1,p1),(c2,p2)):
    print '3 '+str(c0*ngon+p0%ngon)+' '+str(c1*ngon+p1%ngon)+' '+str(c2*ngon+p2%ngon)
    print '3 '+str(c0*ngon+p0%ngon)+' '+str(c2*ngon+p2%ngon)+' '+str(c1*ngon+p1%ngon)

ngon = 16
print('OFF')
print(str(ngon*6)+' '+str(ngon*6*2)+' 0')
circle_points(12.7/2,-5.7/2) #0
circle_points(12.7/2,5.7/2) #1
circle_points(4.8/2,-5.7/2) #2
circle_points(4.8/2,5.7/2) #3
circle_points(2.0/2,-5.7/2) #4
circle_points(2.0/2,5.7/2) #5

for j in range(ngon):
    tri((0,j),(4,j),(4,j+1))
    tri((0,j),(0,j+1),(4,j+1))
    tri((1,j),(5,j),(5,j+1))
    tri((1,j),(1,j+1),(5,j+1))
    tri((2,j),(3,j),(3,j+1))
    tri((2,j),(2,j+1),(3,j+1))
