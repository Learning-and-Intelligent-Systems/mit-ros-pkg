#!/usr/bin/env python

center_x = 0.65
center_y = -0.188
z        = 0.6+0.05/2
pitch    = 0.05

for i in range(8):
    for j in range(8):
        print "[%5.3f %5.3f %5.3f]" % (center_x + (i-3.5)*pitch,
                                       center_y + (j-3.5)*pitch,
                                       z)

        
