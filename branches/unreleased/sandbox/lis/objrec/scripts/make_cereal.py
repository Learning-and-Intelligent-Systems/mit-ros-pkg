#!/usr/bin/env python

h = 29.2 #cm
w = 19.3 #cm
d = 8.0 #cm

def rect(p0,p1,p2,p3):
    print('3 '+str(p0)+' '+str(p1)+' '+str(p2))
    print('3 '+str(p0)+' '+str(p2)+' '+str(p1))
    print('3 '+str(p0)+' '+str(p2)+' '+str(p3))
    print('3 '+str(p0)+' '+str(p3)+' '+str(p2))

print('OFF')
print("8 24 0")
print(str(-d/2.0)+' '+str(-w/2.0)+' '+str(-h/2.0)) #0
print(str(-d/2.0)+' '+str(-w/2.0)+' '+str(h/2.0))  #1
print(str(-d/2.0)+' '+str(w/2.0)+' '+str(-h/2.0))  #2
print(str(-d/2.0)+' '+str(w/2.0)+' '+str(h/2.0))   #3
print(str(d/2.0)+' '+str(-w/2.0)+' '+str(-h/2.0))  #4
print(str(d/2.0)+' '+str(-w/2.0)+' '+str(h/2.0))   #5
print(str(d/2.0)+' '+str(w/2.0)+' '+str(-h/2.0))   #6
print(str(d/2.0)+' '+str(w/2.0)+' '+str(h/2.0))    #7

rect(0,1,3,2) #-d
rect(4,5,7,6) #d
rect(0,1,5,4) #-w
rect(2,3,7,6) #w
rect(0,2,6,4) #-h
rect(1,3,7,5) #-h
