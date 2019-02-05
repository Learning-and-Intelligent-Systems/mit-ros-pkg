#!/usr/bin/env python
import roslib
#load in all the paths to the packages listed in 'manifest.xml'
roslib.load_manifest('simple_controller')

#basic ros python commands:
import rospy

#import message types:
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Polygon
from mapping_msgs.msg import PolygonalMap
from geometry_msgs.msg import Twist
import tf

import copy
from math import  *
boxwidth=.5
boxfront=.3
boxback=1.5

def transformPt(target_frame,pt,header):
    ps=PointStamped()
    ps.header=header 
    ps.point=pt
    return listener.transformPoint(target_frame,ps).point

def transformPoly(target_frame,polyin,header):
    polyout=Polygon()
    for p in polyin.points:
       polyout.points.append(transformPt(target_frame,p,header))
    return polyout
       

def getPersonPts(cloud):
    #find out which points are in front of the robot:
#    print cloud.header
    robotframe="/base_footprint"
    globalframe=cloud.header.frame_id

    listener.waitForTransform(robotframe, globalframe, cloud.header.stamp, rospy.Duration(.20))
    tcloud=listener.transformPointCloud(robotframe,cloud)
    legs=[]
    for pt in tcloud.points:
        if pt.x < boxback and pt.x > boxfront and abs(pt.y) < boxwidth:
            legs.append(pt)
    if len(legs)==1:
#        make a leg next to the current leg
       pt=copy.copy(legs[0])
       pt.y+=.1
       legs.append(pt)
    
    if len(legs)==0:
        return Polygon(),Polygon(),0,0,0
    
    #now find point perpendicular to legs:
    center=Point32()
    center.x=(legs[0].x+legs[1].x)/2.0
    center.y=(legs[0].y+legs[1].y)/2.0
    center.z=legs[0].z
    slope=-(legs[1].x-legs[0].x)/(legs[1].y-legs[0].y)   
#    slope=max(0.0,slope)
    
    target=copy.copy(center)
    norm=1.0/sqrt(1.0+slope*slope)
    target.x-=1.0*norm
    target.y-=slope*.8*norm
    
    if target.x<0.0:
        target.y*=.2
        
    legs.append(center)
    legs.append(target)
    
    #make region display:
    region=Polygon()
    for x,y in [(boxback,-boxwidth),(boxback,boxwidth), (boxfront,boxwidth),(boxfront,-boxwidth)]:
       pt=Point32()
       pt.x=x
       pt.y=y
       region.points.append(copy.copy(pt))
    region.points.append(copy.copy(region.points[0]))
    
    plegs=Polygon()
    plegs.points=legs
    glegs=transformPoly(globalframe,plegs,tcloud.header)
    gregion=transformPoly(globalframe,region,tcloud.header)
    return glegs,gregion,slope/2.0,target.x,target.y



def myCloudCallback(cloud):
    #find out which points are in front of the robot:
    legs,region,slope,x,y=getPersonPts(cloud)
    #publish a polymap that shows the person, and the expected region
    pmap=PolygonalMap()
    pmap.polygons=[legs,region]
    pmap.header=cloud.header
    pub.publish(pmap)
    
    print slope,x,y
    cmd=Twist()
    cmd.angular.z=slope 
    cmd.linear.x=x 
    cmd.linear.y=y 
    pub2.publish(cmd)


#this command registers this process with the ros master
rospy.init_node('waltz_detector')

#register a publisher, to topic '/base_controller/command', of type Twist
pub = rospy.Publisher('/waltzer', PolygonalMap)

#register a publisher, to topic '/base_controller/command', of type Twist
pub2 = rospy.Publisher('/base_controller/command', Twist)

#register a callback for messages of type LaserScan, on the topic "/base_scan"
sub = rospy.Subscriber("/kalman_filt_cloud", PointCloud, myCloudCallback)

listener = tf.TransformListener()
#this line is equivalent to: 
# while(everything is ok)
#   check for messages, call callbacks
#   sleep
rospy.spin()

