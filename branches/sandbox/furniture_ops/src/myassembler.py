#!/usr/bin/env python
import roslib; roslib.load_manifest('laser_assembler')
import rospy; from laser_assembler.srv import *
from sensor_msgs.msg import *

rospy.init_node("test_client")
pub = rospy.Publisher('chaircloud', PointCloud)
rospy.wait_for_service("assemble_scans")
try:
    assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)
    resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
    newcloud=PointCloud()
    newcloud.header=resp.cloud.header
    for c in resp.cloud.channels:
      channel=ChannelFloat32()
      channel.name=c.name
      newcloud.channels.append(channel)
    for i in range(len(resp.cloud.points)):
      p=resp.cloud.points[i]
      if p.z >0.0 and p.x > 0 and p.x < 1.5 and p.y < -1.0 and p.y > -2.0:
	newcloud.points.append(p)
	for c in range(len(resp.cloud.channels)):
	  newcloud.channels[c].values.append(resp.cloud.channels[c].values[c])
    
    pub.publish(newcloud)
    print "published cloud with %u points" % len(newcloud.points)
    
    print "Got cloud with %u points" % len(resp.cloud.points)
    fout=open("cloudout.ply",'w')
    fout.write("ply\n")
    fout.write("format ascii 1.0\n")
    fout.write("element vertex %u\n"%len(newcloud.points))
    fout.write("property float x\n")
    fout.write("property float y\n")
    fout.write("property float z\n")
    fout.write("element face 0\n")
    fout.write("property list uchar int vertex_indices\n")
    fout.write("end_header\n")
    for p in newcloud.points:
      fout.write("%f %f %f\n"%(p.x,p.y,p.z))
    fout.close()
    
    
    
except rospy.ServiceException, e:
    print "Service call failed: %s"%e
