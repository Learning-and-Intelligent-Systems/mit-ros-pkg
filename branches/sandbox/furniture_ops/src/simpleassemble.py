#!/usr/bin/env python
import roslib; roslib.load_manifest('laser_assembler')
import rospy; from laser_assembler.srv import *
from sensor_msgs.msg import *
from sensor_msgs.srv import *

rospy.init_node("flash_assemble")
pub = rospy.Publisher('bigcloud', PointCloud)
pub = rospy.Publisher('bigcloud2', PointCloud2)
rospy.wait_for_service("assemble_scans")
try:
    assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)
    assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)
    resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
    print rospy.get_rostime()
    pub.publish(resp.cloud)
    print "published cloud with %u points" % len(resp.cloud.points)

    
except rospy.ServiceException, e:
    print "Service call failed: %s"%e
