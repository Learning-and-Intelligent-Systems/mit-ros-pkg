#!/usr/bin/env python
import roslib; roslib.load_manifest('furniture')
import rospy
import sys
from geometry_msgs.msg import Point, Polygon
from furniture.msg import Add_Multiplanar, Multiplanar_Model, Pose_2D_With_Z

def talker():
    pub = rospy.Publisher('add_multiplanar', Add_Multiplanar)
    rospy.init_node('talker')

    add_multiplanar_msg = Add_Multiplanar()

##### Huan's Small Box  #####
    add_multiplanar_msg.id = "Small Box"

    add_multiplanar_msg.pose.x = 0.6
    add_multiplanar_msg.pose.y = 0.0
    add_multiplanar_msg.pose.theta = 0.0
    add_multiplanar_msg.pose.z = .64

    length = .1
    width = .2
    height = .1

    # topFace = Polygon()
    # topFace.points = [Point(),Point(),Point(),Point()]

    # topFace.points[0].x = length/2
    # topFace.points[0].y = -width/2
    # topFace.points[0].z = height

    # topFace.points[1].x = length/2
    # topFace.points[1].y = width/2
    # topFace.points[1].z = height

    # topFace.points[2].x = -length/2
    # topFace.points[2].y = width/2
    # topFace.points[2].z = height

    # topFace.points[3].x = -length/2
    # topFace.points[3].y = -width/2
    # topFace.points[3].z = height


    frontFace = Polygon()
    frontFace.points = [Point(),Point(),Point(),Point()]

    frontFace.points[0].x = -length/2 + .5
    frontFace.points[0].y = -width/2 
    frontFace.points[0].z = height +.3

    frontFace.points[1].x = -length/2 +.5
    frontFace.points[1].y = width/2 
    frontFace.points[1].z = height +.3

    frontFace.points[2].x = -length/2 +.5
    frontFace.points[2].y = width/2 
    frontFace.points[2].z = 0 +.3

    frontFace.points[3].x = -length/2 +.5
    frontFace.points[3].y = -width/2 
    frontFace.points[3].z = 0 +.3


    # add_multiplanar_msg.model.polygons.append (topFace)
    add_multiplanar_msg.model.polygons.append (frontFace)

    
    print 'publishing add_multiplanar msg'
    rospy.sleep(.5)
    pub.publish(add_multiplanar_msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
