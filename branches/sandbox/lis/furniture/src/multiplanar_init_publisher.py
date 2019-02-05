#!/usr/bin/env python
import roslib; roslib.load_manifest('furniture')
import rospy
import sys
import math
from geometry_msgs.msg import Point, Polygon
from furniture.msg import Add_Multiplanar, Multiplanar_Model, Pose_2D_With_Z

def talker():
    pub = rospy.Publisher('add_multiplanar', Add_Multiplanar)
    rospy.init_node('talker')

    add_multiplanar_msg = Add_Multiplanar()

##### 1X1 Gazebo Table #####
    # add_multiplanar_msg.id = "Simple Table"

    # add_multiplanar_msg.pose.x = 1.0
    # add_multiplanar_msg.pose.y = 0.0
    # add_multiplanar_msg.pose.theta = 0.0
    # add_multiplanar_msg.pose.z = .65

    # length = 1.0
    # width = 1.0
    # height = 0

##### Slightly Warped table in the lab #####
    # add_multiplanar_msg.id = "Warped Table"

    # add_multiplanar_msg.pose.x = 1.3
    # add_multiplanar_msg.pose.y = 0
    # add_multiplanar_msg.pose.theta = 3.14/2.0
    # add_multiplanar_msg.pose.z = .64

    # length = 1.2
    # width = .6
    # height = 0

    # firstFace = Polygon()
    # firstFace.points = [Point(),Point(),Point(),Point()]

    # firstFace.points[0].x = length/2
    # firstFace.points[0].y = -width/2
    # firstFace.points[0].z = height

    # firstFace.points[1].x = length/2
    # firstFace.points[1].y = width/2
    # firstFace.points[1].z = height

    # firstFace.points[2].x = -length/2
    # firstFace.points[2].y = width/2
    # firstFace.points[2].z = height

    # firstFace.points[3].x = -length/2
    # firstFace.points[3].y = -width/2
    # firstFace.points[3].z = height


##### Small Box Gazebo  #####
    add_multiplanar_msg.id = "Simple Box"

    add_multiplanar_msg.pose.x = 2.5  #1.7
    add_multiplanar_msg.pose.y = 0.0  #.2
    add_multiplanar_msg.pose.theta = 3.5/4
    add_multiplanar_msg.pose.z = 0.0

    length = 0.5
    width = 0.5
    height = 1.0

    firstFace = Polygon()
    firstFace.points = [Point(),Point(),Point(),Point()]

    firstFace.points[0].x = -length/2
    firstFace.points[0].y = -width/2
    firstFace.points[0].z = height

    firstFace.points[1].x = -length/2
    firstFace.points[1].y = width/2
    firstFace.points[1].z = height

    firstFace.points[2].x = -length/2
    firstFace.points[2].y = width/2
    firstFace.points[2].z = 0.0

    firstFace.points[3].x = -length/2
    firstFace.points[3].y = -width/2
    firstFace.points[3].z = 0.0

    secondFace = Polygon()
    secondFace.points = [Point(),Point(),Point(),Point()]

    secondFace.points[0].x = -length/2
    secondFace.points[0].y = width/2
    secondFace.points[0].z = height

    secondFace.points[1].x = length/2
    secondFace.points[1].y = width/2
    secondFace.points[1].z = height

    secondFace.points[2].x = length/2
    secondFace.points[2].y = width/2
    secondFace.points[2].z = 0.0

    secondFace.points[3].x = -length/2
    secondFace.points[3].y = width/2
    secondFace.points[3].z = 0.0

    add_multiplanar_msg.model.polygons.append (firstFace)
    add_multiplanar_msg.model.polygons.append (secondFace)

    
    print 'publishing add_multiplanar msg'
    rospy.sleep(.5)
    pub.publish(add_multiplanar_msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
