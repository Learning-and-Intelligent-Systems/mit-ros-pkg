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

##### White Cabinet  #####
    add_multiplanar_msg.id = "One-shelf Cabinet"

    add_multiplanar_msg.pose.x = 0.75  #1.7
    add_multiplanar_msg.pose.y = 0.0  #.2
    add_multiplanar_msg.pose.theta = 0.0 #-3.14/2
    add_multiplanar_msg.pose.z = 0.66

    length = 0.30
    width = 0.61
    height = 0.51

    ### Top Face ###
    topFace = Polygon()
    topFace.points = [Point(),Point(),Point(),Point()]

    topFace.points[0].x = -length/2
    topFace.points[0].y = -width/2
    topFace.points[0].z = height

    topFace.points[1].x = length/2
    topFace.points[1].y = -width/2
    topFace.points[1].z = height

    topFace.points[2].x = length/2
    topFace.points[2].y = width/2
    topFace.points[2].z = height

    topFace.points[3].x = -length/2
    topFace.points[3].y = width/2
    topFace.points[3].z = height

    add_multiplanar_msg.model.polygons.append (topFace)

    ### Left Face ###
    leftFace = Polygon()
    leftFace.points = [Point(),Point(),Point(),Point()]

    leftFace.points[0].x = -length/2
    leftFace.points[0].y = width/2
    leftFace.points[0].z = height

    leftFace.points[1].x = length/2
    leftFace.points[1].y = width/2
    leftFace.points[1].z = height

    leftFace.points[2].x = length/2
    leftFace.points[2].y = width/2
    leftFace.points[2].z = 0.0

    leftFace.points[3].x = -length/2
    leftFace.points[3].y = width/2
    leftFace.points[3].z = 0.0

    add_multiplanar_msg.model.polygons.append (leftFace)

    ### Right Face ###
    rightFace = Polygon()
    rightFace.points = [Point(),Point(),Point(),Point()]

    rightFace.points[0].x = -length/2
    rightFace.points[0].y = -width/2
    rightFace.points[0].z = height

    rightFace.points[1].x = length/2
    rightFace.points[1].y = -width/2
    rightFace.points[1].z = height

    rightFace.points[2].x = length/2
    rightFace.points[2].y = -width/2
    rightFace.points[2].z = 0.0

    rightFace.points[3].x = -length/2
    rightFace.points[3].y = -width/2
    rightFace.points[3].z = 0.0

    add_multiplanar_msg.model.polygons.append (rightFace)

    ### Bottom Face ### Move up
    botFace = Polygon()
    botFace.points = [Point(),Point(),Point(),Point()]

    botFace.points[0].x = -length/2
    botFace.points[0].y = -width/2
    botFace.points[0].z = 0.02

    botFace.points[1].x = length/2
    botFace.points[1].y = -width/2
    botFace.points[1].z = 0.02

    botFace.points[2].x = length/2
    botFace.points[2].y = width/2
    botFace.points[2].z = 0.02

    botFace.points[3].x = -length/2
    botFace.points[3].y = width/2
    botFace.points[3].z = 0.02

    add_multiplanar_msg.model.polygons.append (botFace)

    ### Back Face ###
    backFace = Polygon()
    backFace.points = [Point(),Point(),Point(),Point()]

    backFace.points[0].x = length/2
    backFace.points[0].y = -width/2
    backFace.points[0].z = height

    backFace.points[1].x = length/2
    backFace.points[1].y = width/2
    backFace.points[1].z = height

    backFace.points[2].x = length/2
    backFace.points[2].y = width/2
    backFace.points[2].z = 0.0

    backFace.points[3].x = length/2
    backFace.points[3].y = -width/2
    backFace.points[3].z = 0.0

    add_multiplanar_msg.model.polygons.append (backFace)

    ### Middle Shelf ###
    midShelf = Polygon()
    midShelf.points = [Point(),Point(),Point(),Point()]

    shelfHeight = 0.295

    midShelf.points[0].x = -length/2
    midShelf.points[0].y = -width/2
    midShelf.points[0].z = shelfHeight

    midShelf.points[1].x = length/2
    midShelf.points[1].y = -width/2
    midShelf.points[1].z = shelfHeight

    midShelf.points[2].x = length/2
    midShelf.points[2].y = width/2
    midShelf.points[2].z = shelfHeight

    midShelf.points[3].x = -length/2
    midShelf.points[3].y = width/2
    midShelf.points[3].z = shelfHeight

    add_multiplanar_msg.model.polygons.append (midShelf)

    
    print 'publishing add_multiplanar msg'
    rospy.sleep(.5)
    pub.publish(add_multiplanar_msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
