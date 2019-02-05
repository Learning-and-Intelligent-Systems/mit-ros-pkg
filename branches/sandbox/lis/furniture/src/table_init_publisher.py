#!/usr/bin/env python
import roslib; roslib.load_manifest('furniture')
import rospy
import sys
from furniture.msg import Add_Table
from geometry_msgs.msg import Point

def talker():
    pub = rospy.Publisher('add_table', Add_Table)
    rospy.init_node('talker')
    # .44,.43
    w = .436
    x = 1
    y = 0
    z = .736

    # input table model and pose here
    add_table_msg = Add_Table()
    add_table_msg.id = 1
    add_table_msg.poly.points = [Point(),Point(),Point(),Point()]
    add_table_msg.poly.points[0].x = x-w/2
    add_table_msg.poly.points[0].y = y-w/2
    add_table_msg.poly.points[0].z = z

    add_table_msg.poly.points[1].x = x-w/2
    add_table_msg.poly.points[1].y = y+w/2
    add_table_msg.poly.points[1].z = z

    add_table_msg.poly.points[2].x = x+w/2
    add_table_msg.poly.points[2].y = y+w/2
    add_table_msg.poly.points[2].z = z

    add_table_msg.poly.points[3].x = x+w/2
    add_table_msg.poly.points[3].y = y-w/2
    add_table_msg.poly.points[3].z = z
    add_table_msg.pose.x = 0
    add_table_msg.pose.y = 0
    add_table_msg.pose.theta = 0
    

    while not rospy.is_shutdown():
        #str = "hello world %s"%rospy.get_time()
        #rospy.loginfo(str)
        print "publishing"
        pub.publish(add_table_msg)
        rospy.sleep(1.0)
        pub.publish(add_table_msg)
        #rospy.sleep(1.0)
        #pub.publish(add_table_msg)
        #rospy.sleep(1.0)
        #pub.publish(add_table_msg)
        #rospy.sleep(1.0)
        sys.exit("Published")
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
