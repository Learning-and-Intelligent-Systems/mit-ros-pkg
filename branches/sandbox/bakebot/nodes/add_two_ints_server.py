#!/usr/bin/env python
import roslib; roslib.load_manifest('bakebot')

from bakebot.srv import *
import rospy

def handle_add_two_ints(req):
    print "returning"
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print "ready to add"
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
