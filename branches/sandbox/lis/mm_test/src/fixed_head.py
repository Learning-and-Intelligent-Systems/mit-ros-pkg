#! /usr/bin/python
import roslib
roslib.load_manifest('pr2_mechanism_controllers')
roslib.load_manifest('actionlib')
import time
from time import *
import rospy
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from geometry_msgs.msg import *

rospy.init_node('move_the_head', anonymous=False)

client = actionlib.SimpleActionClient(
    '/head_traj_controller/point_head_action', PointHeadAction)
#print "Before Begin"

client.wait_for_server()

#print "Before while"
#while True:
while not rospy.is_shutdown():


    g = PointHeadGoal()
    #g.target.header.frame_id = 'base_link'
    g.target.header.frame_id = '/r_gripper_tool_frame'
    #g.target.header.frame_id = '/odom_combined'
    g.target.point.x = 0.0
    g.target.point.y = 0.0
    g.target.point.z = -.3
    g.min_duration = rospy.Duration(.1)

    client.send_goal(g)
    #client.wait_for_result()
    #rospy.sleep(.1)
#    print "Sent"
#    if client.get_state() == GoalStatus.SUCCEEDED:
#        print "Succeeded"
#    else:
#        print "Failed"
