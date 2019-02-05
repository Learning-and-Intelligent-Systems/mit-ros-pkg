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

client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)

client.wait_for_server()

while not rospy.is_shutdown():
    g = PointHeadGoal()
    g.target.header.frame_id = '/r_gripper_r_finger_tip_link'
    g.target.point.x = 0.0
    g.target.point.y = 0.0
    g.target.point.z = 0.2
    g.min_duration = rospy.Duration(.1)

    client.send_goal(g)
    client.wait_for_result()
    rospy.sleep(.1)
