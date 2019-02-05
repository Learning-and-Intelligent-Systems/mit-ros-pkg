#!/usr/bin/env python
import roslib
roslib.load_manifest('simple_controller')
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

#you can find out what topics are being used on the robot by
#  switching to the robot rosmaster (ROS_MASTER_URI="http://pr2mm1:11311")
#  and running 'rostopic list'
#you will find a topic named /base_controller/command
#A 'rostopic info' reveals it to be of type 'geometry_msgs/Twist'
pub = rospy.Publisher('/base_controller/command', Twist)

#this command registers this process with the ros master
rospy.init_node('my_controller')


#Running 'rosmsg show Twist' gives this:
#[geometry_msgs/Twist]:
#geometry_msgs/Vector3 linear
#  float64 x
#  float64 y
#  float64 z
#geometry_msgs/Vector3 angular
#  float64 x
#  float64 y
#  float64 z
cmd = Twist()
#just make a list of velocity control signals:
vels=[0]*30+[.03]*30+[0]*20+[-.03]*30+[0]*30
r = rospy.Rate(30) # 30hz
for v in vels:
    cmd.linear.x=v
    print cmd
    print "---------------"
    pub.publish(cmd)
    r.sleep()
 

vels=[0]*30+[.1]*30+[0]*20+[-.1]*30+[0]*30
for v in vels:
    cmd.angular.z=v
    print cmd
    print "---------------"
    pub.publish(cmd)
    r.sleep()

