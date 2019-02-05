import roslib
roslib.load_manifest('simple_controller')
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

pub = rospy.Publisher('/base_controller/command', Twist)
rospy.init_node('my_controller')

import tf
import copy
from math import  *
from numpy import  *
transformer=tf.Transformer(True, rospy.Duration(10.0))
listener = tf.TransformListener()


def getRobotPosition():
    listener.waitForTransform('/base_link','/odom_combined', rospy.Time(), rospy.Duration(.20))
    t=listener.lookupTransform('/odom_combined','/base_link', rospy.Time())#Note that a time of zero means latest common time
    print t
    return (t[0][0],t[0][1])

def vectorToTarget(target):
    pos=getRobotPosition()

    return (target[0]-pos[0],target[1]-pos[1])

def distanceToTarget(target):
    v=vectorToTarget(target)
    return sqrt(v[0]*v[0]+v[1]*v[1])

#drive in a circle around object
def strafeLeft(target, radius):
    #angle to travel
    da=.2
    cmd = Twist()
    cmd.linear.x=radius*(1-cos(da))
    cmd.linear.y=radius*sin(da)
    cmd.angular.z=-da
    return cmd
    
    

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
target=(4.1,3)
r = rospy.Rate(30) # 30hz
#while distanceToTarget(target) > .01:
#    v=vectorToTarget(target)
#    print v
#    cmd.linear.x=sign(v[0])*min(max(abs(v[0]),.1),1)
#    cmd.linear.y=sign(v[1])*min(max(abs(v[1]),.1),1)
for i in range(300):
    cmd=strafeLeft(target,1.00)
    print cmd
    print "---------------"
    pub.publish(cmd)
    r.sleep()
