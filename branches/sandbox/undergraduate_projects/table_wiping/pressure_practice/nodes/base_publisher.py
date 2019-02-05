#!/usr/bin/env python
import roslib
roslib.load_manifest('pressure_practice')
import rospy
from geometry_msgs.msg import Twist
import tf
import tf.msg


class BasePublisher():
    def __init__(self):
	self.base_pub = rospy.Publisher('cmd_vel', Twist)
	rospy.init_node('base_publisher')
	self.tf_listener = tf.TransformListener()
    def driveForwardOdom(self, dist):
	self.tf_listener.waitForTransform("base_footprint", "odom_combined", rospy.Time(0), rospy.Duration(1.0))
#	start_transform = tf.msg.StampedTransform()
#        current_transform = tf.msg.StampedTransform()
        start_transform = self.tf_listener.lookupTransform("base_footprint", "odom_combined", rospy.Time(0))
        base_cmd = Twist()
        base_cmd.x = 0.25
        base_cmd.y = base_cmd.z = 0
        rate = rospy.Rate(10.0)
        done = False
        while not done:
            self.base_pub.publish(base_cmd)
	    rate.sleep()
	    try:
		self.tf_listener.lookupTransform("base_footprint", "odom_combined", rospy.Time(0))
            except:
		print "Error"
		break
	    relative_transform = tf.Transform(start_transform.inverse()*current_transform)
	    dist_moved = relative_transform.getOrigin().length()
            if dist_moved > dist:
	        done = True



if __name__ == '__main__':
    BP = BasePublisher()
    BP.driveForwardOdom(0.5)
