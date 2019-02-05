#!/usr/bin/env python
import roslib; roslib.load_manifest('furniture')
import rospy
from sensor_msgs.msg import PointCloud

cnt = 0

def callback(msg):
    global cnt
    cnt = cnt + 1
    print "X{" + str(cnt) + "} = [",
    for i in range(0, len(msg.points), 5):
        print str(msg.points[i].x),
        print str(msg.points[i].y),
        print str(msg.points[i].z),
        print ";"
    print "];\n"

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/narrow_stereo_textured/points_throttle", PointCloud, callback, queue_size=100)
    rospy.spin()

if __name__ == '__main__':
    listener()
