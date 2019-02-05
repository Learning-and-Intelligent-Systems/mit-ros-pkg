#!/usr/bin/env python

import roslib
roslib.load_manifest('turntable')
import rospy
import rosbag
import math
from sensor_msgs.msg import *
from lib_robotis import *
import dynamic_reconfigure.client

from std_msgs.msg import Float64
#import ros_robotis as robotis
#from robotis.srv import MoveAng
#from robotis.lib_robotis import *


theta = -100
#client = robotis.ROS_Robotis_Client('turntable')
#rospy.wait_for_service('/robotis/servo_turntable_moveangle')
#client.move_angle(math.radians(theta))
dyn = USB2Dynamixel_Device('/dev/ttyUSB0', 1000000)
servo = Robotis_Servo(dyn, 1)
#servo.move_angle(math.radians(theta), blocking=True)
#bag = rosbag.Bag('test.bag', 'w')


#def callback(msg):e
#    global theta
#    global bag
#    global servo
#    print theta
#    bag.write('points', msg)
#    theta = theta + 10
#    if theta > 150:
#        bag.close()
#        rospy.signal_shutdown("done turning")
#    else:
#        servo.move_angle(math.radians(theta), blocking=True)


if __name__ == '__main__':
    rospy.init_node('turn')
    #rospy.Subscriber('/narrow_stereo_textured/points_throttle', PointCloud, callback, queue_size=10)


    client = dynamic_reconfigure.client.Client('/camera/driver')
    params = {'depth_registration': 1}
    client.update_configuration(params)

    pub_cloud = rospy.Publisher('/scan/points', PointCloud2)



   

    #move_angle = rospy.ServiceProxy('/robotis/servo_turntable_moveangle',MoveAng)
    print theta
    #move_angle(math.radians(theta),math.radians(50), 1)
    servo.move_angle(math.radians(theta), blocking=True)
    raw_input('Press enter to start scanning:')
    for theta in range(-100, 192, 10):
        #move_angle(math.radians(theta),math.radians(50), 1)
        #client.move_angle(math.radians(theta))
        servo.move_angle(math.radians(theta), blocking=True)

        print 'theta = ' + str(theta)
        time.sleep(1)

        for i in range(5):
            #msg = rospy.wait_for_message('/camera/depth_registered/points', PointCloud2)
            msg = rospy.wait_for_message('/points', PointCloud2)
            pub_cloud.publish(msg)
            print 'published scan'

