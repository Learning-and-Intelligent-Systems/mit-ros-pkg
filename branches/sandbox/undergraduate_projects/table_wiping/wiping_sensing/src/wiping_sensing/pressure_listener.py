#!/usr/bin/env python
import roslib; roslib.load_manifest('wiping_sensing')
import rospy
import math
import threading
from fingertip_pressure.msg import PressureInfo
from pr2_msgs.msg import PressureState
import wiping_sensing.msg
pressure_threshold = 1000
#pressure_threshold = 3400
wait_threshold = 0.1


class PressureListener:
    """
    Listens for pressure values for the left gripper and maintains state about whether the gripper is currently touching the table.
    """
    def __init__(self):
        """
        Sets up the subscribtion thread for the pressure listener. 

        May hang if the left gripper is not publishing pressure information.
        """
	self.wait = 0
	self.state = "not_touching"
        self.pressure_threshold = pressure_threshold
        """
        The state of the gripper. Is 'touching' if the gripper is pushing on a table and 'not_touching' if the gripper is not.
        """
        self.avg_pressure = 0
        self.status_pub = rospy.Publisher("l_gripper_force_status", 
                                          wiping_sensing.msg.GripperForceStatus)
        self.thread = threading.Thread(target = self.callback, args = (PressureState(),))
        self.thread.start()
	rospy.Subscriber("/pressure/l_gripper_motor", PressureState, self.callback)


    def callback(self, data):
        """
        Method for the listener to get updated with pressure information. Updates the state information for this pressure_listener.
        """
	frontSensors = data.l_finger_tip[3:5] + data.r_finger_tip[3:5]
	self.avg_pressure = sum(frontSensors)/4.0
	if (self.state =="not_touching" or self.state == "waiting") and self.avg_pressure > self.pressure_threshold:
 	    self.wait = 0
	    self.state = "touching"
	if self.state =="touching" and self.avg_pressure < self.pressure_threshold:
	    self.state = "waiting"
	    self.wait = rospy.get_time()
	if self.state == "waiting" and self.avg_pressure < self.pressure_threshold and rospy.get_time() - self.wait > wait_threshold:
	    self.state = "not_touching"
        status_msg = wiping_sensing.msg.GripperForceStatus()
        status_msg.header.stamp = rospy.Time.now()
        status_msg.tip_pressure = self.avg_pressure
        status_msg.is_touching = (self.state == "touching")
        self.status_pub.publish(status_msg)
	
    def is_touching(self):
        """
        @returns: true if the gripper is pushing down on a surface. False otherwise.
        """
	return self.state == "touching"


if __name__ =="__main__":
    rospy.init_node('listener_test')
    listener = PressureListener()
    while True:
        if rospy.is_shutdown():
	    exit(0)
	print listener.is_touching(), listener.avg_pressure
