import roslib; roslib.load_manifest("drive_base")
import math
import rospy
import actionlib
import drive_base.msg

turnClient = actionlib.SimpleActionClient("turn_base_action", drive_base.msg.turnBaseAction)


if __name__ == '__main__':
    rospy.init_node("cancel_base")
    turnClient.cancel_all_goals()
