import roslib; roslib.load_manifest("drive_base")
import math
import rospy
import actionlib
import drive_base.msg

moveClient = actionlib.SimpleActionClient("move_base_action", drive_base.msg.moveBaseAction)


if __name__ == '__main__':
    rospy.init_node("cancel_base")
    moveClient.cancel_all_goals()
