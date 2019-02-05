import roslib; roslib.load_manifest('base_trajectory_action')
from base_trajectory_action.msg import BaseTrajectoryAction, BaseTrajectoryGoal
from actionlib import SimpleActionClient
from geometry_msgs.msg import Pose2D
import rospy
import numpy as np

def main():
    client = SimpleActionClient("base_trajectory_action", BaseTrajectoryAction)
    client.wait_for_server()
    rospy.loginfo('Found action server!')
    goal = BaseTrajectoryGoal()
    goal.world_frame = "odom_combined"
    goal.robot_frame = "base_footprint"
    goal.trajectory = [Pose2D(x=0, y=0, theta=0)]
    goal.linear_velocity = 0.2;
    goal.angular_velocity = np.pi/4.0;
    goal.angular_error = 0.01;
    goal.linear_error = 0.02;
    rospy.loginfo('Sending goal')
    client.send_goal_and_wait(goal)

rospy.init_node('base_tester_node')
main()
