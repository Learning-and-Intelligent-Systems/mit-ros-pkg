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
    goal.robot_frame = "base_footprint"
    goal.world_frame = "odom_combined"
    for i in range(50):
        goal.trajectory.append(Pose2D(x=-1/49.0*i, y=-2.0/49.0*i, theta=np.pi/49.0*i))
#    goal.trajectory = [Pose2D(x=-1, y=0, theta=np.pi), Pose2D(x=-2, y=-1, theta=0)]
    goal.linear_velocity = 0.3;
    goal.angular_velocity = np.pi/4.0;
    goal.angular_error = 0.01;
    goal.linear_error = 0.02;
    rospy.loginfo('Sending goal')
    client.send_goal_and_wait(goal)

rospy.init_node('base_tester_node')
main()
