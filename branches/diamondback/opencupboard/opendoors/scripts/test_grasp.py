import roslib; roslib.load_manifest('opendoors')
import rospy
import actionlib
import opendoors.msg
from geometry_msgs.msg import Pose , PoseStamped

def main():
	gripper_client = actionlib.SimpleActionClient('grasp_handle_action',opendoors.msg.GraspHandleAction)
        print "waiting for server"
        gripper_client.wait_for_server()
	
        goal_pose = opendoors.msg.GraspHandleGoal()

	pose_stamped = PoseStamped()
	pose_stamped.header.frame_id = "torso_lift_link"
	pose_stamped.header.stamp = rospy.Time.now()
    	pose_stamped.pose.position.x = 0
   	pose_stamped.pose.position.y = -0.7
    	pose_stamped.pose.position.z = 0
   	pose_stamped.pose.orientation.x = 0
    	pose_stamped.pose.orientation.y = 0
   	pose_stamped.pose.orientation.z = 0
   	pose_stamped.pose.orientation.w = 1
	goal_pose.pose_stamped =  pose_stamped
	goal_pose.arm = 'right_arm'
	goal_pose.handle_box.pose_stamped = pose_stamped
	goal_pose.handle_box.dimensions.x = 0.05
	goal_pose.handle_box.dimensions.y = 0.2
	goal_pose.handle_box.dimensions.z = 0.05
	
	print "waiting for right result"
	gripper_client.send_goal_and_wait(goal_pose)

	goal_pose.arm = 'left_arm'
	print "waiting for left result"
	pose_stamped.pose.position.y = 0.7
	gripper_client.send_goal_and_wait(goal_pose)

if __name__ == '__main__':
	rospy.init_node('test_handle_grasp_node')
	main()
