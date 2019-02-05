import roslib; roslib.load_manifest('opendoors')
import rospy
import actionlib
import opendoors.msg
from geometry_msgs.msg import Pose , PoseStamped
import math
from ee_cart_imped_control.msg import EECartImpedGoal ,EECartImpedAction ,StiffPoint

def addTrajectoryPoint(goal, x, y, z, ox, oy, oz, ow,
                           fx, fy, fz, tx, ty, tz, isfx, isfy, isfz,
                           istx, isty, istz, time):
		new_point = len(goal.trajectory)
		#goal.trajectory.resize(new_point+1)
		goal.trajectory.append(StiffPoint())
		goal.trajectory[new_point].pose.position.x = x
		goal.trajectory[new_point].pose.position.y = y
		goal.trajectory[new_point].pose.position.z = z
		goal.trajectory[new_point].pose.orientation.x = ox
		goal.trajectory[new_point].pose.orientation.y = oy
		goal.trajectory[new_point].pose.orientation.z = oz
		goal.trajectory[new_point].pose.orientation.w = ow
		goal.trajectory[new_point].wrench_or_stiffness.force.x = fx
		goal.trajectory[new_point].wrench_or_stiffness.force.y = fy
		goal.trajectory[new_point].wrench_or_stiffness.force.z = fz
		goal.trajectory[new_point].wrench_or_stiffness.torque.x = tx
		goal.trajectory[new_point].wrench_or_stiffness.torque.y = ty
		goal.trajectory[new_point].wrench_or_stiffness.torque.z = tz
		
		goal.trajectory[new_point].is_wrench.append(isfx)
		goal.trajectory[new_point].is_wrench.append(isfy)
		goal.trajectory[new_point].is_wrench.append(isfz)
		goal.trajectory[new_point].is_wrench.append(istx)
		goal.trajectory[new_point].is_wrench.append(isty)
		goal.trajectory[new_point].is_wrench.append(istz)
		goal.trajectory[new_point].time_from_start = rospy.Duration(time)
def main1():
	ee_cart_imped_client = actionlib.SimpleActionClient('r_arm_cart_imped_controller/ee_cart_imped_action', EECartImpedAction)
	ee_cart_imped_client.wait_for_server(rospy.Duration(5.0))
	print "waiting for server"
	goal = EECartImpedGoal() 
	goal.header.frame_id = "torso_lift_link"
	goal.header.stamp = rospy.Time.now()
	
	#return self.tf_listener.lookupTransform('/torso_lift_link', '/l_gripper_tool_frame', rospy.Time(0))
	pose = Pose()
	
	pose.position.x = 0.592
	pose.position.y = -0.386
	pose.position.z = -.196
	pose.orientation.x = 0.0
	pose.orientation.y = 0
	pose.orientation.z = 0
	pose.orientation.w = 1
	radius = 0.6
	#radius*math.sin(math.pi/3)
	#Works for both configuration
#	addTrajectoryPoint (goal,pose.position.x-.2, pose.position.y, pose.position.z, 0, 0, 0, 1,-13, -10 , 2 , 30, 30,0,True, True, True, False, False,False, 1.4)
#	addTrajectoryPoint(goal, pose.position.x - 0.2 , pose.position.y ,  pose.position.z, 0, 0, 0, 1,-4, -4, 2, 30,30, 0,True, True, True, False, False,False, 5)
	addTrajectoryPoint(goal,pose.position.x-.2,pose.position.y,pose.position.z,0,0,0,1,-13,0,0,0,0,0,True,True,True,False,False,False,1.4)
	addTrajectoryPoint(goal,pose.position.x-.2,pose.position.y,pose.position.z,0,0,0,1,-13,0,-5,0,0,0,True,True,True,False,False,False,5)
	
	print "sending goal"
	ee_cart_imped_client.send_goal(goal)
	print "waiting for result"		
	ee_cart_imped_client.wait_for_result()
if __name__ == '__main__':
	rospy.init_node('test_handle_grasp_node')
	main1()
