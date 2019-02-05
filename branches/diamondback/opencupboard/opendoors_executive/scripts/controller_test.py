import roslib; roslib.load_manifest('opendoors_executive')
from opendoors_executive.control_switcher import PR2CMClient
import rospy
import actionlib
import opendoors.msg
from geometry_msgs.msg import Pose, PoseStamped
from ee_cart_imped_control.msg import EECartImpedGoal, EECartImpedAction, StiffPoint
import move_arm_service.srv

def test_cartesian_control(arm):
    print "waiting for server"
    rospy.wait_for_service('move_arm_to_pose')
    move_srv = rospy.ServiceProxy('move_arm_to_pose',
                                  move_arm_service.srv.MoveArm)
    goal_pose = move_arm_service.srv.MoveArmRequest()
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "torso_lift_link"
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.pose.position.x = 0.2
    pose_stamped.pose.position.y = -0.7
    if arm == 'left_arm':
        pose_stamped.pose.position.y = 0.7
    pose_stamped.pose.position.z = 0
    pose_stamped.pose.orientation.x = 0
    pose_stamped.pose.orientation.y = 0
    pose_stamped.pose.orientation.z = 0
    pose_stamped.pose.orientation.w = 1
    goal_pose.pose_stamped =  pose_stamped
    goal_pose.arm = arm
    print "waiting for result"
    move_resp = move_srv(goal_pose)
    print 'result was', move_resp
    

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

def test_force_control(action_name):
	ee_cart_imped_client =\
            actionlib.SimpleActionClient\
            (action_name,\
                 EECartImpedAction)
	ee_cart_imped_client.wait_for_server(rospy.Duration(5.0))
	print "waiting for server"
	goal = EECartImpedGoal() 
	goal.header.frame_id = "torso_lift_link"
	goal.header.stamp = rospy.Time.now()
        if action_name[0] == 'r':
            forcey = -10
        else:
            forcey = 10

        addTrajectoryPoint (goal, 0, 0, 0, 0, 0, 0, 1,
                            -13, forcey, 2, 30, 30, 0,
                            True, True, True, False, False,False, 1.4)
	print "sending goal"
	ee_cart_imped_client.send_goal(goal)
	print "waiting for result"		
	ee_cart_imped_client.wait_for_result()

def main():
    #get_pr2cm_client is a static method of
    #PR2CMClient because we want to guarantee
    #there is only ever one instance of PR2CMClient
    #therefore, to get the client, we call this
    #function rather than a constructor.
    #load the cartesian controllers on the right arm
    PR2CMClient.load_cartesian(True)
    test_cartesian_control('right_arm')
    #load the force/impedance controllers on the right arm
    PR2CMClient.load_ee_cart_imped(True)
    test_force_control('r_arm_cart_imped_controller/ee_cart_imped_action')
    PR2CMClient.load_cartesian(False)
    test_cartesian_control('left_arm')
    #load the force/impedance controllers on the left arm
    PR2CMClient.load_ee_cart_imped(False)
    test_force_control('l_arm_cart_imped_controller/ee_cart_imped_action')


if __name__ == '__main__':
    rospy.init_node('test_controller_switching')
    main()
