
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/PoseStamped.h>
#include <move_arm_service/InterpolatedIK.h>
#include <motion_planning_msgs/PositionConstraint.h>
#include <motion_planning_msgs/OrientationConstraint.h>
#include <motion_planning_msgs/Constraints.h>
#include <motion_planning_msgs/GetMotionPlan.h>
#include <motion_planning_msgs/convert_messages.h>
#include <planning_environment_msgs/GetRobotState.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>

/**
 *\brief Service callback function.  Uses interpolated ik to
 *calculate and move the arm along a collision-free interpolated
 *IK trajectory.
 *
 *@param req The service request containing the starting and ending
 *poses for the arm
 *@param res The result of the service call (pass by reference)
 *@returns True by value and the result of the service call by reference.
 *
 *@todo If the user does not specify an initial position, assume the
 *starting position is the current position of the arm.
 */

bool move_arm(move_arm_service::InterpolatedIK::Request &req,
	      move_arm_service::InterpolatedIK::Response &res) {
  
  std::string link_name, controller_name, ik_name;
  if (req.arm[0] == 'r') {
    link_name = "r_wrist_roll_link";
    controller_name = "r_arm_controller/joint_trajectory_action";
    ik_name = "r_interpolated_ik_motion_plan";
  } else {
    link_name = "l_wrist_roll_link";
    controller_name = "l_arm_controller/joint_trajectory_action";
    ik_name = "l_interpolated_ik_motion_plan";
  }
  res.error_code.val = res.error_code.SUCCESS;
  motion_planning_msgs::PositionConstraint pos_constr;
  motion_planning_msgs::OrientationConstraint orientation_constr;
  motion_planning_msgs::poseStampedToPositionOrientationConstraints
    (req.end_pose, link_name, pos_constr, orientation_constr);
  motion_planning_msgs::Constraints goal_constraints;
  goal_constraints.position_constraints.push_back(pos_constr);
  goal_constraints.orientation_constraints.push_back(orientation_constr);

  //assume we start in the current position
  ros::NodeHandle n;
  ros::ServiceClient state_client = 
    n.serviceClient<planning_environment_msgs::GetRobotState>
    ("/environment_server/get_robot_state");
  planning_environment_msgs::GetRobotState state_srv;
  if (!state_client.call(state_srv)) {
    ROS_ERROR("Failed to call GetRobotState service to find starting state");
    res.error_code.val = res.error_code.INCOMPLETE_ROBOT_STATE;
    return true;
  }
  

  motion_planning_msgs::RobotState start_state;
  start_state = state_srv.response.robot_state;
  start_state.multi_dof_joint_state = motion_planning_msgs::MultiDOFJointState();
  start_state.multi_dof_joint_state.child_frame_ids.push_back(link_name);
  start_state.multi_dof_joint_state.poses.push_back(req.start_pose.pose);
  start_state.multi_dof_joint_state.frame_ids.push_back(req.start_pose.header.frame_id);
  start_state.multi_dof_joint_state.stamp = ros::Time::now();

  motion_planning_msgs::GetMotionPlan motion_plan;
  motion_plan.request.motion_plan_request.start_state = start_state;
  motion_plan.request.motion_plan_request.goal_constraints = goal_constraints;
  motion_plan.request.motion_plan_request.ordered_collision_operations =
    req.ordered_collision_operations;
  motion_plan.request.motion_plan_request.allowed_contacts =
    req.allowed_contacts;
  

  ros::ServiceClient ik_client = 
    n.serviceClient<motion_planning_msgs::GetMotionPlan>
    (ik_name);
  
  if (!ik_client.call(motion_plan)) {
    ROS_ERROR("Call to IK client failed.");
    res.error_code.val = res.error_code.PLANNING_FAILED;
    return true;
  }

  for (size_t i = 0; i < motion_plan.response.trajectory_error_codes.size();
       i++) {
    if (motion_plan.response.trajectory_error_codes[i].val !=
	motion_plan.response.trajectory_error_codes[i].SUCCESS) {
      ROS_INFO("Interpolated IK: Unable to plan path.  Step %u failed with error code %d", 
	       (unsigned int)i, motion_plan.response.trajectory_error_codes[i].val);
      res.error_code = motion_plan.response.trajectory_error_codes[i];
      return true;
    }
  }

  //use the joint trajectory action to execute the plan
  actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>
    traj_client(controller_name);
  while (ros::ok() && !traj_client.waitForServer()) {
    ROS_INFO("Waiting for joint trajectory action");
  }

  pr2_controllers_msgs::JointTrajectoryGoal traj_goal;
  traj_goal.trajectory = motion_plan.response.trajectory.joint_trajectory;
  traj_goal.trajectory.header.stamp = ros::Time::now();
  traj_client.sendGoal(traj_goal);
  traj_client.waitForResult();

  return true;
}


/**
 *\brief Uses interpolated IK to move the PR2 arm a short distance on
 *a collision free trajectory.
 *
 *This is a wrapper around the interpolated_ik service that allows
 *the user to use the service just by specifying a starting and
 *ending pose.  It makes the service easier to use, but also takes
 *away some functionality.
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "interpolated_ik_helper_node");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("run_interpolated_ik",
						  move_arm);
  ROS_INFO("Ready to do interpolated IK trajectories!");
  ros::spin();
  return 0;
}
