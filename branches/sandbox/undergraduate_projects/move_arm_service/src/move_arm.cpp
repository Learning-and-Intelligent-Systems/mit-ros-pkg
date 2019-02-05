#include <ros/ros.h>
#include "move_arm_service/MoveArm.h"
#include <object_manipulator/tools/mechanism_interface.h>
#include <motion_planning_msgs/OrderedCollisionOperations.h>
#include <motion_planning_msgs/LinkPadding.h>

/**
 *\brief Service callback function.  Calls move_arm to move the arm to a pose.
 *
 *@param req The service request.  Includes the pose to move the arm
 *to and which arm to move.
 *@param res The result of the service call (pass by reference).
 *This result is a boolean indicating success (true) or failure (false).
 *@returns True by value and a boolean indicating success or failure
 *by reference.
 */

bool move_arm(move_arm_service::MoveArm::Request &req,
	      move_arm_service::MoveArm::Response &res) {  
  res.succeeded = object_manipulator::mechInterface().
    moveArmToPose(req.arm, req.pose_stamped,
		  motion_planning_msgs::OrderedCollisionOperations(),
		  std::vector<motion_planning_msgs::LinkPadding>());
  return true;
}

/**
 *\brief Moves the PR2 arm to a point using a collision-free trajectory.
 *Wrapper around the move_arm action.
 *
 *This wrapper around the move_arm action takes a pose and moves 
 *the PR2 arm to that pose collision-free.  By making for a much
 *simpler message than the move_arm goal, it makes the action
 *easier to use but also removes some of its functionality.
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "move_arm_helper_node");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("move_arm_to_pose",
						  move_arm);
  ROS_INFO("Ready to move arms!");
  ros::spin();
  return 0;
}
