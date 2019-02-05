#include <ee_cart_imped_action/ee_cart_imped_arm.hh>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>

/**
 *\brief C++ test file for ee_cart_imped_action.
 */

int main(int argc, char **argv) {
  ros::init(argc, argv, "ee_cart_imped_action_cpp_testing_node");

  EECartImpedArm right_arm("r_arm_cart_imped_controller"),
    left_arm("l_arm_cart_imped_controller");
  ee_cart_imped_msgs::EECartImpedGoal r_traj, l_traj;
  
  ROS_INFO("The ee_cart_imped_action does not check for collisions.  Move the robot away from obstacles and press enter when ready to proceed.");
  std::string input;
  getline(std::cin, input);

  ROS_INFO("Moving to initial positions");

  EECartImpedArm::addTrajectoryPoint(r_traj, 0, -0.4, -0.2,
				     0, 0.707, 0, 0.707,
				     1000, 1000, 1000, 100, 100, 100,
				     false, false, false, false, false, false,
				     4.0, "/torso_lift_link");
  right_arm.startTrajectory(r_traj);
  EECartImpedArm::addTrajectoryPoint(l_traj, 0, 0.4, -0.2,
				     0, 0.707, 0, 0.707,
				     1000, 1000, 1000, 100, 100, 100,
				     false, false, false, false, false, false,
				     4.0, "/torso_lift_link");
  left_arm.startTrajectory(l_traj);
  r_traj.trajectory.clear();
  l_traj.trajectory.clear();
  ROS_INFO("Beginning trajectory tests");
  for (int i = 0; i < 2; i++) {
    EECartImpedArm::addTrajectoryPoint(r_traj, 0.6, 0, 1.0, 0.707, 0, 0, 0.707,
				       8, 1000, 5, 100, 100, 100,
				       true, false, true, false, false, false,
				       4.0, "/base_link");
    EECartImpedArm::addTrajectoryPoint(r_traj, 0.6, 0, 1.0, 0.707, 0, 0, 0.707,
				       -3, 1000, 5, 100, 100, 100,
				       true, false, true, false, false, false,
				       6.0, "/base_link");
    ROS_INFO("Press enter for a force trajectory of 2 points on the right arm");
    getline(std::cin, input);
    right_arm.startTrajectory(r_traj);
    r_traj.trajectory.clear();
    EECartImpedArm::addTrajectoryPoint(r_traj, 0.6, 0, 2.0, 0.707, 0, 0, 0.707,
				       1000, 1000, 1000, 1.0, 100, 100,
				       false, false, false, true, false, false,
				       3.0, "/base_link");
    EECartImpedArm::addTrajectoryPoint(r_traj, 0.6, 0, 1.0, 0.5, 0.5, 0.5, 0.5,
				       1000, 1000, 1000, 100, 100, 100,
				       false, false, false, false, false, false,
				       5.0, "/base_link");
    EECartImpedArm::addTrajectoryPoint(r_traj,
				       0.2, -0.6, 1.0, -0.5, 0.5, 0.5, 0.5,
				       1000, 1000, 1000, 100, 100, 100,
				       false, false, false, false, false, false,
				       9.0, "/base_link");
    
    ROS_INFO("Press enter for a torque to stiffness trajectory on the right arm");
    getline(std::cin, input);
    right_arm.startTrajectory(r_traj);
    r_traj.trajectory.clear();
    EECartImpedArm::addTrajectoryPoint(l_traj, 0, 0.4, 0, 0.707, 0, 0, 0.707,
				       1000, 1000, 1000, 100, 100, 100,
				       false, false, false, false, false, false,
				       2.0, "/torso_lift_link");
    EECartImpedArm::addTrajectoryPoint(l_traj, 0, 0.4, 0, 0.707, 0, 0, 0.707,
				       -5, 1000, 1000, 100, 100, 100,
				       true, false, false, false, false, false,
				       4.0, "/torso_lift_link");
    EECartImpedArm::addTrajectoryPoint(l_traj, 
				       0.1, 0.4, -0.2, 0.707, 0, 0, 0.707,
				       1000, 1000, 1000, 100, 100, 100,
				       false, false, false, false, false, false,
				       6.0, "/torso_lift_link");
    ROS_INFO("Press enter for a trajectory on the left arm");
    getline(std::cin, input);
    left_arm.startTrajectory(l_traj);
    l_traj.trajectory.clear();

    EECartImpedArm::addTrajectoryPoint(r_traj, 0.6, 0, 1.0, 0.707, 0, 0, 0.707,
				       1000, 1000, 1000, 100, 100, 100,
				       false, false, false, false, false, false,
				       2.0, "/base_link");
    EECartImpedArm::addTrajectoryPoint(r_traj, 0.6, 0, 0.0, 0.707, 0, 0, 0.707,
				       1000, -10, 1000, 100, 100, 100,
				       false, true, false, false, false, false,
				       4.0, "/torso_lift_link");
    EECartImpedArm::addTrajectoryPoint(r_traj, 0.6, 0, 1.0, 0.707, 0, 0, 0.707,
				       1000, 1000, -5, 100, 100, 100,
				       false, false, true, false, false, false,
				       6.0, "/base_link");
    EECartImpedArm::addTrajectoryPoint(r_traj, 0.1, -0.4, 1.0, 0, 0, 0, 1.0,
				       1000, 1000, 1000, 100, 100, 100,
				       false, false, false, false, false, false,
				       8.0, "/base_link");
    ROS_INFO("Press enter for a mixed trajectory on right arm");
    getline(std::cin, input);
    right_arm.startTrajectory(r_traj);
    r_traj.trajectory.clear();
  }

  ROS_INFO("Beginning canceling tests");
  EECartImpedArm::addTrajectoryPoint(r_traj, 0.6, 0, 1.0, 0, 0, 0, 1,
				     1000, 1000, 1000, 100, 100, 100,
				     false, false, false, false, false, false,
				     4.0, "/base_link");
  right_arm.startTrajectory(r_traj);
  r_traj.trajectory.clear();
  EECartImpedArm::addTrajectoryPoint(r_traj, 0.6, 0, 1.0, 0.707, 0, 0, 0.707,
				     1000, 1000, -5, 100, 100, 100,
				     false, false, true, false, false, false,
				     2.0, "/base_link");
  EECartImpedArm::addTrajectoryPoint(r_traj, 0.6, 0, 1.0, 0, 0, 0, 1.0,
				     1000, 1000, 1000, 100, 100, 100,
				     false, false, false, false, false, false,
				     4.0, "/base_link");
  ROS_INFO("Press enter to test canceling goal");
  getline(std::cin, input);
  right_arm.startTrajectory(r_traj, false);
  right_arm.stopTrajectory();
  ROS_INFO("Press enter to test canceling goal on second trajectory point");
  getline(std::cin, input);
  right_arm.startTrajectory(r_traj, false);
  ros::Duration(2.5).sleep();
  right_arm.stopTrajectory();
  ROS_INFO("Returning to initial position");
  right_arm.startTrajectory(r_traj);
  ROS_INFO("Press enter to test goal preempt");
  getline(std::cin, input);
  right_arm.startTrajectory(r_traj, false);
  ros::Duration(2.5).sleep();
  right_arm.startTrajectory(r_traj);
  ROS_INFO("Press enter to test quick succession of goals.");
  getline(std::cin, input);
  for (int i = 0; i < 50; i++) {
      right_arm.startTrajectory(r_traj, false);
      ros::Duration(0.1).sleep();
  }
  ROS_INFO("Returning to initial position");
  right_arm.startTrajectory(r_traj);
  r_traj.trajectory.clear();
  EECartImpedArm::addTrajectoryPoint(r_traj, 0, -0.4, -0.2, 0, 0.707, 0, 0.707,
				     1000, 1000, 1000, 100, 100, 100,
				     false, false, false, false, false, false,
				     4.0, "/torso_lift_link");
  EECartImpedArm::addTrajectoryPoint(l_traj, 0, 0.4, -0.2, 0, 0.707, 0, 0.707,
				     1000, 1000, 1000, 100, 100, 100,
				     false, false, false, false, false, false,
				     4.0, "/torso_lift_link");
  ROS_INFO("Press enter to test simultaneous arm movement");
  getline(std::cin, input);
  left_arm.startTrajectory(l_traj, false);
  right_arm.startTrajectory(r_traj);
  ROS_INFO("Tests completed");
}

