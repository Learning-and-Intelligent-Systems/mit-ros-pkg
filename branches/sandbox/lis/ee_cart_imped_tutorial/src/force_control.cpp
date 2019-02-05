#include <ee_cart_imped_action/ee_cart_imped_arm.hh>

int main(int argc, char **argv) {
  ros::init(argc, argv, "force_control_test");

  EECartImpedArm arm("r_arm_cart_imped_controller");

  ee_cart_imped_control::EECartImpedGoal traj;

  EECartImpedArm::addTrajectoryPoint(traj, .5, 0, 0, 0, 0, 0, 1,
				     1000, 1000, 1000, 30, 30, 30,
				     false, false, false, false, false, false, 
				     4);

  EECartImpedArm::addTrajectoryPoint(traj, .75, 0, 0, 0, 0, 0, 1,
				     10, 1000, 1000, 30, 30, 30,
				     true, false, false, false, false, false,
				     6);
  arm.startTrajectory(traj);
}
