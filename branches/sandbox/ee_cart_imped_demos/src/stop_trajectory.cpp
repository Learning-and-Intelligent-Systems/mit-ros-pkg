#include <ee_cart_imped_action/ee_cart_imped_arm.hh>

int main(int argc, char **argv) {
  ros::init(argc, argv, "stopTrajectory");

  EECartImpedArm arm("r_arm_cart_imped_controller");
  arm.stopTrajectory();
}
