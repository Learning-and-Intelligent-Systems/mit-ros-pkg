#include <ee_cart_imped_action/ee_cart_imped_arm.hh>

int main(int argc, char **argv) {
  ros::init(argc, argv, "pr2_wipe_screen");

  EECartImpedArm arm("r_arm_cart_imped_controller");
  ee_cart_imped_control::EECartImpedGoal wipe;

   int time = 0;
   EECartImpedArm::addTrajectoryPoint(wipe, 0.6, 0, -0.1, 0, 0, 0, 1,
				      1000, 1000, 1000, 100, 100, 100, false,
				      false, false, false, false, false, (time+=4));
   
   EECartImpedArm::addTrajectoryPoint(wipe, 1, 0, -0.1, 0, 0, 0, 1,
				      6, 1000, 1000, 100, 100, 100, true,
				      false, false, false, false, false, (time+=3));
   EECartImpedArm::addTrajectoryPoint(wipe, 0.9, -0.1, 0, 0, 0, 0, 1,
				      5, 1000, 1000, 100, 100, 100, true,
				      false, false, false, false, false, (time+=3));
   EECartImpedArm::addTrajectoryPoint(wipe, 1, 0, -0.1, 0, 0, 0, 1,
				      10, 1000, 1000, 100, 100, 100, true,
				      false, false, false, false, false, (time+=3));

  arm.startTrajectory(wipe);
}
