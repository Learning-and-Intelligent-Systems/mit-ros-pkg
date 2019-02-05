#include <ee_cart_imped_action/ee_cart_imped_arm.hh>

int main(int argc, char **argv) {
  ros::init(argc, argv, "pr2_wipe_screen");

  EECartImpedArm arm("r_arm_cart_imped_controller");
  ee_cart_imped_control::EECartImpedGoal sweep;

   int time = 0;
   EECartImpedArm::addTrajectoryPoint(sweep, -0.8, -1, 0, 0, 0, -1, 1,
				      1000, 1000, 1000, 100, 100, 100, false,
				      false, false, false, false, false, (time+=4));
   EECartImpedArm::addTrajectoryPoint(sweep, -0.6, -1, -0.2, 0, 0, -1, 1,
				      1000, 1000, 1000, 100, 100, 100, false,
				      false, false, false, false, false, (time+=4));
   EECartImpedArm::addTrajectoryPoint(sweep, 0.6, -1, -1, 0, 0, -1, 1,
				      1000, 1000, -3, 100, 100, 100, false,
				      false, true, false, false, false, (time+=6));
//    EECartImpedArm::addTrajectoryPoint(sweep, 0.5, -1, -1, 0, 0, -1, 1,
// 				      1000, 1000, 0, 100, 100, 100, false,
// 				      false, true, false, false, false, (time+=1));
//    EECartImpedArm::addTrajectoryPoint(sweep, 0.6, -1, -0.1, 0, 0, -1, 1,
// 				      1000, 1000, 1000, 100, 100, 100, false,
// 				      false, false, false, false, false, (time+=3));

  arm.startTrajectory(sweep);
}
