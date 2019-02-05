#include <ee_cart_imped_action/ee_cart_imped_arm.hh>


int main(int argc, char **argv) {
  ros::init(argc, argv, "pr2_cut_cake");
  
  EECartImpedArm arm("r_arm_cart_imped_controller");
  ee_cart_imped_control::EECartImpedGoal cut;
  double startx = 0.55, starty = -.097;
  double cx = 0.58, cy = -.207;
  double sqx = -0.0;
  double sqy = 0.0;
  double sqz = -0.0;
  double sqw = 1;

  double cqx = 0.0;
  double cqy = 0.05;
  double cqz = 0.24;
  double cqw = .968;



  int time = 0;
  double pause_height = -.15;
  pause_height = -.15;
  double down_force = -8;
  int rotimp = 100;

  // get the pen to the starting point and lower to the surface
  EECartImpedArm::addTrajectoryPoint(cut, startx, starty, pause_height, sqx, sqy, sqz, sqw,
				     1000, 1000, 1000, rotimp, rotimp, rotimp, 
                     false, false, false, false, false, false, (time+=3));
  EECartImpedArm::addTrajectoryPoint(cut, startx, starty, -1, 0.05, 0.05, 0, 1,
				     1000, 1000, down_force, rotimp, rotimp, rotimp, 
                     false, false, true, false, false, false, (time+=5));
  EECartImpedArm::addTrajectoryPoint(cut, startx, starty, -1, 0.05, 0.05, 0, 1,
				     1000, 1000, -20, rotimp, rotimp, rotimp, 
                     false, false, true, false, false, false, (time+=5));
  EECartImpedArm::addTrajectoryPoint(cut, startx, starty, pause_height, sqx, sqy, sqz, sqw,
				     1000, 1000, 1000, rotimp, rotimp, rotimp, 
                     false, false, false, false, false, false, (time+=4));
  EECartImpedArm::addTrajectoryPoint(cut, cx, cy, pause_height, cqx, cqy, cqz, cqw,
				     1000, 1000, 1000, rotimp, rotimp, rotimp, 
                     false, false, false, false, false, false, (time+=5));
  EECartImpedArm::addTrajectoryPoint(cut, cx, cy, -1, cqx, cqy, cqz, cqw,
				     1000, 1000, -8, rotimp, rotimp, rotimp, 
                     false, false, true, false, false, false, (time+=5));
  EECartImpedArm::addTrajectoryPoint(cut, cx, cy, -1, cqx, cqy, cqz, cqw,
				     1000, 1000, -20, rotimp, rotimp, rotimp, 
                     false, false, true, false, false, false, (time+=8));
  EECartImpedArm::addTrajectoryPoint(cut, cx, cy, pause_height, cqx, cqy, cqz, cqw,
				     1000, 1000, 1000, rotimp, rotimp, rotimp, 
                     false, false, false, false, false, false, (time+=4));
  EECartImpedArm::addTrajectoryPoint(cut, startx, starty, pause_height, sqx, sqy, sqz, sqw,
				     1000, 1000, 1000, rotimp, rotimp, rotimp, 
                     false, false, false, false, false, false, (time+=3));

  arm.startTrajectory(cut);
}
