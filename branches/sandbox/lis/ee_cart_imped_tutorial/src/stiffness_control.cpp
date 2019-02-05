#include <ee_cart_imped_action/ee_cart_imped_arm.hh>

int main(int argc, char **argv){
  ros::init(argc, argv, "stiffness_control_test");

  EECartImpedArm arm("r_arm_cart_imped_controller");

  ee_cart_imped_control::EECartImpedGoal traj;

  EECartImpedArm::addTrajectoryPoint(traj, 
				     .5, -.2, 1.0, 0, 0, 0, 1,
				     1000, 1000, 1000, 30, 30, 30,
				     false, false, false, false, false, false,
				     4);

  EECartImpedArm::addTrajectoryPoint(traj, 
				     .5, -.2, 2.0, 0, 0, 0, 1,                 // x,y,z,ox,oy,oz,ow
				     1000, 1000, 1000, 30, 30, 30,             // fx,fy,fz,tx,ty,tz
				     false, false, false, false, false, false, // isfx,isfy,isfz,istx,isty,istz
				     30);

  arm.startTrajectory(traj);
}
