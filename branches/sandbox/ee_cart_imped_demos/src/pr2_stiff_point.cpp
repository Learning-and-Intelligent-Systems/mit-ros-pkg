#include <ee_cart_imped_action/ee_cart_imped_arm.hh>


int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_action_test");
  
  EECartImpedArm arm("r_arm_cart_imped_controller");
  ee_cart_imped_control::EECartImpedGoal house; 
//- Translation: [0.665, -0.089, -0.103]
//- Rotation: in Quaternion [0.839, 0.280, -0.368, 0.286]

  double x = 0.65;
  double y = -0.1;
  double z = 0.100; 
  double qx = 0.839;
  double qy = -.2;
  double qz = -.4;
  double qw = -.3;
  
  double stiff_force_x = 1000;
  double stiff_force_y = 10;
  double stiff_force_z = 1000;
  double stiff_force_qx = 0;
  double stiff_force_qy = 0;
  double stiff_force_qz = 0;
  double stiff_force_qw = 0;

  bool is_force_x = false;
  bool is_force_y = false;
  bool is_force_z = false;
  bool is_force_qx = false;
  bool is_force_qy = false;
  bool is_force_qz = false;
  bool is_force_qw = false;

  int draw_time = 0;
  EECartImpedArm::addTrajectoryPoint(house, x, y, z, qx, qy, qz, qw, 
				     stiff_force_x, stiff_force_y, stiff_force_z, 
				     stiff_force_qx, stiff_force_qy, stiff_force_qz,  
				     is_force_x, is_force_y, is_force_z, 
				     is_force_qx, is_force_qy, is_force_qz, 1);
  arm.startTrajectory(house);
}
//- Translation: [0.512, -0.153, -0.331]
// Translation: [0.595, -0.152, -0.338]
// Translation: [0.553, -0.105, -0.336]
//.63 -.14
