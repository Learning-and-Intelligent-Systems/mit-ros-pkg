#include <ee_cart_imped_action/ee_cart_imped_arm.hh>


int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_action_test");
  
  EECartImpedArm arm("r_arm_cart_imped_controller");
  ee_cart_imped_control::EECartImpedGoal square;
  //startx,y w/respect to torso link
  double startx = 0.65, starty = 0;


  int draw_time = 0;
  //double pause_height = -.28;
  //double table_height = -.31;
  double pause_height = -.06;
  double table_height = -.09;
  double down_force = -3;
  double vertline_force = -4.5;
  double text_height = 0.03;
  double text_width = 0.06;
  double between_letter_space = 0.02;
  double scale = 0.5;

  // get the pen to the starting point and lower to the surface
// goal,  
//[ x, y, z, quaternion] of right gripper frame in torso link frame
// impedance or force in x,y,z ax,ay,az 
// boolean impedance=0/force=1  impedance is how hard it will try to get to a certain point
//if doing impedance, don't put it rotation over 100
//also, don't switch force->impedance except by adding point at same location with zero impedance
//example in draw_mit
// time from start of trajectory to this point
  EECartImpedArm::addTrajectoryPoint(square, startx, starty, pause_height, 0, 1, 0, 1,
				     1000, 1000, 1000, 100, 100, 100, false,
				     false, false, false, false, false, (draw_time+=6));
  EECartImpedArm::addTrajectoryPoint(square, startx, starty, -1,  0, 1, 0, 1,
				     1000, 1000, down_force*scale, 100, 100, 100, false,
				     false, true, false, false, false, (draw_time+=4));

  // draw the square
  EECartImpedArm::addTrajectoryPoint(square, startx + text_height, starty, -1,  0, 1, 0, 1,
				     1000, 1000, down_force, 100, 100, 100, false,
				     false, true, false, false, false, (draw_time+=3));
  EECartImpedArm::addTrajectoryPoint(square, startx+text_height, starty - text_height, -1,  0, 1, 0, 1,
				     1000, 1000, down_force, 100, 100, 100, false,
				     false, true, false, false, false, (draw_time+=3));
  EECartImpedArm::addTrajectoryPoint(square, startx, starty - text_height, -1,  0, 1, 0, 1,
				     1000, 1000, vertline_force, 100, 100, 100, false,
				     false, true, false, false, false, (draw_time+=3));
  EECartImpedArm::addTrajectoryPoint(square, startx, starty, -1,  0, 1, 0, 1,
				     1000, 1000, down_force, 100, 100, 100, false,
				     false, true, false, false, false, (draw_time+=3));
  EECartImpedArm::addTrajectoryPoint(square, startx + text_height, starty, -1,  0, 1, 0, 1,
				     1000, 1000, down_force, 100, 100, 100, false,
				     false, true, false, false, false, (draw_time+=3));
  EECartImpedArm::addTrajectoryPoint(square, startx+text_height, starty - text_height, -1,  0, 1, 0, 1,
				     1000, 1000, down_force, 100, 100, 100, false,
				     false, true, false, false, false, (draw_time+=3));
  EECartImpedArm::addTrajectoryPoint(square, startx, starty - text_height, -1,  0, 1, 0, 1,
				     1000, 1000, vertline_force, 100, 100, 100, false,
				     false, true, false, false, false, (draw_time+=3));
  EECartImpedArm::addTrajectoryPoint(square, startx, starty, -1,  0, 1, 0, 1,
				     1000, 1000, down_force, 100, 100, 100, false,
				     false, true, false, false, false, (draw_time+=3));
  arm.startTrajectory(square);
}
//- Translation: [0.512, -0.153, -0.331]
// Translation: [0.595, -0.152, -0.338]
// Translation: [0.553, -0.105, -0.336]
//.63 -.14
