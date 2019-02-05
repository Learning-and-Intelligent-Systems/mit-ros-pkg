#include <ee_cart_imped_action/ee_cart_imped_arm.hh>


int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_action_test");
  
  EECartImpedArm arm("r_arm_cart_imped_controller");
  ee_cart_imped_control::EECartImpedGoal house;
  double startx = 0.65, starty = 0, width = 0.07, height = 0.05, roof = 0.03, interval = 3;


  int draw_time = 0;
  EECartImpedArm::addTrajectoryPoint(house, startx, starty, 0.2, 0, 1, 0, 1,
				     1000, 1000, 1000, 100, 100, 100, false,
				     false, false, false, false, false, (draw_time+=6));
  EECartImpedArm::addTrajectoryPoint(house, startx, starty, 0, 0, 1, 0, 1,
				     1000, 1000, 1000, 100, 100, 100, false,
				     false, false, false, false, false, (draw_time+=2));
  EECartImpedArm::addTrajectoryPoint(house, startx, starty, -1, 0, 1, 0, 1,
				     1000, 1000, -3, 100, 100, 100, false,
				     false, true, false, false, false, 
				     (draw_time+=interval));
  EECartImpedArm::addTrajectoryPoint(house, startx+width, starty, -1, 0, 1, 0, 1,
				     1000, 1000, -3, 100, 100, 100, false,
				     false, true, false, false, false, 
				     (draw_time+=interval));
  EECartImpedArm::addTrajectoryPoint(house, startx+width, starty+height, -1, 0, 1, 0, 1,
				     1000, 1000, -3, 100, 100, 100, false,
				     false, true, false, false, false, 
				     (draw_time+=interval));
  EECartImpedArm::addTrajectoryPoint(house, startx + width/2.0, starty+height+roof, 
				     -1, 0, 1, 0, 1,
				     1000, 1000, -3, 100, 100, 100, false,
				     false, true, false, false, false, 
				     (draw_time+=interval));
  EECartImpedArm::addTrajectoryPoint(house, startx, starty+height, -1, 0, 1, 0, 1,
				     1000, 1000, -3, 100, 100, 100, false,
				     false, true, false, false, false, 
				     (draw_time+=interval));
  EECartImpedArm::addTrajectoryPoint(house, startx+width, starty+height, -1, 0, 1, 0, 1,
				     1000, 1000, -3, 100, 100, 100, false,
				     false, true, false, false, false, 
				     (draw_time+=interval));
  EECartImpedArm::addTrajectoryPoint(house, startx, starty+height, -1, 0, 1, 0, 1,
				     1000, 1000, -3, 100, 100, 100, false,
				     false, true, false, false, false, 
				     (draw_time+=interval));
  EECartImpedArm::addTrajectoryPoint(house, startx, starty, -1, 0, 1, 0, 1,
				     1000, 1000, -3, 100, 100, 100, false,
				     false, true, false, false, false, 
				     (draw_time+=interval));
  arm.startTrajectory(house);
  
}
//- Translation: [0.512, -0.153, -0.331]
// Translation: [0.595, -0.152, -0.338]
// Translation: [0.553, -0.105, -0.336]
//.63 -.14
