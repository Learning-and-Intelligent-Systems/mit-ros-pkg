#include <ee_cart_imped_action/ee_cart_imped_arm.hh>


int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_action_test");
  
  EECartImpedArm arm("r_arm_cart_imped_controller");
  ee_cart_imped_control::EECartImpedGoal m;
  ee_cart_imped_control::EECartImpedGoal i;
  ee_cart_imped_control::EECartImpedGoal t;
  double startx = 0.65, starty = 0, width = 0.07, height = 0.05, roof = 0.03, interval = 3;


  int draw_time = 0;
  //double pause_height = -.28;
  //double table_height = -.31;
  double pause_height = -.06;
  double table_height = -.09;
  double down_force = -4;
  double vertline_force = -6;
  double text_height = 0.1;
  double text_width = 0.06;
  double between_letter_space = 0.02;
  double scale = 0.5;
  int rotimp = 75;

  // get the pen to the starting point and lower to the surface
  EECartImpedArm::addTrajectoryPoint(m, startx, starty, pause_height, 0, 1, 0, 1,
				     1000, 1000, 1000, rotimp, rotimp, rotimp, false,
				     false, false, false, false, false, (draw_time+=6));
  EECartImpedArm::addTrajectoryPoint(m, startx, starty, -1,  0, 1, 0, 1,
				     1000, 1000, down_force*scale, rotimp, rotimp, rotimp, false,
				     false, true, false, false, false, (draw_time+=4));

  // draw the M
  EECartImpedArm::addTrajectoryPoint(m, startx + text_height, starty, -1,  0, 1, 0, 1,
				     1000, 1000, down_force*scale, rotimp, rotimp, rotimp, false,
				     false, true, false, false, false, (draw_time+=3));
  EECartImpedArm::addTrajectoryPoint(m, startx + text_height/2.0, starty - text_width/2, -1,  0, 1, 0, 1,
				     1000, 1000, down_force, rotimp, rotimp, rotimp, false,
				     false, true, false, false, false, (draw_time+=2));
  EECartImpedArm::addTrajectoryPoint(m, startx + text_height, starty - text_width, -1,  0, 1, 0, 1,
				     1000, 1000, down_force, rotimp, rotimp, rotimp, false,
				     false, true, false, false, false, (draw_time+=2));
  EECartImpedArm::addTrajectoryPoint(m, startx, starty - text_width, -1,  0, 1, 0, 1,
				     1000, 1000, vertline_force, rotimp, rotimp, rotimp, false,
				     false, true, false, false, false, (draw_time+=3));
  
  // lift the pen to the pause height and move over a bit
  EECartImpedArm::addTrajectoryPoint(m, startx, starty - text_width, table_height,  0, 1, 0, 1,
				     1000, 1000, 0, rotimp, rotimp, rotimp, false,
				     false, false, false, false, false, (draw_time+=1));
  EECartImpedArm::addTrajectoryPoint(m, startx, starty - text_width, pause_height,  0, 1, 0, 1,
				     1000, 1000, rotimp, rotimp, rotimp, rotimp, false,
				     false, false, false, false, false, (draw_time+=2));
  EECartImpedArm::addTrajectoryPoint(m, startx, starty - text_width - between_letter_space, pause_height,  0, 1, 0, 1,
				     1000, 1000, rotimp, rotimp, rotimp, rotimp, false,
				     false, false, false, false, false, (draw_time+=1));
//
  //// lower pen to the surface
  //// REDEFINE STARTY
  starty = starty - text_width - between_letter_space;
  EECartImpedArm::addTrajectoryPoint(m, startx, starty, -1,  0, 1, 0, 1,
				     1000, 1000, down_force*scale, rotimp, rotimp, rotimp, false,
				     false, true, false, false, false, (draw_time+=4));
//
  //// draw the I
  EECartImpedArm::addTrajectoryPoint(m, startx + text_height*2/3.0, starty, -1,  0, 1, 0, 1,
				     1000, 1000, down_force, rotimp, rotimp, rotimp, false,
				     false, true, false, false, false, (draw_time+=2));
  //lift the pen and move up a little
  EECartImpedArm::addTrajectoryPoint(m, startx + text_height*2/3.0, starty, table_height,  0, 1, 0, 1,
				     1000, 1000, 0, rotimp, rotimp, rotimp, false,
				     false, false, false, false, false, (draw_time+=1));
  EECartImpedArm::addTrajectoryPoint(m, startx + text_height*2/3.0, starty, (pause_height+table_height)/2.0, 
				     0, 1, 0, 1,
				     1000, 1000, 100, rotimp, rotimp, rotimp, false,
				     false, false, false, false, false, (draw_time+=2));
  EECartImpedArm::addTrajectoryPoint(m, startx + text_height, starty, pause_height,  0, 1, 0, 1,
				     1000, 1000, 1000, rotimp, rotimp, rotimp, false,
				     false, false, false, false, false, (draw_time+=2));
//
  //// lower pen to the surface
  EECartImpedArm::addTrajectoryPoint(m, startx + text_height, starty, -1,  0, 1, 0, 1,
				     1000, 1000, down_force*scale*scale, rotimp, rotimp, rotimp, false,
				     false, true, false, false, false, (draw_time+=2));
  //// draw the T
  EECartImpedArm::addTrajectoryPoint(m, startx + text_height, starty - text_width*.75, -1,  0, 1, 0, 1,
				     1000, 1000, down_force, rotimp, rotimp, rotimp, false,
				     false, true, false, false, false, (draw_time+=3));
  EECartImpedArm::addTrajectoryPoint(m, startx, starty - text_width*.75, -1,  0, 1, 0, 1,
				     1000, 1000, vertline_force, rotimp, 100, 100, false,
				     false, true, false, false, false, (draw_time+=4));
//
  // lift the pen up
  //EECartImpedArm::addTrajectoryPoint(m, startx, starty - text_width, pause_height,  0, 1, 0, 1,
				     //1000, 1000, down_force, 100, 100, 100, false,
				     //false, false, false, false, false, (draw_time+=2));
//
  arm.startTrajectory(m);
}
//- Translation: [0.512, -0.153, -0.331]
// Translation: [0.595, -0.152, -0.338]
// Translation: [0.553, -0.105, -0.336]
//.63 -.14
