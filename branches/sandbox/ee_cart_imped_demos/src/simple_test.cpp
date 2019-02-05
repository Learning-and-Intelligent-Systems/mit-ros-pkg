#include <ee_cart_imped_action/ee_cart_imped_action.hh>


int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_action_test");
  
  EECartImpedArm arm("r_arm_cart_imped_controller");
  
  ee_cart_imped_control::EECartImpedGoal msg;
  msg.trajectory.resize(5);
  msg.trajectory[0].pose.position.x = 0.72;
  msg.trajectory[0].pose.position.y = -0.14;
  msg.trajectory[0].pose.position.z = -1;
  msg.trajectory[0].pose.orientation.x = 0;
  msg.trajectory[0].pose.orientation.y = 1;
  msg.trajectory[0].pose.orientation.z = 0;
  msg.trajectory[0].pose.orientation.w = 1;
  msg.trajectory[0].wrench_or_stiffness.force.x = 1000;
  msg.trajectory[0].wrench_or_stiffness.force.y = 1000; 
  msg.trajectory[0].wrench_or_stiffness.force.z = -3;
  msg.trajectory[0].wrench_or_stiffness.torque.x = 100;
  msg.trajectory[0].wrench_or_stiffness.torque.y = 100;
  msg.trajectory[0].wrench_or_stiffness.torque.z = 100;
  msg.trajectory[0].is_wrench.resize(6);
  for (int i = 0; i < 6; i++) {
    msg.trajectory[0].is_wrench[i] = false;
  }
    msg.trajectory[0].is_wrench[2] = true;

  msg.trajectory[1].pose.position.x = 0.69;
  msg.trajectory[1].pose.position.y = -.13;
  msg.trajectory[1].pose.position.z = -1;
  msg.trajectory[1].pose.orientation.x = 0;
  msg.trajectory[1].pose.orientation.y = 1;
  msg.trajectory[1].pose.orientation.z = 0;
  msg.trajectory[1].pose.orientation.w = 1;
  msg.trajectory[1].wrench_or_stiffness.force.x = 1000;
  msg.trajectory[1].wrench_or_stiffness.force.y = 1000; 
  msg.trajectory[1].wrench_or_stiffness.force.z = -3;
  msg.trajectory[1].wrench_or_stiffness.torque.x = 100;
  msg.trajectory[1].wrench_or_stiffness.torque.y = 100;
  msg.trajectory[1].wrench_or_stiffness.torque.z = 100;
  msg.trajectory[1].is_wrench.resize(6);
  for (int i = 0; i < 6; i++) {
    msg.trajectory[1].is_wrench[i] = false;
  }
  msg.trajectory[1].is_wrench[2] = true;

  msg.trajectory[2].pose.position.x = 0.77;
  msg.trajectory[2].pose.position.y = -0.12;
  msg.trajectory[2].pose.position.z = -1;
  msg.trajectory[2].pose.orientation.x = 0;
  msg.trajectory[2].pose.orientation.y = 1;
  msg.trajectory[2].pose.orientation.z = 0;
  msg.trajectory[2].pose.orientation.w = 1;
  msg.trajectory[2].wrench_or_stiffness.force.x = 1000;
  msg.trajectory[2].wrench_or_stiffness.force.y = 1000; 
  msg.trajectory[2].wrench_or_stiffness.force.z = -3;
  msg.trajectory[2].wrench_or_stiffness.torque.x = 100;
  msg.trajectory[2].wrench_or_stiffness.torque.y = 100;
  msg.trajectory[2].wrench_or_stiffness.torque.z = 100;
  msg.trajectory[2].is_wrench.resize(6);
  for (int i = 0; i < 6; i++) {
    msg.trajectory[2].is_wrench[i] = false;
  }
  msg.trajectory[2].is_wrench[2] = true;

  msg.trajectory[3].pose.position.x = 0.78;
  msg.trajectory[3].pose.position.y = -0.05;
  msg.trajectory[3].pose.position.z = -1;
  msg.trajectory[3].pose.orientation.x = 0;
  msg.trajectory[3].pose.orientation.y = 1;
  msg.trajectory[3].pose.orientation.z = 0;
  msg.trajectory[3].pose.orientation.w = 1;
  msg.trajectory[3].wrench_or_stiffness.force.x = 1000;
  msg.trajectory[3].wrench_or_stiffness.force.y = 1000; 
  msg.trajectory[3].wrench_or_stiffness.force.z = -3;
  msg.trajectory[3].wrench_or_stiffness.torque.x = 100;
  msg.trajectory[3].wrench_or_stiffness.torque.y = 100;
  msg.trajectory[3].wrench_or_stiffness.torque.z = 100;
  msg.trajectory[3].is_wrench.resize(6);
  for (int i = 0; i < 6; i++) {
    msg.trajectory[3].is_wrench[i] = false;
  }
  msg.trajectory[3].is_wrench[2] = true;

  msg.trajectory[4].pose.position.x = 0.7;
  msg.trajectory[4].pose.position.y = -0.0;
  msg.trajectory[4].pose.position.z = -1;
  msg.trajectory[4].pose.orientation.x = 0;
  msg.trajectory[4].pose.orientation.y = 1;
  msg.trajectory[4].pose.orientation.z = 0;
  msg.trajectory[4].pose.orientation.w = 1;
  msg.trajectory[4].wrench_or_stiffness.force.x = 1000;
  msg.trajectory[4].wrench_or_stiffness.force.y = 1000; 
  msg.trajectory[4].wrench_or_stiffness.force.z = -3;
  msg.trajectory[4].wrench_or_stiffness.torque.x = 100;
  msg.trajectory[4].wrench_or_stiffness.torque.y = 100;
  msg.trajectory[4].wrench_or_stiffness.torque.z = 100;
  msg.trajectory[4].is_wrench.resize(6);
  for (int i = 0; i < 6; i++) {
    msg.trajectory[4].is_wrench[i] = false;
  }
  msg.trajectory[4].is_wrench[2] = true;

  msg.trajectory[0].time_from_start = ros::Duration(6);
  msg.trajectory[1].time_from_start = ros::Duration(10);
  msg.trajectory[2].time_from_start = ros::Duration(14);
  msg.trajectory[3].time_from_start = ros::Duration(18);
  msg.trajectory[4].time_from_start = ros::Duration(22);
  arm.startTrajectory(msg);
}
