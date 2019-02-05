#include <ee_cart_imped_action/ee_cart_imped_arm.hh>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <wiping_demo/left_arm.py>
#include <geometry_msgs/Pose.h>

typedef actionlib::SimpleActionClient
<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

int main(int argc, char **argv) {
  ros::init(argc, argv, "pr2_turn_page");

  ros::NodeHandle n;

  GripperClient gclient("l_gripper_controller/gripper_action", true);
  pr2_controllers_msgs::Pr2GripperCommandGoal goal;
  goal.command.max_effort = -1;
  while (ros::ok() && !gclient.waitForServer()) {
    ROS_INFO("Waiting for gripper server");
  }

  goal.command.position = 0.0;
  gclient.sendGoal(goal);
  gclient.waitForResult(ros::Duration(2.0));

  EECartImpedArm arm("l_arm_cart_imped_controller");
  ee_cart_imped_control::EECartImpedGoal turnpage;

  // find table
  geometry_msgs::Pose init_wrist;
  init_wrist.position.x = 0.614;
  init_wrist.position.y = 0.172;
  init_wrist.position.z = -0.125;
  init_wrist.orientation.x = -0.5;
  init_wrist.orientation.y = 0.5;
  init_wrist.orientation.z = 0.5;
  init_wrist.orientation.w = 0.5;
  geometry_msgs::Pose init_gripper;
  init_gripper.position.x = 0.614;
  init_gripper.position.y = 0.172;
  init_gripper.position.z = -0.305;
  init_gripper.orientation.x = -0.5;
  init_gripper.orientation.y = 0.5;
  init_gripper.orientation.z = 0.5;
  init_gripper.orientation.w = 0.5;
  wiper = wiping_demo.left_arm.LeftArmWiper();
  wiper.findTable(init_wrist, init_gripper, false);
  ROS_INFO("FOUND TABLE");


  int time = 0;
  // EECartImpedArm::addTrajectoryPoint(turnpage, 0.62, -.10, -0.37, 0, 1, 0, 1,
  // 				     1000, 1000, 1000, 100, 100, 100, false,
  // 				     false, false, false, false, false, (time+=4));
  EECartImpedArm::addTrajectoryPoint(turnpage, 0.0, 0.0, -1, 0, 1, 0, 1,
				     1000, 1000, -7, 100, 100, 100, false,
				     false, true, false, false, false, (time+=4));
  EECartImpedArm::addTrajectoryPoint(turnpage, 0.65, -0.03, -1, 0, 1, 0, 1,
				     1000, 1000, -3.5, 100, 100, 100, false,
				     false, true, false, false, false, (time+=2));
  EECartImpedArm::addTrajectoryPoint(turnpage, 0.65, 0.21, -1, 0, 1, 0, 1,
				     5, 1000, -2, 100, 100, 100, true,
				     false, true, false, false, false, (time += 3));
//   EECartImpedArm::addTrajectoryPoint(turnpage, 0.62, 0.1, -1, 0, 1, 0, 1,
// 				     1000, 1000, 0, 100, 100, 100, false,
// 				     false, true, false, false, false, (time += 1));
  EECartImpedArm::addTrajectoryPoint(turnpage, 0.7, 0.23, -0.3, 0, 1, 0, 1,
 				     1000, 1000, 5, 100, 100, 100, false,
 				     false, true, false, false, false, (time += 6));

  EECartImpedArm::addTrajectoryPoint(turnpage, 0.7, 0.23, 0.15, 0, 1, 0, 1,
				     1000, 1000, 1000, 100, 100, 100, false,
				     false, false, false, false, false, (time +=0));
  
   arm.startTrajectory(turnpage);

//   goal.command.position = 0.08;
//   gclient.sendGoal(goal);
//   gclient.waitForResult(ros::Duration(2.0));
//   time = 0;
//   double pencilx = 0.366, pencily = 0.007, pencilz = -0.292;

//   ee_cart_imped_control::EECartImpedGoal pickpencil;  
  
//   EECartImpedArm::addTrajectoryPoint(pickpencil, 0.7, 0.23, 0, 0, 1, 0, 1,
// 				     1000, 1000, 1000, 100, 100, 100, false,
// 				     false, false, false, false, false, (time +=2));
//   EECartImpedArm::addTrajectoryPoint(pickpencil, pencilx, pencily, pencilz+0.1, 0, 1, 
// 				     0, 1,
// 				     1000, 1000, 1000, 100, 100, 100, false,
// 				     false, false, false, false, false, (time +=6));
//   EECartImpedArm::addTrajectoryPoint(pickpencil, pencilx, pencily, pencilz, 0, 1, 0, 1,
// 				     1000, 1000, 1000, 100, 100, 100, false,
// 				     false, false, false, false, false, (time +=2));
//   arm.startTrajectory(pickpencil);
//   goal.command.position = 0.0;
//   gclient.sendGoal(goal);
//   gclient.waitForResult(ros::Duration(2.0));
  
//   ee_cart_imped_control::EECartImpedGoal house;

//   double startx = 0.6, starty = 0, width = 0.07, height = 0.05, roof = 0.03, 
//     interval = 3;

//   time = 0;
//   EECartImpedArm::addTrajectoryPoint(house, pencilx, pencily, pencilz+0.1, 0, 1, 0, 1,
// 				     1000, 1000, 1000, 100, 100, 100, false,
// 				     false, false, false, false, false, (time +=2));
//   EECartImpedArm::addTrajectoryPoint(house, startx, starty, -0.2, 0, 1, 0, 1,
// 				     1000, 1000, 1000, 100, 100, 100, false,
// 				     false, false, false, false, false, (time+=2));
//   EECartImpedArm::addTrajectoryPoint(house, startx, starty, -1, 0, 1, 0, 1,
// 				     1000, 1000, -5, 100, 100, 100, false,
// 				     false, true, false, false, false, 
// 				     (time+=interval));
//   EECartImpedArm::addTrajectoryPoint(house, startx+width, starty, -1, 0, 1, 0, 1,
// 				     1000, 1000, -3, 100, 100, 100, false,
// 				     false, true, false, false, false, 
// 				     (time+=interval));
//   EECartImpedArm::addTrajectoryPoint(house, startx+width, starty+height, -1, 0, 1, 0, 1,
// 				     1000, 1000, -3, 100, 100, 100, false,
// 				     false, true, false, false, false, 
// 				     (time+=interval));
//   EECartImpedArm::addTrajectoryPoint(house, startx + width/2.0, starty+height+roof, 
// 				     -1, 0, 1, 0, 1,
// 				     1000, 1000, -3, 100, 100, 100, false,
// 				     false, true, false, false, false, 
// 				     (time+=interval));
//   EECartImpedArm::addTrajectoryPoint(house, startx, starty+height, -1, 0, 1, 0, 1,
// 				     1000, 1000, -3, 100, 100, 100, false,
// 				     false, true, false, false, false, 
// 				     (time+=interval));
//   EECartImpedArm::addTrajectoryPoint(house, startx+width, starty+height, -1, 0, 1, 0, 1,
// 				     1000, 1000, -3, 100, 100, 100, false,
// 				     false, true, false, false, false, 
// 				     (time+=interval));
//   EECartImpedArm::addTrajectoryPoint(house, startx, starty+height, -1, 0, 1, 0, 1,
// 				     1000, 1000, -3, 100, 100, 100, false,
// 				     false, true, false, false, false, 
// 				     (time+=interval));
//   EECartImpedArm::addTrajectoryPoint(house, startx, starty, -1, 0, 1, 0, 1,
// 				     1000, 1000, -3, 100, 100, 100, false,
// 				     false, true, false, false, false, 
// 				     (time+=interval));
//  arm.startTrajectory(house);

}
