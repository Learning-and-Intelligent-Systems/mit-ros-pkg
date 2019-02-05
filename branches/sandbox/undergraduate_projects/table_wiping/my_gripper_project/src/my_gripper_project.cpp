#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperFindContactAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperForceServoAction.h>
#include <actionlib/client/simple_action_client.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperFindContactAction> ContactClient;
// Our Action interface type, provided as a typedef for convenience                   
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperForceServoAction> ForceClient;


class Gripper{
private:
  GripperClient* gripper_client_;  
  ContactClient* contact_client_;
  ForceClient* force_client_;

public:
  //Action client initialization
  Gripper(){

    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    gripper_client_ = new GripperClient("r_gripper_sensor_controller/gripper_action", true);
    contact_client_  = new ContactClient("r_gripper_sensor_controller/find_contact",true);
    force_client_  = new ForceClient("r_gripper_sensor_controller/force_servo",true);

    //wait for the gripper action server to come up 
    while(!gripper_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_sensor_controller/gripper_action action server to come up");
    }

    while(!contact_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_sensor_controller/find_contact action server to come up");
    }

    while(!force_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_sensor_controller/force_servo action server to come up");
    }
  }

  ~Gripper(){
    delete gripper_client_;
    delete contact_client_;
    delete force_client_;
  }

  //Open the gripper
  void open(){
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.08;
    open.command.max_effort = -1.0;
    
    ROS_INFO("Sending open goal");
    gripper_client_->sendGoal(open);
    gripper_client_->waitForResult();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper opened!");
    else
      ROS_INFO("The gripper failed to open.");
  }


  //Hold somethign with a constant force in the gripper
  void hold( double holdForce){
    pr2_gripper_sensor_msgs::PR2GripperForceServoGoal squeeze;
    squeeze.command.fingertip_force = holdForce;   // hold with X N of force
    
    ROS_INFO("Sending hold goal");
    force_client_->sendGoal(squeeze);
    force_client_->waitForResult();
    if(force_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Stable force was achieved");
    else
      ROS_INFO("Stable force was NOT achieved");
  }


  //Find two contacts and go into force control mode
  void findTwoContacts(){
    pr2_gripper_sensor_msgs::PR2GripperFindContactGoal findTwo;
    findTwo.command.contact_conditions = findTwo.command.BOTH;  // close until both fingers contact
    findTwo.command.zero_fingertip_sensors = true;   // zero fingertip sensor values before moving
 
    
    ROS_INFO("Sending find 2 contact goal");
    contact_client_->sendGoal(findTwo);
    contact_client_->waitForResult(ros::Duration(5.0));
    if(contact_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Contact found. Left: %d, Right: %d", contact_client_->getResult()->data.left_fingertip_pad_contact, 
           contact_client_->getResult()->data.right_fingertip_pad_contact);
      ROS_INFO("Contact force. Left: %f, Right: %f", contact_client_->getResult()->data.left_fingertip_pad_force, 
           contact_client_->getResult()->data.right_fingertip_pad_force);
    }
    else
      ROS_INFO("The gripper did not find a contact or could not maintain contact force.");
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_gripper");

  Gripper gripper;

  gripper.open();
  sleep(2.0);

  gripper.findTwoContacts();
  gripper.hold(10.0);   // hold with 10 N of force

  return 0;
}

