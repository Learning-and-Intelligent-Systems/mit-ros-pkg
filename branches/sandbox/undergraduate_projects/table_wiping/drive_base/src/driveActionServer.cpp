#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <drive_base/moveBase.h>
#include <actionlib/server/simple_action_server.h>
#include <drive_base/moveBaseAction.h>
#include <drive_base/turnBaseAction.h>

class DriveBaseAction
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh2_;
  ros::NodeHandle nh3_;

  actionlib::SimpleActionServer<drive_base::moveBaseAction> moveAS_;
  actionlib::SimpleActionServer<drive_base::turnBaseAction> turnAS_;
  std::string action_name_;
  ros::Publisher cmd_vel_pub_;

  drive_base::moveBaseFeedback move_feedback_;
  drive_base::moveBaseResult move_result_;
  drive_base::turnBaseFeedback turn_feedback_;
  drive_base::turnBaseResult turn_result_;

  tf::TransformListener listener_;

public:
  DriveBaseAction(std::string name) :
    moveAS_(nh_, "move_base_action", boost::bind(&DriveBaseAction::executeMoveCB, this, _1), false),
    turnAS_(nh2_,"turn_base_action", boost::bind(&DriveBaseAction::executeTurnCB, this, _1), false),
    action_name_(name)
  {
    moveAS_.start();
    turnAS_.start();
    cmd_vel_pub_ = nh3_.advertise<geometry_msgs::Twist>("base_controller/command", 1);
  }

  ~DriveBaseAction(void)
  {
  }
  bool executeMoveCB(const drive_base::moveBaseGoalConstPtr &goal)
  {
    float x_distance = goal->goal_x;
    float y_distance = goal->goal_y;
    float velocity = goal->velocity;
    //wait for the listener to get the first message
    listener_.waitForTransform("base_footprint", "odom_combined", 
                               ros::Time(0), ros::Duration(2.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener_.lookupTransform("base_footprint", "odom_combined", 
                              ros::Time(0), start_transform);
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to go forward at velocity m/s
    int x_sign, y_sign;
    base_cmd.angular.z = 0;

    if(x_distance < 0) x_sign = -1;
    else x_sign = 1;
    if(y_distance < 0) y_sign = -1;
    else y_sign = 1;

    if(x_distance == 0) base_cmd.linear.y = velocity;
    else{
    base_cmd.linear.y = abs(y_distance/x_distance);
    }

    if(y_distance == 0) base_cmd.linear.x = velocity;
    else{
    base_cmd.linear.x = abs(x_distance/y_distance);
    }

    base_cmd.linear.y = (base_cmd.linear.y/(base_cmd.linear.y+base_cmd.linear.x));
    base_cmd.linear.x = x_sign*velocity*(base_cmd.linear.x/(base_cmd.linear.y+base_cmd.linear.x));
    base_cmd.linear.y *= y_sign;
    base_cmd.linear.y *= velocity;
    ros::Rate rate(10.0);
    bool done = false;
    bool preempted = false;
    while (!done && nh_.ok() && ros::ok()&& !preempted)
    {
      //send the drive command
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform("base_footprint", "odom_combined", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      //see how far we've traveled
      tf::Transform relative_transform = start_transform.inverse() * current_transform;
      float dist_moved = relative_transform.getOrigin().length();
      move_feedback_.distance = dist_moved;
      moveAS_.publishFeedback(move_feedback_);
      if(dist_moved > sqrt(x_distance*x_distance + y_distance*y_distance)) done = true;
      if (moveAS_.isPreemptRequested() ){
	done = false;
	preempted = true;
	}
    }
    if (done){
	move_result_.success = true;
	ROS_INFO("%s: Succeeded", action_name_.c_str());
        moveAS_.setSucceeded(move_result_);
	return true;
    }
    else{
	ROS_INFO("%s: Preempted", action_name_.c_str());
	moveAS_.setPreempted();
	return false;
    }
  }


bool executeTurnCB(const drive_base::turnBaseGoalConstPtr &goal)
  {
    float radians = goal->goal_theta;
    bool clockwise = goal->clockwise;
    while(radians < 0) radians += 2*M_PI;
    while(radians > 2*M_PI) radians -= 2*M_PI;

    //wait for the listener to get the first message
    listener_.waitForTransform("base_footprint", "odom_combined", 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener_.lookupTransform("base_footprint", "odom_combined", 
                              ros::Time(0), start_transform);
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to turn at 0.75 rad/s
    base_cmd.linear.x = base_cmd.linear.y = 0.0;
    base_cmd.angular.z = 0.25;
    if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;
    
    //the axis we want to be rotating by
    tf::Vector3 desired_turn_axis(0,0,1);
    if (!clockwise) desired_turn_axis = -desired_turn_axis;
    
    ros::Rate rate(10.0);
    bool done = false;
    bool preempted = false;
    while (!done && nh_.ok() && ros::ok() && !preempted)
    {
      //send the drive command
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform("base_footprint", "odom_combined", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;
      tf::Vector3 actual_turn_axis = 
        relative_transform.getRotation().getAxis();
      float angle_turned = relative_transform.getRotation().getAngle();
      turn_feedback_.turned = angle_turned;
      turnAS_.publishFeedback(turn_feedback_);
      if ( fabs(angle_turned) < 1.0e-2) continue;

      if ( actual_turn_axis.dot( desired_turn_axis ) < 0 ) 
        angle_turned = 2 * M_PI - angle_turned;

      if (angle_turned > radians) done = true;
      if (turnAS_.isPreemptRequested()){
	done = false;
	preempted = true;
	}
    }
    if (done){
        turn_result_.success = true;
	ROS_INFO("%s: Succeeded", action_name_.c_str());
        turnAS_.setSucceeded(turn_result_);
	return true;
    }
    else{
	ROS_INFO("%s: Preempted", action_name_.c_str());
	turnAS_.setPreempted();
	return false;
    }
  }
 };

int main(int argc, char** argv)
{
  printf("starting server");

  ros::init(argc, argv, "driveBase");

  DriveBaseAction driveBase(ros::this_node::getName());
  ros::spin();
  printf("server spinning");

  return 0; 
}

