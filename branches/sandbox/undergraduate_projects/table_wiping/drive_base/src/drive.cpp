#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <drive_base/moveBase.h>
#include <drive_base/turnBase.h>

class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "cmd_vel" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  //! We will be listening to TF transforms as well
  tf::TransformListener listener_;

public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  }

  //! Drive forward a specified distance based on odometry information
  bool driveOdom(drive_base::moveBase::Request  &req,
                 drive_base::moveBase::Response &res )
  {
    double x_distance = req.x;
    double y_distance = req.y;
    double velocity = req.velocity;
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
    while (!done && nh_.ok())
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
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;
      double dist_moved = relative_transform.getOrigin().length();

      if(dist_moved > sqrt(x_distance*x_distance + y_distance*y_distance)) done = true;
    }
    if (done){
        res.success = true;
	return true;
    }
    else{
	res.success = false;
	return false;
    }
  }


bool turnOdom(drive_base::turnBase::Request  &req,
              drive_base::turnBase::Response &res)
  {
    double radians = req.theta;
    bool clockwise = req.clockwise;
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
    while (!done && nh_.ok())
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
      double angle_turned = relative_transform.getRotation().getAngle();
      if ( fabs(angle_turned) < 1.0e-2) continue;

      if ( actual_turn_axis.dot( desired_turn_axis ) < 0 ) 
        angle_turned = 2 * M_PI - angle_turned;

      if (angle_turned > radians) done = true;
    }
    if (done){
        res.success = true;
	return true;
    }
    else{
	res.success = false;
	return false;
    }
  }
 };

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh, m, n;
  RobotDriver driver(nh);
  ros::ServiceServer move_service = n.advertiseService("move_base", &RobotDriver::driveOdom, &driver);
  ros::ServiceServer turn_service = m.advertiseService("turn_base", &RobotDriver::turnOdom, &driver);

  ros::spin();

//  ros::ServiceClient move_client = n.serviceClient<drive_base::moveBase>("move_base");
//  ros::ServiceClient turn_client = m.serviceClient<drive_base::turnBase>("turn_base");

  //init the ROS node


/*  drive_base::moveBase srv;
  srv.request.x = 0.5;
  srv.request.y = 0.5;
  srv.request.velocity = 0.25;
  move_client.call(srv);
*/
}

