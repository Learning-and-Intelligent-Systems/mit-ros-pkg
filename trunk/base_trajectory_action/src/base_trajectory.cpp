#include "base_trajectory_action/base_trajectory.hh"
#include "base_trajectory_action/transform_ros_types.hh"
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

base_trajectory_action::BaseTrajectory::BaseTrajectory(ros::NodeHandle &n) :
  node_(n),
  action_server_(node_, std::string("base_trajectory_action"),
		 boost::bind(&BaseTrajectory::goalCB, this, _1), false) {
  action_server_.registerPreemptCallback(boost::bind(&BaseTrajectory::preemptCB, this));
  vel_pub_ = node_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
  node_.param<double>("maximum_linear_velocity", max_lin_, DEFAULT_MAXIMUM_LINEAR_VELOCITY);
  node_.param<double>("maximum_angular_velocity", max_ang_, DEFAULT_MAXIMUM_ANGULAR_VELOCITY);
  action_server_.start();
  ROS_INFO("Base trajectory action started");
}

//the problem with this is that TF and AMCL both publish too slowly
//odom publishes fast enough but it's not linked to the map

void base_trajectory_action::BaseTrajectory::goalCB(const BaseTrajectoryGoalConstPtr &goal) {
  double max_lin = goal->linear_velocity, max_ang = goal->angular_velocity, linear_error=goal->linear_error,
    angular_error=goal->angular_error, time_error=goal->duration_error;
  if (max_lin <= 0) {
    max_lin = max_lin_/2.0;
  }
  if (max_lin > max_lin_) {
    ROS_WARN("Requested linear velocity of %f, but capped at %f", 
	     max_lin, max_lin_);
    max_lin = max_lin_;
  }
  if (max_ang <= 0) {
    max_ang = max_ang_/2.0;
  }
  if (max_ang > max_ang_) {
    ROS_WARN("Requested angular velocity of %f, but capped at %f", 
	     max_ang, max_ang_);
    max_ang = max_ang_;
  }
  if (linear_error <= 0.000001) {
    linear_error = DEFAULT_LINEAR_ERROR;
  }
  if (angular_error <= 0.000001) {
    angular_error = DEFAULT_ANGULAR_ERROR;
  }
  if (time_error <= 0.0001) {
    time_error = DEFAULT_TIME_ERROR;
  }

  BaseTrajectoryResult result;

  std::string world_frame = goal->world_frame;
  std::string robot_frame = goal->robot_frame;
  ros::Time tf_time = ros::Time::now();
  if (!tf_listener_.waitForTransform(world_frame, robot_frame, tf_time, ros::Duration(2.0))) {
    ROS_ERROR("Unable to find transform from %s to %s", world_frame.c_str(), robot_frame.c_str());
    result.error_code = result.TF_ERROR;
    action_server_.setAborted(result);
    stopBase();
    return;
  }
  tf::StampedTransform robot_trans;
  tf_listener_.lookupTransform(world_frame, robot_frame, tf_time, robot_trans);
  geometry_msgs::Pose2D goal_pose;
  goal_pose.x = robot_trans.getOrigin().x();
  goal_pose.y = robot_trans.getOrigin().y();
  goal_pose.theta = tf::getYaw(robot_trans.getRotation());
  geometry_msgs::Pose2D robot_pose = goal_pose;
  ros::Time start_time = ros::Time::now();
  ros::Duration expected_duration(0.0);
  ros::Duration max_duration_error(time_error);
  int trajectory_index = -1; //current point
  double lin_vel = 0.0, ang_vel = 0.0;
  ros::Rate rate(100);
  
  BaseTrajectoryFeedback feedback;
  feedback.header.frame_id = robot_frame;
  bool new_goal = false;
  geometry_msgs::Twist command;
  command.linear.x = 0.0;
  command.linear.y = 0.0;
  command.angular.z = 0.0;
  ros::Time last_time = ros::Time::now();
  while (ros::ok() and action_server_.isActive()) {
    //find the current position of the goal if we can
    //this is likely the same pose over and over again, but if
    //it's the map frame we care
    //we should use what is published on odom to make this more
    //accurate
    std::string pose_error;
    ros::Time new_tf_time;
    unsigned int tf_error = tf_listener_.getLatestCommonTime(world_frame, robot_frame, new_tf_time, &pose_error);
    ros::Time curr_time = ros::Time::now();
    if (tf_error == tf::NO_ERROR && new_tf_time != tf_time) {
      //new feedback from TF
      tf_time = new_tf_time;
      tf_listener_.lookupTransform(world_frame, robot_frame, tf_time, robot_trans);
      robot_pose.x = robot_trans.getOrigin().x() + (curr_time - tf_time).toSec()*command.linear.x;
      robot_pose.y = robot_trans.getOrigin().y() + (curr_time - tf_time).toSec()*command.linear.y;
      robot_pose.theta = tf::getYaw(robot_trans.getRotation()) + (curr_time - tf_time).toSec()*command.angular.z;
    } else {
      robot_pose.x += (curr_time - last_time).toSec()*command.linear.x;
      robot_pose.y += (curr_time - last_time).toSec()*command.linear.y;
      robot_pose.theta += (curr_time - last_time).toSec()*command.angular.z;
    }

    //figure out the distances
    geometry_msgs::Vector3 delta;
    delta.x = goal_pose.x - robot_pose.x;
    delta.y = goal_pose.y - robot_pose.y;
    double ang_diff = wrap_angle(goal_pose.theta - robot_pose.theta);
    double ang_dist = fabs(ang_diff);
    double lin_dist = norm(delta);
    double lin_time = lin_dist/max_lin;
    double ang_time = ang_dist/max_ang;
    ros::Duration time_left(0.0);
    if (ang_time > lin_time) {
      if (ang_diff > 0) {
	ang_vel = max_ang;
      } else {
	ang_vel = -max_ang;
      }
      lin_vel = lin_dist/ang_time;
      time_left = ros::Duration(ang_time);
    } else {
      lin_vel = max_lin;
      ang_vel = ang_diff/lin_time;
      time_left = ros::Duration(lin_time);
    }
    if (new_goal) {
      expected_duration = time_left;
      new_goal = false;
    }

    if (ang_dist < angular_error && lin_dist < linear_error) {
      trajectory_index++;
      ROS_INFO("Heading for trajectory point %d", trajectory_index);
      if (trajectory_index >= (int)goal->trajectory.size()) {
	break;
      }
      goal_pose = goal->trajectory[trajectory_index];
      start_time = ros::Time::now();
      new_goal = true;
      continue;
    }

    if (curr_time - start_time > expected_duration + max_duration_error) {
      ROS_ERROR("Could not reach goal point.");
      result.error_code = result.BASE_NOT_MOVING;
      action_server_.setAborted(result);
      stopBase();
      return;
    }

    //transform the differences into the frame of the robot
    geometry_msgs::Pose robot_trans = pose2D_to_pose(robot_pose);

    if (lin_dist > 0) {
      command.linear.x = delta.x/lin_dist*lin_vel;
      command.linear.y = delta.y/lin_dist*lin_vel;
    } else {
      command.linear.x = 0.0;
      command.linear.y = 0.0;
    }
    command.linear.z = 0.0;
    command.angular.x = 0.0;
    command.angular.y = 0.0;
    command.angular.z = ang_vel;
    //convert this to the robot's frame
    geometry_msgs::Twist robot_command = command;
    robot_command.linear = inverse_transform_vector(command.linear, robot_trans);
    vel_pub_.publish(robot_command);
    last_time = ros::Time::now(); //this is when the command was published
    feedback.header.stamp = curr_time;
    feedback.waypoint = trajectory_index;
    feedback.expected_waypoint_time = expected_duration;
    feedback.waypoint_start_time = start_time;
    feedback.waypoint_time = curr_time - start_time;
    feedback.last_tf_update = tf_time;
    feedback.goal_pose = goal_pose;
    feedback.robot_pose = robot_pose;
    feedback.command = robot_command;
    action_server_.publishFeedback(feedback);

    rate.sleep();
  }
  stopBase();
  result.error_code = result.SUCCESS;
  action_server_.setSucceeded(result);
  return;
} 
  
void base_trajectory_action::BaseTrajectory::preemptCB() {
  stopBase();
}

void base_trajectory_action::BaseTrajectory::stopBase() {
  geometry_msgs::Twist command;
  command.linear.x = 0.0;
  command.linear.y = 0.0;
  command.linear.z = 0.0;
  command.angular.x = 0.0;
  command.angular.y = 0.0;
  command.angular.z = 0.0;
  vel_pub_.publish(command);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "base_trajectory_action_node");
  ros::NodeHandle n;
  base_trajectory_action::BaseTrajectory bta(n);
  ros::spin();
}
