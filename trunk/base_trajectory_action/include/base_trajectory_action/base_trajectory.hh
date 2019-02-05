#ifndef __BASE_TRAJECTORY_ACTION_BASE_TRAJECTORY_HH__
#define __BASE_TRAJECTORY_ACTION_BASE_TRAJECTORY_HH__

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>
#include "base_trajectory_action/BaseTrajectoryAction.h"
#include "base_trajectory_action/transform_ros_types.hh"

namespace base_trajectory_action {

  const double DEFAULT_MAXIMUM_LINEAR_VELOCITY = 0.7;
  const double DEFAULT_MAXIMUM_ANGULAR_VELOCITY = MATH_PI/2.0;
  const double DEFAULT_LINEAR_ERROR = 0.05;
  const double DEFAULT_ANGULAR_ERROR = 0.05;
  const double DEFAULT_TIME_ERROR = 5.0;

  class BaseTrajectory {
  public:
    BaseTrajectory(ros::NodeHandle &n);
    void goalCB(const BaseTrajectoryGoalConstPtr &goal);
    void preemptCB();
    void stopBase();

  protected:
    ros::NodeHandle node_;
    actionlib::SimpleActionServer<BaseTrajectoryAction> action_server_;
    ros::Publisher vel_pub_;
    double max_lin_, max_ang_;
    tf::TransformListener tf_listener_;

  };
}

#endif //base_trajectory.hh
