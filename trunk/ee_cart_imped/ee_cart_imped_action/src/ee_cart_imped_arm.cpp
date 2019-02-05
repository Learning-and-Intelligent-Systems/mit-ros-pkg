//For detailed comments, see the accompanying .hh file
#include <ee_cart_imped_action/ee_cart_imped_arm.hh>

EECartImpedArm::EECartImpedArm(std::string ns) {
  traj_client_ = new EECartImpedClient(ns+"/ee_cart_imped_action", true);
  while (ros::ok() && !traj_client_->waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the ee_cart_imped_action server");
  }
}

EECartImpedArm::~EECartImpedArm() {
  delete traj_client_;
}

void EECartImpedArm::startTrajectory(ee_cart_imped_msgs::EECartImpedGoal 
				     goal, bool wait) {
  goal.header.stamp = ros::Time::now();
  traj_client_->sendGoal(goal);
  if (wait) {
    bool finishedwithintime = traj_client_->waitForResult(ros::Duration(200.0));
    if (!finishedwithintime) {
      traj_client_->cancelGoal();
      ROS_WARN("ee_cart_imped_action: Timed out while attempting to achieve goal");
    } else {
      actionlib::SimpleClientGoalState state = traj_client_->getState();
      if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
	ROS_INFO("ee_cart_imped_action: Successfully performed trajectory.");
      } else {
	ROS_WARN("ee_cart_imped_action: Failed to complete trajectory");
      }
    }
  }
}

void EECartImpedArm::stopTrajectory() {
  traj_client_->cancelAllGoals();
}

void EECartImpedArm::addTrajectoryPoint
(ee_cart_imped_msgs::EECartImpedGoal &goal, 
 double x, double y,
 double z, double ox, double oy, double oz, double ow,
 double fx, double fy, double fz, double tx,
 double ty, double tz, bool iswfx, bool iswfy,
 bool iswfz, bool iswtx, bool iswty, bool iswtz,
 double ts, std::string frame_id) {
  unsigned int new_point = goal.trajectory.size();
  goal.trajectory.resize(goal.trajectory.size()+1);
  goal.trajectory[new_point].pose.position.x = x;
  goal.trajectory[new_point].pose.position.y = y;
  goal.trajectory[new_point].pose.position.z = z;
  goal.trajectory[new_point].pose.orientation.x = ox;
  goal.trajectory[new_point].pose.orientation.y = oy;
  goal.trajectory[new_point].pose.orientation.z = oz;
  goal.trajectory[new_point].pose.orientation.w = ow;
  goal.trajectory[new_point].wrench_or_stiffness.force.x = fx;
  goal.trajectory[new_point].wrench_or_stiffness.force.y = fy; 
  goal.trajectory[new_point].wrench_or_stiffness.force.z = fz;
  goal.trajectory[new_point].wrench_or_stiffness.torque.x = tx;
  goal.trajectory[new_point].wrench_or_stiffness.torque.y = ty;
  goal.trajectory[new_point].wrench_or_stiffness.torque.z = tz;
  goal.trajectory[new_point].isForceX = iswfx;
  goal.trajectory[new_point].isForceY = iswfy;
  goal.trajectory[new_point].isForceZ = iswfz;
  goal.trajectory[new_point].isTorqueX = iswtx;
  goal.trajectory[new_point].isTorqueY = iswty;
  goal.trajectory[new_point].isTorqueZ = iswtz;
  goal.trajectory[new_point].time_from_start = ros::Duration(ts);
  goal.trajectory[new_point].header.stamp = ros::Time(0);
  goal.trajectory[new_point].header.frame_id = frame_id;
}
