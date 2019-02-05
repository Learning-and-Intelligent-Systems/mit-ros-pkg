//For function comments, see ee_cart_imped_action.hh
//Written by heavily borrowing from joint_trajectory_action.

#include <ee_cart_imped_action/ee_cart_imped_action.hh>

EECartImpedExecuter::EECartImpedExecuter(ros::NodeHandle &n) :
  node_(n),
  action_server_(node_, "ee_cart_imped_action",
		 boost::bind(&EECartImpedExecuter::goalCB, this, _1),
		 boost::bind(&EECartImpedExecuter::cancelCB, this, _1),
		 false),
  has_active_goal_(false) {
  
  ros::NodeHandle pn("~");
  
  //pick up all the constraints from the parameter server
  pn.param("constraints/goal/time", goal_time_constraint_, 0.0);
  pn.param("constraints/goal/position/x", goal_constraints_.position.x, -1.0);
  pn.param("constraints/goal/position/y", goal_constraints_.position.y, -1.0);
  pn.param("constraints/goal/position/z", goal_constraints_.position.z, -1.0);
  pn.param("constraints/goal/orientation/x", goal_constraints_.orientation.x, 
	   -1.0);
  pn.param("constraints/goal/orientation/y", goal_constraints_.orientation.y, 
	   -1.0);
  pn.param("constraints/goal/orientation/z", goal_constraints_.orientation.z, 
	   -1.0);
  pn.param("constraints/goal/orientation/w", goal_constraints_.orientation.w, 
	   -1.0);
  pn.param("constraints/goal/effort",  goal_effort_constraint_, -1.0);
  pn.param("constraints/trajectory/position/x", 
	   trajectory_constraints_.position.x, -1.0);
  pn.param("constraints/trajectory/position/y", 
	   trajectory_constraints_.position.y, -1.0);
  pn.param("constraints/trajectory/position/z", 
	   trajectory_constraints_.position.z, -1.0);
  pn.param("constraints/trajectory/orientation/x", 
	   trajectory_constraints_.orientation.x, -1.0);
  pn.param("constraints/trajectory/orientation/y", 
	   trajectory_constraints_.orientation.y, -1.0);
  pn.param("constraints/trajectory/orientation/z", 
	   trajectory_constraints_.orientation.z, -1.0);
  pn.param("constraints/trajectory/orientation/w", 
	   trajectory_constraints_.orientation.w, -1.0);
  pn.param("constraints/trajectory/effort", trajectory_effort_constraint_,
	   -1.0);
  pn.param("root_name", controller_frame_id_, std::string("/torso_lift_link"));
  
  pub_controller_command_ =
    node_.advertise<ee_cart_imped_msgs::EECartImpedGoal>
    ("command", 1);
  //this is published by the controller
  sub_controller_state_ =
    node_.subscribe("state", 1, 
		    &EECartImpedExecuter::controllerStateCB, this);
  watchdog_timer_ = 
    node_.createTimer(ros::Duration(1.0), &EECartImpedExecuter::watchdog, this);
  
  //is the controller started?
  ros::Time started_waiting_for_controller = ros::Time::now();
  while (ros::ok() && !last_controller_state_) {
    ros::spinOnce();
    if (started_waiting_for_controller != ros::Time(0) &&
	ros::Time::now() > started_waiting_for_controller + 
	ros::Duration(30.0)) {
      ROS_WARN("Waited for the controller for 30 seconds, but it never showed up.");
      started_waiting_for_controller = ros::Time(0);
    }
    ros::Duration(0.1).sleep();
  }
  
  action_server_.start();
  ROS_DEBUG("ee_cart_imped_action server started");
}


EECartImpedExecuter::~EECartImpedExecuter() {
    pub_controller_command_.shutdown();
    //sub_controller_state_.shutdown();
    watchdog_timer_.stop();
}

void EECartImpedExecuter::watchdog(const ros::TimerEvent &e) {
  ros::Time now = ros::Time::now();
  
  // Aborts the active goal if the controller does not appear to be active.
  if (has_active_goal_) {
    bool should_abort = false;
    if (!last_controller_state_) {
      should_abort = true;
      ROS_WARN("Aborting goal because we have never heard a controller state message.");
    } else if ((now - last_controller_state_->header.stamp) > 
	       ros::Duration(5.0)) {
      should_abort = true;
      ROS_WARN("Aborting goal because we haven't heard from the controller in %.3lf seconds",
	       (now - last_controller_state_->header.stamp).toSec());
    }
    
    if (should_abort) {
      // Stops the controller.
      ee_cart_imped_msgs::EECartImpedGoal empty;
      pub_controller_command_.publish(empty);
      
      // Marks the current goal as aborted.
      result_.success = false;
      if (last_controller_state_) {
	result_.desired = last_controller_state_->desired;
	result_.actual_pose = last_controller_state_->actual_pose.pose;
	result_.effort_sq_error = last_controller_state_->effort_sq_error;
      }
      active_goal_.setAborted(result_);
      has_active_goal_ = false;
    }
  }
}

//a new goal! so exciting
void EECartImpedExecuter::goalCB(GoalHandle gh) {

  //should probably do sqome checking to make sure
  //this is a reasonable goal
  
  // Cancels the currently active goal.
  if (has_active_goal_) {
    // Stops the controller.
    ee_cart_imped_msgs::EECartImpedGoal empty;
    pub_controller_command_.publish(empty);
    // Marks the current goal as canceled.
    result_.success = false;
    if (last_controller_state_) {
      result_.desired = last_controller_state_->desired;
      result_.actual_pose = last_controller_state_->actual_pose.pose;
      result_.effort_sq_error = last_controller_state_->effort_sq_error;
    }
    active_goal_.setCanceled(result_);
    has_active_goal_ = false;
  }
  
  current_traj_ = *(gh.getGoal());

  //re-interpret this in the frame of the root name
  ROS_DEBUG("Checking header");
  ee_cart_imped_msgs::EECartImpedGoal transformed_traj;
  transformed_traj.header.frame_id = controller_frame_id_;
  transformed_traj.header.stamp = current_traj_.header.stamp;
  for (size_t i = 0; i < current_traj_.trajectory.size(); i++) {
    ee_cart_imped_msgs::StiffPoint &currpt = current_traj_.trajectory[i];
    if (currpt.header.frame_id.compare(controller_frame_id_) &&
	!currpt.header.frame_id.empty()) {
      ROS_DEBUG("Converting from %s to %s", currpt.header.frame_id.c_str(),
		controller_frame_id_.c_str());
      geometry_msgs::PoseStamped orig, trans;
      ee_cart_imped_msgs::StiffPoint transpt;
      transpt.header.stamp = orig.header.stamp;
      orig.header.frame_id = currpt.header.frame_id;
      orig.header.stamp = ros::Time(0);
      orig.pose = currpt.pose;
      trans.header.frame_id = controller_frame_id_;
      trans.header.stamp = ros::Time(0);
      try {
	tf_listener.transformPose(controller_frame_id_, orig, trans);
      } catch (tf::TransformException ex) {
	ROS_ERROR("ee_cart_imped_action could not transform point.  Error was %s", ex.what());
	return;
      }
      transpt.header.frame_id = trans.header.frame_id;
      transpt.pose = trans.pose;
      transpt.wrench_or_stiffness = currpt.wrench_or_stiffness;
      transpt.isForceX = currpt.isForceX;
      transpt.isForceY = currpt.isForceY;
      transpt.isForceZ = currpt.isForceZ;
      transpt.isTorqueX = currpt.isTorqueX;
      transpt.isTorqueY = currpt.isTorqueY;
      transpt.isTorqueZ = currpt.isTorqueZ;
      transpt.time_from_start = currpt.time_from_start;
      transformed_traj.trajectory.push_back(transpt);
    } else {
      ROS_DEBUG("Point frame id was %s and controller frame id is %s.  No need to transform", currpt.header.frame_id.c_str(), controller_frame_id_.c_str());
      transformed_traj.trajectory.push_back(currpt);
    }
  }
  current_traj_ = transformed_traj;
  
  gh.setAccepted();
  active_goal_ = gh;
  has_active_goal_ = true;  
  // Sends the trajectory along to the controller
  pub_controller_command_.publish(current_traj_);
}

void EECartImpedExecuter::cancelCB(GoalHandle gh) {
  if (active_goal_ == gh) {
    // Stops the controller.
    ee_cart_imped_msgs::EECartImpedGoal empty;
    pub_controller_command_.publish(empty);
    
    // Marks the current goal as canceled.
    result_.success = false;
    if (last_controller_state_) {
      result_.desired = last_controller_state_->desired;
      result_.actual_pose = last_controller_state_->actual_pose.pose;
      result_.effort_sq_error = last_controller_state_->effort_sq_error;
    }
    active_goal_.setCanceled(result_);
    has_active_goal_ = false;
  }
}

void EECartImpedExecuter::controllerStateCB
(const ee_cart_imped_msgs::EECartImpedFeedbackConstPtr &msg) {
  last_controller_state_ = msg;
  ros::Time now = ros::Time::now();
  
  if (!has_active_goal_) {
    return;
  }
  if (current_traj_.trajectory.empty()) {
    return;
  }

  active_goal_.publishFeedback(*msg);
  
  //check to make sure we're within constraints
  if (!checkConstraints(msg, trajectory_constraints_, 
			trajectory_effort_constraint_)) {
    //stop the controller
    ee_cart_imped_msgs::EECartImpedGoal empty;
    pub_controller_command_.publish(empty);
    result_.success = false;
    result_.desired = msg->desired;
    result_.actual_pose = msg->actual_pose.pose;
    result_.effort_sq_error = msg->effort_sq_error;
    active_goal_.setAborted(result_);
    has_active_goal_ = false;
    ROS_WARN("Trajectory constraints violated: aborting!");
    return;
  }
  
  ros::Time end_time = current_traj_.header.stamp +
    current_traj_.trajectory[current_traj_.trajectory.size()-1].time_from_start;
  if (now >= end_time) {
    //are we at the goal?
    if (checkConstraints(msg, goal_constraints_,
			 goal_effort_constraint_)) {
      result_.success = true;
      result_.desired = msg->desired;
      result_.actual_pose = msg->actual_pose.pose;
      result_.effort_sq_error = msg->effort_sq_error;
      active_goal_.setSucceeded(result_);
      has_active_goal_ = false;
    } else if (now >= end_time + ros::Duration(goal_time_constraint_)) {
      ROS_WARN("Did not make goal position");
      result_.success = false;
      result_.desired = msg->desired;
      result_.actual_pose = msg->actual_pose.pose;
      result_.effort_sq_error = msg->effort_sq_error;
      active_goal_.setAborted(result_);
      has_active_goal_ = false;
    }
  }
}

bool EECartImpedExecuter::checkConstraints
(const ee_cart_imped_msgs::EECartImpedFeedbackConstPtr &msg,
 geometry_msgs::Pose pose_constraints, double effort_constraint) {
  if (!msg->desired.isForceX && pose_constraints.position.x >= 0 &&
      fabs(msg->actual_pose.pose.position.x - msg->desired.pose.position.x)
      > pose_constraints.position.x) {
    ROS_WARN("Violated constraint in x; actual %lf, desired %lf, constraint %lf",
	     msg->actual_pose.pose.position.x, msg->desired.pose.position.x,
	     pose_constraints.position.x);
    return false;
  }
  if (!msg->desired.isForceY && pose_constraints.position.y >= 0 &&
      fabs(msg->actual_pose.pose.position.y - msg->desired.pose.position.y)
      > pose_constraints.position.y) {
    ROS_WARN("Violated constraint in y; actual %lf, desired %lf, constraint %lf",
	     msg->actual_pose.pose.position.y, msg->desired.pose.position.y,
	     pose_constraints.position.y);
    return false;
  }
  if (!msg->desired.isForceZ && pose_constraints.position.z >= 0 &&
      fabs(msg->actual_pose.pose.position.z - msg->desired.pose.position.z) 
      > pose_constraints.position.z) {
    ROS_WARN("Violated constraint in z; actual %lf, desired %lf, constraint %lf",
	     msg->actual_pose.pose.position.z, msg->desired.pose.position.z,
	     pose_constraints.position.z);
    
    return false;
  }
  //since the quaternion is the same negative and positive
  //we check both
  bool neg_close = true, pos_close = true;
  if (!msg->desired.isTorqueY && !msg->desired.isTorqueZ &&
      pose_constraints.orientation.x >= 0 &&
      fabs(msg->actual_pose.pose.orientation.x - msg->desired.pose.orientation.x)
      > pose_constraints.orientation.x) {
    pos_close = false;
  }
  if (!msg->desired.isTorqueY && !msg->desired.isTorqueZ &&
      pose_constraints.orientation.x >= 0 &&
      fabs(msg->actual_pose.pose.orientation.x + msg->desired.pose.orientation.x)
      > pose_constraints.orientation.x) {
    neg_close = false;
  }
  if (!msg->desired.isTorqueX && !msg->desired.isTorqueZ &&
      pose_constraints.orientation.y >= 0 &&
      fabs(msg->actual_pose.pose.orientation.y - msg->desired.pose.orientation.y)
      > pose_constraints.orientation.y) {
    pos_close = false;
  }
  if (!msg->desired.isTorqueX && !msg->desired.isTorqueZ &&
      pose_constraints.orientation.y >= 0 &&
      fabs(msg->actual_pose.pose.orientation.y + msg->desired.pose.orientation.y)
      > pose_constraints.orientation.y) {
    neg_close = false;
  }
  if (!msg->desired.isTorqueX && !msg->desired.isTorqueY &&
      pose_constraints.orientation.z >= 0 &&
      fabs(msg->actual_pose.pose.orientation.z - msg->desired.pose.orientation.z)
      > pose_constraints.orientation.z) {
    pos_close = false;
  }
  if (!msg->desired.isTorqueX && !msg->desired.isTorqueY &&
      pose_constraints.orientation.z >= 0 &&
      fabs(msg->actual_pose.pose.orientation.z + msg->desired.pose.orientation.z)
      > pose_constraints.orientation.z) {
    neg_close = false;
  }
  if (!msg->desired.isTorqueX && !msg->desired.isTorqueY &&
      !msg->desired.isTorqueZ && 
      pose_constraints.orientation.w >= 0 &&
      fabs(msg->actual_pose.pose.orientation.w - msg->desired.pose.orientation.w) 
      > pose_constraints.orientation.w) {
    pos_close = false;
  }
  if (!msg->desired.isTorqueX && !msg->desired.isTorqueY &&
      !msg->desired.isTorqueZ && 
      pose_constraints.orientation.w >= 0 &&
      fabs(msg->actual_pose.pose.orientation.w + msg->desired.pose.orientation.w) 
      > pose_constraints.orientation.w) {
    neg_close = false;
  }
  if (!neg_close && !pos_close) {
    ROS_WARN("Unable to achieve constraint in orientation.  Desired: ox = %lf, oy = %lf, oz = %lf, ow = %lf.  Actual: ox = %lf, oy = %lf, oz = %lf, ow = %lf.  Pose_Constraints: ox = %lf, oy = %lf, oz = %lf, ow = %lf", 
	     msg->desired.pose.orientation.x,
	     msg->desired.pose.orientation.y,
	     msg->desired.pose.orientation.z,
	     msg->desired.pose.orientation.w,
	     msg->actual_pose.pose.orientation.x,
	     msg->actual_pose.pose.orientation.y,
	     msg->actual_pose.pose.orientation.z,
	     msg->actual_pose.pose.orientation.w,
	     pose_constraints.orientation.x,
	     pose_constraints.orientation.y,
	     pose_constraints.orientation.z,
	     pose_constraints.orientation.w);
    return false;
  }

  //check the forces
  //if we care about them
  bool check_forces = (msg->desired.isForceX ||
		       msg->desired.isForceY ||
		       msg->desired.isForceZ ||
		       msg->desired.isTorqueX ||
		       msg->desired.isTorqueY ||
		       msg->desired.isTorqueZ);
  if (check_forces && effort_constraint >= 0 &&
      (msg->effort_sq_error > effort_constraint)) {
    ROS_WARN("Violated effort constraint; error: %lf, constraint: %lf",
	     msg->effort_sq_error, effort_constraint);
    return false;
  }
  return true;
}
  
int main(int argc, char** argv) {
  ros::init(argc, argv, "ee_cart_imped_action");
  ros::NodeHandle node;//("~");
  EECartImpedExecuter ecie(node);

  ros::spin();

  return 0;
}
