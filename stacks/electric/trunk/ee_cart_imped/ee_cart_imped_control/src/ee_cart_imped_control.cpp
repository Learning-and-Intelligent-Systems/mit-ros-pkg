//For class and function comments, see 
//include/ee_cart_imped_control/ee_cart_imped_control.hpp
//or the API docs
//
//The structure of this code borrows from the Realtime controller
//KDL tutorial and the joint trajectory spline controller

#include "ee_cart_imped_control/ee_cart_imped_control.hpp"
#include <pluginlib/class_list_macros.h>

using namespace ee_cart_imped_control_ns;

double EECartImpedControlClass::linearlyInterpolate(double time, 
						    double startTime, 
						    double endTime, 
						    double startValue, 
						    double endValue) {
  return startValue + 
    (time - startTime)*
    (endValue - startValue)/(endTime - startTime);
}

ee_cart_imped_msgs::StiffPoint 
EECartImpedControlClass::sampleInterpolation() {

  boost::shared_ptr<const EECartImpedData> desired_poses_ptr;
  desired_poses_box_.get(desired_poses_ptr);
  if (!desired_poses_ptr) {
    ROS_FATAL("ee_cart_imped_control: current trajectory was NULL!");
  }
  const EECartImpedTraj &desiredPoses = desired_poses_ptr->traj;
  const ee_cart_imped_msgs::StiffPoint &initial_point = 
    desired_poses_ptr->initial_point;
  const ros::Time &current_goal_start_time = desired_poses_ptr->starting_time;
  if (desiredPoses.size() == 0) {
    ROS_ERROR("ee_cart_imped_control: Empty trajectory");
    return last_point_;
  }
  if (last_goal_starting_time_ != current_goal_start_time.toSec()) {
    //we have never seen this goal before
    last_point_ = initial_point;
    last_point_.time_from_start = ros::Duration(0);
  }
  last_goal_starting_time_ = current_goal_start_time.toSec();

  // time from the start of the series of points
  double time = robot_state_->getTime().toSec();
  double timeFromStart = time - current_goal_start_time.toSec();  
  ee_cart_imped_msgs::StiffPoint next_point;
    

  //Find the correct trajectory segment to use
  //We don't want a current_goal_index_ because it has
  //the potential to get caught in a bad race condition
  int current_goal_index;
  for (current_goal_index = 0; 
       current_goal_index < (signed int)desiredPoses.size();
       current_goal_index++) {
    if (desiredPoses[current_goal_index].time_from_start.toSec() >= timeFromStart) {
      break;
    }
  }

  if (current_goal_index >= (signed int)desiredPoses.size()) {
    //we're done with the goal, hold the last position
    return desiredPoses[current_goal_index-1];
  }

  //did we move on to the next point?
  if (current_goal_index > 0 && last_point_.time_from_start.toSec() != 
      desiredPoses[current_goal_index-1].time_from_start.toSec()) {
    //this should be where we CURRENTLY ARE
    last_point_.pose.position.x = x_.p(0);
    last_point_.pose.position.y = x_.p(1);
    last_point_.pose.position.z = x_.p(2);
    x_.M.GetQuaternion(last_point_.pose.orientation.x,
		       last_point_.pose.orientation.y,
		       last_point_.pose.orientation.z,
		       last_point_.pose.orientation.w);
    last_point_.wrench_or_stiffness = 
      desiredPoses[current_goal_index-1].wrench_or_stiffness;
    last_point_.isForceX = desiredPoses[current_goal_index-1].isForceX;
    last_point_.isForceY = desiredPoses[current_goal_index-1].isForceY;
    last_point_.isForceZ = desiredPoses[current_goal_index-1].isForceZ;
    last_point_.isTorqueX = desiredPoses[current_goal_index-1].isTorqueX;
    last_point_.isTorqueY = desiredPoses[current_goal_index-1].isTorqueY;
    last_point_.isTorqueZ = desiredPoses[current_goal_index-1].isTorqueZ;
    last_point_.time_from_start = 
      desiredPoses[current_goal_index-1].time_from_start;
  }

  ee_cart_imped_msgs::StiffPoint start_point;
  const ee_cart_imped_msgs::StiffPoint &end_point = 
    desiredPoses[current_goal_index];
  //actually now last_point_ and initial point should be the
  //same if current_goal_index is zero
  if (current_goal_index == 0) {
    start_point = initial_point;
  } else { 
    start_point = last_point_;
  }

  double segStartTime = 0.0;
  if (current_goal_index > 0) {
    segStartTime = 
      desiredPoses[current_goal_index-1].time_from_start.toSec();
  }
  double segEndTime = 
    desiredPoses[current_goal_index].time_from_start.toSec();
  if (segEndTime <= segStartTime) {
    //just stay where we currently are
    next_point.pose.position.x = x_.p(0);
    next_point.pose.position.y = x_.p(1);
    next_point.pose.position.z = x_.p(2);
    x_.M.GetQuaternion(next_point.pose.orientation.x,
		       next_point.pose.orientation.y,
		       next_point.pose.orientation.z,
		       next_point.pose.orientation.w);

  } else {
    next_point.pose.position.x = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       start_point.pose.position.x, end_point.pose.position.x);
 
    next_point.pose.position.y = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       start_point.pose.position.y, end_point.pose.position.y); 

    next_point.pose.position.z = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       start_point.pose.position.z, end_point.pose.position.z); 

    next_point.pose.orientation.x = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       start_point.pose.orientation.x, end_point.pose.orientation.x); 

    next_point.pose.orientation.y = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       start_point.pose.orientation.y, end_point.pose.orientation.y);
 
    next_point.pose.orientation.z = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       start_point.pose.orientation.z, end_point.pose.orientation.z); 

    next_point.pose.orientation.w = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       start_point.pose.orientation.w, end_point.pose.orientation.w); 
  }
  //we don't currently interpolate between wrench
  //and stiffness as generally the user wants one
  //wrench through a full trajectory point and then
  //another at the next trajectory point
  //perhaps, however, we should put in some interpolation
  //to avoid fast transitions between very different wrenches
  //as these can lead to bad behavior
  next_point.wrench_or_stiffness = 
    desiredPoses[current_goal_index].wrench_or_stiffness;
  next_point.isForceX = desiredPoses[current_goal_index].isForceX;
  next_point.isForceY = desiredPoses[current_goal_index].isForceY;
  next_point.isForceZ = desiredPoses[current_goal_index].isForceZ;
  next_point.isTorqueX = desiredPoses[current_goal_index].isTorqueX;
  next_point.isTorqueY = desiredPoses[current_goal_index].isTorqueY;
  next_point.isTorqueZ = desiredPoses[current_goal_index].isTorqueZ;
  next_point.time_from_start = ros::Duration(timeFromStart);
  return next_point;
}

void EECartImpedControlClass::commandCB
(const ee_cart_imped_msgs::EECartImpedGoalConstPtr &msg) {
  if ((msg->trajectory).empty()) {
    //stop the controller
    starting();
    return;
  }
  //this is a new goal
  boost::shared_ptr<EECartImpedData> new_traj_ptr
    (new EECartImpedData());
  if (!new_traj_ptr) {
    ROS_ERROR("Null new trajectory.");
    starting();
    return;
  }
    
  EECartImpedData &new_traj = *new_traj_ptr;
  KDL::Frame init_pos;
  KDL::JntArray q0(kdl_chain_.getNrOfJoints());
  KDL::ChainFkSolverPos_recursive fksolver(kdl_chain_);
  //Operation is in fact const (although not listed as such)
  read_only_chain_.getPositions(q0);    
  fksolver.JntToCart(q0, init_pos);

  new_traj.initial_point.pose.position.x = init_pos.p(0);
  new_traj.initial_point.pose.position.y = init_pos.p(1);
  new_traj.initial_point.pose.position.z = init_pos.p(2);
  init_pos.M.GetQuaternion(new_traj.initial_point.pose.orientation.x,
  			   new_traj.initial_point.pose.orientation.y,
  			   new_traj.initial_point.pose.orientation.z,
  			   new_traj.initial_point.pose.orientation.w);
 		     
  for (size_t i = 0; i < msg->trajectory.size(); i++) {
    new_traj.traj.push_back(msg->trajectory[i]);
  }
  if (!new_traj_ptr) {
    ROS_ERROR("Null new trajectory after filling.");
    starting();
    return;
  }
  new_traj.starting_time = ros::Time::now();
  desired_poses_box_.set(new_traj_ptr);
}

bool EECartImpedControlClass::init(pr2_mechanism_model::RobotState *robot,
        ros::NodeHandle &n) {
    std::string root_name, tip_name;
    node_ = n;
    if (!n.getParam("root_name", root_name))
    {
        ROS_ERROR("No root name given in namespace: %s)",
                n.getNamespace().c_str());
        return false;
    }
    if (!n.getParam("tip_name", tip_name))
    {
        ROS_ERROR("No tip name given in namespace: %s)",
                n.getNamespace().c_str());
        return false;
    }

    // Construct a chain from the root to the tip and prepare the kinematics
    // Note the joints must be calibrated
    if (!chain_.init(robot, root_name, tip_name))
    {
        ROS_ERROR("EECartImpedControlClass could not use the chain from '%s' to '%s'",
                root_name.c_str(), tip_name.c_str());
        return false;
    }


    if (!read_only_chain_.init(robot, root_name, tip_name))
    {
        ROS_ERROR
	  ("EECartImpedControlClass could not use the chain from '%s' to '%s'",
	   root_name.c_str(), tip_name.c_str());
        return false;
    }

    // Store the robot handle for later use (to get time)
    robot_state_ = robot;

    // Construct the kdl solvers in non-realtime
    chain_.toKDL(kdl_chain_);
    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

    // Resize (pre-allocate) the variables in non-realtime
    q_.resize(kdl_chain_.getNrOfJoints());
    qdot_.resize(kdl_chain_.getNrOfJoints());
    tau_.resize(kdl_chain_.getNrOfJoints());
    tau_act_.resize(kdl_chain_.getNrOfJoints());
    J_.resize(kdl_chain_.getNrOfJoints());

    subscriber_ = node_.subscribe("command", 1, &EECartImpedControlClass::commandCB, this);
    controller_state_publisher_.reset
      (new realtime_tools::RealtimePublisher
       <ee_cart_imped_msgs::EECartImpedFeedback>
       (node_, "state", 1));
    controller_state_publisher_->msg_.requested_joint_efforts.resize
      (kdl_chain_.getNrOfJoints());
    controller_state_publisher_->msg_.actual_joint_efforts.resize
      (kdl_chain_.getNrOfJoints());
    updates_ = 0;
  
    
    Kd_.vel(0) = 0.0;        // Translation x                                                   
    Kd_.vel(1) = 0.0;        // Translation y
    Kd_.vel(2) = 0.0;        // Translation z
    Kd_.rot(0) = 0.0;        // Rotation x
    Kd_.rot(1) = 0.0;        // Rotation y
    Kd_.rot(2) = 0.0;        // Rotation z
    
    //Create a dummy trajectory
    boost::shared_ptr<EECartImpedData> dummy_ptr(new EECartImpedData());
    EECartImpedTraj &dummy = dummy_ptr->traj;
    dummy.resize(1);
    dummy[0].time_from_start = ros::Duration(0);
    desired_poses_box_.set(dummy_ptr);
    last_goal_starting_time_ = -1;
    return true;
}

void EECartImpedControlClass::starting() {
  // Get the current joint values to compute the initial tip location.
  KDL::Frame init_pos;
  KDL::JntArray q0(kdl_chain_.getNrOfJoints());
  KDL::ChainFkSolverPos_recursive fksolver(kdl_chain_);
  //this operation is not listed as const but is in fact
  //in the current implementation
  read_only_chain_.getPositions(q0);
  fksolver.JntToCart(q0, init_pos);


  // Also reset the time-of-last-servo-cycle
  last_time_ = robot_state_->getTime();
  ///Hold current position trajectory
  boost::shared_ptr<EECartImpedData> hold_traj_ptr(new EECartImpedData());
  if (!hold_traj_ptr) {
    ROS_ERROR("While starting, trajectory pointer was null");
    return;
  }
  EECartImpedData &hold_traj = *hold_traj_ptr;
  hold_traj.traj.resize(1);
  
  hold_traj.traj[0].pose.position.x = init_pos.p(0);
  hold_traj.traj[0].pose.position.y = init_pos.p(1);
  hold_traj.traj[0].pose.position.z = init_pos.p(2);
  init_pos.M.GetQuaternion((hold_traj.traj[0].pose.orientation.x), 
			   (hold_traj.traj[0].pose.orientation.y), 
			   (hold_traj.traj[0].pose.orientation.z), 
			   (hold_traj.traj[0].pose.orientation.w));
  hold_traj.traj[0].wrench_or_stiffness.force.x = MAX_STIFFNESS;
  hold_traj.traj[0].wrench_or_stiffness.force.y = MAX_STIFFNESS;
  hold_traj.traj[0].wrench_or_stiffness.force.z = MAX_STIFFNESS;
  hold_traj.traj[0].wrench_or_stiffness.torque.x = ACCEPTABLE_ROT_STIFFNESS;
  hold_traj.traj[0].wrench_or_stiffness.torque.y = ACCEPTABLE_ROT_STIFFNESS;
  hold_traj.traj[0].wrench_or_stiffness.torque.z = ACCEPTABLE_ROT_STIFFNESS;
  hold_traj.traj[0].isForceX = false;
  hold_traj.traj[0].isForceY = false;
  hold_traj.traj[0].isForceZ = false;
  hold_traj.traj[0].isTorqueX = false;
  hold_traj.traj[0].isTorqueY = false;
  hold_traj.traj[0].isTorqueZ = false;
  hold_traj.traj[0].time_from_start = ros::Duration(0);
  hold_traj.initial_point = hold_traj.traj[0];
  hold_traj.starting_time = ros::Time::now();
  if (!hold_traj_ptr) {
    ROS_ERROR("During starting hold trajectory was null after filling");
    return;
  }
  //Pass the trajectory through to the update loop
  desired_poses_box_.set(hold_traj_ptr);
}

void EECartImpedControlClass::update()
{
    last_time_ = robot_state_->getTime();

    // Get the current joint positions and velocities
    chain_.getPositions(q_);
    chain_.getVelocities(qdot_);

    // Compute the forward kinematics and Jacobian (at this location)
    jnt_to_pose_solver_->JntToCart(q_, x_);
    jnt_to_jac_solver_->JntToJac(q_, J_);

    for (unsigned int i = 0 ; i < 6 ; i++)
    {
        xdot_(i) = 0;
        for (unsigned int j = 0 ; j < kdl_chain_.getNrOfJoints() ; j++)
            xdot_(i) += J_(i,j) * qdot_.qdot(j);
    }

    ee_cart_imped_msgs::StiffPoint desiredPose = sampleInterpolation();
    

    Fdes_(0) = desiredPose.wrench_or_stiffness.force.x;
    Fdes_(1) = desiredPose.wrench_or_stiffness.force.y;
    Fdes_(2) = desiredPose.wrench_or_stiffness.force.z;
    Fdes_(3) = desiredPose.wrench_or_stiffness.torque.x;
    Fdes_(4) = desiredPose.wrench_or_stiffness.torque.y;
    Fdes_(5) = desiredPose.wrench_or_stiffness.torque.z;

    Kp_.vel(0) = desiredPose.wrench_or_stiffness.force.x;
    Kp_.vel(1) = desiredPose.wrench_or_stiffness.force.y;
    Kp_.vel(2) = desiredPose.wrench_or_stiffness.force.z;
    Kp_.rot(0) = desiredPose.wrench_or_stiffness.torque.x;
    Kp_.rot(1) = desiredPose.wrench_or_stiffness.torque.y;
    Kp_.rot(2) = desiredPose.wrench_or_stiffness.torque.z;

    xd_.p(0) = desiredPose.pose.position.x;
    xd_.p(1) = desiredPose.pose.position.y;
    xd_.p(2) = desiredPose.pose.position.z;
    xd_.M = KDL::Rotation::Quaternion(desiredPose.pose.orientation.x, 
				      desiredPose.pose.orientation.y, 
				      desiredPose.pose.orientation.z, 
				      desiredPose.pose.orientation.w);

    // Calculate a Cartesian restoring force.
    xerr_.vel = x_.p - xd_.p;
    xerr_.rot = 0.5 * (xd_.M.UnitX() * x_.M.UnitX() +
            xd_.M.UnitY() * x_.M.UnitY() +
            xd_.M.UnitZ() * x_.M.UnitZ());  // I have no idea what this is


    // F_ is a vector of forces/wrenches corresponding to x, y, z, tx,ty,tz,tw
    if (desiredPose.isForceX) {
      F_(0) = Fdes_(0);
    } else {
      F_(0) = -Kp_(0) * xerr_(0) - Kd_(0)*xdot_(0);
    }

    if (desiredPose.isForceY) {
      F_(1) = Fdes_(1);
    } else {
      F_(1) = -Kp_(1) * xerr_(1) - Kd_(1)*xdot_(1);
    }

    if (desiredPose.isForceZ) {
      F_(2) = Fdes_(2);
    } else {
      F_(2) = -Kp_(2) * xerr_(2) - Kd_(2)*xdot_(2);
    }

    if (desiredPose.isTorqueX) {
      F_(3) = Fdes_(3);
    } else {
      F_(3) = -Kp_(3) * xerr_(3) - Kd_(3)*xdot_(3);
    }

    if (desiredPose.isTorqueY) {
      F_(4) = Fdes_(4);
    } else {
      F_(4) = -Kp_(4) * xerr_(4) - Kd_(4)*xdot_(4);
    }

    if (desiredPose.isTorqueZ) {
      F_(5) = Fdes_(5);
    } else {
      F_(5) = -Kp_(5) * xerr_(5) - Kd_(5)*xdot_(5);
    }

    // Convert the force into a set of joint torques
    // tau_ is a vector of joint torques q1...qn
    for (unsigned int i = 0 ; i < kdl_chain_.getNrOfJoints() ; i++) {
        // iterate through the vector.  every joint torque is contributed to
        // by the Jacobian Transpose (note the index switching in J access) times
        // the desired force (from impedance OR explicit force)

        // So if a desired end effector wrench is specified, there is no position control on that dof
        // if a wrench is not specified, then there is impedance based (basically p-gain) 
      //position control on that dof
        tau_(i) = 0;
        for (unsigned int j = 0 ; j < 6 ; j++) {
            tau_(i) += J_(j,i) * F_(j);
        }
    }

    // And finally send these torques out
    chain_.setEfforts(tau_);

    //publish the current state
    if (!(updates_ % 10)) {
      if (controller_state_publisher_ && 
	  controller_state_publisher_->trylock()) {
	controller_state_publisher_->msg_.header.stamp = last_time_;
	controller_state_publisher_->msg_.desired = 
	  desiredPose;
	controller_state_publisher_->msg_.actual_pose.pose.position.x = 
	  x_.p.x();
	controller_state_publisher_->msg_.actual_pose.pose.position.y = 
	  x_.p.y();
	controller_state_publisher_->msg_.actual_pose.pose.position.z = 
	  x_.p.z();
	x_.M.GetQuaternion
	  (controller_state_publisher_->msg_.actual_pose.pose.orientation.x,
	   controller_state_publisher_->msg_.actual_pose.pose.orientation.y,
	   controller_state_publisher_->msg_.actual_pose.pose.orientation.z,
	   controller_state_publisher_->msg_.actual_pose.pose.orientation.w);
	controller_state_publisher_->msg_.actual_pose.
	  wrench_or_stiffness.force.x = F_(0);
	controller_state_publisher_->msg_.actual_pose.
	  wrench_or_stiffness.force.y = F_(1);
	controller_state_publisher_->msg_.actual_pose.
	  wrench_or_stiffness.force.z = F_(2);
	controller_state_publisher_->msg_.actual_pose.
	  wrench_or_stiffness.torque.x = F_(3);
	controller_state_publisher_->msg_.actual_pose.
	  wrench_or_stiffness.torque.y = F_(4);
	controller_state_publisher_->msg_.actual_pose.
	  wrench_or_stiffness.torque.z = F_(5);
	chain_.getEfforts(tau_act_);
	double eff_err = 0;
	for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
	  eff_err += (tau_(i) - tau_act_(i))*(tau_(i) - tau_act_(i));
	  controller_state_publisher_->msg_.requested_joint_efforts[i] = 
	    tau_(i);
	  controller_state_publisher_->msg_.actual_joint_efforts[i] = 
	    tau_act_(i);
	}
	controller_state_publisher_->msg_.effort_sq_error = eff_err;
	boost::shared_ptr<const EECartImpedData> desired_poses_ptr;
	desired_poses_box_.get(desired_poses_ptr);
	controller_state_publisher_->msg_.goal = desired_poses_ptr->traj;
	controller_state_publisher_->msg_.initial_point = last_point_;
	controller_state_publisher_->msg_.running_time =
	  robot_state_->getTime() - desired_poses_ptr->starting_time;
	controller_state_publisher_->unlockAndPublish();
      }
    }
    updates_++;
}

void EECartImpedControlClass::stopping() {
  starting();
}


/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(ee_cart_imped_control, EECartImpedControlPlugin,
			ee_cart_imped_control_ns::EECartImpedControlClass,
			pr2_controller_interface::Controller)

