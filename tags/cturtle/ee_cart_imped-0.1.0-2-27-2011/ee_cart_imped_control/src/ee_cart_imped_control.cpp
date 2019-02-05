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
    return (time - startTime) * (endValue - startValue) / 
      (endTime - startTime) + startValue; 
}

ee_cart_imped_control::StiffPoint 
EECartImpedControlClass::sampleInterpolation() {
  // time from the start of the series of points
  double timeFromStart = robot_state_->getTime().toSec() - 
    current_goal_start_time_.toSec();  
    ee_cart_imped_control::StiffPoint next_point;
    
    // have we reached the next point?
    if (timeFromStart > 
	desiredPoses_[current_goal_index_].time_from_start.toSec()) {
      if (current_goal_index_ < (signed int)actualPoses_.size()) {
	//store where we currently are
	actualPoses_[current_goal_index_].pose.position.x = x_.p(0);
	actualPoses_[current_goal_index_].pose.position.y = x_.p(1);
	actualPoses_[current_goal_index_].pose.position.z = x_.p(2);
	x_.M.GetQuaternion(actualPoses_[current_goal_index_].pose.orientation.x,
			   actualPoses_[current_goal_index_].pose.orientation.y,
			   actualPoses_[current_goal_index_].pose.orientation.z,
			   actualPoses_[current_goal_index_].pose.orientation.w);
      }
      current_goal_index_++;
    }

    if (current_goal_index_ >= (signed int)desiredPoses_.size()) {
      current_goal_index_--;
      // no more goals to go to, stay where you are and don't move
      next_point = desiredPoses_[current_goal_index_];
    } else { 
	double start_pos_pos_x, start_pos_pos_y, start_pos_pos_z,
	  start_pos_ori_x, start_pos_ori_y, start_pos_ori_z, start_pos_ori_w,
	  start_ws_pos_x, start_ws_pos_y, start_ws_pos_z, start_ws_ori_x, 
	  start_ws_ori_y, start_ws_ori_z;
	if (current_goal_index_ == 0) {
	  start_pos_pos_x = x0_.p.x();
	  start_pos_pos_y = x0_.p.y();
	  start_pos_pos_z = x0_.p.z();
	  
	  x0_.M.GetQuaternion(start_pos_ori_x, start_pos_ori_y, 
			      start_pos_ori_z, start_pos_ori_w);
	  start_ws_pos_x = F0_.force.x();
	  start_ws_pos_y = F0_.force.y();
	  start_ws_pos_z = F0_.force.z();
	  start_ws_ori_x = F0_.torque.x();
	  start_ws_ori_y = F0_.torque.y();
	  start_ws_ori_z = F0_.torque.z();
	} else {	
	  start_pos_pos_x = 
	    actualPoses_[current_goal_index_-1].pose.position.x;
	  start_pos_pos_y = 
	    actualPoses_[current_goal_index_-1].pose.position.y;
	  start_pos_pos_z = 
	    actualPoses_[current_goal_index_-1].pose.position.z;
	  start_pos_ori_x = 
	    actualPoses_[current_goal_index_-1].pose.orientation.x;
	  start_pos_ori_y = 
	    actualPoses_[current_goal_index_-1].pose.orientation.y;
	  start_pos_ori_z = 
	    actualPoses_[current_goal_index_-1].pose.orientation.z;
	  start_pos_ori_w = 
	    actualPoses_[current_goal_index_-1].pose.orientation.w;
	  start_ws_pos_x = 
	    desiredPoses_[current_goal_index_-1].wrench_or_stiffness.force.x;
	  start_ws_pos_y = 
	    desiredPoses_[current_goal_index_-1].wrench_or_stiffness.force.y;
	  start_ws_pos_z = 
	    desiredPoses_[current_goal_index_-1].wrench_or_stiffness.force.z;
	  start_ws_ori_x = 
	    desiredPoses_[current_goal_index_-1].wrench_or_stiffness.torque.x;
	  start_ws_ori_y = 
	    desiredPoses_[current_goal_index_-1].wrench_or_stiffness.torque.y;
	  start_ws_ori_z = 
	    desiredPoses_[current_goal_index_-1].wrench_or_stiffness.torque.z;
	}
	
        double end_pos_pos_x = 
	  desiredPoses_[current_goal_index_].pose.position.x;
        double end_pos_pos_y = 
	  desiredPoses_[current_goal_index_].pose.position.y;
        double end_pos_pos_z = 
	  desiredPoses_[current_goal_index_].pose.position.z;
        double end_pos_ori_x = 
	  desiredPoses_[current_goal_index_].pose.orientation.x;
        double end_pos_ori_y = 
	  desiredPoses_[current_goal_index_].pose.orientation.y;
        double end_pos_ori_z = 
	  desiredPoses_[current_goal_index_].pose.orientation.z;
        double end_pos_ori_w = 
	  desiredPoses_[current_goal_index_].pose.orientation.w;
	
	double segStartTime = 0;
	if (current_goal_index_ > 0) {
	  segStartTime = 
	    desiredPoses_[current_goal_index_-1].time_from_start.toSec();
	}
        double segEndTime = 
	  desiredPoses_[current_goal_index_].time_from_start.toSec();

        next_point.pose.position.x = linearlyInterpolate(timeFromStart, 
							 segStartTime, 
							 segEndTime, 
							 start_pos_pos_x, 
							 end_pos_pos_x);
        next_point.pose.position.y = linearlyInterpolate(timeFromStart, 
							 segStartTime, 
							 segEndTime, 
							 start_pos_pos_y, 
							 end_pos_pos_y);
        next_point.pose.position.z = linearlyInterpolate(timeFromStart, 
							 segStartTime, 
							 segEndTime, 
							 start_pos_pos_z, 
							 end_pos_pos_z);
        next_point.pose.orientation.x = linearlyInterpolate(timeFromStart, 
							    segStartTime, 
							    segEndTime, 
							    start_pos_ori_x, 
							    end_pos_ori_x); 
        next_point.pose.orientation.y = linearlyInterpolate(timeFromStart, 
							    segStartTime, 
							    segEndTime, 
							    start_pos_ori_y, 
							    end_pos_ori_y); 
        next_point.pose.orientation.z = linearlyInterpolate(timeFromStart, 
							    segStartTime, 
							    segEndTime, 
							    start_pos_ori_z, 
							    end_pos_ori_z); 
        next_point.pose.orientation.w = linearlyInterpolate(timeFromStart, 
							    segStartTime, 
							    segEndTime, 
							    start_pos_ori_w, 
							    end_pos_ori_w); 
	//we don't currently interpolate between wrench
	//and stiffness as generally the user wants one
	//wrench through a full trajectory point and then
	//another at the next trajectory point
	//perhaps, however, we should put in some interpolation
	//to avoid fast transitions between very different wrenches
	//as these can lead to bad behavior
        next_point.wrench_or_stiffness = 
	  desiredPoses_[current_goal_index_].wrench_or_stiffness;
        next_point.is_wrench = desiredPoses_[current_goal_index_].is_wrench;
        next_point.time_from_start = ros::Duration(timeFromStart);
    }
    return next_point;
}

void EECartImpedControlClass::commandCB
(const ee_cart_imped_control::EECartImpedGoalConstPtr &msg) {
  //should cancel the currently active goal
  desiredPoses_ = msg->trajectory;
  //reset the initial position
  x0_.p = x_.p;
  x0_.M = x_.M;
  F0_ = F_;
  if (desiredPoses_.empty()) {
    //stop the controller
    starting();
  } else {
    //this is a new goal
    current_goal_start_time_ = ros::Time::now();
    current_goal_index_ = 0;
    actualPoses_.resize(desiredPoses_.size());
  }
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

    // Store the robot handle for later use (to get time)
    robot_state_ = robot;

    // Construct the kdl solvers in non-realtime
    chain_.toKDL(kdl_chain_);
    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

    // Resize (pre-allocate) the variables in non-realtime
    q_.resize(kdl_chain_.getNrOfJoints());
    q0_.resize(kdl_chain_.getNrOfJoints());
    qdot_.resize(kdl_chain_.getNrOfJoints());
    tau_.resize(kdl_chain_.getNrOfJoints());
    tau_act_.resize(kdl_chain_.getNrOfJoints());
    J_.resize(kdl_chain_.getNrOfJoints());

    subscriber_ = node_.subscribe("command", 1, &EECartImpedControlClass::commandCB, this);
    controller_state_publisher_.reset
      (new realtime_tools::RealtimePublisher
       <ee_cart_imped_control::EECartImpedFeedback>
       (node_, "state", 1));
    updates_ = 0;
    
    wrenchfalse_.resize(6);
    for (int i = 0; i < 6; i++) {
      wrenchfalse_[i] = false;
    }
    
    Kd_.vel(0) = 0.0;        // Translation x                                                   
    Kd_.vel(1) = 0.0;        // Translation y
    Kd_.vel(2) = 0.0;        // Translation z
    Kd_.rot(0) = 0.0;        // Rotation x
    Kd_.rot(1) = 0.0;        // Rotation y
    Kd_.rot(2) = 0.0;        // Rotation z
    return true;
}

void EECartImpedControlClass::starting()
{
    // Get the current joint values to compute the initial tip location.
    chain_.getPositions(q0_);
    jnt_to_pose_solver_->JntToCart(q0_, x0_);

    // Also reset the time-of-last-servo-cycle
    last_time_ = robot_state_->getTime();
    
    desiredPoses_.resize(1);
    actualPoses_.resize(1);

    desiredPoses_[0].pose.position.x = x0_.p(0);
    desiredPoses_[0].pose.position.y = x0_.p(1);
    desiredPoses_[0].pose.position.z = x0_.p(2);
    x0_.M.GetQuaternion((desiredPoses_[0].pose.orientation.x), (desiredPoses_[0].pose.orientation.y), (desiredPoses_[0].pose.orientation.z), (desiredPoses_[0].pose.orientation.w));
    desiredPoses_[0].wrench_or_stiffness.force.x = MAX_STIFFNESS;
    desiredPoses_[0].wrench_or_stiffness.force.y = MAX_STIFFNESS;
    desiredPoses_[0].wrench_or_stiffness.force.z = MAX_STIFFNESS;
    desiredPoses_[0].wrench_or_stiffness.torque.x = ACCEPTABLE_ROT_STIFFNESS;
    desiredPoses_[0].wrench_or_stiffness.torque.y = ACCEPTABLE_ROT_STIFFNESS;
    desiredPoses_[0].wrench_or_stiffness.torque.z = ACCEPTABLE_ROT_STIFFNESS;
    F0_.force.x(MAX_STIFFNESS);
    F0_.force.y(MAX_STIFFNESS);
    F0_.force.z(MAX_STIFFNESS);
    F0_.torque.x(ACCEPTABLE_ROT_STIFFNESS);
    F0_.torque.y(ACCEPTABLE_ROT_STIFFNESS);
    F0_.torque.z(ACCEPTABLE_ROT_STIFFNESS);
    desiredPoses_[0].is_wrench = wrenchfalse_;
    desiredPoses_[0].time_from_start = ros::Duration(0);
    current_goal_start_time_ = ros::Time::now();
    current_goal_index_ = 0;
}

void EECartImpedControlClass::update()
{
    double dt;                    // Servo loop time step

    // Calculate the dt between servo cycles
    dt = (robot_state_->getTime() - last_time_).toSec();
    last_time_ = robot_state_->getTime();
    //current_goal_start_time_ += ros::Duration(dt);

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

    ee_cart_imped_control::StiffPoint desiredPose_ = sampleInterpolation();

    Fdes_(0) = desiredPose_.wrench_or_stiffness.force.x;
    Fdes_(1) = desiredPose_.wrench_or_stiffness.force.y;
    Fdes_(2) = desiredPose_.wrench_or_stiffness.force.z;
    Fdes_(3) = desiredPose_.wrench_or_stiffness.torque.x;
    Fdes_(4) = desiredPose_.wrench_or_stiffness.torque.y;
    Fdes_(5) = desiredPose_.wrench_or_stiffness.torque.z;

    Kp_.vel(0) = desiredPose_.wrench_or_stiffness.force.x;
    Kp_.vel(1) = desiredPose_.wrench_or_stiffness.force.y;
    Kp_.vel(2) = desiredPose_.wrench_or_stiffness.force.z;
    Kp_.rot(0) = desiredPose_.wrench_or_stiffness.torque.x;
    Kp_.rot(1) = desiredPose_.wrench_or_stiffness.torque.y;
    Kp_.rot(2) = desiredPose_.wrench_or_stiffness.torque.z;

    xd_.p(0) = desiredPose_.pose.position.x;
    xd_.p(1) = desiredPose_.pose.position.y;
    xd_.p(2) = desiredPose_.pose.position.z;
    xd_.M = KDL::Rotation::Quaternion(desiredPose_.pose.orientation.x, desiredPose_.pose.orientation.y, desiredPose_.pose.orientation.z, desiredPose_.pose.orientation.w);

    // Calculate a Cartesian restoring force.
    xerr_.vel = x_.p - xd_.p;
    xerr_.rot = 0.5 * (xd_.M.UnitX() * x_.M.UnitX() +
            xd_.M.UnitY() * x_.M.UnitY() +
            xd_.M.UnitZ() * x_.M.UnitZ());  // I have no idea what this is


    // F_ is a vector of forces/wrenches corresponding to x, y, z, tx,ty,tz,tw
    for (unsigned int i = 0 ; i < 6 ; i++) {
      F_(i) = (!desiredPose_.is_wrench[i]) ? 
	(- Kp_(i) * xerr_(i) - Kd_(i)*xdot_(i)): Fdes_(i);
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
	  desiredPose_;
	controller_state_publisher_->msg_.actual_pose.position.x = x_.p.x();
	controller_state_publisher_->msg_.actual_pose.position.y = x_.p.y();
	controller_state_publisher_->msg_.actual_pose.position.z = x_.p.z();
	x_.M.GetQuaternion
	  (controller_state_publisher_->msg_.actual_pose.orientation.x,
	   controller_state_publisher_->msg_.actual_pose.orientation.y,
	   controller_state_publisher_->msg_.actual_pose.orientation.z,
	   controller_state_publisher_->msg_.actual_pose.orientation.w);
	chain_.getEfforts(tau_act_);
	double eff_err = 0;
	for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
	  eff_err += (tau_(i) - tau_act_(i))*(tau_(i) - tau_act_(i));
	}
	controller_state_publisher_->msg_.effort_sq_error = eff_err;
	controller_state_publisher_->msg_.running_time =
	  robot_state_->getTime() - current_goal_start_time_;
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

