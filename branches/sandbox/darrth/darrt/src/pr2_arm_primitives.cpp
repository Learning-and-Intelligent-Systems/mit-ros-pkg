
#include "darrt/object_space.hh"
#include "darrt/pr2_arm_primitives.hh"
#include "darrt/transform_ros_types.hh"

bool darrt::ArmIK::setup(DARRTStateSpace *dspace) {

  dspace_ = dspace;
  robot_space_ = dspace_->robot_state_space()->as<CompoundRobotStateSpace>();
  arm_index_ = robot_space_->getSubspaceIndex(arm_name_);
  arm_space_ = robot_space_->as<RobotChainStateSpace>(arm_index_);
  //note some transforms
  //these may change if we can move the robot base
  world_to_base_ = transformer_.get_transform(transformer_.robot_frame_id(), transformer_.world_frame_id(),
					      *(robot_space_->environment_interface()->kinematicState()));
  world_to_ik_ = transformer_.get_transform(arm_space_->root_name(), transformer_.world_frame_id(),
					    *(robot_space_->environment_interface()->kinematicState()));
  //this one never changes (since we don't move the torso)
  base_to_ik_ = transformer_.get_transform(arm_space_->root_name(), transformer_.robot_frame_id(),
					   *(robot_space_->environment_interface()->kinematicState()));
  return true;
}

ob::State *darrt::ArmIK::arm_state(ob::State *full_state) const {
  return robot_space_->get_state(arm_index_, dspace_->robot_state(full_state));
}

const ob::State *darrt::ArmIK::arm_state(const ob::State *full_state) const {
  return robot_space_->get_state(arm_index_, dspace_->robot_state(full_state));
}

gm::PoseStamped darrt::ArmIK::transform_pose_stamped
(std::string new_frame, const gm::PoseStamped &ps, const ob::State *state) const {
  //fast cheat for common headers
  new_frame = transformer_.relative_frame(new_frame);
  std::string pose_frame = transformer_.relative_frame(ps.header.frame_id);
  if (!new_frame.compare(pose_frame)) {
    return ps;
  }
  gm::PoseStamped new_pose;
  new_pose.header.frame_id = new_frame;
  if ((!new_frame.compare(transformer_.world_frame_id()) || !new_frame.compare(transformer_.robot_frame_id()) ||
       !new_frame.compare(arm_space_->root_name())) &&
      (!pose_frame.compare(transformer_.world_frame_id()) || !pose_frame.compare(transformer_.robot_frame_id()) || 
       !pose_frame.compare(arm_space_->root_name()))) {
    if (new_frame.compare(transformer_.world_frame_id()) && pose_frame.compare(transformer_.world_frame_id())) {
      //must be a transform from base to ik or vice versa
      if (new_frame.compare(transformer_.robot_frame_id())) {
	new_pose.pose = transform_pose(ps.pose, base_to_ik_.transform);
      } else {
	new_pose.pose = inverse_transform_pose(ps.pose, base_to_ik_.transform);
      }
    } else {
      //must be a transform to or from the world frame
      //find the location of the robot's base
      gm::Pose root_pose;
      try {
	unsigned int base_index = robot_space_->getSubspaceIndex("pr2_base");
	//robot can move
	const ob::SE2StateSpace::StateType *base_state = robot_space_->get_state
	  (base_index, dspace_->robot_state(state))->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0);
	gm::Pose2D base_pose2d;
	orc::SE2StateSpace_to_pose2D_msg(*base_state, base_pose2d);
	root_pose.position.x = base_pose2d.x;
	root_pose.position.y = base_pose2d.y;
	root_pose.position.z = 0;
	darrt::yaw_to_quaternion(base_pose2d.theta, root_pose.orientation);
      } catch (ompl::Exception &e) {
	//robot can't move - use original pose
	root_pose = transform_to_pose(world_to_base_.transform);
      }
      if (!pose_frame.compare(arm_space_->root_name()) || !new_frame.compare(arm_space_->root_name())) {
	//this is the pose of the "base" of the arm
	root_pose = transform_pose(transform_to_pose(base_to_ik_.transform), root_pose);
      }
      if (pose_frame.compare(transformer_.world_frame_id())) {
	new_pose.pose = transform_pose(ps.pose, root_pose);
      } else {
	new_pose.pose = inverse_transform_pose(ps.pose, root_pose);
      }
    }
    return new_pose;
  }
  
  pm::KinematicState *kstate = robot_space_->environment_interface()->kinematicState();
  robot_space_->convert_ompl_to_kinematic_state(dspace_->robot_state(state), *kstate);
  gm::PoseStamped to_return = transformer_.transform(new_frame, ps, *kstate);
  
  // if (darrt::distance(to_return.pose.position, new_pose.pose.position) > 0.01 ||
  //     darrt::distance(to_return.pose.orientation, new_pose.pose.orientation) > 0.1) {
  //   ROS_ERROR("Calculation mismatch!");
  //   ROS_INFO_STREAM("Fast calculated new pose =\n" << new_pose << "\nWhile transformed pose =\n" << to_return
  // 		    << "\nDistance: " << darrt::distance(to_return.pose.position, new_pose.pose.position)
  // 		    << "\nAngular distance: " 
  // 		    << darrt::distance(to_return.pose.orientation, new_pose.pose.orientation)
  // 		    << "\nOriginal pose =\n" << ps);
  //   std::string buff;
  //   getline(std::cin, buff);
  // }
  return to_return;
}


gm::Vector3Stamped darrt::ArmIK::transform_vector_stamped
(std::string new_frame, const gm::Vector3Stamped &vs, const ob::State *state) const {
  pm::KinematicState *kstate = robot_space_->environment_interface()->kinematicState();
  robot_space_->convert_ompl_to_kinematic_state(dspace_->robot_state(state), *kstate);
  return transformer_.transform(new_frame, vs, *kstate);
}

bool darrt::ArmIK::feasible(const ob::State *source, double distance,
			    const gm::Vector3 &direction,
			    const ob::State *seed) const {
  gm::PoseStamped ps;
  arm_space_->forward_kinematics(arm_state(source), ps);
  ps.pose.position.x += direction.x*distance;
  ps.pose.position.y += direction.y*distance;
  ps.pose.position.z += direction.z*distance;
  ob::State *ik = arm_space_->allocState();
  if (seed) {
    if (arm_space_->inverse_kinematics(ps, ik, arm_state(seed))) {
      arm_space_->freeState(ik);
      return true;
    }
  }
  if (arm_space_->inverse_kinematics(ps, ik, arm_state(source))) {
    arm_space_->freeState(ik);
    return true;
  }
  if (arm_space_->inverse_kinematics(ps, ik)) {
    arm_space_->freeState(ik);
    return true;
  }
  arm_space_->freeState(ik);
  if (debug_level >= DPROPAGATE) {
    ROS_INFO_STREAM("No IK solution for\n" << ps);
  }
  return false;
}

//in the robot frame
gm::Vector3Stamped darrt::ArmIK::translation(const ob::State *source,
					     const ob::State *destination) const {
  gm::PoseStamped sfk, dfk;
  arm_space_->forward_kinematics(arm_state(source), sfk);
  arm_space_->forward_kinematics(arm_state(destination), dfk);
  gm::PoseStamped spose = transform_pose_stamped(transformer_.world_frame_id(), sfk, source);
  gm::PoseStamped dpose = transform_pose_stamped(transformer_.world_frame_id(),sfk, destination);

  gm::Vector3Stamped delta;

  delta.vector.x = dpose.pose.position.x - spose.pose.position.x;
  delta.vector.y = dpose.pose.position.y - spose.pose.position.y;
  delta.vector.z = dpose.pose.position.z - spose.pose.position.z;
  return delta;
}

gm::Transform darrt::ArmIK::wrist_transform(const ob::State *source,
					    const ob::State *destination) const {
  gm::PoseStamped sfk, dfk;
  arm_space_->forward_kinematics(arm_state(source), sfk);
  sfk = transform_pose_stamped(transformer_.world_frame_id(), sfk, source);
  arm_space_->forward_kinematics(arm_state(destination), dfk);
  dfk = transform_pose_stamped(transformer_.world_frame_id(), dfk, destination);
  gm::Pose origin;
  origin.orientation.w = 1.0;
  return pose_to_transform(transform_pose(inverse_transform_pose(origin, sfk.pose), 
					  dfk.pose));
}

gm::Pose darrt::ArmIK::update_attached_object_position(unsigned int ind, const ob::State *source, 
						       const ob::State *destination) const {
  gm::Transform trans;
  trans = wrist_transform(source, destination);
  
  const ObjectStateSpace *ospace = 
    dspace_->getSubspace(ind)->as<ObjectStateSpace>();
  const ObjectStateSpace::StateType *os;
  os = dspace_->get_state(ind, source)->as<ObjectStateSpace::StateType>();
  
  gm::Pose objpose;
  ospace->object_position(os, objpose);
  return transform_pose(objpose, trans);
}

void darrt::ArmIK::update_attached_object_positions(const ob::State *source,
						    ob::State *result) const {
  
  gm::Transform trans;
  trans = wrist_transform(source, result);

  //update the object positions
  //if the object is attached, it moves just as the robot does
  for (size_t i = 0; i < dspace_->object_indexes().size(); i++) {
    unsigned int ind = dspace_->object_indexes().at(i);
    const ObjectStateSpace *ospace = 
      dspace_->getSubspace(ind)->as<ObjectStateSpace>();
    const ObjectStateSpace::StateType *os;
    os = dspace_->get_state(ind, source)->as<ObjectStateSpace::StateType>();
    
    gm::Pose objpose;
    ospace->object_position(os, objpose);
    if (os->attach_link.size() > 0) {
      //take all the grasp information along
      dspace_->getSubspace(ind)->copyState(dspace_->get_state(ind, result), os);
      objpose = transform_pose(objpose, trans);
    }
    ospace->set_object_position(dspace_->get_state(ind, result), objpose);
  }
}


//note: needing destination in these functions is left over from
//the inactive state set days

bool darrt::ArmIK::propagate_interpolated_ik
(const ob::State *source, const ob::State *destination, double fraction,
 ob::State *result, const ob::State *seed) const {
  gm::Vector3Stamped world_trans = translation(source, destination);
  gm::Vector3Stamped robot_trans = transform_vector_stamped(arm_space_->root_name(), 
							    world_trans, source);
  return propagate_interpolated_ik(source, destination, robot_trans.vector, fraction, result, seed);
}

bool darrt::ArmIK::propagate_interpolated_ik
(const ob::State *source, const ob::State *destination, 
 const gm::Vector3 &trans, double fraction, ob::State *result, 
 const ob::State *seed) const {
  gm::PoseStamped spose;
  arm_space_->forward_kinematics(arm_state(source), spose);
  gm::PoseStamped fpose;
  if (fraction - EPSILON >= 1) {
    fraction = 1.0;
  }
  dspace_->copyState(result, source);
  fpose.header = spose.header;
  fpose.pose.position.x = spose.pose.position.x + fraction*trans.x;
  fpose.pose.position.y = spose.pose.position.y + fraction*trans.y;
  fpose.pose.position.z = spose.pose.position.z + fraction*trans.z;
  fpose.pose.orientation = spose.pose.orientation;
  if (seed) {
    if (arm_space_->inverse_kinematics
	(fpose, arm_state(result), arm_state(seed))) {
      update_attached_object_positions(source, result);
      return true;
    }
  }
  if (arm_space_->inverse_kinematics
      (fpose, arm_state(result), arm_state(source))) {
    update_attached_object_positions(source, result);
    return true;
  }
  if (arm_space_->inverse_kinematics(fpose, arm_state(result))) {
    update_attached_object_positions(source, result);
    return true;
  }
  if (debug_level >= DPROPAGATE) {
    ROS_INFO_STREAM("Propagate interpolated IK: unable to find solution for\n" << fpose
		    << "\nStarting from\n" << spose);
  }
  return false;
}

bool darrt::ArmIK::propagate_interpolated_ik
(const ob::State *source, const gm::Transform &trans, double fraction, 
 ob::State *result, const ob::State *seed) const {
  gm::PoseStamped spose;
  arm_space_->forward_kinematics(arm_state(source), spose);
  gm::PoseStamped fpose;
  if (fraction - EPSILON >= 1) {
    fraction = 1.0;
  }
  dspace_->copyState(result, source);
  fpose.header = spose.header;
  gm::Transform ptrans;
  ptrans.translation.x = fraction*trans.translation.x;
  ptrans.translation.y = fraction*trans.translation.y;
  ptrans.translation.z = fraction*trans.translation.z;

  double total_angle = acos(trans.rotation.w)*2.0; //this is the total angle
  double angle = fraction*total_angle;
  if (total_angle > MATH_PI) {
    angle = 2.0*MATH_PI - fraction*(2.0*MATH_PI - total_angle);
  }
  double sin_ratio = 1.0;
  if (total_angle > std::numeric_limits<double>::epsilon()) {
    sin_ratio = sin(angle/2.0)/sin(total_angle/2.0);
  }
  ptrans.rotation.w = cos(angle/2.0);
  ptrans.rotation.x = trans.rotation.x*sin_ratio;
  ptrans.rotation.y = trans.rotation.y*sin_ratio;
  ptrans.rotation.z = trans.rotation.z*sin_ratio;

  fpose.pose = transform_pose(spose.pose, ptrans);
  if (debug_level == DBETWEEN) {
    ROS_INFO_STREAM("fraction = " << fraction << "fpose = " << fpose << "\n spose = " << spose <<
		    "\n total angle = " << total_angle  << " angle = " << angle << "ptrans = " <<
		    ptrans);
  }

  if (seed) {
    if (arm_space_->inverse_kinematics
	(fpose, arm_state(result), arm_state(seed))) {
      update_attached_object_positions(source, result);
      return true;
    }
  }
  if (arm_space_->inverse_kinematics
      (fpose, arm_state(result), arm_state(source))) {
    update_attached_object_positions(source, result);
    return true;
  }
  if (arm_space_->inverse_kinematics(fpose, arm_state(result))) {
    update_attached_object_positions(source, result);
    return true;
  }
  if (debug_level >= DPROPAGATE) {
    ROS_INFO_STREAM("Propagate interpolated IK: unable to find solution for\n" << fpose
		    << "\nStarting from\n" << spose << "\nfraction = " << fraction);
  }
  return false;
}


gm::PoseStamped darrt::ArmIK::propagate_grasp(const ob::State *destination,
					      const ob::State *source,
					      unsigned int objind) const {
  
  const ObjectStateSpace *ospace = 
    dspace_->object_state_space(objind)->as<ObjectStateSpace>();
  geometry_msgs::Pose opose;
  ospace->object_position(dspace_->get_state(objind, destination), opose);

  //find the wrist pose for which the object is at opose
  //grasp
  geometry_msgs::PoseStamped grps;
  arm_space_->forward_kinematics(arm_state(source), grps);
  gm::PoseStamped grpose = transform_pose_stamped(transformer_.world_frame_id(), grps,
						  source);
  gm::Pose gopose;
  ospace->object_position(dspace_->get_state(objind, source), gopose);
  gm::Pose grasp_pose = inverse_transform_pose(gopose, grpose.pose);
  gm::Pose origin;
  origin.orientation.w = 1.0;
  //inverse grasp in the frame of the object is the pose for the wrist
  gm::Pose wrist_pose = 
    transform_pose(inverse_transform_pose(origin, grasp_pose), opose);
  gm::PoseStamped wps;
  wps.header.frame_id = ospace->object().header.frame_id;
  wps.pose = wrist_pose;
  return wps;
}

ob::State *darrt::ArmIK::propagate_grasp(const ob::State *destination,
					 const ob::State *source,
					 unsigned int objind,
					 const ob::State *seed) const {
  ob::State *result = dspace_->allocState();
  if (propagate_grasp(destination, source, objind, result, seed)) {
    return result;
  }
  dspace_->freeState(result);
  return NULL;
}

bool darrt::ArmIK::propagate_grasp
(const ob::State *destination, const ob::State *source, unsigned int objind,
 ob::State *result, const ob::State *seed) const {
  const ObjectStateSpace *ospace = 
    dspace_->object_state_space(objind)->as<ObjectStateSpace>();
  geometry_msgs::Pose opose;
  ospace->object_position(dspace_->get_state(objind, destination), opose);

  //inverse grasp in the frame of the object is the pose for the wrist
  gm::PoseStamped wps = propagate_grasp(destination, source, objind);
  dspace_->copyState(result, source);
  ospace->set_object_position(dspace_->get_state(objind, result), opose);

  gm::PoseStamped ps = transform_pose_stamped(arm_space_->root_name(), wps, source);
  if (seed) {
    if (arm_space_->inverse_kinematics(ps, arm_state(result), arm_state(seed))) {
      return true;
    }
  }
  if (arm_space_->inverse_kinematics(ps, arm_state(result),
				     arm_state(source))) {
    return true;
  }
  bool ret = arm_space_->inverse_kinematics(ps, arm_state(result));
  if (!ret && debug_level >= DPROPAGATE) {
    ROS_INFO_STREAM("No IK solution for\n" << ps);
  }
  return ret;
}

bool darrt::PR2Arm::setup_arm(const oc::SpaceInformation *si) {
  step_size_ = si->getPropagationStepSize();
  dspace_ = si->getStateSpace()->as<DARRTStateSpace>();
  robot_space_ = dspace_->robot_state_space()->as<CompoundRobotStateSpace>();
  arm_index_ = robot_space_->getSubspaceIndex(arm_name_);
  arm_space_ = robot_space_->getSubspace(arm_index_)->as<RobotChainStateSpace>();
  step_size_ = si->getPropagationStepSize();
  return ik_solver_.setup(dspace_);
}

ob::State *darrt::PR2Arm::arm_state(ob::State *full_state) const {
  return robot_space_->get_state(arm_index_, dspace_->robot_state(full_state));
}

const ob::State *darrt::PR2Arm::arm_state(const ob::State *full_state) const {
  return robot_space_->get_state(arm_index_, dspace_->robot_state(full_state));
}


bool darrt::PR2Arm::execute_path
(const std::vector<const ob::State *> &path) const {
  if (!path.size()) {
    ROS_INFO("Empty path, nothing to execute");
    return true;
  }
  pr2_controllers_msgs::JointTrajectoryGoal goal;
  if (!convert_path_to_joint_trajectory(path, goal.trajectory)) {
    ROS_ERROR("Unable to convert to joint trajectory");
    return false;
  }
  
  actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>
    executor("/"+arm_name_.substr(0,1)+
	     "_arm_controller/joint_trajectory_action");
  ROS_INFO("Waiting for joint trajectory action");
  executor.waitForServer();


  ROS_INFO("Sending trajectory of %zd points.  Should take %f seconds.",
	   goal.trajectory.points.size(), 
	   goal.trajectory.points.back().time_from_start.toSec());
  goal.trajectory.header.stamp = ros::Time::now();
  executor.sendGoal(goal);
  executor.waitForResult();
  ROS_INFO("Returned from executing");
  return true;
}

bool darrt::PR2Arm::open_gripper() const {
  return move_gripper(0.08);
}

bool darrt::PR2Arm::close_gripper() const {
  return move_gripper(0.0);
}

bool darrt::PR2Arm::move_gripper(double pos) const {
  actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>
    gripper("/"+arm_name_.substr(0,1)+"_gripper_controller/gripper_action");
  ROS_INFO("Waiting for gripper action");
  gripper.waitForServer();
  pr2_controllers_msgs::Pr2GripperCommandGoal goal;
  goal.command.position = pos;
  goal.command.max_effort = 50.0;
  gripper.sendGoal(goal);
  return gripper.waitForResult(ros::Duration(8.0));
}



bool darrt::PR2Arm::convert_path_to_joint_trajectory
(const std::vector<const ob::State *> &path, 
 trajectory_msgs::JointTrajectory &traj) const {

  //move into the robot's subspace
  //used to all be references because of seg fault but i think it's ok now
  ob::StateSpacePtr arm_space_ptr = robot_space_->getSubspace(arm_index_);
  ob::SpaceInformationPtr rsi(new ob::SpaceInformation(arm_space_ptr));
  ompl::geometric::PathGeometric rpath(rsi); 

  std::vector<ob::State *> &rstates = rpath.getStates();
  rstates.resize(path.size());
  for (size_t i = 0; i < path.size(); i++) {
    rstates[i] = arm_space_->allocState();
    arm_space_->copyState(rstates[i], arm_state(path[i]));
  }

  an::RobotTrajectory rtraj;

  if (!ompl_ros_conversions::ompl_path_geometric_to_robot_trajectory
      (rpath, arm_space_ptr, arm_space_->ompl_to_robot_mapping(), rtraj,
       arm_space_->physical_joint_group())) {
    ROS_ERROR("Unable to convert ompl path to robot path");
    return false;
  }

  traj = rtraj.joint_trajectory;
  if (!unnormalize_trajectory(traj)) {
    ROS_ERROR("Unable to convert robot path to robot trajectory");
    return false;
  }

  return true;
}
 

bool darrt::PR2Arm::unnormalize_trajectory
(trajectory_msgs::JointTrajectory &traj) const {
  //put a reasonable amount of time between each point
  //i have found that 8cm/s is a good speed
  //with the default settings this is 0.375 seconds between points
  //yes, 10cm/s is too fast even though it is a nice round number
  //hm... 8 is too fast for short hops of which i do have some...
  double dur = step_size_/0.07;
  for (size_t i = 0; i < traj.points.size(); i++) {
    traj.points[i].time_from_start = ros::Duration((i+1.0)*dur);
  }
  //make sure the rolling joints take the shortest
  //path between angles
  for (size_t j = 1; j < traj.points.size(); j++) {
    std::vector<double> newpositions;
    for (size_t i = 0; i < traj.joint_names.size(); i++) {
      double curr = traj.points[j].positions[i];
      //haha should check revolute but don't
      if (!traj.joint_names[i].compare("r_forearm_roll_joint") ||
	  !traj.joint_names[i].compare("l_forearm_roll_joint") ||
	  !traj.joint_names[i].compare("r_wrist_roll_joint") ||
	  !traj.joint_names[i].compare("l_wrist_roll_joint")) {
	double prev = traj.points[j-1].positions[i];
	while (prev - curr > MATH_PI) {
	  curr += 2.0*MATH_PI;
	}
	while (curr - prev > MATH_PI) {
	  curr -= 2.0*MATH_PI;
	}
      }
      newpositions.push_back(curr);
    }
    traj.points[j].positions = newpositions;
  }
  return true;
}

void darrt::PR2Arm::collisionOperations(const ob::State *state, an::OrderedCollisionOperations &ops) const {
  //disables collisions between objects and their support surfaces

  //allow collisions among objects during movement
  an::CollisionOperation op;
  op.operation = op.DISABLE;
  for (unsigned int i = 0; i < dspace_->object_indexes().size(); i++) {
    unsigned int ind = dspace_->object_indexes().at(i);
    const ObjectStateSpace::StateType *os = 
      dspace_->get_state(ind, state)->as<ObjectStateSpace::StateType>();
    op.object1 = dspace_->getSubspace(dspace_->object_indexes().at(i))->getName();
    if (os->support_surface) {
      op.object2 = os->support_surface->name();
      ops.collision_operations.push_back(op);
    } else {
      for (unsigned int j = 0; j < os->touch_links.size(); j++) {
	op.object2 = os->touch_links[j];
	ops.collision_operations.push_back(op);
      }
    }
  }
}

unsigned int darrt::PR2Arm::sampleArmMove(const Primitive *prim, std::string type, 
					  const ob::State *source, 
					  const ob::State *target, 
					  PIList &clist) const {
  PR2ArmTransitInstance *pi; 
  pi = new PR2ArmTransitInstance(prim, type, source, source);
  //only the arm state changes
  arm_space_->copyState(arm_state(pi->destination()), arm_state(target));
  ik_solver_.update_attached_object_positions(source, pi->destination());

  //allow collisions among objects during movement
  an::OrderedCollisionOperations ops;
  collisionOperations(pi->source(), ops);
  pi->add_allowed_collisions(ops);
  
  unsigned int turns =  
    (unsigned int)(arm_space_->joint_space_distance
		   (arm_state(pi->destination()), arm_state(pi->source()))/		
   (step_size_*OBJECT_ROBOT_CONVERSION))+1;
  pi->set_turns(turns);
  clist.push_back(pi);
  return turns;
}

unsigned int darrt::PR2Arm::sampleLineArmMove
(const Primitive *prim, std::string type,
 const ob::State *source, const gm::Transform &trans,
 double step_size, const an::OrderedCollisionOperations &ordered_collisions, 
 PIList &clist) const {

  gm::Quaternion origin;
  origin.w = 1;

  unsigned int turns = (unsigned int)
    (norm(trans.translation)/step_size + 1.0);
  unsigned int aturns = (unsigned int)
    (darrt::distance(trans.rotation, origin)/step_size + 1.0);
  
  if (turns < aturns) {
    turns = aturns;
  }

  std::vector<ob::State *> path;
  double fraction = 0.0;
  for (unsigned int i = 0; i < turns; i++) {
    if (i > 0 && turns > 1) {
      fraction = i/(double)(turns-1);
    }
    DARRTStateSpace::StateType *pstate = 
      dspace_->allocState()->as<DARRTStateSpace::StateType>();
    if (fraction < EPSILON) {
      dspace_->copyState(pstate, source);
    } else {
      if (!ik_solver_.propagate_interpolated_ik(source, trans, fraction, pstate, path[i-1])) {
	if (debug_level >= DPROPAGATE) {
	  ROS_INFO("Unable to propagate interpolated IK with twist far enough");
	}
	for (unsigned int i = 0; i < path.size(); i++) {
	  dspace_->freeState(path[i]);
	}
	return 0;
      }
    }
    path.push_back(pstate);
  }
  StatePathInstance *pi =  new StatePathInstance(prim, type, path);
  //don't do this any mmore
  //pi->add_allowed_collisions(source->as<DARRTStateSpace::StateType>()
  //			     ->allowed_collisions);
  an::OrderedCollisionOperations ops;
  collisionOperations(source, ops);
  pi->add_allowed_collisions(ordered_collisions);
  pi->add_allowed_collisions(ops);
  pi->set_turns(turns);
  clist.push_back(pi);
  return turns;
}

//note: direction should be in the root frame of the arm
unsigned int darrt::PR2Arm::sampleLineArmMove
(const Primitive *prim, std::string type, 
 const ob::State *source, 
 const ob::State *destination, double desired_distance, 
 double min_distance, const gm::Vector3 &direction, 
 double step_size,
 const an::OrderedCollisionOperations &ordered_collisions, 
 PIList &clist, bool reverse) const {
  
  gm::Vector3 trans;
  trans.x = desired_distance*direction.x;
  trans.y = desired_distance*direction.y;
  trans.z = desired_distance*direction.z;

  unsigned int turns = (unsigned int)
    (desired_distance/step_size + 1.0);
  std::vector<ob::State *> path;
  double fraction = 0.0;
  for (unsigned int i = 0; i < turns; i++) {
    if (i > 0 && turns > 1) {
      fraction = i/(double)(turns-1);
    }
    DARRTStateSpace::StateType *pstate = 
      dspace_->allocState()->as<DARRTStateSpace::StateType>();
    if (fraction < EPSILON) {
      dspace_->copyState(pstate, source);
    } else {
      if (!ik_solver_.propagate_interpolated_ik(source, destination, trans, fraction,
						pstate, path[i-1])) {
	if (debug_level >= DPROPAGATE) {
	  ROS_INFO("Propagate interpolated IK: only went distance %f of %f", fraction*desired_distance, desired_distance);
	}
	break;
      }
    }
    path.push_back(pstate);
  }
  if (fraction < 0.999999) {
    double distance = fraction*desired_distance;
    if (distance < min_distance || min_distance < -0.00001) {
      if (debug_level >= DPROPAGATE) {
	ROS_INFO("Unable to propagate interpolated IK far enough.  Distance is %f, minimum %f, fraction %f",
		 distance, min_distance, fraction);
      }
      for (unsigned int i = 0; i < path.size(); i++) {
	dspace_->freeState(path[i]);
      }
      return 0;
    }
  }
  turns = path.size();
  StatePathInstance *pi; 
  if (reverse) {
    std::vector<ob::State *> rpath(path.size(), NULL);
    for (int i = path.size()-1; i >= 0; i--) { 
      rpath[(path.size()-1)-i] = path[i];
    }
    pi = new StatePathInstance(prim, type, rpath);
  } else {
    pi = new StatePathInstance(prim, type, path);
  }
  //pi->add_allowed_collisions(source->as<DARRTStateSpace::StateType>()
  //			     ->allowed_collisions);
  pi->add_allowed_collisions(ordered_collisions);
  an::OrderedCollisionOperations ops;
  collisionOperations(source, ops);
  pi->add_allowed_collisions(ops);
  pi->set_turns(turns);
  clist.push_back(pi);
  return turns;
}




darrt::PR2ArmPrimitiveInstance::PR2ArmPrimitiveInstance
(const Primitive *prim, std::string type, const ob::State *source, const ob::State *destination) :
  CollisionAwarePrimitiveInstance(prim, type, source, destination) {
  dspace_ = prim->space_information()->getStateSpace()->as<DARRTStateSpace>();
  robot_space_ = dspace_->robot_state_space()->as<CompoundRobotStateSpace>();
  arm_ = dynamic_cast<const PR2Arm *>(prim_);
  arm_name_ = arm_->arm_name();
  arm_index_ = robot_space_->getSubspaceIndex(arm_name_);
  arm_space_ = robot_space_->as<RobotChainStateSpace>(arm_index_);
}

darrt::Primitive *darrt::PR2ArmTransit::copy() const {
  PR2ArmTransit *p = new PR2ArmTransit(arm_name_);
  if (is_setup()) {
    p->setup(si_);
  }
  return p;
}

bool darrt::PR2ArmTransit::setup(const oc::SpaceInformation *si) {
  if (!TransitPrimitive::setup(si)) {
    return false;
  }
  return setup_arm(si);
}


bool darrt::PR2ArmTransit::useful
(const ob::State *source, const ob::State *destination) const {
  
  const DARRTStateSpace::StateType *s = 
    source->as<DARRTStateSpace::StateType>();
  const DARRTStateSpace::StateType *d = 
    destination->as<DARRTStateSpace::StateType>();

  
  //we don't use this in if we're holding an object and that object
  //is already in its preferred location

  //we've removed the object being close to work better going backwards
  //shouldn't allow transit to work if we're moving with a spatula
  //because the thing might fall off
  if (arm_space_->near_states(arm_state(source), arm_state(destination)) ||
      !s->valid() || (s->control() && !s->control()->name().substr(0,4).compare("Push"))
      || (d->control() && (!d->control()->name().substr(0,4).compare("Push") ||
			   !d->control()->name().compare("Approach_"+arm_name_)))) {
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("ArmTransit: Not useful because the robot arm doesn't move, the source is invalid, the source control is push or the destination control is push or approach.  Near robot states: %d, source validity: %d, source control: %s, destination control: %s", 
	       arm_space_->near_states(arm_state(source), arm_state(destination)), s->valid(),
	       s->control() ? s->control()->name().c_str():"None",
	       d->control() ? d->control()->name().c_str():"None");
    }
    return false;
  }

  if (!dspace_->near_object_states(source, destination)) {
    //this used to mess things up... i wonder if it still will?
    //yes if we need to go grasp something...
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("ArmTransit: Not useful because object states are far apart");
    }
    return false;
  }
  
  for (size_t i = 0; i < dspace_->object_indexes().size(); i++) {
    unsigned int ind = dspace_->object_indexes().at(i);
    const ObjectStateSpace::StateType *os =
      dspace_->get_state(ind, source)->as<ObjectStateSpace::StateType>();
    if (os->attach_link.size()) {
      if (debug_level >= DPROPAGATE) {
	ROS_INFO("ArmTransit: Not useful because rigid attachment");
      }
      return false;
    }
  }
  return true;
}

bool darrt::PR2ArmTransit::active
(const ob::State *source, const ob::State *destination) const { 

  return si_->isValid(source);
}

unsigned int darrt::PR2ArmTransit::sampleTo
(const ob::State *source, const ob::State *target, PIList &clist) const {
  
  return sampleArmMove(this, name(), source, target, clist);
  
  
}

void darrt::PR2ArmTransit::projectionFunction(const ob::State *from, ob::State *sample) const {
  //this should induce a transit if we're not holding anything
  if (debug_level >= DRRT) {
    ROS_INFO("Using transit projection.");
  }

  //put things on surfaces if they're attached
  for (unsigned int i = 0; i < dspace_->object_indexes().size(); i++) {
    unsigned int ind = dspace_->object_indexes().at(i);
    const ObjectStateSpace *ospace = dspace_->getSubspace(ind)->
      as<ObjectStateSpace>();
    ObjectStateSpace::StateType *os = dspace_->get_state(ind, sample)->as<ObjectStateSpace::StateType>();
    if (!dspace_->get_state(ind, from)->as<ObjectStateSpace::StateType>()->support_surface) {
      const DARRTObject &obj = ospace->object();
      std::vector<const SupportSurface *> surfs(dspace_->support_surfaces().size());
      unsigned int nsurfs = 0;
      for (unsigned int j = 0; j < dspace_->support_surfaces().size(); j++) {
	if (obj.canSupport(dspace_->support_surfaces().at(j))) {
	  surfs[nsurfs] = dspace_->support_surfaces().at(j);
	  nsurfs++;
	}
      }
      if (nsurfs == 0) {
	pause("No surface that can support object "+obj.id);
	continue;
      }
      unsigned int r = rand() % nsurfs;
      gm::Pose tpose = obj.sampleStablePose(surfs[r]);
      ospace->set_object_position(os, tpose);
      os->attach_link = "";
      os->touch_links.clear();
      os->support_surface = surfs[r];
    } else {
      ospace->copyState(os, dspace_->get_state(ind, from));
    }
  }
}

darrt::DARRTProjectionFunction darrt::PR2ArmTransit::getProjectionFunction() const {
  return boost::bind(&darrt::PR2ArmTransit::projectionFunction, this, _1, _2);
}

darrt::PrimitiveInstance *darrt::PR2ArmTransitInstance::copy() const {
  PR2ArmTransitInstance *pi = new PR2ArmTransitInstance(prim_, type_, source_, destination_);
  pi->set_allowed_collisions(allowed_collisions);
  pi->set_turns(turns_);
  return pi;
}

double darrt::PR2ArmTransitInstance::fraction(const ob::State *state) const {
  //are the object states equal?
  if (!dspace_->near_object_states(source_, state)) {
    if (debug_level == DBETWEEN) {
      ROS_INFO("fraction: unequal object states");
    }
    return -1.0;
  }
  return arm_space_->between(arm_->arm_state(state), arm_->arm_state(source_), 
			     arm_->arm_state(destination_));
}

//could check controls here... but that seems like asking for trouble
bool darrt::PR2ArmTransitInstance::contains(const ob::State *state) const {
  double f = fraction(state);
  if (debug_level == DBETWEEN) {
    ROS_INFO_STREAM("Checking if\n" << this->str() << "\ncontains\n" << 
		    dspace_->state_string(state) << "\nf = " << f
		    << " f < 1: " << (f < 1) << " f < 1.0: " << (f < 1.0)
		    << " f < " << 1.0 - EPSILON << ": " << 
		    (f < (1.0 - EPSILON)));
    ROS_INFO("f = %lf", f);
  }
  //between 0 and less than 1
  //or 2 indicates that everything is equal
  ///arggh f manages to be something less than 1
  //and 1 - epsilon when it should just be 1... very confusing
  return ((f >= 0 && f < 0.999999) || (f > 1.1));
}

bool darrt::PR2ArmTransitInstance::propagate(const ob::State *state,
					     double duration,
					     ob::State *result) const {
  double d = arm_space_->joint_space_distance(arm_->arm_state(destination_), 
					      arm_->arm_state(source_));
  double ds = arm_space_->joint_space_distance(arm_->arm_state(state), 
					       arm_->arm_state(source_));

  double newf;
  if (d < EPSILON) {
    newf = 1.0;
  } else {
    newf = (ds + duration*OBJECT_ROBOT_CONVERSION)/d;
  }
  if (newf >= 1.0) {
    newf = 1.0;
  }
  if (newf <= 0) {
    newf = 0;
  }
  dspace_->copyState(result, destination_);
  arm_space_->interpolate(arm_->arm_state(source_), 
			  arm_->arm_state(destination_), newf, 
			  arm_->arm_state(result));
  dynamic_cast<const PR2Arm *>(prim_)->ik_solver()->
    update_attached_object_positions(state, result);

  //set the allowed collisions to match the allowed collisions at
  //the destination
  result->as<DARRTStateSpace::StateType>()->set_allowed_collisions
    (allowed_collisions);
  result->as<DARRTStateSpace::StateType>()->set_control(prim_, type_);

  if (debug_level >= DPROPAGATE) {
    ROS_INFO("Moving to fraction %f along %s gives %s", newf,
	     this->str().c_str(), dspace_->state_string(result).c_str());
  }
  return true;
}


bool darrt::PR2LineArmTransit::setup(const oc::SpaceInformation *si) {
  if (!PR2ArmTransit::setup(si)) {
    return false;
  }
  ROS_INFO("Setting up arm...");
  return setup_arm(si);
}

unsigned int darrt::PR2LineArmTransit::sampleTo
(const ob::State *source, const gm::Transform &trans,
 const an::OrderedCollisionOperations &ordered_collisions,
 PIList &clist) const {

  return sampleLineArmMove(this, name(), source, trans, si_->getPropagationStepSize(),
			   ordered_collisions, clist);
}

//note: direction should be in the root frame of the arm
unsigned int darrt::PR2LineArmTransit::sampleTo
(const ob::State *source, const ob::State *destination, 
 double desired_distance, double min_distance, 
 const gm::Vector3 &direction, 
 const an::OrderedCollisionOperations &ordered_collisions, 
 PIList &clist, bool reverse) const {

  return sampleLineArmMove(this, name(), source, destination, desired_distance,
			   min_distance, direction, si_->getPropagationStepSize(),
			   ordered_collisions, clist, reverse);
}

bool darrt::PR2LineArmTransit::execute
(const std::vector<const ob::State *> &path) const {
  // if (!name().substr(0,8).compare("Approach") || 
  //     !name().substr(0,8).compare("approach")) {
  //   open_gripper();
  // }
  return execute_path(path);
}

darrt::Approach::Approach(std::string arm_name, double desired_distance,
			  double min_distance) :
  PR2LineArmTransit("Approach", arm_name) {
  
  desired_distance_ = desired_distance;
  min_distance_ = min_distance;
  
  arm_transit_ = NULL;
  base_transit_ = NULL;
}

darrt::Primitive *darrt::Approach::copy() const {
  Approach *p = new Approach(arm_name_, desired_distance_, min_distance_);
  if (is_setup()) {
    p->setup(si_);
  }
  return p;
}


bool darrt::Approach::setup(const oc::SpaceInformation *si) {
  if (!PR2LineArmTransit::setup(si)) {
    return false;
  }

  arm_transit_ = NULL;
  base_transit_ = NULL;
  for (size_t i = 0; i < dspace_->primitives().size(); i++) {
    if (!arm_transit_) {
      arm_transit_ = dynamic_cast<const PR2ArmTransit *>
	(dspace_->primitives().at(i));
      if (dynamic_cast<const Retreat *>(dspace_->primitives().at(i)) ||
	  dynamic_cast<const Approach *>(dspace_->primitives().at(i))) {
	//not actually a transit
	arm_transit_ = NULL;
      }
    }
    if (!base_transit_) {
      const PR2BaseManipulation *btnf = dynamic_cast<const PR2BaseManipulation *>
	(dspace_->primitives().at(i));
      if (!btnf) {
	base_transit_ = dynamic_cast<const PR2BaseTransit *>
	  (dspace_->primitives().at(i));
      }
    }
    if (arm_transit_ && base_transit_) {
      break;
    }
  }
  if (!arm_transit_) {
    pause("Unable to find arm transit primitive for approach!");
    is_setup_ = false;
    return false;
  }
  return true;
}

bool darrt::Approach::useful(const ob::State *source,
			     const ob::State *destination) const {

  const DARRTStateSpace::StateType *d =
    destination->as<DARRTStateSpace::StateType>();

  if (dspace_->near_states(source, destination) ||
      !dspace_->near_object_states(source, destination) ||
      !d->control() ||
      (d->control()->name().substr(0,4).compare("Push") &&
       d->control()->name().compare(name()))) {
    //approach is only useful when approaching a destination
    //that is in pick, push, or approach
    return false;
  }

  //since approach will also sample the necessary
  //base and transit moves, we can go ahead and ignore
  //whether "approach" is really needed
  return true;
}

unsigned int darrt::Approach::sampleTo
(const ob::State *source, const ob::State *target, PIList &clist) const {
  an::OrderedCollisionOperations ops;
  an::CollisionOperation op;
  //allow collisions with all objects AND their support surfaces
  //during approach (this is easy...)
  op.operation = op.DISABLE;
  for (unsigned int i = 0; i < dspace_->object_indexes().size(); i++) {
    unsigned int ind = dspace_->object_indexes().at(i);
    const ObjectStateSpace::StateType *os;
    os = dspace_->get_state(ind, source)->as<ObjectStateSpace::StateType>();
    if (!os->support_surface) {
      continue;
    }
    const std::vector<std::string> *touch_links = &(os->touch_links);
    if (!touch_links->size()) {
      //the touching is in the destination
      touch_links = &(dspace_->get_state(ind, target)->
		      as<ObjectStateSpace::StateType>()->touch_links);
    }
    for (unsigned int j = 0; j < touch_links->size(); j++) {
      op.object1 = touch_links->at(j);
      op.object2 = dspace_->getSubspace(ind)->getName();
      ops.collision_operations.push_back(op);
      op.object2 = os->support_surface->name();
      ops.collision_operations.push_back(op);
    }
  }

  //now using the base position, sample the approach
  //this may result in approaches that are kind of long
  //but that's probably OK
  gm::PoseStamped wpose;
  arm_space_->forward_kinematics(arm_state(target), wpose);
  gm::PoseStamped rpose = ik_solver_.transform_pose_stamped
    (arm_space_->root_name(), wpose, target);
  gm::Vector3 xaxis;
  xaxis.x = -1.0;
  gm::Vector3 approach_dir = transform_vector(xaxis, rpose.pose);

  //hm... would be better to get these off a grasp somewhere
  //should probably be storing that - then i could get the
  //right distance as well
  unsigned int turns = 0;
  PIList approach;
  turns += PR2LineArmTransit::sampleTo(target, source, desired_distance_,
				       min_distance_, approach_dir, ops, approach, true);

  const ob::State *pregrasp = target;
  if (approach.size()) {
    pregrasp = approach[0]->source();
  }

  int order = rand() % 2;

  const ob::State *curr_state = source;

  if (order) {
    turns += arm_transit_->sampleTo(curr_state, pregrasp, clist);
    curr_state = clist[clist.size()-1]->destination();
  }

  if (base_transit_) {
    const BaseIK &base_ik = base_transit_->base_ik();
    if (!base_ik.base_space()->near_states(base_ik.base_state(curr_state),
					   base_ik.base_state(pregrasp))) {
      turns += base_transit_->sampleTo(curr_state, pregrasp, clist);
      curr_state = clist[clist.size()-1]->destination();
    }
  }

  if (!order) {
    turns += arm_transit_->sampleTo(curr_state, pregrasp, clist);
    curr_state = clist[clist.size()-1]->destination();
  }
  
  for (unsigned int i = 0; i < approach.size(); i++) {
    clist.push_back(approach[i]);
  }
  approach.clear();
  return turns;
}

darrt::Primitive *darrt::Retreat::copy() const {
  Retreat *p = new Retreat(arm_name_, desired_distance_, min_distance_);
  if (is_setup()) {
    p->setup(si_);
  }
  return p;
}


bool darrt::Retreat::useful(const ob::State *source,
			    const ob::State *destination) const {

  //useful when we're not holding anything, but we just did 
  //a transfer and we want to move the robot
  //note: some transfers result in an inactive robot state
  //we don't know what the robot state will be so we need to wait
  //until the propagate has happened before we can retreat
  const DARRTStateSpace::StateType *s = 
    source->as<DARRTStateSpace::StateType>();
  if (dspace_->near_states(source, destination) ||
      !s->valid() ||
      !s->control() || !s->control()->transfer() || 
      !s->control()->name().compare(name())) {
    return false;
  }
  for (size_t i = 0; i < dspace_->object_indexes().size(); i++) {
    unsigned int ind = dspace_->object_indexes().at(i);
    const ObjectStateSpace::StateType *os =
      dspace_->get_state(ind, source)->as<ObjectStateSpace::StateType>();
    if (!os->support_surface || os->attach_link.size()) {
      //holding something
      if (debug_level >= DPROPAGATE) {
	ROS_INFO("Retreat: holding something");
      }
      return false;
    }
  }
  //retreat along the gripper
  gm::PoseStamped wpose;
  arm_space_->forward_kinematics(arm_state(source), wpose);
  gm::PoseStamped rpose = ik_solver_.transform_pose_stamped
    (arm_space_->root_name(), wpose, source);
  gm::Vector3 xaxis;
  xaxis.x = -1.0;
  xaxis = transform_vector(xaxis, rpose.pose);
  return ik_solver_.feasible(source, min_distance_,  xaxis);
}

unsigned int darrt::Retreat::sampleTo
(const ob::State *source, const ob::State *target, PIList &clist) const {
  an::OrderedCollisionOperations ops;
  an::CollisionOperation op;
  //allow collisions with all objects AND their support surfaces
  //during retreat (this is easy...)
  op.operation = op.DISABLE;
  for (unsigned int i = 0; i < dspace_->object_indexes().size(); i++) {
    unsigned int ind = dspace_->object_indexes().at(i);
    const ObjectStateSpace::StateType *os;
    os = dspace_->get_state(ind, source)->as<ObjectStateSpace::StateType>();
    if (!os->support_surface) {
      continue;
    }
    for (unsigned int j = 0; j < os->touch_links.size(); j++) {
      op.object1 = os->touch_links[j];
      op.object2 = dspace_->getSubspace(ind)->getName();
      ops.collision_operations.push_back(op);
      op.object2 = os->support_surface->name();
      ops.collision_operations.push_back(op);
    }
  }

  //retreat along the gripper
  gm::PoseStamped wpose;
  arm_space_->forward_kinematics(arm_state(source), wpose);
  gm::PoseStamped rpose = ik_solver_.transform_pose_stamped
    (arm_space_->root_name(), wpose, source);
  gm::Vector3 xaxis;
  xaxis.x = -1.0;

  return PR2LineArmTransit::sampleTo(source, target, desired_distance_, min_distance_,
				     transform_vector(xaxis, rpose.pose), ops, clist);
}

darrt::Primitive *darrt::RigidTransfer::copy() const {
  RigidTransfer *p = new RigidTransfer(arm_name_, objid_);
  if (is_setup()) {
    p->setup(si_);
  }
  return p;
}


bool darrt::RigidTransfer::setup(const oc::SpaceInformation *si) {
  if (!TransferPrimitive::setup(si)) {
    return false;
  }
  if (!setup_arm(si)) {
    return false;
  }
  objind_ = dspace_->getSubspaceIndex(objid_);
  base_ik_ = NULL;
  uspat_ = NULL;
  for (unsigned int i = 0; i < dspace_->primitives().size(); i++) {
    const PR2BaseTransit *bt = dynamic_cast<const PR2BaseTransit *>
      (dspace_->primitives().at(i));
    if (bt) {
      base_ik_ = &(bt->base_ik());
      base_transit_name_ = bt->name();
    }
    if (!uspat_) {
      uspat_ = dynamic_cast<const UseSpatula *>(dspace_->primitives().at(i));
    }
  }
  return true;
}

bool darrt::RigidTransfer::useful(const ob::State *source,
				  const ob::State *destination) const {
  //we must want to move the object
  //to someplace not on the table really but we can ignore that for now
  //we have to already be grasping it = control is pickup or rigidtransfer
  //note: some transfer primitives leave the robot state unspecified
  //and we need a full one
  const DARRTStateSpace::StateType *s = 
    source->as<DARRTStateSpace::StateType>();
  
  return (s->valid() && active(source, destination));
}

bool darrt::RigidTransfer::active(const ob::State *source,
				  const ob::State *destination) const {

  if (dspace_->near_object_states(objind_, source, destination)) {
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("Not using rigid transfer because object in goal position");
    }
    return false;
  }
  const ObjectStateSpace::StateType *os = 
    dspace_->get_state(objind_, source)->as<ObjectStateSpace::StateType>();
  if (os->support_surface || !os->attach_link.size()) {
    //not currently grasping
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("Cannot use rigid transfer because object is on surface or not attached");
    }
    return false;
  }
  const ObjectStateSpace *ospace = dspace_->getSubspace(objind_)->as<ObjectStateSpace>();
  gm::Pose dpose;
  ospace->object_position(dspace_->get_state(objind_, destination), dpose);
  //are we trying to put it down on a surface (use place instead)
  gm::Point tpt = object_bottom_point(ospace->object(), dpose);
  bool placing = false;
  if (debug_level >= DPROPAGATE) {
    ROS_INFO_STREAM("Checking place on surface for pose\n" << dpose <<
		    "\n corresponding to point\n" << tpt);
  }
  for (size_t j = 0; j < dspace_->support_surfaces().size(); j++) {
    if (dspace_->support_surfaces().at(j)->on_surface(tpt, TABLE_CONTAINS)) {
      if (debug_level >= DPROPAGATE) {
	ROS_INFO("Rigid transfer not useful because attempting to place on table");
      }
      placing = true;
      break;
    }
  }
  if (placing) {
    return false;
  }

  for (unsigned int i = 0; i < dspace_->object_indexes().size(); i++) {
    unsigned int ind = dspace_->object_indexes().at(i);
    if (ind == objind_) {
      continue;
    }
    const ObjectStateSpace::StateType *ss = dspace_->get_state(ind, source)->as<ObjectStateSpace::StateType>();
    const ObjectStateSpace::StateType *ts = dspace_->get_state(ind, destination)->as<ObjectStateSpace::StateType>();
    if (ss->attach_link.size() || ts->attach_link.size()) {
      if (debug_level >= DPROPAGATE) {
	ROS_INFO("RigidTransfer not useful because multiple things attached");
      }
      return false;
    }
  }
  return true;
}

unsigned int darrt::RigidTransfer::sampleTo
(const ob::State *source, const ob::State *target, PIList &clist) const {
  std::vector<unsigned int> objinds;

  //does the target already have the thing in the correct position?
  //in that case, move there directly
  gm::Pose held_pose = ik_solver_.update_attached_object_position(objind_, source, target);
  const ObjectStateSpace *ospace = dspace_->getSubspace(objind_)->as<ObjectStateSpace>();
  ob::State *held_state = ospace->allocState();
  ospace->set_object_position(held_state, held_pose);
  if (ospace->near_states(held_state, dspace_->get_state(objind_, target))) {
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("Rigid transfering directly to target state");
    }
    ospace->freeState(held_state);
    return sampleMove(source, target, clist);
  }
  ospace->freeState(held_state);

  ob::State *result = NULL;
  if (!base_ik_) {
    result = ik_solver_.propagate_grasp(target, source, objind_, NULL);
  } else {
    result = dspace_->allocState();
    dspace_->copyState(result, source);
    gm::PoseStamped wps = ik_solver_.propagate_grasp(target, source, objind_);
    bool found_ik = false;
    for (unsigned int i = 0; i < BASE_TRIES + 2; i++) {
      if (i == 0) {
	//first try the destination base position
	base_ik_->base_space()->copyState
	  (base_ik_->base_state(result),
	   base_ik_->base_state(target));
      } else if (i == 1) {
	//then the source position
	base_ik_->base_space()->copyState
	  (base_ik_->base_state(result),
	   base_ik_->base_state(source));
      } else {
	base_ik_->get_base_pose_for_wrist(wps.pose, result);
      }
      gm::PoseStamped ps = ik_solver_.transform_pose_stamped(arm_space_->root_name(), wps, result);
      if (arm_space_->inverse_kinematics(ps, arm_state(result), arm_state(source))) {
	found_ik = true;
	break;
      }
      if (arm_space_->inverse_kinematics(ps, arm_state(result))) {
	found_ik = true;
	break;
      }
    }
    if (!found_ik) {
      dspace_->freeState(result);
      result = NULL;
    }
  }
  
  if (!result) {
    if (debug_level >= DRRT) {
      ROS_INFO("Unable to find IK solution (even moving base)");
    }
    return 0;
  }  
  unsigned int turns = sampleMove(source, result, clist);
  dspace_->freeState(result);
  return turns;
}

unsigned int darrt::RigidTransfer::sampleMove
(const ob::State *source, const ob::State *result, PIList &clist) const {
  unsigned int turns = 0;
  const ob::State *curr = source;
  unsigned int order = rand()%2;
  
  if (order) {
    turns += sampleArmMove(this, name(), curr, result, clist);
    curr = clist[clist.size()-1]->destination();
  }
  if (base_ik_ && !base_ik_->base_space()->near_states
      (base_ik_->base_state(source),
       base_ik_->base_state(result))) {
    turns += base_ik_->sampleBaseMove(this, base_transit_name_, curr, result, clist);
    curr = clist[clist.size()-1]->destination();
  }
  if (!order) {
    turns += sampleArmMove(this, name(), curr, result, clist);
  }

  return turns;
}

void darrt::RigidTransfer::projectionFunction(const ob::State *from, ob::State *sample) const {
  if (debug_level >= DRRT) {
    ROS_INFO("Using rigid transfer projection.");
  }

  //this is kind of very specific for spatula world

  //only allow objid to move
  //what if we're holding something else?
  //SHOULD put it down as part of rigid transfer
  //but we're not sure about how to do that right now
  //so in that case, just return the sampled state

  //if it's already attached,
  //figure out where to put the robot to get the thing here
  //we have to do this in case we have two things attached to the
  //robot
  const ObjectStateSpace::StateType *fs = dspace_->get_state(objind_, from)
    ->as<ObjectStateSpace::StateType>();

  bool use_spat = false;
  //put down anything we're already holding
  for (unsigned int i = 0; i < dspace_->object_indexes().size(); i++) {
    unsigned int ind = dspace_->object_indexes().at(i);
    if (ind == objind_) {
      continue;
    }
    const ObjectStateSpace *ospace = dspace_->getSubspace(ind)->
      as<ObjectStateSpace>();
    ObjectStateSpace::StateType *os = dspace_->get_state(ind, sample)->as<ObjectStateSpace::StateType>();
    if (uspat_ && ind == uspat_->objind()) {
      gm::Pose object_pose, spose, fpose;
      gm::Vector3Stamped had;
      uspat_->chooseToolUse(from, sample, object_pose, spose, fpose, had);
      ospace->set_object_position(os, object_pose);
      for (unsigned int j = 0; j < dspace_->support_surfaces().size(); j++) {
	const SupportSurface *surf = dspace_->support_surfaces().at(j);
	if (surf->supporting(ospace->object(), object_pose)) {
	  os->support_surface = surf;
	  os->attach_link = "";
	  os->touch_links.clear();
	}
      }
      if (dspace_->get_state(ind, from)->as<ObjectStateSpace::StateType>()->attach_link.size()) {
	use_spat = true;
      }
      continue;
    }
    if (!dspace_->get_state(ind, from)->as<ObjectStateSpace::StateType>()->support_surface) {
      const DARRTObject &obj = ospace->object();
      std::vector<const SupportSurface *> surfs(dspace_->support_surfaces().size());
      unsigned int nsurfs = 0;
      for (unsigned int j = 0; j < dspace_->support_surfaces().size(); j++) {
	if (obj.canSupport(dspace_->support_surfaces().at(j))) {
	  surfs[nsurfs] = dspace_->support_surfaces().at(j);
	  nsurfs++;
	}
      }
      if (nsurfs == 0) {
	pause("No surface that can support object "+obj.id);
	continue;
      }
      unsigned int r = rand() % nsurfs;
      gm::Pose tpose = obj.sampleStablePose(surfs[r]);
      ospace->set_object_position(os, tpose);
      os->attach_link = "";
      os->touch_links.clear();
      os->support_surface = surfs[r];
    } else {
      ospace->copyState(os, dspace_->get_state(ind, from));
    }
  }

  //solve for a position of the robot where it's holding the spatula
  //this involves picking up the spatula...
  if (uspat_ && (!fs->attach_link.size() || use_spat)) {
    //pick up the spatula
    std::vector<DiscreteGraspsPrimitive::GraspWithIK> pickup_grasps;
    uspat_->pickup().grasp_ik(sample, from, pickup_grasps);
    if (!pickup_grasps.size()) {
      if (debug_level >= DRRT) {
	pause("Unable to get any grasps in rigid transfer primitive.");
      }
      return;
    }
    unsigned int r = rand() % pickup_grasps.size();
    dspace_->copyState(sample, pickup_grasps[r].grasp_ik);
    for (unsigned int i = 0; i < pickup_grasps.size(); i++) {
      dspace_->freeState(pickup_grasps[i].grasp_ik);
    }
    const ObjectStateSpace *sspace = dspace_->getSubspace(objind_)->
      as<ObjectStateSpace>();
    ObjectStateSpace::StateType *ss = dspace_->get_state(objind_, sample)->
      as<ObjectStateSpace::StateType>();
    const PickupPrimitive::RigidGrasp *rg = 
      dynamic_cast<const PickupPrimitive::RigidGrasp * >(pickup_grasps[r].grasp);
    if (!rg) {
      pause("Return from pickup not a rigid grasp!");
      return;
    }
    ss->attach_link = rg->attach_link;
    ss->touch_links = rg->touch_links;
    ss->support_surface = NULL;
    return;
  }
  

  if (fs->attach_link.size()) {
    //make this the outcome of a single rigid transfer
    PIList clist;
    sampleTo(from, sample, clist);
    if (!clist.size()) {
      if (debug_level >= DPROPAGATE) {
	ROS_INFO("Unable to find rigid transfer in projection");
      }
      //something went wrong, oh well
      return;
    }
    dspace_->copyState(sample, clist[clist.size()-1]->destination());
    for (unsigned int i = 0; i < clist.size(); i++) {
      delete clist[i];
    }
    clist.clear();
  }


    

}

darrt::DARRTProjectionFunction darrt::RigidTransfer::getProjectionFunction() const {
  return boost::bind(&darrt::RigidTransfer::projectionFunction, this, _1, _2);
}


darrt::DiscreteGraspsPrimitive::DiscreteGraspsPrimitive
(std::string name, std::string arm_name, std::string objid) :
  TransferPrimitive(name+"-"+arm_name+"-"+objid), PR2Arm(arm_name) {
  base_transit_ = NULL;
  transit_ = NULL;
  retreat_ = NULL;
  approach_ = NULL;
  objid_ = objid;
}

bool darrt::DiscreteGraspsPrimitive::setup(const oc::SpaceInformation *si) {
  if (!TransferPrimitive::setup(si)) {
    return false;
  }
  if (!setup_arm(si)) {
    return false;
  }
  objind_ = dspace_->getSubspaceIndex(objid_);
  //also find a transit primitive
  max_retreat_dist_ = 0.0;
  transit_ = NULL;
  base_transit_ = NULL;
  retreat_ = NULL;
  approach_ = NULL;
  for (size_t i = 0; i < dspace_->primitives().size(); i++) {
    if (!transit_) {
      transit_ = dynamic_cast<const PR2ArmTransit *>
	(dspace_->primitives().at(i));
      if (dynamic_cast<const Retreat *>(dspace_->primitives().at(i)) ||
	  dynamic_cast<const Approach *>(dspace_->primitives().at(i))) {
	//not actually a transit
	transit_ = NULL;
      }
    }
    if (!base_transit_) {
      const PR2BaseManipulation *btnf = dynamic_cast<const PR2BaseManipulation *>
	(dspace_->primitives().at(i));
      if (!btnf) {
	base_transit_ = dynamic_cast<const PR2BaseTransit *>
	  (dspace_->primitives().at(i));
      }
    }
    const Retreat *r = dynamic_cast<const Retreat *>
      (dspace_->primitives().at(i));
    if (r && r->desired_distance() > max_retreat_dist_) {
      max_retreat_dist_ = r->desired_distance();
      retreat_ = r;
    }
    if (!approach_) {
      approach_ = dynamic_cast<const Approach *>
	(dspace_->primitives().at(i));
      if (dynamic_cast<const Retreat *>(dspace_->primitives().at(i))) {
	approach_ = NULL;
      }
    }
  }
  if (!transit_) {
    pause("Unable to find transit primitive for discrete grasps!");
    is_setup_ = false;
    return false;
  }
  return true;
}

gm::Pose darrt::DiscreteGraspsPrimitive::get_grasp_pose
(const gm::Transform &grasp, const gm::Pose &opose) const {
  gm::Pose origin;
  origin.orientation.w = 1.0;
  return transform_pose(inverse_transform_pose(origin, grasp), opose);
}

bool darrt::DiscreteGraspsPrimitive::useful
(const ob::State *source, const ob::State *target) const {
  return manipulableObject(source, target) &&
    active(source, target);
}

bool darrt::DiscreteGraspsPrimitive::manipulableObject(const ob::State *source, const ob::State *target) const {
  if (dspace_->near_object_states(objind_, source, target)) {
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("DG primitive not useful because object already at goal.");
    }
    return false;
  }
  //are we holding anything?
  for (unsigned int i = 0; i < dspace_->object_indexes().size(); i++) {
    unsigned int ind = dspace_->object_indexes().at(i);
    if (dspace_->get_state(ind, source)->as<ObjectStateSpace::StateType>()->attach_link.size()) {
      if (debug_level >= DPROPAGATE) {
	ROS_INFO("DG primitive not useful because something is already attached.");
      }
      return false;
    }
  }
  return true;
}

bool darrt::DiscreteGraspsPrimitive::active
(const ob::State *source, const ob::State *target) const {
  if (has_grasp(source, target, true)) {
    return true;
  }
  if (debug_level >= DPROPAGATE) {
    ROS_INFO("In grasps active, object does not have grasp");
  }
  return false;
}

unsigned int darrt::DiscreteGraspsPrimitive::sampleTo
(const ob::State *source, const ob::State *target, PIList &clist) const {
        
  GraspList graspsv;
 
  grasps(source, target, graspsv);
  while (graspsv.size() && ros::ok()) {
    unsigned int gr = rand() % graspsv.size();
    GraspWithIK grasp;
    if (!grasp_ik(source, target, *(graspsv[gr]), grasp)) {
      delete graspsv[gr];
      graspsv[gr] = graspsv.back();
      graspsv[graspsv.size()-1] = NULL;
      graspsv.pop_back();
      continue;
    }
    unsigned int turns = 0;
    std::string objid = dspace_->getSubspace(objind_)->getName();
    const SupportSurface *surf = dspace_->get_state(objind_, source)->
      as<ObjectStateSpace::StateType>()->support_surface;
    
    //approach from pregrasp to grasp
    //collision operations
    //object+gripper
    //object+surface
    //gripper+surface
    //the rest of their objects and their support surfaces
    an::OrderedCollisionOperations ops;
    an::CollisionOperation op;
    op.operation = op.DISABLE;
    for (unsigned int i = 0; i < dspace_->object_indexes().size(); i++) {
      unsigned int ind = dspace_->object_indexes().at(i);
      const SupportSurface *ssf = dspace_->get_state(ind, source)->
	as<ObjectStateSpace::StateType>()->support_surface;
      if (ssf) {
	op.object1 = dspace_->getSubspace(ind)->getName();
	op.object2 = ssf->name();
	ops.collision_operations.push_back(op);
      }
    }
    for (size_t i = 0; i < grasp.grasp->touch_links.size(); i++) {
      op.object1 = grasp.grasp->touch_links[i];
      op.object2 = objid;
      ops.collision_operations.push_back(op);
      if (surf) {
	op.object2 = surf->name();
	ops.collision_operations.push_back(op);
      }
    }
    PIList full_traj;
    
    PIList approach;
    turns += static_cast<const PR2LineArmTransit *>(approach_)->
      sampleTo(grasp.grasp_ik, target,
	       grasp.grasp->desired_distance, 
	       grasp.grasp->min_distance, 
	       grasp.direction_to_pregrasp, 
	       ops, approach, true);
    if (!turns || !approach.size()) {
      if (debug_level >= DRRT) {
	ROS_INFO("Unable to find approach to pregrasp from %s in direction (%f %f %f), min distance %f desired %f gr = %u",
		 dspace_->state_string(grasp.grasp_ik).c_str(),
		 grasp.direction_to_pregrasp.x, grasp.direction_to_pregrasp.y,
		 grasp.direction_to_pregrasp.z, grasp.grasp->min_distance, 
		 grasp.grasp->desired_distance, gr);
      }
      for (unsigned int i = 0; i < approach.size(); i++) {
	delete approach[i];
      }
      dspace_->freeState(grasp.grasp_ik);
      grasp.grasp_ik = NULL;
      delete grasp.grasp;
      grasp.grasp = NULL;
      delete graspsv[gr];
      graspsv[gr] = graspsv.back();
      graspsv[graspsv.size()-1] = NULL;
      graspsv.pop_back();
      continue;
    }
    
    ob::State *pregrasp = approach[0]->source();
    //transit to pregrasp
    const ob::State *curr = source;
    
    const Primitive *sprim = dynamic_cast<const CollisionAwareState *>
      (source)->control();
    //if we were transfering, retreat first
    if (retreat_ && sprim && sprim->transfer()) {
      if (debug_level >= DPROPAGATE) {
	ROS_INFO("USING RETREAT: control = %s", sprim->name().c_str());
      }
      turns += retreat_->sampleTo(curr, pregrasp, full_traj);
      curr = full_traj[full_traj.size()-1]->destination();
    }
    
    int order = rand() % 2; //randomize whether we move the arm or the base first
    
    if (order) {
      turns += transit_->sampleTo(curr, pregrasp, full_traj);
      curr = full_traj[full_traj.size()-1]->destination();
    }
    if (base_transit_ && !base_transit_->base_ik().base_space()->near_states
	(base_transit_->base_ik().base_state(pregrasp),
	 base_transit_->base_ik().base_state(source))) {
      //ROS_INFO("base movement happened");
      turns += base_transit_->sampleTo(curr, pregrasp, full_traj);
      curr = full_traj[full_traj.size()-1]->destination();
    }
    if (!order) {
      turns += transit_->sampleTo(curr, pregrasp, full_traj);
      curr = full_traj[full_traj.size()-1]->destination();
    }
    for (unsigned int i = 0; i < approach.size(); i++) {
      full_traj.push_back(approach[i]);
    }
    // for (unsigned int i = 0; i < full_traj.size(); i++) {
    // 	ROS_INFO("full_traj %d = %p", i, full_traj[i]);
    // }
    // pause("Check if pointers repeated above");
    approach.clear();
    unsigned int sample_turns = sample_from_grasp(source, target, grasp, full_traj);    
    turns += sample_turns;
    if (!sample_turns) {
      if (debug_level >= DRRT) {
	ROS_INFO("Discrete grasps sampling from grasp made no progress");
      }
      //can't actually go anywhere
      for (unsigned int i = 0; i < full_traj.size(); i++) {
	delete full_traj[i];
      }
      dspace_->freeState(grasp.grasp_ik);
      grasp.grasp_ik = NULL;
      delete grasp.grasp;
      grasp.grasp = NULL;
      delete graspsv[gr];
      graspsv[gr] = graspsv.back();
      graspsv[graspsv.size()-1] = NULL;
      graspsv.pop_back();
      continue;
    } 
    for (unsigned int i = 0; i < full_traj.size(); i++) {
      clist.push_back(full_traj[i]);
    }
    for (size_t i = 0; i < graspsv.size(); i++) {
      delete graspsv[i];
    }
    return turns; 
  }
  if (debug_level >= DRRT) {
    ROS_INFO("Can't use discrete grasp for object %u", objind_);
  }
  return 0;
}

bool darrt::DiscreteGraspsPrimitive::grasp_ik
(const ob::State *source, const ob::State *destination,
 std::vector<GraspWithIK> &gss) const {
  GraspList grasp_transforms;
  if (!grasps(source, destination, grasp_transforms)) {
    ROS_ERROR("Grasp states: unable to get any grasps");
    return false;
  }
  //try to find inverse kinematics for the grasp
  for (size_t i = 0; i < grasp_transforms.size(); i++) {
    GraspWithIK gs;
    if (grasp_ik(source, destination, *grasp_transforms[i], gs)) {
      gss.push_back(gs);
    }
    delete grasp_transforms[i];
  }
  return true;
}

bool darrt::DiscreteGraspsPrimitive::grasp_ik
(const ob::State *source, const ob::State *destination,
 const Grasp &grasp, GraspWithIK &grasp_with_ik) const {
  const ObjectStateSpace *ospace = 
    dspace_->getSubspace(objind_)->as<ObjectStateSpace>();
  gm::Pose opose;
  ospace->object_position(dspace_->get_state(objind_, source), opose);

 
  gm::PoseStamped rpose; //this is the pose of the wrist
  rpose.header.frame_id = ospace->object().header.frame_id;
  rpose.pose = get_grasp_pose(grasp.grasp, opose);
  grasp_with_ik.grasp = grasp.copy();
  grasp_with_ik.grasp_ik = dspace_->allocState();
  dspace_->copyState(grasp_with_ik.grasp_ik, source);   
 
  //figure out the pose for the robot in this state
  unsigned int base_tries = 1;
  if (base_transit_) {
    base_tries = BASE_TRIES;
  }

  for (unsigned int i = 0; i < base_tries; i++) {
    int use_source = rand() % 2;
    if (i == 0 && base_transit_) {
      base_transit_->base_ik().base_space()->copyState
	(base_transit_->base_ik().base_state(grasp_with_ik.grasp_ik),
	 base_transit_->base_ik().base_state(destination));
    } else if (i == 1 && use_source && base_transit_) {
      base_transit_->base_ik().base_space()->copyState
	(base_transit_->base_ik().base_state(grasp_with_ik.grasp_ik),
	 base_transit_->base_ik().base_state(source));
    } else if (base_transit_) {
      base_transit_->base_ik().get_base_pose_for_wrist(rpose.pose, grasp_with_ik.grasp_ik);
    } 

    if (debug_level >= DPROPAGATE && base_transit_) {
      ROS_INFO("Destination is %s", dspace_->state_string(destination).c_str());
      ROS_INFO_STREAM("DG::grasp_ik: Trying base pose\n" << base_transit_->base_ik().base_pose(grasp_with_ik.grasp_ik));
    }    
    //inverse kinematics for the grasp
    gm::PoseStamped irpose = ik_solver_.transform_pose_stamped
      (arm_space_->root_name(), rpose, grasp_with_ik.grasp_ik);
    if (!arm_space_->inverse_kinematics(irpose, arm_state(grasp_with_ik.grasp_ik), 
					arm_state(source)) &&
	!arm_space_->inverse_kinematics(irpose, arm_state(grasp_with_ik.grasp_ik))) {

      if (i > 1 && debug_level >= DPROPAGATE) {
	dspace_->display_state(grasp_with_ik.grasp_ik, "ik_grasp", 0, 0.05,
			       Displayable::PURPLEPALATE, 0.5);
    
	visualization_msgs::MarkerArray marray;
	visualization_msgs::Marker marker;
	marker.header.frame_id = rpose.header.frame_id;
	marker.ns = "ik_state";
	marker.id = 4000;
	marker.type = marker.ARROW;
	marker.action = marker.ADD;
	marker.pose = rpose.pose;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.r = 0;
	marker.color.g = 1;
	marker.color.b = 0;
	marker.color.a = 1;
	marray.markers.push_back(marker);
	dspace_->display(marray);
	
	pause("Unable to find IK solution for arrow");
      }
      continue;
    }

    gm::Vector3 xaxis;
    xaxis.x = -1.0;
    grasp_with_ik.direction_to_pregrasp = transform_vector(xaxis, irpose.pose);
    //we also want a solution that works with these
    std::vector<gm::PoseStamped> poses_to_check;
    extraGraspPoses(grasp_with_ik, irpose, source, destination, poses_to_check);

    ob::State *ik_state = arm_space_->allocState();

    bool found_ik = true;

    for (unsigned int j = 0; j < poses_to_check.size(); j++) {
      
      if (!arm_space_->inverse_kinematics(poses_to_check[j], ik_state, arm_state(source)) &&
      	  !arm_space_->inverse_kinematics(poses_to_check[j], ik_state)) {
      	found_ik = false;
	if (i > 1 && debug_level >= DPROPAGATE) {
	  //debugging
	  dspace_->display_state(grasp_with_ik.grasp_ik, "ik_grasp", 0, 0.05,
				 Displayable::PURPLEPALATE, 0.5);
	  gm::PoseStamped world_pose = ik_solver_.transform_pose_stamped
	    (rpose.header.frame_id, poses_to_check[j], grasp_with_ik.grasp_ik);
	  
	  visualization_msgs::MarkerArray marray;
	  visualization_msgs::Marker marker;
	  marker.header.frame_id = world_pose.header.frame_id;
	  marker.ns = "ik_state";
	  marker.id = 4000;
	  marker.type = marker.ARROW;
	  marker.action = marker.ADD;
	  marker.pose = world_pose.pose;
	  marker.scale.x = 0.1;
	  marker.scale.y = 0.1;
	  marker.scale.z = 0.1;
	  marker.color.r = 0;
	  marker.color.g = 1;
	  marker.color.b = 0;
	  marker.color.a = 1;
	  marray.markers.push_back(marker);
	  dspace_->display(marray);
	  pause("Unable to find IK solution for arrow");
	}
      	break;
      }
    }
    arm_space_->freeState(ik_state);
    if (found_ik) {
      return true;
    }
  }
  
  dspace_->freeState(grasp_with_ik.grasp_ik);
  grasp_with_ik.grasp_ik = NULL;
  if (debug_level >= DRRT) {
    ROS_INFO("No IK for grasp (%f %f %f)", rpose.pose.position.x,
	     rpose.pose.position.y, rpose.pose.position.z);
  }
  return false;
}

bool darrt::DiscreteGraspsPrimitive::has_grasp
(const ob::State *source, const ob::State *destination, 
 bool requires_pregrasp) const {
  GraspList grasp_transforms;
  if (!grasps(source, destination, grasp_transforms) ||
      !grasp_transforms.size()) {
    ROS_ERROR("Grasp states: unable to get any grasps");
    return false;
  }
  
  for (unsigned int i = 0; i < grasp_transforms.size(); i++) {
    delete grasp_transforms[i];
  }
  return true;
}

void darrt::DiscreteGraspsPrimitive::extraGraspPoses(const GraspWithIK &grasp, 
						     const gm::PoseStamped &grasp_pose,
						     const ob::State *source,
						     const ob::State *target,
						     std::vector<gm::PoseStamped> &extra_poses) const {
  //also check the minimum pregrasp
  if (grasp.grasp->min_distance > DIST_EPS) {
    double md = grasp.grasp->min_distance;
    gm::PoseStamped epose = grasp_pose;
    epose.pose.position.x += md*grasp.direction_to_pregrasp.x;
    epose.pose.position.y += md*grasp.direction_to_pregrasp.y;
    epose.pose.position.z += md*grasp.direction_to_pregrasp.z;
    extra_poses.push_back(epose);
  }
}

bool darrt::DiscreteGraspsPrimitive::feasible_grasp
(const ob::State *source, const ob::State *destination, 
 const Grasp &grasp, bool requires_pregrasp) const {
  GraspWithIK gs;
  if (!grasp_ik(source, destination, grasp, gs)) {
    return false;
  }
  if (!requires_pregrasp) {
    return true;
  }
  bool feasible_pregrasp = ik_solver_.feasible(gs.grasp_ik, grasp.min_distance,
					       gs.direction_to_pregrasp);
  dspace_->freeState(gs.grasp_ik);
  return feasible_pregrasp;
}

darrt::Push::Push(std::string arm_name, std::string objid, std::vector<std::string> pl,
		  double table_offset) :
  DiscreteGraspsPrimitive("Push", arm_name, objid) {
  pushing_links_ = pl;
  table_offset_ = table_offset;
}

darrt::Primitive *darrt::Push::copy() const {
  Push *p = new Push(arm_name_, objid_, pushing_links_, table_offset_);
  if (is_setup()) {
    p->setup(si_);
  }
  return p;
}

bool darrt::Push::setup(const oc::SpaceInformation *si) {
  return DiscreteGraspsPrimitive::setup(si);
}

bool darrt::Push::manipulableObject(const ob::State *source, const ob::State *destination) const {

  const ObjectStateSpace *ospace = dspace_->getSubspace(objind_)
    ->as<ObjectStateSpace>();
  const ObjectStateSpace::StateType *os = dspace_->get_state(objind_, source)
    ->as<ObjectStateSpace::StateType>();
  const ObjectStateSpace::StateType *ds = dspace_->get_state(objind_, destination)
    ->as<ObjectStateSpace::StateType>();

  if (!DiscreteGraspsPrimitive::manipulableObject(source, destination)) {
    return false;
  }

  if (!os->support_surface || (os->support_surface != ds->support_surface)) {
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("Push not useful because object not on table.");
    }
    return false;
  }

  gm::Pose opose, dpose;
  ospace->object_position(os, opose);
  ospace->object_position(ds, dpose);
  if (debug_level >= DPROPAGATE) {
    ROS_INFO("In push active source height is %f and destination height is %f",
	     opose.position.z, dpose.position.z);
  }
  return inrange(opose.position.z, dpose.position.z, DIST_EPS);
}

bool darrt::Push::grasps
(const ob::State *source, const ob::State *destination, GraspList &grasps) const {
  Grasp *grasp = best_grasp(source, destination);
  if (!grasp) {
    return false;
  }
  grasps.push_back(grasp);
  return true;
}

darrt::DiscreteGraspsPrimitive::Grasp *darrt::Push::best_grasp
(const ob::State *source, const ob::State *destination) const{
  const ObjectStateSpace *ospace =
    dspace_->getSubspace(objind_)->as<ObjectStateSpace>();

  gm::Pose spose, tpose;
  ospace->object_position(dspace_->get_state(objind_, source), spose);
  ospace->object_position(dspace_->get_state(objind_, destination), tpose);

  double dx = tpose.position.x - spose.position.x;
  double dy = tpose.position.y - spose.position.y;
  //this is what we should do but we should store it
  //i'm too lazy to implement it right now so we'll just hope
  //find the radius of the object (we assume its base is essentially a circle)
  std::vector<gm::Point> corners = darrt::corners(ospace->object().shapes[0]);
  //midpoints
  std::vector<gm::Point> midpts(6); 
  midpts[0].x = corners[0].x;
  midpts[1].y = corners[0].y;
  midpts[2].z = corners[0].z;
  midpts[3].x = corners[corners.size()-1].x;
  midpts[4].y = corners[corners.size()-1].y;
  midpts[5].z = corners[corners.size()-1].z;
  double minx = MATH_INF, maxx = -1.0*MATH_INF, miny = MATH_INF, maxy = -1.0*MATH_INF;
  for (unsigned int i = 0; i < midpts.size(); i++) {
    gm::Point tpt = transform_point(midpts[i], spose);
    if (tpt.x < minx) {
      minx = tpt.x;
    }
    if (tpt.x > maxx) {
      maxx = tpt.x;
    }
    if (tpt.y < miny) {
      miny = tpt.y;
    }
    if (tpt.y > maxy) {
      maxy = tpt.y;
    }
  }
  double R = (maxx - minx + maxy - miny)/4.0 + PUSH_OFFSET;
  //if (debug_level >= DPROPAGATE) {
  //ROS_INFO("Pushing radius (including offset) is %f, dx = %f, dy = %f", R, dx, dy);
  //}
  //double R = ospace->object().shapes[0].dimensions[0]/2.0 + PUSH_OFFSET;
  double l = sqrt(dx*dx + dy*dy);
  gm::PoseStamped rpose;
  rpose.header.frame_id = ospace->object().header.frame_id;
  rpose.header.stamp = ros::Time(0);
  if (l > EPSILON) {
    rpose.pose.position.x = spose.position.x - dx*R/l;
    rpose.pose.position.y = spose.position.y - dy*R/l;
  } else {
    //no grasp is actually needed
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("Push: already at goal (distance = %f)", l);
      ROS_INFO_STREAM("tpose =\n" << tpose << "\nspose=\n" << spose);
    }
    return NULL;
  }
  //this is really bad if the object is not in the frame expected
  //0.17 is the offset for the gripper
  //(actually, it's 0.18, but you want to be a low enough to be sure you
  //hit the plate)
  gm::Point bpt = object_bottom_point(ospace->object(), spose);
  gm::Point tpt = object_top_point(ospace->object(), spose);
  rpose.pose.position.z = spose.position.z + (tpt.z - bpt.z) + GRIPPER_LENGTH -
    PUSH_INTO_TABLE;
  geometry_msgs::Quaternion hand_ori;
  hand_ori.y = sqrt(2.0)/2.0;
  hand_ori.w = sqrt(2.0)/2.0;
  // hand_ori.x = -0.5;
  // hand_ori.y = 0.5;
  // hand_ori.z = 0.5;
  // hand_ori.w = 0.5;
  apply_yaw_to_quaternion(atan2(dy, dx), hand_ori, rpose.pose.orientation);
  //this is the actual pose of the robot's hand
  //what actually gets passed back is the pose of the object
  gm::Transform grasp_trans = 
    pose_to_transform(inverse_transform_pose(spose, rpose.pose));
  return new Grasp(grasp_trans, pushing_links_, 0.0, MAX_APPROACH_DISTANCE);
}

unsigned int darrt::Push::sample_from_grasp
(const ob::State *source, const ob::State *target, 
 const GraspWithIK &grasp_with_ik, PIList &clist) const {
  
  const ob::State *grasp = grasp_with_ik.grasp_ik;

  const ObjectStateSpace *ospace =
    dspace_->object_state_space(objind_)->as<ObjectStateSpace>();
  const ObjectStateSpace::StateType *os =
    dspace_->get_state(objind_, grasp)->as<ObjectStateSpace::StateType>();
  if (!os->support_surface) {
    pause("Called sample_from_grasp when object was not on table for state "+
	  dspace_->state_string(grasp));
    return 0;
  } 


  gm::Pose spose, tpose;
  ospace->object_position(dspace_->get_state(objind_, grasp), spose);
  ospace->object_position(dspace_->get_state(objind_, target), tpose);
  gm::Pose lpose;
  lpose.position = os->support_surface->last_point_on_surface
    (spose.position, tpose.position, table_offset_);
  //keep things at the same height
  lpose.position.z = spose.position.z;
  lpose.orientation = spose.orientation;
  if (debug_level >= DPROPAGATE) {
    ROS_INFO_STREAM("Original last point:\n" << tpose.position <<
		    "\nLast point on table:\n" << lpose.position);
  }

  ob::State *destination = ospace->allocState();
  ospace->set_object_position(destination, lpose);

  //figure out the whole path since we need the last point
  unsigned int turns = 
    (unsigned int)(ospace->distance(destination, os)/si_->getPropagationStepSize())+2;
  std::vector<ob::State *> path;
  for (unsigned int i = 0; i < turns; i++) {
    double fraction = 0.0;
    if (i > 0 && turns > 1) {
      fraction = i/(double)(turns-1);
    }
    DARRTStateSpace::StateType *pstate = 
      dspace_->allocState()->as<DARRTStateSpace::StateType>();
    if (fraction < EPSILON) {
      dspace_->copyState(pstate, grasp);
    } else {
      ospace->interpolate(os, destination, fraction, dspace_->get_state(objind_, pstate));
      if (!ik_solver_.propagate_grasp(pstate, grasp, objind_, pstate, path[i-1])) {
	break;
      }
    }
    dspace_->get_state(objind_, pstate)->as<ObjectStateSpace::StateType>()->
      touch_links = pushing_links_;
    path.push_back(pstate);
  }
  ospace->freeState(destination);
  turns = path.size();
  if (debug_level >= DPROPAGATE) {
    ROS_INFO("Pushing turns = %u", turns);
  }
  if (turns <= 1) {
    //can't actually push
    return 0;
  }
    
  StatePathInstance *pi = new StatePathInstance(this, name(), path);
  std::string objid = ospace->getName();
  //disable collisions:
  //objects+support surface
  //object+gripper
  //gripper+table
  an::OrderedCollisionOperations ops;
  collisionOperations(source, ops);
  pi->add_allowed_collisions(ops);
  an::CollisionOperation op;
  op.operation = op.DISABLE;
  // for (unsigned int i = 0; i < dspace_->object_indexes().size(); i++) {
  //   unsigned int ind = dspace_->object_indexes().at(i);
  //   const SupportSurface *ssf = dspace_->get_state(ind, source)->
  //     as<ObjectStateSpace::StateType>()->support_surface;
  //   if (ssf) {
  //     op.object1 = dspace_->getSubspace(ind)->getName();
  //     op.object2 = ssf->name();
  //     pi->add_allowed_collision(op);
  //   }
  // }
  for (size_t i = 0; i < grasp_with_ik.grasp->touch_links.size(); i++){
    op.object1 = grasp_with_ik.grasp->touch_links[i];
    op.object2 = os->support_surface->name();
    pi->add_allowed_collision(op);
    op.object2 = objid;
    pi->add_allowed_collision(op);
  }
  // //there are multiple ik solutions for the last pose (since we try to keep the
  // //object moving along a straight line) so we will find one as we propagate...
  // //don't set it here
  // pi->destination()->as<DARRTStateSpace::StateType>()->
  //   set_inactive(dspace_->robot_index());
  // pi->destination()->as<DARRTStateSpace::StateType>()->set_invalid();
  pi->set_turns(turns);
  clist.push_back(pi);
  return turns;
}

void darrt::Push::extraGraspPoses(const GraspWithIK &grasp,
				  const gm::PoseStamped &grasp_pose,
				  const ob::State *source,
				  const ob::State *target,
				  std::vector<gm::PoseStamped> &extra_poses) const {
  //make sure we can make the pre-grasp
  DiscreteGraspsPrimitive::extraGraspPoses(grasp, grasp_pose, source, target, extra_poses);
  
  //also make sure we can make at least one unit of progress along the push

  const ObjectStateSpace *ospace =
    dspace_->object_state_space(objind_)->as<ObjectStateSpace>();
  const ObjectStateSpace::StateType *os =
    dspace_->get_state(objind_, grasp.grasp_ik)->as<ObjectStateSpace::StateType>();

  if (!os->support_surface) {
    pause("Called push without object on support surface!");
    return;
  }

  gm::Pose spose, tpose;
  ospace->object_position(os, spose);
  ospace->object_position(dspace_->get_state(objind_, target), tpose);

  gm::Pose lpose;
  lpose.position = os->support_surface->last_point_on_surface
    (spose.position, tpose.position, table_offset_);
  //keep things at the same height
  lpose.position.z = spose.position.z;
  lpose.orientation = spose.orientation;

  double dx = lpose.position.x - spose.position.x;
  double dy = lpose.position.y - spose.position.y;
  double dist = sqrt(dx*dx + dy*dy);

  unsigned int turns = 
    (unsigned int)(dist/si_->getPropagationStepSize())+2;
  
  double fraction = 1.0/(double)(turns-1);

  gm::PoseStamped new_object_pose_w;
  new_object_pose_w.header.frame_id = ospace->object().header.frame_id;
  new_object_pose_w.pose = spose;
  new_object_pose_w.pose.position.x += fraction*dx;
  new_object_pose_w.pose.position.y += fraction*dy;

  gm::PoseStamped orig_object_pose_w;
  orig_object_pose_w.header = new_object_pose_w.header;
  orig_object_pose_w.pose = spose;

  //in the IK root frame
  gm::PoseStamped new_object_pose = ik_solver_.transform_pose_stamped
    (grasp_pose.header.frame_id, new_object_pose_w, grasp.grasp_ik);
  
  gm::PoseStamped orig_object_pose = ik_solver_.transform_pose_stamped
    (grasp_pose.header.frame_id, orig_object_pose_w, grasp.grasp_ik);


  //the frame free grasp
  gm::Pose ff_grasp = inverse_transform_pose(orig_object_pose.pose, grasp_pose.pose);

  gm::Pose origin;
  origin.orientation.w = 1.0;

  //propagate the grasp

  gm::PoseStamped propagated_grasp;
  propagated_grasp.header = grasp_pose.header;
  propagated_grasp.pose = transform_pose(inverse_transform_pose(origin, ff_grasp), 
					 new_object_pose.pose);
  extra_poses.push_back(propagated_grasp);
}

//this won't sample the edge of the table... fix that!
void darrt::Push::projectionFunction(const ob::State *from, ob::State *sample) const {
  //really should put down anything we are holding...
  if (debug_level >= DRRT) {
    ROS_INFO("Using push projection.");
  }

  gm::Pose spose;
  ObjectStateSpace::StateType *os = dspace_->get_state(objind_, sample)->as<ObjectStateSpace::StateType>();
  const ObjectStateSpace::StateType *fs = dspace_->get_state(objind_, from)->as<ObjectStateSpace::StateType>();
  const ObjectStateSpace *ospace = dspace_->getSubspace(objind_)->as<ObjectStateSpace>();
  ospace->object_position(os, spose);
  gm::Pose fpose;
  ospace->object_position(fs, fpose);
  spose.position.z = fpose.position.z;
  spose.orientation = fpose.orientation;
  ospace->set_object_position(os, spose);
  //does this mean it is now on a surface?
  if (!os->attach_link.size()) {
    if (fs->support_surface && ospace->object().canSupport(fs->support_surface)) {
      if (!fs->support_surface->supporting(ospace->object(), spose)) {
	spose = ospace->object().sampleStablePose(fs->support_surface);
	spose.orientation = fpose.orientation;
	ospace->set_object_position(os, spose);
      }
      os->support_surface = fs->support_surface;
      os->attach_link = "";
      os->touch_links.clear();
    } else {
      for (unsigned int i = 0; i < dspace_->support_surfaces().size(); i++) {
	const SupportSurface *surf = dspace_->support_surfaces().at(i);
	if (surf->supporting(ospace->object(), spose)) {
	  os->support_surface = surf;
	  os->attach_link = "";
	  os->touch_links.clear();
	}
      }
    }
  }
	  
    
  //move any attached objects to a surface
  //also moves the object to a surface if it was
  //attached in the original state
  for (unsigned int i = 0; i < dspace_->object_indexes().size(); i++) {
    unsigned int ind = dspace_->object_indexes().at(i);
    const ObjectStateSpace *ospace = dspace_->getSubspace(ind)->
      as<ObjectStateSpace>();
    ObjectStateSpace::StateType *os = dspace_->get_state(ind, sample)->as<ObjectStateSpace::StateType>();
    if (!dspace_->get_state(ind, from)->as<ObjectStateSpace::StateType>()->support_surface) {
      const DARRTObject &obj = ospace->object();
      std::vector<const SupportSurface *> surfs(dspace_->support_surfaces().size());
      unsigned int nsurfs = 0;
      for (unsigned int j = 0; j < dspace_->support_surfaces().size(); j++) {
	if (obj.canSupport(dspace_->support_surfaces().at(j))) {
	  surfs[nsurfs] = dspace_->support_surfaces().at(j);
	  nsurfs++;
	}
      }
      if (nsurfs == 0) {
	pause("No surface that can support object "+obj.id);
	continue;
      }
      unsigned int r = rand() % nsurfs;
      gm::Pose tpose = obj.sampleStablePose(surfs[r]);
      ospace->set_object_position(os, tpose);
      os->attach_link = "";
      os->touch_links.clear();
      os->support_surface = surfs[r];
    } else if (ind != objind_) {
      ospace->copyState(os, dspace_->get_state(ind, from));
    }
  }
}

darrt::DARRTProjectionFunction darrt::Push::getProjectionFunction() const {
  return boost::bind(&darrt::Push::projectionFunction, this, _1, _2);
}


darrt::PickupPrimitive::PickupPrimitive
(std::string arm_name, std::string objid, const RigidGraspList &grasps, 
 const gm::Vector3 &lift, double min_lift_distance, double desired_lift_distance) : 
  DiscreteGraspsPrimitive("Pickup", arm_name, objid) {
  grasps_ = grasps;
  lift_ = lift;
  min_lift_distance_ = min_lift_distance;
  desired_lift_distance_ = desired_lift_distance;
}

darrt::Primitive *darrt::PickupPrimitive::copy() const {
  PickupPrimitive *p = new PickupPrimitive(arm_name_, objid_, grasps_, lift_, min_lift_distance_,
					   desired_lift_distance_);
  if (is_setup()) {
    p->setup(si_);
  }
  return p;
}


bool darrt::PickupPrimitive::setup(const oc::SpaceInformation *si) {
  if (!DiscreteGraspsPrimitive::setup(si)) {
    return false;
  }
  return true;
}


// bool darrt::PickupPrimitive::active(const ob::State *source,
// 				    const ob::State *target) const {
//   //are we trying to move this object?
//   if (dspace_->near_object_states(objind_, source, target)) {
//     if (debug_level >= DPROPAGATE) {
//       ROS_INFO("%s does not move", objid_.c_str());
//     }
//     return false;
//   }
//   const ObjectStateSpace::StateType *os = 
//     dspace_->get_state(objind_, source)->as<ObjectStateSpace::StateType>();
    
//   //is this object sitting on a surface?
//   const SupportSurface *surface = os->support_surface;
//   if (!surface) {
//     if (debug_level >= DPROPAGATE) {
//       ROS_INFO("%s is not on a surface", objid_.c_str());
//     }
//     return false;
//   }

//   //should check that we're not already holding something?

//   //check if any grasps are feasible
//   const ObjectStateSpace *ospace = 
//     dspace_->getSubspace(objind_)->as<ObjectStateSpace>();
//   gm::Pose opose;
//   ospace->object_position(os, opose);
//   for (unsigned int i = 0; i < grasps_.size(); i++) {
//     if (grasps_[i].min_distance_from_surface >= 0) {
//       gm::Pose grasp = get_grasp_pose(grasps_[i].grasp, opose);
//       double dist = surface->distance(grasp.position);
//       // if (debug_level >= DPROPAGATE) {
//       // 	ROS_INFO("Grasp %u at (%f, %f, %f) is distance %f away from surface (requires > %f)", i, grasp.position.x, 
//       // 		 grasp.position.y, grasp.position.z,
//       // 		 dist, grasps_[i].min_distance_from_surface);
//       // }
//       if (dist < grasps_[i].min_distance_from_surface) {
// 	continue;
//       }
//     }
//     GraspWithIK gs;
//     if (!grasp_ik(objind_, source, target, grasps_[i], gs)) {
//       //no IK solution for this grasp
//       continue;
//     }
//     if (!ik_solver_.feasible(gs.grasp_ik, grasps_[i].min_distance, gs.direction_to_pregrasp)) {
//       dspace_->freeState(gs.grasp_ik);
//       //no IK solution for the pregrasp
//       continue;
//     }
//     //check IK for the lift if we need one
//     bool has_lift = min_lift_distance_ < 0.00001 || ik_solver_.feasible(gs.grasp_ik, min_lift_distance_, lift_);
//     dspace_->freeState(gs.grasp_ik);
//     if (has_lift) {
//       return true;
//     }
//   }
//   return false;
// }

darrt::DiscreteGraspsPrimitive::Grasp *darrt::PickupPrimitive::best_grasp
(const ob::State *source, const ob::State *destination) const{
  GraspList gl;
  if (!grasps(source, destination, gl, true) || !gl.size()) {
    return NULL;
  }
  for (unsigned int i = 1; i < gl.size(); i++) {
    delete gl[i];
  }
  return gl[0];
}

bool darrt::PickupPrimitive::grasps
(const ob::State *source, const ob::State *destination,
 GraspList &grasplist) const {
  return grasps(source, destination, grasplist, false);
}

bool darrt::PickupPrimitive::grasps
(const ob::State *source, const ob::State *destination,
 GraspList &grasps, bool return_first_found) const {

  const ObjectStateSpace::StateType *os = 
    dspace_->get_state(objind_, source)->as<ObjectStateSpace::StateType>();

  //is this object already grasped?
  if (os->attach_link.size()) {
    return false;
  }

  const DARRTStateSpace::StateType *d = destination->as<DARRTStateSpace::StateType>();
  const ObjectStateSpace::StateType *dos = 
    dspace_->get_state(objind_, d)->as<ObjectStateSpace::StateType>();

  //is this object sitting on a surface?
  const SupportSurface *surface = os->support_surface;

  //now check the destination - is the object already grasped?
  if (dos->attach_link.size()) {
    //use the grasp that the robot is already using
    if (debug_level >= DDISTANCE) {
      ROS_INFO("Pickup: using grasp used at destination: %s", dspace_->state_string(destination).c_str());
    }
    const ObjectStateSpace *ospace = 
      dspace_->object_state_space(objind_)->as<ObjectStateSpace>();
    gm::Pose opose;
    ospace->object_position(dos, opose);
    gm::PoseStamped grps;
    ik_solver_.arm_space()->forward_kinematics(ik_solver_.arm_state(destination), grps);
    gm::PoseStamped grpose = ik_solver_.transform_pose_stamped(ospace->object().header.frame_id, 
							       grps, destination);
    gm::Pose grasp = inverse_transform_pose(opose, grpose.pose);
    //try to find the matching grasp - use the minimum angle since that gets messed up
    double min_angular_distance = 1;
    int grasp_ind = -1;
    for (unsigned int i = 0; i < grasps_.size(); i++) {
      double mda = darrt::distance(grasps_[i].grasp.rotation, grasp.orientation);
      if (fabs(mda) < min_angular_distance &&
	  darrt::distance(vector_to_point(grasps_[i].grasp.translation), grasp.position) < 0.05) {
	min_angular_distance = fabs(mda);
	grasp_ind = i;
      }
    }
    if (grasp_ind < 0) {
      ROS_ERROR_STREAM("Unable to find matching grasp for\n" << grasp);
      ROS_ERROR("Destination is %s", dspace_->state_string(destination).c_str());
      std::string buff;
      //getline(std::cin, buff);
      RigidGrasp *rg = new RigidGrasp(pose_to_transform(grasp), dos->touch_links,
				      dos->attach_link, 0, MAX_APPROACH_DISTANCE);
      grasps.push_back(rg);
      return true;
    }

    if (surface && grasps_[grasp_ind].min_distance_from_surface >= 0) {
      gm::Pose spose;
      ospace->object_position(os, spose);
      gm::Pose actual_grasp_pose = get_grasp_pose(grasps_[grasp_ind].grasp, spose); 
      if (surface->distance(actual_grasp_pose.position) < grasps_[grasp_ind].min_distance_from_surface) {
	return true;
      }
    }
    RigidGrasp *rg = new RigidGrasp(pose_to_transform(grasp), dos->touch_links,
				    dos->attach_link, grasps_[grasp_ind].min_distance,
				    grasps_[grasp_ind].desired_distance,
				    grasps_[grasp_ind].min_distance_from_surface);
    grasps.push_back(rg);
    return true;
  }

  // if (!surface) {
  //   if (debug_level >= DDISTANCE) {
  //     ROS_INFO("Pickup: Call to grasps when object is not on surface");
  //   }
  //   return false;
  // }

  const ObjectStateSpace *ospace = 
    dspace_->getSubspace(objind_)->as<ObjectStateSpace>();
  gm::Pose opose;
  ospace->object_position(os, opose);

  for (size_t i = 0; i < grasps_.size(); i++) {
    if (surface && grasps_[i].min_distance_from_surface >= 0) {
      gm::Pose grasp = get_grasp_pose(grasps_[i].grasp, opose);
      if (surface->distance(grasp.position) < 
	  grasps_[i].min_distance_from_surface) {
	continue;
      }
    }
    grasps.push_back(new RigidGrasp(grasps_[i]));
    if (return_first_found) {
      return true;
    }
  }
  return true;
}


unsigned int darrt::PickupPrimitive::sample_from_grasp
(const ob::State *source, const ob::State *destination,
 const GraspWithIK &grasp_with_ik,PIList &clist) const {
  const ob::State *grasp = grasp_with_ik.grasp_ik;
  const ObjectStateSpace::StateType *os = 
    source->as<DARRTStateSpace::StateType>()->
    as<ObjectStateSpace::StateType>(objind_);
  if (os->attach_link.size()) {
    pause("Called pickup sample from grasp but object " + makestring(objind_)
	  + " i is already attached " + "in state " +dspace_->state_string(source));
    return 0;
  }

  const RigidGrasp *rg = 
    dynamic_cast<const RigidGrasp *>(grasp_with_ik.grasp);
  if (!rg) {
    pause("Cannot cast grasp to rigid grasp in pickup...");
  }
  std::string al = rg->attach_link;

  ob::State *attached = dspace_->allocState();
  dspace_->copyState(attached, grasp);
  ObjectStateSpace::StateType *opis =
    dspace_->get_state(objind_, attached)->as<ObjectStateSpace::StateType>();
  opis->support_surface = NULL;
  opis->attach_link = al;
  opis->touch_links = rg->touch_links;

  const ObjectStateSpace *ospace =
    dspace_->getSubspace(objind_)->as<ObjectStateSpace>();
  gm::Pose lpose;
  ospace->object_position(dspace_->get_state(objind_, attached), lpose);

  lpose.position.x += desired_lift_distance_*lift_.x;
  lpose.position.y += desired_lift_distance_*lift_.y;  
  lpose.position.z += desired_lift_distance_*lift_.z;

  ob::State *objdest = ospace->allocState();
  ospace->set_object_position(objdest, lpose);

  //figure out the whole path since we need the last point
  unsigned int turns =
    (unsigned int)(ospace->distance(objdest, opis)/(si_->getPropagationStepSize()))+1;
  std::vector<ob::State *> path;
  double fraction = 0.0;
  for (unsigned int i = 0; i < turns; i++) {
    if (i == 0) {
      path.push_back(attached);
      continue;
    }
    if (i > 0 && turns > 1) {
      fraction = i/(double)(turns-1);
    }
    ob::State *pstate = dspace_->allocState();
    ospace->interpolate(opis, objdest, fraction, dspace_->get_state(objind_, pstate));
    if (!ik_solver_.propagate_grasp(pstate, attached, objind_, pstate, path[i-1])) {
      break;
    }
    path.push_back(pstate);
  }
  if (fraction < 0.999999) {
    double distance = fraction*desired_lift_distance_;
    if (distance < min_lift_distance_ || min_lift_distance_ < -0.0001) {
      if (debug_level >= DPROPAGATE) {
	ROS_INFO("Unable to propagate lift far enough.  Distance is %f, minimum %f, fraction %f",
		 distance, min_lift_distance_, fraction);
      }
      for (unsigned int i = 0; i < path.size(); i++) {
	dspace_->freeState(path[i]);
      }
      return 0;
    }
  }
  turns = path.size();
  StatePathInstance *pi = new StatePathInstance(this, name(), path);

  //when lifting we allow
  //plate + touching_links
  //plate + table
  //touching_links+table
  std::string objid = dspace_->getSubspace(objind_)->getName();

  an::OrderedCollisionOperations ops;
  collisionOperations(source, ops);
  pi->add_allowed_collisions(ops);
  an::CollisionOperation op;
  op.operation = op.DISABLE;
  // for (unsigned int i = 0; i < dspace_->object_indexes().size(); i++) {
  //   unsigned int ind = dspace_->object_indexes().at(i);
  //   const SupportSurface *ssf = dspace_->get_state(ind, source)->
  //     as<ObjectStateSpace::StateType>()->support_surface;
  //   if (ssf) {
  //     op.object1 = dspace_->getSubspace(ind)->getName();
  //     op.object2 = ssf->name();
  //     pi->add_allowed_collision(op);
  //   }
  // }

  for (size_t j = 0; j < grasp_with_ik.grasp->touch_links.size(); j++){
    op.object1 = objid;
    op.object2 = grasp_with_ik.grasp->touch_links[j];
    pi->add_allowed_collision(op);
    if (os->support_surface) {
      op.object1 = os->support_surface->name();
      pi->add_allowed_collision(op);
    }
  }

  ospace->freeState(objdest);

  pi->set_turns(turns);
  clist.push_back(pi);
  return turns;
}

void darrt::PickupPrimitive::extraGraspPoses(const GraspWithIK &grasp,
					     const gm::PoseStamped &grasp_pose,
					     const ob::State *source,
					     const ob::State *target,
					     std::vector<gm::PoseStamped> &extra_poses) const {
  //make sure we can make the pre-grasp
  DiscreteGraspsPrimitive::extraGraspPoses(grasp, grasp_pose, source, target, extra_poses);
  
  if (min_lift_distance_ <= 0) {
    return;
  }

  //also make sure we can make at least one unit of progress along the push
  const ObjectStateSpace *ospace =
    dspace_->object_state_space(objind_)->as<ObjectStateSpace>();
  gm::Pose spose, tpose;
  ospace->object_position(dspace_->get_state(objind_, grasp.grasp_ik), spose);
  
  //this is the new position of the object
  gm::PoseStamped new_object_pose_w;
  new_object_pose_w.header.frame_id = ospace->object().header.frame_id;
  new_object_pose_w.pose = spose;
  new_object_pose_w.pose.position.x += min_lift_distance_*lift_.x;
  new_object_pose_w.pose.position.y += min_lift_distance_*lift_.y;
  new_object_pose_w.pose.position.z += min_lift_distance_*lift_.z;
  
  //in the IK root frame
  gm::PoseStamped new_object_pose = ik_solver_.transform_pose_stamped
    (grasp_pose.header.frame_id, new_object_pose_w, grasp.grasp_ik);
  
  gm::PoseStamped orig_object_pose_w;
  orig_object_pose_w.header = new_object_pose_w.header;
  orig_object_pose_w.pose = spose;
  gm::PoseStamped orig_object_pose = ik_solver_.transform_pose_stamped
    (grasp_pose.header.frame_id, orig_object_pose_w, grasp.grasp_ik);

  //the frame free grasp
  gm::Pose ff_grasp = inverse_transform_pose(orig_object_pose.pose, grasp_pose.pose);

  gm::Pose origin;
  origin.orientation.w = 1.0;

  //propagate the grasp
  gm::PoseStamped propagated_grasp;
  propagated_grasp.header = grasp_pose.header;
  propagated_grasp.pose = transform_pose(inverse_transform_pose(origin, ff_grasp), 
					 new_object_pose.pose);

  extra_poses.push_back(propagated_grasp);
}


bool darrt::PickupPrimitive::execute
(const std::vector<const ob::State *> &path) const {
  close_gripper();
  return execute_path(path);
}


darrt::PlacePrimitive::PlacePrimitive
(std::string arm_name, const gm::Vector3 &approach_direction,
 double min_distance, double desired_distance) :
  TransferPrimitive("Place"), PR2Arm(arm_name) {
  approach_direction_ = approach_direction;
  min_distance_ = min_distance;
  desired_distance_ = desired_distance;
}

bool darrt::PlacePrimitive::setup(const oc::SpaceInformation *si) {
  if (!TransferPrimitive::setup(si)) {
    return false;
  }
  if (!setup_arm(si)) {
    return false;
  }

  rigid_transfer_ = NULL;
  for (unsigned int i = 0; i < dspace_->primitives().size(); i++) {
    rigid_transfer_ = dynamic_cast<const RigidTransfer *>(dspace_->primitives().at(i));
    if (rigid_transfer_) {
      break;
    }
  }
  if (!rigid_transfer_) {
    ROS_ERROR("Unable to find rigid transfer.");
    return false;
  }
  return true;
}

bool darrt::PlacePrimitive::useful(const ob::State *source, 
				   const ob::State *destination) const {
  //ASSUMES TABLES HAVE Z AS THE NORMAL

  //useful when we are actually trying to put something we're holding down 
  //on a table
  //OR we are holding something and trying to transit or move something else
  std::vector< pair<unsigned int, unsigned int> > placed_objects;

  return placedObjects(source, destination, placed_objects) &&
    source->as<DARRTStateSpace::StateType>()->valid();

  //now we always have a place

  // Place *place = get_place(source, destination);
  // if (!place) {
  //   return false;
  // }
  // return true;
  //check that there is a pre-place
  // bool preplace = true;
  // if (!ik_solver_.feasible(place->place, min_distance_, 
  // 			   place->direction_to_preplace, source)) {
  //   preplace = false;
  // }
  // dspace_->freeState(place->place);
  // delete place;
  // return preplace;
}


unsigned int darrt::PlacePrimitive::sampleTo(const ob::State *source,
					     const ob::State *target, 
					     PIList &clist) const {
  Place *place = get_place(source, target);
  if (!place) {
    return 0;
  }

  const ObjectStateSpace *ospace = dspace_->getSubspace(place->objind)->
    as<ObjectStateSpace>();
  //figure out how far back we can go
  unsigned int turns = 
    (unsigned int)(desired_distance_/(si_->getPropagationStepSize()))+1;
  gm::Pose apose;
  ospace->object_position(dspace_->get_state(place->objind, place->place), apose);
  apose.position.x += desired_distance_*place->direction_to_preplace.x;
  apose.position.y += desired_distance_*place->direction_to_preplace.y;
  apose.position.z += desired_distance_*place->direction_to_preplace.z;
  ob::State *objdest = ospace->allocState();
  ospace->set_object_position(objdest, apose);
  std::vector<ob::State *> reversed_path;
  for (unsigned int i = 0; i < turns; i++) {
    if (i == 0) {
      reversed_path.push_back(place->place);
      continue;
    }
    double fraction = 0.0;
    if (turns > 1) {
      fraction = i/(double)(turns-1);
    }
    ob::State *pstate = dspace_->allocState();
    ospace->interpolate(dspace_->get_state(place->objind, place->place), objdest,
			fraction, dspace_->get_state(place->objind, pstate));
    if (!ik_solver_.propagate_grasp(pstate, place->place, place->objind, pstate, 
				    reversed_path[i-1])) {
      break;
    }
    reversed_path.push_back(pstate);
  }
  //reverse the path
  std::vector<ob::State *> path(reversed_path.size(), NULL);
  for (unsigned int i = 0; i < reversed_path.size(); i++) {
    path[i] = reversed_path[reversed_path.size()-1-i];
  }
  turns = path.size();

  
  StatePathInstance *pi = new StatePathInstance(this, name(), path);
  pi->set_turns(turns);

  //add the ordered collisions
  std::string objid = dspace_->object_state_space(place->objind)->getName();
  std::string tableid = 
    dspace_->support_surfaces().at(place->tableind)->name();
  
  an::CollisionOperation op;
  op.object1 = objid;
  op.object2 = tableid;
  op.operation = op.DISABLE;
  pi->add_allowed_collision(op);

  std::vector<std::string> touch_links = dspace_->get_state
    (place->objind, source)->as<ObjectStateSpace::StateType>()->touch_links;

  for (size_t i = 0; i < touch_links.size(); i++) {
    op.object1 = objid;
    op.object2 = touch_links[i];
    pi->add_allowed_collision(op);
    op.object1 = tableid;
    pi->add_allowed_collision(op);
  }

  an::OrderedCollisionOperations ops;
  collisionOperations(source, ops);
  pi->add_allowed_collisions(ops);

  turns += rigid_transfer_->sampleMove(source, path[0], clist);
  clist.push_back(pi);
  ROS_INFO("The set of primitives returned from place is:");
  for (unsigned int i = 0; i < clist.size(); i++) {
    ROS_INFO("%s", clist[i]->str().c_str());
  }
  return turns;
}

bool darrt::PlacePrimitive::placedObjects
(const ob::State *source, const ob::State *destination, 
 std::vector< pair<unsigned int, unsigned int> > &placed_objects) const {
  const DARRTStateSpace::StateType *s = 
    source->as<DARRTStateSpace::StateType>();
  std::vector<unsigned int> attached_objects;
  for (size_t i = 0; i < dspace_->object_indexes().size(); i++) {
    unsigned int ind = dspace_->object_indexes().at(i);
    const ObjectStateSpace::StateType *os = 
      s->as<ObjectStateSpace::StateType>(ind);
    if (!os->support_surface) {
      attached_objects.push_back(ind);
    }
  }

  while (attached_objects.size()) {
    int rand_obj = rand() % attached_objects.size();
    unsigned int attached_object = attached_objects[rand_obj];
    const ObjectStateSpace *ospace = 
      dspace_->object_state_space(attached_object)->as<ObjectStateSpace>();
    if (dspace_->near_object_states(attached_object, source, destination)) {
      attached_objects[rand_obj] = attached_objects.back();
      attached_objects.pop_back();
      continue;
    }
    //are we putting this object on a table?
    gm::Pose dpose;
    ospace->object_position(dspace_->get_state(attached_object, destination),
			    dpose);
    gm::Point tpt = object_bottom_point(ospace->object(), dpose);
    for (unsigned int i = 0; i < dspace_->support_surfaces().size(); i++) {
      if (dspace_->support_surfaces().at(i)->
	  on_surface(tpt, TABLE_CONTAINS)){
	if (debug_level >= DPROPAGATE) {
	  ROS_INFO("Attempting to place on table %s", 
		   dspace_->support_surfaces().at(i)->name().c_str());
	}
	placed_objects.push_back(make_pair(attached_object, i));
      }
    }
    attached_objects[rand_obj] = attached_objects.back();
    attached_objects.pop_back();
  }
  return placed_objects.size() > 0;
}

darrt::PlacePrimitive::Place *darrt::PlacePrimitive::get_place
(const ob::State *source, const ob::State *destination) const {

  std::vector< pair<unsigned int, unsigned int> > placed_objects;
  if (!placedObjects(source, destination, placed_objects)) {
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("Object not being placed on surface");
    }
    return NULL;
  }

  for (unsigned int i = 0; i < placed_objects.size(); i++) {
    unsigned int attached_object = placed_objects[i].first;
    const ObjectStateSpace *ospace = 
      dspace_->object_state_space(attached_object)->as<ObjectStateSpace>();
    gm::Pose dpose;
    ospace->object_position(dspace_->get_state(attached_object, destination),
			    dpose);
    //is there an IK solution for this?
    ob::State *ik_result = NULL;
    const BaseIK *base_ik = rigid_transfer_->base_ik();
    if (!base_ik) {
      ik_result = ik_solver_.propagate_grasp(destination, source, 
					     attached_object, NULL);
    } else {
      ik_result = dspace_->allocState();
      dspace_->copyState(ik_result, source);
      ospace->set_object_position(dspace_->get_state(attached_object, 
						     ik_result), dpose);
      gm::PoseStamped wps = ik_solver_.propagate_grasp(destination, source, 
						       attached_object);
      bool found_ik = false;
      for (unsigned int j = 0; j < BASE_TRIES; j++) {
	if (j > 0) {
	  base_ik->get_base_pose_for_wrist(wps.pose, ik_result);
	}
	gm::PoseStamped ps = ik_solver_.transform_pose_stamped
	  (arm_space_->root_name(), wps, ik_result);
	if (arm_space_->inverse_kinematics(ps, arm_state(ik_result), 
					   arm_state(source))) {
	  found_ik = true;
	  break;
	}
	if (arm_space_->inverse_kinematics(ps, arm_state(ik_result))) {
	  found_ik = true;
	  break;
	}
      }
      if (!found_ik) {
	dspace_->freeState(ik_result);
	ik_result = NULL;
      }
    }
    
    if (!ik_result) {
      if (debug_level >= DRRT) {
	ROS_INFO("Unable to find IK solution for place");
      }
      continue;
    }
    ObjectStateSpace::StateType *oop = dspace_->get_state
      (attached_object, ik_result)->as<ObjectStateSpace::StateType>();
    oop->support_surface = dspace_->support_surfaces().at(i);
    oop->attach_link = "";
    //leave the touch links for now
    //oop->touch_links.clear();
    gm::Vector3Stamped napp;
    napp.header.frame_id = ik_solver_.transformer().world_frame_id();
    napp.vector.x = -1.0*approach_direction_.x;
    napp.vector.y = -1.0*approach_direction_.y;      
    napp.vector.z = -1.0*approach_direction_.z;
    napp = ik_solver_.transform_vector_stamped(arm_space_->root_name(), napp, 
					       source); 
    return new Place(ik_result, napp.vector, attached_object, 
		     placed_objects[i].second);
  }
  if (debug_level >= DPROPAGATE) {
    ROS_INFO("No IK solution");
  }
  return NULL;				    
}




gm::Pose darrt::PlacePrimitive::get_object_pose_on_table
(const an::CollisionObject &obj, const gm::Point &point_on_table,
 const gm::Quaternion &object_orientation) const {
  //ASSUMES TABLES ARE IN THE XY PLANE!!!!
  gm::Pose trans;
  trans.orientation = object_orientation;
  gm::Point tpt = object_top_point(obj, trans);
  gm::Point bpt = object_bottom_point(obj, trans);
  double height = tpt.z - bpt.z;
  gm::Pose objpose;
  objpose.position.x = point_on_table.x;
  objpose.position.y = point_on_table.y;
  objpose.position.z = point_on_table.z + height;
  objpose.orientation = object_orientation;
  return objpose;
}

bool darrt::PlacePrimitive::execute
(const std::vector<const ob::State *> &path) const {
  if (!execute_path(path)) {
    return false;
  }
  open_gripper();
  return true;
}

darrt::StatePathInstance::StatePathInstance
(const Primitive *prim, std::string type, const std::vector<ob::State *> &state_path) :
  CollisionAwarePrimitiveInstance(prim, type, state_path[0], state_path[state_path.size()-1]) {
  
  path_ = state_path;
  curr_state_ = new int();
  *curr_state_ = -1;
  turns_ = path_.size();
  dspace_ = prim->space_information()->getStateSpace()->as<DARRTStateSpace>();
}

darrt::PrimitiveInstance *darrt::StatePathInstance::copy() const {
  std::vector<ob::State *> cpath(path_.size(), NULL);
  for (unsigned int i = 0; i < cpath.size(); i++) {
    cpath[i] = dspace_->allocState();
    dspace_->copyState(cpath[i], path_[i]);
  }
  StatePathInstance *pi = new StatePathInstance(prim_, type_, cpath);
  pi->set_allowed_collisions(allowed_collisions);
  pi->set_turns(turns_);
  return pi;
}

bool darrt::StatePathInstance::contains(const ob::State *state) const {
  if (debug_level == DBETWEEN) {
    ROS_INFO("In state path, curr state is %d", (*curr_state_));
  }
  if ((*curr_state_) >= static_cast<int>(path_.size())) {
    return false;
  }
  return dspace_->near_states(state, path_[*curr_state_]);
}

bool darrt::StatePathInstance::propagate
(const ob::State *state, const double duration, ob::State *result) const {
  if ((*curr_state_) < 0) {
    if (duration < 0) {
      (*curr_state_) = path_.size()-1;
    } else {
      (*curr_state_) = 0;
    }
  }
  if ((*curr_state_) >= static_cast<int>(path_.size()) || (*curr_state_) < 0) {
    pause("Call to state path instance when curr state is negative or greater than the number of states on path!");
    dspace_->copyState(result, state);
    return false;
  }
  dspace_->copyState(result, path_[*curr_state_]);
  if (duration < 0) {
    (*curr_state_)--;
  } else {
    (*curr_state_)++;
  }
  result->as<DARRTStateSpace::StateType>()->set_control(prim_, type_);
  
  result->as<DARRTStateSpace::StateType>()->set_allowed_collisions(allowed_collisions);

  return true;
}

darrt::StatePathInstance::~StatePathInstance() {
  for (unsigned int i = 0; i < path_.size(); i++) {
    dspace_->freeState(path_[i]);
  }
  delete curr_state_;
}

darrt::UseSpatula::UseSpatula(std::string arm_name,
			      std::string spatid,
			      std::string objid,
			      std::string blockid,
			      const PickupPrimitive::RigidGraspList &spatula_grasps,
			      const gm::Vector3 &spatula_lift,
			      double horizontal_approach_distance,
			      double vertical_approach_distance,
			      double lift_distance,
			      double spatula_min_lift_distance,
			      double spatula_desired_lift_distance) :
  TransferPrimitive("UseSpatula-"+arm_name+"-"+spatid+"-"+blockid), 
  PR2Arm(arm_name),
  pickup_(arm_name, spatid, spatula_grasps, spatula_lift,
	  spatula_min_lift_distance, spatula_desired_lift_distance) {
  spatid_ = spatid;
  objid_ = objid;
  blockid_ = blockid;
  push_ = NULL;
  rigid_transfer_ = NULL;
  base_transit_ = NULL;
  retreat_ = NULL;
  obj_ = NULL;
  block_ = NULL;
  spatula_ = NULL;
  horizontal_approach_distance_ = horizontal_approach_distance;
  vertical_approach_distance_ = vertical_approach_distance;
  lift_distance_ = lift_distance;
  ROS_INFO("horizontal approach distace = %f, vertical approach distance = %f, lift distance = %f",
	   horizontal_approach_distance_, vertical_approach_distance_, lift_distance_);
  arm_transit_name_ = "ArmTransit-"+arm_name;
  approach_name_ = "Approach-"+arm_name;
}

darrt::Primitive *darrt::UseSpatula::copy() const {
  UseSpatula *p = new UseSpatula(arm_name_, spatid_, objid_, blockid_, 
				 pickup_.getGrasps(), pickup_.getLift(),
				 horizontal_approach_distance_,
				 vertical_approach_distance_,
				 lift_distance_, pickup_.getMinLiftDistance(),
				 pickup_.getDesiredLiftDistance());
  if (is_setup()) {
    p->setup(si_);
  }
  return p;
}

bool darrt::UseSpatula::setup(const oc::SpaceInformation *si) {

  is_setup_ = false;
  if (!TransferPrimitive::setup(si)) {
    return false;
  }
  if (!setup_arm(si)) {
    return false;
  }
  //find the object and spatula indices
  //here
  objind_ = dspace_->getSubspaceIndex(objid_);
  spatind_ = dspace_->getSubspaceIndex(spatid_);
  blockind_ = dspace_->getSubspaceIndex(blockid_);

  ROS_INFO("DEBUG LEVEL = %d", debug_level);

  if (debug_level >= DPROPAGATE) {
    ROS_INFO("objind = %u, spatind = %u, blockind = %u", 
	     objind_, spatind_, blockind_);
  }

  obj_ = &(dspace_->getSubspace(objind_)->as<ObjectStateSpace>()->object());
  block_ = &(dspace_->getSubspace(blockind_)->as<ObjectStateSpace>()->object());  
  spatula_ = &(dspace_->getSubspace(spatind_)->as<ObjectStateSpace>()->object());

  if (obj_->shapes.size() < 1 || obj_->shapes[0].type != an::Shape::CYLINDER) {
    pause("UseSpatula: Wrong type for object.  Should be a cylinder.");
    return false;
  }
  if (obj_->shapes.size() > 1) {
    ROS_WARN("UseSpatula: Too many shapes for object.");
  }
  if (block_->shapes.size() < 1 || block_->shapes[0].type != an::Shape::BOX) {
    pause("UseSpatula: Wrong type for block.  Should be a box.");
    return false;
  }
  if (block_->shapes.size() > 1) {
    ROS_WARN("UseSpatula: Too many shapes for block.");
  }

  if (spatula_->shapes.size() < 2 || spatula_->shapes[0].type != an::Shape::BOX ||
      spatula_->shapes[1].type != an::Shape::CYLINDER) {
    pause("UseSpatula: Wrong type for spatula.");
    return false;
  }
  if (spatula_->shapes.size() > 2) {
    ROS_WARN("UseSpatula: Too many shapes for spatula.");
  }
  
  push_ = NULL;
  rigid_transfer_ = NULL;
  base_transit_ = NULL;
  retreat_ = NULL;
  //this should check that pickup_ and push_ are for the right
  //objects - change DiscreteGrasp to be per object I think
  for (unsigned int i = 0; i < dspace_->primitives().size(); i++) {
    if (!push_) {
      push_ = dynamic_cast<const Push *>(dspace_->primitives().at(i));
      if (push_ && push_->objid() != objid_) {
	push_ = NULL;
      }
    }
    if (!rigid_transfer_) {
      rigid_transfer_ = dynamic_cast<const RigidTransfer *>(dspace_->primitives().at(i));
      if (rigid_transfer_ && rigid_transfer_->objid() != spatid_) {
	rigid_transfer_ = NULL;
      }
    }
    if (!base_transit_) {
      base_transit_ = dynamic_cast<const PR2BaseTransit *>(dspace_->primitives().at(i));
    }
    if (dynamic_cast<const Retreat *>(dspace_->primitives().at(i))) {
      retreat_ = dynamic_cast<const Retreat *>(dspace_->primitives().at(i));
    } else if (dynamic_cast<const Approach *>(dspace_->primitives().at(i))) {
      approach_name_ = dspace_->primitives().at(i)->name();
    } else if (dynamic_cast<const PR2ArmTransit *>(dspace_->primitives().at(i))) {
      arm_transit_name_ = dspace_->primitives().at(i)->name();
    }
  }
  if (!push_) {
    pause("Unable to find push primitive for UseSpatula!");
    return false;
  }
  if (!rigid_transfer_) {
    pause("Unable to find transit primitive for UseSpatula!");
    return false;
  }
  if (!base_transit_) {
    pause("Unable to find base transit primitive for UseSpatula!");
    return false;
  }
  if (!retreat_) {
    pause("Unable to find retreat primitive for UseSpatula");
    return false;
  }
  //pickup shouldn't (right now) be one of the primitives already created
  //we make our own
  if (!pickup_.setup(si)) {
    return false;
  }
  
  is_setup_ = true;
  return true;
}

darrt::Primitive *darrt::SpatulaTransfer::copy() const {
  SpatulaTransfer *p = new SpatulaTransfer(arm_name_, spatid_, objid_);
  if (is_setup()) {
    p->setup(si_);
  }
  return p;
}

bool darrt::UseSpatula::useful(const ob::State *source, const ob::State *target) const {
  //really we should just check the projection function
  //oh no because things might happen later
  //for now let's just go with:
  //Nothing is grasped
  //thing moves
  //later we should make this better
  //this should also be useful if we're holding the spatula and the 
  //thing is already in a pushing position
  const ObjectStateSpace::StateType *ts =
      dspace_->get_state(objind_, source)->as<ObjectStateSpace::StateType>();
  const ObjectStateSpace::StateType *bs =
      dspace_->get_state(blockind_, source)->as<ObjectStateSpace::StateType>();
  if (debug_level >= DPROPAGATE) {
    ROS_INFO("UseSpatula: Near object states: %d, Same table: %d",
	     dspace_->near_object_states(objind_, source, target),
	     ts->support_surface == bs->support_surface);
  }
  if ((dspace_->near_object_states(objind_, source, target) && 
       dspace_->near_object_states(spatind_, source, target)) ||
      ts->support_surface != bs->support_surface) {
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("UseSpatula: not useful because object is at goal or block is not on same table.");
    }
    return false;
  }
  //already used the spatula
  if (ts->attach_link.size() || !ts->support_surface) {
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("UseSpatula: not useful because the object is not sitting on a table in the starting pose");
    }
    return false;
  }
  

  if (dspace_->get_state(objind_, target)->as<ObjectStateSpace::StateType>()
      ->support_surface == ts->support_surface &&
      (objectBlockSide(target) < 0 || dspace_->get_state(spatind_, target)->as<ObjectStateSpace::StateType>()
       ->support_surface)) {
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("UseSpatula: not useful because the object is sitting on the same surface as it started and not against the block or the spatula is still on the support surface");
    }
    return false;
  }
  
  for (unsigned int i = 0; i < dspace_->object_indexes().size(); i++) {
    unsigned int ind = dspace_->object_indexes().at(i);
    const ObjectStateSpace::StateType *os =
      dspace_->get_state(ind, source)->as<ObjectStateSpace::StateType>();
    if (os->attach_link.size()) {
      if (ind != spatind_) {
	if (debug_level >= DPROPAGATE) {
	  ROS_INFO("UseSpatula: Holding object %d", ind);
	}
	return false;
      } else if (objectBlockSide(source) < 0) {
	if (debug_level >= DPROPAGATE) {
	  ROS_INFO("UseSpatula: Not useful because holding spatula in the wrong place");
	}
	return false;
      }
    }
  }
  return true;
}


/** SampleTo: Does the whole thing I think: 
 * 1) Push sponge to block
 * 2) Pick up spatula
 * 3) Rigid transfer spatula
 * 4) Use spatula to pick up sponge
 * Whether the sponge is tilted or held on the spatula depends
 * the parameters - this will work for either
 *
 * Currently this probably only works going forwards
 */

unsigned int darrt::UseSpatula::sampleTo(const ob::State *source,
					 const ob::State *target,
					 PIList &clist) const {
  //first push the sponge to the correct place
  //create the state after pushing
  //we care only about where the object is
  //either side of the block should work
  
  unsigned int turns = 0;

  gm::Pose object_pose, spatula_pose, flat_spatula_pose;
  gm::Vector3Stamped horizontal_approach_direction;

  chooseToolUse(source, target, object_pose, spatula_pose, flat_spatula_pose,
		horizontal_approach_direction);
  const ob::State *curr = source;
  ob::State *interstate = dspace_->allocState();
  
  gm::Pose curr_object_pose;
  dspace_->getSubspace(objind_)->as<ObjectStateSpace>()->
    object_position(dspace_->get_state(objind_, source), curr_object_pose);
  PIList traj;

  //if we need to do pushing or picking do that
  //create an intermediate state with the object in the correct pose
  //we use target so that we have both base positions
  dspace_->copyState(interstate, target);
  dspace_->getSubspace(objind_)->as<ObjectStateSpace>()
    ->set_object_position(dspace_->get_state(objind_, interstate), object_pose);
  

  //now sample to the pushing position if necessary
  if (darrt::distance(object_pose.position, curr_object_pose.position) > DIST_EPS) {
    if (debug_level >= DPROPAGATE) {
      ROS_INFO ("Doing pushing");
    }
    
    unsigned int push_turns = push_->sampleTo(curr, interstate, traj);
    if (!push_turns) {
      if (debug_level >= DPROPAGATE) {
	pause("Unable to sample push in spatula sample to.  Distance between poses is "
	      +makestring(darrt::distance(object_pose.position, curr_object_pose.position)));
      }
      dspace_->freeState(interstate);
      return 0;
    }
    curr = traj[traj.size()-1]->destination();
    turns += push_turns;

    //retreat from the pushing
    turns += retreat_->sampleTo(curr, target, traj);
      
    curr = traj[traj.size()-1]->destination();

    //did we get far enough?
    dspace_->getSubspace(objind_)->as<ObjectStateSpace>()->
      object_position(dspace_->get_state(objind_, curr), curr_object_pose);
    if (darrt::distance(object_pose.position, curr_object_pose.position) > DIST_EPS) {
      if (debug_level >= DPROPAGATE) {
	ROS_INFO("Unable to push far enough in UseSpatula (remaining distance = %f.  Returning only push",
		 darrt::distance(object_pose.position, curr_object_pose.position));
	ROS_INFO_STREAM("Object pose = " << object_pose << "\nCurr object pose =" << curr_object_pose);
      }
      for (unsigned int i = 0; i < traj.size(); i++) {
	clist.push_back(traj[i]);
      }
      traj.clear();
      dspace_->freeState(interstate);
      return turns;
    }
  }

  if (!dspace_->get_state(spatind_, curr)->as<ObjectStateSpace::StateType>()
      ->attach_link.size()) {
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("Doing pickup!");
    }
    //go pick up the spatula
    //do this here so we know the touch links
    //and how the spatula is being held
    unsigned int pickup_turns = pickup_.sampleTo(curr, interstate, traj);
  
    if (!pickup_turns) {
      ROS_ERROR("Unable to pick up spatula");
      for (unsigned int i = 0; i < traj.size(); i++) {
	delete traj[i];
      }
      dspace_->freeState(interstate);
      return 0;
    }

    turns += pickup_turns;
    curr = traj[traj.size()-1]->destination();
  }

  //should we rigid transfer the spatula?
  if (dspace_->near_object_states(objind_, curr, target)) {
    //move the spatula to its final destination
    for (unsigned int i = 0; i < traj.size(); i++) {
      clist.push_back(traj[i]);
    }
    traj.clear();
    dspace_->freeState(interstate);
    if (dspace_->get_state(spatind_, target)->as<ObjectStateSpace::StateType>()
	->attach_link.size()) {
      //move to this position
      unsigned int order = rand() % 2;
      if (order) {
	turns += sampleArmMove(rigid_transfer_, arm_transit_name_, curr, target, clist);
	curr = clist[clist.size()-1]->destination();
      }
      if (!base_transit_->base_ik().base_space()->near_states
	  (base_transit_->base_ik().base_state(curr),
	   base_transit_->base_ik().base_state(target))) {
	turns += base_transit_->base_ik().sampleBaseMove
	  (rigid_transfer_, base_transit_->name(), curr, target, clist);
	curr = clist[clist.size()-1]->destination();
      }
      if (!order) {
	turns += sampleArmMove(rigid_transfer_, arm_transit_name_, 
			       curr, target, clist);
      }
      return turns;
    }
    return turns + rigid_transfer_->sampleTo(curr, target, clist);
  }

  dspace_->copyState(interstate, curr);

  //set the spatula in the desired position in the intermediate state
  dspace_->getSubspace(spatind_)->as<ObjectStateSpace>()
    ->set_object_position(dspace_->get_state(spatind_, interstate), spatula_pose);

  //do the IK for the robot when the rigidly gripped
  //spatula is in the correct position
  gm::PoseStamped wps = ik_solver_.propagate_grasp(interstate, curr, spatind_);

  //collision operations are
  //all objects + their surfaces
  //spatula+gripper
  //spatula+object
  //spatula+object's surface
  //spatula+block
  //object+block
  //other objects + surfaces
  //might be wrong when going backwards!!
  an::OrderedCollisionOperations ops;
  an::CollisionOperation op;
  op.operation = op.DISABLE;
  op.object1 = spatid_;
  op.object2 = objid_;
  ops.collision_operations.push_back(op); //spatula+object
  op.object2 = blockid_;
  ops.collision_operations.push_back(op); //spatula+block
  op.object1 = objid_;
  ops.collision_operations.push_back(op); //object+block
  for (unsigned int i = 0; i < dspace_->object_indexes().size(); i++) {
    unsigned int ind = dspace_->object_indexes().at(i);
    if (ind == spatind_) {
      continue;
    }
    //disable collision between this object and the surface
    //supporting it			
    const SupportSurface *surf = dspace_->get_state(ind, source)->
      as<ObjectStateSpace::StateType>()->support_surface;
    if (surf) {
      op.object1 = dspace_->getSubspace(ind)->getName();
      op.object2 = surf->name();
      ops.collision_operations.push_back(op); //object+support surface
    }
  }
  std::string surf_name = dspace_->get_state(objind_, source)->
    as<ObjectStateSpace::StateType>()->support_surface->name();
  op.object1 = spatid_;
  op.object2 = surf_name;
  ops.collision_operations.push_back(op); //spatula+surface
  
  const std::vector<std::string> &touch_links = 
    dspace_->get_state(spatind_, curr)->
    as<ObjectStateSpace::StateType>()->touch_links;
  for (unsigned int i = 0; i < touch_links.size(); i++) {
    op.object2 = touch_links[i];
    ops.collision_operations.push_back(op); //gripper+spatula
  }
  
  //use twice as many base tries since this is a harder problem
  bool found_base_position = false;
  PIList h_approach, f_approach;
  gm::Vector3Stamped direction;

  for (unsigned int i = 0; i < 2*BASE_TRIES; i++) {
    if (i == 0) {
      base_transit_->base_ik().base_space()->copyState
	(base_transit_->base_ik().base_state(interstate),
	 base_transit_->base_ik().base_state(target));
    } else if (i == 1) {
      base_transit_->base_ik().base_space()->copyState
	(base_transit_->base_ik().base_state(interstate),
	 base_transit_->base_ik().base_state(curr));
    } else {
      base_transit_->base_ik().get_base_pose_for_wrist(wps.pose, interstate);
    }
    gm::PoseStamped ps = ik_solver_.transform_pose_stamped(arm_space_->root_name(), wps, interstate);
    if (!arm_space_->inverse_kinematics(ps, arm_state(interstate), arm_state(curr))) {
      if (!arm_space_->inverse_kinematics(ps, arm_state(interstate))) {
	continue;
      }
    }

    //plan the horizontal and vertical approaches first
    //because these are more constrained in the IK
        
    //first the transition from angled to flat
    //transform both poses into the robot's frame
    gm::PoseStamped fpose, apose;
    fpose.header.frame_id = ik_solver_.transformer().world_frame_id();
    apose.header.frame_id = ik_solver_.transformer().world_frame_id();
    fpose.pose = flat_spatula_pose;
    apose.pose = spatula_pose;
    fpose = ik_solver_.transform_pose_stamped(arm_space_->root_name(), fpose, interstate);
    apose = ik_solver_.transform_pose_stamped(arm_space_->root_name(), apose, interstate);
    gm::Pose origin;
    origin.orientation.w = 1.0;
    gm::Transform rtrans = pose_to_transform(transform_pose(inverse_transform_pose(origin, apose.pose), fpose.pose));
    unsigned int f_approach_turns = sampleLineArmMove(this, approach_name_, interstate, rtrans, 
						      si_->getPropagationStepSize(), ops, f_approach);
    if (!f_approach_turns) {
      if (debug_level >= DPROPAGATE) {
	ROS_INFO("Try %u: Unable to find transition from horizontal to flat for spatula", i);
      }
      continue;
    }

    //for this, the direction needs to be
    //in the root frame of the arm (not the world frame)
    direction = ik_solver_.transform_vector_stamped(arm_space_->root_name(), 
						    horizontal_approach_direction, interstate);
    
    unsigned int h_approach_turns = sampleLineArmMove(this, approach_name_, interstate, target, 
						      horizontal_approach_distance_,
						      spatula_->shapes[0].dimensions[1]+BLOCK_OFFSET, 
						      direction.vector, si_->getPropagationStepSize(),
						      ops, h_approach, true);
    if (!h_approach_turns || !h_approach.size()) {
      if (debug_level >= DPROPAGATE) {
	ROS_INFO("Turn %u: Unable to find horizonal approach for spatula", i);
      }
      for (unsigned int i = 0; i < f_approach.size(); i++) {
	delete f_approach[i];
      }
      f_approach.clear();
      for (unsigned int i = 0; i < h_approach.size(); i++) {
	delete h_approach[i];
      }
      h_approach.clear();
      continue;
    }
    turns += f_approach_turns + h_approach_turns;
    found_base_position = true;
    break;
  }

  if (!found_base_position) {
    ROS_ERROR("No solution for UseSpatula even moving base");
    for (unsigned int i = 0; i < traj.size(); i++) {
      delete traj[i];
    }
    dspace_->freeState(interstate);
    return 0;
  }


  //now the vertical approach, same operations
  direction.vector.x = 0;
  direction.vector.y = 0;
  direction.vector.z = 1;
  PIList v_approach;
  turns += sampleLineArmMove(this, approach_name_, h_approach[0]->source(), target, 
			     vertical_approach_distance_,
			     0, direction.vector, si_->getPropagationStepSize(),
			     ops, v_approach, true);
  
  //the state before starting in on the spatula
  const ob::State *preuse = h_approach[0]->source();
  if (v_approach.size()) {
    preuse = v_approach[0]->source();
  }
  
  //transfer the spatula to the position for the vertical approach
  //this is actually a "transit" because we know the full ending
  //position of the robot
  int order = rand() % 2;
  
  //at this point we know everything works
  //so we can start using the real list
  for (unsigned int i = 0; i < traj.size(); i++) {
    clist.push_back(traj[i]);
  }
  traj.clear();

  if (order) {
    turns += sampleArmMove(rigid_transfer_, arm_transit_name_, curr, preuse, clist);
    curr = clist[clist.size()-1]->destination();
  }
  if (base_transit_ && !base_transit_->base_ik().base_space()->near_states
      (base_transit_->base_ik().base_state(preuse),
       base_transit_->base_ik().base_state(curr))) {
    turns += base_transit_->base_ik().sampleBaseMove
      (rigid_transfer_, base_transit_->name(), curr, preuse, clist);
    curr = clist[clist.size()-1]->destination();
  }
  if (!order) {
    turns += sampleArmMove(rigid_transfer_, arm_transit_name_, curr, preuse, clist);
    curr = clist[clist.size()-1]->destination();
  }

  //add in the vertical and horizontal approaches to
  //the trajectory
  for (unsigned int i = 0; i < v_approach.size(); i++) {
    clist.push_back(v_approach[i]);
  }
  for (unsigned int i = 0; i < h_approach.size(); i++) {
    clist.push_back(h_approach[i]);
  }
  for (unsigned int i = 0; i < f_approach.size(); i++) {
    clist.push_back(f_approach[i]);
  }

  v_approach.clear();
  h_approach.clear();
  f_approach.clear();

  curr = clist[clist.size()-1]->destination();
  dspace_->copyState(interstate, curr);

  //update the object's position
  //it has moved all the way over to the block
  const ObjectStateSpace *ospace = dspace_->getSubspace(objind_)->as<ObjectStateSpace>();
  gm::Pose obj_pose;
  ospace->object_position(dspace_->get_state(objind_, curr), obj_pose);
  obj_pose.position.x -= BLOCK_OFFSET*horizontal_approach_direction.vector.x;
  obj_pose.position.y -= BLOCK_OFFSET*horizontal_approach_direction.vector.y;
  obj_pose.position.z -= BLOCK_OFFSET*horizontal_approach_direction.vector.z;
  ObjectStateSpace::StateType *os = dspace_->get_state(objind_, interstate)->
    as<ObjectStateSpace::StateType>();
  ospace->set_object_position(os, obj_pose);
  
  //now note that the object is attached to the spatula
  //we do this by actually attaching to the robot's hand
  //it's easier with how the code is set up and is the same
  //thing since the spatula is a rigid transformation
  //from the robot's hand to the object
  os->support_surface = NULL;
  os->attach_link = dspace_->get_state(spatind_, interstate)->
    as<ObjectStateSpace::StateType>()->attach_link;
  os->touch_links.clear();
  os->touch_links.push_back(spatid_);

  turns += sampleLineArmMove(this, approach_name_, interstate, target, lift_distance_, 0, 
			     direction.vector, si_->getPropagationStepSize(),
			     ops, clist);
  
  dspace_->freeState(interstate);
  return turns;
}

//returns (top, bottom)
std::pair<unsigned int, unsigned int> darrt::UseSpatula::blockOrientation(const ob::State *state) const {
  //find where the block is
  const ObjectStateSpace *block_space = dspace_->getSubspace(blockind_)->
    as<ObjectStateSpace>();
  const ObjectStateSpace::StateType *block_state = dspace_->get_state(blockind_, state)->
    as<ObjectStateSpace::StateType>();
  gm::Pose block_pose;
  block_space->object_position(block_state, block_pose);

  //figure out some things about the block's positioning
  double dims[3];
  for (unsigned int i = 0; i < 3; i++) {
    dims[i] = block_->shapes[0].dimensions[i];
  }
  //side midpoints in the block's frame
  gm::Point mids[6];
  
  mids[0].x = dims[0]/2.0;
  mids[1].x = -1.0*dims[0]/2.0;
  mids[2].y = dims[1]/2.0;
  mids[3].y = -1.0*dims[1]/2.0;
  mids[4].z = dims[2]/2.0;
  mids[5].z = -1.0*dims[2]/2.0;
  
  //transform them into the world frame
  gm::Point tmids[6];
  for (unsigned int i = 0; i < 6; i++) {
    tmids[i] = transform_point(mids[i], block_pose);
  }
  
  //we ignore the points with highest and lowest z values
  //these are above and below the block rather than
  //on the sides
  unsigned int top = 0, bottom = 5;
  double maxz = tmids[0].z, minz = tmids[5].z;
  for (unsigned int i = 1; i < 6; i++) {
    if (tmids[i].z > maxz) {
      top = i;
      maxz = tmids[i].z;
    }
    if (tmids[i-1].z < minz) {
      bottom = i-1;
      minz = tmids[i-1].z;
    }
  }
  return make_pair(top, bottom);
}

int darrt::UseSpatula::objectBlockSide(const ob::State *state) const {
  
  const ObjectStateSpace *block_space = dspace_->getSubspace(blockind_)->
    as<ObjectStateSpace>();
  const ObjectStateSpace::StateType *block_state = dspace_->get_state(blockind_, state)->
    as<ObjectStateSpace::StateType>();
  gm::Pose block_pose;
  block_space->object_position(block_state, block_pose);


  const ObjectStateSpace *ospace = dspace_->getSubspace(objind_)->
    as<ObjectStateSpace>();
  const ObjectStateSpace::StateType *ostate = dspace_->get_state(objind_, state)->
    as<ObjectStateSpace::StateType>();
  gm::Pose opose;
  ospace->object_position(ostate, opose);
  gm::Pose relpose = inverse_transform_pose(opose, block_pose);

  //we're going to try to push against bmid
  //we assume the thing we're pushing is circular
  //the block_offset allows us not to have to do an approach here
  //it should be small
  double radius = obj_->shapes[0].dimensions[0] + BLOCK_OFFSET;

  unsigned int top = blockOrientation(state).first;

  if (top != 0 && top != 1 && inrange(relpose.position.x, radius + block_->shapes[0].dimensions[0]/2.0, DIST_EPS)
      && (((top == 4 || top == 5) && inrange(relpose.position.y, 0.0, DIST_EPS)) || 
	  ((top == 2 || top == 3) && inrange(relpose.position.z, 0.0, DIST_EPS)))) {
    return 0;
  }
  
  if (top != 0 && top != 1 && inrange(relpose.position.x, -1.0*(radius + block_->shapes[0].dimensions[0]/2.0), DIST_EPS)
	     && (((top == 4 || top == 5) && inrange(relpose.position.y, 0.0, DIST_EPS)) || 
		 ((top == 2 || top == 3) && inrange(relpose.position.z, 0.0, DIST_EPS)))) {
    return 1;
  } 

  if (top != 2 && top != 3 && inrange(relpose.position.y, radius + block_->shapes[0].dimensions[1]/2.0, DIST_EPS)
      && (((top == 4 || top == 5) && inrange(relpose.position.x, 0.0, DIST_EPS)) || 
	  ((top == 0 || top == 1) && inrange(relpose.position.z, 0.0, DIST_EPS)))) {
    return 2;
  } 

  if (top != 2 && top != 3 && inrange(relpose.position.y, -1.0*(radius + block_->shapes[0].dimensions[1]/2.0), DIST_EPS)
      && (((top == 4 || top == 5) && inrange(relpose.position.x, 0.0, DIST_EPS)) || 
	  ((top == 0 || top == 1) && inrange(relpose.position.z, 0.0, DIST_EPS)))) {
    return 3;
  }

  if (top != 4 && top != 5 && inrange(relpose.position.z, radius + block_->shapes[0].dimensions[2]/2.0, DIST_EPS)
      && (((top == 2 || top == 3) && inrange(relpose.position.x, 0.0, DIST_EPS)) || 
	  ((top == 0 || top == 1) && inrange(relpose.position.y, 0.0, DIST_EPS)))) {
    return  4;
  }
  if (top != 4 && top != 5 && inrange(relpose.position.z, -1.0*(radius + block_->shapes[0].dimensions[2]/2.0), DIST_EPS)
      && (((top == 2 || top == 3) && inrange(relpose.position.x, 0.0, DIST_EPS)) || 
	  ((top == 0 || top == 1) && inrange(relpose.position.y, 0.0, DIST_EPS)))) {
    return 5;
  }

  return -1;
}


void darrt::UseSpatula::chooseToolUse(const ob::State *source,
				      const ob::State *target,
				      gm::Pose &object_pose,
				      gm::Pose &spatula_pose,
				      gm::Pose &flat_spatula_pose,
				      gm::Vector3Stamped &horizontal_approach_direction) const {
  //find where the block is
  const ObjectStateSpace *block_space = dspace_->getSubspace(blockind_)->
    as<ObjectStateSpace>();
  const ObjectStateSpace::StateType *block_state = dspace_->get_state(blockind_, source)->
    as<ObjectStateSpace::StateType>();
  gm::Pose block_pose;
  block_space->object_position(block_state, block_pose);

  
  std::pair<unsigned int, unsigned int> ori = blockOrientation(source);
  unsigned int top = ori.first;
  unsigned int bottom = ori.second;
  int side = objectBlockSide(target);
  if (side < 0 || dspace_->get_state(spatind_, source)->as<ObjectStateSpace::StateType>()
      ->attach_link.size()) {
    side = objectBlockSide(source);
    if (side >= 0 && debug_level >= DPROPAGATE) {
      ROS_INFO("chooseToolUse: Using source pushing side.");
    }
  } else {
    ROS_INFO("chooseToolUse: Using target pushing side");
  }
  if (side < 0) {
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("chooseToolUse: Choosing random pushing side.");
    }
    
    //this happens and it's OK when using the projection function
    //from rigid transfer when we're going backwards
    // if (dspace_->get_state(spatind_, source)->as<ObjectStateSpace::StateType>()
    // 	->attach_link.size()) {
    //   pause("Spatula attached but object not in correct position!!!");
    // }
    side = rand() % 4;
    
    if (static_cast<unsigned int>(side) >= top) {
      side++;
    }
    if (static_cast<unsigned int>(side) >= bottom) {
      side++;
    }
  }
 
  const ObjectStateSpace *ospace = dspace_->getSubspace(objind_)->
    as<ObjectStateSpace>();
  const ObjectStateSpace::StateType *ostate = dspace_->get_state(objind_, source)->
    as<ObjectStateSpace::StateType>();
  gm::Pose opose;
  ospace->object_position(ostate, opose);
  
  //the spatula should be all the way under the object
  //this means the origin of the spatula should be at
  //the edge of the object when the object is pushed 
  //against the block

  double spat_offset = 2.0*obj_->shapes[0].dimensions[0];
  if (spat_offset < spatula_->shapes[0].dimensions[1]) {
    spat_offset = spatula_->shapes[0].dimensions[1];
  }

  //figure out the pushed object's position
  gm::Point pushpt;
  horizontal_approach_direction.header.frame_id = 
    ik_solver_.transformer().world_frame_id();
  horizontal_approach_direction.vector.x = 0;
  horizontal_approach_direction.vector.y = 0;
  horizontal_approach_direction.vector.z = 0;
  spatula_pose.position.x = 0;
  spatula_pose.position.y = 0;
  spatula_pose.position.z = 0;
  spatula_pose.orientation.x = 0;
  spatula_pose.orientation.y = 0;
  spatula_pose.orientation.z = 0;
  spatula_pose.orientation.w = 0;

  flat_spatula_pose.position.x = 0;
  flat_spatula_pose.position.y = 0;
  flat_spatula_pose.position.z = 0;
  flat_spatula_pose.orientation.x = 0;
  flat_spatula_pose.orientation.y = 0;
  flat_spatula_pose.orientation.z = 0;
  flat_spatula_pose.orientation.w = 0;

  gm::Pose x_trans;
  x_trans.orientation.x = sin(-1.0*MATH_PI/30.0);
  x_trans.orientation.w = cos(-1.0*MATH_PI/30.0);
  
  switch(top) {
    //set the height correctly
  case 0:
    //top is +x
    pushpt.x = (obj_->shapes[0].dimensions[1] - block_->shapes[0].dimensions[0])/2.0;
    flat_spatula_pose.position.x = (spatula_->shapes[0].dimensions[2] - block_->shapes[0].dimensions[0])/2.0;
    break;
  case 1:
    //top is -x
    pushpt.x = -1.0*(obj_->shapes[0].dimensions[1] - block_->shapes[0].dimensions[0])/2.0;
    flat_spatula_pose.position.x = -1.0*(spatula_->shapes[0].dimensions[2] - block_->shapes[0].dimensions[0])/2.0;
    break;
  case 2:
    //top is +y
    pushpt.y = (obj_->shapes[0].dimensions[1] - block_->shapes[0].dimensions[1])/2.0;
    flat_spatula_pose.position.y = (spatula_->shapes[0].dimensions[2] - block_->shapes[0].dimensions[1])/2.0;
    break;
  case 3:
    //top is -y
    pushpt.y = -1.0*(obj_->shapes[0].dimensions[1] - block_->shapes[0].dimensions[1])/2.0;
    flat_spatula_pose.position.y = -1.0*(spatula_->shapes[0].dimensions[2] - block_->shapes[0].dimensions[1])/2.0;
    break;
  case 4:
    //top is +z
    pushpt.z = (obj_->shapes[0].dimensions[1] - block_->shapes[0].dimensions[2])/2.0;
    flat_spatula_pose.position.z = (spatula_->shapes[0].dimensions[2] - block_->shapes[0].dimensions[2])/2.0;
    break;
  case 5:
    //top is -z
    pushpt.z = -1.0*(obj_->shapes[0].dimensions[1] - block_->shapes[0].dimensions[2])/2.0;
    flat_spatula_pose.position.z = -1.0*(spatula_->shapes[0].dimensions[2] - block_->shapes[0].dimensions[2])/2.0;
    break;
  }

  const std::vector<double> &dims = block_->shapes[0].dimensions;
  double radius = obj_->shapes[0].dimensions[0] + BLOCK_OFFSET;

  switch(side) {
  case 0:
    pushpt.x = dims[0]/2.0 + radius;
    flat_spatula_pose.position.x = dims[0]/2.0 + spat_offset;
    flat_spatula_pose.orientation.z = sin(MATH_PI/4.0);
    flat_spatula_pose.orientation.w = cos(MATH_PI/4.0);
    spatula_pose = transform_pose(x_trans, flat_spatula_pose);
    horizontal_approach_direction.vector.x = 1;
    break;
  case 1:
    pushpt.x = -1.0*(dims[0]/2.0 + radius);
    flat_spatula_pose.position.x = -1.0*(dims[0]/2.0 + spat_offset);
    flat_spatula_pose.orientation.z = sin(-1.0*MATH_PI/4.0);
    flat_spatula_pose.orientation.w = cos(-1.0*MATH_PI/4.0);
    spatula_pose = transform_pose(x_trans, flat_spatula_pose);
    horizontal_approach_direction.vector.x = -1;
    break;
  case 2:
    pushpt.y = dims[1]/2.0 + radius;
    flat_spatula_pose.position.y = dims[1]/2.0 + spat_offset;
    flat_spatula_pose.orientation.z = sin(MATH_PI/2.0);
    flat_spatula_pose.orientation.w = cos(MATH_PI/2.0);
    spatula_pose = transform_pose(x_trans, flat_spatula_pose);
    horizontal_approach_direction.vector.y = 1;
    break;
  case 3:
    pushpt.y = -1.0*(dims[1]/2.0 + radius);
    flat_spatula_pose.position.y = -1.0*(dims[1]/2.0 + spat_offset);
    flat_spatula_pose.orientation.z = 0;
    flat_spatula_pose.orientation.w = 1;
    spatula_pose = transform_pose(x_trans, flat_spatula_pose);
    horizontal_approach_direction.vector.y = -1;
    break;
  case 4:
    {
      pushpt.z = dims[2]/2.0 + radius;
      flat_spatula_pose.position.z = dims[2]/2.0 + spat_offset;
      //we want to first rotate 90 degrees around the x axis
      //and then 90 degrees around the (resulting) y axis
      //i think?
      gm::Quaternion first_rot, second_rot;
      first_rot.x = sin(MATH_PI/4.0);
      first_rot.w = cos(MATH_PI/4.0);
      second_rot.y = sin(MATH_PI/4.0);
      second_rot.w = cos(MATH_PI/4.0);
      multiply(second_rot, first_rot, flat_spatula_pose.orientation);
      spatula_pose = transform_pose(x_trans, flat_spatula_pose);
      horizontal_approach_direction.vector.z = 1;
      break;
    }
  case 5:
    {
      pushpt.z = -1.0*(dims[2]/2.0 + radius);
      flat_spatula_pose.position.z = -1.0*(dims[2]/2.0 + spat_offset);
      //we want to first rotate -90 degrees around the x axis
      //and then 90 degrees around the (resulting) y axis
      //i think?
      gm::Quaternion first_rot, second_rot;
      first_rot.x = sin(-1.0*MATH_PI/4.0);
      first_rot.w = cos(-1.0*MATH_PI/4.0);
      second_rot.y = sin(MATH_PI/4.0);
      second_rot.w = cos(MATH_PI/4.0);
      multiply(second_rot, first_rot, flat_spatula_pose.orientation);
      spatula_pose = transform_pose(x_trans, flat_spatula_pose);
      horizontal_approach_direction.vector.z = -1;
      break;
    }
  }

  dspace_->getSubspace(objind_)->as<ObjectStateSpace>()
    ->object_position(dspace_->get_state(objind_, source), object_pose);
  // double height = object_pose.position.z;
  object_pose.position = transform_point(pushpt, block_pose);
  // object_pose.position.z = height;

  spatula_pose = transform_pose(spatula_pose, block_pose);

  flat_spatula_pose = transform_pose(flat_spatula_pose, block_pose);


  //this is the direction 
  horizontal_approach_direction.vector = 
    transform_vector(horizontal_approach_direction.vector, block_pose);

  if (debug_level >= DPROPAGATE) {
    gm::Pose origin;
    origin.orientation.w = 1.0;
    // ROS_INFO_STREAM("UseSpatula: Object's position is\n" << object_pose <<
    // 		    "\nSpatula's position is\n" << spatula_pose <<
    // 		    "\nFlat spatula's position is\n" << flat_spatula_pose <<
    // 		    "\nDirection is\n" << horizontal_approach_direction);
  }
}

darrt::SpatulaTransfer::SpatulaTransfer(std::string arm_name,
					std::string spatid,
					std::string objid) :  
  TransferPrimitive(std::string("SpatulaTransfer")+"-"+
		    arm_name+"-"+spatid+"-"+objid),  
  PR2Arm(arm_name) {
  spatid_ = spatid;
  objid_ = objid;
  base_transit_ = NULL;
  obj_ = NULL;
  spatula_ = NULL;
  approach_name_ = "Approach-"+arm_name;
}

bool darrt::SpatulaTransfer::setup(const oc::SpaceInformation *si) {

  is_setup_ = false;
  if (!TransferPrimitive::setup(si)) {
    return false;
  }
  if (!setup_arm(si)) {
    return false;
  }
  //find the object and spatula indices
  //here
  objind_ = dspace_->getSubspaceIndex(objid_);
  spatind_ = dspace_->getSubspaceIndex(spatid_);

  obj_ = &(dspace_->getSubspace(objind_)->as<ObjectStateSpace>()->object());
  spatula_ = &(dspace_->getSubspace(spatind_)->as<ObjectStateSpace>()->object());
  base_transit_ = NULL;
  uspat_ = NULL;
  //this should check that pickup_ and push_ are for the right
  //objects - change DiscreteGrasp to be per object I think
  for (unsigned int i = 0; i < dspace_->primitives().size(); i++) {
    if (!dynamic_cast<const Retreat *>(dspace_->primitives().at(i))) {
      if (dynamic_cast<const Approach *>(dspace_->primitives().at(i))) {
	approach_name_ = dspace_->primitives().at(i)->name();
      } else if (dynamic_cast<const PR2ArmTransit *>(dspace_->primitives().at(i))) {
	arm_transit_name_ = dspace_->primitives().at(i)->name();
      }
    }
    if (!base_transit_) {
      base_transit_ = dynamic_cast<const PR2BaseTransit *>(dspace_->primitives().at(i));
    }
    if (!uspat_) {
      uspat_ = dynamic_cast<const UseSpatula *>(dspace_->primitives().at(i));
    }
  }
  if (!base_transit_) {
    pause("Unable to find base transit primitive for SpatulaTransfer!");
    return false;
  }
  if (!uspat_) {
    pause("Unable to find UseSpaulta primitive for SpatulaTransfer!");
    return false;
  }
  
  is_setup_ = true;
  return true;
}

bool darrt::SpatulaTransfer::useful(const ob::State *source, const ob::State *target) const {
  //

  if (!dspace_->get_state(objind_, source)->as<ObjectStateSpace::StateType>()->attach_link.size() ||
      !dspace_->get_state(spatind_, source)->as<ObjectStateSpace::StateType>()->attach_link.size()) {
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("Spatula transfer not useful because object not attached to spatula");
    }
    return false;
  }

  const ObjectStateSpace *ospace = dspace_->getSubspace(objind_)
    ->as<ObjectStateSpace>();
  const ObjectStateSpace::StateType *ts = dspace_->get_state(objind_, target)
    ->as<ObjectStateSpace::StateType>();
  const ObjectStateSpace::StateType *os = dspace_->get_state(objind_, source)
    ->as<ObjectStateSpace::StateType>();

  if (dspace_->near_object_states(objind_, source, target)) {
    //is the robot in the right place?  if not, will putting it there work?
    if (!arm_space_->near_states(arm_state(source), arm_state(target)) ||
	!base_transit_->base_ik().base_space()->near_states
	(base_transit_->base_ik().base_state(source),
	 base_transit_->base_ik().base_state(target))) {
      //if we move to this, will the object be in the right place?
      gm::Pose newpose = ik_solver_.update_attached_object_position(objind_, source, target);
      gm::Pose oldpose;
      ospace->object_position(os, oldpose);
      if (ospace->object().nearPoses(newpose, oldpose)) {
	return true;
      } else if (debug_level >= DPROPAGATE) {
	ROS_INFO_STREAM("Robot destination does not have the object in the correct place.  Correct pose is\n" 
			<< oldpose << "\nWhile destination pose is\n" << newpose);
      }
    }
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("SpatulaTransfer not useful because object in goal pose.");
    }
    return false;
  }
      
  //also make sure target is a rotation in z from source
  //for now assume that there is no rotation around
  //the other two axes
  gm::Pose opose, tpose;
  ospace->object_position(ts, tpose);
  ospace->object_position(os, opose);
  if (!inrange(tpose.orientation.x, 0, ANGLE_EPS) || !inrange(tpose.orientation.y, 0, ANGLE_EPS) ||
      !inrange(opose.orientation.x, 0, ANGLE_EPS) || !inrange(opose.orientation.y, 0, ANGLE_EPS)) {
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("Spatula transfer not useful because orientations are wrong");
    }
    return false;
  }
  return true;
}


/** SampleTo: Does the whole thing I think: 
 *  Only allows first a forward back line
 *  Then an up/down line (yay for simplicity)
 *  Assume for now that rotation doesn't matter?
 **/

unsigned int darrt::SpatulaTransfer::sampleTo(const ob::State *source,
					      const ob::State *target,
					      PIList &clist) const {

  //figure out where the wrist should be in the ending position
  //only rotations around the z axis should be allowed...
  //just assume only yaw for right now

  const ObjectStateSpace *ospace = dspace_->getSubspace(objind_)
    ->as<ObjectStateSpace>();
  const ObjectStateSpace::StateType *ts = dspace_->get_state(objind_, target)
    ->as<ObjectStateSpace::StateType>();
  const ObjectStateSpace::StateType *os = dspace_->get_state(objind_, source)
    ->as<ObjectStateSpace::StateType>();

  gm::PoseStamped wps, tpose;
  tpose.header.frame_id = ik_solver_.transformer().world_frame_id();

  //check if the object is in the right place in the robot's final position
  gm::Pose newpose = ik_solver_.update_attached_object_position(objind_, source, target);
  gm::Pose oldpose;
  ospace->object_position(ts, oldpose);
  bool robot_positioned = false;
  if (ospace->object().nearPoses(newpose, oldpose)) {
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("Using wrist pose from final position.");
    }							       
    robot_positioned = true;
    //because we don't care about angles for round objects
    //the final object pose and the pose achieved by the robot
    //might be different
    //if the final pose has the spatula correctly positioned for
    //the object (which it does sometimes going backwards) then we
    //want to use this wrist pose
    tpose.pose = newpose;
    gm::PoseStamped aps;
    arm_space_->forward_kinematics(arm_state(target), aps);
    wps = ik_solver_.transform_pose_stamped(ik_solver_.transformer().world_frame_id(), aps, target);
  } else {
    ospace->object_position(ts, tpose.pose);
    //figure out where the wrist should be from where the object should be
    wps = ik_solver_.propagate_grasp(target, source, objind_);
  }


  //a special case
  if (robot_positioned && dspace_->near_object_states(objind_, source, target)) {
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("Adjusting the IK in SpatulaTransfer.");
    }
    unsigned int order = rand() % 2;
    const ob::State *curr_state = source;
    unsigned int turns = 0;
    if (order) {
      turns += sampleArmMove(this, arm_transit_name_, curr_state, target, clist);
      curr_state = clist[clist.size()-1]->destination();
    }
    const BaseIK &base_ik = base_transit_->base_ik();
    if (!base_ik.base_space()->near_states(base_ik.base_state(curr_state),
					   base_ik.base_state(target))) {
      turns += base_transit_->base_ik().sampleBaseMove(this, base_transit_->name(), curr_state, target, clist);
      curr_state = clist[clist.size()-1]->destination();
    }
    
    if (!order) {
      turns += sampleArmMove(this, arm_transit_name_, curr_state, target, clist);
      curr_state = clist[clist.size()-1]->destination();
    }
    return turns;
  } 

  ob::State *ending_state = dspace_->allocState();
  dspace_->copyState(ending_state, source);


  //choose an order (base move, horizontal move, vertical move...)
  unsigned int order = rand() % 2;

  //allow some extra tries because this is a harder problem
  for (unsigned int i = 0; i < 2*BASE_TRIES; i++) {
    //figure out a base position
    bool base_move = true;
    if (i == 0) {
      //first try the destination base position
      base_transit_->base_ik().base_space()->copyState
	(base_transit_->base_ik().base_state(ending_state),
	 base_transit_->base_ik().base_state(target));
      if (debug_level >= DPROPAGATE) {
	ROS_INFO_STREAM("Trying target base position\n" <<
			base_transit_->base_ik().base_pose(ending_state));
      }
    } else if (i == 1) {
      base_move = false;
      //then the source position
      base_transit_->base_ik().base_space()->copyState
	(base_transit_->base_ik().base_state(ending_state),
	 base_transit_->base_ik().base_state(source));
      if (debug_level >= DPROPAGATE) {
	ROS_INFO_STREAM("Trying source base position\n" <<
			base_transit_->base_ik().base_pose(ending_state));
      }
    } else {
      base_transit_->base_ik().get_base_pose_for_wrist(wps.pose, ending_state);
    }
    ik_solver_.update_attached_object_positions(source, ending_state);


    //horizontal distance and direction in the arm's frame
    //this is the target pose in the arm's frame
    gm::PoseStamped tps = ik_solver_.transform_pose_stamped(arm_space_->root_name(), tpose, ending_state);
    //this is the object's pose in ending state in the world frame
    gm::PoseStamped eps;
    eps.header.frame_id = ik_solver_.transformer().world_frame_id();
    ospace->object_position(dspace_->get_state(objind_, ending_state), eps.pose);
    eps = ik_solver_.transform_pose_stamped(arm_space_->root_name(), eps, ending_state);
    //transform in the robot's frame
    gm::Pose origin;
    origin.orientation.w = 1.0;
    gm::Transform htrans = pose_to_transform(transform_pose(inverse_transform_pose(origin, eps.pose), tps.pose));

    an::OrderedCollisionOperations ops;
    PIList first_move, second_move;
    unsigned int b_turns = 0, h_turns;
    //now see if we can actually do these things...
    if (order == 0) {
      //base move first
      const ob::State *curr = source;
      if (base_move) {
	b_turns = base_transit_->base_ik().sampleBaseMove(this, base_transit_->name(), source, ending_state, first_move);
	curr = first_move[first_move.size()-1]->destination();
      }
      //horizontal move second
      h_turns = sampleLineArmMove(this, approach_name_, curr, htrans, si_->getPropagationStepSize(),
				  ops, second_move);
      if (!h_turns) {
	if (debug_level >= DPROPAGATE) {
	  ROS_INFO("SpatulaTransfer try %u: Unable to find level motion.", i);
	}
	for (unsigned int i = 0; i < first_move.size(); i++) {
	  delete first_move[i];
	}
	first_move.clear();
	continue;
      }
    } else {
      //horizontal move first
      h_turns = sampleLineArmMove(this, approach_name_, source, htrans, si_->getPropagationStepSize(),
				  ops, first_move);
      if (!h_turns) {
	if (debug_level >= DPROPAGATE) {
	  ROS_INFO("SpatulaTransfer try %u: Unable to find level motion.", i);
	}
	continue;
      }
      //base move second
      ob::State *curr = first_move[first_move.size()-1]->destination();
      if (base_move) {
	b_turns = base_transit_->base_ik().sampleBaseMove(this, base_transit_->name(), curr, ending_state, second_move);
      }
    }
    
    dspace_->freeState(ending_state);
    for (unsigned int i = 0; i < first_move.size(); i++) {
      clist.push_back(first_move[i]);
    }
    for (unsigned int i = 0; i < second_move.size(); i++) {
      clist.push_back(second_move[i]);
    }
    first_move.clear();
    second_move.clear();
    return b_turns + h_turns;
  }

  if (debug_level >= DRRT) {
    ROS_INFO("Unable to find SpatulaTransfer");
  }
  return 0;
}

void darrt::SpatulaTransfer::projectionFunction(const ob::State *from, ob::State *sample) const {
  if (debug_level >= DRRT) {
    ROS_INFO("Using spatula transfer projection.");
  }

  //make the angle the final state have only a yaw for the object
  //really should fill in robot/spatula position...
  gm::Pose opose;
  const ObjectStateSpace *ospace = dspace_->getSubspace(objind_)->as<ObjectStateSpace>();
  ObjectStateSpace::StateType *os = dspace_->get_state(objind_, sample)->
    as<ObjectStateSpace::StateType>();
  ospace->object_position(os, opose);
  double yaw = quaternion_to_yaw(opose.orientation);
  yaw_to_quaternion(yaw, opose.orientation);
  ospace->set_object_position(os, opose);

  //if we're already holding this, this needs to be the outcome of a
  //single use of this primitive

  if (dspace_->get_state(objind_, from)->as<ObjectStateSpace::StateType>()->attach_link.size()) {
    PIList clist;
    //int actual_debug_level = debug_level;
    //debug_level = DMINIMAL;
    sampleTo(from, sample, clist);
    //debug_level = actual_debug_level;
    if (!clist.size()) {
      ROS_ERROR("Unable to project spatula transfer correctly");
      //something went wrong, oh well
      return;
    }
    dspace_->copyState(sample, clist[clist.size()-1]->destination());
    for (unsigned int i = 0; i < clist.size(); i++) {
      delete clist[i];
    }
    clist.clear();
  } 
}

darrt::DARRTProjectionFunction darrt::SpatulaTransfer::getProjectionFunction() const {
  return boost::bind(&darrt::SpatulaTransfer::projectionFunction, this, _1, _2);
}
