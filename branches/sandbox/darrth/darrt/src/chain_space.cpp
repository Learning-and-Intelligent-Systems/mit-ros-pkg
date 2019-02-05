//local includes
#include "darrt/chain_space.hh"
#include "darrt/transform_ros_types.hh"
#include "darrt/utils.hh"

//ROS includes
#include <ros/ros.h>
#include <arm_navigation_msgs/GetRobotState.h>
#include <arm_navigation_msgs/RobotState.h>
#include <geometry_msgs/Pose.h>
#include <planning_environment/models/model_utils.h>


//quite a lot of this parallels the planning group implementation
//in ompl_ros_interface
//you could just initialize a joint_space_planner, but that requires
//exporting the ompl_ros_interface library and makes us dependent on its
//quirks
darrt::RobotChainStateSpace::RobotChainStateSpace
(const ros::NodeHandle &node_handle, const std::string &group_name,
 EnvironmentInterface *env) :
  RobotStateSpace(), Displayable("robot_state_markers"),
  kinematics_loader_(std::string("kinematics_base"), 
		     std::string("kinematics::KinematicsBase")) {
  initialized_ = false;
  node_ = node_handle;
  group_name_ = group_name;
  environment_interface_ = env;
  //this is all that needs to be done here
  //the heavy lifting is all in init
  init();
}

void darrt::RobotChainStateSpace::setup() {
  if (!initialized_) {
    init();
  }
}

bool darrt::RobotChainStateSpace::init() {
  initialized_ = false;


  //figure out the actual name of the group
  //and get the collision model
  if (!initialize_physical_group()) {
    return false;
  }

  ROS_INFO("Locked is: %d", isLocked());

  //initialize the kinematics solver
  if (!initialize_kinematics()) {
    return false;
  }

  ROS_INFO("Locked is: %d", isLocked());

  //initialize this space with the correct OMPL subspaces
  //also returns conversions to and from the kinematic state
  //we actually only use the robot mappings we'll get later
  if (!ompl_ros_conversions::joint_group_to_ompl_state_space
      (physical_joint_group_, this, ompl_to_kinematic_mapping_, 
       kinematic_to_ompl_mapping_)) {
    return false;
  }  


  an::RobotState robot_state;
  //get the current robot state as an example state
  ros::ServiceClient state_client =
    node_.serviceClient<an::GetRobotState>
    ("/environment_server/get_robot_state");
  an::GetRobotState srv;
  if (!state_client.call(srv)) {
    ROS_ERROR("Unable to get example robot state");
    return false;
  }

  if (!restrict_to_group(srv.response.robot_state, robot_state)) {
    return false;
  }

  //get the conversions to and from the RobotState message
  if (!ompl_ros_conversions::get_ompl_state_to_robot_state_mapping
      (this, robot_state, ompl_to_robot_mapping_, true)) {
    ROS_ERROR("Unable to get ompl state to robot state mapping");
    return false;
  }
  if (!ompl_ros_conversions::get_robot_state_to_ompl_state_mapping
      (robot_state, this, robot_to_ompl_mapping_, true)) {
    ROS_ERROR("Unable to get robot state to ompl state mapping");
    return false;
  }

  setName(physical_joint_group_->getName());
  initialized_ = true;
  return true;
}

bool darrt::RobotChainStateSpace::initialize_physical_group() {
  //copied exactly from ompl_ros_conversions
  std::string physical_group_name;
  if(!environment_interface_->getKinematicModel()->hasModelGroup(group_name_)) {
    if(!node_.hasParam(group_name_+"/physical_group")) {
      ROS_ERROR("No physical group specified for %s",group_name_.c_str());
      return false;
    } else {
      node_.getParam(group_name_+"/physical_group",physical_group_name);
    }
  } else {
    physical_group_name = group_name_;
  }

  //Setup the actual (physical) groups
  physical_joint_group_ = environment_interface_->getKinematicModel()->
    getModelGroup(physical_group_name);

  return true;
}

bool darrt::RobotChainStateSpace::initialize_kinematics() {
  //get the kinematics solver name
  ROS_INFO("Initializing kinematics");
  std::string physical_group_name = physical_joint_group_->getName();
  if (!node_.hasParam(physical_group_name + "/tip_name")) {
    ROS_ERROR("Cannot initialize kinematics: No tip name for group %s in namespace %s",
	      physical_group_name.c_str(), node_.getNamespace().c_str());
    return false;
  }
  //this is a TERRIBLE way to get the joint limits :p
  kinematics_msgs::GetKinematicSolverInfo ksrv;
  ros::ServiceClient kclient = node_.serviceClient
    <kinematics_msgs::GetKinematicSolverInfo>
    ("/pr2_"+physical_group_name+"_kinematics/get_ik_solver_info");
  if (kclient.call(ksrv)) {
    kinematic_info_ = ksrv.response.kinematic_solver_info;
  } else {
    ROS_ERROR("Unable to get kinematic solver information for group %s",
	      physical_group_name.c_str());
    return false;
  }

  node_.getParam(physical_group_name + "/tip_name", end_effector_name_);
  if (!node_.hasParam(physical_group_name + "/root_name")) {
    ROS_ERROR("Cannot initialize kinematics: No root name for group %s",
	      physical_group_name.c_str());
    return false;
  }
  node_.getParam(physical_group_name + "/root_name", root_name_);
  if (!node_.hasParam(physical_group_name + "/kinematics_solver")) {
    ROS_ERROR("Cannot initialize kinematics: No solver specified for group %s",
	      physical_group_name.c_str());
    return false;
  }
  node_.getParam(physical_group_name + "/kinematics_solver",
		 kinematics_solver_name_);
  ROS_INFO_STREAM("Trying to initialize kinematics solver "
		  << kinematics_solver_name_ << " for group "
		  << physical_group_name << " with tip "
		  << end_effector_name_ << " and root "
		  << root_name_);
	
  if (!kinematics_loader_.isClassAvailable(kinematics_solver_name_)) {
    ROS_ERROR("Cannot initialize kinematics: pluginlib does not have class %s",
	      kinematics_solver_name_.c_str());
    return false;
  }
  try {
    kinematics_solver_ = kinematics_loader_.createUnmanagedInstance
      (kinematics_solver_name_);
  } catch (pluginlib::PluginlibException &ex) {
    ROS_ERROR("Cannot initialize kinematics: plugin failed to load.  Error: %s",
	      ex.what());
    return false;
  }

  //the 0.01 matches the argument from ompl_ros_ik_sampler
  //for fuerte:
  if (!kinematics_solver_->initialize(group_name_, root_name_,
				      end_effector_name_, 
				      0.01)) {
    ROS_ERROR_STREAM("Cannot initialize kinematics: "
		     << "unable to initialize solver for group "
		     << group_name_);
    return false;
  }
  return true;
}

bool darrt::RobotChainStateSpace::restrict_to_group
(const an::RobotState &full_state, an::RobotState &group_state) const {
  group_state.joint_state.header = full_state.joint_state.header;
  group_state.joint_state.name.clear();
  group_state.joint_state.position.clear(); 
  group_state.joint_state.velocity.clear();
  group_state.joint_state.effort.clear();
  group_state.multi_dof_joint_state.joint_names.clear();
  group_state.multi_dof_joint_state.frame_ids.clear();
  group_state.multi_dof_joint_state.child_frame_ids.clear();
  group_state.multi_dof_joint_state.poses.clear();
  const std::vector<std::string> &links = kinematics_solver_->getJointNames();
  group_state.joint_state.name.resize(links.size());
  group_state.joint_state.position.resize(links.size());
  for (size_t i = 0; i < links.size(); i++) {
    bool matched = false;
    for (size_t j = 0; j < full_state.joint_state.name.size(); j++) {
      if (!links[i].compare(full_state.joint_state.name[j])) {
	matched = true;
	group_state.joint_state.name[i] = links[i];
	group_state.joint_state.position[i] = 
	  full_state.joint_state.position[j];
	break;
      }
    }
    if (!matched) {
      ROS_ERROR("Converting robot state to group: unable to match link %s",
		links[i].c_str());
      return false;
    }
  }
  return true;
}

double darrt::RobotChainStateSpace::distance(const ob::State *state1, 
					     const ob::State *state2) const {
  return joint_space_distance(state1, state2);
}

double darrt::RobotChainStateSpace::cartesian_distance(const ob::State *state1,
						       const ob::State *state2) const {
  gm::PoseStamped pose1, pose2;
  forward_kinematics(state1, pose1);
  forward_kinematics(state2, pose2);
  return darrt::distance(pose1.pose.position, pose2.pose.position);
}

double darrt::RobotChainStateSpace::joint_space_distance(const ob::State *state1,
							 const ob::State *state2) const {
  // double dist = -1;
  // const ob::CompoundState *s1 = state1->as<ob::CompoundState>();
  // const ob::CompoundState *s2 = state2->as<ob::CompoundState>();
  // for (unsigned int i = 0; i < getSubspaceCount(); i++) {
  //   double sdist = getSubspace(i)->distance(s1->components[i], s2->components[i]);
  //   if (sdist > dist) {
  //     dist = sdist;
  //   }
  // }
  //return dist;
  //this adds... i think we want maximum... but it's kind
  //of a pain to calculate because the RealVectorStateSpace
  //has a bunch of joints.  so i guess this is reasonable.
  return ob::CompoundStateSpace::distance(state1, state2)/((double)getSubspaceCount());;
}

void darrt::RobotChainStateSpace::printState(const ob::State *state,
					     std::ostream &out) const {
  out << "(Joints: [";
  const ob::CompoundState *s = state->as<ob::CompoundState>();
  for (size_t i = 0; i < components_.size(); i++) {
    switch(ompl_to_robot_mapping_.mapping_type[i]) {
    case ompl_ros_conversions::SO2: 
      {
	out << s->components[i]->as<ob::SO2StateSpace::StateType>()->value;
	out << " ";
	break;
      }

    case ompl_ros_conversions::REAL_VECTOR: 
      {
	for (unsigned int j = 0; 
	     j < components_[i]->as<ob::RealVectorStateSpace>()->getDimension();
	     j++) {
	  out << s->components[i]->
	    as<ob::RealVectorStateSpace::StateType>()->values[j];
	  out << " ";
	}
	break;
      }
    default:
      ROS_ERROR("Unknown mapping type %d in printState",
		ompl_to_robot_mapping_.mapping_type[i]);
      std::string buff;
      //getline(std::cin, buff);
      break;
    }
  }
  out << "]";
  
  geometry_msgs::PoseStamped pose;
  forward_kinematics(state, pose);

  out << ", Wrist Pose: ("+pose.header.frame_id << ", " << pose.pose.position.x
      << ", " << pose.pose.position.y << ", " << pose.pose.position.z
      << ")";
  out << ")";
}


visualization_msgs::MarkerArray darrt::RobotChainStateSpace::displayable_state
(const ob::State *state, std::string ns, int id, 
 double scale, ColorPalate p, double alpha, bool minimal) const {
  //set up the colors
  float r, g, b;
  Displayable::interpret_palate(p, r, g, b);

  geometry_msgs::PoseStamped robot_pose;
  forward_kinematics(state, robot_pose);
  visualization_msgs::MarkerArray marray;
  marray.markers.push_back
    (make_marker(robot_pose, visualization_msgs::Marker::CUBE,
		   scale, r, g, b, alpha, ns, id));
  return marray;
}

void darrt::RobotChainStateSpace::set_joint_positions(const std::vector<double> &positions, ob::State *state) const {
  joint_positions_to_ompl_state(positions, robot_to_ompl_mapping_, kinematics_solver_->getJointNames(), state);
}

bool darrt::RobotChainStateSpace::near_states
(const ob::State *s1, const ob::State *s2, double deps, double aeps) const {
  //check each subspace individually
  //kind of assumes joints are angles
  //should do something smarter involving the mapping but who cares for now

  for (size_t i = 0; i < components_.size(); i++) {
    if (fabs(components_[i]->distance(s1->as
				      <ob::CompoundState>()->components[i], 
				      s2->as
				      <ob::CompoundState>()->components[i])) 
	> aeps) {
      return false;
    }
  }
  return true;
}

double darrt::RobotChainStateSpace::between(const ob::State *state,
					    const ob::State *source,
					    const ob::State *destination,
					    double deps, double aeps) const {
  const ob::CompoundState *s = state->as<ob::CompoundState>();
  const ob::CompoundState *s1 = source->as<ob::CompoundState>();
  const ob::CompoundState *s2 = destination->as<ob::CompoundState>();

  if (debug_level == DBETWEEN) {
    ROS_INFO("IN BETWEEN");
    std::ostringstream str;
    str << "Source is\n";
    ob::CompoundStateSpace::printState(source, str);
    str << "\nDestination is\n";
    ob::CompoundStateSpace::printState(destination, str);
    str << "\nState is";
    ob::CompoundStateSpace::printState(state, str);
    ROS_INFO("%s", str.str().c_str());
  }
  double f = 2.0;
  for (size_t i = 0; i < components_.size(); i++) {
    double b = -1;
    switch(ompl_to_robot_mapping_.mapping_type[i]) {
    case ompl_ros_conversions::SO2: 
      {
	b = darrt::between
	  (s->components[i]->as<ob::SO2StateSpace::StateType>(),
	   s1->components[i]->as<ob::SO2StateSpace::StateType>(),
	   s2->components[i]->as<ob::SO2StateSpace::StateType>(), aeps);
	break;
      }
    case ompl_ros_conversions::REAL_VECTOR: 
      {
	b = darrt::between(s->components[i]->as
			   <ob::RealVectorStateSpace::StateType>(),
			   s1->components[i]->as
			   <ob::RealVectorStateSpace::StateType>(),
			   s2->components[i]->as
			   <ob::RealVectorStateSpace::StateType>(),
			   components_[i]->as<ob::RealVectorStateSpace>()->
			   getDimension(), deps);
	break;
      }
    default:
      ROS_ERROR("Unknown mapping type %d in between",
		ompl_to_robot_mapping_.mapping_type[i]);
      std::string buff;
      //getline(std::cin, buff);
    }
     
    if (debug_level == DBETWEEN) {
      ROS_INFO("b = %f, f = %f", b, f);
    }
    if (b < 0) {
      return -1;
    }
    if (f > 1.0) {
      f = b;
    } else if (b < 1.1 && fabs(f - b) > deps) {
      return -1;
    }
  }
  return f;
}

bool darrt::RobotChainStateSpace::forward_kinematics
(const ob::State *state, geometry_msgs::PoseStamped &pose) const {
  std::vector<std::string> link_names = kinematics_solver_->getJointNames();  

  std::vector<double> positions;
  if (!ompl_state_to_joint_positions(state, ompl_to_robot_mapping_, link_names,
				     positions)) {
    ROS_ERROR("robot_position: unable to get joint positions");
    return false;
  }

  std::vector<geometry_msgs::Pose> poses;
  if (!kinematics_solver_->getPositionFK(kinematics_solver_->getJointNames(), 
					 positions, poses)) {
    ROS_ERROR("robot_position: unable to get position FK");
    return false;
  }
  //the last pose is (hopefully) the pose of the end effector
  pose.header.frame_id = kinematics_solver_->getBaseName();
  pose.header.stamp = ros::Time::now();
  pose.pose = poses[poses.size()-1];
  return true;
}

bool darrt::RobotChainStateSpace::inverse_kinematics
(const geometry_msgs::PoseStamped &pose, std::map<std::string, double> &pos_map) const {
  std::vector<double> positions, seed;
  if (!inverse_kinematics(pose, positions, seed)) {
    return false;
  }
  if (positions.size() != kinematics_solver_->getJointNames().size()) {
    ROS_ERROR("Inverse kinematics: Number of joint positions and names is not the same!");
    return false;
  }

  for (size_t i = 0; i < kinematics_solver_->getJointNames().size(); i++) {
    pos_map[kinematics_solver_->getJointNames().at(i)] = positions[i];
  }
  return true;
}

bool darrt::RobotChainStateSpace::inverse_kinematics
(const geometry_msgs::PoseStamped &pose, std::vector<double> &positions, 
 const std::vector<double> &seed_positions_in) const {
  std::string pframe = pose.header.frame_id;
  if (pframe[0] == '/') {
    pframe = pframe.substr(1,pframe.size());
  }
  std::string bframe = kinematics_solver_->getBaseName();
  if (bframe[0] == '/') {
    bframe = bframe.substr(1,bframe.size());
  }
  if (pframe.compare(bframe)) {
    ROS_ERROR_STREAM("Pose is " << pose);
    pause("Inverse kinematics can only solve for poses in the base frame of the solver: "
	  + bframe);
    return false;
  }

  gm::Pose ik_pose = pose.pose;
	  
  std::vector<double> seed_positions = seed_positions_in;
  if (seed_positions.size() != kinematics_solver_->getJointNames().size()) {
    ROS_DEBUG("Generating random seed");
    //seed randomly
    seed_positions.resize(kinematics_solver_->getJointNames().size(), 0.0);
    for (size_t i = 0; i < seed_positions.size(); i++) {
      if (!kinematic_info_.limits[i].has_position_limits) {
	//assume it's an angle i think
	seed_positions[i] = ((double)rand())/((double)RAND_MAX)*2.0*MATH_PI;
      } else {
	double minl = kinematic_info_.limits[i].min_position;
	double maxl = kinematic_info_.limits[i].max_position;
	seed_positions[i] = 
	  ((double)rand())/((double)RAND_MAX)*(maxl-minl)+minl;
      }
    }  
  }
 
  int error_code;
  if (!kinematics_solver_->searchPositionIK(ik_pose, seed_positions, 5.0, positions,
					    error_code)) {
    ROS_DEBUG("inverse kinematics: unable to get position IK for pose (%f, %f, %f, %f, %f, %f, %f).  Error: %d", ik_pose.position.x, ik_pose.position.y, ik_pose.position.z, ik_pose.orientation.x, ik_pose.orientation.y, ik_pose.orientation.z, ik_pose.orientation.w, error_code);
    return false;
  }
  return true;
  
}

bool darrt::RobotChainStateSpace::inverse_kinematics
(const geometry_msgs::PoseStamped &pose, ob::State *state) const {
  return inverse_kinematics(pose, state, NULL);
}

bool darrt::RobotChainStateSpace::inverse_kinematics
(const geometry_msgs::PoseStamped &pose, ob::State *state,  const ob::State *seed_state) 
  const {
  std::vector<double> seed_positions;
  if (seed_state) {
    std::ostringstream ostr;
    ostr << "Seed state provided: ";
    printState(seed_state, ostr);
    ROS_DEBUG("%s", ostr.str().c_str());
    if (!ompl_state_to_joint_positions(seed_state, ompl_to_robot_mapping_,
				       kinematics_solver_->getJointNames(), 
				       seed_positions)) {
      ROS_WARN("pose to ompl state: seed state provided but unable to get seed joint positions");
      
    }
  }
 
  std::vector<double> positions;
  if (!inverse_kinematics(pose, positions, seed_positions)) {
    return false;
  }

  //now convert the joint positions to an ompl state
  if (!joint_positions_to_ompl_state(positions, robot_to_ompl_mapping_,
				     kinematics_solver_->getJointNames(),
				     state)) {
    ROS_ERROR("pose to ompl state: unable to convert joint positions to ompl state");
    return false;
  }
  return true;
  
}

bool darrt::RobotChainStateSpace::update_model
(const ob::State *new_state, EnvironmentInterface *env) const {
  
  planning_models::KinematicState::JointStateGroup *joint_state_group = 
    env->kinematicState()->getJointStateGroup(physical_joint_group_->getName());

  if (!ompl_ros_conversions::ompl_state_to_kinematic_state_group
      (new_state, ompl_to_kinematic_mapping_, joint_state_group)) {
    ROS_ERROR("Unable to convert ompl state to joint state group");
    return false;
  }
  std::vector<planning_models::KinematicState::JointState *> joint_states =
    joint_state_group->getJointStateVector();
  
  for(unsigned int i=0; i < joint_states.size(); i++) {
    if(!joint_states[i]->areJointStateValuesWithinBounds()) {
      ROS_ERROR("State violates joint limits for Joint %s",
		joint_states[i]->getName().c_str());
      return false;
    }
  }

  joint_state_group->updateKinematicLinks();
  return true;
}

//will add names to robot state if necessary
void darrt::RobotChainStateSpace::convert_ompl_to_robot_state
(const ob::State *ompl_state, an::RobotState &robot_state) const {
  //create a robot state just corresponding to this chain of joints
  an::RobotState restricted_robot_state;
  restricted_robot_state.joint_state.name = kinematics_solver_->getJointNames();
  restricted_robot_state.joint_state.position.resize
    (restricted_robot_state.joint_state.name.size(), 0.0);
  orc::ompl_state_to_robot_state(ompl_state, ompl_to_robot_mapping_, 
				 restricted_robot_state);
  //match it up with the passed in state
  for (unsigned int i = 0; i < restricted_robot_state.joint_state.name.size(); i++) {
    bool matched = false;
    for (unsigned int j = 0; j < robot_state.joint_state.name.size(); j++) {
      if (!restricted_robot_state.joint_state.name[i].compare
	  (robot_state.joint_state.name[j])) {
	matched = true;
	robot_state.joint_state.position[j] = 
	  restricted_robot_state.joint_state.position[i];
	break;
      }
    }
    if (!matched) {
      robot_state.joint_state.name.push_back(restricted_robot_state.joint_state.name[i]);
      robot_state.joint_state.position.push_back
	(restricted_robot_state.joint_state.position[i]);
    }
  }
}

void darrt::RobotChainStateSpace::convert_robot_to_ompl_state
(const an::RobotState &robot_state, ob::State *ompl_state) const {
  an::RobotState restricted_robot_state;
  restrict_to_group(robot_state, restricted_robot_state);
  orc::robot_state_to_ompl_state(restricted_robot_state, robot_to_ompl_mapping_,
				 ompl_state, false);
}

//kinematic state must already be initialized with the robot model
//(there's no other way to create one so that's not worrying)
void darrt::RobotChainStateSpace::convert_ompl_to_kinematic_state
(const ob::State *ompl_state, pm::KinematicState &kstate) const {
  planning_models::KinematicState::JointStateGroup *joint_state_group = 
    kstate.getJointStateGroup(physical_joint_group_->getName());
  orc::ompl_state_to_kinematic_state_group(ompl_state, ompl_to_kinematic_mapping_, joint_state_group);
  joint_state_group->updateKinematicLinks();
}

void darrt::RobotChainStateSpace::convert_kinematic_to_ompl_state
(const pm::KinematicState &kstate, ob::State *ompl_state) const {
  orc::kinematic_state_group_to_ompl_state
    (kstate.getJointStateGroup(physical_joint_group_->getName()), 
			       kinematic_to_ompl_mapping_, ompl_state);
}
