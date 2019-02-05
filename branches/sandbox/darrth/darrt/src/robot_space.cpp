//local includes
#include "darrt/chain_space.hh"
#include "darrt/robot_base_space.hh"
#include "darrt/robot_space.hh"
#include "darrt/transform_ros_types.hh"
#include "darrt/utils.hh"

//ROS includes
#include <ros/ros.h>
#include <arm_navigation_msgs/GetRobotState.h>
#include <arm_navigation_msgs/RobotState.h>
#include <geometry_msgs/Pose.h>
#include <planning_environment/models/model_utils.h>


darrt::CompoundRobotStateSpace::CompoundRobotStateSpace
(const ros::NodeHandle &node_handle, EnvironmentInterface *env) :
  RobotStateSpace(), Displayable("robot_state_markers") {
  node_ = node_handle;
  environment_interface_ = env;
  display_id_ = new int(0);
  setName("CompoundRobot");
}

void darrt::CompoundRobotStateSpace::addSubspace
(std::string group_name, const ob::RealVectorBounds &bounds) {
  std::string physical_group_name;
  if(!environment_interface_->getKinematicModel()->hasModelGroup(group_name)) {
    if(!node_.hasParam(group_name+"/physical_group")) {
      ROS_ERROR("No physical group specified for %s",group_name.c_str());
      return;
    } else {
      node_.getParam(group_name+"/physical_group",physical_group_name);
    }
  } else {
    physical_group_name = group_name;
  }

  const pm::KinematicModel::JointModelGroup *physical_joint_group = 
    environment_interface_->getKinematicModel()->getModelGroup(physical_group_name);
  ob::StateSpacePtr subspace;
  if (physical_joint_group->getJointModels().size() == 1 &&
      dynamic_cast<const pm::KinematicModel::FloatingJointModel *>
      (physical_joint_group->getJointModels().at(0))) {
    subspace.reset(new RobotBaseStateSpace
		   (node_, group_name, environment_interface_, bounds));
  } else {
    subspace.reset(new RobotChainStateSpace(node_, group_name, environment_interface_));
  }
  ob::CompoundStateSpace::addSubspace(subspace, 1.0);
}

ob::State *darrt::CompoundRobotStateSpace::get_state(unsigned int ind, ob::State *state) 
  const {
  return state->as<ob::CompoundState>()->components[ind];
}

const ob::State *darrt::CompoundRobotStateSpace::get_state
(unsigned int ind, const ob::State *state) const {
  return state->as<ob::CompoundState>()->components[ind];
}

darrt::RobotStateSpace *darrt::CompoundRobotStateSpace::get_subspace(unsigned int ind) {
  return getSubspace(ind)->as<RobotStateSpace>();
}

const darrt::RobotStateSpace *darrt::CompoundRobotStateSpace::get_subspace
(unsigned int ind) const {
  return getSubspace(ind)->as<RobotStateSpace>();
}

void darrt::CompoundRobotStateSpace::printState(const ob::State *state,
					     std::ostream &out) const {
  out << "(RobotState: ";
  for (unsigned int i = 0; i < getSubspaceCount(); i++) {
    get_subspace(i)->printState(get_state(i, state), out);
  }
}

ob::State *darrt::CompoundRobotStateSpace::allocState() const {
  StateType *state = new StateType();
  allocStateComponents(state);
  return state;
}

double darrt::CompoundRobotStateSpace::distance(const ob::State *destination, const ob::State *source) const {
  double dist = -1.0;
  const StateType *d = destination->as<StateType>();
  for (unsigned int i = 0; i < getSubspaceCount(); i++) {
    double sdist = getSubspace(i)->distance(d->components[i], source->as<StateType>()->components[i]);
    if (debug_level >= DDISTANCE) {
      ROS_INFO("\tDistance in robot subspace %s is %f", getSubspace(i)->getName().c_str(), sdist);
    }
    if (sdist > dist) {
      dist = sdist;
    }
  }
  return dist;
}

void darrt::CompoundRobotStateSpace::convert_ompl_to_robot_state
(const ob::State *ompl_state, an::RobotState &robot_state) const {
  
  pm::KinematicState *kstate = environment_interface_->kinematicState();
  //note: we don't use convert robot state here because that may return
  //a robot state missing some joints
  convert_ompl_to_kinematic_state(ompl_state, *kstate);
  pe::convertKinematicStateToRobotState
    (*kstate, robot_state.joint_state.header.stamp, 
     robot_state.joint_state.header.frame_id, robot_state);
}

void darrt::CompoundRobotStateSpace::convert_robot_to_ompl_state
(const an::RobotState &robot_state, ob::State *ompl_state) const {
  for (unsigned int i = 0; i < getSubspaceCount(); i++) {
    get_subspace(i)->convert_robot_to_ompl_state(robot_state, get_state(i, ompl_state));
  }
}

void darrt::CompoundRobotStateSpace::convert_ompl_to_kinematic_state
(const ob::State *ompl_state, pm::KinematicState &kstate) const {
  
  //kstate = *(environment_interface_->kinematicState());
  for (unsigned int i = 0; i < getSubspaceCount(); i++) {
    get_subspace(i)->convert_ompl_to_kinematic_state(get_state(i, ompl_state), kstate);
  }
  kstate.updateKinematicLinks();
}

void darrt::CompoundRobotStateSpace::convert_kinematic_to_ompl_state
(const pm::KinematicState &kstate, ob::State *ompl_state) const {
  for (unsigned int i = 0; i < getSubspaceCount(); i++) {
    get_subspace(i)->convert_kinematic_to_ompl_state(kstate, get_state(i, ompl_state));
  }
}

visualization_msgs::MarkerArray darrt::CompoundRobotStateSpace::displayable_state
(const ob::State *state, std::string ns, int id, double scale, ColorPalate p, 
 double alpha, bool minimal) const {
  pm::KinematicState *kstate = environment_interface_->kinematicState();
  convert_ompl_to_kinematic_state(state, *kstate);

  visualization_msgs::MarkerArray marray;
  std_msgs::ColorRGBA color;
  Displayable::interpret_palate(p, color.r, color.g, color.b);
  color.a = alpha;

  std::vector<std::string> *names = NULL;
  if (minimal) {
    names = new std::vector<std::string>(5);
    names->at(0) = std::string("r_gripper_r_finger_tip_link");
    names->at(1) = std::string("r_gripper_l_finger_tip_link");
    names->at(2) = std::string("r_gripper_r_finger_link");
    names->at(3) = std::string("r_gripper_l_finger_link");
    names->at(4) = std::string("r_gripper_palm_link");
  }
  environment_interface_->getRobotMarkersGivenState(*kstate, marray, color, ns, ros::Duration(0), names);
  if (names) {
    delete names;
  }
  //we want the ids to be unique but we don't want another namespace every 
  //time...
  for (unsigned int i = 0; i < marray.markers.size(); i++) {
    marray.markers[i].id = id*1000+i;
  }
  return marray;
}

bool darrt::CompoundRobotStateSpace::near_states
(const ob::State *s1, const ob::State *s2, double deps, double aeps) const {
  for (unsigned int i = 0; i < getSubspaceCount(); i++) {
    if (!get_subspace(i)->near_states(get_state(i, s1), get_state(i, s2),
				      deps, aeps)) {
      return false;
    }
  }
  return true;
}

double darrt::CompoundRobotStateSpace::between(const ob::State *state,
					       const ob::State *source,
					       const ob::State *destination,
					       double deps, double aeps) const {

  double f = 2.0;
  for (unsigned int i = 0; i < getSubspaceCount(); i++) {
    double b = get_subspace(i)->between
      (get_state(i, state), get_state(i, source), get_state(i, destination), 
       deps, aeps);
     
    if (debug_level == DBETWEEN) {
      ROS_INFO("top level robot between: b = %f, f = %f", b, f);
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

bool darrt::CompoundRobotStateSpace::update_model(const ob::State *new_state, 
						  EnvironmentInterface *env) const {
  bool ret = true;
  for (unsigned int i = 0; i < getSubspaceCount(); i++) {
    if (!get_subspace(i)->update_model(get_state(i, new_state), env)) {
      ret = false;
    }
  }
  return ret;
}

darrt::RobotStateValidityChecker::RobotStateValidityChecker
(ob::SpaceInformation *si, 
 collision_space::EnvironmentModel::AllowedCollisionMatrix 
 &default_allowed_collision_matrix, bool update_model) :
  CollisionAwareStateValidityChecker(si, default_allowed_collision_matrix) {
  update_model_ = update_model;
  space_ = si->getStateSpace()->as<RobotStateSpace>();
  ros::NodeHandle n("~");
  collision_pub_ =
    n.advertise<visualization_msgs::Marker>("collisions", 1);
}

bool darrt::RobotStateValidityChecker::init
(const an::GetMotionPlan::Request &request) {
  path_constraint_evaluator_set_.clear();
  an::Constraints path_constraints = 
    get_physical_constraints(request.motion_plan_request.path_constraints);

  path_constraint_evaluator_set_.add(path_constraints.joint_constraints);
  path_constraint_evaluator_set_.add(path_constraints.position_constraints);
  path_constraint_evaluator_set_.add(path_constraints.orientation_constraints);
  path_constraint_evaluator_set_.add(path_constraints.visibility_constraints);
  return true;
}

an::Constraints darrt::RobotStateValidityChecker::
get_physical_constraints(const an::Constraints &constraints) {
  an::Constraints result_constraints;
  for(unsigned int i=0; i < constraints.joint_constraints.size(); i++) {
    if(environment_interface_->getKinematicModel()->
       hasJointModel(constraints.joint_constraints[i].joint_name)) {
      result_constraints.joint_constraints.push_back
	(constraints.joint_constraints[i]);
    }
  }
  for(unsigned int i=0; i < constraints.position_constraints.size(); i++) {
    if(environment_interface_->getKinematicModel()->hasLinkModel
       (constraints.position_constraints[i].link_name)) {
      result_constraints.position_constraints.push_back
	(constraints.position_constraints[i]);
    }
  }

  for(unsigned int i=0; i < constraints.orientation_constraints.size(); i++) {
    if(environment_interface_->getKinematicModel()->hasLinkModel
       (constraints.orientation_constraints[i].link_name)) {
      result_constraints.orientation_constraints.push_back
	(constraints.orientation_constraints[i]);
    }
  }

  for(unsigned int i=0; i < constraints.visibility_constraints.size(); i++) {
    if(environment_interface_->getKinematicModel()->hasLinkModel
       (constraints.visibility_constraints[i].sensor_pose.header.frame_id)) {
      result_constraints.visibility_constraints.push_back
	(constraints.visibility_constraints[i]);
    }
  }

  return result_constraints;
}


bool darrt::RobotStateValidityChecker::isValid(const ob::State *state) 
  const {
  if (!environment_interface_) {
    ROS_ERROR("Must call setup before checking validity!");
    return false;
  }

  if (update_model_) {
    if (!space_->update_model(state, environment_interface_)) {
      return false;
    }
  }
  //first check the approximate model
  pr2_collision_checker::PR2CollisionSpace *aenv = environment_interface_->approximateEnvironment();
  bool approx_coll = true;
  if (aenv) {
    std::vector<double> langles, rangles;
    BodyPose pose;
    environment_interface_->robotPose(rangles, langles, pose);
    std::string rarma("["), larma("[");
    for (unsigned int i = 0; i < rangles.size(); i++) {
      rarma += makestring(rangles[i])+" ";
      larma += makestring(langles[i])+" ";
    }
    rarma += "]";
    larma += "]";
      
    //check collisions using the approximate mesh
    unsigned char dist_temp=255;
    int debug_code;
    bool verbose = false;
    if (debug_level >= DCOLLISIONS) {
      verbose = true;
      ROS_INFO("Doing approximate check");
      if (aenv->isBodyValid(pose.x, pose.y, pose.theta, pose.z, dist_temp)) {
	ROS_INFO("Valid body pose");
      } else {
	ROS_INFO("Invalid body pose");
      }
      if (aenv->checkCollisionArmsToBody(langles, rangles, pose, dist_temp)) {
	ROS_INFO("No collision between body and arms");
      } else {
	ROS_INFO("Collision betwen body and arms");
      }
      if (aenv->checkCollisionArms(langles, rangles, pose, true, dist_temp, debug_code)) {
	ROS_INFO("No collision between arms and world");
      } else {
	ROS_INFO("Collision between world and arms");
      }
    }
    if (aenv->isBodyValid(pose.x, pose.y, pose.theta, pose.z, dist_temp) &&
	aenv->checkCollisionArmsToBody(langles, rangles, pose, dist_temp) &&
	aenv->checkCollisionArms(langles, rangles, pose, verbose, dist_temp, debug_code)) {
      approx_coll = false;
    }
  }
  if (!approx_coll) {
    if (debug_level >= DCOLLISIONS) {
      ROS_INFO("Not doing full collision check.  Yay!");
    }
    return true;
  }

  const planning_models::KinematicState *kinematic_state = 
    environment_interface_->kinematicState();

  if(!path_constraint_evaluator_set_.decide(kinematic_state, false)) {
    ROS_DEBUG("Path constraints violated");
    return false;
  }

  //ros::WallTime n2 = ros::WallTime::now();
  if(environment_interface_->
     isKinematicStateInCollision(*kinematic_state)) {
    if (debug_level >= DRRT) {
      ROS_INFO("State is in collision");
    }
    if (debug_level >= DPROPAGATE) {
      std::vector<arm_navigation_msgs::ContactInformation> contacts;
      environment_interface_->getAllCollisionsForState(*kinematic_state,
							    contacts);
      ROS_INFO("There are %u contacts", (unsigned int)contacts.size());
      for (size_t i = 0; i < contacts.size(); i++) {
	ROS_INFO("Contact between %s and %s", 
		 contacts[i].contact_body_1.c_str(),
		 contacts[i].contact_body_2.c_str());
	//publish it
	geometry_msgs::PoseStamped cp;
	cp.header = contacts[i].header;
	cp.header.stamp = ros::Time(0);
	cp.pose.position = contacts[i].position;
	cp.pose.orientation.w = 1.0;
	display_marker(collision_pub_, cp, visualization_msgs::Marker::SPHERE,
		       0.05, 1.0, 0.0, 0.0, 0.8, contacts[i].contact_body_1+
		       "_"+contacts[i].contact_body_2, i);
      }
    }
  
    //ROS_INFO_STREAM("Positive collision check took " << (ros::WallTime::now()-n2).toSec());
    if (!approx_coll) {
      pause("Refined collision check collision, but no approximate collision!");
    }
    return false;
  }
  //ROS_INFO_STREAM("Negative collision check took " << (ros::WallTime::now()-n2).toSec());
  if (debug_level >= DCOLLISIONS && approx_coll) {
    pause("No refined collision, but an approximate collision!");
  }
  return true;
}
