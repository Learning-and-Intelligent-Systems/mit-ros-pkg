//local includes
#include "darrt/robot_base_space.hh"
#include "darrt/transform_ros_types.hh"
#include "darrt/utils.hh"

//ROS includes
#include <ros/ros.h>
#include <arm_navigation_msgs/GetRobotState.h>
#include <arm_navigation_msgs/RobotState.h>
#include <planning_environment/models/model_utils.h>


//quite a lot of this parallels the planning group implementation
//in ompl_ros_interface
//you could just initialize a joint_space_planner, but that requires
//exporting the ompl_ros_interface library and makes us dependent on its
//quirks
darrt::RobotBaseStateSpace::RobotBaseStateSpace
(const ros::NodeHandle &node_handle, const std::string &group_name,
 EnvironmentInterface *env, const ob::RealVectorBounds &bounds) : 
  RobotStateSpace(), bounds_(bounds) {
  initialized_ = false;
  node_ = node_handle;
  group_name_ = group_name;
  environment_interface_ = env;
  //this is all that needs to be done here
  //the heavy lifting is all in init
  init();
}

void darrt::RobotBaseStateSpace::setup() {
  if (!initialized_) {
    init();
  }
}

bool darrt::RobotBaseStateSpace::init() {
  initialized_ = false;

  //figure out the actual name of the group
  //and get the collision model
  if (!initialize_physical_group()) {
    return false;
  }

  //does the physical group have a single multi DOF joint?
  const std::vector<const pm::KinematicModel::JointModel *> joints = 
    physical_joint_group_->getJointModels();
  if (joints.size() != 1) {
    ROS_ERROR("Robot base group must have exactly one multi DOF joint.  Group %s has %zu",
	      group_name_.c_str(), joints.size());
    return false;
  }
  multi_dof_joint_name_ = joints[0]->getName();
  multi_dof_joint_ = dynamic_cast<const pm::KinematicModel::FloatingJointModel *>
    (joints[0]);
  if (!multi_dof_joint_) {
    ROS_ERROR("Robot base group's joint must be a multi DOF joint.  Joint %s is not.",
	      multi_dof_joint_name_.c_str());
    return false;
  }

  //add an SE2 state space corresponding to this joint
  ob::StateSpacePtr subspace(new ob::SE2StateSpace());
  subspace->as<ob::SE2StateSpace>()->setBounds(bounds_);
  subspace->setName(multi_dof_joint_name_);
  addSubspace(subspace, 1.0);

  setName(physical_joint_group_->getName());
  initialized_ = true;
  return true;
}

bool darrt::RobotBaseStateSpace::initialize_physical_group() {
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


darrt::RobotBaseStateSpace::BaseState *darrt::RobotBaseStateSpace::base_state
(ob::State *state) const {
  return state->as<ob::CompoundState>()->as<BaseState>(0);
}


const darrt::RobotBaseStateSpace::BaseState *darrt::RobotBaseStateSpace::base_state
(const ob::State *state) const {
  return state->as<ob::CompoundState>()->as<BaseState>(0);
}

gm::Pose2D darrt::RobotBaseStateSpace::get_pose(const ob::State *state) const {
  gm::Pose2D pose;
  orc::SE2StateSpace_to_pose2D_msg(*(base_state(state)), pose);
  return pose;
}

void darrt::RobotBaseStateSpace::set_pose(ob::State *state, const gm::Pose2D &pose) const {
  orc::pose2D_msg_to_SE2StateSpace(pose, *(base_state(state)));
}

void darrt::RobotBaseStateSpace::printState(const ob::State *state,
					    std::ostream &out) const {
  const BaseState *s = base_state(state);
  out << "(Base: (" << s->getX() << ", " << s->getY() << ", " << s->getYaw() << "))";
}


bool darrt::RobotBaseStateSpace::near_states
(const ob::State *state1, const ob::State *state2, double deps, double aeps) const {
  const BaseState *s1 = base_state(state1);
  const BaseState *s2 = base_state(state2);
 
  return angular_distance(s1->getYaw(), s2->getYaw()) < aeps &&
    distance2D(s1->getX(), s1->getY(), s2->getX(), s2->getY()) < deps;
}

double darrt::RobotBaseStateSpace::between(const ob::State *state,
					    const ob::State *source,
					    const ob::State *destination,
					    double deps, double aeps) const {
  return darrt::between(base_state(state), base_state(source), base_state(destination),
			deps, aeps);
}

double darrt::RobotBaseStateSpace::distance(const ob::State *state1, 
					     const ob::State *state2) const {
  //we care more about x, y than we do about theta
  const BaseState *s1 = base_state(state1);
  const BaseState *s2 = base_state(state2);
  return distance2D(s1->getX(), s1->getY(), s2->getX(), s2->getY()) + 
    0.1*angular_distance(s1->getYaw(), s2->getYaw());
}

bool darrt::RobotBaseStateSpace::update_model
(const ob::State *new_state, EnvironmentInterface *env) const {
  convert_ompl_to_kinematic_state(new_state, *(env->kinematicState()));
  return true;
}

//will add names to robot state if necessary
void darrt::RobotBaseStateSpace::convert_ompl_to_robot_state
(const ob::State *ompl_state, an::RobotState &robot_state) const {
  gm::Pose2D pose = get_pose(ompl_state);
  bool found = false;
  for (unsigned int i = 0; i < robot_state.multi_dof_joint_state.joint_names.size(); i++){
    if (!multi_dof_joint_name_.compare(robot_state.multi_dof_joint_state.joint_names[i])){
      found = true;
      robot_state.multi_dof_joint_state.poses[i] = pose2D_to_ground_pose(pose);
    }
  }
  if (!found) {
    robot_state.multi_dof_joint_state.joint_names.push_back(multi_dof_joint_name_);
    robot_state.multi_dof_joint_state.frame_ids.push_back
      (multi_dof_joint_->getParentFrameId());
    robot_state.multi_dof_joint_state.child_frame_ids.push_back
      (multi_dof_joint_->getChildFrameId());
    robot_state.multi_dof_joint_state.poses.push_back(pose2D_to_ground_pose(pose));
  }
}

void darrt::RobotBaseStateSpace::convert_robot_to_ompl_state
(const an::RobotState &robot_state, ob::State *ompl_state) const {
  for (unsigned int i = 0; i < robot_state.multi_dof_joint_state.joint_names.size(); i++){
    if (!robot_state.multi_dof_joint_state.joint_names[i].compare(multi_dof_joint_name_)){
      orc::pose_msg_to_SE2StateSpace(robot_state.multi_dof_joint_state.poses[i],
				     *(base_state(ompl_state)));
      return;
    }
  }
  ROS_ERROR_STREAM("Unable to find multi DOF joint state " << multi_dof_joint_name_
		   << " in robot state\n" << robot_state); 
}

//kinematic state must already be initialized with the robot model
//(there's no other way to create one so that's not worrying)
void darrt::RobotBaseStateSpace::convert_ompl_to_kinematic_state
(const ob::State *ompl_state, pm::KinematicState &kstate) const {

  gm::Pose pose = pose2D_to_ground_pose(get_pose(ompl_state));
  
  pm::KinematicState::JointStateGroup *joint_state_group = 
    kstate.getJointStateGroup(physical_joint_group_->getName());
  pm::KinematicState::JointState *multi_dof_joint = 
    joint_state_group->getJointState(multi_dof_joint_name_);
  std::vector<double> values = multi_dof_joint->getJointStateValues();
  
  values[0] = pose.position.x;
  values[1] = pose.position.y;
  values[5] = pose.orientation.z;
  values[6] = pose.orientation.w;
  multi_dof_joint->setJointStateValues(values);
  joint_state_group->updateKinematicLinks();
}

void darrt::RobotBaseStateSpace::convert_kinematic_to_ompl_state
(const pm::KinematicState &kstate, ob::State *ompl_state) const {
  BaseState *s = base_state(ompl_state);

  const pm::KinematicState::JointStateGroup *joint_state_group = 
    kstate.getJointStateGroup(physical_joint_group_->getName());
  const pm::KinematicState::JointState *multi_dof_joint = 
    joint_state_group->getJointState(multi_dof_joint_name_);
  std::vector<double> values = multi_dof_joint->getJointStateValues();

  
  s->setX(values[0]);
  s->setY(values[1]);
  gm::Quaternion q;
  q.x = values[3];
  q.y = values[4];
  q.z = values[5];
  q.w = values[6];
  s->setYaw(quaternion_to_yaw(q));
}
