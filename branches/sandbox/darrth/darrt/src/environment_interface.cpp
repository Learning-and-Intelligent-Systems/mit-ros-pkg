#include "darrt/environment_interface.hh"
#include "darrt/utils.hh"

darrt::EnvironmentInterface::EnvironmentInterface(const std::string &description) :
  pe::CollisionModels(description) {
  kinematic_state_ = NULL;
  initApproxEnvironment();
}

darrt::EnvironmentInterface::EnvironmentInterface(const std::string &description, 
						  const pm::KinematicState &kinematic_state) :
  pe::CollisionModels(description) {

  kinematic_state_ = new pm::KinematicState(kinematic_state);
  initApproxEnvironment();
}

darrt::EnvironmentInterface::~EnvironmentInterface() {
  if (kinematic_state_) {
    delete kinematic_state_;
  }
  if (approximate_env_) {
    delete approximate_env_;
  }
  if (grid_) {
    delete grid_;
  }
  if (rarm_) {
    delete rarm_;
  }
  if (larm_) {
    delete larm_;
  }
}

void darrt::EnvironmentInterface::initApproxEnvironment() {
  approximate_env_ = NULL;
  grid_ = NULL;
  rarm_ = NULL;
  larm_ = NULL;
  //ok, this is lame but i don't care right now - make it more general
  rarm_names_.resize(7);
  larm_names_.resize(7);
  rarm_names_[0] = "_shoulder_pan_joint";
  rarm_names_[1] = "_shoulder_lift_joint";
  rarm_names_[2] = "_upper_arm_roll_joint";
  rarm_names_[3] = "_elbow_flex_joint";
  rarm_names_[4] = "_forearm_roll_joint";
  rarm_names_[5] = "_wrist_flex_joint";
  rarm_names_[6] = "_wrist_roll_joint";
  for (int i = 0; i < 7; i++) {
    larm_names_[i] = "l" + rarm_names_[i];
    rarm_names_[i] = "r" + rarm_names_[i];
  }
  

  ros::NodeHandle nh("~");

  
  std::string rarm_filename, larm_filename;
  double resolution;
  nh.param<std::string>("/move_base/planner/left_arm_description_file", larm_filename, "");
  nh.param<std::string>("/move_base/planner/right_arm_description_file", rarm_filename, "");
  nh.param("/move_base/collision_space/resolution",resolution,0.02);
  
  FILE *rarm = fopen(rarm_filename.c_str(), "r");
  if (!rarm) {
    ROS_ERROR("Unable to open right arm file %s.  Not using approximate checks.", rarm_filename.c_str());
    return;
  }
  FILE *larm = fopen(larm_filename.c_str(), "r");
  if (!larm) {
    ROS_ERROR("Unable to open left arm file %s.  Not using approximate checks.", larm_filename.c_str());
    fclose(rarm);
    return;
  }
  rarm_ = new sbpl_arm_planner::SBPLArmModel(rarm);
  rarm_->setResolution(resolution);
  if (!rarm_->initKDLChainFromParamServer()) {
    ROS_ERROR("Unable to initialize right arm.  Not using approximate checks.");
    fclose(rarm);
    fclose(larm);
    return;
  }
  larm_ = new sbpl_arm_planner::SBPLArmModel(larm);
  larm_->setResolution(resolution);
  fclose(rarm);
  fclose(larm);
  if (!larm_->initKDLChainFromParamServer()) {
    ROS_ERROR("Unable to initialize left arm.  Not using approximate checks.");
    return;
  }
  ROS_INFO("Arms are:");
  std::string rs, ls;
  rarm_->printArmDescription(rs);
  larm_->printArmDescription(ls);
  ROS_INFO("%s", rs.c_str());
  ROS_INFO("%s", ls.c_str());

  //read the parameters off the server
  std::string reference_frame;
  double size_x, size_y, size_z, origin_x, origin_y, origin_z;
  nh.param<std::string>("/move_base/collision_space/reference_frame",reference_frame,"base_link");
  nh.param("/move_base/collision_space/occupancy_grid/origin_x",origin_x,-0.6);
  nh.param("/move_base/collision_space/occupancy_grid/origin_y",origin_y,-1.15);
  nh.param("/move_base/collision_space/occupancy_grid/origin_z",origin_z,-0.05);  
  nh.param("/move_base/collision_space/occupancy_grid/size_x",size_x,1.6);
  nh.param("/move_base/collision_space/occupancy_grid/size_y",size_y,1.8);
  nh.param("/move_base/collision_space/occupancy_grid/size_z",size_z,1.4);

  grid_ = new sbpl_arm_planner::OccupancyGrid(size_x, size_y, size_z, resolution, origin_x, origin_y, origin_z);
  grid_->setReferenceFrame(reference_frame);
  approximate_env_ = new pr2_collision_checker::PR2CollisionSpace(rarm_, larm_, grid_);
  if (!approximate_env_->getSphereGroups()) {
    ROS_ERROR("Unable to get sphere groups.  Approximate collision checking will be turned off.");
    delete approximate_env_;
    approximate_env_ = NULL;
    return;
  }

  updateApproximateEnvironment();
}

void darrt::EnvironmentInterface::updateApproximateEnvironment() {
  if (!approximate_env_) {
    return;
  }
  an::CollisionMap cmap;
  getCollisionSpaceCollisionMap(cmap);
  grid_->updateFromCollisionMap(cmap);
  //so we don't keep copying this around
  approximate_env_->storeCollisionMap(cmap);

  //add in the objects in the world
  std::vector<an::CollisionObject> cos;
  getCollisionSpaceCollisionObjects(cos);
  for (unsigned int i = 0; i < cos.size(); i++) {
    approximate_env_->processCollisionObjectMsg(cos[i]);
  }
}

void darrt::EnvironmentInterface::removeObjectFromApproximateEnvironment(const an::CollisionObject &obj) {
  if (approximate_env_) {
    approximate_env_->removeCollisionObject(obj);
  }
}

void darrt::EnvironmentInterface::robotPose
(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &pose) const{ 
  if (!kinematic_state_) {
    return;
  }
  an::RobotState robot_state;
  //possibly not the most efficient way to do this... we'll see if it matters
  pe::convertKinematicStateToRobotState(*kinematic_state_, robot_state.joint_state.header.stamp, 
					getWorldFrameId(), robot_state);
  langles.resize(larm_names_.size());
  rangles.resize(rarm_names_.size());
  for (unsigned int i = 0; i < robot_state.joint_state.name.size(); i++) {
    if (!robot_state.joint_state.name[i].compare(torso_joint())) {
      pose.z = robot_state.joint_state.position[i];
      continue;
    }
    bool matched = false;
    for (unsigned int j = 0; j < rangles.size(); j++) {
      if (!rarm_names_[j].compare(robot_state.joint_state.name[i])) {
	rangles[j] = robot_state.joint_state.position[i];
	matched = true;
	break;
      }
    }
    if (matched) {
      continue;
    }
    for (unsigned int j = 0; j < langles.size(); j++) {
      if (!larm_names_[j].compare(robot_state.joint_state.name[i])) {
	langles[j] = robot_state.joint_state.position[i];
	break;
      }
    }
  }
  pose.x = robot_state.multi_dof_joint_state.poses[0].position.x;
  pose.y = robot_state.multi_dof_joint_state.poses[0].position.y;
  pose.theta = quaternion_to_yaw(robot_state.multi_dof_joint_state.poses[0].orientation);
}

visualization_msgs::Marker darrt::EnvironmentInterface::getGridMarker() {
  visualization_msgs::Marker marker;
  grid_->getVisualizationMarker(marker);
  return marker;
}
