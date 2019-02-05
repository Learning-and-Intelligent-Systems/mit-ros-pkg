#include "darrt/state_transformer.hh"
#include "darrt/transform_ros_types.hh"

darrt::StateTransformer::StateTransformer() :
  robot_model_("/robot_description") {
  world_frame_id_ = relative_frame(robot_model_.getWorldFrameId());
  robot_frame_id_ = relative_frame(robot_model_.getRobotFrameId());
}

std::string darrt::StateTransformer::relative_frame(std::string frame_id) 
  const {
  if (!frame_id.size() || frame_id[0] != '/') {
    return frame_id;
  }
  return frame_id.substr(1, frame_id.size());
}

bool darrt::StateTransformer::world_transform
(std::string frame_id, const planning_models::KinematicState &state,
 tf::Transform &transform) const {
  
  if (!frame_id.compare(world_frame_id())) {
    //identity transform
    transform.setIdentity();
    return true;
  }
  
  if (!frame_id.compare(robot_frame_id())) {
    transform = state.getRootTransform();
    return true;
  }

  const planning_models::KinematicState::LinkState *link =
    state.getLinkState(frame_id);
  if (!link) {
    ROS_ERROR("Unable to find link %s in kinematic state", frame_id.c_str());
    return false;
  }
    
  transform = link->getGlobalLinkTransform();
  return true; 
}

//this returns the pose of the origin of to_frame in from_frame
//this is NOT the transform to transform a point from from_frame into to_frame
//that transform is the inverse of the one returned by this
//confusing... but that's what TF does
gm::TransformStamped darrt::StateTransformer::get_transform
(std::string to_frame_id, std::string from_frame_id, 
 const an::RobotState &robot_state) const {

  planning_models::KinematicState kstate(robot_model_.getKinematicModel());
  planning_environment::setRobotStateAndComputeTransforms(robot_state, kstate);
  return get_transform(to_frame_id, from_frame_id, kstate);
}

gm::TransformStamped darrt::StateTransformer::get_transform
(std::string to_frame_id, std::string from_frame_id, 
 const planning_models::KinematicState &kstate) const {
  
  std::string to_frame = relative_frame(to_frame_id);
  std::string from_frame = relative_frame(from_frame_id);
  ROS_DEBUG("Transforming from %s to %s", from_frame.c_str(), to_frame.c_str());
  gm::TransformStamped ts;
  ts.header.frame_id = from_frame_id;
  ts.child_frame_id = to_frame_id;
  if (!to_frame.compare(from_frame)) {
    ts.transform.rotation.w = 1.0;
    return ts;
  }
  
  tf::Transform global_to_to, global_to_from;
  if (!world_transform(from_frame, kstate, global_to_from)) {
    throw StateTransformException
      ("Unable to get a transformation from world frame for " + from_frame);
  }
  if (!world_transform(to_frame, kstate, global_to_to)) {
    throw StateTransformException
      ("Unable to get a transformation from world frame for " + to_frame);
  }
  tf::Transform trans = (global_to_from.inverse())*global_to_to;
  tf::transformTFToMsg(trans, ts.transform);
  return ts;
}

gm::Point darrt::StateTransformer::transform
(std::string new_frame_id, std::string old_frame_id, const gm::Point &point,
 const an::RobotState &robot_state) const {
  gm::TransformStamped ts = get_transform(old_frame_id, new_frame_id, robot_state);
  return transform_point(point, ts.transform);
}

gm::Quaternion darrt::StateTransformer::transform
(std::string new_frame_id, std::string old_frame_id, const gm::Quaternion &quat,
 const an::RobotState &robot_state) const {
  gm::TransformStamped ts = get_transform(old_frame_id, new_frame_id, 
					  robot_state);
  return transform_quaternion(quat, ts.transform);
}

gm::Vector3 darrt::StateTransformer::transform
(std::string new_frame_id, std::string old_frame_id, const gm::Vector3 &vector,
 const an::RobotState &robot_state) const {
  gm::TransformStamped ts = get_transform(old_frame_id, new_frame_id, robot_state);
  return transform_vector(vector, ts.transform);
}

gm::Pose darrt::StateTransformer::transform
(std::string new_frame_id, std::string old_frame_id, const gm::Pose &pose,
 const an::RobotState &robot_state) const {
  gm::TransformStamped ts = get_transform(old_frame_id, new_frame_id, 
					  robot_state);
  return transform_pose(pose, ts.transform);
}

gm::PointStamped darrt::StateTransformer::transform
(std::string new_frame_id, const gm::PointStamped &point,
 const an::RobotState &robot_state) const {
  gm::TransformStamped ts = get_transform(point.header.frame_id, new_frame_id, 
					  robot_state);
  gm::PointStamped newpt;
  newpt.header.frame_id = new_frame_id;
  newpt.point = transform_point(point.point, ts.transform);
  return newpt;
}

gm::QuaternionStamped darrt::StateTransformer::transform
(std::string new_frame_id, const gm::QuaternionStamped &quat,
 const an::RobotState &robot_state) const {
  gm::TransformStamped ts = get_transform(quat.header.frame_id, new_frame_id, 
					  robot_state);
  gm::QuaternionStamped newquat;
  newquat.header.frame_id = new_frame_id;
  newquat.quaternion = transform_quaternion(quat.quaternion, ts.transform);
  return newquat;
}

gm::Vector3Stamped darrt::StateTransformer::transform
(std::string new_frame_id, const gm::Vector3Stamped &vs,
 const an::RobotState &robot_state) const {
  gm::TransformStamped ts = get_transform(vs.header.frame_id, new_frame_id, 
					  robot_state);
  gm::Vector3Stamped newvs;
  newvs.header.frame_id = new_frame_id;
  newvs.vector = transform_vector(vs.vector, ts.transform);
  return newvs;
}


gm::PoseStamped darrt::StateTransformer::transform
(std::string new_frame_id, const gm::PoseStamped &pose,
 const an::RobotState &robot_state) const {
  gm::TransformStamped ts = get_transform(pose.header.frame_id, new_frame_id, 
					  robot_state);
  gm::PoseStamped newps;
  newps.header.frame_id = new_frame_id;
  newps.pose = transform_pose(pose.pose, ts.transform);
  return newps;
}


gm::Point darrt::StateTransformer::transform
(std::string new_frame_id, std::string old_frame_id, const gm::Point &point,
 const planning_models::KinematicState &robot_state) const {
  gm::TransformStamped ts = get_transform(old_frame_id, new_frame_id, robot_state);
  return transform_point(point, ts.transform);
}

gm::Quaternion darrt::StateTransformer::transform
(std::string new_frame_id, std::string old_frame_id, const gm::Quaternion &quat,
 const planning_models::KinematicState &robot_state) const {
  gm::TransformStamped ts = get_transform(old_frame_id, new_frame_id, 
					  robot_state);
  return transform_quaternion(quat, ts.transform);
}

gm::Vector3 darrt::StateTransformer::transform
(std::string new_frame_id, std::string old_frame_id, const gm::Vector3 &vector,
 const planning_models::KinematicState &robot_state) const {
  gm::TransformStamped ts = get_transform(old_frame_id, new_frame_id, robot_state);
  return transform_vector(vector, ts.transform);
}

gm::Pose darrt::StateTransformer::transform
(std::string new_frame_id, std::string old_frame_id, const gm::Pose &pose,
 const planning_models::KinematicState &robot_state) const {
  gm::TransformStamped ts = get_transform(old_frame_id, new_frame_id, 
					  robot_state);
  return transform_pose(pose, ts.transform);
}

gm::PointStamped darrt::StateTransformer::transform
(std::string new_frame_id, const gm::PointStamped &point,
 const planning_models::KinematicState &robot_state) const {
  gm::TransformStamped ts = get_transform(point.header.frame_id, new_frame_id, 
					  robot_state);
  gm::PointStamped newpt;
  newpt.header.frame_id = new_frame_id;
  newpt.point = transform_point(point.point, ts.transform);
  return newpt;
}

gm::QuaternionStamped darrt::StateTransformer::transform
(std::string new_frame_id, const gm::QuaternionStamped &quat,
 const planning_models::KinematicState &robot_state) const {
  gm::TransformStamped ts = get_transform(quat.header.frame_id, new_frame_id, 
					  robot_state);
  gm::QuaternionStamped newquat;
  newquat.header.frame_id = new_frame_id;
  newquat.quaternion = transform_quaternion(quat.quaternion, ts.transform);
  return newquat;
}

gm::Vector3Stamped darrt::StateTransformer::transform
(std::string new_frame_id, const gm::Vector3Stamped &vs,
 const planning_models::KinematicState &robot_state) const {
  gm::TransformStamped ts = get_transform(vs.header.frame_id, new_frame_id, 
					  robot_state);
  gm::Vector3Stamped newvs;
  newvs.header.frame_id = new_frame_id;
  newvs.vector = transform_vector(vs.vector, ts.transform);
  return newvs;
}


gm::PoseStamped darrt::StateTransformer::transform
(std::string new_frame_id, const gm::PoseStamped &pose,
 const planning_models::KinematicState &robot_state) const {
  gm::TransformStamped ts = get_transform(pose.header.frame_id, new_frame_id, 
					  robot_state);
  gm::PoseStamped newps;
  newps.header.frame_id = new_frame_id;
  newps.pose = transform_pose(pose.pose, ts.transform);
  return newps;
}

