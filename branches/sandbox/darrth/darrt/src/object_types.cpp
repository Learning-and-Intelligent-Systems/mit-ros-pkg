#include "darrt/object_types.hh"
#include "darrt/pr2_arm_primitives.hh" //for gripper length ewwwww
#include "darrt/transform_ros_types.hh"
#include "darrt/utils.hh"

void darrt::DARRTObject::copyCollisionObject(const an::CollisionObject &co) {
  header = co.header;
  id = co.id;
  padding = co.padding;
  operation = co.operation;
  shapes = co.shapes;
  poses = co.poses;
}

gm::Pose darrt::DARRTObject::sampleStablePose(const SupportSurface *surface) const {
  gm::Point pt = surface->randomPointOnSurface();
  gm::Pose sp;
  sp.position = pt;
  sp.orientation.w = 1.0;
  //adjust the height
  gm::Point tp = object_top_point(*this, sp);
  gm::Point bp = object_bottom_point(*this, sp);
  sp.position.z += (tp.z - bp.z)/2.0;
  return sp;
}

double darrt::DARRTObject::distance(const gm::Pose &p1, const gm::Pose &p2) const {
  return (darrt::distance(p1.position, p2.position) + darrt::distance(p1.orientation, p2.orientation))/2.0;
}

bool darrt::DARRTObject::nearPoses(const gm::Pose &p1, const gm::Pose &p2,
				   double deps, double aeps) const {
  return darrt::distance(p1.position, p2.position) <= deps &&
    darrt::distance(p1.orientation, p2.orientation) <= aeps;
}

bool darrt::DARRTObject::canSupport(const SupportSurface *surface) const {
  for (unsigned int i = 0; i < support_surfaces_.size(); i++) {
    if (support_surfaces_[i] == surface->name()) {
      if (debug_level >= DRRT) {
	ROS_INFO("%s can support %s", surface->name().c_str(), id.c_str());
      }
      return true;
    }
  }
  if (debug_level >= DRRT) {
    ROS_INFO("%s cannot support %s", surface->name().c_str(), id.c_str());
  }
  return false;
}

double darrt::RoundObject::distance(const gm::Pose &p1, const gm::Pose &p2) const {
  return darrt::distance(p1.position, p2.position);
}

bool darrt::RoundObject::nearPoses(const gm::Pose &p1, const gm::Pose &p2,
				   double deps, double aeps) const {
  //allows arbitrary rotation in z
  //for now assume that's everything
  return darrt::distance(p1.position, p2.position) <= deps;
}

darrt::SpatulaObject::SpatulaObject(std::string spatid,
				    std::string frame_id,
				    const std::vector<std::string> &support_surfaces,
				    const std::vector<double> &paddle_dimensions,
				    const std::vector<double> &handle_dimensions,
				    double angle, gm::Pose starting_pose) : 
  DARRTObject(support_surfaces) {
  angle_ = angle;
  starting_pose_ = starting_pose;
  header.frame_id = frame_id;
  header.stamp = ros::Time(0);
  id = spatid;
  operation.operation = an::CollisionObjectOperation::ADD;

  //we have two shapes joined together
  //a box and a handle
  paddle_.type = an::Shape::BOX;
  paddle_.dimensions = paddle_dimensions;
  if (paddle_.dimensions.size() != 3) {
    pause("Wrong number of dimensions for paddle in spatula!", -1, true);
  }

  handle_.type = an::Shape::CYLINDER;
  handle_.dimensions = handle_dimensions;
  if (handle_.dimensions.size() != 2) {
    pause("Wrong number of dimensions for handle in spatula!", -1, true);
  }

  shapes.resize(2);
  shapes[0] = paddle_;
  shapes[1] = handle_;

  //we put the "origin" in the center
  poses.resize(2);
  poses[0].position.y = paddle_.dimensions[1]/2.0;
  poses[0].orientation.w = 1;

  double total_angle = darrt::MATH_PI/2.0-angle_;
  poses[1].position.y = -1.0*handle_.dimensions[1]/2.0*sin(total_angle);
  poses[1].position.z = handle_.dimensions[1]/2.0*cos(total_angle);
  poses[1].orientation.x = sin(total_angle/2.0);
  poses[1].orientation.w = cos(total_angle/2.0);
}

void darrt::SpatulaObject::getGrasps(std::vector<gm::Transform> &grasps) const {

  double pos_on_handle = handle_.dimensions[1] - 0.1;
  gm::Transform inv_grasp;
  inv_grasp.translation.y = GRIPPER_LENGTH;
  inv_grasp.translation.z = pos_on_handle/2.0;
  inv_grasp.rotation.z = sin(-1.0*MATH_PI/4.0);
  inv_grasp.rotation.w = cos(-1.0*MATH_PI/4.0);
  gm::Transform grasp;
  grasp = pose_to_transform(transform_pose(transform_to_pose(inv_grasp), poses[1]));
  gm::Pose origin;
  origin.orientation.w = 1.0;
  grasp = pose_to_transform(inverse_transform_pose(origin, grasp));
  grasps.push_back(grasp);
}

gm::Pose darrt::SpatulaObject::sampleStablePose(const SupportSurface *surface) const {
  if (debug_level >= DRRT) {
    ROS_ERROR("Returning spatula starting point because it can't move.  THIS IS WRONG");
    ROS_INFO_STREAM("Returning\n" << starting_pose_);
  }
  return starting_pose_;
}
