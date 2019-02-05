#ifndef __DARRT_TRANSFORM_ROS_TYPES_H__
#define __DARRT_TRANSFORM_ROS_TYPES_H__

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <arm_navigation_msgs/CollisionObject.h>

namespace darrt {

  geometry_msgs::Point transform_point(const geometry_msgs::Point &p, 
				       const geometry_msgs::Pose &transform);
  geometry_msgs::Point transform_point
  (const geometry_msgs::Point &p, const geometry_msgs::Transform &transform);
  geometry_msgs::Point inverse_transform_point
  (const geometry_msgs::Point &p, const geometry_msgs::Pose &transform);
  geometry_msgs::Point inverse_transform_point
  (const geometry_msgs::Point &p, const geometry_msgs::Transform &transform);
  geometry_msgs::Quaternion transform_quaternion
  (const geometry_msgs::Quaternion &quat, 
   const geometry_msgs::Pose &transform);
  geometry_msgs::Quaternion transform_quaternion
  (const geometry_msgs::Quaternion &quat, 
   const geometry_msgs::Transform &transform);
  geometry_msgs::Quaternion inverse_transform_quaternion
  (const geometry_msgs::Quaternion &quat, 
   const geometry_msgs::Pose &transform);
  geometry_msgs::Quaternion inverse_transform_quaternion
  (const geometry_msgs::Quaternion &quat, 
   const geometry_msgs::Transform &transform);
  geometry_msgs::Vector3 transform_vector(const geometry_msgs::Vector3 &p, 
				       const geometry_msgs::Pose &transform);
  geometry_msgs::Vector3 transform_vector
  (const geometry_msgs::Vector3 &p, const geometry_msgs::Transform &transform);
  geometry_msgs::Vector3 inverse_transform_vector
  (const geometry_msgs::Vector3 &p, const geometry_msgs::Pose &transform);
  geometry_msgs::Vector3 inverse_transform_vector
  (const geometry_msgs::Vector3 &p, const geometry_msgs::Transform &transform);
  geometry_msgs::Pose transform_pose
  (const geometry_msgs::Pose &pose, const geometry_msgs::Pose &transform);
  geometry_msgs::Pose transform_pose
  (const geometry_msgs::Pose &pose, 
   const geometry_msgs::Transform &transform);
  geometry_msgs::Pose inverse_transform_pose
  (const geometry_msgs::Pose &pose, const geometry_msgs::Pose &transform);
  geometry_msgs::Pose inverse_transform_pose
  (const geometry_msgs::Pose &pose, 
   const geometry_msgs::Transform &transform);

  geometry_msgs::Transform pose_to_transform
  (const geometry_msgs::Pose &pose);
  geometry_msgs::Pose transform_to_pose
  (const geometry_msgs::Transform &trans);
  geometry_msgs::Pose2D pose_to_pose2D
  (const geometry_msgs::Pose &pose);
  geometry_msgs::Pose pose2D_to_pose
  (const geometry_msgs::Pose2D &pose);
  geometry_msgs::Pose pose2D_to_ground_pose
  (const geometry_msgs::Pose2D &pose);
  geometry_msgs::Vector3 point_to_vector(const geometry_msgs::Point &p);
  geometry_msgs::Point vector_to_point(const geometry_msgs::Vector3 &v);

  geometry_msgs::Vector3 cross(const geometry_msgs::Vector3 &u, const geometry_msgs::Vector3 &v);
  double dot(const geometry_msgs::Vector3 &u, const geometry_msgs::Vector3 &v);
  double norm(const geometry_msgs::Vector3 &v);
  geometry_msgs::Vector3 subtract(const geometry_msgs::Vector3 &u, const geometry_msgs::Vector3 &v);

  std::vector<geometry_msgs::Point> corners(const arm_navigation_msgs::Shape &co);
  geometry_msgs::Point object_bottom_point(const arm_navigation_msgs::CollisionObject &obj,
					   const geometry_msgs::Pose &pose);
  geometry_msgs::Point object_top_point(const arm_navigation_msgs::CollisionObject &obj,
					const geometry_msgs::Pose &pose);

  class Line {
  public:
    geometry_msgs::Point p1, p2;
    double x_min, x_max, y_min, y_max, slope, y_int;
    bool vertical;
    Line(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2);
    bool contains(const geometry_msgs::Point &p) const;
    bool intersection(const Line &l, geometry_msgs::Point &p) const;
    double length() const;
    geometry_msgs::Point nearest_point(const geometry_msgs::Point &p) const;
  };
}

#endif //transform_ros_types.hh
