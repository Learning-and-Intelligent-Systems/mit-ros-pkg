#ifndef __BTA_TRANSFORM_ROS_TYPES_H__
#define __BTA_TRANSFORM_ROS_TYPES_H__

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <arm_navigation_msgs/CollisionObject.h>

namespace base_trajectory_action {

  const double MATH_PI = 3.14159265;

  double wrap_angle(double a);
  double wrap_angle_positive(double a);
  double angular_distance(double a1, double a2);
  void distance(const geometry_msgs::Pose2D &p1, const geometry_msgs::Pose2D &p2,
		double &linear, double &angular);

  void multiply(const geometry_msgs::Quaternion &q1,
		const geometry_msgs::Quaternion &q2,
		geometry_msgs::Quaternion &q);
  void conjugate(const geometry_msgs::Quaternion &q_in,
		 geometry_msgs::Quaternion &q_out);

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

  double quaternion_to_phi(const geometry_msgs::Quaternion &q);
  double quaternion_to_theta(const geometry_msgs::Quaternion &q);
  double quaternion_to_psi(const geometry_msgs::Quaternion &q);
  double quaternion_to_yaw(const geometry_msgs::Quaternion &q);
  void yaw_to_quaternion(double a, geometry_msgs::Quaternion &q);

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

}

#endif //transform_ros_types.hh
