#include "base_trajectory_action/transform_ros_types.hh"

double base_trajectory_action::wrap_angle(double a) {
  while (a < -1.0*MATH_PI) {
    a += 2.0*MATH_PI;
  }
  while (a > MATH_PI) {
    a -= 2.0*MATH_PI;
  }
  return a;
}

double base_trajectory_action::wrap_angle_positive(double a) {
  while (a < 0) {
    a += 2.0*MATH_PI;
  }
  while (a > 2.0*MATH_PI) {
    a -= 2.0*MATH_PI;
  }
  return a;
}

double base_trajectory_action::angular_distance(double a1, double a2) {
  return wrap_angle(a1 - a2);
}

void base_trajectory_action::distance(const geometry_msgs::Pose2D &p1, const geometry_msgs::Pose2D &p2,
				      double &linear, double &angular) {
  linear = sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
  angular = angular_distance(p1.theta, p2.theta);
}


void base_trajectory_action::multiply(const geometry_msgs::Quaternion &q1,
		     const geometry_msgs::Quaternion &q2,
		     geometry_msgs::Quaternion &q) {
  q.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
  q.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
  q.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
  q.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
}

void base_trajectory_action::conjugate(const geometry_msgs::Quaternion &q_in,
				       geometry_msgs::Quaternion &q_out) {
  q_out.x = q_in.x;
  q_out.y = q_in.y;
  q_out.z = q_in.z;
  q_out.w = -1.0*q_in.w;
}


geometry_msgs::Point base_trajectory_action::transform_point
(const geometry_msgs::Point &pt, const geometry_msgs::Pose &transform) {
  return transform_point(pt, pose_to_transform(transform));
}

geometry_msgs::Point base_trajectory_action::transform_point
(const geometry_msgs::Point &pt, const geometry_msgs::Transform &transform) {
  const geometry_msgs::Quaternion &quat = transform.rotation;
  const geometry_msgs::Vector3 &trans = transform.translation;
  //create the rotation matrix (or i can't keep this all straight
  double row00 = 1.0 - 2*quat.y*quat.y - 2*quat.z*quat.z;
  double row01 = 2*quat.x*quat.y - 2*quat.z*quat.w;
  double row02 = 2*quat.x*quat.z + 2*quat.y*quat.w;

  double row10 = 2*quat.x*quat.y + 2*quat.z*quat.w;
  double row11 = 1 - 2*quat.x*quat.x - 2*quat.z*quat.z;
  double row12 = 2*quat.y*quat.z - 2*quat.x*quat.w;

  double row20 = 2*quat.x*quat.z - 2*quat.y*quat.w;
  double row21 = 2*quat.y*quat.z + 2*quat.x*quat.w;
  double row22 = 1 - 2*quat.x*quat.x - 2*quat.y*quat.y;
  
  geometry_msgs::Point tp;
  tp.x = row00*pt.x + row01*pt.y + row02*pt.z + trans.x;
  tp.y = row10*pt.x + row11*pt.y + row12*pt.z + trans.y;
  tp.z = row20*pt.x + row21*pt.y + row22*pt.z + trans.z;
  
  return tp;
}

geometry_msgs::Point base_trajectory_action::inverse_transform_point
(const geometry_msgs::Point &pt, const geometry_msgs::Pose &transform) {
  return inverse_transform_point(pt, pose_to_transform(transform));
}

geometry_msgs::Point base_trajectory_action::inverse_transform_point
(const geometry_msgs::Point &pt, const geometry_msgs::Transform &transform) {
  const geometry_msgs::Quaternion &quat = transform.rotation;
  const geometry_msgs::Vector3 &trans = transform.translation;
  //note: this is called often enough that we don't want to
  //create vectors

  double row00 = 1.0 - 2*quat.y*quat.y - 2*quat.z*quat.z;
  double row01 = 2*quat.x*quat.y - 2*quat.z*quat.w;
  double row02 = 2*quat.x*quat.z + 2*quat.y*quat.w;

  double row10 = 2*quat.x*quat.y + 2*quat.z*quat.w;
  double row11 = 1 - 2*quat.x*quat.x - 2*quat.z*quat.z;
  double row12 = 2*quat.y*quat.z - 2*quat.x*quat.w;

  double row20 = 2*quat.x*quat.z - 2*quat.y*quat.w;
  double row21 = 2*quat.y*quat.z + 2*quat.x*quat.w;
  double row22 = 1 - 2*quat.x*quat.x - 2*quat.y*quat.y;

  geometry_msgs::Point tp;
  tp.x = row00*(pt.x - trans.x) + row10*(pt.y - trans.y)
    + row20*(pt.z - trans.z);
  tp.y = row01*(pt.x - trans.x) + row11*(pt.y - trans.y)
    + row21*(pt.z - trans.z);
  tp.z = row02*(pt.x - trans.x) + row12*(pt.y - trans.y)
    + row22*(pt.z - trans.z);
  return tp;
}

geometry_msgs::Quaternion base_trajectory_action::transform_quaternion
(const geometry_msgs::Quaternion &quat, const geometry_msgs::Pose &transform) {
  return transform_quaternion(quat, pose_to_transform(transform));
}

geometry_msgs::Quaternion base_trajectory_action::transform_quaternion
(const geometry_msgs::Quaternion &quat, 
 const geometry_msgs::Transform &transform) {
  geometry_msgs::Quaternion ret;
  multiply(transform.rotation, quat, ret);
  return ret;
}

geometry_msgs::Quaternion base_trajectory_action::inverse_transform_quaternion
(const geometry_msgs::Quaternion &quat, const geometry_msgs::Pose &transform) {
  return inverse_transform_quaternion(quat, pose_to_transform(transform));
}

geometry_msgs::Quaternion base_trajectory_action::inverse_transform_quaternion
(const geometry_msgs::Quaternion &quat, 
 const geometry_msgs::Transform &transform) {
  geometry_msgs::Quaternion conj, ret;
  conjugate(transform.rotation, conj);
  multiply(conj, quat, ret);
  return ret;
}

geometry_msgs::Vector3 base_trajectory_action::transform_vector
(const geometry_msgs::Vector3 &vector, const geometry_msgs::Pose &transform) {
  return transform_vector(vector, pose_to_transform(transform));
}

geometry_msgs::Vector3 base_trajectory_action::transform_vector
(const geometry_msgs::Vector3 &vector, const geometry_msgs::Transform &transform) {
  geometry_msgs::Point p;
  p.x = vector.x;
  p.y = vector.y;  
  p.z = vector.z;
  geometry_msgs::Transform t = transform;
  t.translation.x = 0.0;
  t.translation.y = 0.0;  
  t.translation.z = 0.0;
  geometry_msgs::Point tp = transform_point(p, t);
  geometry_msgs::Vector3 v;
  v.x = tp.x;
  v.y = tp.y;
  v.z = tp.z;
  return v;
}


geometry_msgs::Vector3 base_trajectory_action::inverse_transform_vector
(const geometry_msgs::Vector3 &vector, const geometry_msgs::Pose &transform) {
  return inverse_transform_vector(vector, pose_to_transform(transform));
}

geometry_msgs::Vector3 base_trajectory_action::inverse_transform_vector
(const geometry_msgs::Vector3 &vector, const geometry_msgs::Transform &transform) {
  geometry_msgs::Point p;
  p.x = vector.x;
  p.y = vector.y;  
  p.z = vector.z;
  geometry_msgs::Transform t = transform;
  t.translation.x = 0.0;
  t.translation.y = 0.0;  
  t.translation.z = 0.0;
  geometry_msgs::Point tp = inverse_transform_point(p, t);
  geometry_msgs::Vector3 v;
  v.x = tp.x;
  v.y = tp.y;
  v.z = tp.z;
  return v;
}


geometry_msgs::Pose base_trajectory_action::transform_pose
(const geometry_msgs::Pose &pose, const geometry_msgs::Pose &transform) {
  return transform_pose(pose, pose_to_transform(transform));
}

geometry_msgs::Pose base_trajectory_action::transform_pose
(const geometry_msgs::Pose &pose, const geometry_msgs::Transform &transform) {
  geometry_msgs::Pose p;
  p.position = transform_point(pose.position, transform);
  p.orientation = transform_quaternion(pose.orientation, transform);
  return p;
}

geometry_msgs::Pose base_trajectory_action::inverse_transform_pose
(const geometry_msgs::Pose &pose, const geometry_msgs::Pose &transform) {
  return inverse_transform_pose(pose, pose_to_transform(transform));
}

geometry_msgs::Pose base_trajectory_action::inverse_transform_pose
(const geometry_msgs::Pose &pose, const geometry_msgs::Transform &transform) {
  geometry_msgs::Pose p;
  p.position = inverse_transform_point(pose.position, transform);
  p.orientation = inverse_transform_quaternion(pose.orientation, transform);
  return p;
}

geometry_msgs::Transform base_trajectory_action::pose_to_transform
(const geometry_msgs::Pose &pose) {
  geometry_msgs::Transform trans;
  trans.translation.x = pose.position.x;
  trans.translation.y = pose.position.y;
  trans.translation.z = pose.position.z;

  trans.rotation.x = pose.orientation.x;
  trans.rotation.y = pose.orientation.y;  
  trans.rotation.z = pose.orientation.z;
  trans.rotation.w = pose.orientation.w;
  return trans;
}

geometry_msgs::Pose base_trajectory_action::transform_to_pose
(const geometry_msgs::Transform &trans) {
  geometry_msgs::Pose pose;
  
  pose.position.x = trans.translation.x;
  pose.position.y = trans.translation.y;
  pose.position.z = trans.translation.z;

  pose.orientation.x = trans.rotation.x;
  pose.orientation.y = trans.rotation.y;
  pose.orientation.z = trans.rotation.z;
  pose.orientation.w = trans.rotation.w;
  return pose;
}

double base_trajectory_action::quaternion_to_phi(const geometry_msgs::Quaternion &q) {
  return atan2(2.0*(q.w*q.x + q.y*q.z), 1.0 - 2.0*(q.x*q.x + q.y*q.y));
} 

double base_trajectory_action::quaternion_to_theta(const geometry_msgs::Quaternion &q) {
  return asin(2.0*(q.w*q.y - q.z*q.x));
} 

//these are the same thing
double base_trajectory_action::quaternion_to_psi(const geometry_msgs::Quaternion &q) {
  return atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
} 

double base_trajectory_action::quaternion_to_yaw(const geometry_msgs::Quaternion &q) {
  return quaternion_to_psi(q);
} 



void base_trajectory_action::yaw_to_quaternion(double a, 
			      geometry_msgs::Quaternion &q) {
  q.x = 0.0;
  q.y = 0.0;
  q.z = sin(a/2.0);
  q.w = cos(a/2.0);
}


geometry_msgs::Pose2D base_trajectory_action::pose_to_pose2D
(const geometry_msgs::Pose &pose) {
  geometry_msgs::Pose2D p;
  p.x = pose.position.x;
  p.y = pose.position.y;
  p.theta = quaternion_to_yaw(pose.orientation);
  return p;
}



geometry_msgs::Pose base_trajectory_action::pose2D_to_pose
(const geometry_msgs::Pose2D &pose) {
  geometry_msgs::Pose p;
  p.position.x = pose.x;
  p.position.y = pose.y;
  p.orientation.z = sin(pose.theta/2.0);
  p.orientation.w = cos(pose.theta/2.0);
  return p;
}

geometry_msgs::Pose base_trajectory_action::pose2D_to_ground_pose
(const geometry_msgs::Pose2D &pose) {
  geometry_msgs::Pose p;
  p.position.x = pose.x;
  p.position.y = pose.y;
  p.position.z = 0.0;
  p.orientation.x = 0.0;
  p.orientation.y = 0.0;
  p.orientation.z = sin(pose.theta/2.0);
  p.orientation.w = cos(pose.theta/2.0);
  return p;
}

geometry_msgs::Point base_trajectory_action::vector_to_point(const geometry_msgs::Vector3 &v) {
  geometry_msgs::Point p;
  p.x = v.x;
  p.y = v.y;
  p.z = v.z;
  return p;
}

geometry_msgs::Vector3 base_trajectory_action::cross(const geometry_msgs::Vector3 &u, const geometry_msgs::Vector3 &v) {
  geometry_msgs::Vector3 c;
  c.x = u.y*v.z - u.z*v.y;
  c.y = u.z*v.x - u.x*v.z;
  c.z = u.x*v.y - u.y*v.x;
  return c;
}

double base_trajectory_action::dot(const geometry_msgs::Vector3 &u, const geometry_msgs::Vector3 &v) {
  return u.x*v.x + u.y*v.y + u.z*v.z;
}

double base_trajectory_action::norm(const geometry_msgs::Vector3 &v) {
  return sqrt(dot(v, v));
}

geometry_msgs::Vector3 base_trajectory_action::subtract(const geometry_msgs::Vector3 &u, const geometry_msgs::Vector3 &v) {
  geometry_msgs::Vector3 s;
  s.x = u.x - v.x;
  s.y = u.y - v.y;
  s.z = u.z - v.z;
  return s;
}
