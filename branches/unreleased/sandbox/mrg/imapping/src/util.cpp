/*
 * util.cpp
 *
 *  Created on: Feb 13, 2011
 *      Author: hordurj
 */

#include <bot_core/rotations.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>

/***
 * Convert a Transform to an Isometry
 */
void Transform2Iso(const tf::Transform & transform, Eigen::Isometry3d & iso)
{
  btVector3 o = transform.getOrigin();
  btQuaternion q = transform.getRotation();
  iso.setIdentity();
  iso.translate(Eigen::Vector3d(o.x(),o.y(),o.z()));
  iso.rotate(Eigen::Quaterniond(q.w(),q.x(),q.y(),q.z()));
}

double rad_2_deg(const double & a)
{
  return a/M_PI*180.0;
}

void convert_to_roll_pitch_yaw(const Eigen::Isometry3d & x, double* rpy)
{
  Eigen::Quaterniond rotation(x.rotation());
  double q[4];
  q[0] = rotation.w();
  q[1] = rotation.x();
  q[2] = rotation.y();
  q[3] = rotation.z();
  bot_quat_to_roll_pitch_yaw (q, rpy);
}

int64_t time_to_utime(const ros::Time & t)
{
  return t.sec * 1E6 + t.nsec/1E3;
}
