/*
 * util.hpp
 *
 *  Created on: Feb 13, 2011
 *      Author: hordurj
 */

#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <isam/isam.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>

void Transform2Iso(const tf::Transform & transform, Eigen::Isometry3d & iso);
double rad_2_deg(const double & a);
void convert_to_roll_pitch_yaw(const Eigen::Isometry3d & x, double* rpy);
int64_t time_to_utime(const ros::Time & t);
void odometry_to_pose3d(const nav_msgs::Odometry::ConstPtr& odom, isam::Pose3d & pose);

#endif /* UTIL_HPP_ */
