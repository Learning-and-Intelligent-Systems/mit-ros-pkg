/*
 * util.hpp
 *
 *  Created on: Feb 13, 2011
 *      Author: hordurj
 */

#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>

extern void Transform2Iso(const tf::Transform & transform, Eigen::Isometry3d & iso);
extern double rad_2_deg(const double & a);
extern void convert_to_roll_pitch_yaw(const Eigen::Isometry3d & x, double* rpy);
extern int64_t time_to_utime(const ros::Time & t);

#endif /* UTIL_HPP_ */
