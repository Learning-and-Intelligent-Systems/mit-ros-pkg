/*
 * visualization.hpp
 *
 *  Created on: Feb 13, 2011
 *      Author: hordurj
 */

#ifndef VISUALIZATION_HPP_
#define VISUALIZATION_HPP_

#include <lcm/lcm.h>
#include <bot/core/lcmtypes/lcmtypes.h>
#include <lcmtypes/bot_core.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "frame_common/stereo.h"

namespace visual_odometry
{
  extern void displayImages(int width, int height, frame_common::DenseStereo & disp);
  extern void resetPose(lcm_t* lcmref);
  extern void sendPose(lcm_t* lcmref, uint64_t utime, const Eigen::Isometry3d & cam_to_local);
}

#endif /* VISUALIZATION_HPP_ */
