/*
 * visualization.cpp
 *
 *  Created on: Feb 13, 2011
 *      Author: hordurj
 */

#include "imapping/visualization.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <bot_core/rotations.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include "mrlcm_collection_config_t.h"
#include "mrlcm_color_t.h"
#include "mrlcm_cov_collection_t.h"
#include "mrlcm_cov_t.h"
#include "mrlcm_link_collection_t.h"
#include "mrlcm_link_t.h"
#include "mrlcm_obj_collection_t.h"
#include "mrlcm_obj_t.h"
#include "mrlcm_point3d_list_collection_t.h"
#include "mrlcm_point3d_list_t.h"
#include "mrlcm_point3d_t.h"
#include "mrlcm_points_collection_t.h"
#include "mrlcm_points_t.h"
#include "mrlcm_property_t.h"
#include "mrlcm_reset_collections_t.h"

// colormap for disparities, RGB
static unsigned char colormap[768] =
  { 150, 150, 150,
    107, 0, 12,
    106, 0, 18,
    105, 0, 24,
    103, 0, 30,
    102, 0, 36,
    101, 0, 42,
    99, 0, 48,
    98, 0, 54,
    97, 0, 60,
    96, 0, 66,
    94, 0, 72,
    93, 0, 78,
    92, 0, 84,
    91, 0, 90,
    89, 0, 96,
    88, 0, 102,
    87, 0, 108,
    85, 0, 114,
    84, 0, 120,
    83, 0, 126,
    82, 0, 131,
    80, 0, 137,
    79, 0, 143,
    78, 0, 149,
    77, 0, 155,
    75, 0, 161,
    74, 0, 167,
    73, 0, 173,
    71, 0, 179,
    70, 0, 185,
    69, 0, 191,
    68, 0, 197,
    66, 0, 203,
    65, 0, 209,
    64, 0, 215,
    62, 0, 221,
    61, 0, 227,
    60, 0, 233,
    59, 0, 239,
    57, 0, 245,
    56, 0, 251,
    55, 0, 255,
    54, 0, 255,
    52, 0, 255,
    51, 0, 255,
    50, 0, 255,
    48, 0, 255,
    47, 0, 255,
    46, 0, 255,
    45, 0, 255,
    43, 0, 255,
    42, 0, 255,
    41, 0, 255,
    40, 0, 255,
    38, 0, 255,
    37, 0, 255,
    36, 0, 255,
    34, 0, 255,
    33, 0, 255,
    32, 0, 255,
    31, 0, 255,
    29, 0, 255,
    28, 0, 255,
    27, 0, 255,
    26, 0, 255,
    24, 0, 255,
    23, 0, 255,
    22, 0, 255,
    20, 0, 255,
    19, 0, 255,
    18, 0, 255,
    17, 0, 255,
    15, 0, 255,
    14, 0, 255,
    13, 0, 255,
    11, 0, 255,
    10, 0, 255,
    9, 0, 255,
    8, 0, 255,
    6, 0, 255,
    5, 0, 255,
    4, 0, 255,
    3, 0, 255,
    1, 0, 255,
    0, 4, 255,
    0, 10, 255,
    0, 16, 255,
    0, 22, 255,
    0, 28, 255,
    0, 34, 255,
    0, 40, 255,
    0, 46, 255,
    0, 52, 255,
    0, 58, 255,
    0, 64, 255,
    0, 70, 255,
    0, 76, 255,
    0, 82, 255,
    0, 88, 255,
    0, 94, 255,
    0, 100, 255,
    0, 106, 255,
    0, 112, 255,
    0, 118, 255,
    0, 124, 255,
    0, 129, 255,
    0, 135, 255,
    0, 141, 255,
    0, 147, 255,
    0, 153, 255,
    0, 159, 255,
    0, 165, 255,
    0, 171, 255,
    0, 177, 255,
    0, 183, 255,
    0, 189, 255,
    0, 195, 255,
    0, 201, 255,
    0, 207, 255,
    0, 213, 255,
    0, 219, 255,
    0, 225, 255,
    0, 231, 255,
    0, 237, 255,
    0, 243, 255,
    0, 249, 255,
    0, 255, 255,
    0, 255, 249,
    0, 255, 243,
    0, 255, 237,
    0, 255, 231,
    0, 255, 225,
    0, 255, 219,
    0, 255, 213,
    0, 255, 207,
    0, 255, 201,
    0, 255, 195,
    0, 255, 189,
    0, 255, 183,
    0, 255, 177,
    0, 255, 171,
    0, 255, 165,
    0, 255, 159,
    0, 255, 153,
    0, 255, 147,
    0, 255, 141,
    0, 255, 135,
    0, 255, 129,
    0, 255, 124,
    0, 255, 118,
    0, 255, 112,
    0, 255, 106,
    0, 255, 100,
    0, 255, 94,
    0, 255, 88,
    0, 255, 82,
    0, 255, 76,
    0, 255, 70,
    0, 255, 64,
    0, 255, 58,
    0, 255, 52,
    0, 255, 46,
    0, 255, 40,
    0, 255, 34,
    0, 255, 28,
    0, 255, 22,
    0, 255, 16,
    0, 255, 10,
    0, 255, 4,
    2, 255, 0,
    8, 255, 0,
    14, 255, 0,
    20, 255, 0,
    26, 255, 0,
    32, 255, 0,
    38, 255, 0,
    44, 255, 0,
    50, 255, 0,
    56, 255, 0,
    62, 255, 0,
    68, 255, 0,
    74, 255, 0,
    80, 255, 0,
    86, 255, 0,
    92, 255, 0,
    98, 255, 0,
    104, 255, 0,
    110, 255, 0,
    116, 255, 0,
    122, 255, 0,
    128, 255, 0,
    133, 255, 0,
    139, 255, 0,
    145, 255, 0,
    151, 255, 0,
    157, 255, 0,
    163, 255, 0,
    169, 255, 0,
    175, 255, 0,
    181, 255, 0,
    187, 255, 0,
    193, 255, 0,
    199, 255, 0,
    205, 255, 0,
    211, 255, 0,
    217, 255, 0,
    223, 255, 0,
    229, 255, 0,
    235, 255, 0,
    241, 255, 0,
    247, 255, 0,
    253, 255, 0,
    255, 251, 0,
    255, 245, 0,
    255, 239, 0,
    255, 233, 0,
    255, 227, 0,
    255, 221, 0,
    255, 215, 0,
    255, 209, 0,
    255, 203, 0,
    255, 197, 0,
    255, 191, 0,
    255, 185, 0,
    255, 179, 0,
    255, 173, 0,
    255, 167, 0,
    255, 161, 0,
    255, 155, 0,
    255, 149, 0,
    255, 143, 0,
    255, 137, 0,
    255, 131, 0,
    255, 126, 0,
    255, 120, 0,
    255, 114, 0,
    255, 108, 0,
    255, 102, 0,
    255, 96, 0,
    255, 90, 0,
    255, 84, 0,
    255, 78, 0,
    255, 72, 0,
    255, 66, 0,
    255, 60, 0,
    255, 54, 0,
    255, 48, 0,
    255, 42, 0,
    255, 36, 0,
    255, 30, 0,
    255, 24, 0,
    255, 18, 0,
    255, 12, 0,
    255,  6, 0,
    255,  0, 0,
  };

namespace visual_odometry
{
  void displayImages(int width, int height, frame_common::DenseStereo & disp)
  {
    cv::Mat_<cv::Vec3b> disparity_color_;
    disparity_color_.create(height, width);

    // Colormap and display the disparity image
    float min_disparity = 0;
    float max_disparity = 256;
    float multiplier = 255.0f / (max_disparity - min_disparity);

    for (int row = 0; row < disparity_color_.rows; ++row) {
      for (int col = 0; col < disparity_color_.cols; ++col) {
        int index = (disp.lookup_disparity(col,row) - min_disparity) * multiplier + 0.5;
        index = std::min(255, std::max(0, index));

        // Fill as BGR
        disparity_color_(row, col)[2] = colormap[3*index + 0];
        disparity_color_(row, col)[1] = colormap[3*index + 1];
        disparity_color_(row, col)[0] = colormap[3*index + 2];
      }
    }

    cv::imshow("disparity", disparity_color_);
  }

  void resetPose(lcm_t* lcmref)
  {
    mrlcm_obj_collection_t objs;
    size_t n = 0;
    objs.id = 1;
    objs.name = (char*) "Camera";
    objs.type = MRLCM_OBJ_COLLECTION_T_AXIS3D;
    objs.reset = false;
    objs.nobjs = 0;
    objs.objs = 0;
    mrlcm_obj_collection_t_publish(lcmref, "OBJ_COLLECTION", &objs);
  }

  void sendPose(lcm_t* lcmref, uint64_t utime, const Eigen::Isometry3d & cam_to_local)
  {
    static bool first = false;

    Eigen::Vector3d translation(cam_to_local.translation());
    Eigen::Quaterniond rotation(cam_to_local.rotation());

    double rpy[3];
    double q[4];

    q[0] = rotation.w();
    q[1] = rotation.x();
    q[2] = rotation.y();
    q[3] = rotation.z();

    bot_quat_to_roll_pitch_yaw (q, rpy);

    bot_core_pose_t pose_msg;
    memset(&pose_msg, 0, sizeof(pose_msg));
    pose_msg.utime = utime; // msg->timestamp;
    pose_msg.pos[0] = translation[0];
    pose_msg.pos[1] = translation[1];
    pose_msg.pos[2] = translation[2];
    pose_msg.orientation[0] = rotation.w();
    pose_msg.orientation[1] = rotation.x();
    pose_msg.orientation[2] = rotation.y();
    pose_msg.orientation[3] = rotation.z();
    bot_core_pose_t_publish(lcmref, "POSE", &pose_msg);

    mrlcm_obj_collection_t objs;
    size_t n = 1;
    objs.id = 1; // collection.id();
    objs.name = (char*) "Camera"; // collection.name().c_str();
    objs.type = MRLCM_OBJ_COLLECTION_T_AXIS3D; // MRLCM_OBJ_COLLECTION_T_POSE;
    objs.reset = false;
    objs.nobjs = 1; //n;
    mrlcm_obj_t poses[n];
  //              for (size_t i = 0; i < n; i++) {
                          //const isam::Pose3d & pose = collection(i).pose;
    poses[0].id = utime; // collection(i).utime;
    poses[0].x = cam_to_local.translation()[0] ;//pose.x();
    poses[0].y = cam_to_local.translation()[1];//pose.y();
    poses[0].z = cam_to_local.translation()[2];//pose.z();
    poses[0].yaw = rpy[2]; // pose.yaw();
    poses[0].pitch = rpy[1]; //pose.pitch();
    poses[0].roll = rpy[0]; //pose.roll();
  //              }
    objs.objs = poses;
    mrlcm_obj_collection_t_publish(lcmref, "OBJ_COLLECTION", &objs);
  }
}

