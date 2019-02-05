/*
 * vo.cpp
 *
 *  Created on: Feb 10, 2011
 *      Author: hordurj
 *
 *  This program tests the librgdb visual odometry package.
 *
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/stereo_camera_model.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include "librgbd-odometry/rgbd_odometry.hpp"
#include "frame_common/stereo.h"
#include "frame_common/camparams.h"
#include <lcm/lcm.h>
#include <bot/core/lcmtypes/lcmtypes.h>
#include <bot_core/rotations.h>
#include <lcmtypes/bot_core.h>
#include "imapping/util.hpp"
#include "imapping/visualization.hpp"
#include "viewer.h"

using namespace Eigen;
using namespace rgbd_odometry;
using namespace visual_odometry;

lcm_t* g_lcm;

class StereoDepth : public rgbd_odometry::DepthSource
{
public:
  StereoDepth();
  ~StereoDepth();

  void setCamParams(const frame_common::CamParams & cam_params) { cam_params_ = cam_params; }
  void setStereo(frame_common::DenseStereo* stereo) { stereo_=stereo; }
  void setDisparityData(const uint16_t* disparity);

  virtual bool haveXyz(int u, int v);
  virtual bool haveXyz(float u, float v);

  virtual Eigen::Vector3d getXyz(int u, int v);
  virtual Eigen::Vector3d getXyz(float u, float v);

  virtual double getBaseline() { return cam_params_.tx; }
private:
  const uint16_t* disparity_;
  frame_common::CamParams cam_params_;
  frame_common::DenseStereo* stereo_;
};

class VONode
{
public:
  VONode();
  void imageCb(const sensor_msgs::ImageConstPtr& l_image,
               const sensor_msgs::CameraInfoConstPtr& l_cam_info,
               const sensor_msgs::ImageConstPtr& r_image,
               const sensor_msgs::CameraInfoConstPtr& r_cam_info);
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  image_transport::SubscriberFilter l_image_sub_, r_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub_, r_info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
  sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;

  rgbd_odometry::RgbdOdometry* odom_;
  rgbd_odometry::CameraIntrinsics calib_;

  StereoDepth* depth_producer_;
  image_geometry::StereoCameraModel cam_model_;
  bool calib_initialized_;

  Eigen::Isometry3d last_pose_;
  Eigen::Isometry3d last_cam_pose_;
  Eigen::Isometry3d last_base_pose_;

  tf::TransformListener* listener_;

  tf::StampedTransform last_base_tf_;

  Viewer viewer_;

  // Object collections
  ObjectCollection col_vo_;
  ObjectCollection col_odo_;
  ObjectCollection col_vo_odo_;

  // Link collections
  LinkCollection link_vo_accept_;
  LinkCollection link_vo_reject_;
  LinkCollection link_odo_accept_;
  LinkCollection link_odo_reject_;
  LinkCollection link_vo_odo_accept_;
  LinkCollection link_vo_odo_reject_;

  Eigen::Isometry3d pose_vo_;
  Eigen::Isometry3d pose_odo_;
  Eigen::Isometry3d pose_vo_odo_;

  int64_t last_utime_;

};


StereoDepth::StereoDepth()
{
}

StereoDepth::~StereoDepth()
{
}

void StereoDepth::setDisparityData(const uint16_t* disparity)
{
}

bool StereoDepth::haveXyz(int u, int v)
{
  double disp = stereo_->lookup_disparity(u,v);
  return disp>0.0;
}

bool StereoDepth::haveXyz(float u, float v)
{
  double disp = stereo_->lookup_disparity(u,v);
  return disp>0.0;
}

Eigen::Vector3d StereoDepth::getXyz(int u, int v)
{
  double disp = stereo_->lookup_disparity(u,v);
  if (disp > 0.0)      // valid point
  {
    double fx = cam_params_.fx;
    double cx = cam_params_.cx;
    double cy = cam_params_.cy;
    double tx = cam_params_.tx;

    // x,y,z
    double w = tx / disp;
    double x = ((double)u - cx)*w;
    double y = ((double)v - cy)*w;
    double z = fx*w;

    return Eigen::Vector3d(x,y,z);
  }
  else
  {
    return Eigen::Vector3d(NAN, NAN, NAN);
  }
}

Eigen::Vector3d StereoDepth::getXyz(float u, float v)
{
  int u_i = (int)u;
  int v_i = (int)v;
  float u_f = u - u_i;
  float v_f = v - v_i;
  float w[4] = { (1-u_f) * (1-v_f),
                 u_f * (1-v_f),
                 (1-u_f) * v_f,
                 u_f * v_f };
  float u_idx[4] = { u,  u+1, u,   u+1 };
  float v_idx[4] = { v,  v,   v+1, v+1 };

  int valid = 0;
  double d[4];
  float wmax = -1.0;
  int imax = 0;
  for (int i=0; i<4; i++) {
    d[i] = stereo_->lookup_disparity(u_idx[i],v_idx[i]);
    if (d[i] > 0.0) ++valid;
    if (w[i] > wmax) {
        wmax = w[i];
        imax = i;
    }
  }
  double disp = 0.0;
  if (valid == 4) {
    for (int i=0; i<4; i++) disp += w[i] * d[i];
  }
  else if (valid>0)
  {
    disp = d[imax];
  }
  else return Eigen::Vector3d(NAN, NAN, NAN);

  double fx = cam_params_.fx;
  double cx = cam_params_.cx;
  double cy = cam_params_.cy;
  double tx = cam_params_.tx;
  // x,y,z
  double ww = tx / disp;
  double x = ((double)u - cx)*ww;
  double y = ((double)v - cy)*ww;
  double z = fx*ww;

  return Eigen::Vector3d(x,y,z);
}

VONode::VONode() : it_(nh_),
                   sync_(3),
                   calib_initialized_(false),
                   viewer_(g_lcm),
                   col_vo_(1, "Visual odometry"),
                   col_odo_(2, "Odometry"),
                   col_vo_odo_(3, "VO + ODO"),
                   link_vo_accept_(4, "Visual odometry - Accept"),
                   link_vo_reject_(5, "Visual odometry - Reject"),
                   link_odo_accept_(6, "Odometry - Accept"),
                   link_odo_reject_(7, "Odometry - Reject"),
                   link_vo_odo_accept_(8, "VO + ODO - Accept"),
                   link_vo_odo_reject_(9, "VO + ODO - Reject"),
                   last_utime_(0)
{
  l_image_sub_.subscribe(it_, "left/image_rect", 1);
  l_info_sub_.subscribe(nh_, "left/camera_info", 1);
  r_image_sub_.subscribe(it_, "right/image_rect", 1);
  r_info_sub_.subscribe(nh_, "right/camera_info", 1);

  sync_.connectInput(l_image_sub_, l_info_sub_, r_image_sub_, r_info_sub_);
  sync_.registerCallback( boost::bind(&VONode::imageCb, this, _1, _2, _3, _4) );

  // Temporary calibration it will be set
  // by the first CameraInfo messages
  calib_.width = 640;
  calib_.height = 480;
  calib_.fx = 421.60728;
  calib_.cx = 316.56639999999999;
  calib_.cy = 253.0677;
  calib_.k1 = 0;
  calib_.k2 = 0;

  odom_ = new rgbd_odometry::RgbdOdometry(&calib_);
  depth_producer_ = new StereoDepth();
  //depth_producer_ = new PrimeSenseDepth(&state->kcal);

  last_pose_ = Eigen::Isometry3d::Identity();
  last_cam_pose_ = Eigen::Isometry3d::Identity();
  last_base_pose_ = Eigen::Isometry3d::Identity();

  pose_vo_ = Isometry3d::Identity();
  pose_odo_ = Isometry3d::Identity();
  pose_vo_odo_ = Isometry3d::Identity();

  listener_ = new tf::TransformListener();
  last_base_tf_.setIdentity();
}

isam::Pose3d isometry_to_pose(const Isometry3d & Q)
{
  Eigen::Vector3d t(Q.translation());
  double rpy[3];
  convert_to_roll_pitch_yaw(Q,rpy);

  return isam::Pose3d(t[0],t[1],t[2],rpy[2],rpy[1],rpy[0]);
}

void VONode::imageCb(const sensor_msgs::ImageConstPtr& l_image,
		             const sensor_msgs::CameraInfoConstPtr& l_cam_info,
		             const sensor_msgs::ImageConstPtr& r_image,
		             const sensor_msgs::CameraInfoConstPtr& r_cam_info)
{
  sensor_msgs::CvBridge bridge;
  try
  {
    cv::Mat left = bridge.imgMsgToCv(l_image, "mono8");
    cv::Mat right = bridge.imgMsgToCv(r_image, "mono8");
    cv::imshow("view left", left);
    cv::imshow("view right", right);

    cam_model_.fromCameraInfo(l_cam_info, r_cam_info);

    frame_common::CamParams cam_params;
    cam_params.fx = cam_model_.left().fx();
    cam_params.fy = cam_model_.left().fy();
    cam_params.cx = cam_model_.left().cx();
    cam_params.cy = cam_model_.left().cy();
    cam_params.tx = cam_model_.baseline();

    frame_common::DenseStereo disp(left, right, 64, 0 );
    int width = l_cam_info->width;
    int height = l_cam_info->height;

    depth_producer_->setCamParams(cam_params);
    depth_producer_->setStereo(&disp);

    if (!calib_initialized_)
    {
      calib_.width = width;
      calib_.height = height;
      calib_.fx = cam_model_.left().fx();
      calib_.cx = cam_model_.left().cx();
      calib_.cy = cam_model_.left().cy();

      odom_->setCalibration(&calib_);
      calib_initialized_ = true;
    }
    odom_->processFrame(left.data, depth_producer_);
    Eigen::Isometry3d cam_to_local = odom_->getPose();

    tf::StampedTransform transform;
    tf::StampedTransform camera_to_base;
    ros::Duration timeout(1.0);

    // "/base_link"
    // "/wide_stereo_optical_frame"
    // "/wide_stereo_link"
    bool gotTransform = false;
    try{
      gotTransform = listener_->waitForTransform ("/odom_combined", "/wide_stereo_link",
          l_image->header.stamp, timeout);
      if (gotTransform)
      {
        listener_->lookupTransform("/odom_combined", "/base_link", l_image->header.stamp, transform);

        gotTransform = listener_->waitForTransform ("/wide_stereo_optical_frame", "/base_link",
            l_image->header.stamp, timeout);
        if (gotTransform)
        {
          listener_->lookupTransform("/base_link", "/wide_stereo_optical_frame", l_image->header.stamp, camera_to_base);
        }
      }
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    if (!gotTransform ) return;

    if (last_utime_ == 0)
    {
      last_cam_pose_ = cam_to_local;
      last_base_tf_ = transform;
    }

    tf::Transform delta_tf = last_base_tf_.inverse() * transform;
    Eigen::Isometry3d delta_odo;
    Transform2Iso(delta_tf, delta_odo);

    Eigen::Isometry3d camera_to_base_iso;
    Transform2Iso(camera_to_base, camera_to_base_iso);

    bool accepted = true;
    MotionEstimateStatusCode estim_status = odom_->getMotionEstimateStatus();
    if (estim_status == SUCCESS)
    {
      Eigen::Isometry3d delta = camera_to_base_iso * last_cam_pose_.inverse() * cam_to_local * camera_to_base_iso.inverse();
      Eigen::Isometry3d diff = delta_odo.inverse() * delta;
      pose_vo_ = pose_vo_ * delta;
      pose_vo_odo_ = pose_vo_odo_ * delta;
      last_pose_ = last_pose_ * delta;
      if (diff.translation().norm() > 0.1)
        std::cout << "norm: " << diff.translation().norm() << std::endl;
    }
    else
    {
      // Use some other estimate of the motion
      // std::cout << "delta_iso: " << delta_iso.translation()[0] << " " << delta_iso.translation()[1] << " " << delta_iso.translation()[2] << std::endl;
      pose_vo_odo_ = pose_vo_odo_ * delta_odo;
      last_pose_ = last_pose_ * delta_odo;
      accepted = false;
    }
    pose_odo_ = pose_odo_ * delta_odo;

    last_cam_pose_ = cam_to_local;
    last_base_tf_ = transform;

    int64_t utime = time_to_utime(l_image->header.stamp);

    sendPose(g_lcm, utime, last_pose_);
    displayImages(width,height,disp);

    // Add nodes
    col_vo_.add(utime, isometry_to_pose(pose_vo_));
    col_odo_.add(utime, isometry_to_pose(pose_odo_));
    col_vo_odo_.add(utime, isometry_to_pose(pose_vo_odo_));

    // Add links
    if (last_utime_ > 0)
    {
      // Add links
      if (accepted)
      {
        link_vo_accept_.add(utime, 1, last_utime_, 1, utime);
        link_odo_accept_.add(utime, 2, last_utime_, 2, utime);
        link_vo_odo_accept_.add(utime, 3, last_utime_, 3, utime);
      }
      else
      {
        link_vo_reject_.add(utime, 1, last_utime_, 1, utime);
        link_odo_reject_.add(utime, 2, last_utime_, 2, utime);
        link_vo_odo_reject_.add(utime, 3, last_utime_, 3, utime);
      }
    }

    // Send all the viewer collections
    viewer_.sendCollection(col_vo_,false);
    viewer_.sendCollection(col_odo_,false);
    viewer_.sendCollection(col_vo_odo_,false);
    viewer_.sendCollection(link_vo_accept_,false);
    viewer_.sendCollection(link_vo_reject_,false);
    viewer_.sendCollection(link_odo_accept_,false);
    viewer_.sendCollection(link_odo_reject_,false);
    viewer_.sendCollection(link_vo_odo_accept_,false);
    viewer_.sendCollection(link_vo_odo_reject_,false);

    col_vo_.clear();
    col_odo_.clear();
    col_vo_odo_.clear();
    link_vo_accept_.clear();
    link_odo_accept_.clear();
    link_vo_odo_accept_.clear();
    link_vo_reject_.clear();
    link_odo_reject_.clear();
    link_vo_odo_reject_.clear();

    last_utime_ = utime;
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", l_image->encoding.c_str());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vo");

  g_lcm = lcm_create(NULL);

  visual_odometry::resetPose(g_lcm);

  cvNamedWindow("view left");
  cvNamedWindow("view right");
  cvNamedWindow("disparity");
  cvStartWindowThread();

  VONode vo;

  ros::spin();

  cvDestroyWindow("view");
  return 0;
}
