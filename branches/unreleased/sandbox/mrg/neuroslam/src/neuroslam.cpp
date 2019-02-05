/*
 * neuroslam.cpp
 *
 *  Created on: Feb 9, 2011
 *      Author: Hordur Johannsson
 *
 * Neuro SLAM is a re-implementation of the RatSLAM algorithm.
 *
 * Reference: Persistent Navigation and Mapping using a Biologically
 *            Inspired SLAM system, Michael Milford and Gordon Wyeth,
 *            IJRR 2009
 *
 */

#include <iostream>
#include <map>
#include <cmath>
#include <algorithm>
#include <Eigen/Core>
#include <bot_core/rotations.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/stereo_camera_model.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/CvBridge.h>
#include <nav_msgs/Odometry.h>

#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_comparison.hpp>
#include <boost/unordered_map.hpp>
#include <boost/thread.hpp>
#include <boost/functional/hash.hpp>

#include <isam/isam.h>

#include "slam_tools/util.hpp"
#include "slam_tools/viewer.hpp"
#include "neuroslam/neuroslam.hpp"

using std::min;
using std::max;
using namespace Eigen;
using namespace neuroslam;

lcm_t* g_lcm;

class NeuroSlam
{
public:
  NeuroSlam();
  void addOdometry(int64_t utime, const isam::Pose3d & delta);
  bool setView(const View & view) {
    boost::mutex::scoped_lock lock(mutex_);
    return vc_.setView(view);
  }

  void update();
private:
  ViewCells vc_;
  //PoseCells pc_;
  PoseCellsSparse pc_;
  ExperienceMap em_;

  int64_t last_update_;
  int64_t last_utime_;
  isam::Pose3d delta_;
  isam::Pose3d pose_;
  isam::Pose3d first_cell_pose_;

  // We need to synchronize access to the view and pose cells/
  boost::mutex mutex_;

  Viewer viewer_;

  // Object collections
  ObjectCollection col_vo_;
  ObjectCollection col_ns_;
  ObjectCollection col_emap_;
  ObjectCollection col_emap_active_;

  // Link collections
  LinkCollection link_vo_;
  LinkCollection link_ns_;
  LinkCollection link_emap_;
};

NeuroSlam::NeuroSlam() : pc_(&vc_),
                         em_(&pc_, &vc_),
                         last_update_(0.0),
                         last_utime_(0.0),
                         viewer_(g_lcm),
                         col_vo_(1, "Odometry", MRLCM_OBJ_COLLECTION_T_POSE),
                         col_ns_(2, "Pose cell", MRLCM_OBJ_COLLECTION_T_POSE),
                         col_emap_(5, "Experience Map", MRLCM_OBJ_COLLECTION_T_POSE),
                         col_emap_active_(6, "Experience Map - Active", MRLCM_OBJ_COLLECTION_T_SQUARE),
                         link_vo_(3, "Odometry - path"),
                         link_ns_(4, "Pose cells - path"),
                         link_emap_(7, "Experience Map - Links")
{
}

void NeuroSlam::addOdometry(int64_t utime, const isam::Pose3d & delta)
{
  boost::mutex::scoped_lock lock(mutex_);

  pc_.addOdometry(utime, delta);
  last_utime_ = utime;
  delta_ = delta_.oplus(delta);
  pose_ = pose_.oplus(delta);

  std::cout << "Delta: " << delta << std::endl;
}

void NeuroSlam::update()
{
  boost::mutex::scoped_lock lock(mutex_);

  pc_.update();
  if (last_update_ > 0.0)
  {
    Experience current_experience = em_.getExperience();
    em_.update(last_utime_, pose_);
    //for (int i=0; i<50; i++)  em_.relaxConstraints();
    em_.relaxConstraints();
    Experience new_experience = em_.getExperience();

    if (current_experience.id != new_experience.id)
    {
      const std::vector<Experience> & experiences = em_.getExperiences();
      for (size_t i=1; i<experiences.size();++i) {
        col_emap_.add(experiences[i].utime, experiences[i].pose);
  //      col_emap_.add(new_experience.utime, new_experience.pose); // @todo only add if new
      }

      col_emap_active_.add(new_experience.utime, new_experience.pose);

      viewer_.sendCollection(col_emap_,false);
      viewer_.sendCollection(col_emap_active_,true);
      col_emap_.clear();
      col_emap_active_.clear();

      if (current_experience.id != -1) {
        link_emap_.add(last_utime_, 5, current_experience.utime, 5, new_experience.utime);
        viewer_.sendCollection(link_emap_,false);
        link_emap_.clear();
      }
    }
  }

  // Visualize pose cells
  const Array<ArrayXXd, Dynamic, 1> & P = pc_.poseNetwork();
  int width = P(0).rows();
  int height = P(0).cols();

  cv::Mat img(cv::Size(600,600), CV_32FC1);
  cv::Mat cells(cv::Size(width,height), CV_32FC1);

  for (int x=0; x<width; ++x) {
    for (int y=0; y<height; ++y) {
      cells.at<float>(x,y) = 0.0;
      for (int t=0; t<P.rows(); ++t) {
        cells.at<float>(x,y) += P(t)(x,y);
      }
      cells.at<float>(x,y) = pow(cells.at<float>(x,y),0.2);
    }
  }

  cv::resize(cells, img, cv::Size(600,600), 0, 0, cv::INTER_NEAREST);
  cv::imshow("PoseCells", img);
  //cv::imshow("PoseCells", cells);

  // Update trajectory
  if (last_update_ == 0.0) {
    first_cell_pose_ = pc_.getCellPose();
  }

  // Add nodes
  if (last_utime_ > 0.0) {
    std::cout << "PoseCell: " << (pc_.getCellPose()).ominus(first_cell_pose_) << std::endl;
    std::cout << "Odometry: " << pose_ << std::endl;

    col_vo_.add(last_utime_, pose_);
    col_ns_.add(last_utime_, (pc_.getCellPose()).ominus(first_cell_pose_));

    viewer_.sendCollection(col_vo_,false);
    viewer_.sendCollection(col_ns_,false);
  }

  // Add links
  if (last_update_ > 0.0) {
    link_vo_.add(last_utime_, 1, last_update_, 1, last_utime_);
    link_ns_.add(last_utime_, 2, last_update_, 2, last_utime_);
    viewer_.sendCollection(link_vo_,false);
    viewer_.sendCollection(link_ns_,false);
  }

  delta_ = isam::Pose3d(0.0,0.0,0.0,0.0,0.0,0.0);
  last_update_ = last_utime_;
}

class ROSAdapter
{
public:
  ROSAdapter(NeuroSlam* ns);
  void run();
private:
  void on_image(const sensor_msgs::ImageConstPtr& l_image,
                           const sensor_msgs::CameraInfoConstPtr& l_cam_info,
                           const sensor_msgs::ImageConstPtr& r_image,
                           const sensor_msgs::CameraInfoConstPtr& r_cam_info);
  void on_odometry(const nav_msgs::Odometry::ConstPtr& odom);

  ros::NodeHandle nh_;
  NeuroSlam* ns_;
  ros::Subscriber odom_sub_;
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter l_image_sub_, r_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub_, r_info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
  sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;

  int odo_count_;
  int view_count_;
  nav_msgs::Odometry last_odom_;
  isam::Pose3d last_pose_;
  int64_t last_utime_;

  int64_t last_view_utime_;
  View last_view_;

  boost::mutex mutex_;
};

ROSAdapter::ROSAdapter(NeuroSlam* ns) : ns_(ns),
                                        it_(nh_),
                                        sync_(3),
                                        odo_count_(0),
                                        view_count_(0),
                                        last_utime_(0)
{
  odom_sub_ = nh_.subscribe("odometry", 100, &ROSAdapter::on_odometry, this);

  l_image_sub_.subscribe(it_, "/wide_stereo/left/image_rect", 1);
  l_info_sub_.subscribe(nh_, "/wide_stereo/left/camera_info", 1);
  r_image_sub_.subscribe(it_, "/wide_stereo/right/image_rect", 1);
  r_info_sub_.subscribe(nh_, "/wide_stereo/right/camera_info", 1);

  sync_.connectInput(l_image_sub_, l_info_sub_, r_image_sub_, r_info_sub_);
  sync_.registerCallback( boost::bind(&ROSAdapter::on_image, this, _1, _2, _3, _4) );

  // @todo  notify the neuro slam
}

void ROSAdapter::on_image(const sensor_msgs::ImageConstPtr& l_image,
                         const sensor_msgs::CameraInfoConstPtr& l_cam_info,
                         const sensor_msgs::ImageConstPtr& r_image,
                         const sensor_msgs::CameraInfoConstPtr& r_cam_info)
{

  sensor_msgs::CvBridge bridge;
  try
  {
    cv::Mat left = bridge.imgMsgToCv(l_image, "mono8");
    cv::Mat right = bridge.imgMsgToCv(r_image, "mono8");

    cv::Mat tag(cv::Size(12,8), CV_8UC1);
    cv::Mat img(cv::Size(300,300), CV_8UC1);

    cv::resize(left, tag, cv::Size(12,8), 0, 0, cv::INTER_AREA);
    cv::resize(tag, img, cv::Size(300,300), 0, 0, cv::INTER_NEAREST);
    {
      boost::mutex::scoped_lock lock(mutex_);

      View v(12*8);
      for (int i=0; i<(12*8); ++i) v(i) = tag.at<unsigned char>(i);
      last_view_utime_ = time_to_utime(l_image->header.stamp);
      last_view_ = v;
      view_count_ = 1;
    }

    cv::imshow("tag", img);
    cv::imshow("view left", left);
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", l_image->encoding.c_str());
  }
}

void ROSAdapter::on_odometry(const nav_msgs::Odometry::ConstPtr& odom)
{
  boost::mutex::scoped_lock lock(mutex_);
  last_odom_ = *odom;
  odo_count_ = 1;
}

void ROSAdapter::run()
{
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    View view(12*8);
    int64 view_utime = 0;
    int64 utime = 0;
    isam::Pose3d pose;

    int odo_count;
    int view_count;

    {
      // Read shared variables
      boost::mutex::scoped_lock lock(mutex_);
      view_count = view_count_;
      odo_count = odo_count_;
      if (view_count>0 && odo_count>0)
      {
        view = last_view_;
        view_utime = last_view_utime_;
        odometry_to_pose3d(boost::make_shared<nav_msgs::Odometry>(last_odom_), pose);
        utime = time_to_utime(last_odom_.header.stamp);
        view_count_ = 0;
        odo_count_ = 0;
      }
    }

    if (view_count>0 && odo_count>0)
    {
      if (last_utime_ == 0) {
        last_utime_ = utime;
        last_pose_ = pose;
      }

      std::cout << "RUN: time diff " << (view_utime - utime)*1E-6 << std::endl;
      isam::Pose3d delta = pose.ominus(last_pose_);
      ns_->setView(view);
      ns_->addOdometry(utime, delta);
      ns_->update();

      last_pose_ = pose;
      last_utime_ = utime;
    }

    loop_rate.sleep();
  }
}

/*
 * Visualization
 *
 * Show the posecell network in 2d
 *          -- " --          in 3d
 *
 * Show active view cells
 * Show view cell connections for active view cells
 *
 * Show the experience map
 * Show trajectory of max activated pose cell
 *
 */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "neuroslam");
  g_lcm = lcm_create(NULL);

  NeuroSlam neuro_slam;
  ROSAdapter rosadapter(&neuro_slam);
  cvNamedWindow("PoseCells");
  cvNamedWindow("tag");
  cvNamedWindow("view left");
  cvStartWindowThread();

  boost::thread neuroThead(&ROSAdapter::run, &rosadapter);

  ros::spin();

}

