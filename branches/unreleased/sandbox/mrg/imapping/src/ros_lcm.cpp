/*
 * ros_lcm.cpp
 *
 *  Created on: Feb 4, 2011
 *      Author: hordurj
 *
 *  A ROS to LCM bridge
 *
 */

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <lcm/lcm.h>
#include <lcmtypes/bot_core.h>

#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <message_filters/time_synchronizer.h>
#include "laser_geometry/laser_geometry.h"
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/image_encodings.h>

class LCMBridge
{
public:
    LCMBridge();
    void on_lidar_data(const sensor_msgs::LaserScan::ConstPtr& scan_in);
    void imageCb(const sensor_msgs::ImageConstPtr& l_image,
                 const sensor_msgs::CameraInfoConstPtr& l_cam_info,
                 const sensor_msgs::ImageConstPtr& r_image,
                 const sensor_msgs::CameraInfoConstPtr& r_cam_info);
    void on_image(const sensor_msgs::ImageConstPtr& msg,
      const sensor_msgs::CameraInfoConstPtr& info_msg);


private:
    void publish_lidar(const sensor_msgs::LaserScan::ConstPtr& scan_in);

    ros::NodeHandle nh_;
    tf::TransformListener* listener_;
    lcm_t* lcm_;
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter l_image_sub_, r_image_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub_, r_info_sub_;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
    sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;
    ros::Subscriber scan_sub_;
    image_transport::CameraSubscriber sub_camera_;
};

LCMBridge::LCMBridge() : it_(nh_), sync_(3)
{
    lcm_ = lcm_create(NULL);
    listener_ = 0;
    scan_sub_ = nh_.subscribe("base_scan", 100, &LCMBridge::on_lidar_data, this);

    l_image_sub_.subscribe(it_, "/wide_stereo/left/image_rect", 1);
    l_info_sub_.subscribe(nh_, "/wide_stereo/left/camera_info", 1);
    r_image_sub_.subscribe(it_, "/wide_stereo/right/image_rect", 1);
    r_info_sub_.subscribe(nh_, "/wide_stereo/right/camera_info", 1);

    sync_.connectInput(l_image_sub_, l_info_sub_, r_image_sub_, r_info_sub_);
    sync_.registerCallback( boost::bind(&LCMBridge::imageCb, this, _1, _2, _3, _4) );

    sub_camera_ = it_.subscribeCamera("image", 1, &LCMBridge::on_image, this);
}

void LCMBridge::on_image(const sensor_msgs::ImageConstPtr& msg,
    const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  namespace enc = sensor_msgs::image_encodings;

  try
  {
    //cv_bridge::CvImagePtr color_image = cv_bridge::toCvCopy(msg, enc::BGR8);
    cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(msg, enc::MONO8);
    int width = img->image.size().width;
    int height = img->image.size().height;

    bot_core_image_t image;
    const int* tptr = (const int*)(&(msg->header.stamp));
    int secs = *tptr;
    int nsecs = *(tptr+1);
    int64_t utime = secs * 1E6 + nsecs/1E3;
    size_t size = width*height;

    image.utime = utime;
    image.size = size;
    image.data = new unsigned char[image.size];
    image.nmetadata = 0;
    image.metadata = 0;
    image.width = width;
    image.height = height;
    image.row_stride = image.width;
    image.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
    std::copy(img->image.begin<uint8_t>(), img->image.end<uint8_t>(), image.data);

    bot_core_image_t_publish(lcm_, "IMAGE", &image);

    delete [] image.data;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert image from '%s'.", e.what());
  }
}

void LCMBridge::imageCb(const sensor_msgs::ImageConstPtr& l_image,
                             const sensor_msgs::CameraInfoConstPtr& l_cam_info,
                             const sensor_msgs::ImageConstPtr& r_image,
                             const sensor_msgs::CameraInfoConstPtr& r_cam_info)
{
  bot_core_image_t image;
  const int* tptr = (const int*)(&(l_image->header.stamp));
  int secs = *tptr;
  int nsecs = *(tptr+1);
  int64_t utime = secs * 1E6 + nsecs/1E3;

  size_t size = l_image->width * l_image->height;

  std::cout << "step: " << l_image->step << std::endl;
  image_geometry::StereoCameraModel cam_model;
  cam_model.fromCameraInfo(l_cam_info, r_cam_info);
  //std::cout << "fx: " << cam_model.left().fx()
  //          << "fy: " << cam_model.left().fy()
  //          << "cx: " << cam_model.left().cx()
  //          << "cy: " << cam_model.left().cy()
  //          << "tx: " << cam_model.baseline() << std::endl;

  image.utime = utime;
  image.size = 2 * size;
  image.data = new unsigned char[image.size];
  image.nmetadata = 0;
  image.metadata = 0;
  image.width = l_image->width;
  image.height = l_image->height * 2;
  image.row_stride = image.width;
  image.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;

  for (size_t i=0; i<size; ++i) image.data[i] = l_image->data[i];
  for (size_t i=0; i<size; ++i) image.data[i+size] = r_image->data[i];

  bot_core_image_t_publish(lcm_, "CAMLCM_IMAGE", &image);

  delete [] image.data;
}

void LCMBridge::on_lidar_data(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    publish_lidar(scan_in);
}

void LCMBridge::publish_lidar(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    const int* tptr = (const int*)(&(scan_in->header.stamp));
    int secs = *tptr;
    int nsecs = *(tptr+1);
    int64_t utime = secs * 1E6 + nsecs/1E3;

    bot_core_planar_lidar_t lidar;
    lidar.nintensities = scan_in->intensities.size();
    lidar.intensities = new float[lidar.nintensities];
    lidar.nranges = scan_in->ranges.size();
    lidar.ranges = new float[lidar.nranges];
    lidar.rad0 = scan_in->angle_min;
    lidar.radstep = scan_in->angle_increment;
    lidar.utime = utime;

    for (int i=0; i<lidar.nintensities; i++) lidar.intensities[i] = scan_in->intensities[i];
    for (int i=0; i<lidar.nranges; i++) lidar.ranges[i] = scan_in->ranges[i];

    bot_core_planar_lidar_t_publish(lcm_, "LASER", &lidar);

    delete [] lidar.intensities;
    delete [] lidar.ranges;

    return ;

    bot_core_pose_t pose;
    memset(&pose,0,sizeof(bot_core_pose_t));
    pose.utime = utime;
    if (!listener_) listener_ = new tf::TransformListener();

    tf::StampedTransform transform;
    ros::Duration timeout(1.0);
    bool gotTransform = false;
    try{
        gotTransform = listener_->waitForTransform ("/odom_combined", "/base_laser_link",
                        scan_in->header.stamp, timeout);
        if (gotTransform)
        {
            listener_->lookupTransform("/odom_combined", "/base_laser_link", scan_in->header.stamp, transform);
        }
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }

    if (gotTransform)
    {
        pose.pos[0] = transform.getOrigin().x();
        pose.pos[1] = transform.getOrigin().y();
        pose.pos[2] = transform.getOrigin().z();

        pose.orientation[1] = transform.getRotation().getX();
        pose.orientation[2] = transform.getRotation().getY();
        pose.orientation[3] = transform.getRotation().getZ();
        pose.orientation[0] = transform.getRotation().getW();

        bot_core_pose_t_publish(lcm_,"POSE",&pose);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roslcm");

    LCMBridge lcmbridge;

    ros::spin();
}
