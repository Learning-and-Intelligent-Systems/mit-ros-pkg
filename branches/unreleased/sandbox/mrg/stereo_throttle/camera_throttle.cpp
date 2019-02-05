/*
Node to output throttled versions of the stereo camera raw images and camera_info (which need to stay synchronized)
*/

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h> 

using std::string;
using namespace sensor_msgs;
using namespace message_filters;

typedef sync_policies::ExactTime<Image, Image> StereoCameraSyncPolicy;

// class for broadcasting synchronized, throttled versions of a stereo camera topic
class StereoCameraThrottler
{
private:
  ros::NodeHandle node_;    
  ros::Publisher left_image_pub_;  
  ros::Publisher right_image_pub_;  
  ros::Publisher left_camera_info_pub_;  
  ros::Publisher right_camera_info_pub_;  
  message_filters::Subscriber<Image> left_sub_;
  message_filters::Subscriber<Image> right_sub_;
  Synchronizer<StereoCameraSyncPolicy> sync_;
  ros::Subscriber left_info_sub_;
  ros::Subscriber right_info_sub_;
  CameraInfo left_cam_info_;
  CameraInfo right_cam_info_;
  bool received_left_cam_info_;
  bool received_right_cam_info_;
  double msg_rate_;
  double last_msg_time_;
  string image_topic_;

public:

  StereoCameraThrottler(string image_topic, string out_topic, double rate):
    left_sub_(node_, image_topic + string("/left/image_raw"), 1),
    right_sub_(node_, image_topic + string("/right/image_raw"), 1),
    sync_(StereoCameraSyncPolicy(10), left_sub_, right_sub_)
  {
    //store the topic and initial rate and initialize the last message time
    image_topic_ = image_topic;
    msg_rate_ = rate;
    last_msg_time_ = 0;
    received_left_cam_info_ = 0;
    received_right_cam_info_ = 0;
    
    //subscribe to the left and right camera info messages 
    left_info_sub_ = node_.subscribe(image_topic_ + string("/left/camera_info"), 5, &StereoCameraThrottler::left_info_cb, this);
    right_info_sub_ = node_.subscribe(image_topic_ + string("/right/camera_info"), 5, &StereoCameraThrottler::right_info_cb, this);
							
    //register the callback for the topic synchronizer
    sync_.registerCallback(boost::bind(&StereoCameraThrottler::stereo_cb, this, _1, _2));

    //broadcast the throttled versions on out_topic
    left_image_pub_ = node_.advertise<Image>(out_topic + string("/left/image_raw"), 10);
    right_image_pub_ = node_.advertise<Image>(out_topic + string("/right/image_raw"), 10);
    left_camera_info_pub_ = node_.advertise<CameraInfo>(out_topic + string("/left/camera_info"), 10);
    right_camera_info_pub_ = node_.advertise<CameraInfo>(out_topic + string("/right/camera_info"), 10);

    ROS_INFO("done init for %s", (image_topic_ + string("/left/image_raw")).c_str());
  }
  
  //record the left camera info (once)
  void left_info_cb(const CameraInfoConstPtr& cam_info)
  {
    if(received_left_cam_info_) return;
    left_cam_info_ = *cam_info;
    received_left_cam_info_ = 1;
    left_camera_info_pub_.publish(left_cam_info_);
    ROS_INFO("got left_cam_info for %s", image_topic_.c_str());
  }

  //record the right camera info (once)
  void right_info_cb(const CameraInfoConstPtr& cam_info)
  {
    if(received_right_cam_info_) return;
    right_cam_info_ = *cam_info;
    received_right_cam_info_ = 1;
    right_camera_info_pub_.publish(right_cam_info_);
    ROS_INFO("got right_cam_info for %s", image_topic_.c_str());
  }
				   

  //re-broadcast the stereo images and their info, throttled to the desired frequencies
  void stereo_cb(const ImageConstPtr& left_image, const ImageConstPtr& right_image)
  {
    //ROS_INFO("got synchronized image for %s", image_topic_.c_str());
    if(ros::Time::now().toSec() - last_msg_time_ > 1/msg_rate_)
    {
      left_image_pub_.publish(left_image);
      right_image_pub_.publish(right_image);
      if(received_left_cam_info_){
	left_cam_info_.header.stamp = left_image->header.stamp;
	left_camera_info_pub_.publish(left_cam_info_);
      }
      if(received_right_cam_info_){
	right_cam_info_.header.stamp = right_image->header.stamp;
	right_camera_info_pub_.publish(right_cam_info_);
      }
      last_msg_time_ = ros::Time::now().toSec();
      //ROS_INFO("broadcasting synchronized image for %s", image_topic_.c_str());
    }
  }

};


int main(int argc, char** argv)
{
  if(argc < 4)
  {
    puts("usage: throttle_stereo NARROW_STEREO_RATE NARROW_STEREO_TEXTURED_RATE WIDE_STEREO_RATE\n\n");
    return 1;
  }
  
  double narrow_stereo_rate = atof(argv[1]);
  double narrow_stereo_textured_rate = atof(argv[2]);
  double wide_stereo_rate = atof(argv[3]);

  ros::init(argc, argv, "throttle_stereo_images");

  //create throttler-synchronizers for the wide, narrow, and narrow_textured stereo cameras
  StereoCameraThrottler *narrow_stereo_throttler = new StereoCameraThrottler(string("/narrow_stereo"), 
					     string("/narrow_stereo_throttled"), narrow_stereo_rate);
  StereoCameraThrottler *narrow_stereo_textured_throttler = new StereoCameraThrottler(string("/narrow_stereo_textured"), 
					     string("/narrow_stereo_textured_throttled"), narrow_stereo_textured_rate);
  StereoCameraThrottler *wide_stereo_throttler = new StereoCameraThrottler(string("/wide_stereo"), 
					     string("/wide_stereo_throttled"), wide_stereo_rate);

  ros::spin();

  delete narrow_stereo_throttler;
  delete narrow_stereo_textured_throttler;
  delete wide_stereo_throttler;

  return 0;
}
