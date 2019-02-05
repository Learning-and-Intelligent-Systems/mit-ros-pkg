/*
 * test_dbow.cpp
 *
 *  Created on: Feb 13, 2011
 *      Author: hordurj
 */

/*
 * ros_lcm.cpp
 *
 *  Created on: Feb 4, 2011
 *      Author: hordurj
 *
 *  A ROS to LCM bridge
 *
 */

#include <map>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <lcm/lcm.h>
#include <bot/core/lcmtypes/lcmtypes.h>
#include <lcmtypes/bot_core.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

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
#include "opencv2/opencv.hpp"

#include "DUtils.h"
#include "DBow.h"

using namespace DBow;
using namespace DUtils;

class DBowTest
{
public:
  DBowTest();
  void imageCb(const sensor_msgs::ImageConstPtr& l_image,
                 const sensor_msgs::CameraInfoConstPtr& l_cam_info,
                 const sensor_msgs::ImageConstPtr& r_image,
                 const sensor_msgs::CameraInfoConstPtr& r_cam_info);

  void createVocabulary(const std::vector<std::vector<float> > & features);
  void match();
  void load();
  double query(const std::vector<float> & features);

private:
  ros::NodeHandle nh_;
  tf::TransformListener* listener_;
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter l_image_sub_, r_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub_, r_info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
  sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;

  std::vector<std::vector<float> > features_;

  int frame_count_;

  Database* db_;

  bool vocab_saved_;
  bool create_vocabulary_;

  std::map<EntryId, cv::Mat> keyframes_;
  std::string vocab_name_;

};

DBowTest::DBowTest() : it_(nh_), sync_(3), frame_count_(0), db_(0), vocab_saved_(false),
                       create_vocabulary_(false), vocab_name_("my_vocab_text_64.txt")
{
  listener_ = 0;

  l_image_sub_.subscribe(it_, "/wide_stereo/left/image_rect", 1);
  l_info_sub_.subscribe(nh_, "/wide_stereo/left/camera_info", 1);
  r_image_sub_.subscribe(it_, "/wide_stereo/right/image_rect", 1);
  r_info_sub_.subscribe(nh_, "/wide_stereo/right/camera_info", 1);

  sync_.connectInput(l_image_sub_, l_info_sub_, r_image_sub_, r_info_sub_);
  sync_.registerCallback( boost::bind(&DBowTest::imageCb, this, _1, _2, _3, _4) );

  if (!create_vocabulary_) load();
}

void DBowTest::createVocabulary(const std::vector<std::vector<float> > & features)
{
  const int k = 9;
  const int L = 3;
  HVocParams params(k, L, 64); // 64 or 128 bit surf features
  HVocabulary voc(params);

  // std::vector<std::vector<float > > features;
  voc.Create(features);
  voc.StopWords(0.01f);

  std::cout << "Create Vocabulary: Info: " << voc.RetrieveInfo().toString() << std::endl;

  voc.Save(vocab_name_.c_str(), false);
}

void DBowTest::match()
{
  /*
  BowVector v1, v2;
  voc.Transform(features[i], v1);
  voc.Transform(features[j], v2);
  voc.Score(v1, v2);
  */
}

void DBowTest::load()
{
  HVocabulary* voc = new HVocabulary("small_vocabulary.txt");
  //HVocabulary* voc = new HVocabulary(vocab_name_.c_str());
  db_ = new Database(*voc);
  delete voc;
}

double DBowTest::query(const std::vector<float> & features)
{
  QueryResults ret;
  db_->Query(ret, features, 2);
  if (ret[1].Score > 0.1) {
    std::cout << "Match-0: " << ret[0].Id << " " << ret[0].Score << std::endl;
    std::cout << "Match-1: " << ret[1].Id << " " << ret[1].Score << std::endl;

    imshow("Match", keyframes_[ret[0].Id]);
  }
  return ret[0].Score;
}

void DBowTest::imageCb(const sensor_msgs::ImageConstPtr& l_image,
                       const sensor_msgs::CameraInfoConstPtr& l_cam_info,
                       const sensor_msgs::ImageConstPtr& r_image,
                       const sensor_msgs::CameraInfoConstPtr& r_cam_info)
{
  sensor_msgs::CvBridge bridge;
  cv::Mat img = bridge.imgMsgToCv(l_image, "mono8");
  cv::Mat img_color;
  cv::Mat mask;

  std::vector<float> features;
  vector<cv::KeyPoint> keypoints;

  cv::SURF surf(400, 4, 2, 64);

  surf(img, mask, keypoints, features);
  // sift(img, mask, keypoints, features);
  cv::cvtColor(img, img_color, CV_GRAY2BGR);

  for (size_t i=0;i<keypoints.size();i++)
  {
    if (keypoints[i].response > 700)
    {
      cv::circle(img_color, keypoints[i].pt, keypoints[i].size/10.0, cv::Scalar(0,0,255,255), 2);
    }
  }

  if (create_vocabulary_)
  {
    if (frame_count_%10 == 0)
    {
      features_.push_back(features);
    }
    std::cout << "Image: " << frame_count_ << " " << features_.size() << std::endl;
    if(features_.size() == 100 && !vocab_saved_)
    {
      createVocabulary(features_);
      vocab_saved_ = true;
    }
  }
  else
  {
    double score = query(features);

    if (score < 0.4) // frame_count_%10 == 0)
    {
        std::cout << "Add keyframe:  " << score << std::endl;
        EntryId id = db_->AddEntry(features);
        //cv::Mat frame(img);
        keyframes_[id] = img.clone(); //frame;
    }
  }

  ++frame_count_;

  imshow("Image", img_color);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_dbow");

  cvNamedWindow( "Image", CV_WINDOW_AUTOSIZE);
  cvNamedWindow( "Match", CV_WINDOW_AUTOSIZE);
  cvStartWindowThread();

  DBowTest dbow_test;

  ros::spin();
}
