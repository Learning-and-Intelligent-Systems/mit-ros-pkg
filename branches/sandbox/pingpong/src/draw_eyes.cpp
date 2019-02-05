#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <cxcore.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <pingpong/EyesState.h>
#include <pingpong/EyeState.h>

#include "util.h"

using namespace std;


typedef struct {
  list<sensor_msgs::Image> left;
  list<sensor_msgs::Image> right;
} stereo_queue_t;


// constants
#define RED CV_RGB(255,0,0)
#define GREEN CV_RGB(0,255,0)
#define BLUE CV_RGB(0,0,255)


// global variables
ros::Publisher left_pub, right_pub;
stereo_queue_t stereo_queue;
pingpong::EyesState eyes;
bool got_eyes = false;


void draw_table(cv_bridge::CvImagePtr cv_ptr, pingpong::EyeState &eye)
{
  // outside boundary
  for (uint i = 0; i < eye.table_corners.size(); i++) {
    cv::Point2d p0(eye.table_corners[i].x, eye.table_corners[i].y);
    cv::Point2d p1(eye.table_corners[(i+1)%4].x, eye.table_corners[(i+1)%4].y);
    cv::line(cv_ptr->image, p0, p1, GREEN);
  }
  /* middle line
  cv::Point2d p0((eye.table_corners[0].x + eye.table_corners[1].x)/2.0,
		 (eye.table_corners[0].y + eye.table_corners[1].y)/2.0);
  cv::Point2d p1((eye.table_corners[2].x + eye.table_corners[3].x)/2.0,
		 (eye.table_corners[2].y + eye.table_corners[3].y)/2.0);
  cv::line(cv_ptr->image, p0, p1, GREEN);
  */
}

void draw_roi(cv_bridge::CvImagePtr cv_ptr, pingpong::EyeState &eye)
{
  cv::Point2d p1(eye.roi.x, eye.roi.y);
  cv::Point2d p2(eye.roi.x + eye.roi.w, eye.roi.y);
  cv::Point2d p3(eye.roi.x + eye.roi.w, eye.roi.y + eye.roi.h);
  cv::Point2d p4(eye.roi.x, eye.roi.y + eye.roi.h);
  cv::line(cv_ptr->image, p1, p2, RED);
  cv::line(cv_ptr->image, p2, p3, RED);
  cv::line(cv_ptr->image, p3, p4, RED);
  cv::line(cv_ptr->image, p4, p1, RED);
}

void draw_ball_trajectory(cv_bridge::CvImagePtr cv_ptr, pingpong::EyeState &eye)
{
  if (eye.ball_trajectory.size() == 0)
    return;

  // past
  for (uint i = 0; i < eye.ball_trajectory.size(); i++) {
    pingpong::Circle &b = eye.ball_trajectory[i];
    cv::circle(cv_ptr->image, cv::Point(b.x, b.y), ceil(b.r), RED, 1);
  }

  // future
  double r = ceil(eye.ball_trajectory.back().r);
  for (uint i = 0; i < eye.predicted_ball_trajectory.size(); i++) {
    pingpong::Circle &b = eye.predicted_ball_trajectory[i];
    cv::circle(cv_ptr->image, cv::Point(b.x, b.y), r, MAGENTA, 1);
  }
}

void draw_eyes(const sensor_msgs::Image &left, const sensor_msgs::Image &right)
{
  // convert to opencv
  cv_bridge::CvImagePtr cv_ptr_left, cv_ptr_right;
  try {
    cv_ptr_left =
      cv_bridge::toCvCopy(left, sensor_msgs::image_encodings::BGR8);
    cv_ptr_right =
      cv_bridge::toCvCopy(right, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  draw_table(cv_ptr_left, eyes.left);
  draw_table(cv_ptr_right, eyes.right);

  draw_roi(cv_ptr_left, eyes.left);  
  draw_roi(cv_ptr_right, eyes.right);

  draw_ball_trajectory(cv_ptr_left, eyes.left);
  draw_ball_trajectory(cv_ptr_right, eyes.right);

  // convert to ROS
  sensor_msgs::Image I1, I2;
  cv_ptr_left->toImageMsg(I1);
  cv_ptr_right->toImageMsg(I2);

  left_pub.publish(I1);
  right_pub.publish(I2);
}


//------------------- STEREO IMAGE QUEUE ------------------//

bool order_image_stamps(sensor_msgs::Image &left, sensor_msgs::Image &right)
{
  return left.header.stamp < right.header.stamp;
}

void stereo_queue_process(stereo_queue_t &sq)
{
  sq.left.sort(order_image_stamps);
  sq.right.sort(order_image_stamps);

  while (sq.left.size() > 0 && sq.right.size() > 0) {
    if (fabs(sq.left.front().header.stamp.toSec() - sq.right.front().header.stamp.toSec()) < .000001) {
      sensor_msgs::Image &left = sq.left.front();
      sensor_msgs::Image &right = sq.right.front();
      //if (left.header.stamp.toSec() == eyes.stamp.toSec())
	draw_eyes(left, right);
      if (left.header.stamp.toSec() <= eyes.stamp.toSec()) {
	sq.left.pop_front();
	sq.right.pop_front();
      }
      else
	break;
    }
    else if (order_image_stamps(sq.left.front(), sq.right.front()) && sq.left.size() > 5)
      sq.left.pop_front();
    else if (order_image_stamps(sq.right.front(), sq.left.front()) && sq.right.size() > 5)
    sq.right.pop_front();
    else
      break;
  }

  if (sq.left.size() > 100 || sq.right.size() > 100) {
    sq.left.clear();
    sq.right.clear();
  }
}

//---------------------- CALLBACKS -----------------//

void left_image_callback(const sensor_msgs::Image &msg)
{
  if (got_eyes) {
    stereo_queue.left.push_back(msg);
    stereo_queue_process(stereo_queue);
  }
}

void right_image_callback(const sensor_msgs::Image &msg)
{
  if (got_eyes) {
    stereo_queue.right.push_back(msg);
    stereo_queue_process(stereo_queue);
  }
}

void eyes_state_callback(const pingpong::EyesState &msg)
{
  //printf("Got eyes state\n"); //dbug
  got_eyes = true;
  eyes = msg;
  stereo_queue_process(stereo_queue);
}


int main(int argc, char *argv[])
{
  // init ROS
  ros::init(argc, argv, "pingpongdraweyes");
  ros::NodeHandle nh;
  ros::Subscriber left_image_sub = nh.subscribe("/svstereo/left/image_raw", 10, left_image_callback);
  ros::Subscriber right_image_sub = nh.subscribe("/svstereo/right/image_raw", 10, right_image_callback);
  //ros::Subscriber left_image_sub = nh.subscribe("/svstereo/left/image_rect", 10, left_image_callback);
  //ros::Subscriber right_image_sub = nh.subscribe("/svstereo/right/image_rect", 10, right_image_callback);
  ros::Subscriber eyes_sub = nh.subscribe("eyes_state", 1, eyes_state_callback);
  left_pub = nh.advertise<sensor_msgs::Image>("/svstereo/left/eyes", 1);
  right_pub = nh.advertise<sensor_msgs::Image>("/svstereo/right/eyes", 1);
  ros::spin();

  return 0;
}
