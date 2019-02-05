#include <algorithm>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <cxcore.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/stereo_camera_model.h>
#include <bingham/util.h>
#include "util.h"

using namespace std;


typedef struct {
  list<sensor_msgs::Image> left;
  list<sensor_msgs::Image> right;
} stereo_queue_t;

stereo_queue_t stereo_queue;
image_geometry::StereoCameraModel stereo_camera_model;
tf::Transform table_camera_transform;
double table_camera_transform_fitness = 1e32;
cv::Point3d table_corners[4];
ros::Publisher left_pub, right_pub;

// corner order: NW, NE, SE, SW (where NORTH points towards the far side of the table)
double table_corners_left[4][2] = {{86, -170}, {505, -150}, {510, 597}, {89, 612}};
  //{{86, -242}, {505, -221}, {510, 525}, {89, 537}};
double table_corners_right[4][2] = {{44, -167}, {465, -146}, {469, 600}, {47, 611}};
  //{{44, -242}, {465, -221}, {469, 525}, {47, 537}};

#define RED CV_RGB(255,0,0)
#define GREEN CV_RGB(0,255,0)
#define BLUE CV_RGB(0,0,255)



void draw_table(const sensor_msgs::Image &left, const sensor_msgs::Image &right)
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

  // draw table
  cv::Point3d corners[4];
  table_corners_from_transform(corners, table_camera_transform);
  cv::Point2d left_corners[4], right_corners[4];
  for (uint i = 0; i < 4; i++) {
    left_corners[i] = stereo_camera_model.left().project3dToPixel(corners[i]);
    right_corners[i] = stereo_camera_model.right().project3dToPixel(corners[i]);
  }
  for (uint i = 0; i < 4; i++) {
    cv::line(cv_ptr_left->image, left_corners[i], left_corners[(i+1)%4], GREEN);
    cv::line(cv_ptr_right->image, right_corners[i], right_corners[(i+1)%4], GREEN);
  }

  // convert to ROS
  sensor_msgs::Image I1, I2;
  cv_ptr_left->toImageMsg(I1);
  cv_ptr_right->toImageMsg(I2);

  left_pub.publish(I1);
  right_pub.publish(I2);
}


//------------------- STEREO IMAGE QUEUE ------------------//

void process_stereo_images(const sensor_msgs::Image &left, const sensor_msgs::Image &right)
{
  tf::Transform new_transform = table_camera_transform;
  double new_fitness = fit_table(new_transform, left, right, stereo_camera_model);

  printf("min fitness = %f, new fitness = %f\n", table_camera_transform_fitness, new_fitness);

  static int cnt=0;
  cnt++;

  if (cnt < 100 || new_fitness < table_camera_transform_fitness) {
    table_camera_transform_fitness = new_fitness;
    table_camera_transform = new_transform;
  }

  VectorXf xq = transform_to_vector(table_camera_transform);
  printf("tf::Transform(tf::Quaternion(%f,%f,%f,%f), tf::Point(%f,%f,%f))\n",
	 xq(4),xq(5),xq(6),xq(3),xq(0),xq(1),xq(2));

  draw_table(left, right);
}

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
      process_stereo_images(sq.left.front(), sq.right.front());
      sq.left.pop_front();
      sq.right.pop_front();
    }
    else if (order_image_stamps(sq.left.front(), sq.right.front()) && sq.left.size() > 5)
      sq.left.pop_front();
    else if (order_image_stamps(sq.right.front(), sq.left.front()) && sq.right.size() > 5)
    sq.right.pop_front();
    else
      break;
  }

  //printf("SQ %d %d\n", sq.left.size(), sq.right.size());
}


//---------------------- CALLBACKS -----------------//

void left_image_callback(const sensor_msgs::Image &msg)
{
  //cout << "L " << msg.header.stamp << endl;
  
  stereo_queue.left.push_back(msg);
  stereo_queue_process(stereo_queue);
}

void right_image_callback(const sensor_msgs::Image &msg)
{
  //cout << "R " << msg.header.stamp << endl;

  stereo_queue.right.push_back(msg);
  stereo_queue_process(stereo_queue);
}


//---------------------- GET CAMERA MODEL --------------------//

void get_camera_model()
{
  ROS_INFO("Waiting for camera model...");

  sensor_msgs::CameraInfoConstPtr cinfo_left_ptr =
    ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/svstereo/left/camera_info");

  sensor_msgs::CameraInfoConstPtr cinfo_right_ptr =
    ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/svstereo/right/camera_info");

  stereo_camera_model.fromCameraInfo(cinfo_left_ptr, cinfo_right_ptr);

  ROS_INFO("Got camera model.");
}


void get_table()
{
  //double table_min_depth = 3.0;               // meters
  //double table_max_depth = 8.0;               // meters
  //double min_disparity = stereo_camera_model.getDisparity(table_max_depth);
  //double max_disparity = stereo_camera_model.getDisparity(table_min_depth);
  //printf("disparity range = [%.2f, %.2f]\n", min_disparity, max_disparity);

  // get table corners in 3D camera coordinates
  for (uint i = 0; i < 4; i++) {
    double x = table_corners_left[i][0];
    double y = (table_corners_left[i][1] + table_corners_right[i][1]) / 2.0;
    double disparity = table_corners_left[i][0] - table_corners_right[i][0];
    cv::Point2d p_image(x, y);
    stereo_camera_model.projectDisparityTo3d(p_image, disparity, table_corners[i]);
  }

  // get table coordinate frame (origin is at SW corner, X is east, Y is north, and Z is up)
  cv::Point3d &NW = table_corners[0];
  //cv::Point3d &NE = table_corners[1];
  cv::Point3d &SE = table_corners[2];
  cv::Point3d &SW = table_corners[3];
  table_camera_transform.setOrigin(tf::Vector3(SW.x, SW.y, SW.z));
  double **R = new_matrix2(3,3);
  R[0][0] = SE.x - SW.x;
  R[1][0] = SE.y - SW.y;
  R[2][0] = SE.z - SW.z;
  R[0][1] = NW.x - SW.x;
  R[1][1] = NW.y - SW.y;
  R[2][1] = NW.z - SW.z;
  R[0][2] = R[1][0]*R[2][1] - R[2][0]*R[1][1];
  R[1][2] = R[2][0]*R[0][1] - R[0][0]*R[2][1];;
  R[2][2] = R[0][0]*R[1][1] - R[1][0]*R[0][1];;
  double d[3];
  for (uint i = 0; i < 3; i++)
    d[i] = sqrt(R[0][i]*R[0][i] + R[1][i]*R[1][i] + R[2][i]*R[2][i]);
  for (uint i = 0; i < 3; i++)
    for (uint j = 0; j < 3; j++)
      R[i][j] /= d[j];
  double q[4];
  rotation_matrix_to_quaternion(q, R);
  free_matrix2(R);
  table_camera_transform.setRotation(tf::Quaternion(q[1], q[2], q[3], q[0]));
}


//----------------------- MAIN --------------------//

int main(int argc, char *argv[])
{
  // init ROS
  ros::init(argc, argv, "pingpongfindtable");
  ros::NodeHandle nh;

  get_camera_model();
  get_table();

  ros::Subscriber sub_left_image = nh.subscribe("/svstereo/left/image_rect", 10, left_image_callback);
  ros::Subscriber sub_right_image = nh.subscribe("/svstereo/right/image_rect", 10, right_image_callback);
  //ros::Subscriber sub_left_image = nh.subscribe("/svstereo/left/image_raw", 10, left_image_callback);
  //ros::Subscriber sub_right_image = nh.subscribe("/svstereo/right/image_raw", 10, right_image_callback);
  left_pub = nh.advertise<sensor_msgs::Image>("/svstereo/left/eyes", 1);
  right_pub = nh.advertise<sensor_msgs::Image>("/svstereo/right/eyes", 1);

  ros::spin();

  return 0;
}
