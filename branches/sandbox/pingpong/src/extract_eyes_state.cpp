#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <pingpong/EyesState.h>


FILE *ball_state_fp;


void save_image(char *filename, const sensor_msgs::Image &image_msg)
{
  static sensor_msgs::CvBridge img_bridge;

  //printf("w = %d, h = %d\n", image_msg.width, image_msg.height); //dbug

  if (!img_bridge.fromImage(image_msg, "bgr8"))
    ROS_ERROR("Unable to convert %s image to bgr8", image_msg.encoding.c_str());
  IplImage *image = img_bridge.toIpl();
  if (image) {
    cvSaveImage(filename, image);
    ROS_INFO("Saved image %s", filename);
  }
}

void eyes_state_callback(const pingpong::EyesStateConstPtr& msg)
{
  double t_msg = msg->stamp.toSec();
  double t_ball = msg->ball_stamp.toSec();
  //printf("msg time = %.3f, ball time = %.3f, diff = %f\n", t_msg, t_ball, t_msg - t_ball);

  if (t_msg - t_ball > .0001)
    return;

  // save ball images
  static int cnt = 0;
  char filename[128];
  sprintf(filename, "left%04d.png", cnt);
  save_image(filename, msg->left.ball_image);
  sprintf(filename, "right%04d.png", cnt);
  save_image(filename, msg->right.ball_image);
  cnt++;

  // save ball timestamp and positions
  fprintf(ball_state_fp, "%f %f %f %f %f %f %f\n", t_ball, msg->ball_pos_cam.x, msg->ball_pos_cam.y, msg->ball_pos_cam.z,
	  msg->ball_pos_table.x, msg->ball_pos_table.y, msg->ball_pos_table.z);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "extract_eyes_state", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("eyes_state", 100, eyes_state_callback);

  printf("Opening ball_state.txt...");
  ball_state_fp = fopen("ball_state.txt", "w");
  if (ball_state_fp == NULL) {
    ROS_ERROR("Failed to open ball_state.txt.\n");
    return -1;
  }
  printf("done.\n");

  ros::spin();

  printf("Closing ball_state...");
  fclose(ball_state_fp);
  printf("done.\n");

  return 0;
}
