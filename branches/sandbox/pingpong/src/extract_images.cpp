#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>


sensor_msgs::CvBridge img_bridge_;


void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
  static int cnt = 0;

  if (!img_bridge_.fromImage(*msg, "bgr8"))
    ROS_ERROR("Unable to convert %s image to bgr8", msg->encoding.c_str());

  IplImage *image = img_bridge_.toIpl();
  if (image) {
    char filename[128];
    sprintf(filename, "frame%04d.png", cnt++);
    cvSaveImage(filename, image);
    ROS_INFO("Saved image %s", filename);
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "extract_images", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  std::string topic = nh.resolveName("image");
  if (topic == "/image") {
    ROS_WARN("extract_images: image has not been remapped! Typical command-line usage:\n"
             "\t$ ./extract_images image:=<image topic> [transport]");
  }

  ros::Subscriber sub = nh.subscribe(topic, 100, image_cb);

  ros::spin();

  return 0;
}
