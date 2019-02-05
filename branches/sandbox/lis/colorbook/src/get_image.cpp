#include <ros/ros.h>
#include <polled_camera/GetPolledImage.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <iostream>

#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/CvBridge.h>

using namespace std;

class Camera{
private:
  ros::NodeHandle *n_;
  std::string name_;
  bool polled_;
  bool enabled;

  ros::ServiceClient camera_trigger;

  ros::Subscriber image_sub;
  std::string image_topic_name;
  sensor_msgs::Image image_msg;
  ros::Time image_msg_received;

public:
  Camera(const std::string& name, ros::NodeHandle& n, bool polled=false):
    n_(&n),
    name_(name),
    enabled(true),
    polled_(polled)
  {

    if(polled_) {
      std::string serv = "/" + name + "/request_image";
      if(!ros::service::waitForService(serv,ros::Duration(2.0))) {
	ROS_WARN("Unable to connect to the polled camera service: %s, aborting.",serv.c_str());
	enabled = false;
	return;
      }
      camera_trigger = n_->serviceClient<polled_camera::GetPolledImage>(serv);
      name_ += "/polled_camera";
      }
    
    image_topic_name = "/"+name_+"/image_rect_color";
    ROS_INFO("Subscribing to %s",image_topic_name.c_str());
    image_sub = n_->subscribe(image_topic_name,1,&Camera::image_cb, this);

  }
  bool requestImage() {
    if(polled_ && enabled) {
      polled_camera::GetPolledImage srv;
      srv.request.response_namespace="polled_camera";
      srv.request.timeout = ros::Duration(5.0);
      if (!camera_trigger.call(srv)){
	cout << camera_trigger.call(srv) << endl;
	ROS_ERROR("Failed to call camera trigger service");
	return false;
      }
      ROS_INFO("Requested a polled image from %s",name_.c_str());
      return true;
    }
    else {
      return false;
    }
  }
  bool getImage(sensor_msgs::Image &image_result,
		ros::Time earliestReceived) {
    if(!enabled) {
      return false;
    }
    if(earliestReceived>image_msg_received && earliestReceived>image_msg.header.stamp) {
      sensor_msgs::ImageConstPtr image
	= ros::topic::waitForMessage<sensor_msgs::Image>(image_topic_name, ros::Duration(10.0));
      if(image==0) {
	ROS_WARN("Didn't receive a %s message for 10 seconds.",image_topic_name.c_str());
      }
      else {
	image_msg = *image;
	image_msg_received = ros::Time::now();
      }
    }
    image_result = image_msg;
    return true;
  }
private:
  void image_cb(const sensor_msgs::ImageConstPtr& image){
    cout << "callback" << endl;
    image_msg = *image;
    image_msg_received = ros::Time::now();
  }

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_getter");
  ros::NodeHandle n;
  Camera prosilica("prosilica", n, false);
  sensor_msgs::Image image;
  ros::Time start = ros::Time::now();
  prosilica.requestImage();
  prosilica.getImage(image, start);
  sensor_msgs::Image::ConstPtr imgptr = boost::make_shared<sensor_msgs::Image>(image);
  sensor_msgs::CvBridge bridge;
  IplImage* cvImage = bridge.imgMsgToCv(imgptr, (string) "passthrough");
  cvSaveImage("prosilica_images/test_image.jpg", cvImage);
}
