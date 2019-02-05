/*
 * surveyer.cpp
 *
 *  Created on: Oct 8, 2010
 *      Author: garratt
 */
#include "ros/ros.h"
//#include "geometry_msgs/PoseArray.h"
#include "polled_camera/GetPolledImage.h"
//#include <image_transport/image_transport.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_assembler/AssembleScans.h>
#include <tf/transform_listener.h>

#include <simple_controller/SimpleMoveAction.h>
#include <actionlib/client/simple_action_client.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include "object_survey/GetCameraScan.h"
#include <object_survey/SurveyResults.h>
#include <object_survey/CameraScan.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <dynamic_reconfigure/Reconfigure.h>
//pointcloud to pointcloud2 with viewpoint (stamped at each point)
int addViewPoint(sensor_msgs::PointCloud &cloud, tf::StampedTransform transform,sensor_msgs::PointCloud2 &output){
   sensor_msgs::ChannelFloat32 v1;
   v1.name="vp_x";
   v1.values=std::vector<float>(cloud.points.size(),transform.getOrigin().x());
   cloud.channels.push_back(v1);
   v1.name="vp_y";
   v1.values=std::vector<float>(cloud.points.size(),transform.getOrigin().y());
   cloud.channels.push_back(v1);
   v1.name="vp_z";
   v1.values=std::vector<float>(cloud.points.size(),transform.getOrigin().z());
   cloud.channels.push_back(v1);

   // Convert to the new point cloud format
   if (!sensor_msgs::convertPointCloudToPointCloud2 (cloud, output))
   {
     ROS_ERROR ("[cloud_grabber] Conversion from sensor_msgs::PointCloud to sensor_msgs::PointCloud2 failed!");
     return -1;
   }
   return 0;

}


class Camera{
private:
  ros::NodeHandle *n_;
  std::string name_;
  bool polled_;
  bool enabled;

  ros::ServiceClient camera_trigger;

  ros::Subscriber image_sub;
  std::string image_topic_name;
  ros::Subscriber info_sub;
  std::string info_topic_name;
  sensor_msgs::Image image_msg;
  ros::Time image_msg_received;
  sensor_msgs::CameraInfo info_msg;
  ros::Time info_msg_received;

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
    info_topic_name = "/"+name_+"/camera_info";
    ROS_INFO("Subscribing to %s",info_topic_name.c_str());
    info_sub = n_->subscribe(info_topic_name,1,&Camera::info_cb, this);
  }
  bool requestImage() {
    if(polled_ && enabled) {
      polled_camera::GetPolledImage srv;
      srv.request.response_namespace="polled_camera";
      if (!camera_trigger.call(srv)){
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
		sensor_msgs::CameraInfo &info_result,
		ros::Time earliestReceived) {
    if(!enabled) {
      return false;
    }

    if(earliestReceived>image_msg_received && earliestReceived>image_msg.header.stamp) {
      sensor_msgs::ImageConstPtr image
	= ros::topic::waitForMessage<sensor_msgs::Image>(image_topic_name,ros::Duration(2.0));
      if(image==0) {
	ROS_WARN("Didn't receive a %s message for 2 seconds.",image_topic_name.c_str());
      }
      else {
	image_msg = *image;
	image_msg_received = ros::Time::now();
      }
    }
    image_result = image_msg;

    if(earliestReceived>info_msg_received && earliestReceived>info_msg.header.stamp) {
      sensor_msgs::CameraInfoConstPtr info
	= ros::topic::waitForMessage<sensor_msgs::CameraInfo>(info_topic_name,ros::Duration(2.0));
      if(info==0) {
	ROS_WARN("Didn't receive a %s message for 2 seconds.",info_topic_name.c_str());
      }
      else {
	info_msg = *info;
	info_msg_received = ros::Time::now();
      }
    }
    info_result = info_msg;
    return true;
  }
private:
  void image_cb(const sensor_msgs::ImageConstPtr& image){
    image_msg = *image;
    image_msg_received = ros::Time::now();
  }
  void info_cb(const sensor_msgs::CameraInfoConstPtr& info){
    info_msg = *info;
    info_msg_received = ros::Time::now();
  }

};

#define RECEIVE_ONE_STEREO

class StereoCamera {
private:
  ros::NodeHandle *n_;
  std::string name_;
  std::string frame_;
  tf::TransformListener *listener_;
#ifndef RECEIVE_ONE_STEREO
  ros::Subscriber sub;
#endif
  sensor_msgs::PointCloud cloud_msg;
  ros::Time cloud_msg_received;
  std::string cloud_topic_name;
public:
  StereoCamera(const std::string& name, const std::string& frame, ros::NodeHandle& n, tf::TransformListener& listener):
    n_(&n), name_(name), frame_(frame),
    listener_(&listener)
  {
    cloud_topic_name = "/" + name_ + "/points";
#ifndef RECEIVE_ONE_STEREO
    ROS_INFO("Subscribing to %s",cloud_topic_name.c_str());
    sub = n_->subscribe(cloud_topic_name,1,&StereoCamera::cb, this);
#endif
  }

  bool getCloud(const std::string& worldframe,
		sensor_msgs::PointCloud2 &cloud_result,
		geometry_msgs::TransformStamped &tf_result,
		ros::Time earliestReceived
		) {

#ifdef RECEIVE_ONE_STEREO
    ROS_INFO("Subscribing to %s",cloud_topic_name.c_str());
    ros::Subscriber sub = n_->subscribe(cloud_topic_name,1,&StereoCamera::cb, this);
#endif

    tf::StampedTransform trans;
    listener_->lookupTransform(worldframe, frame_, ros::Time(0,0), trans);
    tf::transformStampedTFToMsg(trans,tf_result);


    if(earliestReceived>cloud_msg_received && earliestReceived>cloud_msg.header.stamp) {
      sensor_msgs::PointCloudConstPtr cloud
	= ros::topic::waitForMessage<sensor_msgs::PointCloud>(cloud_topic_name,ros::Duration(2.0));
      if(cloud==0) {
	ROS_WARN("Didn't receive a %s message for 2 seconds.",cloud_topic_name.c_str());
      }
      else {
	cloud_msg = *cloud;
	cloud_msg_received = ros::Time::now();
      }
    }


    addViewPoint(cloud_msg, trans, cloud_result);

    return true;
  }
private:
  void cb(const sensor_msgs::PointCloudConstPtr& msg_ptr){
    cloud_msg = *msg_ptr;
    cloud_msg_received = ros::Time::now();
  }
};


class CameraSurveyer{


private:
  tf::TransformListener listener;
  ros::NodeHandle* n_;
  ros::ServiceClient parameter_setter;

  Camera prosilica;
  Camera narrow_stereo_right, narrow_stereo_left;
  StereoCamera narrow_stereo;
  Camera wide_stereo_right, wide_stereo_left;
  StereoCamera wide_stereo;

  ros::Subscriber narrow_stereo_cloud;
  ros::Subscriber wide_stereo_cloud;

  ros::Subscriber highdef_caminfo, highdef_transform;
  
  ros::ServiceServer camera_service;

  ros::ServiceServer scan_service;
  actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> point_head_client;

  std::string worldframe;

public:
  CameraSurveyer(ros::NodeHandle& n):
    listener(ros::Duration(120.0)),
    prosilica("prosilica",n,true),
    narrow_stereo_right("narrow_stereo/right",n),
    narrow_stereo_left("narrow_stereo/left",n),
    narrow_stereo("narrow_stereo_textured","narrow_stereo_optical_frame",n,listener),
    wide_stereo_right("wide_stereo/right",n),
    wide_stereo_left("wide_stereo/left",n),
    wide_stereo("wide_stereo","wide_stereo_optical_frame",n,listener),
    point_head_client("/head_traj_controller/point_head_action", true)
  {
    n_ = &n;

    updateParameters();

    //start services
    parameter_setter = n_->serviceClient<dynamic_reconfigure::Reconfigure>("/camera_synchronizer_node/set_parameters");

    ROS_INFO("Waiting for services...");
    point_head_client.waitForServer();
    ros::service::waitForService("/camera_synchronizer_node/set_parameters");
    ROS_INFO("Services found.");

    scan_service = n_->advertiseService("get_camera_scan", &CameraSurveyer::scan_cb, this);
    ROS_INFO("Began advertising service get_camera_scan.");

  }

  void updateParameters(){
    n_->param("world_frame", worldframe, std::string("/odom_combined")); //TODO: change this back to "/world"
  }


  int pointHead(geometry_msgs::PointStamped ps){
    pr2_controllers_msgs::PointHeadGoal g;
    g.target = ps;
    g.min_duration = ros::Duration(0.5);
    point_head_client.sendGoal(g);
    bool result = point_head_client.waitForResult(ros::Duration(2.0));
    if(!result) {
      ROS_WARN("Pointing the head took a long time!");
      return 1;
    }
    sleep(.2);
    return 0;
  }

  int setProjector(bool enable){
     dynamic_reconfigure::Reconfigure srv;
     srv.request.config.ints.push_back(dynamic_reconfigure::IntParameter());
     srv.request.config.ints.back().name="projector_mode";
     if(enable) {
       srv.request.config.ints.back().value=3; //ProjectorOn
       srv.request.config.ints.push_back(dynamic_reconfigure::IntParameter());
       srv.request.config.ints.back().name="narrow_stereo_trig_mode";
       srv.request.config.ints.back().value=5; //"AlternateProjector"

       srv.request.config.ints.push_back(dynamic_reconfigure::IntParameter());
       srv.request.config.ints.back().name="wide_stereo_trig_mode";
       srv.request.config.ints.back().value=3; //"WithProjector"
     }
     else {
       srv.request.config.ints.back().value=1; //ProjectorOff
       srv.request.config.ints.push_back(dynamic_reconfigure::IntParameter());
       srv.request.config.ints.back().name="narrow_stereo_trig_mode";
       srv.request.config.ints.back().value=2; //"IgnoreProjector"

       srv.request.config.ints.push_back(dynamic_reconfigure::IntParameter());
       srv.request.config.ints.back().name="wide_stereo_trig_mode";
       srv.request.config.ints.back().value=2; //"IgnoreProjector"
     }
     
	
     if (parameter_setter.call(srv))
     {
       return 0;
     }
     else
     {
       ROS_ERROR("Failed to call service /camera_synchronizer_node/set_parameters");
       return 1;
     }
  }

  int takeCameraData(object_survey::CameraScan & scan)
  {
    setProjector(false);
    ros::Time start = ros::Time::now();
    prosilica.requestImage();
    narrow_stereo_right.getImage(scan.narrow_stereo_right_image, scan.narrow_stereo_right_caminfo,start);
    narrow_stereo_left.getImage(scan.narrow_stereo_left_image, scan.narrow_stereo_left_caminfo,start);
    wide_stereo_right.getImage(scan.wide_stereo_right_image, scan.wide_stereo_right_caminfo,start);
    wide_stereo_left.getImage(scan.wide_stereo_left_image, scan.wide_stereo_left_caminfo,start);
    prosilica.getImage(scan.highdef_image, scan.highdef_caminfo,start);
    tf::StampedTransform trans;
    listener.lookupTransform(worldframe, "/high_def_optical_frame", ros::Time(0,0), trans);
    tf::transformStampedTFToMsg(trans,scan.highdef_transform);

    setProjector(true);
    start = ros::Time::now();
    narrow_stereo.getCloud(worldframe, scan.narrow_stereo_cloud, scan.narrow_stereo_transform,start);
    wide_stereo.getCloud(worldframe, scan.wide_stereo_cloud, scan.wide_stereo_transform,start);
    setProjector(false);
    return 0;
  }

  bool scan_cb(object_survey::GetCameraScan::Request &req, object_survey::GetCameraScan::Response &res){
    ROS_INFO("GetCameraScan: Pointing head.");
    if(pointHead(req.focus_point)) return false;
    ROS_INFO("GetCameraScan: Taking camera data.");
    if(takeCameraData(res.scan)) return false;
    ROS_INFO("GetCameraScan: done.");
    return true;
  }

};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_surveyer");
  ros::NodeHandle n;
  CameraSurveyer surveyer(n);
  //ros::MultiThreadedSpinner spinner(4);
  //spinner.spin();
  ros::spin();
  return 0;
}
