/*
 * cluster_proc.cpp
 *
 *  Created on: Oct 19, 2010
 *      Author: garratt
 */


#include <ros/ros.h>
#include <ros/names.h>
#include <ros/master.h>
#include <ros/this_node.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <image_geometry/pinhole_camera_model.h>
#include <boost/foreach.hpp>

#include "image_proc/processor.h"

#include <algorithm>
#include "siftviewer/ProcessImageCluster.h"
#include "siftviewer/ProcessImage.h"
#include <posedetection_msgs/Feature0DDetect.h>
#include <image_geometry/pinhole_camera_model.h>

//#include "pcl/kdtree/kdtree.h"
//#include <boost/thread/mutex.hpp>
#include <flann/flann.h>

#include <flann/flann.hpp>
#include <flann/io/hdf5.h>

#include <sys/time.h>


//this node just takes a raw image and rectifies it and finds the sift features



class ImageProcNode
{
private:
  ros::NodeHandle n_;


  // Processing state (OK for these to be members in single-threaded case)
  image_proc::Processor processor_;
  image_geometry::PinholeCameraModel model_;
  image_proc::ImageSet processed_images_;
  ros::ServiceServer cluster_service, image_service;
  ros::ServiceClient imagesifter;

public:

  ImageProcNode()
  {
    // Advertise outputs
	  cluster_service = n_.advertiseService("/cluster_proc", &ImageProcNode::cluster_cb, this);
	  image_service = n_.advertiseService("/image_proc", &ImageProcNode::image_cb, this);
	  imagesifter = n_.serviceClient<posedetection_msgs::Feature0DDetect>("Feature0DDetect");
  }

  bool cluster_cb(siftviewer::ProcessImageCluster::Request &req, siftviewer::ProcessImageCluster::Response &res){
	sensor_msgs::ImageConstPtr p;
	// Update the camera model
	model_.fromCameraInfo(req.caminfo);
	int flag = image_proc::Processor::RECT_COLOR;
	//convert all the images to rectified color:
	std::vector<sensor_msgs::Image> rect_images(req.raw_images.size());
	for(uint i=0;i<req.raw_images.size();i++){
	   ROS_INFO("Converting image %d out of %d",i,req.raw_images.size());
		// Process raw image into colorized and/or rectified outputs
		if (!processor_.process(boost::make_shared< sensor_msgs::Image const>(req.raw_images[i]), model_, processed_images_, flag))
		  return false;

		rect_images[i].header.stamp = req.raw_images[i].header.stamp;
		rect_images[i].header.frame_id = req.raw_images[i].header.frame_id;
		fillImage(rect_images[i], processed_images_.color_encoding,
				processed_images_.rect_color.rows,
				processed_images_.rect_color.cols,
				processed_images_.rect_color.step,
				const_cast<uint8_t*>(processed_images_.rect_color.data));
	}

	//now find sift features for the images:
	std::vector< std::vector<int> > matches(rect_images.size());
	//   std::vector<posedetection_msgs::Feature0D> features(rect_images.size());
	res.features.resize(rect_images.size());
	for(uint i=0;i<rect_images.size();i++){
	   posedetection_msgs::Feature0DDetect srv;
	   srv.request.image=rect_images[i];
	   if (!imagesifter.call(srv)){
		 ROS_ERROR("Failed to call imagesift service");
		 return false;
	   }
	  res.features[i]=srv.response.features;
	}

	return true;
  }

  bool image_cb(siftviewer::ProcessImage::Request &req, siftviewer::ProcessImage::Response &res){
	sensor_msgs::ImageConstPtr p;
    // Update the camera model
    model_.fromCameraInfo(req.caminfo);
    int flag = image_proc::Processor::RECT_COLOR;
    //convert the image to rectified color:
    sensor_msgs::Image rect_image;
	if (!processor_.process(boost::make_shared< sensor_msgs::Image const>(req.raw_image), model_, processed_images_, flag))
	  return false;
	rect_image.header.stamp = req.raw_image.header.stamp;
	rect_image.header.frame_id = req.raw_image.header.frame_id;
	fillImage(rect_image, processed_images_.color_encoding,
			processed_images_.rect_color.rows,
			processed_images_.rect_color.cols,
			processed_images_.rect_color.step,
			const_cast<uint8_t*>(processed_images_.rect_color.data));

	//now find sift features for the images:
	posedetection_msgs::Feature0DDetect srv;
	srv.request.image=rect_image;
	if (!imagesifter.call(srv)){
	 ROS_ERROR("Failed to call imagesift service");
	 return false;
	}
	res.features=srv.response.features;
	return true;
  }


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cluster_proc", ros::init_options::AnonymousName);

  ImageProcNode proc;

  ros::spin();
  return 0;
}
