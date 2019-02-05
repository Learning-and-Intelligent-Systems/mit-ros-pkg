/*
 * ros_kinect.h
 *
 *  Created on: Jul 5, 2012
 */

#ifndef ROS_KINECT_H_
#define ROS_KINECT_H_

#include "io.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl-1.6/pcl/pcl_base.h>
#include <pcl-1.6/pcl/point_types.h>
#include <pcl-1.6/pcl/ros/conversions.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace objrec {

class ros_kinect {
private:
	bool waiting_for_cloud;
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
//	camera_info info;
	scene_info info;
	ros::Subscriber cloud_sub;
	tf::TransformListener listener;
	std::string world_frame, camera_frame;
public:
	ros_kinect(
			const std::string & cloud_topic       = "/head_mount_kinect/depth_registered/points",
			const std::string & camera_info_topic = "/head_mount_kinect/depth/camera_info",
			const std::string & world_frame       = "/base_link",
			const std::string & camera_frame      = "/head_mount_kinect_rgb_link"
			) : waiting_for_cloud(false), world_frame(world_frame), camera_frame(camera_frame) {
		std::cout << "Connecting to the ROS kinect..." << std::flush;
		ros::NodeHandle n;
		//if a node was just started, there seem to be some problems immediately connecting to the camera, so
		//wait for a tenth of a second just in case this is a new node.
		ros::Rate r(10);
		r.sleep();

		double timeout = 10.0;
		sensor_msgs::PointCloud2ConstPtr pc =
				ros::topic::waitForMessage<sensor_msgs::PointCloud2>(cloud_topic, ros::Duration(timeout));
		if (pc == 0) {
			std::ostringstream err;
			err << "No message received from the topic: " << cloud_topic
					<< ", in " << timeout << " seconds, aborting." << std::endl;
			throw std::runtime_error(err.str());
		}
		pcl::fromROSMsg(*pc, cloud);

//		sensor_msgs::CameraInfoConstPtr ci =
//				ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic, ros::Duration(timeout));
//		if(!ci) {
//			std::ostringstream err;
//			err << "No message received from the topic: " << camera_info_topic
//					<< ", in " << timeout << " seconds, aborting." << std::endl;
//			throw std::runtime_error(err.str());
//		}
//		info.fx = ci->P[0];
//		info.fy = ci->P[5];
//		info.cx = ci->P[2];
//		info.cy = ci->P[6];
//		info.Tx = ci->P[3];
//		info.Ty = ci->P[7];
//		info.width = ci->width;
//		info.height = ci->height;

		cloud_sub = n.subscribe(cloud_topic, 1, &ros_kinect::cloud_callback, this);
		std::cout << " done." << std::endl;
	}

//	void get(cv::Mat_<cv::Vec3b>& image, cv::Mat_<float>& depth, scene_info& info) {
//		waiting_for_cloud = true;
//		while (waiting_for_cloud) ros::spinOnce();
//		image.create(cloud.height,cloud.width);
//		depth.create(cloud.height,cloud.width);
//		int ind = 0;
//		for (int imy = 0; imy < (int)cloud.height; imy++) {
//			for (int imx = 0; imx < (int)cloud.width; imx++, ind++) {
//				image(imy, imx) = cv::Vec3b(cloud.points[ind].b, cloud.points[ind].g, cloud.points[ind].r);
//				float x = cloud.points[ind].x;
//				float y = cloud.points[ind].y;
//				float z = cloud.points[ind].z;
//				depth(imy,imx) = sqrt(x*x+y*y+z*z);
//			}
//		}
//		info = this->info;
//	}
	void get(cv::Mat_<cv::Vec3b>& image, cv::Mat_<cv::Vec3f>& xyz, scene_info& info) {
		waiting_for_cloud = true;
		while (waiting_for_cloud) ros::spinOnce();
		image.create(cloud.height,cloud.width);
		xyz.create(cloud.height,cloud.width);
		int ind = 0;
		for (int imy = 0; imy < (int)cloud.height; imy++) {
			for (int imx = 0; imx < (int)cloud.width; imx++, ind++) {
				image(imy, imx) = cv::Vec3b(cloud.points[ind].b, cloud.points[ind].g, cloud.points[ind].r);
				float x = cloud.points[ind].x;
				float y = cloud.points[ind].y;
				float z = cloud.points[ind].z;
				xyz(imy,imx) = cv::Vec3f(x,y,z);
			}
		}
		info = this->info;
	}
private:
//	inline matrix3x3 to_matrix3x3(const tf::Matrix3x3 m) {
//		matrix3x3 r;
//		tf::Vector3 row1 = m.getRow(0); r.m11 = row1.x(); r.m12 = row1.y(); r.m13 = row1.z();
//		tf::Vector3 row2 = m.getRow(1); r.m21 = row2.x(); r.m22 = row2.y(); r.m23 = row2.z();
//		tf::Vector3 row3 = m.getRow(2); r.m31 = row3.x(); r.m32 = row3.y(); r.m33 = row3.z();
//		return r;
//	}
	void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& pc) {
		if (waiting_for_cloud) {
			pcl::fromROSMsg(*pc, cloud);
			tf::StampedTransform st;
			//bool success = false;
			//while(!success) {
			  try {
			    listener.waitForTransform(world_frame,camera_frame,/*pc->header.stamp*/ros::Time(0), ros::Duration(3.0));
			    listener.lookupTransform( world_frame,camera_frame,/*pc->header.stamp*/ros::Time(0), st);
			    //success = true;
			  } catch(const tf::ExtrapolationException& e) {}
			  //}
			
			tf::Vector3 origin = st.getOrigin();

//			info.pos.x = origin.x();
//			info.pos.y = origin.y();
//			info.pos.z = origin.z();
//			info.rot = to_matrix3x3(st.getBasis());
//			info.inv_rot = to_matrix3x3(st.getBasis().inverse());

			info.camera_height_above_floor = origin.z();
			tf::Vector3 row3 = st.getBasis().getRow(2);
			info.camera_tilt_angle = atan2(-row3.x(),sqrt(row3.y()*row3.y()+row3.z()*row3.z()))*180.0/M_PI;
			info.table_height = 0;

			waiting_for_cloud = false;
		}
	}
};

} //namespace objrec

#endif /* ROS_KINECT_H_ */
