//============================================================================
// Name        : mapper.cpp
// Author      : Will Grathwohl
// Version     : 1.0
// Copyright   : N/A
// Description : Attempt to map world using a simple wandering robot and octomap
//============================================================================
#include <octomap/octomap.h>
#include <octomap_ros/OctomapROS.h>
#include <octomap/OcTreeLabeled.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_tools/pcl_utils.h>
#include <pcl_tools/segfast.hpp>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>



class camera {
	private:

		ros::NodeHandle n_;
		ros::Publisher cloudpub;
		ros::Subscriber sub_;
		octomap::OcTreeROS mapTree;
		pcl::PointXYZ origin;
		tf::TransformListener tl_;
		float tfY, tfX,tfZ;

	public:
		camera():mapTree(.1),tl_(ros::Duration(10.0)){
		cloudpub = n_.advertise<sensor_msgs::PointCloud2> ("cloudout", 1);
		sub_ = n_.subscribe("/camera/rgb/points", 1, &camera::cloudcb, this);
		origin.x = 0;
		origin.y = 0;
		origin.z = 0;
		tfY = -.124;
		tfX = 0;
		tfZ = .509;
		}


	void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan){
		timeval t0 = g_tick();
		pcl::PointXYZ origin;
		tf::StampedTransform trans, trans2;
		if(!tl_.waitForTransform("/odom",scan->header.frame_id,scan->header.stamp,ros::Duration(.5))){
				ROS_INFO("tf failed");
		        return;
		}
		tl_.lookupTransform(scan->header.frame_id,"/odom",scan->header.stamp, trans);
		tl_.lookupTransform("/base_link","/odom",scan->header.stamp, trans2);
		btVector3 tempVector = trans.getOrigin();
		origin.x = tempVector.getX();
		origin.y = tempVector.getY();
		origin.z = tempVector.getZ();
		btQuaternion rot = trans.getRotation();
		std::cout<< rot.getAxis().getX()<< " "<< rot.getAxis().getY()<< " "<< rot.getAxis().getZ()<< " "<< rot.getAngle()<<std::endl;
		sensor_msgs::PointCloud2 cloudFinal;
		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::fromROSMsg(*scan,cloud);
		mapTree.insertScan(cloud, pcl::PointXYZ(0.,0.,0.),origin, trans2.getRotation());
		pcl::toROSMsg(cloud,cloudFinal);
		cloudFinal.header=scan->header;
		cloudpub.publish(cloudFinal);

	}
	void write(){
		mapTree.octree.writeBinary("map_of_space.bt");
	}
};


int main(int argc, char** argv){
	ros::init(argc, argv, "BiliMapper");
	ros::NodeHandle n;
	camera cam;
	ros::spin();
	cam.write();
	return 0;
}


