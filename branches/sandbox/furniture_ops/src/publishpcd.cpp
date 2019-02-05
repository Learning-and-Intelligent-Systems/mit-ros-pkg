#include <ros/ros.h>

#include <boost/thread.hpp>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include <string>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>



using namespace std;
int
  main (int argc, char** argv)
{
	ros::init(argc, argv, "from_pcd");
	ros::NodeHandle nh_;

	string filename;
	if(argc>1){
		filename=argv[1];
	}

	sensor_msgs::PointCloud2 cloud_blob;
	pcl::PointCloud<pcl::PointXYZ> cloud;

	if (pcl::io::loadPCDFile (filename, cloud_blob) == -1)
	{
	ROS_ERROR ("Couldn't read file test_pcd.pcd");
	return (-1);
	}
	ROS_INFO ("Loaded %d data points from test_pcd.pcd with the following fields: %s", (int)(cloud_blob.width * cloud_blob.height), pcl::getFieldsList (cloud_blob).c_str ());


     ros::Publisher pub_points2_ = nh_.advertise<sensor_msgs::PointCloud2> ("from_pcd", 100);
     for(int i=0;i<3;i++){
    	 cout<<"publishing..."<<endl;
    	 cloud_blob.header.frame_id="/odom_combined";
		 pub_points2_.publish (cloud_blob);
		 sleep(1);
     }
  return (0);
}

