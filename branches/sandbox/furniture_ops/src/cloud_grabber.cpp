#include <ros/ros.h>

#include <boost/thread.hpp>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include <string>
#include <iostream>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_assembler/AssembleScans.h>
#include <tf/transform_listener.h>



void addViewPoint(sensor_msgs::PointCloud &cloud, tf::StampedTransform transform){
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

}





using namespace std;
int main (int argc, char** argv)
{
	ros::init(argc, argv, "cloud_grabber");
	ros::NodeHandle nh_;
	cout<<"waiting for assemble_scans service..."<<endl;
	ros::service::waitForService("assemble_scans");

	cout<<"launching grabber..."<<endl;
    boost::mutex m_mutex_;
    ros::Publisher pub_points_, pub_points2_;
    tf::TransformListener listener(ros::Duration(2.0));

    int queue_size_(100);
    string  points_out_("/points_out"), points2_out_("/points2_out");
	pub_points_ = nh_.advertise<sensor_msgs::PointCloud> (points_out_, queue_size_);
	pub_points2_ = nh_.advertise<sensor_msgs::PointCloud2> (points2_out_, queue_size_);
	ros::ServiceClient assembler = nh_.serviceClient<laser_assembler::AssembleScans>("assemble_scans");

	laser_assembler::AssembleScans srv;
	srv.request.begin = ros::Time(0,0);
	srv.request.end   = ros::Time::now();
	if (assembler.call(srv))
		printf("Got cloud with %u points\n", (uint)srv.response.cloud.points.size());
	else{
		printf("Service call failed\n");
		return 0;
	}

	//Add viewpoint to cloud:
	//add it into PointCloud as a channel, will propogate through
	tf::StampedTransform transform;
	sleep(1);
	listener.lookupTransform(srv.response.cloud.header.frame_id, "/laser_tilt_mount_link", ros::Time(0,0), transform);

	//add the viewpoint as channels:
	addViewPoint(srv.response.cloud,transform);

	sensor_msgs::PointCloud2 output;
	// Convert to the new point cloud format
	if (!sensor_msgs::convertPointCloudToPointCloud2 (srv.response.cloud, output))
	{
	  ROS_ERROR ("[cloud_grabber] Conversion from sensor_msgs::PointCloud to sensor_msgs::PointCloud2 failed!");
	  return 0;
	}
	ROS_DEBUG ("[cloud_grabber] Publishing a PointCloud2 with %d points on %s.", output.height * output.width, nh_.resolveName (points2_out_).c_str ());
	pub_points2_.publish (output);
	pub_points_.publish (srv.response.cloud);

	cout<<"save as?:"<<endl;
	string filename;

	cin>>filename;

	if(filename.length()>3){
		cout<<"saving as: "<<filename<<endl;
		pcl::PointCloud<pcl::PointWithViewpoint> pcloud;
		pcl::fromROSMsg (output, pcloud);
		pcl::io::savePCDFileASCII (filename, pcloud);
		cout<<"frame: "<<output.header.frame_id<<endl;
		ROS_INFO ("Saved %d data points to %s.", (int)pcloud.points.size (),filename.c_str());
	}
	else
		cout<<"Ok, Quitting. "<<endl;

  return (0);
}

