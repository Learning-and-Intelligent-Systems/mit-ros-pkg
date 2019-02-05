#include <iostream>

#include <ros/ros.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

using namespace std;


int main(int argc, char **argv)
{
  // init ROS
  ros::init(argc, argv, "tracker");
  ros::NodeHandle nh;
  ros::Publisher pub_downsampled;
  pub_downsampled = nh.advertise<sensor_msgs::PointCloud2>("downsampled", 1);
  tf::TransformListener tfListener;

  // Point clouds
  sensor_msgs::PointCloud new_cloud;
  sensor_msgs::PointCloud2 new_cloud2;

  // PCL objects
  pcl::PointCloud<pcl::PointXYZ> cloud, cloud_downsampled, model, registered_model;
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  grid.setFilterFieldName ("z");
  grid.setLeafSize (0.01, 0.01, 0.01);
  grid.setFilterLimits (0.4, 1.6);
  //pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp;
  
  //////////////////////////////////////////////////////////////////
  //playing with parameters (the same we used with bookbot):
  icp.setRANSACOutlierRejectionThreshold (2.0);
  icp.setMaximumIterations(200);
  //////////////////////////////////////////////////////////////////

  cout << "Initialized node" << endl;

  int nscans = 0;

  while (nh.ok ()) {

    // Spin until we get a PointCloud message
    sensor_msgs::PointCloudConstPtr cloud_blob_ptr =
      ros::topic::waitForMessage<sensor_msgs::PointCloud>("/narrow_stereo_textured/points");
    nscans++;

    cout << "Received point cloud with " << cloud_blob_ptr->points.size() << " points" << endl;

    // Transform point cloud to base_footprint coordinates
    try {
      tfListener.transformPointCloud("/base_footprint", *cloud_blob_ptr, new_cloud);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
    }	

    cout << "Transformed point cloud to /base_footprint" << endl;

    // Convert to PCL
    sensor_msgs::convertPointCloudToPointCloud2(new_cloud, new_cloud2);
    pcl::fromROSMsg(new_cloud2, cloud);

    cout << "Converted to PCL" << endl;

    // Downsample the data    
    grid.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud));
    grid.filter (cloud_downsampled);
		
    cout << "Downsampled point cloud to " << cloud_downsampled.points.size() << " points" << endl;	

    // Cluster points in XY space
    //pcl::PointCloud<pcl::PointXYZ> 
    //cloud_downsampled.points[]


    // Register the model with the current scan
    if (nscans == 3) {
      model = cloud_downsampled;
      //registered_model = model;
    }
    else if (nscans > 3) {
      icp.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(model));
      icp.setInputTarget (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud_downsampled));
      icp.align (registered_model);
      model = registered_model;
    }
    //model = registered_model; <- if model is outside the else-if, it will always change to empty point cloud

    // Publish the data
    pcl::toROSMsg(model, new_cloud2);
    pub_downsampled.publish(new_cloud2);

  }
}
