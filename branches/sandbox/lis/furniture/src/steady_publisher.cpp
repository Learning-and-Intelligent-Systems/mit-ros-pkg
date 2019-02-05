#include <iostream>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

using namespace std;

bool compareTransforms(geometry_msgs::TransformStamped a, geometry_msgs::TransformStamped b) {
  return true; // TODO
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "steady_publisher");
  ros::NodeHandle nh;
  ros::Publisher pub_pointcloud;
  pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("steady_pointcloud", 1);

  tf::TransformListener tfListener;
  tf::StampedTransform transform;

  geometry_msgs::TransformStamped prevTransform;
  geometry_msgs::TransformStamped currTransform;

  cout << "Initialized steady_publisher node" << endl;

  try {
    tfListener.lookupTransform("/odom_combined", "/base_footprint", ros::Time(0), transform);
    transformStampedTFToMsg(transform, prevTransform);

    while (ros::ok()) {
      sensor_msgs::PointCloudConstPtr cloud_blob_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud>("/points");
      try {
	tfListener.lookupTransform("/odom_combined", "/base_footprint", ros::Time(0), transform);
	transformStampedTFToMsg(transform, currTransform);
	if (compareTransforms(prevTransform, currTransform)) {
	  pub_pointcloud(*cloud_blob_ptr); // Crashes here!
	  cout << "Published point cloud with " << cloud_blob_ptr->points.size() << " points" << endl;
	} else {
	  ROS_WARN("The robot was moving during the recording of point cloud. Message disregarded");
	}
	prevTransform = currTransform;
      } catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
      }
    }
  } catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
}
