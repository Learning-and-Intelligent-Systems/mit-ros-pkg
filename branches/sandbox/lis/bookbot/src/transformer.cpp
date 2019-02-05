#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <iostream>

using namespace std;

tf::TransformListener *tfListener;
ros::Publisher pub;
int i = 0;

void callback(sensor_msgs::PointCloud message) {
  //cout << message.header.frame_id << endl;
  sensor_msgs::PointCloud newCloud;
  try {
    //tfListener->transformPointCloud("frame", message, newCloud); //change frame here
    tfListener->transformPointCloud("/base_footprint", message, newCloud);
		sensor_msgs::PointCloud2 newCloud2;
    cout << "Cloud #" << i++ << endl;
		sensor_msgs::convertPointCloudToPointCloud2(newCloud, newCloud2);
    pub.publish(newCloud2);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
  }	
}

void listener(ros::NodeHandle n) {
  //ros::Subscriber sub = n.subscribe("points", 10, callback);
  ros::Subscriber sub = n.subscribe("/narrow_stereo/points", 10, callback);
  ros::spin();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_transformer");
  ros::NodeHandle n;
  tfListener = new tf::TransformListener;
  pub = n.advertise<sensor_msgs::PointCloud2>("transformedPointCloud2", 10);
  //pub = n.advertise<sensor_msgs::PointCloud>("/points_in", 10);
  listener(n);
}

