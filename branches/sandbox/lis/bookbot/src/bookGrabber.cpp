#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <bookbot/GrabberAction.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include "pcl/filters/passthrough.h"
#include "pcl/point_types.h"
#include "pcl_visualization/pcl_visualizer.h"

using namespace std;

sensor_msgs::PointCloud2 pCloud2;

void callback(sensor_msgs::PointCloud2 message) {
	pCloud2 = message;
}


class GrabberAction {
protected:
	ros::NodeHandle n;
	actionlib::SimpleActionServer<bookbot::GrabberAction> as;
	std::string action_name;
	bookbot::GrabberFeedback feedback;
	bookbot::GrabberResult result;


public:
  GrabberAction(std::string name):
	  as(n, name, boost::bind(&GrabberAction::executeCB, this, _1)), action_name(name) {
  }

  ~GrabberAction(void) {
  }

	void filter(pcl::PointCloud<pcl::PointXYZ> &cloud, char* axis, float lowerLimit, float upperLimit) {
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud));
		pass.setFilterFieldName(axis);
		pass.setFilterLimits (lowerLimit, upperLimit);
		pass.filter(cloud);
	}

  void executeCB(const bookbot::GrabberGoalConstPtr &goal) {
		//grabbing the book action goes here
		//goal->book.cover.lowerLeft.point.x, goal->book.cover.lowerLeft.point.y, goal->book.cover.lowerLeft.point.z
		//goal->book.cover.lowerRight
		//goal->book.cover.upperLeft
		//goal->book.cover.upperRight
		ROS_INFO("Working on grabbing the book...");
		bool pulled = false;
		while (ros::ok() && !pulled) {
			//Code for pulling the book goes here

			//now filter out the points
			ros::Subscriber pclSub = n.subscribe("/points2_out", 10, callback);
			//ros::Subscriber pclSub = n.subscribe("transformedPointCloud2", 10, callback);
			bool pclRcvd = false;
			while (ros::ok() && !pclRcvd) {
				ros::spinOnce();
			}
		
			pcl::PointCloud<pcl::PointXYZ> cloud;
			pcl::fromROSMsg(pCloud2, cloud);
			pcl_visualization::PCLVisualizer vis;	
			vis.addCoordinateSystem();
			vis.addPointCloud(cloud);	
			vis.spin();
			vis.removePointCloud();
			filter(cloud, (char*) "x", 0, (goal->book.cover.lowerLeft.point.x + goal->book.cover.lowerRight.point.x)/2 - 0.01);
			filter(cloud, (char*) "y", goal->book.cover.lowerLeft.point.y, goal->book.cover.lowerRight.point.y);
			filter(cloud, (char*) "z", goal->book.cover.lowerLeft.point.z + 0.01, goal->book.cover.upperLeft.point.z + 0.01);
			vis.addPointCloud(cloud);	
			vis.spin();
			vis.removePointCloud();
			pulled = cloud.points.size() > 4000;
			cout << cloud.points.size() << endl;
		}

		//Grabbing the PointCloud goes here
		result.result = true;
		as.setSucceeded(result);
	}
};


int main(int argc, char** argv) {
	ros::init(argc, argv, "bookGrabber");
	GrabberAction grabber(ros::this_node::getName());
	ros::spin();
}
	
