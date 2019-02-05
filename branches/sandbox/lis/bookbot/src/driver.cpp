#include <ros/ros.h>

#include <bookbot/BBox.h>
#include <bookbot/Book.h>
#include <bookbot/BookList.h>
#include <bookbot/PullStatus.h>
#include <bookbot/Table.h>

#include <tf/transform_listener.h>

#include <iostream>

#include <object_manipulation_msgs/GraspableObject.h>

#include "pcl/filters/passthrough.h"
#include "pcl/point_types.h"
//#include "pcl_visualization/pcl_visualizer.h"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <cv.h>
#include <highgui.h>

using namespace std;
using namespace cv;

ros::Publisher tablePub;
ros::Publisher graspablePub;
ros::Publisher graspablePub2;
bookbot::Table table;

bool tableSent = false;
bool descriptionReceived = false;
bool planeFound = true;
bool pullDone;
bool pickDone;
bool pclRcvd = false;
bool booksLeft = true;
bookbot::Book chosenBook;
sensor_msgs::PointCloud2 pCloud2;
sensor_msgs::PointCloud pCloud;
sensor_msgs::PointCloud pCloud12;

tf::TransformListener *tfListener;

void readTableCoordinates() {
  table.corners[0].x = 0.43; table.corners[0].y = -0.58; table.corners[0].z = 0.67;
  table.corners[1].x = 1.03; table.corners[1].y = -0.58; table.corners[1].z = 0.67;
  table.corners[2].x = 1.03; table.corners[2].y = 0.61; table.corners[2].z = 0.67;
  table.corners[3].x = 0.43; table.corners[3].y = 0.61; table.corners[3].z = 0.67;
}

void filter(pcl::PointCloud<pcl::PointXYZ> &cloud, char* axis, float lowerLimit, float upperLimit) {
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud));
  pass.setFilterFieldName(axis);
  pass.setFilterLimits (lowerLimit, upperLimit);
  pass.filter(cloud);
}

void callback(sensor_msgs::PointCloud message) {
  pCloud = message;
	pclRcvd = true;
}

void pullCallback(bookbot::PullStatus s) {
	pullDone = true;
}

void pickCallback(bookbot::PullStatus s) {
	pickDone = true;
}

void bookManipulation(ros::NodeHandle n) {
  ROS_INFO("Filtering out the book");
	bool pulled = false;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	object_manipulation_msgs::GraspableObject graspable;
	graspable.type = graspable.POINT_CLUSTER;
	ros::Rate rate(10);
	while (ros::ok() && !pulled) {
		pullDone = false;
		//ros::Subscriber pclSub = n.subscribe("transformedPointCloud2", 10, callback);
		ros::Subscriber pclSub = n.subscribe("/narrow_stereo_textured/points", 1, callback);
		while (ros::ok() && !pclRcvd) {
		  ros::spinOnce();
		}
		pclSub.shutdown();
		try {
			tfListener->transformPointCloud("/base_link", pCloud, pCloud);
			sensor_msgs::convertPointCloudToPointCloud2(pCloud, pCloud2);
			pcl::fromROSMsg(pCloud2, cloud);
			/*pcl_visualization::PCLVisualizer vis;	
		 	vis.addCoordinateSystem();
			vis.addPointCloud(cloud);	
			vis.spin();
			vis.removePointCloud();*/
			cout << chosenBook << endl;
			filter(cloud, (char*) "x", 0, (chosenBook.cover.lowerLeft.point.x + chosenBook.cover.lowerRight.point.x)/2 + 0.13);
			filter(cloud, (char*) "y", chosenBook.cover.lowerRight.point.y, chosenBook.cover.lowerLeft.point.y);
			filter(cloud, (char*) "z", chosenBook.cover.lowerLeft.point.z + 0.02, chosenBook.cover.upperLeft.point.z + 0.01);
			/*vis.addPointCloud(cloud);	
			vis.spin();
			vis.removePointCloud();*/
			//cout << cloud.points.size() << endl;
			//convert to sensor_msgs PointCloud
			pcl::toROSMsg(cloud, pCloud2);
			sensor_msgs::convertPointCloud2ToPointCloud(pCloud2, pCloud);
			graspable.cluster = pCloud;
		
			ros::Subscriber pullPickSub = n.subscribe("pullStatus", 10, pullCallback); //check with Huan the name of the node
			while (ros::ok() && !pullDone) {	
				graspablePub.publish(graspable);
				ros::spinOnce();
				//rate.sleep();
			}
			ROS_INFO("Huan's message received");
			pclSub = n.subscribe("/narrow_stereo_textured/points", 1, callback);
			pclRcvd = false;
			sleep(1);
			while (ros::ok() && !pclRcvd) {
				ros::spinOnce();
			}
			cout << pCloud.header << endl;
			try {
				tfListener->transformPointCloud("/base_link", pCloud, pCloud);
				sensor_msgs::convertPointCloudToPointCloud2(pCloud, pCloud2);
				pcl::fromROSMsg(pCloud2, cloud);
				/*pcl_visualization::PCLVisualizer vis;	
			 	vis.addCoordinateSystem();
				vis.addPointCloud(cloud);
				vis.spin();
				vis.removePointCloud();*/
				filter(cloud, (char*) "x", 0, (chosenBook.cover.lowerLeft.point.x + chosenBook.cover.lowerRight.point.x)/2 - 0.01);
				filter(cloud, (char*) "y", chosenBook.cover.lowerRight.point.y, chosenBook.cover.lowerLeft.point.y);
				filter(cloud, (char*) "z", chosenBook.cover.lowerLeft.point.z + 0.02, chosenBook.cover.upperLeft.point.z + 0.01);
				pulled = cloud.points.size() > 500;
				cout << "Filtered size: " << cloud.points.size() << endl;
				/*vis.addPointCloud(cloud);	
				vis.spin();
				vis.removePointCloud();*/
			} catch (tf::TransformException ex) {
				cout << "Error in inner TF" << endl;
			  ROS_ERROR("%s",ex.what());
			}
		} catch (tf::TransformException ex) {
			cout << "Error in outer TF" << endl;
	    ROS_ERROR("%s",ex.what());
	  }
		
	}
	pickDone = false;
	pcl::toROSMsg(cloud, pCloud2);
	sensor_msgs::convertPointCloud2ToPointCloud(pCloud2, pCloud);
	graspable.cluster = pCloud;
	ros::Subscriber pullPickSub = n.subscribe("pickStatus", 10, pickCallback);
	while (ros::ok() && !pickDone) {	
	  graspablePub2.publish(graspable);
	  ros::spinOnce();
	}
}

bool bookCmp(bookbot::Book b1, bookbot::Book b2) {
  //return b1.cover.lowerLeft.y < b2.cover.lowerLeft.y;
	return b1.id < b2.id;
}

void bookCallback(bookbot::BookList booklist) {
	ROS_INFO("Booklist received");
  descriptionReceived = true;
	if (booklist.oprah.size()==0) {
		ROS_INFO("Whoops, no books found");
		booksLeft = false;
	} else {
	  sort(booklist.oprah.begin(), booklist.oprah.end(), bookCmp);
		cout << "Choose the book to grasp. Available books:" << endl;
		for (int i = 0; i < (int) booklist.oprah.size()-1; i++) {
		  cout << booklist.oprah[i].id << ", ";
		}
		cout << booklist.oprah[booklist.oprah.size()-1].id << endl;
		int choice;
		bool found = false;
		while (ros::ok() && !found) {
		  cin >> choice;
		  for (int i = 0; i < (int) booklist.oprah.size() && !found; i++) {
		    if (booklist.oprah[i].id == choice) {
					found = true;
					chosenBook = booklist.oprah[i];
					/*chosenBook.cover.lowerLeft.header.stamp = ros::Time::now() - ros::Duration(1.0);
					chosenBook.cover.lowerRight.header.stamp = ros::Time::now() - ros::Duration(1.0);
					chosenBook.cover.upperLeft.header.stamp = ros::Time::now() - ros::Duration(1.0);
					chosenBook.cover.upperRight.header.stamp = ros::Time::now() - ros::Duration(1.0);
					try {
						cout << "Transforming" << endl;
						tfListener->transformPoint("/base_link", chosenBook.cover.lowerLeft, chosenBook.cover.lowerLeft);
						tfListener->transformPoint("/base_link", chosenBook.cover.lowerRight, chosenBook.cover.lowerRight);
						tfListener->transformPoint("/base_link", chosenBook.cover.upperLeft, chosenBook.cover.upperLeft);
						tfListener->transformPoint("/base_link", chosenBook.cover.upperRight, chosenBook.cover.upperRight);
					} catch (tf::TransformException ex) {
						cout << "Error in bookCallback" << endl;
						ROS_ERROR("%s",ex.what());
					}*/
					return;
		  	}
		  }
		  cout << "Book not available, try again." << endl;
		}
		//chosenBook = booklist.oprah[0];
	}
}

void tableCallback(bookbot::BBox message) {
  tableSent = true;
  planeFound = message.success;
}

void bringTheBook(ros::NodeHandle n) {
  while (ros::ok() && booksLeft) {
    //readBookOrientation();
    readTableCoordinates();
    //move_base_msgs::MoveBaseGoal goal;
    //setGoal(goal);
    //driveRobot(goal);
    ros::Subscriber tableSub = n.subscribe("bookBox", 1, tableCallback);
    ros::Rate rate(10);
    while(!tableSent && ros::ok()) {
      tablePub.publish(table);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("Table sent");
    if (planeFound) {
      ROS_INFO("Plane found");
      ros::Subscriber bookSub = n.subscribe("books", 1, bookCallback);
      //descriptionReceived = true;
      while (!descriptionReceived && ros::ok()) {	
				ros::spinOnce();
				rate.sleep();
      }
      bookManipulation(n);
    }
    else {
      ROS_INFO("Plane not found");
    }
  }
}	

int main(int argc, char** argv) {
  ros::init(argc, argv, "bookbot_driver");
  ros::NodeHandle n;
  tfListener = new tf::TransformListener;
  tablePub = n.advertise<bookbot::Table>("table", 10);
  graspablePub = n.advertise<object_manipulation_msgs::GraspableObject>("objectToPull", 10);
	graspablePub2 = n.advertise<object_manipulation_msgs::GraspableObject>("objectToPick", 10);
  bringTheBook(n);
}
