#define EIGEN_DONT_ALIGN 1

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <bookbot/BBox.h>
#include <bookbot/Table.h>
#include <cmath>
//#include <ctime>
//#include <fstream>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <string>

#include "pcl/filters/passthrough.h"
//#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
//#include "pcl/filters/extract_indices.h"
//#include "pcl_visualization/pcl_visualizer.h"


using namespace std;

ros::Publisher pub;
int ctr = 0;
bool written = false;
tf::TransformListener *tfListener;
bookbot::Table table;

const double PI = 3.14159;

void transformPointXYZ(pcl::PointXYZ &point, double angle, geometry_msgs::Point newOrigin) {
   point.x = point.x - newOrigin.x;
   point.y = point.y - newOrigin.y;
   point.x = point.x*cos(angle)+point.y*sin(angle);
   point.y = point.y*cos(angle)-point.x*sin(angle);
}
void transformPoint(geometry_msgs::PointStamped &point, double angle, geometry_msgs::Point newOrigin) {
   point.point.x = point.point.x - newOrigin.x;
   point.point.y = point.point.y - newOrigin.y;
   point.point.x = point.point.x*cos(angle)+point.point.y*sin(angle);
   point.point.y = point.point.y*cos(angle)-point.point.x*sin(angle);
}

void filter(pcl::PointCloud<pcl::PointXYZ> &cloud, char* axis, float lowerLimit, float upperLimit) {
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud));
   pass.setFilterFieldName(axis);
   pass.setFilterLimits (lowerLimit, upperLimit);
   pass.filter(cloud);
}

void findPlane(pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointIndices &inliers, pcl::ModelCoefficients &coefficients) {
	Eigen::Vector3f v = Eigen::Vector3f(1.0, 0.0, 0.0);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_ORIENTED_PLANE);
	seg.setAxis(v);
	seg.setEpsAngle(15.0);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.02);
	seg.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud));
	seg.segment (inliers, coefficients);
}

double length(pcl::PointXYZ p1, pcl::PointXYZ p2) {
	return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

bookbot::BBox findBBox(pcl::PointCloud<pcl::PointXYZ>cloud, pcl::PointIndices inliers, pcl::ModelCoefficients coeffs) {
	double zPrime[] = {-coeffs.values[0]*coeffs.values[2], -coeffs.values[1]*coeffs.values[2], 1-coeffs.values[2]*coeffs.values[2]};
	double lenZ = sqrt(zPrime[0]*zPrime[0] + zPrime[1]*zPrime[1] + zPrime[2]*zPrime[2]);
	double v[3], w[3];
	for (int i = 0; i<3; i++) v[i] = zPrime[i]/lenZ;
	w[0] = coeffs.values[1]*v[2] - coeffs.values[2]*v[1]; 
	w[1] = coeffs.values[2]*v[0] - coeffs.values[0]*v[2];
	w[2] = coeffs.values[0]*v[1] - coeffs.values[1]*v[0];
	float points[inliers.indices.size()][2];
	for (int i = 0; i<(int) inliers.indices.size(); i++) {
		points[i][0] = v[0]*cloud.points[inliers.indices[i]].x + v[1]*cloud.points[inliers.indices[i]].y + v[2]*cloud.points[inliers.indices[i]].z;
		points[i][1] = w[0]*cloud.points[inliers.indices[i]].x + w[1]*cloud.points[inliers.indices[i]].y + w[2]*cloud.points[inliers.indices[i]].z;
	}
	long xmin, xmax, ymin, ymax;
	xmin = xmax = ymin = ymax = 0;
	for (int i = 1; i < (int) inliers.indices.size(); i++) {
		if (points[i][0]<points[xmin][0])
	   	xmin = i;
	   if (points[i][0]>points[xmax][0])
	      xmax = i;
	   if (points[i][1]<points[ymin][1])
	      ymin = i;
	   if (points[i][1]>points[ymax][1])
	      ymax = i;
	}
	bookbot::BBox box;
	box.lowerLeft.header = cloud.header;
	box.upperRight.header = cloud.header;
	box.lowerRight.header = cloud.header;
	box.upperLeft.header = cloud.header;
	
	box.lowerLeft.point.x = -coeffs.values[3]*coeffs.values[0] + points[xmin][0]*v[0] + points[ymin][1]*w[0];
	box.lowerLeft.point.y = -coeffs.values[3]*coeffs.values[1] + points[xmin][0]*v[1] + points[ymin][1]*w[1];
	box.lowerLeft.point.z = -coeffs.values[3]*coeffs.values[2] + points[xmin][0]*v[2] + points[ymin][1]*w[2];

	box.upperRight.point.x = -coeffs.values[3]*coeffs.values[0] + points[xmax][0]*v[0] + points[ymax][1]*w[0];
	box.upperRight.point.y = -coeffs.values[3]*coeffs.values[1] + points[xmax][0]*v[1] + points[ymax][1]*w[1];
	box.upperRight.point.z = -coeffs.values[3]*coeffs.values[2] + points[xmax][0]*v[2] + points[ymax][1]*w[2];

	box.upperLeft.point.x = -coeffs.values[3]*coeffs.values[0] + points[xmax][0]*v[0] + points[ymin][1]*w[0];
	box.upperLeft.point.y = -coeffs.values[3]*coeffs.values[1] + points[xmax][0]*v[1] + points[ymin][1]*w[1];
	box.upperLeft.point.z = -coeffs.values[3]*coeffs.values[2] + points[xmax][0]*v[2] + points[ymin][1]*w[2];

	box.lowerRight.point.x = -coeffs.values[3]*coeffs.values[0] + points[xmin][0]*v[0] + points[ymax][1]*w[0];
	box.lowerRight.point.y = -coeffs.values[3]*coeffs.values[1] + points[xmin][0]*v[1] + points[ymax][1]*w[1];
	box.lowerRight.point.z = -coeffs.values[3]*coeffs.values[2] + points[xmin][0]*v[2] + points[ymax][1]*w[2];

	box.success = true;	
	return box;
}

void callback(sensor_msgs::PointCloud2 cloudMessage) {
	if (table.corners[0].x!=table.corners[2].x) {
		cout << "Cloud #"<< ctr++ << endl;
		//std::clock_t start = std::clock();
		geometry_msgs::Point tmpPoints[4];
		for (int i = 0; i<4; i++) {
			tmpPoints[i].x = table.corners[i].x;
			tmpPoints[i].y = table.corners[i].y;
			tmpPoints[i].z = table.corners[i].z;
		}
		double dx = tmpPoints[1].x - tmpPoints[0].x;
		double dy = tmpPoints[1].y - tmpPoints[0].y;
		double angle = atan2(dy, dx);
	
		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::fromROSMsg(cloudMessage, cloud);

		//transform points to table frame
		for (int i = 0; i < (int) cloud.points.size(); i++)
			transformPointXYZ(cloud.points[i], angle, tmpPoints[0]);

		for (int i = 3; i >=0; i--) {
		  tmpPoints[i].x -= tmpPoints[0].x;
			tmpPoints[i].y -= tmpPoints[0].y;
		}

		//set table borders
		double left = (tmpPoints[0].x + tmpPoints[3].x)/2;
		double right = (tmpPoints[1].x + tmpPoints[2].x)/2;
		double top = (tmpPoints[3].y + tmpPoints[2].y)/2;
		double bottom = (tmpPoints[0].y + tmpPoints[1].y)/2;
		double height = 0.0;
		for (int i=0; i<4; i++) {
		   height += tmpPoints[i].z;
		}
		height/=4.0;
		double maxBookHeight = 0.45;

		//filter out the table and the booktop
		filter(cloud, (char*) "x", left, right);
		filter(cloud, (char*) "y", bottom, top);
		filter(cloud, (char*) "z", height + 0.02, height + maxBookHeight);		
		/*pcl_visualization::PCLVisualizer vis;	
		vis.addCoordinateSystem();
		vis.addPointCloud(cloud);	
		vis.spinOnce();
		vis.removePointCloud();*/

		double zmax = 0;
		for (int i = 0; i < (int) cloud.points.size(); i++) {
			if (cloud.points[i].z>cloud.points[zmax].z)
		   	zmax = i;
		}
		double booktop = cloud.points[zmax].z;
		filter(cloud, (char*) "z", height + 0.02, booktop); //old one cuts off 3 cm from the top

		/*vis.addPointCloud(cloud);
		vis.spin();
		vis.removePointCloud();*/
	
		//find a plane
		pcl::PointIndices inliers;
		pcl::ModelCoefficients coeffs;
		findPlane(cloud, inliers, coeffs);
		bookbot::BBox bbox;
		if (inliers.indices.size()==0) {
			bbox.success = false;
			ROS_INFO("Bounding box not found");
		}
		else {
			bbox = findBBox(cloud, inliers, coeffs);
		
			//need to rotate back
			angle = -angle;
			geometry_msgs::Point oldOrigin;
			oldOrigin.x = -table.corners[0].x; oldOrigin.y = -table.corners[0].y; oldOrigin.z = -table.corners[0].z;
			transformPoint(bbox.lowerLeft, angle, oldOrigin);
			transformPoint(bbox.upperRight, angle, oldOrigin);
			transformPoint(bbox.lowerRight, angle, oldOrigin);
			transformPoint(bbox.upperLeft, angle, oldOrigin);
		
			//for printing to file
			/*if (pts2.size() > 10000 && !written) {
				written = true;
				//cout << "printing" << endl;
				ofstream output("cloud.txt");
				for (int i = 0; i<pts2.size(); i++) {
					transformPointXYZ(pts2[i], angle, oldOrigin);
					output << pts2[i].x << " " << pts2[i].y << " " << pts2[i].z << endl;
				}
				output.close();
				//cout << "printed" << endl;
			}*/
		
			//string frame("/output_frame");
			string frame = "/narrow_stereo_optical_frame";
			//cout << "Points in narrow stereo:" << endl;
			try {
				tfListener->transformPoint(frame, bbox.lowerLeft, bbox.lowerLeft);
				tfListener->transformPoint(frame, bbox.upperRight, bbox.upperRight);
				tfListener->transformPoint(frame, bbox.lowerRight, bbox.lowerRight);
				tfListener->transformPoint(frame, bbox.upperLeft, bbox.upperLeft);
				//cout << bbox << endl;
				//cout << "Time to process: " << (std::clock() - start) / (double)CLOCKS_PER_SEC << endl;
				ROS_INFO("Publishing bounding box");
				pub.publish(bbox);
			} catch (tf::TransformException ex) {
				ROS_ERROR("%s",ex.what());
				cout << "Error: " << ex.what() << endl;
			}	
		}
	}
}

void tableCallback(bookbot::Table message) {
	table = message;
	ROS_INFO("Table received");
}

void listener(ros::NodeHandle n) {
	ros::Subscriber tableSub = n.subscribe("table", 1, tableCallback);
	while (table.corners[0].x==table.corners[2].x && ros::ok()) {
		ros::spinOnce();
	}
	//ros::Subscriber pclSub = n.subscribe("/points2_out", 10, callback);
	ros::Subscriber pclSub = n.subscribe("transformedPointCloud2", 10, callback);
	ros::spin();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "stack_finder");
	ros::NodeHandle n;
	tfListener = new tf::TransformListener;
	pub = n.advertise<bookbot::BBox>("bookBox", 10);
	table.corners[0].x = 0;
	table.corners[2].x = 0;
	listener(n);
}
