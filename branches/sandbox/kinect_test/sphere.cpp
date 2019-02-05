/*
 * sphere.cpp
 *
 *  Created on: Jan 23, 2012
 */
#include <limits>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
//#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>



inline int rangeZ(double x, double y, double z, double xc, double yc, double zc, double r) {
	double new_r = (z*r)/zc;
	double d = -((xc-x)*(xc-x)+(yc-y)*(yc-y));
	double a = r*r+d;
	double b = new_r*new_r+d;
//	std::cout << "z-0.02=" << z-0.02 << ", b=" << b << " z+0.02=" << z+0.02 << std::endl;
	if(b<0 || a<0) {
		return -2;
	}
	else if(x!=x || y!=y) {
		return -1;
	}

	b = zc-sqrt(a);
	if(b<=z+0.02 && b>=z-0.02) {
		return 1;
	}
	else {
		return 0;
	}
}

inline double pixel_rad(double rad, double z) {
	double pixel_r=69.3*(1.0/z)*rad/0.1;
	return pixel_r;
}

inline int clip(int lo, int x, int hi) {
	return std::max(std::min(x,hi),lo);
}


inline double percentage_in_region(double x, double y, double z, double rad, int uc, int vc, const pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
	double cz = z + rad;
	double cyl = 0;
	double reg = 0;
	if(x!=x || y!=y || z!=z) {
		return 0;
	}
	double pixel_r = pixel_rad(rad,z);

	int r = pixel_r*2+1;
	int n = 25;
	int start = vc-pixel_r;
	int mx = 480;
	for(int v = clip(0,start,mx);v<clip(0,r+start,mx);v+=ceil((double)r/n)) {
		int start = uc-pixel_r;
		int mx = 640;
		for(int u = clip(0,start,mx);u<clip(0,r+start,mx);u+=ceil((double)r/n)) {
			pcl::PointXYZRGB p = cloud(u,v);
//			double dist = sqrt((uc-u)*(uc-u)+(vc-v)*(vc-v));
			int d = rangeZ(p.x,p.y,p.z,x,y,cz,rad);
			if(d==-1) {
				cyl+=1;
			}
			else if(d==0) {
				cyl=cyl+1;
			}
			else if(d==1) {
				cyl=cyl+1;
				reg=reg+1;
			}
		}
	}
//	std::cout << "cyl: " << cyl << " reg: " << reg << std::endl;
	if(cyl==0) return 0;
	return (reg/cyl)*100.0;
}

class KinectCamera {
private:
	bool waitingForCloud;
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	ros::Subscriber cloud_sub;
public:
	KinectCamera(const std::string & cloud_topic_name = "/camera/rgb/points") :
		waitingForCloud(false) {
		ros::NodeHandle n;
		//if a node was just started, there seem to be some problems immediately connecting to the camera, so
		//wait for a tenth of a second just in case this is a new node.
		ros::Rate r(10);
		r.sleep();

		double timeout = 10.0;
		sensor_msgs::PointCloud2ConstPtr C = ros::topic::waitForMessage<
				sensor_msgs::PointCloud2>(cloud_topic_name, ros::Duration(timeout));
		if (C == 0) {
			std::ostringstream err;
			err << "No message received from the topic: " << cloud_topic_name
					<< ", in " << timeout << " seconds, aborting." << std::endl;
			throw std::runtime_error(err.str());
		}
		pcl::fromROSMsg(*C, cloud);

		std::cout << "Subscribing to " << cloud_topic_name << "..."
				<< std::flush;
		cloud_sub = n.subscribe(cloud_topic_name, 1,
				&KinectCamera::cloud_callback, this);
		std::cout << " done." << std::endl;

	}
	void get() {
		waitingForCloud = true;
		ros::Rate r(100);
		while (waitingForCloud) {
			ros::spinOnce();
			r.sleep();
		}
	}
	pcl::PointCloud<pcl::PointXYZRGB>& getLastCloud() {
		return cloud;
	}
	cv::Mat_<cv::Vec3b> getLastCloudAsImage() {
		cv::Mat_<cv::Vec3b> image(cloud.height,cloud.width);
		int ind = 0;
		for (int imy = 0; imy < (int)cloud.height; imy++) {
			for (int imx = 0; imx < (int)cloud.width; imx++, ind++) {
				image.at<cv::Vec3b>(imy, imx) =
						cv::Vec3b(cloud.points[ind].b, cloud.points[ind].g, cloud.points[ind].r);
			}
		}
		return image;
	}
	cv::Mat_<float> getLastCloudAsMat() {
		cv::Mat_<float> depth(cloud.height,cloud.width);
		int ind = 0;
		for (int imy = 0; imy < (int)cloud.height; imy++) {
			for (int imx = 0; imx < (int)cloud.width; imx++, ind++) {
				depth.at<float>(imy, imx) = cloud.points[ind].z;
			}
		}
		return depth;
	}
private:
	void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& C) {
		if (waitingForCloud) {
			pcl::fromROSMsg(*C, cloud);
			waitingForCloud = false;
		}
	}
};


int main(int argc, char** argv) {
	ros::init(argc, argv, "sphere");
	rosbag::Bag bag;
	if(argc<2) {
		KinectCamera camera("/camera/rgb/points");
		while(true) {
			camera.get();
			cv::Mat_<cv::Vec3b> image = camera.getLastCloudAsImage();
			cv::Mat_<cv::Vec3b> debug_image(image.rows,image.cols);
			pcl::PointCloud<pcl::PointXYZRGB> cloud = camera.getLastCloud();
			for(int v=0;v<image.rows;v++) {
				for(int u=0;u<image.cols;u++) {
					pcl::PointXYZRGB& pt = cloud(u,v);
					double p = percentage_in_region(pt.x,pt.y,pt.z,0.1,u,v,cloud);
					p = (p*255.0)/100.0;
					debug_image.at<cv::Vec3b>(v,u) = cv::Vec3b(p,p,p);
				}
			}
			cv::imshow("test",image);
			cv::waitKey(100);
			cv::imshow("debugwindow",debug_image);
			cv::waitKey(100);
		}
	}
	bag.open(argv[1], rosbag::bagmode::Read);
	std::vector<std::string> topics;
	std::string image_topic_name = "/camera/rgb/points";
	topics.push_back(image_topic_name);
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	BOOST_FOREACH(rosbag::MessageInstance const m, view) {
		if (m.getTopic() == image_topic_name || ("/" + m.getTopic() == image_topic_name)) {
			sensor_msgs::PointCloud2::ConstPtr pointcloud = m.instantiate<sensor_msgs::PointCloud2>();
			pcl::PointCloud < pcl::PointXYZRGB > cloud;
			sensor_msgs::Image im;
			pcl::fromROSMsg(*pointcloud, cloud);
			pcl::toROSMsg(*pointcloud, im);
			cv::Mat_<cv::Vec3b> image = cv_bridge::toCvCopy(im, "bgr8")->image;
			cv::Mat_<cv::Vec3b> debug_image(image.rows,image.cols);

			for(int v=0;v<image.rows;v++) {
				for(int u=0;u<image.cols;u++) {
					pcl::PointXYZRGB& pt = cloud(u,v);
					double p = percentage_in_region(pt.x,pt.y,pt.z,0.1,u,v,cloud);
					p = (p*255.0)/100.0;

					debug_image.at<cv::Vec3b>(v,u) = cv::Vec3b(p,p,p);
				}
			}
			cv::imshow("test",image);
			cv::waitKey(100);
			cv::imshow("debugwindow",debug_image);
			cv::waitKey(100);
		}
	}
}
