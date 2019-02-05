#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

#include <pcl/io/pcd_io.h>

#include <furniture/color_util.h>
#include "util.h"

using namespace::std;

int main(int argc, char **argv) {
  // init ROS
  ros::init(argc, argv, "table_analyzer");
  
  int currFile = -1;
  ros::Time lastPublish;

  ofstream outFile;
  
  ros::NodeHandle nh;

  ros::Publisher pub;
  pub = nh.advertise<sensor_msgs::PointCloud2>("table", 1);
  
  // Parameters
  double z_min, z_max, plane_dist, plane_angle, cluster_dist, normals_angle;
  int k_knn, plane_min_size, cluster_min_size;

  sensor_msgs::PointCloud2 table;
  
  tf::TransformListener tfListener;
  ros::Rate rate(15.0);

  //nh.param("table_analyzer/z_min", z_min, 0.5);
  nh.param("table_analyzer/z_min", z_min, 0.4);
  //nh.param("table_analyzer/z_max", z_max, 0.9);
  nh.param("table_analyzer/z_max", z_max, 0.8);
  nh.param("table_analyzer/k_knn", k_knn, 20);
  nh.param("table_analyzer/plane_distance_tolerance", plane_dist, 0.03);
  nh.param("table_analyzer/plane_angle_tolerance", plane_angle, 0.087);
  nh.param("table_analyzer/plane_min_size", plane_min_size, 300);
  nh.param("table_analyzer/cluster_min_size", cluster_min_size, 300);
  nh.param("table_analyzer/cluster_distance_tolerance", cluster_dist, 0.3);
  nh.param("table_analyzer/normals_angle_tolerance", normals_angle, 0.25);

  ROS_INFO("Initialized node");
  
  while (nh.ok()) {
    ROS_INFO("Entered loop");
    // Spin until we get a PointCloud message
    sensor_msgs::PointCloud2ConstPtr cloud_blob_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/rgb/points");
    // Transform point cloud to base_footprint coordinates
    try {
      
      
      // get sensor pose
      tf::StampedTransform tf_transform;
      const string s = "/base_footprint";
      if (!tfListener.waitForTransform(s, "/openni_rgb_optical_frame", cloud_blob_ptr->header.stamp, ros::Duration(1.0))) {
	cout << cloud_blob_ptr->header.stamp << endl;
	cout << ros::Time::now() << endl;
	cout << ros::WallTime::now() << endl;
	ROS_ERROR("No transform...");
	continue;
      }
      // tfListener.waitForTransform(s, "/openni_rgb_optical_frame", cloud_blob_ptr->header.stamp, ros::Duration(5.0));
      // tfListener.lookupTransform(s, cloud_blob_ptr->header.frame_id, cloud_blob_ptr->header.stamp, tf_transform);
      tfListener.lookupTransform(s, "/openni_rgb_optical_frame", cloud_blob_ptr->header.stamp, tf_transform);
      Eigen::Affine3f sensor_pose;
      //tfListener->lookupTransform(msg.header.frame_id, target_frame, msg.header.stamp, tf_transform);
      transformTFToEigen(tf_transform, sensor_pose);
      ROS_INFO("Got cloud with %d points", cloud_blob_ptr->height * cloud_blob_ptr->width);
 
      // Convert to PCL
      pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
      pcl::fromROSMsg(*cloud_blob_ptr, pcl_cloud);
      ROS_INFO("Converted to PCL of size %ld", pcl_cloud.size());

      pcl::transformPointCloud(pcl_cloud, pcl_cloud, sensor_pose);
      pcl_cloud.header.frame_id = s; // PCL transform doesn't change the point cloud

      downsample_cloud(pcl_cloud, pcl_cloud, z_min, z_max);

      // Segment out the plane
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

      segment_plane(pcl_cloud, inliers, coefficients, plane_dist, plane_angle, plane_min_size);
      ROS_INFO("Segmented the plane of size %ld", inliers->indices.size());
      
      if (inliers->indices.size() == 0) continue;

      extract_indices<pcl::PointXYZRGB>(pcl_cloud, inliers, pcl_cloud);

      // Clustering
      std::vector<pcl::PointIndices> cluster_indices;
      cluster_data(pcl_cloud, cluster_indices, cluster_min_size, cluster_dist);

      size_t largest_idx = 0;
      for (size_t i = 1; i < cluster_indices.size(); ++i) {
	if (cluster_indices[i].indices.size() > cluster_indices[largest_idx].indices.size()) {
	  largest_idx = i;
	}
      }

      *inliers = cluster_indices[largest_idx];
      extract_indices(pcl_cloud, inliers, pcl_cloud);

      pcl::toROSMsg(pcl_cloud, table);
      pub.publish(table);

      // Dump the RGB values from the point cloud into a file for MATLAB postprocessing
      //if (currFile == -1 || ros::Time::now() - lastPublish >= ros::Duration(dumpingFreq)) {
	ROS_WARN("Dumping data");
	++currFile;
	lastPublish = ros::Time::now();
	stringstream ss;
	ss << currFile << ".txt";
	cout << ss.str() << endl;
	outFile.open((char *) ss.str().c_str());

	/*ss.str(std::string());
	ss << pcl_cloud.size() << "\n";
	outFile << ss.str();*/

	for (size_t i = 0; i < pcl_cloud.size(); ++i) {
	  ss.str(std::string());
	  
	  // unpack rgb into r/g/b
	  uint32_t rgb = *reinterpret_cast<int*>(&pcl_cloud.points[i].rgb);
	  uint8_t r = (rgb >> 16) & 0x0000ff;
	  uint8_t g = (rgb >> 8)  & 0x0000ff;
	  uint8_t b = (rgb)       & 0x0000ff;
	  ss << (int) r << " " << (int) g << " " << (int) b << "\n";
	  outFile << ss.str();
	}
		
	outFile.close();
	//}
	
	ss.str(std::string());
	ss << "LAB_" << currFile << ".txt";
	cout << ss.str() << endl;
	outFile.open((char *) ss.str().c_str());

	furniture::convert_rgb_to_lab(pcl_cloud);

	for (size_t i = 0; i < pcl_cloud.size(); ++i) {
	  ss.str(std::string());
	  
	  // unpack rgb into r/g/b
	  uint32_t rgb = *reinterpret_cast<int*>(&pcl_cloud.points[i].rgb);
	  int r = (int) (rgb >> 16) & 0x0000ff;
	  int g = (int) (rgb >> 8)  & 0x0000ff;
	  int b = (int) (rgb)       & 0x0000ff;
	  // Offset back
	  //g -= 110;
	  //b -= 110;
	  ss << (int) r << " " << (int) g << " " << (int) b << "\n";
	  outFile << ss.str();
	}
		
	outFile.close();
      
    } catch (tf::TransformException& ex) {
      ROS_WARN("[point_cloud_callback] TF exception:\n%s", ex.what());
    }

  }
}
