#ifndef TABLE_DETECTOR_UTIL_H_
#define TABLE_DETECTOR_UTIL_H_

#include <furniture/common.h>

#include <furniture/All_Hulls.h>
#include <furniture/Table_clouds.h>
#include <furniture/Table_Poses.h>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/range_image/range_image.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/registration/transforms.h>
#include <pcl/point_types.h>

#include <visualization_msgs/Marker.h>

using namespace std;

double angle3D(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2, geometry_msgs::Point32 p3);

bool flat(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2, geometry_msgs::Point32 p3);

void convert_ROS_to_PCL(sensor_msgs::PointCloud cloud, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud);
void convert_PCL_to_ROS(pcl::PointCloud<pcl::PointXYZ> pcl_cloud, sensor_msgs::PointCloud &cloud);

template <class PointT>
void downsample_cloud(pcl::PointCloud<PointT> initial_cloud, pcl::PointCloud<PointT> &cloud_downsampled, double z_min, double z_max) {
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud (initial_cloud.makeShared());
  vg.setFilterFieldName("z");
  vg.setFilterLimits(z_min, z_max);
  vg.setFilterLimitsNegative(false);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (cloud_downsampled);
  ROS_INFO("Downsampled point cloud to %ld points", cloud_downsampled.points.size());
}

void estimate_normals(pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointCloud<pcl::Normal> &cloud_normals, int k_knn);
void remove_non_vertical(sensor_msgs::PointCloud cloud_ROS, pcl::PointCloud<pcl::Normal> cloud_normals, sensor_msgs::PointCloud &cloud_horizontal, double normals_angle);

template <class PointT>
void cluster_data(pcl::PointCloud<PointT> cloud, vector<pcl::PointIndices> &cluster_indices, int cluster_min_size, double cluster_dist) {
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (0.05); // 5cm
  ec.setMinClusterSize (1000);
  ec.setMaxClusterSize (100000);
  ec.setSearchMethod (boost::make_shared<pcl::KdTreeFLANN<PointT> > ());
  ec.setInputCloud(cloud.makeShared());
  ec.extract (cluster_indices);
  ROS_INFO("Found %ld clusters", cluster_indices.size());
}

template <class PointT>
void extract_indices(pcl::PointCloud<PointT> cloud, pcl::PointIndices::Ptr indices, pcl::PointCloud<PointT> &extracted_cloud) {
  pcl::ExtractIndices<PointT> extract;	
  extract.setInputCloud (cloud.makeShared());
  extract.setIndices (indices);
  extract.setNegative (false);
  extract.filter (extracted_cloud);
}

template <class PointT>
void segment_plane(pcl::PointCloud<PointT> cloud, pcl::PointIndices::Ptr &plane_inliers, pcl::ModelCoefficients::Ptr &plane_coefficients, double plane_dist, double plane_angle, int plane_min_size) {
  Eigen::Vector3f v = Eigen::Vector3f(0.0, 0.0, 1.0); // z-axis is normal
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setAxis(v);
  seg.setEpsAngle(plane_angle);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (plane_dist);
  seg.setInputCloud (cloud.makeShared());
  seg.segment (*plane_inliers, *plane_coefficients);
  ROS_INFO("Segmented the plane of size %ld", plane_inliers->indices.size());
}
void flatten_plane(pcl::PointCloud<pcl::PointXYZ> cloud, pcl::ModelCoefficients plane_coefficients, pcl::PointCloud<pcl::PointXYZ> &cloud_projected);
void find_convex_hull(pcl::PointCloud<pcl::PointXYZ> cloud_projected, geometry_msgs::PolygonStamped &current_hull);
void create_markers(furniture::All_Hulls hulls, vector<visualization_msgs::Marker> &markers);	

void find_tables(const pcl::PointCloud<pcl::PointXYZ> &initial_cloud, furniture::Table_clouds &table_clouds, furniture::All_Hulls &hulls, double z_min = 0.4, double z_max = 1.1, int k_knn = 20, double plane_dist = 0.03, double plane_angle = 0.087, int plane_min_size = 300, int cluster_min_size = 300, double cluster_dist = 0.3, double normals_angle = 0.25);

void transformTFToEigen(const tf::Transform &t, Eigen::Affine3f &k);

void fit_table_models(const pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Affine3f sensor_pose,
		      const furniture::All_Hulls &hulls, furniture::Table_Poses &table_poses);

#endif
