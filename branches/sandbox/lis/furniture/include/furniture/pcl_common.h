#ifndef PCL_COMMON_H_
#define PCL_COMMON_H_

#include <vector>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

namespace furniture {

  template <class PointT>
    void convert_ROS_to_PCL(const sensor_msgs::PointCloud &cloud, pcl::PointCloud<PointT> &pcl_cloud) {
    sensor_msgs::PointCloud2 cloud2;
    sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);
    pcl::fromROSMsg(cloud2, pcl_cloud);
  }
  
  template <class PointT>
    void convert_PCL_to_ROS(const pcl::PointCloud<PointT> &pcl_cloud, sensor_msgs::PointCloud &cloud) {
    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(pcl_cloud, cloud2);
    sensor_msgs::convertPointCloud2ToPointCloud(cloud2, cloud);
  }
  
  template <class PointT>
  void downsample_cloud(const pcl::PointCloud<PointT> &initial_cloud, pcl::PointCloud<PointT> &cloud_downsampled) {
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (initial_cloud.makeShared());
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (cloud_downsampled);
  }
  
  template <class PointT>
  void downsample_cloud(const pcl::PointCloud<PointT> &initial_cloud, pcl::PointCloud<PointT> &cloud_downsampled, double z_min, double z_max) {
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (initial_cloud.makeShared());
    vg.setFilterFieldName("z");
    vg.setFilterLimits(z_min, z_max);
    vg.setFilterLimitsNegative(false);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (cloud_downsampled);
  }
  
  void compute_normals(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_with_normals, int k_knn = 15) {
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> normal_estimation;
    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr kdtree (new pcl::KdTreeFLANN<pcl::PointXYZRGB> ());  
    kdtree->setEpsilon(0.);
    normal_estimation.setSearchMethod(kdtree);
    normal_estimation.setKSearch(k_knn);
    normal_estimation.setInputCloud(cloud.makeShared());
    // normal_estimation.setViewPoint(cloud.sensor_origin_(0), cloud.sensor_origin_(1), cloud.sensor_origin_(2));
    normal_estimation.compute(cloud_with_normals);
    
    for (uint i = 0; i < cloud.points.size(); i++) {
      cloud_with_normals.points[i].x = cloud.points[i].x;
      cloud_with_normals.points[i].y = cloud.points[i].y;
      cloud_with_normals.points[i].z = cloud.points[i].z;
      cloud_with_normals.points[i].rgb = cloud.points[i].rgb;
    }
  }

  void compute_normals(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_with_normals, double radius) {
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> normal_estimation;
    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr kdtree (new pcl::KdTreeFLANN<pcl::PointXYZRGB>());  
    kdtree->setEpsilon(0.);
    normal_estimation.setSearchMethod(kdtree);
    normal_estimation.setRadiusSearch(radius);
    normal_estimation.setInputCloud(cloud.makeShared());
    // normal_estimation.setViewPoint(cloud.sensor_origin_(0), cloud.sensor_origin_(1), cloud.sensor_origin_(2));
    normal_estimation.compute(cloud_with_normals);
    for (uint i = 0; i < cloud.points.size(); i++) {
      cloud_with_normals.points[i].x = cloud.points[i].x;
      cloud_with_normals.points[i].y = cloud.points[i].y;
      cloud_with_normals.points[i].z = cloud.points[i].z;
      cloud_with_normals.points[i].rgb = cloud.points[i].rgb;
    }
  }

  void normals_integral_image(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_with_normals) {
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> normal_estimation;
    normal_estimation.setNormalEstimationMethod(normal_estimation.AVERAGE_3D_GRADIENT);
    normal_estimation.setMaxDepthChangeFactor(0.02f);
    normal_estimation.setNormalSmoothingSize(10.0f);
    normal_estimation.setInputCloud(cloud.makeShared());
    normal_estimation.compute(cloud_with_normals);
    for (uint i = 0; i < cloud.points.size(); i++) {
      cloud_with_normals.points[i].x = cloud.points[i].x;
      cloud_with_normals.points[i].y = cloud.points[i].y;
      cloud_with_normals.points[i].z = cloud.points[i].z;
      cloud_with_normals.points[i].rgb = cloud.points[i].rgb;
    }
  }
  
  template <class PointT>
  void cluster_data(const pcl::PointCloud<PointT> &cloud, std::vector<pcl::PointIndices> &cluster_indices, int cluster_min_size, double cluster_dist) {
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.05); // 5cm
    ec.setMinClusterSize (1000);
    ec.setMaxClusterSize (100000);
    pcl::KdTreeFLANN<PointT> kdtree = new pcl::KdTreeFLANN<PointT>();  
    ec.setSearchMethod(kdtree.makeShared());
    ec.setInputCloud(cloud.makeShared());
    ec.extract (cluster_indices);
  }
  
  template <class PointT>
    void extract_indices(const pcl::PointCloud<PointT> &cloud, pcl::PointIndices::Ptr indices, pcl::PointCloud<PointT> &extracted_cloud) {
    pcl::ExtractIndices<PointT> extract;	
    extract.setInputCloud (cloud.makeShared());
    extract.setIndices (indices);
    extract.setNegative (false);
    extract.filter (extracted_cloud);
  }
  
  template <class PointT>
    void segment_plane(const pcl::PointCloud<PointT> &cloud, pcl::PointIndices::Ptr &plane_inliers, pcl::ModelCoefficients::Ptr &plane_coefficients, double plane_dist, double plane_angle, int plane_min_size) {
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
  }
  
  /* convert a tf transform to an eigen3 transform */
  void transformTFToEigen(const tf::Transform &t, Eigen::Affine3f &k)
  {
    for(int i=0; i<3; i++)
      {
	k.matrix()(i,3) = t.getOrigin()[i];
	for(int j=0; j<3; j++)
	  {
	    k.matrix()(i,j) = t.getBasis()[i][j];
	  }
      }
    // Fill in identity in last row
    for (int col = 0 ; col < 3; col ++)
      k.matrix()(3, col) = 0;
    k.matrix()(3,3) = 1;
  }

  template <class PointT>
  bool validPoint(PointT point) {
    return (pcl_isfinite(point.x) && pcl_isfinite(point.y) && pcl_isfinite(point.z));
  }
  template <class PointNormals>
  bool validNormals(PointNormals point) {
    return (pcl_isfinite(point.normal_x) && pcl_isfinite(point.normal_y) && pcl_isfinite(point.normal_z));
  }
} // namespace

#endif
