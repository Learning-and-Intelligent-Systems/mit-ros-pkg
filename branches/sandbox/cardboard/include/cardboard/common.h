#ifndef CARDBOARD_COMMON_H_
#define CARDBOARD_COMMON_H_
#ifndef EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#endif

#include <ros/ros.h>
#include <furniture/All_Hulls.h>
#include <furniture/Table_Polygons.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/segmentation/extract_polygonal_prism_data.hpp>
#include <pcl_ros/transforms.h>
//#include <pcl_visualization/pcl_visualizer.h>
//#include <pcl_visualization/range_image_visualizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>


#include <string>
#include <vector>
#include <list>

#include <math.h>
#include <stdlib.h>


// eigen
#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <Eigen/LU>

using namespace std;
using namespace Eigen;


typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::Normal> PointCloudN;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudXYZN;
typedef pcl::KdTree<pcl::PointXYZ> KdTreeXYZ;




#endif
