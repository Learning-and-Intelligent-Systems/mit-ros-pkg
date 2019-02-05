#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/statistical_outlier_removal.h"

#include "pcl/filters/voxel_grid.h"

  typedef pcl::PointWithViewpoint Point;
/* ---[ */
int
  main (int argc, char** argv)
{
  sensor_msgs::PointCloud2   cloud2_filtered,cloud_filtered2;
  pcl::PointCloud<Point> cloud, cloud_filtered;
  // Fill in the cloud data
  pcl::PCDReader reader;
//  reader.read ("frontstatic.pcd", cloud);
  reader.read ("asdfasdf2", cloud);

  ROS_INFO ("PointCloud before filtering: %d data points (%s).", cloud.width * cloud.height, pcl::getFieldsList (cloud).c_str ());

  pcl::StatisticalOutlierRemoval<Point> sor2;
  sor2.setInputCloud (boost::make_shared<pcl::PointCloud<Point> >(cloud));
  sor2.setMeanK (50);
  sor2.setStddevMulThresh (1.0);
  sor2.filter (cloud_filtered);
  ROS_INFO ("PointCloud after filtering outliers: %d data points (%s).", cloud_filtered.width * cloud_filtered.height, pcl::getFieldsList (cloud_filtered).c_str ());

  pcl::toROSMsg(cloud_filtered,cloud2_filtered);

  // Create the filtering object
  pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
  sor.setInputCloud (boost::make_shared<sensor_msgs::PointCloud2> (cloud2_filtered));
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (cloud_filtered2);

  ROS_INFO ("PointCloud after downsampling: %d data points (%s).", cloud_filtered2.width * cloud_filtered2.height, pcl::getFieldsList (cloud_filtered2).c_str ());



  pcl::PCDWriter writer;
  writer.write ("staticfront_downsampled2.pcd", cloud_filtered2,Eigen3::Vector4f::Zero (),  Eigen3::Quaternionf::Identity (), false);

  return (0);
}
