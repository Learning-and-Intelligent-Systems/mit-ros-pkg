/*
 * pcl_helpers.h
 *
 *  Created on: Oct 5, 2010
 *      Author: garratt
 */

#ifndef PCL_HELPERS_H_
#define PCL_HELPERS_H_



#include "pcl/ModelCoefficients.h"

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/statistical_outlier_removal.h"

#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/extract_indices.h"

#include "pcl/filters/passthrough.h"
#include "pcl/features/normal_3d.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/kdtree/organized_data.h"

#include <list>

#include <sys/time.h>
  timeval g_tick();
  double g_tock(timeval tprev);

  typedef pcl::PointWithViewpoint Point;
  void getSubCloud(pcl::PointCloud<Point> &cloudin, std::vector<int> &ind, pcl::PointCloud<Point> &cloudout,bool setNegative=false);

  void getSubCloud(pcl::PointCloud<Point> &cloudin, pcl::PointIndices &ind, pcl::PointCloud<Point> &cloudout,bool setNegative=false);
  void getSubCloud(pcl::PointCloud<pcl::PointXYZINormal> &cloudin,  std::vector<int> &ind, pcl::PointCloud<pcl::PointXYZINormal> &cloudout,bool setNegative);

void segment(pcl::PointCloud<Point> &cloud, std::vector<pcl::PointCloud<Point> > &clusters);
void readPCD(std::string filename, sensor_msgs::PointCloud2 &cloudout);

void readPCD(std::string filename, pcl::PointCloud<Point> &cloudout);

//this function converts a pcl::PointWithViewpoint to pcl::PointXYZINormal if needed
//return -1 of cannot convert, 0 if OK
int readPCD(std::string filename, pcl::PointCloud<pcl::PointXYZINormal> &cloudout);

//determines if cloud is pcl::PointWithViewpoint or pcl::PointXYZINormal
std::string detectCloudType(sensor_msgs::PointCloud2 &cloud);

void myFlipNormals(double vx, double vy, double vz, pcl::PointCloud<pcl::PointXYZINormal> &ncloud);


void getNormals(pcl::PointCloud<pcl::PointWithViewpoint> &cloudin, pcl::PointCloud<pcl::PointXYZINormal> &cloud_normals);
void getNormals(pcl::PointCloud<Point> &cloudin, pcl::PointCloud<pcl::Normal> &cloud_normals);

//this function just extracts the normals from the integrated cloud
void getNormals(pcl::PointCloud<pcl::PointXYZINormal> &cloudin, pcl::PointCloud<pcl::Normal> &cloud_normals);


//Why is this so slow?
void getNormalsOMP(pcl::PointCloud<Point> &cloudin, pcl::PointCloud<pcl::Normal> &cloud_normals);

//void getVertical(pcl::PointCloud<Point> &cloudin){
//	pcl::PointIndices ind;
//    pcl::PassThrough<pcl::Normal> pass_;
//    pass_.setFilterFieldName ("normal[2]");
//    pass_.setFilterLimits (min_z_bounds_, max_z_bounds_);
//    pass_.setInputCloud (boost::make_shared<const pcl::PointCloud<Point> > (cloudin));
//    pass_.filter (cloud_filtered);
//
//
//}

//could do this with a passthrough, but come on...
void segmentFloor(pcl::PointCloud<Point> &cloudin, pcl::PointCloud<Point> &cloudout,bool negativeFilter=true);


//could do this with a passthrough, but come on...
void segmentPplHeight(pcl::PointCloud<Point> &cloudin, pcl::PointCloud<Point> &cloudout);
void removeOutliers(pcl::PointCloud<Point> &cloudin, pcl::PointCloud<Point> &cloudout);
void downSample(pcl::PointCloud<Point> &cloudin, pcl::PointCloud<Point> &cloudout);
void segfast(pcl::PointCloud<Point> &cloud, std::vector<pcl::PointCloud<Point> > &cloud_clusters, double cluster_tol=.2);













#endif /* PCL_HELPERS_H_ */
