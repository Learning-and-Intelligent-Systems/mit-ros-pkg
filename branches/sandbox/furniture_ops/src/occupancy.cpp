/*
 * occupancy.cpp
 *
 *  Created on: Oct 3, 2010
 *      Author: garratt
 */

#include <stdio.h>
#include <octomap/octomap.h>
#include <octomath/Utils.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

using namespace std;
using namespace octomap;
using namespace octomath;

typedef pcl::PointWithViewpoint Point;


void readPCD(std::string filename, pcl::PointCloud<Point> &cloudout){
	  // Fill in the cloud data
	  pcl::PCDReader reader;
	  sensor_msgs::PointCloud2 cloud;
	  reader.read (filename, cloud);
	  pcl::fromROSMsg (cloud,cloudout);
}

int main(int argc, char** argv) {


  //##############################################################
  pcl::PointCloud<Point> cloud;
  OcTree tree (0.01);

  //  point3d origin (10.01, 10.01, 10.02);
  point3d origin (0.01, 0.01, 0.02);
  point3d point_on_surface (2.01,0.01,0.01);

  readPCD("staticfront_downsampled2.pcd", cloud);

  ROS_INFO ("PointCloud before segmentation: %d data points.", cloud.width * cloud.height);

  for(int i=0; i<cloud.points.size();i++){

	  Point p = cloud.points[i];
	  point3d origin (p.vp_x, p.vp_y, p.vp_z), point_on_surface (p.x,p.y,p.z);
      if (!tree.insertRay(origin, origin+point_on_surface)) {
        cout << "ERROR while inserting ray from " << origin << " to " << point_on_surface << endl;
      }


  }


  cout << "done." << endl;
  cout << "writing to sphere.bt..." << endl;
  tree.writeBinaryConst("testscan.bt");

//  // -----------------------------------------------
//
//  cout << "casting rays ..." << endl;
//
//  OcTree sampled_surface (0.05);
//
//  point3d direction = point3d (1.0,0.0,0.0);
//  point3d obstacle(0,0,0);
//
//  unsigned int hit (0);
//  unsigned int miss (0);
//  double mean_dist(0);
//
//  for (int i=0; i<360; i++) {
//    for (int j=0; j<360; j++) {
//      if (!tree.castRay(origin, direction, obstacle, true, 3.)) {
//        miss++;
//      }
//      else {
//        hit++;
//        mean_dist += (obstacle - origin).norm2();
//        sampled_surface.updateNode(obstacle, true);
//      }
//      direction.rotate_IP (0,0,DEG2RAD(1.));
//    }
//    direction.rotate_IP (0,DEG2RAD(1.),0);
//  }
//  cout << "done." << endl;
//
//  mean_dist /= (double) hit;
//  std::cout << " hits / misses: " << hit  << " / " << miss << std::endl;
//  std::cout << " mean obstacle dist: " << mean_dist << std::endl;
//
//  cout << "writing sampled_surface.bt" << endl;
//  sampled_surface.writeBinary("sampled_surface.bt");


}
