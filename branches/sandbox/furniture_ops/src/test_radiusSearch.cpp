/*
 * test_radiusSearch.cpp
 *
 *  Created on: Oct 12, 2010
 *      Author: garratt
 */

#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/kdtree/kdtree_ann.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

using namespace std;


timeval g_tick(){
   struct timeval tv;
   gettimeofday(&tv, NULL);
   return tv;
}

double g_tock(timeval tprev)
{
   struct timeval tv;
   gettimeofday(&tv, NULL);
   return (double)(tv.tv_sec-tprev.tv_sec) + (tv.tv_usec-tprev.tv_usec)/1000000.0;
}

int
  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width    = 1000;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);
  int ptindex=0;
  for(double x = -.5; x < .5; x+=.1)
     for(double y = -.5; y < .5; y+=.1)
        for(double z = -.5; z < .5; z+=.1){
           cloud.points[ptindex].x=x;
           cloud.points[ptindex].y=y;
           cloud.points[ptindex].z=z;
           ptindex++;
        }


  pcl::KdTreeFLANN<pcl::PointXYZ> flann_tree;
  pcl::KdTreeANN<pcl::PointXYZ> ann_tree;
  flann_tree.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cloud));
  ann_tree.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cloud));
  pcl::PointXYZ testpt(0.0,0.0,0.0);
  vector<int> indices;
  vector<float> dists;
  timeval t0;
  t0=g_tick();
  int tests[] = {1,10,100,1000,10000,-1};
  for(int i=0;i<6;i++){
     t0=g_tick();
     if(!flann_tree.radiusSearch(testpt,1.1,indices,dists,tests[i]))  //find all the points close to this point
        cout<<"radius search failed!"<<endl;
     cout<<"returns from flann radius search (checks="<<tests[i]<<"): "<<indices.size()<<"  time: "<<g_tock(t0)<<endl;
  }

  t0=g_tick();
  if(!ann_tree.radiusSearch(testpt,1.0,indices,dists,1000))  //find all the points close to this point
     cout<<"radius search failed!"<<endl;
  cout<<"number of returns from ann radius search: "<<indices.size()<<"  time: "<<g_tock(t0)<<endl;


  return (0);
}
