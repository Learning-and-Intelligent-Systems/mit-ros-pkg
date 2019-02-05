/*
 * pcd_to_obj.cpp
 *
 *  Created on: Oct 5, 2010
 *      Author: garratt
 */
//
//    fout=open("cloudout.ply",'w')
//    fout.write("ply\n")
//    fout.write("format ascii 1.0\n")
//    fout.write("element vertex %u\n"%len(newcloud.points))
//    fout.write("property float x\n")
//    fout.write("property float y\n")
//    fout.write("property float z\n")
//    fout.write("element face 0\n")
//    fout.write("property list uchar int vertex_indices\n")
//    fout.write("end_header\n")
//
//


#include "furniture_ops/pcl_helpers.hpp"




int
  main (int argc, char** argv)
{

   pcl::PointCloud<pcl::PointXYZ> pcloud;
   string filename="pcds/backleftbigger_object15.pcd";
   pcl::PCDReader reader;
   sensor_msgs::PointCloud2 cloud;
   reader.read (filename, cloud);
   pcl::fromROSMsg (cloud,pcloud);
   pcl::PointXYZ pt,ref(0,0,0);
   std::cout<<"closest dist = "<<getClosestPoint(pcloud,ref,pt)<<std::endl;


  return 0;
}

