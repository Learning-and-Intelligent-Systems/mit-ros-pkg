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
#include "pcl/point_types.h"
typedef pcl::PointWithViewpoint Point;
#include "pcl_helpers.h"
#include <fstream>

using namespace std;

void writePLY(pcl::PointCloud<pcl::PointXYZINormal> &cloud){
	std::ofstream outf;
	outf.open("object.ply",ios::out);
	outf<<"ply\n"<<"format ascii 1.0\n"<<"element vertex "<<cloud.points.size()<<endl;
	outf<<"property float x\nproperty float y\nproperty float z\n";
	outf<<"property float nx\nproperty float ny\nproperty float nz\n";
	outf<<"element face 0\n";
	outf<<"element face 0\nproperty list uchar int vertex_indices\nend_header\n";
	for (int i = 0; i < cloud.points.size(); ++i) {
		outf<<cloud.points[i].x<<" "<<cloud.points[i].y<<" "<<cloud.points[i].z;
		outf<<" "<<cloud.points[i].normal[0]<<" "<<cloud.points[i].normal[1]<<" "<<cloud.points[i].normal[2]<<endl;
	}
	outf.close();

}




int
  main (int argc, char** argv)
{
//
//  pcl::PointCloud<Point> cloud;
//
//
//  readPCD("object02.pcd", cloud);

   pcl::PointCloud<pcl::PointXYZINormal> cloud_normals;
   readPCD("fullcluster.pcd", cloud_normals);
//  timeval t0=g_tick();
//  getNormals(cloud,cloud_normals);
//  myFlipNormals(cloud.points[0].vp_x,cloud.points[0].vp_y,cloud.points[0].vp_z,cloud_normals);
//
//
//
//
//  std::cout<<"get normals took:  "<<g_tock(t0)<<std::endl;

  writePLY(cloud_normals);



  return 0;
}

