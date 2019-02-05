/*
 * object_analysis.cpp
 *
 *  Created on: Oct 17, 2010
 *      Author: garratt
 */

//this program is designed to analyse the data from object_survey, to get a more accurate registration of poses

#include "ScanAnalyzer.h"
#include "pcl/io/pcd_io.h"




void writeMesh2(pcl::PointCloud<pcl::PointXYZINormal> &cloud, std::string filename_base, std::string object_name){
  mesher::MeshParams params;
  params.object_name=object_name;
  params.texture="Gazebo/Terrain";
  params.SolverDivide=20;
  params.IsoDivide=10;
  params.KernelDepth=10;
  params.Depth=10;
  Eigen3::Vector4f centroid;
  pcl::compute3DCentroid(cloud,centroid);
  params.origin[0]=-centroid(0);
  params.origin[1]=-centroid(1);

  writePLY(cloud,"testply.ply");

  mesher::cloud2Mesh(cloud,filename_base,params);
}

int main(int argc, char **argv) {
  if(argc!=3){
    std::cout<<"USAGE "<<argv[0]<<" file_in.bag file_out(no extension) "<<std::endl;
    return -1;
  }

  ros::init(argc, argv, "alignment");
  ScanAnalyzer analyzer(argv[1]);
  analyzer.AlignClouds(2);
  string bagout=argv[2];
  bagout+=".bag";
  ROS_INFO("writing to bag file %s",bagout.c_str());
  analyzer.writeBag(bagout);
  std::string plyname=argv[2];


  for(int j=0;j<17;j++) {
    pcl::PointCloud<pcl::PointXYZRGB> narrow_cloud;
    analyzer.getNarrowStereoCloud(narrow_cloud,j);
    std::stringstream narrow_fname;
    narrow_fname << "narrow" << j << ".pcd";
    pcl::io::savePCDFile(narrow_fname.str(),narrow_cloud);
    ROS_INFO("saved a narrow stereo cloud to %s",narrow_fname.str().c_str());


    pcl::PointCloud<pcl::PointXYZRGB> wide_cloud;
    analyzer.getWideStereoCloud(wide_cloud,j);
    std::stringstream wide_fname;
    wide_fname << "wide" << j << ".pcd";
    pcl::io::savePCDFile(wide_fname.str(),wide_cloud);
    ROS_INFO("saved a wide stereo cloud to %s",wide_fname.str().c_str());
  }

  pcl::PointCloud<pcl::PointXYZINormal> cloud;
  analyzer.getNearestStereoClusterWithoutDownsampling(cloud);
  writeMesh2(cloud,plyname,"chair");



  return 0;
}
