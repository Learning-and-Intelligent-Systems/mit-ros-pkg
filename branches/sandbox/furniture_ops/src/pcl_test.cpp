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
#include <list>

#include <pcl/registration/registration.h>
#include <pcl/registration/icp_nl.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "pcl/features/normal_3d.h"
#include "pcl/features/fpfh.h"
#include "pcl/registration/registration.h"
#include "pcl/registration/icp.h"
#include "pcl/registration/icp_nl.h"
#include "pcl/registration/ia_ransac.h"
using namespace pcl;
using namespace pcl::io;
using namespace std;


double ptDist(Point a,Point b){
	return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z);
}
double ptDist(pcl::PointXYZINormal a,pcl::PointXYZINormal b){
	return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z);
}
void getHighWalls(pcl::PointCloud<pcl::PointXYZINormal> &cloudin,pcl::PointCloud<pcl::PointXYZINormal> &cloudout){
	timeval t0=g_tick();
	std::vector<int> ind;
	for(uint i=0;i<cloudin.points.size(); i++){
		if(cloudin.points[i].z >2.0){
			ind.push_back(i);
		}
	}
	getSubCloud(cloudin,ind,cloudout,false);
	std::cout<<"getHighWalls took:  "<<g_tock(t0)<<"  for "<<cloudout.points.size()<<" indices."<<std::endl;

}

void getLocalArea(pcl::PointCloud<pcl::PointXYZINormal> &cloudin,pcl::PointXYZINormal center,double radius,pcl::PointCloud<pcl::PointXYZINormal> &cloudout){
	timeval t0=g_tick();
//	pcl::KdTreeFLANN<pcl::PointXYZINormal> tree;
//	  tree.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZINormal> > (cloudin));
	  vector<int> ind;
//	  vector<float> dists(1000);
//
//	if(!tree.radiusSearch(cloudin,0,radius,ind,dists,100000))
//	  cout<<"radius search failed!"<<endl;
	double srad=radius*radius;
	for(uint i=0;i<cloudin.points.size(); i++){
		if(ptDist(center,cloudin.points[i])< srad){
			ind.push_back(i);
		}
	}


	getSubCloud(cloudin,ind,cloudout,false);
	std::cout<<"getLocalArea took:  "<<g_tock(t0)<<"  for "<<cloudout.points.size()<<" indices."<<std::endl;
}

Eigen3::Matrix4f alignClouds(PointCloud<PointXYZINormal> &snormcloud, PointCloud<PointXYZINormal> &tnormcloud, PointCloud<PointXYZINormal> &aligned){
	  boost::shared_ptr<const PointCloud<PointXYZINormal> > cloud_source_ptr, cloud_target_ptr;
	  cloud_source_ptr = boost::make_shared<const PointCloud<PointXYZINormal> > (snormcloud);
	  cloud_target_ptr = boost::make_shared<const PointCloud<PointXYZINormal> > (tnormcloud);

	  // Initialize estimators for surface normals and FPFH features
	  KdTreeANN<PointXYZINormal> tree;

	  FPFHEstimation<PointXYZINormal, PointXYZINormal, FPFHSignature33> fpfh_est;
	  fpfh_est.setSearchMethod (boost::make_shared<KdTreeANN<PointXYZINormal> > (tree));
	  fpfh_est.setRadiusSearch (0.05);
	  PointCloud<FPFHSignature33> features_source, features_target;

	  fpfh_est.setInputCloud (cloud_source_ptr);
	  fpfh_est.setInputNormals (cloud_source_ptr);
	  fpfh_est.compute (features_source);

	  fpfh_est.setInputCloud (cloud_target_ptr);
	  fpfh_est.setInputNormals (cloud_target_ptr);
	  fpfh_est.compute (features_target);

	  // Initialize Sample Consensus Initial Alignment (SAC-IA)
	  SampleConsensusInitialAlignment<PointXYZINormal, PointXYZINormal, FPFHSignature33> reg;
	  reg.setMinSampleDistance (0.05);
	  reg.setMaxCorrespondenceDistance (0.2);
	  reg.setMaximumIterations (1000);

	  reg.setInputCloud (cloud_source_ptr);
	  reg.setInputTarget (cloud_target_ptr);
	  reg.setSourceFeatures (boost::make_shared<PointCloud<FPFHSignature33> > (features_source));
	  reg.setTargetFeatures (boost::make_shared<PointCloud<FPFHSignature33> > (features_target));
	  cout<<"aligning"<<endl;
	  // Register
	  reg.align (aligned);
	  return reg.getFinalTransformation();

}

int
  main (int argc, char** argv)
{

//  pcl::PointCloud<Point> cloud,lcloud,rcloud;
  pcl::PointCloud<pcl::PointXYZINormal> snormcloud,tnormcloud,aligned;
  pcl::PointCloud<pcl::PointXYZINormal> snormall,tnormall,schair,tchair;

//  readPCD("justfront.pcd", snormall);
//  readPCD("justfrontrightchair.pcd", tnormall);
  readPCD("pcds/backleftbigger_cloud_ndownsampled.pcd", snormall);
  readPCD("pcds/frontright_cloud_ndownsampled.pcd", tnormall);
  readPCD("pcds/backleftbigger_object15.pcd", schair);
  readPCD("pcds/frontright_object08.pcd", tchair);
//  getHighWalls(snormall,snormcloud);
//  getHighWalls(tnormall,tnormcloud);
  getLocalArea(snormall,schair.points[0],1.0,snormcloud);
  getLocalArea(tnormall,tchair.points[0],1.0,tnormcloud);






//  readPCD("justfront.pcd", rcloud);
  //convert clouds into pointXYZINORMAL
//  getNormals(lcloud,lnormcloud);
//  getNormals(rcloud,rnormcloud);

//  pcl::PointXYZ diff(float(random()%100)/1000.0 -.5, float(random()%100)/1000.0 -.5,float(random()%100)/1000.0 -.05);
//   cout<<diff<<endl;
//  int ind=random()%lnormcloud.points.size();
//  pcl::PointXYZ diff(rnormcloud.points[0].x-lnormcloud.points[ind].x, rnormcloud.points[0].y-lnormcloud.points[ind].y, rnormcloud.points[0].z-lnormcloud.points[ind].z);
//
//  for(uint i=0;i<lnormcloud.points.size();i++){
//	  lnormcloud.points[i].x+=diff.x;
//	  lnormcloud.points[i].y+=diff.y;
//	  lnormcloud.points[i].z+=diff.z;
//  }


//  //test registration
//  pcl::IterativeClosestPointNonLinear<pcl::PointXYZINormal,pcl::PointXYZINormal> icp;
//  icp.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZINormal> > (lnormcloud));
//  icp.setInputTarget(boost::make_shared<pcl::PointCloud<pcl::PointXYZINormal> > (rnormcloud));
//  icp.setMaximumIterations (500);
//  icp.setTransformationEpsilon (1e-8);
//  icp.setMaxCorrespondenceDistance (0.05);
//
//
//
////  icp.setSourceFeature(boost::make_shared<pcl::PointCloud<pcl::PointXYZINormal> > (lnormcloud),string("xyznorm"));
////  icp.setTargetFeature(boost::make_shared<pcl::PointCloud<pcl::PointXYZINormal> > (rnormcloud),string("xyznorm"));
//
//  icp.align(aligned); // is lnormcloud, moved
//  while(!icp.hasConverged()){
//	  cout<<"not converged.  "<<endl;
//	  icp.align(aligned);
//  }

  // Create shared pointers
  boost::shared_ptr<const PointCloud<PointXYZINormal> > cloud_source_ptr, cloud_target_ptr;
  cloud_source_ptr = boost::make_shared<const PointCloud<PointXYZINormal> > (snormcloud);
  cloud_target_ptr = boost::make_shared<const PointCloud<PointXYZINormal> > (tnormcloud);

  // Initialize estimators for surface normals and FPFH features
  KdTreeANN<PointXYZINormal> tree;

  NormalEstimation<PointXYZINormal, Normal> norm_est;
  norm_est.setSearchMethod (boost::make_shared<KdTreeANN<PointXYZINormal> > (tree));
  norm_est.setRadiusSearch (0.05);
  PointCloud<Normal> snormals,tnormals;

  FPFHEstimation<PointXYZINormal, PointXYZINormal, FPFHSignature33> fpfh_est;
  fpfh_est.setSearchMethod (boost::make_shared<KdTreeANN<PointXYZINormal> > (tree));
  fpfh_est.setRadiusSearch (0.05);
  PointCloud<FPFHSignature33> features_source, features_target;

  cout<<"estimating normals and features"<<endl;
  // Estimate the FPFH features for the source cloud
//  norm_est.setInputCloud (cloud_source_ptr);
//  norm_est.compute (normals);

  getNormals(snormcloud,snormals); //just extract the normals we calculated earlier
  fpfh_est.setInputCloud (cloud_source_ptr);
  fpfh_est.setInputNormals (cloud_source_ptr);
  fpfh_est.compute (features_source);

  cout<<"estimating normals and features"<<endl;
  // Estimate the FPFH features for the target cloud
//  norm_est.setInputCloud (cloud_target_ptr);
//  norm_est.compute (normals);
  getNormals(tnormcloud,tnormals); //just extract the normals we calculated earlier
  fpfh_est.setInputCloud (cloud_target_ptr);
  fpfh_est.setInputNormals (cloud_target_ptr);
  fpfh_est.compute (features_target);

  // Initialize Sample Consensus Initial Alignment (SAC-IA)
  SampleConsensusInitialAlignment<PointXYZINormal, PointXYZINormal, FPFHSignature33> reg;
  reg.setMinSampleDistance (0.05);
  reg.setMaxCorrespondenceDistance (0.2);
  reg.setMaximumIterations (1000);

  reg.setInputCloud (cloud_source_ptr);
  reg.setInputTarget (cloud_target_ptr);
  reg.setSourceFeatures (boost::make_shared<PointCloud<FPFHSignature33> > (features_source));
  reg.setTargetFeatures (boost::make_shared<PointCloud<FPFHSignature33> > (features_target));
  cout<<"aligning"<<endl;
  // Register
  reg.align (aligned);


  aligned+=tnormcloud;

  pcl::io::savePCDFile ("pcds/aligned_walls3.pcd", aligned,false);









//  timeval t0=g_tick();
//  readPCD("cloud_pplheight.pcd", cloud);
//  std::cout<<"read pcd took:  "<<g_tock(t0)<<std::endl;


//  t0=g_tick();
//  pcl::KdTreeFLANN<Point> tree;
//  tree.setInputCloud(boost::make_shared<pcl::PointCloud<Point> > (cloud));
//  vector<int> minitree(cloud.points.size(),-1);
//  vector<int> heads;
//  std::cout<<"set input took:  "<<g_tock(t0)<<std::endl;
//  vector<int> indices(1000);
//  vector<float> dists(1000);
//  t0=g_tick();
//  tree.nearestKSearch(cloud.points[0],1000,indices,dists);
//  std::cout<<"nearestKSearch took:  "<<g_tock(t0)<<std::endl;
//
//  std::vector<pcl::PointCloud<Point> > cloud_clusters;
//  segfast(cloud,cloud_clusters,.2);



//  //fast cluster:
//  double cluster_tol=.2;
//
//  t0=g_tick();
//  for(int i=0; i<cloud.points.size();i++){
//	  if(minitree[i]==-1){
//		  heads.push_back(i);
//		  if(!tree.radiusSearch(cloud,i,cluster_tol/2.3,indices,dists))
//			  cout<<"radius search failed!"<<endl;
////		  cout<<i<<" --> "<<indices.size()<<endl;
//		  for(int j=0;j<indices.size();j++){
//			  minitree[indices[j]]=heads.size()-1;
//		  }
//	  }
//
//  }
//  std::cout<<"radiusSearch took:  "<<g_tock(t0)<<"s  found "<<heads.size()<<" heads"<<std::endl;
//
////now, cluster heads:
//
//  tree.setInputCloud(boost::make_shared<pcl::PointCloud<Point> > (cloud),boost::make_shared<std::vector<int> >  (heads));
//  int searching,currenthead;
//  vector<int> heads2;
//  vector<int> minitree2(heads.size(),-1);
//  list<int> tosearch;
//  for(int i=0; i<minitree2.size();i++){
//	  if(minitree2[i]==-1){
//		  heads2.push_back(heads[i]);
//		  tosearch.push_back(i);
//		  currenthead=heads2.size()-1; //references an index in heads2
//		  minitree2[i]=currenthead;
//		  while(tosearch.size()>0){
//			  searching=tosearch.front();
//			  tosearch.pop_front();
//			  if(!tree.radiusSearch(cloud.points[heads[searching]],cluster_tol,indices,dists))
//				  cout<<"radius search failed!"<<endl;
//			  cout<<i<<" --> "<<searching<<" --> "<<indices.size()<<endl;
//			  for(int j=0;j<indices.size();j++)
//				  if(minitree2[indices[j]]==-1){//found untouched point
//					  minitree2[indices[j]]=currenthead; //claim it
//					  tosearch.push_back(indices[j]);   //add it to list of points to search
//				  }
//		  }
//	  }
//
//  }
//  std::cout<<"radiusSearch took:  "<<g_tock(t0)<<"s  found "<<heads2.size()<<" heads"<<std::endl;
//  vector<vector<int> > clusters(heads2.size());
//  for(int j=0;j<minitree.size();j++){
////	  int root=minitree2[minitree[j]]; //the root of this node
//	  clusters[minitree2[minitree[j]]].push_back(j);
//  }
//
//
//  std::cout<<"clustering took:  "<<g_tock(t0)<<"s  found "<<heads2.size()<<" heads"<<std::endl;
//  for(int j=0;j<heads2.size();j++){
//	  cout<<"cluster "<<j<<"  -->  "<<clusters[j].size()<<endl;
//  }
//
//  std::vector<pcl::PointCloud<Point> > cloud_clusters(clusters.size());
//  for(int i=0;i<clusters.size(); i++){
//	  getSubCloud(cloud,clusters[i],cloud_clusters[i]);
//  }
//  char filename[50];
//  for(int i=0;i<cloud_clusters.size(); i++){
//	  sprintf(filename,"cluster%02d.pcd",i);
//	  pcl::io::savePCDFileASCII (filename, cloud_clusters[i]);
//  }

//   pcl::PointCloud<pcl::PointXYZ> cloud_normals;
//
//  timeval t0=g_tick();
//  getNormals(cloud,cloud_normals);
//  std::cout<<"get normals took:  "<<g_tock(t0)<<std::endl;
//  cout<<"point 0: "<<cloud.points[0]<<endl<<cloud_normals.points[0]<<endl;




  return 0;
}

