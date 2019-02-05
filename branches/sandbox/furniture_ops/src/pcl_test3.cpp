/*
 * pcl_test3.cpp
 *
 *  Created on: Oct 18, 2010
 *      Author: garratt
 */

#include <pcl/filters/statistical_outlier_removal.h>
#include "pcl/ModelCoefficients.h"

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/extract_indices.h"

#include "pcl/filters/passthrough.h"
#include "pcl/features/normal_3d.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_ann.h"
#include "pcl/kdtree/organized_data.h"
#include <list>
#include <vector>


//finds point in target cloud that are near points in ref_cloud
//this works by searching for nearest neighbors of a bunch of points in the ref cloud
//it is approximate, because we don't really check every point in ref_cloud
int MymatchPoints(pcl::PointCloud<pcl::PointXYZINormal> &ref_cloud, pcl::PointCloud<pcl::PointXYZINormal> &target_cloud, vector<int> &t_inds, double cluster_tol=.2){
   double smaller_tol = cluster_tol/2.3;
//   timeval t0=g_tick();
   pcl::KdTreeFLANN<pcl::PointXYZINormal> rtree,ttree;
   rtree.setInputCloud(ref_cloud.makeShared());
   ttree.setInputCloud(target_cloud.makeShared());
   vector<int> minitree(ref_cloud.points.size(),-1), nearstatus(target_cloud.points.size(),0);
//   std::vector<int> t_inds;
   vector<int> indices;
   vector<float> dists;
   vector<int> heads;

   //First 'downsample' the points into clusters slightly less than half the cluster tolerance
   //minitree keeps track of who each point belongs to (which is an index of the 'head' array)
   for(uint i=0; i<ref_cloud.points.size();i++){
     if(minitree[i]==-1){

    	 heads.push_back(i);
    	 //first, find all the points in target_cloud near this point
//        ttree.radiusSearch(ref_cloud.points[i],cluster_tol,indices,dists); //don't check for failures, since could mean no points
//        for(uint j=0;j<indices.size();j++)
//        	nearstatus[indices[j]]=1; //mark these points as 'near'
      //now find all the points in ref_cloud that are nearby, so we don't search this area again
        if(!rtree.radiusSearch(ref_cloud.points[i],smaller_tol,indices,dists)) { cout<<"radius search failed!"<<endl; return -1; }
		for(uint j=0;j<indices.size();j++)
			minitree[indices[j]]=0; //mark these points as 'searched'
     }
   }

   for(uint i=0; i<heads.size();i++){
	   ttree.radiusSearch(ref_cloud.points[heads[i]],cluster_tol,indices,dists); //don't check for failures, since could mean no points
	   for(uint j=0;j<indices.size();j++)
		nearstatus[indices[j]]=1; //mark these points as 'near'
   }

   //now convert nearstatus into an array of indices
   int tcount=0;
   for(uint i=0; i<target_cloud.points.size();i++){
	   if(nearstatus[i]){
		   t_inds.push_back(i);
		   tcount++;
	   }
   }
   t_inds.resize(tcount);
   std::cout<<"finding nearcloud:  found "<<tcount<<" out of "<<target_cloud.points.size()<<" pts"<<std::endl;
   return 0;
}


int main(){
	pcl::PointCloud<pcl::PointXYZINormal> ref,target;
	   pcl::PCDReader reader;
	   sensor_msgs::PointCloud2 cloud;
	   reader.read ("cloud1.pcd", cloud);
	   pcl::fromROSMsg (cloud,ref);
	   reader.read ("cloud2.pcd", cloud);
	   pcl::fromROSMsg (cloud,target);
	   std::vector<int> inds;
	   MymatchPoints(ref,target,inds);





}


