/*
 * pcl_helpers.hpp
 *
 *  Created on: Oct 13, 2010
 *      Author: garratt
 */
#ifndef PCL_HELPERS_H_
#define PCL_HELPERS_H_

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





template <typename type1,typename type2 >
void cp3dPt(type1 &dest, type2 &src ){
   dest.x=src.x;
   dest.y=src.y;
   dest.z=src.z;
}
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

void myFlipNormals(double vx, double vy, double vz, pcl::PointCloud<pcl::PointXYZINormal> &ncloud){
   for(uint i=0;i<ncloud.points.size();i++){
      pcl::PointXYZINormal p=ncloud.points[i];
      double dotprod=(p.normal[0]*(vx-p.x)+p.normal[1]*(vy-p.y)+p.normal[2]*(vz-p.z));
      if(dotprod<0){
//       cout<<"flipping"<<endl;
         ncloud.points[i].normal[0]*=-1.0;
         ncloud.points[i].normal[1]*=-1.0;
         ncloud.points[i].normal[2]*=-1.0;
      }
   }
}


void getNormals(pcl::PointCloud<pcl::PointWithViewpoint> &cloudin, pcl::PointCloud<pcl::PointXYZINormal> &cloud_normals){
   timeval t0=g_tick();
   pcl::PointCloud<pcl::PointWithViewpoint>::ConstPtr cloud_ = boost::make_shared<const pcl::PointCloud<pcl::PointWithViewpoint> > (cloudin);
   pcl::KdTree<pcl::PointWithViewpoint>::Ptr normals_tree_;
    int k_ = 10;                                // 50 k-neighbors by default
    normals_tree_ = boost::make_shared<pcl::KdTreeANN<pcl::PointWithViewpoint> > ();
    pcl::NormalEstimation<pcl::PointWithViewpoint, pcl::PointXYZINormal> n3d_;
    n3d_.setKSearch (k_);
    n3d_.setSearchMethod (normals_tree_);
    n3d_.setInputCloud (cloud_);
    n3d_.compute (cloud_normals);
    for(uint i=0;i<cloudin.points.size(); i++){
      cloud_normals.points[i].x=cloudin.points[i].x;
      cloud_normals.points[i].y=cloudin.points[i].y;
      cloud_normals.points[i].z=cloudin.points[i].z;
    }
    myFlipNormals(cloudin.points[0].vp_x,cloudin.points[0].vp_y,cloudin.points[0].vp_z,cloud_normals);
   std::cout<<"finding normals took:  "<<g_tock(t0)<<"  for "<<cloud_normals.points.size()<<" indices."<<std::endl;
}


template <typename PointT>
void getSubCloud(pcl::PointCloud<PointT> &cloudin,  std::vector<int> &ind, pcl::PointCloud<PointT> &cloudout,bool setNegative=false){
   pcl::ExtractIndices<PointT> extract;
   // Extract the inliers
   extract.setInputCloud (boost::make_shared<pcl::PointCloud<PointT> > (cloudin));
   extract.setIndices (boost::make_shared<std::vector<int> > (ind));
   extract.setNegative (setNegative);
   extract.filter (cloudout);
//    ROS_INFO ("Subcloud representing the planar component: %d data points.", cloudout.width * cloudout.height);

}

//could do this with a passthrough, but come on...
template <typename PointT>
void segmentFloor(pcl::PointCloud<PointT> &cloudin, pcl::PointCloud<PointT> &cloudout,bool negativeFilter=true){
   timeval t0=g_tick();
   std::vector<int> ind;
   for(uint i=0;i<cloudin.points.size(); i++){
      if(abs(cloudin.points[i].z) <.15){
         ind.push_back(i);
      }
   }
   getSubCloud(cloudin,ind,cloudout,negativeFilter);
   std::cout<<"segmentFloor took:  "<<g_tock(t0)<<"  for "<<cloudout.points.size()<<" indices."<<std::endl;
}


//could do this with a passthrough, but come on...
template <typename PointT>
void segmentPplHeight(pcl::PointCloud<PointT> &cloudin, pcl::PointCloud<PointT> &cloudout){
   timeval t0=g_tick();
   std::vector<int> ind;
   for(uint i=0;i<cloudin.points.size(); i++){
      if(cloudin.points[i].z > 0.0 && cloudin.points[i].z < 2.0){
         ind.push_back(i);
      }
   }
   getSubCloud(cloudin,ind,cloudout,false);
   std::cout<<"segmentPplHeight took:  "<<g_tock(t0)<<"  for "<<cloudout.points.size()<<" indices."<<std::endl;
}

template <typename PointT>
void segmentHeight(pcl::PointCloud<PointT> &cloudin, pcl::PointCloud<PointT> &cloudout,double height){
   timeval t0=g_tick();
   std::vector<int> ind;
   for(uint i=0;i<cloudin.points.size(); i++){
      if(cloudin.points[i].z > height){ //|| cloudin.points[i].z <.1){
         ind.push_back(i);
      }
   }
   getSubCloud(cloudin,ind,cloudout,false);
   std::cout<<"segmentHeight took:  "<<g_tock(t0)<<"  for "<<cloudout.points.size()<<" indices."<<std::endl;
}

template <typename PointT>
void removeOutliers(pcl::PointCloud<PointT> &cloudin, pcl::PointCloud<PointT> &cloudout){
   timeval t0=g_tick();
   pcl::StatisticalOutlierRemoval<PointT> sor2;
   sor2.setInputCloud (boost::make_shared<pcl::PointCloud<PointT> >(cloudin));
   sor2.setMeanK (50);
   sor2.setStddevMulThresh (1.0);
   sor2.filter (cloudout);
   std::cout<<"filtering outliers took:  "<<g_tock(t0)<<"  for "<<cloudout.points.size()<<" indices."<<std::endl;
}

//template <typename PointT>
//void downSample(pcl::PointCloud<PointT> &cloudin, pcl::PointCloud<PointT> &cloudout){
//   timeval t0=g_tick();
//   // Create the filtering object
//   pcl::VoxelGrid<PointT> sor;
//   sor.setInputCloud (boost::make_shared<pcl::PointCloud<PointT> > (cloudin));
//   sor.setLeafSize (0.01, 0.01, 0.01);
//   sor.filter (cloudout);
//   std::cout<<"downsampling took:  "<<g_tock(t0)<<"  for "<<cloudout.points.size()<<" indices."<<std::endl;
//}

template <typename PointT>
void downSample(pcl::PointCloud<PointT> &cloudin, pcl::PointCloud<PointT> &cloudout, double leafsize=.01){
   timeval t0=g_tick();
   // Create the filtering object
   pcl::VoxelGrid<PointT> sor;
   sor.setInputCloud (boost::make_shared<pcl::PointCloud<PointT> > (cloudin));
   sor.setLeafSize (leafsize, leafsize, leafsize);
   sor.filter (cloudout);
   std::cout<<"downsampling took:  "<<g_tock(t0)<<"  for "<<cloudout.points.size()<<" indices."<<std::endl;
}
template <typename PointT>
double ptdist(PointT a,PointT b){
   return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z);
}


template <typename PointT>
void segfast(pcl::PointCloud<PointT> &cloud, std::vector<pcl::PointCloud<PointT> > &cloud_clusters, double cluster_tol=.2){
   double smaller_tol = cluster_tol/2.3;
   timeval t0=g_tick();
   pcl::KdTreeFLANN<PointT> tree,tree2;
   tree.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> > (cloud));
   vector<int> minitree(cloud.points.size(),-1);
   vector<int> heads,indices;
   vector<float> dists;

   //First 'downsample' the points into clusters slightly less than half the cluster tolerance
   //minitree keeps track of who each point belongs to (which is an index of the 'head' array)
   for(uint i=0; i<cloud.points.size();i++){
     if(minitree[i]==-1){    //if no one has claimed this point, make it a head
        heads.push_back(i);
        if(!tree.radiusSearch(cloud,i,smaller_tol,indices,dists))  //find all the points close to this point
           cout<<"radius search failed!"<<endl;
        for(uint j=0;j<indices.size();j++){
           minitree[indices[j]]=heads.size()-1; //assign these points to the current head.
                                       // this overwrites previous claims, but it's ok
        }
     }
   }
// std::cout<<"radiusSearch took:  "<<g_tock(t0)<<"s  found "<<heads.size()<<" heads"<<std::endl;

   //now, we have much fewer points to cluster, but since the initial downsampling was less than
   //half the cluster tolerance,we are guaranteed to find all the points within the tolerance
   //(except if  [ clustertol < distance between heads < cluster_tol+ smaller cluster tol] AND the head closest points in the two heads are < cluster_tol
   //I need to make something to deal with that...
   tree2.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> > (cloud),boost::make_shared<std::vector<int> >  (heads));
   int searching,currenthead;
   //heads2 is the cluster id.  we now search to make sure we check all the points in our cluster
   //minitree2 corresponds to the heads array, so the points in minitree2 are at heads[i]
   std::vector<int> heads2, minitree2(heads.size(),-1);
   std::list<int> tosearch;
   for(uint i=0; i<minitree2.size();i++){
     if(minitree2[i]==-1){
        heads2.push_back(heads[i]);
        tosearch.push_back(i);
        currenthead=heads2.size()-1; //references an index in heads2
        minitree2[i]=currenthead;
        while(tosearch.size()>0){
           searching=tosearch.front();
           tosearch.pop_front();
           if(!tree2.radiusSearch(cloud.points[heads[searching]],cluster_tol,indices,dists))
              cout<<"radius search failed!"<<endl;
//         cout<<i<<" --> "<<searching<<" --> "<<indices.size()<<endl;
           for(uint j=0;j<indices.size();j++)
              if(minitree2[indices[j]]==-1){//found untouched point (which means this root touched it)
                 minitree2[indices[j]]=currenthead; //claim it
                 tosearch.push_back(indices[j]);   //add it to list of points to search
              }
        }
     }

   }

   timeval t1=g_tick();
   vector<int> indices1;
   vector<float> dists1;
   vector<int> deletedclusters;
   //now we need to check to see if any of the heads that were NOT clustered together are closer than cluster_tol+smaller_tol:
   for(uint i=0; i<minitree2.size();i++){
      tree2.radiusSearch(cloud.points[heads[i]],cluster_tol+smaller_tol,indices,dists);
      for(uint j=0;j<indices.size();j++)
         if(minitree2[indices[j]] != minitree2[i] && i<j){//if the two heads are not in the same cluster
            cout<<"head "<<i<<" ("<<minitree2[i]<<") is "<<sqrt(dists[j])<<" from head "<<indices[j]<<" ("<<minitree2[indices[j]]<<")"<<endl;
            //now we have to find if there is a point between head a and head b that is < cluster_tol from both
            PointT heada=cloud.points[heads[i]], headb=cloud.points[heads[indices[j]]];
            PointT inbetween;
            inbetween.x=(heada.x+headb.x)/2.0;
            inbetween.y=(heada.y+headb.y)/2.0;
            inbetween.z=(heada.z+headb.z)/2.0;

            tree.radiusSearch(inbetween,cluster_tol,indices1,dists1); //search in the full tree
            double distthresh=cluster_tol*cluster_tol; //radius search gives squared distances...
            for(uint k=0;k<indices1.size();k++){
               if(ptdist(cloud.points[indices1[k]],heada ) < distthresh && ptdist(cloud.points[indices1[k]],headb ) < distthresh ){
                  //there is a point that these two clusters share -> they should be merged
                  cout<<"head "<<i<<" ("<<minitree2[i]<<") shares "<<indices1[k]<<" with head "<<indices[j]<<" ("<<minitree2[indices[j]]<<")"<<endl;
                  //rename all heads in cluster b to cluster a
                  int clusterb=minitree2[indices[j]];
                  for(uint m=0;m<minitree2.size();m++){
                     if(minitree2[m]==clusterb) minitree2[m]=minitree2[i];
                  }
                  deletedclusters.push_back(clusterb);
                  break;
               }

            }

         }
   }

   std::cout<<"checking overlaps took:  "<<g_tock(t1)<<"s  found "<<deletedclusters.size()<<" overlaps"<<std::endl;





   std::cout<<"radiusSearch took:  "<<g_tock(t0)<<"s  found "<<heads2.size()<<" heads"<<std::endl;
   vector<vector<int> > clusters(heads2.size());
   //now, for each point in the cloud find it's head --> then the head it clustered to. that is it's cluster id!
   for(uint j=0;j<minitree.size();j++){
     clusters[minitree2[minitree[j]]].push_back(j);
   }


   std::cout<<"clustering took:  "<<g_tock(t0)<<"s  found "<<heads2.size()<<" heads"<<std::endl;
   for(uint j=0;j<heads2.size();j++){
     cout<<"cluster "<<j<<"  -->  "<<clusters[j].size()<<endl;
   }

   //erase the deleted clusters
   bool loop=true;
   while(loop){
      loop=false;
      for(uint i=0;i<clusters.size(); i++)
         if(clusters[i].size()==0){
            clusters.erase(clusters.begin()+i);
            loop=true;
         }


   }

   cloud_clusters.resize(clusters.size());
   for(uint i=0;i<clusters.size(); i++){
     getSubCloud(cloud,clusters[i],cloud_clusters[i]);
   }

}


template <typename PointT>
double getClosestPoint(pcl::PointCloud<PointT> &cloud, PointT ref, PointT &point){
   pcl::KdTreeFLANN<PointT> tree;
   tree.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> > (cloud));
   std::vector<int> ind(20);
   std::vector<float> dists(20);
   tree.nearestKSearch(ref,1,ind,dists);
   point=cloud.points[ind[0]];
   return dists[0];
}


//finds point in target cloud that are near points in ref_cloud
//this works by searching for nearest neighbors of a bunch of points in the ref cloud
//it is approximate, because we don't really check every point in ref_cloud
template <typename PointT>
int matchPoints(pcl::PointCloud<PointT> &ref_cloud, pcl::PointCloud<PointT> &target_cloud, pcl::PointCloud<PointT> &cloudout, double cluster_tol=.2){
   double smaller_tol = cluster_tol/2.3;
   timeval t0=g_tick();
   pcl::KdTreeFLANN<PointT> rtree,ttree;
   rtree.setInputCloud(ref_cloud.makeShared());
   ttree.setInputCloud(target_cloud.makeShared());
   vector<int> minitree(ref_cloud.points.size(),-1), nearstatus(target_cloud.points.size(),0);
   vector<int> indices,heads;
   vector<float> dists;

   //First 'downsample' the points into clusters slightly less than half the cluster tolerance
   //minitree keeps track of who each point belongs to (which is an index of the 'head' array)
   for(uint i=0; i<ref_cloud.points.size();i++){
     if(minitree[i]==-1){
    	 heads.push_back(i);
        //find all the points in ref_cloud that are nearby
        if(!rtree.radiusSearch(ref_cloud.points[i],smaller_tol,indices,dists)) { cout<<"radius search failed!"<<endl; return -1; }
		for(uint j=0;j<indices.size();j++)
			minitree[indices[j]]=0; //mark these points as 'searched'
		minitree[i]=1;  //if we want to remember where we searched.
     }
   }
   //now find the points in the target that are close to the ref:
   for(uint i=0; i<heads.size();i++){
	   ttree.radiusSearch(ref_cloud.points[heads[i]],cluster_tol,indices,dists); //don't check for failures, since could mean no points
	   for(uint j=0;j<indices.size();j++)
		nearstatus[indices[j]]=1; //mark these points as 'near'
   }


   //now convert nearstatus into an array of indices
   int tcount=0;
   std::vector<int> t_inds;
   for(uint i=0; i<target_cloud.points.size();i++){
	   if(nearstatus[i]){
		   t_inds.push_back(i);
		   tcount++;
	   }
   }
   t_inds.resize(tcount);
   getSubCloud(target_cloud,t_inds,cloudout);
   std::cout<<"finding nearcloud took:  "<<g_tock(t0)<<"s  found "<<tcount<<" out of "<<target_cloud.points.size()<<" pts"<<std::endl;
   return 0;
}

#endif
