/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Garratt Gallagher
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name Garratt Gallagher nor the names of other
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/



//this program is designed to analyse the data from object_survey, to get a more accurate registration of poses

//[object_survey/SurveyScan]:
//sensor_msgs/PointCloud2 cloud
//geometry_msgs/TransformStamped laser_transform
//geometry_msgs/TransformStamped camera_transform
//geometry_msgs/TransformStamped base_transform
//sensor_msgs/Image[] images
//sensor_msgs/CameraInfo caminfo
//geometry_msgs/Pose object_pose

//#include <furniture_ops/pcl_helpers.hpp>

#include "pcl/io/pcd_io.h"
#include "pcl/segmentation/extract_clusters.h"
#include <sys/time.h>
#include <list>
#include <fstream>
#include <nnn/nnn.hpp>


#include <pcl_tools/segfast.hpp>
#include <pcl_tools/clusterevaluation.hpp>



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Decompose a Point Cloud into clusters based on the Euclidean distance between points. Uses Hierarchical Clustering, making it faster
  * \param cloud the point cloud message
  * \param cluster_tol the spatial cluster tolerance as a measure in L2 Euclidean space
  * \param cclusters the resultant clusters containing point indices (as a vector of vector of ints)
  * \param min_pts_per_cluster minimum number of points that a cluster may contain (default = 1)
  */
template <typename PointT>
void extractEuclideanClustersFastNNN(pcl::PointCloud<PointT> &cloud, std::vector<std::vector<int> > &clusters, double cluster_tol=.2){
	int min_pts_per_cluster=1;
   double smaller_tol = cluster_tol/2.1;
   SplitCloud2<PointT> sc2a(cloud,cluster_tol*2.0);
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
           ROS_WARN("radius search failed!");
        for(uint j=0;j<indices.size();j++){
           minitree[indices[j]]=heads.size()-1; //assign these points to the current head.
                                       // this overwrites previous claims, but it's ok
        }
     }
   }

   //now, we have much fewer points to cluster, but since the initial downsampling was less than
   //half the cluster tolerance,we are guaranteed to find all the points within the tolerance
   //(except if  [ clustertol < distance between heads < cluster_tol+ smaller cluster tol] AND the head closest points in the two heads are < cluster_tol
   //The next code block deals with that...
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
           for(uint j=0;j<indices.size();j++)
              if(minitree2[indices[j]]==-1){//found untouched point (which means this root touched it)
                 minitree2[indices[j]]=currenthead; //claim it
                 tosearch.push_back(indices[j]);   //add it to list of points to search
              }
        }
     }

   }
   clusters.resize(heads2.size());
   //now, for each point in the cloud find its head --> then the head it clustered to. that is its cluster id!
   for(uint j=0;j<minitree.size();j++){
     clusters[minitree2[minitree[j]]].push_back(j);
   }
   //now we need to check to see if any of the heads that were NOT clustered together are closer than cluster_tol+smaller_tol.
   //This covers the exception noted in the code block above
   //this is where an adversary could really kill this algorithm, since this check could be polynomial in cloud size. in real circumstances, it is very quick.
   vector<int> indices1;
   vector<float> dists1;
   double distthresh=cluster_tol*cluster_tol; //radius search gives squared distances...
   double automergethresh=cluster_tol*cluster_tol/4.0; //pts are always in same clustr if they are both less than half the tolerance away from the same point
   for(uint i=0; i<minitree2.size();i++){ //for every head
      tree2.radiusSearch(cloud.points[heads[i]],cluster_tol+2*smaller_tol,indices,dists);
      for(uint j=0;j<indices.size();j++){ //for every head that is close to this head
         if(minitree2[indices[j]] != minitree2[i]){//if the two heads are not in the same cluster (and only check each combo once)
            //now we have to find if there is a point between head a and head b that is < cluster_tol from both
            //our best chance of finding it is to start searching at a point halfway between them
            PointT heada=cloud.points[heads[i]], headb=cloud.points[heads[indices[j]]];
            PointT inbetween;
            inbetween.x=(heada.x+headb.x)/2.0;
            inbetween.y=(heada.y+headb.y)/2.0;
            inbetween.z=(heada.z+headb.z)/2.0;
            if(!tree.radiusSearch(inbetween,cluster_tol,indices1,dists1)){ //search in the full tree
               //nothing close to either cluster. these clusters are separate, so don't consider
               continue; //go to next j
            }
            //must match all points that return against each other
            bool shouldmerge=false;
            int mergefrom,mergeinto;
            for(int k=0;k<indices1.size()-1;k++){
                for(uint m=k+1;m<indices1.size();m++){
                  if(minitree2[minitree[indices1[k]]] != minitree2[minitree[indices1[m]]]) //if different clusters
                     if((dists1[k]<automergethresh && dists1[m]<automergethresh) ||
                     (squaredEuclideanDistance(cloud.points[indices1[k]],cloud.points[indices1[m]] ) < distthresh)){
                        mergefrom=minitree2[minitree[indices1[k]]];
                        mergeinto=minitree2[minitree[indices1[m]]];
                        if((mergefrom !=minitree2[i] && mergefrom !=minitree2[indices[j]]) || (mergeinto !=minitree2[i] && mergeinto !=minitree2[indices[j]])){
//                          if(globalverbose)
//                             ROS_WARN("point %d in cluster %d is %f from point %d in cluster %d, but these points won't be merged. since we are concidering %d and %d.",
//                                   indices1[k],minitree2[minitree[indices1[k]]],
//                                   euclideanDistance(cloud.points[indices1[k]],cloud.points[indices1[m]]),
//                                   indices1[m],minitree2[minitree[indices1[m]]],minitree2[i],minitree2[indices[j]]);
                        }
                        else{
//                           if(globalverbose)
//                              ROS_WARN("point %d in cluster %d is %f from point %d in cluster %d",
//                                    indices1[k],minitree2[minitree[indices1[k]]],
//                                    euclideanDistance(cloud.points[indices1[k]],cloud.points[indices1[m]]),
//                                    indices1[m],minitree2[minitree[indices1[m]]]);
                           shouldmerge=true;
                           break;  //TODO: shouldn't actually break, but rather see if multiple clusters should merge
                        }
                     }

                }
               if(shouldmerge){

                  //there is a point that these two clusters share -> they should be merged
//                  if(globalverbose)
//                     ROS_INFO("head %d (in cluster %d) shares a point with head %d (in cluster %d). Merging cluster %d (%d pts) into cluster %d (%d pts)",
//                       i,minitree2[i],indices[j],minitree2[indices[j]],minitree2[indices[j]],clusters[minitree2[indices[j]]].size(), minitree2[i],  clusters[minitree2[i]].size());
                  //rename all heads in cluster b to cluster a
                  int clusterb=minitree2[indices[j]];
                  for(uint m=0;m<minitree2.size();m++){
                     if(minitree2[m]==clusterb) minitree2[m]=minitree2[i];
                  }
                  break;
               } //if the point is shared
            } //for every point close to both heads
         } //if a head is close, and from a different cluster
      }//for every nearby head
   }  //for every head

   clusters.clear();
   clusters.resize(heads2.size());
   //now, for each point in the cloud find its head --> then the head it clustered to. that is its cluster id!
   for(uint j=0;j<minitree.size();j++){
     clusters[minitree2[minitree[j]]].push_back(j);
   }
   //erase the deleted clusters, and the ones under the minimum size
   std::vector<int> deletedclusters;
   for(int i=clusters.size()-1;i>=0; i--)
      if(clusters[i].size() < min_pts_per_cluster){
         clusters.erase(clusters.begin()+i);
         deletedclusters.push_back(i);
      }
}





template <typename PointT>
void testexisting(pcl::PointCloud<PointT> &smallcloud, std::vector< std::vector<int> > &inds,double cluster_tol ){
      pcl::EuclideanClusterExtraction<PointT> clusterer;
     clusterer.setClusterTolerance(cluster_tol);
     clusterer.setMinClusterSize(1);
     clusterer.setMaxClusterSize(1000000);
     clusterer.setInputCloud(smallcloud.makeShared());
     std::vector<pcl::PointIndices> ptinds;
     clusterer.extract(ptinds);
     inds.resize(ptinds.size());
 	 for(uint i=0;i<ptinds.size();++i)
 		inds[i]=ptinds[i].indices;
}

//template <typename PointT>
//double testExtractFast(pcl::PointCloud<PointT> &smallcloud, std::vector< std::vector<int> > &inds,double cluster_tol){
//	timeval t0=g_tick();
//    extractEuclideanClustersFast2(smallcloud,inds,cluster_tol);
//    return g_tock(t0);
//}













void printUsage(string progname){
	std::cout<<"USAGE "<<progname<<" filename.pcd <cluster_tolerance> [options]"<<std::endl;
//	std::cout<<"Options: "<<std::endl;
//	std::cout<<"-l1 : Load pre-computed results of standard clustering algorithm"<<std::endl;
//	std::cout<<"-l2 : Load pre-computed results of new clustering algorithm"<<std::endl;
//	std::cout<<"-s1 : Save computed results of standard clustering algorithm"<<std::endl;
//	std::cout<<"-s2 : Save computed results of new clustering algorithm"<<std::endl;
//	std::cout<<"-debug : run evaluative version of new algorithm to find out why it broke"<<std::endl;
//	std::cout<<"-time : run evaluative version of new algorithm to find out what takes the time"<<std::endl;
//	std::cout<<"-nocompare : don't compare the evaluations.  Will only compute results if it is being saved"<<std::endl;
}


int main(int argc, char **argv) {
	if(argc<3){
		printUsage(argv[0]);
		return -1;
	}
	ClusterEvaluation<pcl::PointXYZ> evaluation(argv[1],atof(argv[2]));

//	std::cout<<"Loading std: "<<std::endl;
//	evaluation.loadResults("std");
//	std::cout<<"Loading new: "<<std::endl;
//	evaluation.loadResults("new");
	evaluation.testAlgorithm("std",&testexisting);
	evaluation.testAlgorithm("new",&extractEuclideanClustersFastNNN);
//	std::cout<<"unwrapping: "<<std::endl;
	std::vector<int> labels;
	unwrapClusters(evaluation.clusterings[0].inds,labels);
	std::cout<<"evaluating: "<<std::endl;
	evaluation.evaluateResults();
	evaluation.printResults();


	return 0;
}





