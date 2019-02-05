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

#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/segmentation/extract_clusters.h"
#include <sys/time.h>
#include <list>
#include <fstream>
//
//#include "pcl/segmentation/extract_clusters.hpp"
//#include "ScanAnalyzer.h"


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

  //keeps  track of clustering result
struct PtMap{
	std::vector<int> ptindices; //map indices used in this struct to the outside (possibly to the real point cloud)
	std::map<int,int> 	 rmap;  //this maps the outside indices to inside: i = rmap[ptindices[i]]
							//note that o = ptindices[rmap[o]] might not work, since

	std::vector<int> clusterindices; //indicates which cluster each pt belongs to
	std::vector<int> heads;  // the index of the 'head' of each cluster, i.e where the radius search originated

	void setInds(std::vector<int> &inds){
		ptindices=inds;
		for(uint i=0;i<ptindices.size();i++)
			rmap[ptindices[i]]=i;
	}

	//simply runs radius search to grab all the points nearby until everything is taken
	//results in each pt assigned a label, and a list of who grabbed each point
	template <typename PointT>
	void simpleDownsample(pcl::PointCloud<PointT> &cloud, double cluster_tol=.2){
	   pcl::KdTreeFLANN<PointT> tree;
	   tree.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> > (cloud), boost::make_shared<std::vector<int> > (ptindices));
	   std::vector<int> indices;
	   std::vector<float> dists;
	   heads.clear();
	   clusterindices.resize(ptindices.size(),-1);
	   for(uint i=0; i<clusterindices.size();i++){
		 if(clusterindices[i]==-1){    //if no one has claimed this point, make it a head
			heads.push_back(i);
			if(!tree.radiusSearch(cloud.points[ptindices[i]],cluster_tol,indices,dists))  //find all the points close to this point
			   ROS_WARN("radius search failed!");
			for(uint j=0;j<indices.size();j++){
				clusterindices[rmap[indices[j]]]=heads.size()-1; //assign these points to the current head.
										   // this overwrites previous claims, but it's ok
			}
		 }
	   }
	}

};



//template <typename PointT>
//void simpleDownsample(pcl::PointCloud<PointT> &cloud, PtMap &mapping, double cluster_tol=.2){
//	   pcl::KdTreeFLANN<PointT> tree;
//	   tree.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> > (cloud), boost::make_shared<std::vector<int> > (mapping.ptindices));
//	   vector<int> indices;
//	   vector<float> dists;
//	   heads.clear();
//	   clusterindices.resize(cloud.points.size(),-1);
//	   for(uint i=0; i<clusterindices.size();i++){
//	     if(clusterindices[i]==-1){    //if no one has claimed this point, make it a head
//	        heads.push_back(i);
//	        if(!tree.radiusSearch(cloud,i,cluster_tol,indices,dists))  //find all the points close to this point
//	           ROS_WARN("radius search failed!");
//	        for(uint j=0;j<indices.size();j++){
//	        	clusterindices[indices[j]]=heads.size()-1; //assign these points to the current head.
//	                                       // this overwrites previous claims, but it's ok
//	        }
//	     }
//	   }
//}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Decompose a Point Cloud into clusters based on the Euclidean distance between points. Uses Hierarchical Clustering, making it faster
  * \param cloud the point cloud message
  * \param cluster_tol the spatial cluster tolerance as a measure in L2 Euclidean space
  * \param cclusters the resultant clusters containing point indices (as a vector of vector of ints)
  * \param min_pts_per_cluster minimum number of points that a cluster may contain (default = 1)
  */
template <typename PointT>
void extractEuclideanClustersFast2(pcl::PointCloud<PointT> &cloud, std::vector<std::vector<int> > &clusters, double cluster_tol=.2, int min_pts_per_cluster=1){
   bool globalverbose=false;
//   double smaller_tol = cluster_tol/2.3;
   double smaller_tol = cluster_tol/2.1;
   pcl::KdTreeFLANN<PointT> tree,tree2;
   tree.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> > (cloud));
   std::vector<int> minitree(cloud.points.size(),-1);
   std::vector<int> heads,indices;
   std::vector<float> dists;

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
              std::cout<<"radius search failed!"<<std::endl;
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
   std::vector<int> indices1;
   std::vector<float> dists1;
   double distthresh=cluster_tol*cluster_tol; //radius search gives squared distances...
   double automergethresh=cluster_tol*cluster_tol/4.0; //pts are always in same clustr if they are both less than half the tolerance away from the same point
   timeval t1;
   double time1,time2;
   bool verbose=false;
   for(uint i=0; i<minitree2.size();i++){ //for every head
      verbose=false;
//	  if(i==1452 || i==1508 || i==1509) verbose=true;
//	  else verbose=false;
	  if(verbose) std::cout<<"head "<<i<<"  in cluster "<<minitree2[i]<<std::endl;
		 tree2.radiusSearch(cloud.points[heads[i]],cluster_tol+2*smaller_tol,indices,dists);
	  if(verbose) {
	     std::cout<<"  radius search found: "<<indices.size()<<"  matches"<<std::endl;
//		 for(uint rind=0;rind<indices.size();rind++)
//			 cout<<" -> "<<indices[rind]<<" in cluster "<<minitree2[indices[rind]]<<std::endl;

	  }
//      tree2.radiusSearch(cloud.points[heads[i]],cluster_tol+smaller_tol,indices,dists);
      for(uint j=0;j<indices.size();j++){ //for every head that is close to this head
  	     if(verbose)
  	      std::cout<<" -> "<<indices[j]<<" in cluster "<<minitree2[indices[j]]<<std::endl;
         if(minitree2[indices[j]] != minitree2[i]){//if the two heads are not in the same cluster (and only check each combo once)
            t1=g_tick();
            if(verbose)
                  ROS_INFO("head %d (in cluster %d) is %f from head %d (in cluster %d). Checking if the two heads share points.",
        			i,minitree2[i],sqrt(dists[j]),indices[j],minitree2[indices[j]]);
            //now we have to find if there is a point between head a and head b that is < cluster_tol from both
            //our best chance of finding it is to start searching at a point halfway between them
            PointT heada=cloud.points[heads[i]], headb=cloud.points[heads[indices[j]]];
            PointT inbetween;
            inbetween.x=(heada.x+headb.x)/2.0;
            inbetween.y=(heada.y+headb.y)/2.0;
            inbetween.z=(heada.z+headb.z)/2.0;
            //the farthest 2 points could be from the centroid of the two heads and still be less than
            //cluster_tol apart is: cluster_tol + smaller_tol -dist_between_heads/2)
//            std::cout<<cluster_tol+smaller_tol-sqrt(dists[j])<<std::endl;
//            double searchrad=cluster_tol+smaller_tol-sqrt(dists[j]);
//            indices1.resize(0);

            if(!tree.radiusSearch(inbetween,cluster_tol,indices1,dists1)){ //search in the full tree
               if(globalverbose)
                  ROS_ERROR("Radius Search Failed");
               //nothing close to either cluster. these clusters are separate, so don't consider
               continue; //go to next j
            }
            time1=g_tock(t1);
            t1=g_tick();
            //must match all points that return against each other
            bool shouldmerge=false;
            int mergefrom,mergeinto;
            for(int k=0;k<(int)indices1.size()-1;k++){
                for(uint m=k+1;m<indices1.size();m++){
                	if(minitree2[minitree[indices1[k]]] != minitree2[minitree[indices1[m]]]) //if different clusters
                		if((dists1[k]<automergethresh && dists1[m]<automergethresh) ||
           				(squaredEuclideanDistance(cloud.points[indices1[k]],cloud.points[indices1[m]] ) < distthresh)){
                		   mergefrom=minitree2[minitree[indices1[k]]];
                		   mergeinto=minitree2[minitree[indices1[m]]];
                		   if((mergefrom !=minitree2[i] && mergefrom !=minitree2[indices[j]]) || (mergeinto !=minitree2[i] && mergeinto !=minitree2[indices[j]])){
                		     if(globalverbose)
                		        ROS_WARN("point %d in cluster %d is %f from point %d in cluster %d, but these points won't be merged. since we are concidering %d and %d.",
                                   indices1[k],minitree2[minitree[indices1[k]]],
                                   euclideanDistance(cloud.points[indices1[k]],cloud.points[indices1[m]]),
                                   indices1[m],minitree2[minitree[indices1[m]]],minitree2[i],minitree2[indices[j]]);
                		   }
                		   else{
                           if(globalverbose)
                              ROS_WARN("point %d in cluster %d is %f from point %d in cluster %d",
                                    indices1[k],minitree2[minitree[indices1[k]]],
                                    euclideanDistance(cloud.points[indices1[k]],cloud.points[indices1[m]]),
                                    indices1[m],minitree2[minitree[indices1[m]]]);
                           shouldmerge=true;
                           break;  //TODO: shouldn't actually break, but rather see if multiple clusters should merge
                		   }
                		}

                }

//            	if(verbose){
//					cout<<"    -> dist: ("<<sqrt(dists1[k])<<") "<<indices1[k]<<" in cluster "<<minitree2[minitree[indices1[k]]];
//					cout<<"  "<<euclideanDistance(cloud.points[indices1[k]],heada )<<"  from "<<heads[i];
//					cout<<"  "<<euclideanDistance(cloud.points[indices1[k]],headb )<<"  from "<<heads[indices[j]]<<std::endl;
//            	}
               if(shouldmerge){

                  //there is a point that these two clusters share -> they should be merged
                  if(globalverbose)
                     ROS_INFO("head %d (in cluster %d) shares a point with head %d (in cluster %d). Merging cluster %d (%d pts) into cluster %d (%d pts)",
            			  i,minitree2[i],indices[j],minitree2[indices[j]],minitree2[indices[j]],(int)clusters[minitree2[indices[j]]].size(), minitree2[i],  (int)clusters[minitree2[i]].size());
                  //rename all heads in cluster b to cluster a
                  int clusterb=minitree2[indices[j]];
                  for(uint m=0;m<minitree2.size();m++){
                     if(minitree2[m]==clusterb) minitree2[m]=minitree2[i];
                  }
                  break;
               } //if the point is shared

//               cout<<"       "<<k<<"  "<<g_tock(t1)<<std::endl;
            } //for every point close to both heads
            time2=g_tock(t1);
//            ROS_INFO("    radius searching took %f for %d pts.  the for loops took %f",time1,indices1.size(),time2);

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
      if((int)clusters[i].size() < min_pts_per_cluster){
         clusters.erase(clusters.begin()+i);
         deletedclusters.push_back(i);
//         if(globalverbose)
//            cout<<"erasing "<<i<<endl;
      }

//   int clusterofinterest=22;
//   int p1=clusters[clusterofinterest][0];
//   //get original cluster:
//   int offset=0;
//   for(uint j=0;j<deletedclusters.size();j++)
//      if(deletedclusters[j]<=clusterofinterest)
//         offset++;
//   cout<<"Cluster "<<clusterofinterest<<" was originally "<<clusterofinterest+offset<<endl;
//   clusterofinterest+=offset;
//   cout<<"cluster size: "<<clusters[clusterofinterest].size()<<endl;
//   cout<<"heads:  ";
//   for(uint j=0;j<minitree2.size();j++){
//      if(minitree2[j]==clusterofinterest)
//         cout<<j<<"  ";
//   }
//   cout<<endl;
//   cout<<"cluster 22, pt 0: "<<p1<<" head: "<<minitree[p1]<<" which is pt "<<heads[minitree[p1]]<<std::endl;
//   int p2=clusters[43][0];
//   cout<<"cluster 43, pt 0: "<<p2<<" head: "<<minitree[p2]<<" which is pt "<<heads[minitree[p2]]<<std::endl;
//   cout<<"distance between heads: "<<ptdist(cloud.points[heads[minitree[p1]]], cloud.points[heads[minitree[p2]]])<<std::endl;




}

template <typename PointT>
double ptdist(PointT a,PointT b){
   return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z);
}








//template <typename PointT>
//void segfast(pcl::PointCloud<PointT> &cloud, std::vector<std::vector<int> > &clusters, double cluster_tol=.2){
//   double smaller_tol = cluster_tol/2.3;
//   timeval t0=g_tick();
//   pcl::KdTreeFLANN<PointT> tree,tree2;
//   tree.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> > (cloud));
//   vector<int> minitree(cloud.points.size(),-1);
//   vector<int> heads,indices;
//   vector<float> dists;
//
//   //First 'downsample' the points into clusters slightly less than half the cluster tolerance
//   //minitree keeps track of who each point belongs to (which is an index of the 'head' array)
//   for(uint i=0; i<cloud.points.size();i++){
//     if(minitree[i]==-1){    //if no one has claimed this point, make it a head
//        heads.push_back(i);
//        if(!tree.radiusSearch(cloud,i,smaller_tol,indices,dists))  //find all the points close to this point
//           cout<<"radius search failed!"<<endl;
//        for(uint j=0;j<indices.size();j++){
//           minitree[indices[j]]=heads.size()-1; //assign these points to the current head.
//                                       // this overwrites previous claims, but it's ok
//        }
//     }
//   }
//// std::cout<<"radiusSearch took:  "<<g_tock(t0)<<"s  found "<<heads.size()<<" heads"<<std::endl;
//
//   //now, we have much fewer points to cluster, but since the initial downsampling was less than
//   //half the cluster tolerance,we are guaranteed to find all the points within the tolerance
//   //(except if  [ clustertol < distance between heads < cluster_tol+ smaller cluster tol] AND the head closest points in the two heads are < cluster_tol
//   //I need to make something to deal with that...
//   tree2.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> > (cloud),boost::make_shared<std::vector<int> >  (heads));
//   int searching,currenthead;
//   //heads2 is the cluster id.  we now search to make sure we check all the points in our cluster
//   //minitree2 corresponds to the heads array, so the points in minitree2 are at heads[i]
//   std::vector<int> heads2, minitree2(heads.size(),-1);
//   std::list<int> tosearch;
//   for(uint i=0; i<minitree2.size();i++){
//     if(minitree2[i]==-1){
//        heads2.push_back(heads[i]);
//        tosearch.push_back(i);
//        currenthead=heads2.size()-1; //references an index in heads2
//        minitree2[i]=currenthead;
//        while(tosearch.size()>0){
//           searching=tosearch.front();
//           tosearch.pop_front();
//           if(!tree2.radiusSearch(cloud.points[heads[searching]],cluster_tol,indices,dists))
//              cout<<"radius search failed!"<<endl;
////         cout<<i<<" --> "<<searching<<" --> "<<indices.size()<<endl;
//           for(uint j=0;j<indices.size();j++)
//              if(minitree2[indices[j]]==-1){//found untouched point (which means this root touched it)
//                 minitree2[indices[j]]=currenthead; //claim it
//                 tosearch.push_back(indices[j]);   //add it to list of points to search
//              }
//        }
//     }
//
//   }
//
//   timeval t1=g_tick();
//   vector<int> indices1;
//   vector<float> dists1;
//   vector<int> deletedclusters;
//   bool verbose=false;
//   //now we need to check to see if any of the heads that were NOT clustered together are closer than cluster_tol+smaller_tol:
//   for(uint i=0; i<minitree2.size();i++){
//	  if(i==2255 || i==2296) verbose=true;
//	  else verbose=false;
//	  if(verbose) cout<<"head "<<i<<std::endl;
//      tree2.radiusSearch(cloud.points[heads[i]],cluster_tol+smaller_tol,indices,dists);
//	  if(verbose) {
//	     cout<<"  radius search found: "<<indices.size()<<"  matches"<<std::endl;
//	     for(uint rind=0;rind<indices.size();rind++)
//		     cout<<" -> "<<indices[rind]<<" in cluster "<<minitree2[indices[rind]]<<std::endl;
//
//	  }
//      for(uint j=0;j<indices.size();j++)
//         if(minitree2[indices[j]] != minitree2[i] && i<indices[j]){//if the two heads are not in the same cluster
//            cout<<"head "<<i<<" ("<<minitree2[i]<<") is "<<sqrt(dists[j])<<" from head "<<indices[j]<<" ("<<minitree2[indices[j]]<<")"<<endl;
//            //now we have to find if there is a point between head a and head b that is < cluster_tol from both
//            PointT heada=cloud.points[heads[i]], headb=cloud.points[heads[indices[j]]];
//            PointT inbetween;
//            inbetween.x=(heada.x+headb.x)/2.0;
//            inbetween.y=(heada.y+headb.y)/2.0;
//            inbetween.z=(heada.z+headb.z)/2.0;
//
//            tree.radiusSearch(inbetween,cluster_tol,indices1,dists1); //search in the full tree
//            double distthresh=cluster_tol*cluster_tol; //radius search gives squared distances...
//            for(uint k=0;k<indices1.size();k++){
//               if(ptdist(cloud.points[indices1[k]],heada ) < distthresh && ptdist(cloud.points[indices1[k]],headb ) < distthresh ){
//                  //there is a point that these two clusters share -> they should be merged
//                  cout<<"head "<<i<<" ("<<minitree2[i]<<") shares "<<indices1[k]<<" with head "<<indices[j]<<" ("<<minitree2[indices[j]]<<")"<<endl;
//                  //rename all heads in cluster b to cluster a
//                  int clusterb=minitree2[indices[j]];
//                  for(uint m=0;m<minitree2.size();m++){
//                     if(minitree2[m]==clusterb) minitree2[m]=minitree2[i];
//                  }
//                  deletedclusters.push_back(clusterb);
//                  break;
//               }
//
//            }
//
//         }
//   }
//
//   std::cout<<"checking overlaps took:  "<<g_tock(t1)<<"s  found "<<deletedclusters.size()<<" overlaps"<<std::endl;
//
//
//
//
//
//   std::cout<<"radiusSearch took:  "<<g_tock(t0)<<"s  found "<<heads2.size()<<" heads"<<std::endl;
//   clusters.resize(heads2.size());
//   //now, for each point in the cloud find it's head --> then the head it clustered to. that is it's cluster id!
//   for(uint j=0;j<minitree.size();j++){
//     clusters[minitree2[minitree[j]]].push_back(j);
//   }
//
//   int p1=clusters[8][47];
//   cout<<"cluster 8, pt 47: "<<clusters[8][47]<<" head: "<<minitree[p1]<<" which is pt "<<heads[minitree[p1]]<<std::endl;
//   int p2=clusters[11][1];
//   cout<<"cluster 11, pt 1: "<<clusters[11][1]<<" head: "<<minitree[p2]<<" which is pt "<<heads[minitree[p2]]<<std::endl;
//   cout<<"distance between heads: "<<ptdist(cloud.points[heads[minitree[p1]]], cloud.points[heads[minitree[p2]]])<<std::endl;
//
//
//
////   std::cout<<"clustering took:  "<<g_tock(t0)<<"s  found "<<heads2.size()<<" heads"<<std::endl;
////   for(uint j=0;j<heads2.size();j++){
////     cout<<"cluster "<<j<<"  -->  "<<clusters[j].size()<<endl;
////   }
//
//   //erase the deleted clusters
//   bool loop=true;
//   while(loop){
//      loop=false;
//      for(uint i=0;i<clusters.size(); i++)
//         if(clusters[i].size()==0){
//            clusters.erase(clusters.begin()+i);
//            loop=true;
//         }
//
//
//   }
//
//}
//



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Decompose a Point Cloud into clusters based on the Euclidean distance between points. Uses Hierarchical Clustering, making it faster
  * \param cloud the point cloud message
  * \param cluster_tol the spatial cluster tolerance as a measure in L2 Euclidean space
  * \param cclusters the resultant clusters containing point indices (as a vector of vector of ints)
  * \param min_pts_per_cluster minimum number of points that a cluster may contain (default = 1)
  * \param pt1 test index 1
  * \param pt2 test index 2
  */
template <typename PointT>
void extractEuclideanClustersFast3(pcl::PointCloud<PointT> &cloud, std::vector<std::vector<int> > &clusters, double cluster_tol=.2, int min_pts_per_cluster=1, int pt1=0, int pt2=0){
   bool globalverbose=false;
   bool debugging = (pt1>=0);
   int numsearches1=0, numsearches2=0, numsearches3=0, numsearches4=0;
   timeval t0,t2;
   t0=g_tick();
   t2=g_tick();
   double inittime1, inittime2, searches1, searches2,checkingtime,cleanuptime;
//   double smaller_tol = cluster_tol/2.3;
   double smaller_tol = cluster_tol/2.1;
   pcl::KdTreeFLANN<PointT> tree,tree2;
   tree.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> > (cloud));
   std::vector<int> minitree(cloud.points.size(),-1);
   std::vector<int> heads,indices;
   std::vector<float> dists;
   inittime1=g_tock(t0);
   t0=g_tick();
   int pt1head=0,pt2head=0, pt2head2=0, pt1head2=0;

   //First 'downsample' the points into clusters slightly less than half the cluster tolerance
   //minitree keeps track of who each point belongs to (which is an index of the 'head' array)
   for(uint i=0; i<cloud.points.size();i++){
     if(minitree[i]==-1){    //if no one has claimed this point, make it a head
        heads.push_back(i);
        numsearches1++;
        if(!tree.radiusSearch(cloud,i,smaller_tol,indices,dists))  //find all the points close to this point
           ROS_WARN("radius search failed!");
        for(uint j=0;j<indices.size();j++){
           minitree[indices[j]]=heads.size()-1; //assign these points to the current head.
                                       // this overwrites previous claims, but it's ok
        }
     }
   }
   if(debugging){
	   pt1head=minitree[pt1];
	   pt2head=minitree[pt2];
	   ROS_INFO("Point 1: %d is in subcluster %d.   Point 2: %d is in subcluster %d. ",pt1,pt1head,pt2,pt2head);
   }
   searches1=g_tock(t0);
   //now, we have much fewer points to cluster, but since the initial downsampling was less than
   //half the cluster tolerance,we are guaranteed to find all the points within the tolerance
   //(except if  [ clustertol < distance between heads < cluster_tol+ smaller cluster tol] AND the head closest points in the two heads are < cluster_tol
   //The next code block deals with that...

   t0=g_tick();
   tree2.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> > (cloud),boost::make_shared<std::vector<int> >  (heads));
   int searching,currenthead;
   //heads2 is the cluster id.  we now search to make sure we check all the points in our cluster
   //minitree2 corresponds to the heads array, so the points in minitree2 are at heads[i]
   std::vector<int> heads2, minitree2(heads.size(),-1);
   std::list<int> tosearch;
   inittime2=g_tock(t0);
   t0=g_tick();
   for(uint i=0; i<minitree2.size();i++){
     if(minitree2[i]==-1){
        heads2.push_back(heads[i]);
        tosearch.push_back(i);
        currenthead=heads2.size()-1; //references an index in heads2
        minitree2[i]=currenthead;
        while(tosearch.size()>0){
           searching=tosearch.front();
           tosearch.pop_front();
           numsearches2++;
           if(!tree2.radiusSearch(cloud.points[heads[searching]],cluster_tol,indices,dists))
              std::cout<<"radius search failed!"<<std::endl;
           for(uint j=0;j<indices.size();j++)
              if(minitree2[indices[j]]==-1){//found untouched point (which means this root touched it)
                 minitree2[indices[j]]=currenthead; //claim it
                 tosearch.push_back(indices[j]);   //add it to list of points to search
              }
        }
     }

   }
   searches2=g_tock(t0);

   int interestcluster1=-1,interestcluster2=-1;
   if(debugging){
	   pt1head2=minitree2[pt1head];
	   pt2head2=minitree2[pt2head];
	   ROS_INFO("Point 1 is in cluster %d.   Point 2 is in cluster %d. ",pt1head2,pt2head2);
	   interestcluster1=std::min(pt1head2,pt2head2);
	   interestcluster2=std::max(pt1head2,pt2head2);
   }

   t0=g_tick();
   clusters.resize(heads2.size());
   //now, for each point in the cloud find its head --> then the head it clustered to. that is its cluster id!
   for(uint j=0;j<minitree.size();j++){
     clusters[minitree2[minitree[j]]].push_back(j);
   }
   //now we need to check to see if any of the heads that were NOT clustered together are closer than cluster_tol+smaller_tol.
   //This covers the exception noted in the code block above
   //this is where an adversary could really kill this algorithm, since this check could be polynomial in cloud size. in real circumstances, it is very quick.
   std::vector<int> indices1;
   std::vector<float> dists1;
   double distthresh=cluster_tol*cluster_tol; //radius search gives squared distances...
   double automergethresh=cluster_tol*cluster_tol/4.0; //pts are always in same clustr if they are both less than half the tolerance away from the same point
   timeval t1;
   double time1,time2;
   bool verbose=false;
   for(uint i=0; i<minitree2.size();i++){ //for every head
      verbose=false;
//      if(count(interestclusters.begin(),interestclusters.end(),minitree2[i])) verbose=true;
//	  if(i==1452 || i==1508 || i==1509) verbose=true;
//	  else verbose=false;
	  if(verbose) std::cout<<"head "<<i<<"  in cluster "<<minitree2[i]<<std::endl;
	  	 numsearches3++;
		 tree2.radiusSearch(cloud.points[heads[i]],cluster_tol+2*smaller_tol,indices,dists);
	  if(verbose) {
	     std::cout<<"  radius search found: "<<indices.size()<<"  matches"<<std::endl;
//		 for(uint rind=0;rind<indices.size();rind++)
//			 cout<<" -> "<<indices[rind]<<" in cluster "<<minitree2[indices[rind]]<<std::endl;

	  }
//      tree2.radiusSearch(cloud.points[heads[i]],cluster_tol+smaller_tol,indices,dists);
      for(uint j=0;j<indices.size();j++){ //for every head that is close to this head
//   	     if(count(interestclusters.begin(),interestclusters.end(),minitree2[indices[j]]))  verbose=true;
  	     if(verbose && minitree2[indices[j]] != minitree2[i])
  	      std::cout<<" -> "<<indices[j]<<" in cluster "<<minitree2[indices[j]]<<std::endl;
         if(minitree2[indices[j]] != minitree2[i]){//if the two heads are not in the same cluster (and only check each combo once)
            t1=g_tick();
            if(debugging && minitree2[i]==interestcluster1 && minitree2[indices[j]] == interestcluster2) verbose=true;
            else verbose=false;
            if(verbose)
                  ROS_WARN("head %d (in cluster %d) is %f from head %d (in cluster %d). Checking if the two heads share points.",
        			i,minitree2[i],sqrt(dists[j]),indices[j],minitree2[indices[j]]);
            //now we have to find if there is a point between head a and head b that is < cluster_tol from both
            //our best chance of finding it is to start searching at a point halfway between them
            PointT heada=cloud.points[heads[i]], headb=cloud.points[heads[indices[j]]];
            if(verbose) std::cout<<"heada: "<<heada.x<<", "<<heada.y<<", "<<heada.z<<" "<<std::endl;
            if(verbose) std::cout<<"headb: "<<headb.x<<", "<<headb.y<<", "<<headb.z<<" "<<std::endl;
            PointT inbetween;
            inbetween.x=(heada.x+headb.x)/2.0;
            inbetween.y=(heada.y+headb.y)/2.0;
            inbetween.z=(heada.z+headb.z)/2.0;
            if(verbose) std::cout<<"in between pt: "<<inbetween.x<<", "<<inbetween.y<<", "<<inbetween.z<<" "<<std::endl;
            //the farthest 2 points could be from the centroid of the two heads and still be less than
            //cluster_tol apart is: cluster_tol + smaller_tol -dist_between_heads/2)
//            std::cout<<cluster_tol+smaller_tol-sqrt(dists[j])<<std::endl;
//            double searchrad=cluster_tol+smaller_tol-sqrt(dists[j]);
//            indices1.resize(0);
            numsearches4++;
            if(!tree.radiusSearch(inbetween,cluster_tol*2.0,indices1,dists1)){ //search in the full tree
               if(globalverbose || verbose)
                  ROS_ERROR("Radius Search for in between pt Failed");
               //nothing close to either cluster. these clusters are separate, so don't consider
               continue; //go to next j
            }
      	   if(verbose)
      	      std::cout<<"  full radius search found: "<<indices1.size()<<"  matches"<<std::endl;
            time1=g_tock(t1);
            t1=g_tick();
            //must match all points that return against each other
            bool shouldmerge=false;
            int mergefrom,mergeinto;
            for(int k=0;k<(int)indices1.size()-1;k++){
//            	if(verbose)
//          		 std::cout<<k<<": "<<minitree2[minitree[indices1[k]]]<<endl;
                for(uint m=k+1;m<indices1.size();m++){
                	if(verbose)
                	  std::cout<<k<<" ("<<indices1[k]<<"): "<<minitree2[minitree[indices1[k]]]<<" --> "
              		     <<m<<" ("<<indices1[m]<<"): "<<minitree2[minitree[indices1[m]]]
              		     <<"  "<<euclideanDistance(cloud.points[indices1[k]],cloud.points[indices1[m]])<<std::endl;

                	if(minitree2[minitree[indices1[k]]] != minitree2[minitree[indices1[m]]]){ //if different clusters

                		if((dists1[k]<automergethresh && dists1[m]<automergethresh) ||
           				(squaredEuclideanDistance(cloud.points[indices1[k]],cloud.points[indices1[m]] ) < distthresh)){
                		   mergefrom=minitree2[minitree[indices1[k]]];
                		   mergeinto=minitree2[minitree[indices1[m]]];
                		   if((mergefrom !=minitree2[i] && mergefrom !=minitree2[indices[j]]) || (mergeinto !=minitree2[i] && mergeinto !=minitree2[indices[j]])){
                		     if(globalverbose || verbose)
                		        ROS_WARN("point %d in cluster %d is %f from point %d in cluster %d, but these points won't be merged. since we are concidering %d and %d.",
                                   indices1[k],minitree2[minitree[indices1[k]]],
                                   euclideanDistance(cloud.points[indices1[k]],cloud.points[indices1[m]]),
                                   indices1[m],minitree2[minitree[indices1[m]]],minitree2[i],minitree2[indices[j]]);
                		   }
                		   else{
                           if(globalverbose || verbose){
                              ROS_WARN("point %d in cluster %d is %f from point %d in cluster %d",
                                    indices1[k],minitree2[minitree[indices1[k]]],
                                    euclideanDistance(cloud.points[indices1[k]],cloud.points[indices1[m]]),
                                    indices1[m],minitree2[minitree[indices1[m]]]);

                              PointT heada=cloud.points[indices1[k]], headb=cloud.points[indices1[m]];
                              std::cout<<"pt1: "<<heada.x<<", "<<heada.y<<", "<<heada.z<<" "<<std::endl;
                              std::cout<<"pt2: "<<headb.x<<", "<<headb.y<<", "<<headb.z<<" "<<std::endl;
                           }
                           shouldmerge=true;
                           break;  //TODO: shouldn't actually break, but rather see if multiple clusters should merge
                		   }
                		}
                	}

                }

               if(shouldmerge){

                  //there is a point that these two clusters share -> they should be merged
                  if(   globalverbose ||
                        verbose       ||
                       ( debugging &&
                           (minitree2[i]==interestcluster1 ||
                            minitree2[indices[j]]==interestcluster2  ||
                            minitree2[indices[j]]==interestcluster1 ||
                            minitree2[i]==interestcluster2)
                        )    )
                     ROS_INFO("head %d (in cluster %d) shares a point with head %d (in cluster %d). Merging cluster %d (%d pts) into cluster %d (%d pts)",
            			  i,minitree2[i],indices[j],minitree2[indices[j]],minitree2[indices[j]],(int)clusters[minitree2[indices[j]]].size(), minitree2[i],  (int)clusters[minitree2[i]].size());
                  //rename all heads in cluster b to cluster a
                  int clusterb=minitree2[indices[j]];
                  for(uint m=0;m<minitree2.size();m++){
                     if(minitree2[m]==clusterb) minitree2[m]=minitree2[i];
                  }
                  break;
               } //if the point is shared

//               std::cout<<"       "<<k<<"  "<<g_tock(t1)<<std::endl;
            } //for every point close to both heads
            time2=g_tock(t1);
//            ROS_INFO("    radius searching took %f for %d pts.  the for loops took %f",time1,indices1.size(),time2);

         } //if a head is close, and from a different cluster
      }//for every nearby head
   }  //for every head

   checkingtime=g_tock(t0);
   t0=g_tick();
   clusters.clear();
   clusters.resize(heads2.size());
   //now, for each point in the cloud find its head --> then the head it clustered to. that is its cluster id!
   for(uint j=0;j<minitree.size();j++){
     clusters[minitree2[minitree[j]]].push_back(j);
   }
   //erase the deleted clusters, and the ones under the minimum size
   std::vector<int> deletedclusters;
   for(int i=clusters.size()-1;i>=0; i--)
      if((int)clusters[i].size() < min_pts_per_cluster){
         clusters.erase(clusters.begin()+i);
         deletedclusters.push_back(i);
      }

   cleanuptime=g_tock(t0);
   printf("cloud: %d  Init1: ( %.3f, %.3f), searching took ( %.3f, %.3f) for  (%d, %d) searches, checking: %.3f for (%d, %d) searches,   cleaning:  %.4f    total %f \n",(int)cloud.size(),inittime1,inittime2,searches1,searches2,numsearches1,numsearches2,checkingtime,numsearches3,numsearches4, cleanuptime,g_tock(t2));

}
















template <typename PointT>
bool checkClusterMistake(int c1, int c2, std::vector<std::vector<int> > &clusters, pcl::PointCloud<PointT> &cloud, double tol,int &pt1, int &pt2){
   bool valid_split=true;
	for(uint i=0;i<clusters[c1].size();i++)
		for(uint j=0;j<clusters[c2].size();j++){
			float pdist=pcl::euclideanDistance(cloud.points[clusters[c1][i]],cloud.points[clusters[c2][j]]);
			if(pdist < tol){
				ROS_ERROR("Point %d in cluster %d is %f from point %d in cluster %d",i,c1,pdist,j,c2);
	            pt1=clusters[c1][i];
	            pt2=clusters[c2][j];
	            valid_split=false;
	            return false;
				valid_split=false;
			}
		}
   if(valid_split)
	   ROS_ERROR("cluster %d is at least %f from cluster %d",c1,tol,c2);
   return valid_split;

}

template <typename PointT>
bool checkClusterMistake(int c1, int c2, std::vector<pcl::PointIndices > &clusters, pcl::PointCloud<PointT> &cloud, double tol,int &pt1, int &pt2){
   bool valid_split=true;
   for(uint i=0;i<clusters[c1].indices.size();i++)
      for(uint j=0;j<clusters[c2].indices.size();j++){
         float pdist=pcl::euclideanDistance(cloud.points[clusters[c1].indices[i]],cloud.points[clusters[c2].indices[j]]);
         if(pdist < tol){
        	 ROS_ERROR("Point %d in cluster %d is %f from point %d in cluster %d",i,c1,pdist,j,c2);
            pt1=clusters[c1].indices[i];
            pt2=clusters[c2].indices[j];
            valid_split=false;
            return false;
         }
      }
   if(valid_split)
	   ROS_ERROR("cluster %d is at least %f from cluster %d",c1,tol,c2);
   return valid_split;
}


struct MatchReport{
   bool matched;
   //sample indices to debug segmentation
   int pt1,pt2;

   std::vector<int> pairing;
   std::vector< std::vector<int> > mapping1,mapping2;
   std::vector<int> bad1, bad2;
   std::vector< std::vector<int> > getSplits(int cluster){
      std::vector< std::vector<int> > out;
      if(cluster==1)
         for(uint i=0;i<mapping2.size();i++){
            if(mapping2[i].size() > 2){
               out.push_back(std::vector<int>(mapping2[i].size()-1));
               for(uint j=1;j<mapping2[i].size();j++)
                  out.back()[j-1]=bad1[mapping2[i][j]];
            }
         }
      else
         for(uint i=0;i<mapping1.size();i++){
            if(mapping1[i].size() > 2){
               out.push_back(std::vector<int>(mapping1[i].size()-1));
               for(uint j=1;j<mapping1[i].size();j++)
                  out.back()[j-1]=bad2[mapping1[i][j]];
            }
         }
      return out;
   }

};




bool matchclusterings(std::vector<pcl::PointIndices> &c1, std::vector<std::vector<int> > &c2, MatchReport &report){

   bool failed = false;
   if(c1.size() != c2.size()){
      ROS_DEBUG("number of clusters does not match: c1->%d,  c2->%d",(int)c1.size(),(int)c2.size());
		failed=true;
	}
	report.pairing.resize(c2.size(),-1);
	for(uint i=0;i<c1.size();i++){ //match each cluster in c1 to one in c2
		bool paired=false;
		for(uint j=0;j<c2.size();j++){
			if(c1[i].indices.size()==c2[j].size() && report.pairing[j]==-1){
				paired=true;
				for(uint k=0;k<c1[i].indices.size();k++)
					if(c1[i].indices[k]!=c2[j][k]){
//						std::cout<<" clusters c1["<<i<<"] and c2["<<j<<"] both had "<<c1[i].indices.size();
//						std::cout<<"indices, but c1["<<i<<"]["<<k<<"] = "<<c1[i].indices[k]<<" and c2["<<j<<"]["<<k<<"] = ";
//						std::cout<<c2[j][k]<<std::endl;
						paired=false;
						break;
					}
			}
			if(paired){
				report.pairing[j]=i;
				break;
			}

		}//for each c2
		if(!paired){
		   ROS_DEBUG("cloud not find a match for cluster c1[%d], size %d",i,(int)c1[i].indices.size());
//			std::cout<<"cloud not find a match for cluster c1["<<i<<"], size "<<c1[i].indices.size()<<std::endl;
			report.bad1.push_back(i);
			failed=true;
		}
	} // for each c1
	for(uint j=0;j<c2.size();j++)
	   if(report.pairing[j]==-1){
         ROS_DEBUG("cloud not find a match for cluster c2[%d], size %d",j,(int)c2[j].size());
//         std::cout<<"cloud not find a match for cluster c2["<<j<<"], size "<<c2[j].size()<<std::endl;
         report.bad2.push_back(j);
         failed=true;
	   }
	//------------------ Step 2: for diagnostic purposes, find how the pairing failed------------------
   std::vector< std::vector<int> > mapping1,mapping2;
	report.mapping1.resize(report.bad1.size());
   report.mapping2.resize(report.bad2.size());
	for(uint j=0;j<report.bad2.size();j++)
	   report.mapping2[j].push_back(0); //the first element will keep a count of the indices we've seen
	for(uint i=0;i<report.bad1.size();i++){
	   report.mapping1[i].push_back(0); //the first element will keep a count of the indices we've seen
	   for(uint j=0;j<report.bad2.size();j++){
	      int sharedpts=0;
	      for(uint k=0;k<c1[report.bad1[i]].indices.size();k++){ //for each of the indices in the c1 cluster
	         for(uint m=0;m<c2[report.bad2[j]].size();m++){ //compare to the indices of the c2 cluster
//	            std::cout<<"comparing "<<c1[report.bad1[i]].indices[k]<<" to:  "<<c2[report.bad2[j]][m]<<std::endl;
              if(c1[report.bad1[i]].indices[k]==c2[report.bad2[j]][m])
                 sharedpts++;
	         }
	      }
	      if(sharedpts){
	         report.mapping1[i].push_back(j);
	         report.mapping1[i][0]+=sharedpts;
            report.mapping2[j].push_back(i);
            report.mapping2[j][0]+=sharedpts;
	      }
	   }//for each bad2
	   if(report.mapping1[i][0] != (int)c1[report.bad1[i]].indices.size())
	      ROS_DEBUG("%d points in c1[%d] were unaccounted for!",(int)c1[report.bad1[i]].indices.size()-report.mapping1[i][0],report.bad1[i]);
//	   if(report.mapping1[i].size() > 2){
//	      std::cout<<"c1["<<report.bad1[i]<<"] split into "<<report.mapping1[i].size()-1<<"  clusters: ";
//	      for(uint j=1;j<report.mapping1[i].size();j++)
//	         std::cout<<report.bad2[report.mapping1[i][j]]<<"  ";
//	      std::cout<<std::endl;
//	   }
	}//for each bad1

//   for(uint i=0;i<report.bad2.size();i++){
//      if(report.mapping2[i][0] != c2[report.bad2[i]].size())
//         ROS_ERROR("%d points in c2[%d] were unaccounted for!",c2[report.bad2[i]].size()-report.mapping2[i][0],report.bad2[i]);
//      if(report.mapping2[i].size() > 2){
//         std::cout<<"c2["<<report.bad2[i]<<"] split into "<<report.mapping2[i].size()-1<<"  clusters: ";
//         for(uint j=1;j<report.mapping2[i].size();j++)
//            std::cout<<report.bad1[report.mapping2[i][j]]<<"  ";
//         std::cout<<std::endl;
//      }
//   }
   report.matched=!failed;
	return !failed;

}

//TODO: append suffix that describes clustering tolerance
void writeCorrectClusters(std::string filein, std::vector<pcl::PointIndices> &inds,double processingtime,double cluster_tol){
   //write number of clusters
   std::ofstream outf;
   char filename[500];
   sprintf(filename,"%s_%.2f_std_clustering",filein.c_str(),cluster_tol);
   outf.open(filename,std::ios::out);
   outf<<inds.size()<<std::endl;
   for (uint i = 0; i < inds.size(); ++i) {
      //for each cluster:
      //write number of points, then list the indices
      outf<<inds[i].indices.size()<<" ";
      for (uint j = 0; j < inds[i].indices.size(); ++j)
         outf<<inds[i].indices[j]<<" ";
      outf<<std::endl;
   }
   outf<<processingtime<<std::endl;

   outf.close();
}

//TODO: append suffix that describes clustering tolerance
void writeResultingClusters(std::string filein, std::vector< std::vector<int> > &inds,double processingtime,double cluster_tol){
   //write number of clusters
   std::ofstream outf;
   char filename[500];
   sprintf(filename,"%s_%.2f_new_clustering",filein.c_str(),cluster_tol);
   outf.open(filename,std::ios::out);
   outf<<inds.size()<<std::endl;
   for (uint i = 0; i < inds.size(); ++i) {
      //for each cluster:
      //write number of points, then list the indices
      outf<<inds[i].size()<<" ";
      for (uint j = 0; j < inds[i].size(); ++j)
         outf<<inds[i][j]<<" ";
      outf<<std::endl;
   }
   outf<<processingtime<<std::endl;

   outf.close();
}


int readCorrectClusters(std::string filein, std::vector<pcl::PointIndices> &inds, double &ptime,double cluster_tol){
   std::ifstream inf;
   char filename[500];
   sprintf(filename,"%s_%.2f_std_clustering",filein.c_str(),cluster_tol);
   inf.open(filename,std::ios::in);
   int vsize;
   inf>>vsize;
   inds.resize(vsize);
   for (uint i = 0; i < inds.size(); ++i) {
      inf>>vsize;
      inds[i].indices.resize(vsize);
      for (uint j = 0; j < inds[i].indices.size(); ++j)
         inf>>inds[i].indices[j];
   }
   inf>>ptime;
   inf.close();
   return 0;
}

int readResultingClusters(std::string filein, std::vector< std::vector<int> > &inds, double &ptime,double cluster_tol){
   std::ifstream inf;
   char filename[500];
   sprintf(filename,"%s_%.2f_new_clustering",filein.c_str(),cluster_tol);
   inf.open(filename,std::ios::in);
   int vsize;
   inf>>vsize;
   inds.resize(vsize);
   for (uint i = 0; i < inds.size(); ++i) {
      inf>>vsize;
      inds[i].resize(vsize);
      for (uint j = 0; j < inds[i].size(); ++j)
         inf>>inds[i][j];
   }
   inf>>ptime;
   inf.close();
   return 0;
}

double testexisting(pcl::PointCloud<pcl::PointXYZINormal> &smallcloud, std::vector<pcl::PointIndices> &inds,double cluster_tol ){
    timeval t0=g_tick();
     pcl::EuclideanClusterExtraction<pcl::PointXYZINormal> clusterer;
     clusterer.setClusterTolerance(cluster_tol);
     clusterer.setMinClusterSize(1);
     clusterer.setMaxClusterSize(1000000);
     clusterer.setInputCloud(smallcloud.makeShared());
     clusterer.extract(inds);
     return g_tock(t0);
}

double testexisting(pcl::PointCloud<pcl::PointXYZINormal> &smallcloud, std::vector< std::vector<int> > &inds,double cluster_tol){
	timeval t0=g_tick();
    extractEuclideanClustersFast2(smallcloud,inds,cluster_tol);
    return g_tock(t0);
}

void compareResults(std::vector< std::vector<int> > &inds2, std::vector<pcl::PointIndices> &inds, MatchReport &report, pcl::PointCloud<pcl::PointXYZINormal> &smallcloud){
	//at the moment, only care about if it failed, not how...
	matchclusterings(inds,inds2,report);
	return;

		if(!matchclusterings(inds,inds2,report)){
		   std::vector<std::vector<int> > splits;
		   splits=report.getSplits(1);
	      if(splits.size())
	         std::cout<<"checking cluster 1 splits: "<<std::endl;
		   for (uint i = 0; i < splits.size(); ++i)
		      //only check pairwise splits at the moment...
		      checkClusterMistake(splits[i][0],splits[i][1],inds,smallcloud,.2,report.pt1,report.pt2);

	      splits=report.getSplits(2);
	      if(splits.size())
	         std::cout<<"checking cluster 2 splits: "<<std::endl;
	      for (uint i = 0; i < splits.size(); ++i)
	         //only check pairwise splits at the moment...
	         if(!checkClusterMistake(splits[i][0],splits[i][1],inds2,smallcloud,.2,report.pt1,report.pt2)){
	        	 std::cout<<"check points "<<report.pt1<<" "<<report.pt2<<std::endl;
	        	 break;
	         }
		}
}

struct ClusterEvaluation{
	std::string filename;

	double cluster_tol;
	std::vector<std::vector<int> > c2;
	std::vector<pcl::PointIndices> c1;
	MatchReport report;

	double time1,time2;
	timeval t0;
	 pcl::PointCloud<pcl::PointXYZINormal> smallcloud;

	ClusterEvaluation(std::string f){
		filename=f;
		cluster_tol=.2;
	}

	int loadCloud(){
		int ret= pcl::io::loadPCDFile(filename,smallcloud);
		if(ret) std::cerr<<" failed to load "<<filename<<" the cloud file."<<std::endl;
		return ret;
	}



	//load pre-calculated cluster results from standard algorithm
	int loadResults1()    {
		int ret= readCorrectClusters(filename,c1,time1,cluster_tol);
		if(ret) std::cerr<<" failed to load "<<filename<<".clusters1, the pre-calculated cluster results from standard clustering algorithm"<<std::endl;
		return ret;
	}
	//load pre-calculated cluster results from new algorithm
	int loadResults2()    {
		int ret= readResultingClusters(filename,c2,time2,cluster_tol);
		if(ret) std::cerr<<" failed to load "<<filename<<".clusters1, the pre-calculated cluster results from new clustering algorithm"<<std::endl;
		return ret;
	}
	void saveResults1(){ writeCorrectClusters(filename,c1,time1,cluster_tol); }
	void saveResults2(){ writeResultingClusters(filename,c2,time2,cluster_tol); }
	void computeResults1(){ time1=testexisting(smallcloud,c1,cluster_tol);	}
	void computeResults2(){ time2=testexisting(smallcloud,c2,cluster_tol);	}
	void evaluateResults(){
			if(!matchclusterings(c1,c2,report)){
			   std::vector<std::vector<int> > splits;
			   splits=report.getSplits(1);
		      if(splits.size())
		         std::cout<<"checking cluster 1 splits: "<<std::endl;
			   for (uint i = 0; i < splits.size(); ++i)
			      //only check pairwise splits at the moment...
			      checkClusterMistake(splits[i][0],splits[i][1],c1,smallcloud,cluster_tol,report.pt1,report.pt2);

		      splits=report.getSplits(2);
		      if(splits.size())
		         std::cout<<"checking cluster 2 splits: "<<std::endl;
		      for (uint i = 0; i < splits.size(); ++i)
		         //only check pairwise splits at the moment...
		         if(!checkClusterMistake(splits[i][0],splits[i][1],c2,smallcloud,cluster_tol,report.pt1,report.pt2)){
		        	 std::cout<<"check points "<<report.pt1<<" "<<report.pt2<<std::endl;
		        	 break;
		         }
			}
	}

	void printResult(){
		int cloudsize=0;
		if(smallcloud.size())
		   cloudsize=smallcloud.size();
		else
		   for(uint i=0;i<c2.size(); i++) 
			cloudsize+=c2[i].size();
		if(report.matched)
		    std::cout<<"new: "<<time2<<"  existing: "<<time1<<"  cluster tol: "<<cluster_tol<<"  clusters: "<<c1.size()<<" cloud: "<<cloudsize<<"  "<<filename<<" matched "<<std::endl;
		else
		    std::cout<<"new: "<<time2<<"  existing: "<<time1<<"  cluster tol: "<<cluster_tol<<"  clusters: "<<c1.size()<<" ("<<c2.size()<<") cloud: "<<cloudsize<<"  "<<filename<<" failed to match!!!"<<std::endl;
	}

};

void printUsage(std::string progname){
	std::cout<<"USAGE "<<progname<<" filename.pcd [options]"<<std::endl;
	std::cout<<"Options: "<<std::endl;
	std::cout<<"-tol <cluster_tol> : specify cluster spacing parameter"<<std::endl;
	std::cout<<"-l1 : Load pre-computed results of standard clustering algorithm"<<std::endl;
	std::cout<<"-l2 : Load pre-computed results of new clustering algorithm"<<std::endl;
	std::cout<<"-s1 : Save computed results of standard clustering algorithm"<<std::endl;
	std::cout<<"-s2 : Save computed results of new clustering algorithm"<<std::endl;
	std::cout<<"-debug : run evaluative version of new algorithm to find out why it broke"<<std::endl;
	std::cout<<"-time : run evaluative version of new algorithm to find out what takes the time"<<std::endl;
	std::cout<<"-nocompare : don't compare the evaluations.  Will only compute results if it is being saved"<<std::endl;
}

int main(int argc, char **argv) {
	if(argc<2){
		printUsage(argv[0]);
		return -1;
	}
	bool save1=false, save2=false, load1=false, load2=false, nocompare=false,debugmode=false,timemode=false;
	ClusterEvaluation evaluation(argv[1]);

	int argiter=1;
	double cluster_tol=.2;
	while(++argiter < argc){
		if(strcmp(argv[argiter],"-l1")==0) load1=true;
		if(strcmp(argv[argiter],"-tol")==0 && argiter+1 < argc) cluster_tol=atof(argv[++argiter]);
		if(strcmp(argv[argiter],"-l2")==0) load2=true;
		if(strcmp(argv[argiter],"-s1")==0) save1=true;
		if(strcmp(argv[argiter],"-s2")==0) save2=true;
		if(strcmp(argv[argiter],"-debug")==0) debugmode=true;
		if(strcmp(argv[argiter],"-time")==0) timemode=true;
		if(strcmp(argv[argiter],"-nocompare")==0) nocompare=true;
	}
	evaluation.cluster_tol=cluster_tol;

	if(timemode){
		evaluation.loadCloud();
		extractEuclideanClustersFast3(evaluation.smallcloud,evaluation.c2,cluster_tol,1,-1,-1);
		return 0;
	}


	if(load1 && evaluation.loadResults1()) return -1;
	if(load2 && evaluation.loadResults2()) return -1;
	if((!load1 || !load2) && (!nocompare || save1 || save2 ) && evaluation.loadCloud())  return -1;  //load cloud if we need to do actual calculations

	//if we need to do actual calculations, do it here:
	if(!load1 && (!nocompare || save1) )
		evaluation.computeResults1();
	if(!load2 && (!nocompare || save2))
		evaluation.computeResults2();

	if(save1)
		evaluation.saveResults1();

	if(save2)
		evaluation.saveResults2();

	if(!nocompare){
		evaluation.evaluateResults();
		evaluation.printResult();
	}

	if(debugmode && !evaluation.report.matched){
		extractEuclideanClustersFast3(evaluation.smallcloud,evaluation.c2,cluster_tol,1,evaluation.report.pt1,evaluation.report.pt2);
	}

	return 0;
}





