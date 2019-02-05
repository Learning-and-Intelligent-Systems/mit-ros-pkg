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



#ifndef SEGFAST_H_
#define SEGFAST_H_

#include "pcl/io/pcd_io.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/features/feature.h"
#include "nnn/nnn.hpp"
#include <sys/time.h>
#include <list>
#include <fstream>

using namespace std;

#ifndef GTICK
#define GTICK
#include <sys/time.h>
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

#endif



  //keeps  track of clustering result
template <typename PointT>
struct PtMap{
	std::vector<int> ptindices; //map indices used in this struct to the outside (possibly to the real point cloud)
	map<int,int> 	 rmap;  //this maps the outside indices to inside: i = rmap[ptindices[i]]
							//note that o = ptindices[rmap[o]] might not work, since

	double _cluster_tol;
	pcl::PointCloud<PointT> *_cloud;
	std::vector<int> clusterindices; //indicates which cluster each pt belongs to
	std::vector<int> loners;  // the these are points who form their own cluster of one

	//these are of size heads.size():
	std::vector<int> heads;  // the index of the 'head' of each cluster, i.e where the radius search originated
	std::vector<int> clusterindices2; //indicates which cluster each head belongs to
	std::vector< std::vector<int> > pairings;
	std::vector<int> initialgrabs; //counts how many pts each head initially gets
   std::vector< std::vector<int> > tosearch; //list of pairs of heads that are candidates for merging
   std::vector<float> farpt;  // the distance to the farthest point in the

	//these are of size heads2.size()
	std::vector<int> heads2;  // the index of the 'head' of each super cluster, i.e where the radius search originated
	std::vector< std::vector<int> > clusters;  //the final clusters

	SplitCloud2<PointT> *_sc2;


	//debugging tools
	std::vector<int> overlapcount; //counts how many radii a pt falls within
	int randseed;

	void setInds(std::vector<int> &inds){
		ptindices=inds;
		for(uint i=0;i<ptindices.size();i++)
			rmap[ptindices[i]]=i;
	}

	PtMap(pcl::PointCloud<PointT> &cloud, double cluster_tol){
		_cloud=&cloud;
		clusterindices.resize(_cloud->size(),-1);
		_cluster_tol=cluster_tol;
	}


	//for when there is no special indexing:
	void setInds(int s){
		ptindices.resize(s);
		for(uint i=0;i<ptindices.size();i++){
			ptindices[i]=i;
			rmap[i]=i;
		}
	}
	int getUsec(){
	     struct timeval tv;
	     gettimeofday(&tv, NULL);
	     return tv.tv_usec;
	  }
	//simply runs radius search to grab all the points nearby until everything is taken
	//results in each pt assigned a label, and a list of who grabbed each point

	void simpleDownsample(pcl::PointCloud<PointT> &cloud, double cluster_tol=.2){
		if(!ptindices.size()) //if we haven't initialized pt indices, just use the cloud size
			setInds(cloud.size());
	   pcl::KdTreeFLANN<PointT> tree;
	   tree.setInputCloud(cloud.makeShared(), boost::make_shared<std::vector<int> > (ptindices));
	   vector<int> indices;
	   vector<float> dists;
	   heads.clear();
	   overlapcount.resize(ptindices.size(),0);//DEBUG: count the number of overlaps



      srand(getUsec());
      //DEBUG:: test seeding randomly to begin with:
      for(int j=0; j<randseed;j++){
       int i = rand()%clusterindices.size();
       if(clusterindices[i]==-1){    //if no one has claimed this point, make it a head
         pairings.push_back(std::vector<int>());//DEBUG: keeping track of pairing
         heads.push_back(i);
         if(!tree.radiusSearch(cloud.points[ptindices[i]],cluster_tol,indices,dists))  //find all the points close to this point
            ROS_WARN("radius search failed!");
         initialgrabs.push_back(indices.size()); //DEBUG: count how many pts each head initially gets
         for(uint j=0;j<indices.size();j++){
            if(clusterindices[rmap[indices[j]]]>-1){ //DEBUG: if this pt is already claimed, it indicates a pairing
               if(!count(pairings[heads.size()-1].begin(),pairings[heads.size()-1].end(),clusterindices[rmap[indices[j]]]))
                  pairings[heads.size()-1].push_back(clusterindices[rmap[indices[j]]]);
            }
            clusterindices[rmap[indices[j]]]=heads.size()-1; //assign these points to the current head. this overwrites previous claims, but it's ok
            overlapcount[rmap[indices[j]]]++;  //DEBUG: count the number of overlaps


         }
       }
      }
	   for(uint i=0; i<clusterindices.size();i++){
		 if(clusterindices[i]==-1){    //if no one has claimed this point, make it a head
			pairings.push_back(std::vector<int>());//DEBUG: keeping track of pairing
			heads.push_back(i);
			if(!tree.radiusSearch(cloud.points[ptindices[i]],cluster_tol,indices,dists))  //find all the points close to this point
			   ROS_WARN("radius search failed!");
			initialgrabs.push_back(indices.size()); //DEBUG: count how many pts each head initially gets
			for(uint j=0;j<indices.size();j++){
				if(clusterindices[rmap[indices[j]]]>-1){ //DEBUG: if this pt is already claimed, it indicates a pairing
					if(!count(pairings[heads.size()-1].begin(),pairings[heads.size()-1].end(),clusterindices[rmap[indices[j]]]))
						pairings[heads.size()-1].push_back(clusterindices[rmap[indices[j]]]);
				}
				clusterindices[rmap[indices[j]]]=heads.size()-1; //assign these points to the current head. this overwrites previous claims, but it's ok
				overlapcount[rmap[indices[j]]]++;  //DEBUG: count the number of overlaps


			}
		 }
	   }
	}


   //simply runs radius search to grab all the points nearby until everything is taken
   //results in each pt assigned a label, and a list of who grabbed each point

   void simpleDownsample2(pcl::PointCloud<PointT> &cloud, double cluster_tol=.2){
      timeval t0=g_tick();
      if(!ptindices.size()) //if we haven't initialized pt indices, just use the cloud size
         setInds(cloud.size());
      std::cout<<"  setinds took "<<g_tock(t0)<<endl; t0=g_tick();
      pcl::KdTreeFLANN<PointT> tree;
      tree.setInputCloud(cloud.makeShared(), boost::make_shared<std::vector<int> > (ptindices));

      std::cout<<"  setupcloud took "<<g_tock(t0)<<endl; t0=g_tick();
      vector<int> indices;
      vector<float> dists;
      heads.clear();

      int searchcount=0;
      for(uint i=0; i<clusterindices.size();i++){
       if(clusterindices[i]==-1){    //if no one has claimed this point, make it a head
         heads.push_back(i);
         searchcount++;
         if(!tree.radiusSearch(cloud.points[ptindices[i]],cluster_tol,indices,dists))  //find all the points close to this point
            ROS_WARN("radius search failed!");
         for(uint j=0;j<indices.size();j++){
            clusterindices[rmap[indices[j]]]=heads.size()-1; //assign these points to the current head. this overwrites previous claims, but it's ok

         }
       }
      }
      std::cout<<"  searching took "<<g_tock(t0)<<" for "<<searchcount<<" searches"<<endl;

   }


   //simply runs radius search to grab all the points nearby until everything is taken
   //results in each pt assigned a label, and a list of who grabbed each point

   void simpleDownsampleNNN2(pcl::PointCloud<PointT> &cloud, double cluster_tol=.2){
      timeval t0=g_tick();
//      if(!ptindices.size()) //if we haven't initialized pt indices, just use the cloud size
//         setInds(cloud.size());
//      std::cout<<"  setinds took "<<g_tock(t0)<<endl; t0=g_tick();
      _sc2 =  new SplitCloud2<PointT>(cloud,3.0*cluster_tol);
      std::cout<<"  setupcloud took "<<g_tock(t0)<<endl; t0=g_tick();
      vector<int> indices;
      vector<float> dists;
      heads.clear();

      int searchcount=0;
      for(uint i=0; i<clusterindices.size();i++){
       if(clusterindices[i]==-1){    //if no one has claimed this point, make it a head
           pairings.push_back(std::vector<int>());//DEBUG: keeping track of pairing
         heads.push_back(i);
         searchcount++;
//         initialgrabs.push_back(_sc2->AssignInds(cloud.points[i],clusterindices,cluster_tol,heads.size()-1));  //find all the points close to this point
         _sc2->NNN(cloud.points[i],indices,dists,cluster_tol);  //find all the points close to this point
         initialgrabs.push_back(indices.size());
         float maxdist;
         for(uint j=0;j<indices.size();j++){
            if(j==0 || maxdist<dists[j]) maxdist=dists[j];
            if(clusterindices[indices[j]]>-1){ //DEBUG: if this pt is already claimed, it indicates a pairing
				if(!count(pairings.back().begin(),pairings.back().end(),clusterindices[indices[j]]))
					pairings.back().push_back(clusterindices[indices[j]]);
			}
            clusterindices[indices[j]]=heads.size()-1; //assign these points to the current head. this overwrites previous claims, but it's ok

//			overlapcount[indices[j]]++;  //DEBUG: count the number of overlaps
         }
         farpt.push_back(sqrt(maxdist));
       }
      }
      std::cout<<"  searching took "<<g_tock(t0)<<" for "<<searchcount<<" searches"<<endl;

   }

   //simply runs radius search to grab all the points nearby until everything is taken
   //results in each pt assigned a label, and a list of who grabbed each point

   void simpleDownsampleNNN(pcl::PointCloud<PointT> &cloud, double cluster_tol=.2){
      timeval t0=g_tick();
      std::cout<<"  setinds took "<<g_tock(t0)<<endl; t0=g_tick();
      SplitCloud<PointT> sc(cloud,cluster_tol);
      std::cout<<"  setupcloud took "<<g_tock(t0)<<endl; t0=g_tick();
      vector<int> indices;
      vector<float> dists;
      heads.clear();

      int searchcount=0;
      for(uint i=0; i<clusterindices.size();i++){
       if(clusterindices[i]==-1){    //if no one has claimed this point, make it a head
         heads.push_back(i);
         searchcount++;
         sc.NNN(cloud.points[i],indices,cluster_tol);  //find all the points close to this point
         for(uint j=0;j<indices.size();j++){
            clusterindices[indices[j]]=heads.size()-1; //assign these points to the current head. this overwrites previous claims, but it's ok
         }
       }
      }
      std::cout<<"  searching took "<<g_tock(t0)<<" for "<<searchcount<<" searches"<<endl;

   }


//The next step in clustering: cluster the heads
   void headCluster(){
	      timeval t0=g_tick();
	      pcl::KdTreeFLANN<PointT> tree2;
	   tree2.setInputCloud(_cloud->makeShared(),boost::make_shared<std::vector<int> >  (heads));
	      std::cout<<"  setupcloud took "<<g_tock(t0)<<endl; t0=g_tick();
	   int searching,currenthead;
	      vector<int> indices;
	      vector<float> dists;
	      heads2.clear();
	   //heads2 is the cluster id.  we now search to make sure we check all the points in our cluster
	   //minitree2 corresponds to the heads array, so the points in minitree2 are at heads[i]
//	   std::vector<int> heads2;
	   clusterindices2.resize(heads.size(),-1);
	   std::list<int> tosearch;
	      int searchcount=0;
	   for(uint i=0; i<clusterindices2.size();i++){
	     if(clusterindices2[i]==-1){
	        heads2.push_back(heads[i]);
	        tosearch.push_back(i);
	        currenthead=heads2.size()-1; //references an index in heads2
	        clusterindices2[i]=currenthead;
	        while(tosearch.size()>0){
	           searching=tosearch.front();
	           tosearch.pop_front();
	           searchcount++;
	           if(!tree2.radiusSearch(_cloud->points[heads[searching]],_cluster_tol,indices,dists))
	              cout<<"radius search failed!"<<endl;
	           for(uint j=0;j<indices.size();j++)
	              if(clusterindices2[indices[j]]==-1){//found untouched point (which means this root touched it)
	                 clusterindices2[indices[j]]=currenthead; //claim it
	                 tosearch.push_back(indices[j]);   //add it to list of points to search
	              }
	        }
	     }
	   }
	      std::cout<<"  searching took "<<g_tock(t0)<<" for "<<searchcount<<" searches, getting "<<heads2.size()<<"  clusters"<<endl;
   }

   void headClusterNNN(){
	      timeval t0=g_tick();

//	      SplitCloud2 sc2(*_cloud,_cluster_tol);
//	      std::cout<<"  setupcloud took "<<g_tock(t0)<<endl; t0=g_tick();
	      _sc2->useInds(heads);
	      std::cout<<"  setupcloud2 took "<<g_tock(t0)<<endl; t0=g_tick();
	   int searching,currenthead;
	      vector<int> indices;
	      vector<float> dists;
	      heads2.clear();
	   //heads2 is the cluster id.  we now search to make sure we check all the points in our cluster
	   //minitree2 corresponds to the heads array, so the points in minitree2 are at heads[i]
//	   std::vector<int> heads2;
	   clusterindices2.resize(heads.size(),-1);
	   std::list<int> tosearch;
	      int searchcount=0;
	   for(uint i=0; i<clusterindices2.size();i++){
	     if(clusterindices2[i]==-1){
	        heads2.push_back(heads[i]);
	        tosearch.push_back(i);
	        currenthead=heads2.size()-1; //references an index in heads2
	        clusterindices2[i]=currenthead;
	        while(tosearch.size()>0){
	           searching=tosearch.front();
	           tosearch.pop_front();
	           searchcount++;
	           _sc2->NNN(_cloud->points[heads[searching]],indices,_cluster_tol,true);
	           for(uint j=0;j<indices.size();j++)
	              if(clusterindices2[indices[j]]==-1){//found untouched point (which means this root touched it)
	                 clusterindices2[indices[j]]=currenthead; //claim it
	                 tosearch.push_back(indices[j]);   //add it to list of points to search
	              }
	        }
	     }
	   }
	      std::cout<<"  searching took "<<g_tock(t0)<<" for "<<searchcount<<" searches, getting "<<heads2.size()<<"  clusters"<<endl;
   }

void recomputeClusters(){
	clusters.clear();
    clusters.resize(heads2.size());
    //now, for each point in the cloud find its head --> then the head it clustered to. that is its cluster id!
    for(uint j=0;j<clusterindices.size();j++){
    	clusters[clusterindices2[clusterindices[j]]].push_back(j);
    }
}


//go through the heads and extract every one that initially only grabbed one pt.
//these heads will never be joined with other heads
//this provides a big speedup when dealing with small cluster tolerances
void swapOutLoners(){
	   std::vector< int> tempheads(heads.size());
	   std::vector< int> headsremap(heads.size());
	   loners.resize(heads.size());
	   int numloners=0,numheads=0;
	   for(uint i=0;i<heads.size();++i)
		   if(initialgrabs[i] == 1){
			 loners[numloners++]=heads[i];
			 clusters[clusterindices2[i]].clear();
			 headsremap[i]=-1;
		   }
		   else{
			 headsremap[i]=numheads;
			 tempheads[numheads++]=heads[i];
		   }

//	   heads.swap(tempheads);
//	   heads.resize(numheads);
	   loners.resize(numloners);
	   std::vector< int> heads2remap(clusters.size());

	   //now redo clusterindices2, and clusters:
	   std::vector< std::vector<int> > clusters2(clusters.size());
	   int numclusters=0;
	   for(uint i=0; i<clusters.size();++i)
		   if(clusters[i].size()){
				heads2remap[i]=numclusters;
				clusters[i].swap(clusters2[numclusters++]);
		   }
		   else{
				heads2remap[i]=-1;
		   }

	   clusters.swap(clusters2);
	   clusters.resize(numclusters);

	   for(uint i=0;i<clusterindices2.size();++i)
		   clusterindices2[i]=heads2remap[clusterindices2[i]];


	   //sanity check. takes a while...
	   std::cout<<numloners<<" loners."<<std::endl;
//	   for(uint i=0;i<loners.size();++i){
//		   int lcluster=clusterindices2[clusterindices[loners[i]]];
//		   int lhead=clusterindices[loners[i]];
//		   for(uint j=0;j<clusterindices2.size();++j){
//			   if(clusterindices2[j]==lcluster && j!=lhead){
//				   cout<<"head "<<j<<" is in the same sluster as a loner, head "<<lhead<<endl;
//			   }
//
//		   }
//
//	   }

}

//adds all the single point clusters back in to the main clusters array
void addLonersBack(){
	for(uint i=0;i<loners.size();++i)
		clusters.push_back(vector<int>(1,loners[i]));
}


int analyzePairings(){
	std::vector<int> clustering(pairings.size(),-1);
//	clustering.resize(pairings.size(),-1);
	int numclusters=0;
	std::vector<int> tosee,seen;
//	std::vector< std::vector<int> > clusters(pairings.size());
	clusters.clear();
	clusters.resize(pairings.size());
	int current;
//	for(int i=300; i >=0; i--){
		for(int i=pairings.size()-1; i >=0; i--){
		if(clustering[i]!=-1) continue;
		if(pairings[i].size()==0){//it saw no marked pts
			clusters[numclusters].push_back(i);
			clustering[i]=numclusters++;
			continue;
		}
		clusters[numclusters].push_back(i);
		clustering[i]=numclusters;
		tosee=pairings[i];
		while(tosee.size()){
			current=tosee.back();
			tosee.pop_back();
			if(clustering[current]!=numclusters){ //if we haven't seen this one before in this tree
//				seen.push_back(current);
				//see if we just discovered a previously explored tree:
				if(clustering[current] != -1){ //yes, we did!
					//this tree is fully explored, so send all the nodes to the seen
					//clustering gives the index of the clusters array
					int oldind=clustering[current];
					for(uint j=0;j<clusters[oldind].size();j++){
						clustering[clusters[oldind][j]]=numclusters; //mark all the heads in the found cluster with the new mark
					}
					//copy all of the points in the found cluster into this one
					clusters[numclusters].insert(clusters[numclusters].end(),clusters[oldind].begin(),clusters[oldind].end());
					clusters[oldind].clear();
				}
				else{
					clusters[numclusters].push_back(current);
					if(pairings[current].size())
						tosee.insert(tosee.end(),pairings[current].begin(),pairings[current].end());
				}
				//now mark the current head as ours:
				clustering[current]=numclusters;
			}
		}
		//finished exploring.  increment and continue
		numclusters++;
	}
	clusters.resize(numclusters);
	clusterindices2=clustering;
	return numclusters;
//	printAll("points in cluster ",".",clustering);
//

}


void checkClustering(int min_pts_per_cluster=1){
	recomputeClusters();
	int zeroclusters=0;
	for(uint i=0;i<clusters.size();i++)
		if(clusters[i].size()==1)
			zeroclusters++;
	ROS_INFO("%d zero sized clusters",zeroclusters);
   //now we need to check to see if any of the heads that were NOT clustered together are closer than cluster_tol+smaller_tol.
   //This covers the exception noted in the code block above
   //this is where an adversary could really kill this algorithm, since this check could be polynomial in cloud size. in real circumstances, it is very quick.
   vector<int> indices1;
   vector<int> indices;
   int searchcount=0,comparecount=0,mergecount=0;
   double distthresh=_cluster_tol*_cluster_tol; //radius search gives squared distances...
//   double automergethresh=_cluster_tol*_cluster_tol/4.0; //pts are always in same cluster if they are both less than half the tolerance away from the same point
   timeval t1,t0=g_tick();
   double time1,time2;
//   bool verbose=false;
   for(uint i=0; i<clusterindices2.size();i++){ //for every head
      _sc2->NNN(_cloud->points[heads[i]],indices,2.0*_cluster_tol,true);
	  for(uint j=0;j<indices.size();j++){ //for every head that is close to this head
		 if(clusterindices2[indices[j]] != clusterindices2[i] && indices[j] > i){//if the two heads are not in the same cluster (and only check each combo once)

			//now we have to find if there is a point between head a and head b that is < cluster_tol from both
			//our best chance of finding it is to start searching at a point halfway between them
			PointT heada=_cloud->points[heads[i]], headb=_cloud->points[heads[indices[j]]];
			PointT inbetween;
			inbetween.x=(heada.x+headb.x)/2.0;
			inbetween.y=(heada.y+headb.y)/2.0;
			inbetween.z=(heada.z+headb.z)/2.0;
			searchcount++;
			t1=g_tick();
		    _sc2->NNN(inbetween,indices1,_cluster_tol); //search on the full tree
			time2+=g_tock(t1);
		    if(!indices1.size())
		    	continue;
			//must match all points that return against each other
			bool shouldmerge=false;
			int mergefrom,mergeinto;
			t1=g_tick();
			for(int k=0;k<((int)indices1.size())-1;k++){
				for(uint m=k+1;m<indices1.size();m++){
					if(clusterindices2[clusterindices[indices1[k]]] != clusterindices2[clusterindices[indices1[m]]]  && (clusterindices2[clusterindices[indices1[k]]] == clusterindices2[i] || clusterindices2[clusterindices[indices1[m]]] ==clusterindices2[i] )){ //if different clusters
						comparecount++;
						if((pcl::squaredEuclideanDistance(_cloud->points[indices1[k]],_cloud->points[indices1[m]] ) < distthresh)){
						   mergefrom=clusterindices2[clusterindices[indices1[k]]];
						   mergeinto=clusterindices2[clusterindices[indices1[m]]];
						   if(!((mergefrom !=clusterindices2[i] && mergefrom !=clusterindices2[indices[j]]) || (mergeinto !=clusterindices2[i] && mergeinto !=clusterindices2[indices[j]]))){
							   shouldmerge=true;
								mergecount++;
							   break;  //TODO: shouldn't actually break, but rather see if multiple clusters should merge
						   }
//						   else
//							   ROS_ERROR("Found another cluster's match");
						}
					}
				}
			   if(shouldmerge){
				  //there is a point that these two clusters share -> they should be merged
				  //rename all heads in cluster b to cluster a
				  int clusterb=clusterindices2[indices[j]];
				  for(uint m=0;m<clusterindices2.size();m++) if(clusterindices2[m]==clusterb) clusterindices2[m]=clusterindices2[i];
				  break;
			   } //if the point is shared
			} //for every point close to both heads

			time1+=g_tock(t1);
		 } //if a head is close, and from a different cluster
	  }//for every nearby head
   }  //for every head
   t1=g_tick();
   recomputeClusters();
   //erase the deleted clusters, and the ones under the minimum size
   std::vector<int> deletedclusters;
   std::vector< std::vector<int> > clusters2(clusters.size());
   int numclusters=0;
   for(int i=clusters.size()-1;i>=0; i--)
	   if((int)clusters[i].size() >= min_pts_per_cluster){
		clusters[i].swap(clusters2[numclusters++]);
	   }
   clusters.swap(clusters2);
   clusters.resize(numclusters);
//	  if((int)clusters[i].size() < min_pts_per_cluster){
//		 clusters.erase(clusters.begin()+i);
//		 deletedclusters.push_back(i);
//   }
   std::cout<<searchcount<<" searches took "<<time2<<"   "<<comparecount<<" compares for "<<mergecount<<" merges took "<<time1<<" rest took "<<g_tock(t1)<<endl;
}


//search through the heads and find heads that are between 1 and 3 * cluster_tol from a particular head
void findMissedCandidates(){
  tosearch.clear();
   std::vector<vector<int> > miniclusters(heads.size());
   for(uint i=0; i<clusterindices.size();i++)
      miniclusters[clusterindices[i]].push_back(i);
   int singles=0;
   vector<int> indices,indices1;
   tosearch.resize(clusterindices2.size());
   double distthresh=_cluster_tol*_cluster_tol; //radius search gives squared distances...

   for(uint i=0; i<clusterindices2.size();i++){ //for every head
      if(clusterindices2[i]==-1) continue;   //skip the head if it is a loner
      _sc2->NNN(_cloud->points[heads[i]],indices,2.0*_cluster_tol+farpt[i],true);     //search for nearby heads
//      _sc2->NNN(_cloud->points[heads[i]],indices,3.0*_cluster_tol,true);     //search for nearby heads
      for(uint j=0;j<indices.size();j++){ //for every head that is close to this head
         if(clusterindices2[indices[j]]==-1) continue;  //skip the head if it is a loner
         if(clusterindices2[indices[j]] == clusterindices2[i] || indices[j] < (int)i)//if the two heads are not in the same cluster (and only check each combo once)
            continue;
//         if(miniclusters[i].size()==1 ) continue;
//         if(miniclusters[indices[j]].size()==1 ) continue;
//         if(pcl::squaredEuclideanDistance(_cloud->points[heads[indices[j]]],_cloud->points[heads[i]]) < distthresh) singles++;
//         if(initialgrabs[indices[j]]==2) singles++;
//         if(pcl::squaredEuclideanDistance(_cloud->points[heads[indices[j]]],_cloud->points[heads[i]]) < 4.0* distthresh) continue;
         //now indices[j] is a nearby head that is from a different cluster
         tosearch[i].push_back(indices[j]);
         singles+=indices.size()-indices1.size();
      }//for every head that is close to this head
   }//for every head

   cout<<singles<<" singles."<<endl;
}

//evaluate whether the findMissedCandidates is finding everything correctly
void checktoSearch(std::vector<int> &labels,int numclusters){
	std::vector< std::vector<int> > headclusters(numclusters);
	std::vector< std::vector<int> > reclusters(numclusters);
	std::vector< std::vector<int> > reclustercheck(numclusters);
	for(uint i=0; i<heads.size();++i){
		headclusters[labels[heads[i]]].push_back(i);
		if(!count(reclusters[labels[heads[i]]].begin(),reclusters[labels[heads[i]]].end(), clusterindices2[i]))
			reclusters[labels[heads[i]]].push_back(clusterindices2[i]);
	}
//	headclusters maps from cluster number to heads in that cluster
//	reclusters maps from cluster number to current cluster #s in that cluster

	//now to check tosearch:
	//for each end cluster:
	for(uint endcluster=0; endcluster<reclusters.size();++endcluster){


	}


}



//c1 and c2 are heads with different clusters that should be merged
//assume c1<c2
void merge(int c1, int c2){
   //merge the higher index into the lower index
   int cl2=clusterindices2[c2];
   for(uint i=0; i<clusters[cl2].size();++i){
      clusterindices2[clusters[cl2][i]] = clusterindices2[c1]; //remap the cluster that each head in c2's cluster points to
   }
   clusters[clusterindices2[c1]].insert(clusters[clusterindices2[c1]].end(), clusters[cl2].begin(),clusters[cl2].end()); //move all the heads from cluster 2 to cluster 1
   //now check that no head in the merged cluster wants to search against a head in its own cluster:
   for(uint i=0; i<tosearch.size();++i){
      for(uint j=0;j<tosearch[i].size();++j){
         if(clusterindices2[tosearch[i][j]] == clusterindices2[i] )
            tosearch[i].erase(tosearch[i].begin() + j);
      }
   }
}

bool isMatch(std::vector<int> &pts1, std::vector<int> &pts2, double &distthresh){
   for(uint k=0; k<pts1.size();++k){
      for(uint m=0; m<pts2.size();++m){
         if(pcl::squaredEuclideanDistance(_cloud->points[pts1[k]],_cloud->points[pts2[m]] ) <= distthresh)
            return true;
      }
   }
   return false;
}


//given a set of indices within cluster_tol of a point, find the points that are of different clusters, and < cluster_tol from each other
bool findMatches(std::vector<int> &pts, std::vector<int> &clusts, std::vector<int> &clustremap){
   double distthresh=_cluster_tol*_cluster_tol; //radius search gives squared distances...
   clusts.clear();
   std::vector< std::vector<int> > clustpts;
   for(uint i=0; i<pts.size();++i){
      bool found=false;
      for(uint j=0;j<clusts.size();++j){
         if(clusts[j]==clusterindices2[clusterindices[pts[i]]]){
            clustpts[j].push_back(pts[i]);
            found=true;
            break;
         }
      }
      if(!found){
         clusts.push_back(clusterindices2[clusterindices[pts[i]]]);
         clustpts.push_back(std::vector<int>(1,pts[i]));
      }
   }
   bool ret=false;
   //now all the points are organized in clusters
   //so compare the clusters to each other by pts:
   clustremap=clusts;
   for(uint i=0; i<clusts.size()-1;++i)
      for(uint j=i+1;j<clusts.size();++j){
         if(clustremap[j]==clustremap[i]) continue; //we've already matched these clusters!
         if(isMatch(clustpts[i],clustpts[j],distthresh)){
            clustremap[j]=clustremap[i];

            ret=true;
         }
      }
   return ret;
}

//merge a set of clusters, indicated by the remapping
int merge( std::vector<int> &clusts, std::vector<int> &clustremap){

   int removedsearches=0;
   int numclusters=0;
   for(uint c1=0; c1<clusts.size();++c1){
      if(clustremap[c1]==clusts[c1]) continue;
      numclusters++;
      //merge the higher index into the lower index
      int cl2=clusts[c1], cl1=clustremap[c1];
      for(uint i=0; i<clusters[cl2].size();++i){
         clusterindices2[clusters[cl2][i]] = cl1; //remap the cluster that each head in c2's cluster points to
      }
      clusters[cl1].insert(clusters[cl1].end(), clusters[cl2].begin(),clusters[cl2].end()); //move all the heads from cluster 2 to cluster 1
      clusters[cl2].clear();
   }

   for(uint c1=0; c1<clustremap.size();++c1){
      if(clustremap[c1]==clusts[c1] || clustremap[c1]==-1) continue;
      int cl1=clustremap[c1];
      for(uint i=0; i<clustremap.size();++i) if(clustremap[i]==cl1) clustremap[i]=-1; //remove this cluster from the remapping
      //now check that no head in the merged cluster wants to search against a head in its own cluster:
      //TODO: why does this not work to just search in the new cluster?
      for(uint i=0; i<clusters[cl1].size();++i){ //for all the points in the new cluster
         for(int j=tosearch[i].size()-1; j>=0;--j){ //for all the heads that that pt wants to search
            if(clusterindices2[tosearch[i][j]] == clusterindices2[i] ){
               tosearch[i].erase(tosearch[i].begin() + j);
               removedsearches++;
            }
         }
      }
   }
//   cout<<"   nc: "<<numclusters<<"    rs: "<<removedsearches<<endl;
   return removedsearches;
}

void checkClustering3(int min_pts_per_cluster=1){
// recomputeClusters();
   //now we need to check to see if any of the heads that were NOT clustered together are closer than cluster_tol+smaller_tol.
   //This covers the exception noted in the code block above
   //this is where an adversary could really kill this algorithm, since this check could be polynomial in cloud size. in real circumstances, it is very quick.
   vector<int> indices1;
   vector<int> indices;
   int searchcount=0,comparecount=0,mergecount=0;
   double distthresh=_cluster_tol*_cluster_tol; //radius search gives squared distances...

//   double automergethresh=_cluster_tol*_cluster_tol/4.0; //pts are always in same cluster if they are both less than half the tolerance away from the same point
   timeval t1,t0=g_tick();
   double time1=0,time2=0;

   findMissedCandidates(); //generates a tosearch variable, which indicates pairs of heads to compare
   cout<<" find candidates took "<<g_tock(t0)<<endl;

   vector<int> clusts, clustremap;
   for(uint i=0; i<tosearch.size();++i){
      while(tosearch[i].size()){
         int pt2=tosearch[i].back();
         PointT heada=_cloud->points[heads[i]], headb=_cloud->points[heads[tosearch[i].back()]];
         tosearch[i].pop_back(); //not going to do this search again

         PointT inbetween;
         inbetween.x=(heada.x+headb.x)/2.0;
         inbetween.y=(heada.y+headb.y)/2.0;
         inbetween.z=(heada.z+headb.z)/2.0;
         searchcount++;
         t1=g_tick();
          _sc2->NNN(inbetween,indices1,_cluster_tol); //search on the full tree
         time2+=g_tock(t1);
         t1=g_tick();
         if(indices1.size()<2) continue;
         if(findMatches(indices1,clusts,clustremap)){//a merging was found!
//            bool correctmatch=false;
//            for(uint k=0;k<clusts.size(); k++)
//               if(clusts[k]!=clustremap[k] && (clusts[k]==clusterindices2[pt2] || clusts[k]==clusterindices2[i]))
//                  correctmatch=true;
//            if(correctmatch && pcl::squaredEuclideanDistance(_cloud->points[heads[i]],_cloud->points[heads[pt2]]) < 4.0* distthresh)
//               cout<<heads[i]<<" and "<<heads[pt2]<<" were "<<pcl::euclideanDistance(_cloud->points[heads[i]],_cloud->points[heads[pt2]])<<" apart"<<endl;
            comparecount+=merge(clusts,clustremap);   //this may remove a bunch of tosearch subindices, but since we are in a while loop, we just keep going...
            mergecount++;
         }
         time1+=g_tock(t1);
      }
   }
   t1= g_tick();
   std::vector< std::vector<int> > clusters2(clusters.size());
   int numclusters=0;
   for(int i=clusters.size()-1;i>=0; i--)
      if((int)clusters[i].size() >= min_pts_per_cluster){
      clusters[i].swap(clusters2[numclusters++]);
      }
   clusters.swap(clusters2);
   clusters.resize(numclusters);
   int numheads=0;
   for(int i=clusters.size()-1;i>=0; i--)
      numheads+=clusters[i].size();
   cout<<"clusters contain "<<numheads<<" heads"<<endl;

   std::cout<<searchcount<<" searches took "<<time2<<"   "<<comparecount<<" compares for "<<mergecount<<" merges took "<<time1<<" rest took "<<g_tock(t1)<<endl;
}




void checkClustering2(int min_pts_per_cluster=1){
//	recomputeClusters();
   //now we need to check to see if any of the heads that were NOT clustered together are closer than cluster_tol+smaller_tol.
   //This covers the exception noted in the code block above
   //this is where an adversary could really kill this algorithm, since this check could be polynomial in cloud size. in real circumstances, it is very quick.
   vector<int> indices1;
   vector<int> indices;
   int searchcount=0,comparecount=0,mergecount=0;
   double distthresh=_cluster_tol*_cluster_tol; //radius search gives squared distances...

//   double automergethresh=_cluster_tol*_cluster_tol/4.0; //pts are always in same cluster if they are both less than half the tolerance away from the same point
   timeval t1,t0=g_tick();
   double time1,time2;
//   bool verbose=false;
   for(uint i=0; i<clusterindices2.size();i++){ //for every head
	   if(clusterindices2[i]==-1) continue;	//skip the head if it is a loner
      _sc2->NNN(_cloud->points[heads[i]],indices,3.0*_cluster_tol,true);
	  for(uint j=0;j<indices.size();j++){ //for every head that is close to this head
		 if(clusterindices2[indices[j]]==-1) continue;	//skip the head if it is a loner
		 if(clusterindices2[indices[j]] != clusterindices2[i] && indices[j] > i){//if the two heads are not in the same cluster (and only check each combo once)
			 //if the heads are within 2*cluster_tol, they are not related, or they would already be clustered.
			 if(pcl::squaredEuclideanDistance(_cloud->points[heads[indices[j]]],_cloud->points[heads[i]]) < 4.0* distthresh)
				 continue;
			//now we have to find if there is a point between head a and head b that is < cluster_tol from both
			//our best chance of finding it is to start searching at a point halfway between them
			PointT heada=_cloud->points[heads[i]], headb=_cloud->points[heads[indices[j]]];
			PointT inbetween;
			inbetween.x=(heada.x+headb.x)/2.0;
			inbetween.y=(heada.y+headb.y)/2.0;
			inbetween.z=(heada.z+headb.z)/2.0;
			searchcount++;
			t1=g_tick();
		    _sc2->NNN(inbetween,indices1,_cluster_tol); //search on the full tree
			time2+=g_tock(t1);
		    if(!indices1.size())
		    	continue;
			//must match all points that return against each other
			bool shouldmerge=false;
			int mergefrom,mergeinto;
			t1=g_tick();
			for(int k=0;k<((int)indices1.size())-1;k++){
				for(uint m=k+1;m<indices1.size();m++){
					if(clusterindices2[clusterindices[indices1[k]]] != clusterindices2[clusterindices[indices1[m]]]  && (clusterindices2[clusterindices[indices1[k]]] == clusterindices2[i] || clusterindices2[clusterindices[indices1[m]]] ==clusterindices2[i] )){ //if different clusters
						comparecount++;
						if((pcl::squaredEuclideanDistance(_cloud->points[indices1[k]],_cloud->points[indices1[m]] ) < distthresh)){
						   mergefrom=clusterindices2[clusterindices[indices1[k]]];
						   mergeinto=clusterindices2[clusterindices[indices1[m]]];
						   if(!((mergefrom !=clusterindices2[i] && mergefrom !=clusterindices2[indices[j]]) || (mergeinto !=clusterindices2[i] && mergeinto !=clusterindices2[indices[j]]))){
							   shouldmerge=true;
								mergecount++;
							   break;  //TODO: shouldn't actually break, but rather see if multiple clusters should merge
						   }
//						   else
//							   ROS_ERROR("Found connection between  %d and %d  while inspecting %d and %d ",mergefrom, mergeinto,clusterindices2[i],clusterindices2[indices[j]] );
						}
					}
				}
			   if(shouldmerge){
				  //there is a point that these two clusters share -> they should be merged
				  //rename all heads in cluster b to cluster a
				  int clusterb=clusterindices2[indices[j]];
				  for(uint m=0;m<clusterindices2.size();m++) if(clusterindices2[m]==clusterb) clusterindices2[m]=clusterindices2[i];
				  break;
			   } //if the point is shared
			} //for every point close to both heads

			time1+=g_tock(t1);
		 } //if a head is close, and from a different cluster
	  }//for every nearby head
   }  //for every head
   t1=g_tick();
//   recomputeClusters();
   //erase the deleted clusters, and the ones under the minimum size
   std::vector<int> deletedclusters;
   std::vector< std::vector<int> > clusters2(clusters.size());
   int numclusters=0;
   for(int i=clusters.size()-1;i>=0; i--)
	   if((int)clusters[i].size() >= min_pts_per_cluster){
		clusters[i].swap(clusters2[numclusters++]);
	   }
   clusters.swap(clusters2);
   clusters.resize(numclusters);
//	  if((int)clusters[i].size() < min_pts_per_cluster){
//		 clusters.erase(clusters.begin()+i);
//		 deletedclusters.push_back(i);
//   }
   std::cout<<searchcount<<" searches took "<<time2<<"   "<<comparecount<<" compares for "<<mergecount<<" merges took "<<time1<<" rest took "<<g_tock(t1)<<endl;
}



//   template <typename PointT>
//   bool checkcluster( pcl::PointCloud<PointT> &cloud,std::vector<int>  &inds, double tol,int pt){
//         for(uint i=0;i<inds.size();i++)
//            if(pcl::euclideanDistance(cloud.points[pt],cloud.points[inds[i]]) > tol){
//               ROS_ERROR("Point index %d is %f from point %d ",i,pcl::euclideanDistance(cloud.points[pt],cloud.points[inds[i]]),pt);
//               return false;
//            }
//         return true;
//   }
//
//   //performs nearest neighbor search in a simplistic fashion
//   template <typename PointT>
//   void bigNN(pcl::PointCloud<PointT> &cloud, PointT pt, std::vector<int> &inds, double radius){
//      inds.clear();
//      double r2=radius*radius;
//      double smallerrad=radius/sqrt(3);
//      double diffx,diffy,diffz;
//      for(uint i=0;i<cloud.points.size(); i++){
//         diffx=fabs(cloud.points[i].x - pt.x);
//         diffy=fabs(cloud.points[i].y - pt.y);
//         diffz=fabs(cloud.points[i].z - pt.z);
//         if(diffx < radius && diffy < radius && diffz < radius){ //in outer box
//            if(diffx < smallerrad && diffy < smallerrad && diffz < smallerrad) //also in inner box - include!
//               inds.push_back(i);
//            else//between the boxes: check for actual distance
//               if(diffx*diffx+diffy*diffy+diffz*diffz < r2)
//                  inds.push_back(i);
//         }
//      }
//   }
//
//   //performs nearest neighbor search in a simplistic fashion
//   //this version assumes inds is the size of cloud, and just marks the points
//   template <typename PointT>
//   void bigNN(pcl::PointCloud<PointT> &cloud, PointT &pt, std::vector<int> &inds, double radius, int mark){
//      double r2=radius*radius;
//      double smallerrad=radius/sqrt(3);
//      double diffx,diffy,diffz;
//      for(uint i=0;i<cloud.points.size(); i++){
//         diffx=fabs(cloud.points[i].x - pt.x);
//         diffy=fabs(cloud.points[i].y - pt.y);
//         diffz=fabs(cloud.points[i].z - pt.z);
//         if(diffx < radius && diffy < radius && diffz < radius){ //in outer box
////            if(diffx < smallerrad && diffy < smallerrad && diffz < smallerrad) //also in inner box - include!
////               inds[i]=mark;
////            else//between the boxes: check for actual distance
//               if(diffx*diffx+diffy*diffy+diffz*diffz < r2)
//                  inds[i]=mark;
//         }
//      }
//   }
//
//   //simply runs radius search to grab all the points nearby until everything is taken
//   //results in each pt assigned a label, and a list of who grabbed each point
//   //this version uses the bigNN function
//   template <typename PointT>
//   void bigDownsample(pcl::PointCloud<PointT> &cloud, double cluster_tol=.2){
//      timeval t0=g_tick();
//      clusterindices.resize(cloud.size(),-1);
//      std::cout<<"resizing took "<<g_tock(t0)<<endl; t0=g_tick();
//      int searchcount=0;
//      for(uint i=0; i<clusterindices.size();i++){
//       if(clusterindices[i]==-1){    //if no one has claimed this point, make it a head
//         heads.push_back(i);
//         searchcount++;
//         bigNN(cloud,cloud.points[i],clusterindices,cluster_tol,heads.size()-1);
//       }
//      }
//      std::cout<<"searching took "<<g_tock(t0)<<" for "<<searchcount<<" searches"<<endl;
//   }

//   template <typename PointT>
//   void testNNs(pcl::PointCloud<PointT> &cloud, double cluster_tol=.2){
//      double setupanntime,setupflanntime;
//      timeval t0=g_tick();
//      pcl::KdTreeANN<PointT> tree;
//      tree.setInputCloud(cloud.makeShared());
//      setupanntime= g_tock(t0); t0=g_tick();
////      std::cout<<"ANN setupcloud took "<<g_tock(t0)<<endl; t0=g_tick();
//      pcl::KdTreeFLANN<PointT> ftree;
//      ftree.setInputCloud(cloud.makeShared());
//      setupflanntime= g_tock(t0); t0=g_tick();
////      std::cout<<"FLANN setupcloud took "<<g_tock(t0)<<endl; t0=g_tick();
//      vector<int> indices,indices2, indices3;
//      vector<float> dists;
//      double time1=0,time2=0,time3=0;
//
//      srand(getUsec());
//      srand(0);
//      for(int trialnum=0; trialnum<randseed;trialnum++){
//         srand(getUsec());
//        int ind = rand()%cloud.points.size();
//        indices3.clear();
//        indices2.clear();
//        indices.clear();
//        t0=g_tick();
//        tree.radiusSearch(cloud.points[ind],cluster_tol,indices,dists);
//        time1+=g_tock(t0);
//        t0=g_tick();
//        ftree.radiusSearch(cloud.points[ind],cluster_tol,indices3,dists);
//        time3+=g_tock(t0);
//        t0=g_tick();
//        bigNN(cloud,cloud.points[ind],indices2,cluster_tol);
//        time2+=g_tock(t0);
////        if(indices.size() != indices2.size()){
////           ROS_ERROR("trial: %d   index sizes %d and %d do not match!",trialnum,indices.size(),indices2.size());
////           if(indices2.size()> indices.size())
////              if(checkcluster(cloud,indices2,cluster_tol,ind))
////                 ROS_ERROR("trial: %d   all points in indices2 are ok...",trialnum);
////        }
////
////        for(uint i=0;i<indices.size();i++){
////           bool found=false;
////           for(uint j=0;j<indices2.size();j++){
////              if(indices[i]==indices2[j]){
////                 found=true;
////                 break;
////              }
////           }
////           if(!found){
////              if(indices.size() != indices2.size()){
////                 ROS_ERROR("index sizes %d and %d do not match!",indices.size(),indices2.size());
////                 return;
////              }
////
////           }
//
//
////        }
//      }
//      float div=(float)randseed;
//      ROS_INFO("%d size cloud, setup: ANN: %.3f, FLANN: %.3f  Ave Search: ANN: %.6f  FLANN: %.6f  naive: %.6f",cloud.points.size(), setupanntime,setupflanntime,time1/div,time3/div,time2/div);
////      std::cout<<"searching took:  ANN: "<<time1/div<<" FLANN: "<<time3/div<<" naive: "<<time2/div<<" for "<<randseed<<" searches"<<endl;
//   }

//   //divides cloud into 8 clusters
//   template <typename PointT>
//   void planeseg(pcl::PointCloud<PointT> &cloud, double x, double y, double z){
//      Eigen::Vector4f vec;
//      pcl::compute3DCentroid(cloud,vec);
//      timeval t0=g_tick();
//      std::vector<int> ind;
//      clusterindices.resize(cloud.points.size(),0);
//      for(uint i=0;i<cloud.points.size(); i++){
//         if(cloud.points[i].z > z) clusterindices[i] +=4;
//         if(cloud.points[i].y > y) clusterindices[i] +=2;
//         if(cloud.points[i].x > x) clusterindices[i] +=1;
//      }
//      std::cout<<"planeseg took:  "<<g_tock(t0)<<"  for "<<cloud.points.size()<<" indices."<<std::endl;
//   }




};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Decompose a Point Cloud into clusters based on the Euclidean distance between points. Uses Hierarchical Clustering, making it faster
  * \param cloud the point cloud message
  * \param cluster_tol the spatial cluster tolerance as a measure in L2 Euclidean space
  * \param cclusters the resultant clusters containing point indices (as a vector of vector of ints)
  * \param min_pts_per_cluster minimum number of points that a cluster may contain (default = 1)
  */
template <typename PointT>
void segfast(pcl::PointCloud<PointT> &cloud, std::vector<std::vector<int> > &clusters, double cluster_tol=.2, int min_pts_per_cluster=1){

    PtMap<PointT> pmap(cloud,cluster_tol);

	timeval t0;
	timeval ttot=g_tick();
	t0=g_tick();
	pmap.simpleDownsampleNNN2(cloud,cluster_tol);
	cout<<"downsampleNNN2 took: "<<g_tock(t0)<<std::endl;
	cout<<std::endl<<"cluster heads: "<<std::endl;

	t0=g_tick();
	int clusternum=pmap.analyzePairings();
	cout<<"ptMap.analyzepairings took: "<<g_tock(t0)<<"  for "<<clusternum<<std::endl;


	t0=g_tick();
	pmap.swapOutLoners();
	cout<<"swapOutLoners took: "<<g_tock(t0)<<std::endl;

	t0=g_tick();
	pmap._sc2->useInds(pmap.heads);
	cout<<"useinds took: "<<g_tock(t0)<<std::endl;

	t0=g_tick();
	pmap.checkClustering3();
	pmap.addLonersBack();
	cout<<"check cluster took: "<<g_tock(t0)<<std::endl;
	cout<<"full cluster time: "<<g_tock(ttot)<<std::endl;
	cout<<"cloud size: "<<cloud.points.size()<<" heads: "<<pmap.heads.size()<<" clusters: "<<pmap.clusters.size() <<std::endl;

	clusters.swap(pmap.clusters);

}


//give an approximate, quick segmentation:
template <typename PointT>
int quikseg(pcl::PointCloud<PointT> &cloud, std::vector<int>  clusterind, double cluster_tol=.2){

    PtMap<PointT> pmap(cloud,cluster_tol);

	timeval t0;
	timeval ttot=g_tick();
	t0=g_tick();
	pmap.simpleDownsampleNNN2(cloud,cluster_tol);
	cout<<"downsampleNNN2 took: "<<g_tock(t0)<<std::endl;
	cout<<std::endl<<"cluster heads: "<<std::endl;

	t0=g_tick();
	int clusternum=pmap.analyzePairings();
	cout<<"ptMap.analyzepairings took: "<<g_tock(t0)<<"  for "<<clusternum<<std::endl;

	for(uint i=0;i<pmap.clusterindices.size();++i)
		pmap.clusterindices[i]=pmap.clusterindices2[pmap.clusterindices[i]];

	clusterind.swap(pmap.clusterindices);
	return clusternum;
}

//give an approximate, quick segmentation:
template <typename PointT>
void quikdownsample(pcl::PointCloud<PointT> &cloud, std::vector<int>  heads, double cluster_tol=.2){

    PtMap<PointT> pmap(cloud,cluster_tol);

	timeval t0;
	timeval ttot=g_tick();
	t0=g_tick();
	pmap.simpleDownsampleNNN(cloud,cluster_tol);
	heads.swap(pmap.heads);
	cout<<"downsampleNNN took: "<<g_tock(t0)<<std::endl;
}

//a debug function to figure out where segfast goes wrong.
template <typename PointT>
void checksegfast(pcl::PointCloud<PointT> &cloud, std::vector<int>  &labels, double cluster_tol=.2, int min_pts_per_cluster=1){

    PtMap<PointT> pmap(cloud,cluster_tol);

	timeval t0;
	timeval ttot=g_tick();
	t0=g_tick();
	pmap.simpleDownsampleNNN2(cloud,cluster_tol);
	cout<<"downsampleNNN2 took: "<<g_tock(t0)<<std::endl;
	cout<<std::endl<<"cluster heads: "<<std::endl;

	t0=g_tick();
	int clusternum=pmap.analyzePairings();
	cout<<"ptMap.analyzepairings took: "<<g_tock(t0)<<"  for "<<clusternum<<std::endl;


	t0=g_tick();
	pmap.swapOutLoners();
	cout<<"swapOutLoners took: "<<g_tock(t0)<<std::endl;

	t0=g_tick();
	pmap._sc2->useInds(pmap.heads);
	cout<<"useinds took: "<<g_tock(t0)<<std::endl;

	t0=g_tick();
	pmap.checkClustering3();
	pmap.addLonersBack();
	cout<<"check cluster took: "<<g_tock(t0)<<std::endl;
	cout<<"full cluster time: "<<g_tock(ttot)<<std::endl;
	cout<<"cloud size: "<<cloud.points.size()<<" heads: "<<pmap.heads.size()<<" clusters: "<<pmap.clusters.size() <<std::endl;


}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Decompose a Point Cloud into clusters based on the Euclidean distance between points. Uses Hierarchical Clustering, making it faster
  * \param cloud the point cloud message
  * \param cluster_tol the spatial cluster tolerance as a measure in L2 Euclidean space
  * \param cclusters the resultant clusters containing point indices (as a vector of vector of ints)
  * \param min_pts_per_cluster minimum number of points that a cluster may contain (default = 1)
  */
template <typename PointT>
void extractEuclideanClustersFast2(pcl::PointCloud<PointT> &cloud, std::vector<std::vector<int> > &clusters, double cluster_tol=.2, int min_pts_per_cluster=1){
   double smaller_tol = cluster_tol/2.1;
   pcl::KdTreeFLANN<PointT> tree,tree2;
   tree.setInputCloud(cloud.makeShared());
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
   tree2.setInputCloud(cloud.makeShared(),boost::make_shared<std::vector<int> >  (heads));
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
                        if(!((mergefrom !=minitree2[i] && mergefrom !=minitree2[indices[j]]) || (mergeinto !=minitree2[i] && mergeinto !=minitree2[indices[j]]))){
                        shouldmerge=true;
                        break;  //TODO: shouldn't actually break, but rather see if multiple clusters should merge
                        }
                     }

                }
               if(shouldmerge){

                  //there is a point that these two clusters share -> they should be merged
//                  if(globalverbose)
//                     ROS_INFO("head %d (in cluster %d) shares a point with head %d (in cluster %d). Merging cluster %d (%d pts) into cluster %d (%d pts)",
//                     i,minitree2[i],indices[j],minitree2[indices[j]],minitree2[indices[j]],clusters[minitree2[indices[j]]].size(), minitree2[i],  clusters[minitree2[i]].size());
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






////a function to determine how much overlap there is between clusters
////cloud: point cloud to work over
////heads: heads of clusters, represented by indexes into the cloud
////overlaps: for each point, the number of clusters that are within tol
//void calculateOverlap(pcl::PointCloud<PointT> &cloud, std::vector<int> &heads, std::vector<int> &overlapcount, double tol){
//	//brute force method...
//   overlapcount.resize(cloud.points.size());
//   pcl::KdTreeFLANN<PointT> tree;
//   tree.setInputCloud(cloud.makeShared());
//   vector<int> indices;
//   vector<float> dists;
//   for(uint i=0; i<clusterindices.size();i++){
//	 if(clusterindices[i]==-1){    //if no one has claimed this point, make it a head
//		heads.push_back(i);
//		if(!tree.radiusSearch(cloud.points[ptindices[i]],cluster_tol,indices,dists))  //find all the points close to this point
//		   ROS_WARN("radius search failed!");
//		for(uint j=0;j<indices.size();j++){
//			clusterindices[rmap[indices[j]]]=heads.size()-1; //assign these points to the current head.
//									   // this overwrites previous claims, but it's ok
//		}
//	 }
//   }
//}







#endif /* SEGFAST_H_ */
