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



#ifndef CLUSTEREVALUATION_HPP_
#define CLUSTEREVALUATION_HPP_


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

//convert a vector of vector of ints into a vector with the points labeled with the cluster number
void unwrapClusters(std::vector<std::vector<int> > &clusters, std::vector<int> &labels){
	int cloudsize=0;
	for(uint i=0;i<clusters.size();++i)
		cloudsize+=clusters[i].size();
	labels.clear();
	labels.resize(cloudsize,-1);

	for(uint i=0;i<clusters.size();++i)
		for(uint j=0;j<clusters[i].size();++j){
			if(clusters[i][j] >= cloudsize){
				ROS_ERROR("label index out of bounds!");
				return;
			}
			labels[clusters[i][j]]=i;
		}

	for(uint i=0;i<labels.size();++i)
		if(labels[i]<0)
			ROS_ERROR("label %i was not assigned!",(int)i);



}


////convert vector of PointIndices to a vector of vector of ints
//void PItoVector(std::vector<pcl::PointIndices > &clusterin,std::vector<std::vector<int> > &clusterout ){
//	clusterout.resize(clusterin.size());
//	for(uint i=0;i<clusterin.size();++i)
//		   clusterout[i]=clusterin[i].indices;
//}

//checks to see if two clusters should have been joined
//if the return is false, indicating that the clusters should be joined, then
//pt1 and pt2 are points in the two clusters that prove that the clusters are within cluster_tol
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
   return valid_split;
}



struct MatchReport{
	MatchReport(std::string s1, std::string s2){
		suffix1=s1; suffix2=s2;
	}
	std::string suffix1,suffix2;
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




bool matchclusterings(std::vector<std::vector<int> > &c1, std::vector<std::vector<int> > &c2, MatchReport &report){

   bool failed = false;
   if(c1.size() != c2.size()){
      ROS_DEBUG("number of clusters does not match: c1->%d,  c2->%d",c1.size(),c2.size());
		failed=true;
	}
	report.pairing.resize(c2.size(),-1);
	for(uint i=0;i<c1.size();i++){ //match each cluster in c1 to one in c2
		bool paired=false;
		for(uint j=0;j<c2.size();j++){
			if(c1[i].size()==c2[j].size() && report.pairing[j]==-1){
				paired=true;
				for(uint k=0;k<c1[i].size();k++)
					if(c1[i][k]!=c2[j][k]){
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
		   ROS_DEBUG("cloud not find a match for cluster c1[%d], size %d",i,c1[i].size());
//			std::cout<<"cloud not find a match for cluster c1["<<i<<"], size "<<c1[i].size()<<std::endl;
			report.bad1.push_back(i);
			failed=true;
		}
	} // for each c1
	for(uint j=0;j<c2.size();j++)
	   if(report.pairing[j]==-1){
         ROS_DEBUG("cloud not find a match for cluster c2[%d], size %d",j,c2[j].size());
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
	      for(uint k=0;k<c1[report.bad1[i]].size();k++){ //for each of the indices in the c1 cluster
	         for(uint m=0;m<c2[report.bad2[j]].size();m++){ //compare to the indices of the c2 cluster
//	            cout<<"comparing "<<c1[report.bad1[i]][k]<<" to:  "<<c2[report.bad2[j]][m]<<std::endl;
              if(c1[report.bad1[i]][k]==c2[report.bad2[j]][m])
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
	   if(report.mapping1[i][0] != c1[report.bad1[i]].size())
	      ROS_DEBUG("%d points in c1[%d] were unaccounted for!",c1[report.bad1[i]].size()-report.mapping1[i][0],report.bad1[i]);
	}//for each bad1

   report.matched=!failed;
	return !failed;

}




//template <typename PointT>
//void compareResults(std::vector< std::vector<int> > &inds2, std::vector<pcl::PointIndices> &inds, MatchReport &report, pcl::PointCloud<PointT> &smallcloud){
//	//at the moment, only care about if it failed, not how...
//	matchclusterings(inds,inds2,report);
//	return;
//
//		if(!matchclusterings(inds,inds2,report)){
//		   std::vector<std::vector<int> > splits;
//		   splits=report.getSplits(1);
//	      if(splits.size())
//	         std::cout<<"checking cluster 1 splits: "<<std::endl;
//		   for (uint i = 0; i < splits.size(); ++i)
//		      //only check pairwise splits at the moment...
//		      checkClusterMistake(splits[i][0],splits[i][1],inds,smallcloud,.2,report.pt1,report.pt2);
//
//	      splits=report.getSplits(2);
//	      if(splits.size())
//	         std::cout<<"checking cluster 2 splits: "<<std::endl;
//	      for (uint i = 0; i < splits.size(); ++i)
//	         //only check pairwise splits at the moment...
//	         if(!checkClusterMistake(splits[i][0],splits[i][1],inds2,smallcloud,.2,report.pt1,report.pt2)){
//	        	 cout<<"check points "<<report.pt1<<" "<<report.pt2<<endl;
//	        	 break;
//	         }
//		}
//}

struct clusterResults{
	clusterResults(std::string _suffix, double _tol){
		suffix=_suffix;
		cluster_tol=_tol;
	}

	std::vector<std::vector<int> > inds;
	double ptime;
	std::string suffix; //indicates what algorithm was used
	//suffixes currently used:
	//          new-> extractFast
	//          std-> existing PCL implimentation

	double cluster_tol;

	void writeClusters(std::string filein){
	   //write number of clusters
	   std::ofstream outf;
	   char filename[500];
	   sprintf(filename,"%s_%.2f_%s_clustering",filein.c_str(),cluster_tol,suffix.c_str());
	   outf.open(filename,ios::out);
	   outf<<inds.size()<<endl;
	   for (uint i = 0; i < inds.size(); ++i) {
	      //for each cluster:
	      //write number of points, then list the indices
	      outf<<inds[i].size()<<" ";
	      for (uint j = 0; j < inds[i].size(); ++j)
	         outf<<inds[i][j]<<" ";
	      outf<<endl;
	   }
	   outf<<ptime<<endl;
	   outf.close();
	}

	int readClusters(string filein){
	   std::ifstream inf;
	   char filename[500];
	   sprintf(filename,"%s_%.2f_%s_clustering",filein.c_str(),cluster_tol,suffix.c_str());
	   inf.open(filename,ios::in);
	   if(!inf.is_open()){
		   ROS_ERROR("failed to load %s",filename);
		   return -1;
	   }
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




};



  //keeps  track of clustering result
template <typename PointT>
struct ClusterEvaluation{
	string filename;
	bool allmatched;

	double cluster_tol;
	pcl::PointCloud<PointT> smallcloud;
	//a vector of clustering results from different algorithms
	std::vector< clusterResults > clusterings;
	std::vector< MatchReport > reports;

	ClusterEvaluation(string f, double tolerance){
		filename=f;
		cluster_tol=tolerance;
		int ret= pcl::io::loadPCDFile(filename,smallcloud);
		if(ret) {
			cerr<<" failed to load "<<filename<<" the cloud file."<<endl;
			exit(-1);
		}
	}



	//load pre-calculated cluster results
	int loadResults(string suffix)    {
		clusterings.push_back(clusterResults(suffix,cluster_tol));
		int ret = clusterings.back().readClusters(filename);
		if(ret){
			cerr<<" failed to load "<<suffix<<" results"<<endl;
			clusterings.pop_back();
		}
		return ret;
	}

	void saveResults(const char * suffix="*"){
		if(strcmp(suffix,"*")==0)
		for(uint i=0;i<clusterings.size();++i)
			if(strcmp(suffix,"*")==0 || clusterings[i].suffix.compare(suffix)==0)
				clusterings[i].writeClusters(filename);
	}

	void testAlgorithm(string suffix, void (*clusterfunc)(pcl::PointCloud<PointT> &, std::vector< std::vector<int> > &,double)){
		timeval t0=g_tick();
		clusterings.push_back(clusterResults(suffix,cluster_tol));
		clusterfunc(smallcloud,clusterings.back().inds,cluster_tol);
	    clusterings.back().ptime=g_tock(t0);
	}


	void evaluateResults(){
		if(clusterings.size()<2) return;
		allmatched=true;
		for(uint c1=0;c1<clusterings.size()-1;++c1){
			int c2=c1+1;
			reports.push_back(MatchReport(clusterings[c1].suffix,clusterings[c2].suffix));
			if(!matchclusterings(clusterings[c1].inds,clusterings[c1].inds,reports.back())){
				allmatched=false;
			   std::vector<std::vector<int> > splits;
			   splits=reports.back().getSplits(1);
		      if(splits.size())
		         std::cout<<"checking cluster 1 splits: "<<std::endl;
			   for (uint i = 0; i < splits.size(); ++i)
			      //only check pairwise splits at the moment...
			      checkClusterMistake(splits[i][0],splits[i][1],clusterings[c1].inds,smallcloud,cluster_tol,reports.back().pt1,reports.back().pt2);

		      splits=reports.back().getSplits(2);
		      if(splits.size())
		         std::cout<<"checking cluster 2 splits: "<<std::endl;
		      for (uint i = 0; i < splits.size(); ++i)
		         //only check pairwise splits at the moment...
		         if(!checkClusterMistake(splits[i][0],splits[i][1],clusterings[c2].inds,smallcloud,cluster_tol,reports.back().pt1,reports.back().pt2)){
		        	 cout<<"check points "<<reports.back().pt1<<" "<<reports.back().pt2<<endl;
		        	 break;
		         }
			}
		}
	}

	void printResults(){
		if(clusterings.size()<2) return;
		int cloudsize=0;
		if(smallcloud.size())
		   cloudsize=smallcloud.size();
		else
		   for(uint i=0;i<clusterings[0].inds.size(); i++)
			cloudsize+=clusterings[0].inds[i].size();
		for(uint c1=0;c1<clusterings.size();++c1)
			std::cout<<clusterings[c1].suffix<<": "<<clusterings[c1].ptime<<"  ";
	    std::cout<<"  cluster tol: "<<cluster_tol<<"  clusters: "<<clusterings[0].inds.size()<<" cloud: "<<cloudsize<<"  "<<filename;

		if(allmatched)
			std::cout<<" matched "<<std::endl;
		else
		    std::cout<<" failed to match!!!"<<std::endl;
	}

};
#endif /* CLUSTEREVALUATION_HPP_ */
