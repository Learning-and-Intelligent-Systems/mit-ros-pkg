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




#include <iomanip>
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

#include "segfast.h"


void printAll( string s1, string s2, vector<int> &data){
    int maxsize=0;
    for(uint i=0;i<data.size();i++)
    	if(data[i]>maxsize) maxsize=data[i];
    std::vector<int> ocounts(maxsize+1,0);
    for(uint i=0;i<data.size();i++)
    	ocounts[data[i]]++;
    for(uint i=0;i<ocounts.size();i++)
    	if(ocounts[i])
    		std::cout<<setw(6)<<ocounts[i]<<" "<<s1<<i<<" "<<s2<<std::endl;
}


void miniHist(int indmax, vector<int> bounds, string s1, string s2, vector<int> &data){
	std::vector<int> counts(indmax+bounds.size()+2,0);
	bounds.push_back(INT_MAX); //this makes it easier for assigning...
    for(uint i=0;i<data.size();i++)
    	if(data[i]<=indmax)
    		counts[data[i]]++;
    	else{
    		for(uint j=0;j<bounds.size();j++) //data[i] is guaranteed to be w/in bounds
    			if(data[i]<=bounds[j]){
    				counts[indmax+j+1]++;
    				break;
    			}
    	}
    //now print out the summary:
    //for the individual counts:
    for(uint i=0;i<=indmax;i++)
    	if(counts[i])
    		std::cout<<setw(6)<<counts[i]<<" "<<s1<<setw(4)<<i<<" "<<s2<<std::endl;
    //between indmax and the first bounds
	if(counts[indmax+1])
		std::cout<<setw(6)<<counts[indmax+1]<<" "<<s1<<setw(4)<<" between "<<setw(4)<<indmax+1<<" and "<<setw(4)<<bounds[0]<<" "<<s2<<std::endl;
	//between the given bounds
	for(uint i=0;i<bounds.size()-2;i++)
		if(counts[indmax+2+i])
			std::cout<<setw(6)<<counts[indmax+2+i]<<" "<<s1<<setw(4)<<" between "<<setw(4)<<bounds[i]+1<<" and "<<setw(4)<<bounds[i+1]<<" "<<s2<<std::endl;
	//greater than the last bound
	bounds.pop_back();
	if(counts.back())
		std::cout<<setw(6)<<counts.back()<<" "<<s1<<setw(4)<<" more than "<<setw(4)<<bounds.back()+1<<" "<<s2<<std::endl;

}

//int analyzePairings(std::vector< std::vector<int> > &pairings, std::vector<int> &clustering){
//	clustering.resize(pairings.size(),-1);
//	int clusters=0;
//	std::vector<int> tosee,seen;
//	int current;
////	for(int i=300; i >=0; i--){
//		for(int i=pairings.size()-1; i >=0; i--){
//		if(clustering[i]!=-1) continue;
//		if(pairings[i].size()==0){//it saw no marked pts
//			clustering[i]=clusters++;
//			continue;
//		}
//		seen.clear();
//		seen.push_back(i);
//		tosee=pairings[i];
//		while(tosee.size()){
//			current=tosee.back();
//			tosee.pop_back();
//			if(!count(seen.begin(),seen.end(),current)){
//				seen.push_back(current);
//				//see if we just discovered a previously explored tree:
//				if(clustering[current] != -1){ //yes, we did!
//					//this tree is fully explored, so send all the nodes to the seen
//					for(uint j=0;j<clustering.size();j++)
//						if(clustering[j]==clustering[current])
//							seen.push_back(j);
//				}
//				else
//					if(pairings[current].size())
//						tosee.insert(tosee.end(),pairings[current].begin(),pairings[current].end());
//
//			}
//		}
//		//now have a fully explored tree in seen
////		cout<<"tree size "<<seen.size()<<endl;
//		for(uint j=0;j<seen.size();j++)
//			clustering[seen[j]]=clusters;
//		clusters++;
//
//
//
//	}
//	return clusters;
////	printAll("points in cluster ",".",clustering);
////
//
//}

//
//int main(int argc, char **argv) {
//	if(argc<3){
//		std::cout<<"Usage: filename.pcd <tolerance>"<<std::endl;
//		return -1;
//	}
//
//	pcl::PointCloud<pcl::PointXYZ> cloud;
//    pcl::io::loadPCDFile(argv[1],cloud);
//    double cluster_tol=atof(argv[2]);
//
////    PtMap pmap4(cloud,cluster_tol);
////
////
//////    pmap.testNNs(cloud,atof(argv[2]));
//////    return 0;
////
////    timeval t0;
//////    t0=g_tick();
//////    pmap.simpleDownsample(cloud,cluster_tol/2.01);
//////    cout<<"downsample took: "<<g_tock(t0)<<std::endl;
//////
//////     t0=g_tick();
//////    pmap2.simpleDownsample2(cloud,cluster_tol/2.01);
//////    cout<<"downsample2 took: "<<g_tock(t0)<<std::endl;
////
//////    t0=g_tick();
//////   pmap3.simpleDownsampleNNN(cloud,cluster_tol/2.01);
//////   cout<<"downsampleNNN took: "<<g_tock(t0)<<std::endl;
////    timeval ttot=g_tick();
////   t0=g_tick();
////  pmap4.simpleDownsampleNNN2(cloud,cluster_tol);
////  cout<<"downsampleNNN2 took: "<<g_tock(t0)<<std::endl;
////
////
////  cout<<std::endl<<"cluster heads: "<<std::endl;
//////  t0=g_tick();
//////  pmap.headCluster();
//////  cout<<"normal cluster heads took: "<<g_tock(t0)<<std::endl;
////
//////  t0=g_tick();
//////  pmap4.headClusterNNN();
//////  cout<<"NNN cluster heads took: "<<g_tock(t0)<<std::endl;
////
////  int tbounds[]={50,100,500,1000,10000};
////  vector<int> bounds;
////  bounds.assign(tbounds,tbounds+6);
////
////
////  vector<int> psize(pmap4.pairings.size());
////  for(uint i=0;i<pmap4.pairings.size();i++)
////  	psize[i]=pmap4.pairings[i].size();
////
//////  pmap4.pairings.resize(300);
//////  t0=g_tick();
//////  vector<int> clustering;
//////  int clusternum=analyzePairings(pmap4.pairings,clustering);
//////  cout<<"analyzepairings took: "<<g_tock(t0)<<"  for "<<clusternum<<std::endl;
////
////  t0=g_tick();
////  int clusternum=pmap4.analyzePairings();
////  cout<<"ptMap.analyzepairings took: "<<g_tock(t0)<<"  for "<<clusternum<<std::endl;
////
////
////  t0=g_tick();
////  pmap4.swapOutLoners();
////  cout<<"swapOutLoners took: "<<g_tock(t0)<<std::endl;
////
////    t0=g_tick();
////    pmap4._sc2->useInds(pmap4.heads);
////    cout<<"useinds took: "<<g_tock(t0)<<std::endl;
////
//////    t0=g_tick();
//////    pmap4.findMissedCandidates();
//////    cout<<"find candidates took: "<<g_tock(t0)<<std::endl;
//////
//////    vector<int> tosearchsize(pmap4.tosearch.size());
//////    for(uint i=0;i<pmap4.tosearch.size();i++)
//////       tosearchsize[i]=pmap4.tosearch[i].size();
//////    miniHist(10,bounds,"heads have ","heads to search",tosearchsize);
////
////
////    t0=g_tick();
////    pmap4.checkClustering3();
////    cout<<"check cluster took: "<<g_tock(t0)<<std::endl;
////
////    cout<<"full cluster time: "<<g_tock(ttot)<<std::endl;
////    cout<<"cloud size: "<<cloud.points.size()<<" heads: "<<pmap4.heads.size()<<" clusters: "<<pmap4.clusters.size() <<" ("<<pmap4.clusters.size()+pmap4.loners.size()<<")"<<std::endl;
//
////  printAll( "heads paired to ","other heads",psize);
//
////   miniHist(10,bounds,"heads initially grabbed ","points",pmap4.initialgrabs);
////  return 0;
//
//
//    //---------------Count overlaps----------------------------
////    printAll("points have ","overlaps",pmap.overlapcount);
//
//    //---------------Count cluster size----------------------------
////    std::vector<int> headsize(pmap.heads.size(),0);
////    for(uint i=0;i<cloud.points.size();i++)
////    	headsize[pmap.clusterindices[i]]++;
//
////    miniHist(10,bounds,"heads have","points",headsize);
//
//    //---------------Count cluster initial size----------------------------
////    miniHist(10,bounds,"heads initially grabbed ","points",pmap.initialgrabs);
//
//    //---------------count pairings----------------------------
////    vector<int> psize(pmap.pairings.size());
////    for(uint i=0;i<pmap.pairings.size();i++)
////    	psize[i]=pmap.pairings[i].size();
////
////    printAll( "heads paired to ","other heads",psize);
//
////    vector<int> clustering;
////    int clusternum=analyzePairings(pmap.pairings,clustering);
////
////    std::vector<int> ccount(clusternum,0);
////	for(uint j=0;j<clustering.size();j++)
////		ccount[clustering[j]]++;
////
////	int ind=0;
////	map<int,int> cmap;
////	for(int j=0;j<clusternum;j++)
////		if(ccount[j])
////			cmap[j]=ind++;
////
////
////
////
////	vector< vector<int> > clusters(ind);
////	for(uint j=0;j<clustering.size();j++)
////		clusters[cmap[clustering[j]]].push_back(j);
//
//
////	int lonercount=0;
////	for(uint j=0;j<clusters.size();j++)
////		if(clusters[j].size()==1 && pmap.initialgrabs[clusters[j][0]]==1)
////			lonercount++;
////
////	cout<<lonercount<<" loners."<<endl;
////	int lastloner=0,ll2=0;
////	for(uint i=0;i<pmap.initialgrabs.size();i++){
////		if(pmap.initialgrabs[i]==1 && clusters[cmap[clustering[i]]].size() != 1){
////			cout<<"non loner: "<<i<<" head: "<<pmap.clusterindices[i]<<" size: "<<clusters[cmap[clustering[i]]].size()<<std::endl;
////			lastloner=i;
////		}
////	}
////	for(uint i=0;i<pmap.pairings.size();i++){
////		for(uint j=0;j<pmap.pairings[i].size();j++)
////			if(pmap.pairings[i][j]==lastloner){
////				cout<<i<<" head: "<<pmap.clusterindices[i]<<" claimed "<<lastloner<<" as a pairing"<<endl;
////				ll2=i;
////			}
////	}
////	cout<<ll2<<": initgrab="<<pmap.initialgrabs[ll2]<<"  "<<cloud.points[ll2].x<<", "<<cloud.points[ll2].y<<", "<<cloud.points[ll2].z<<", "<<endl;
////	cout<<lastloner<<": initgrab="<<pmap.initialgrabs[lastloner]<<"  "<<cloud.points[lastloner].x<<", "<<cloud.points[lastloner].y<<", "<<cloud.points[lastloner].z<<", "<<endl;
//
//
////	miniHist(10,bounds,"clusters have ","heads",ccount);
////	printAll("clusters have ","heads",ccount);
//  //  int totalsize=0;
//        vector<int> csize(pmap4.clusters.size());
//        for(uint i=0;i<pmap4.clusters.size();i++){
//        	csize[i]=pmap4.clusters[i].size();
//  //      	totalsize+=csize[i];
//        }
//  //      cout<<totalsize<<endl;
//	miniHist(10,bounds,"clusters have ","heads",csize);
////	printAll("clusters have ","heads",csize);
//
////    cout<<"cloud size: "<<cloud.points.size()<<" heads: "<<pmap.heads.size()<<std::endl;
//
//
//}


//currently testing with:
//testlogs/testcloud05_1.0.pcd .07


