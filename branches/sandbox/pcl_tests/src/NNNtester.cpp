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



#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include "pcl/filters/extract_indices.h"
#include "pcl/io/pcd_io.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/features/feature.h"
#include <sys/time.h>
#include <sys/resource.h>
#include <list>
#include <fstream>

#include "nnn/nnn.hpp"


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

int getUsec(){
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_usec;
}

long int getMem(){
	int who = RUSAGE_SELF;
	struct rusage usage;
	int ret;
	ret = getrusage(who, &usage);
	return usage.ru_maxrss;
//	cout<<"max set size: "<<usage.ru_maxrss<<"  data mem: "<<usage.ru_idrss<<"  stack mem: "<<usage.ru_isrss<<endl;
}



boost::mutex data_mutex,print_mutex;


struct jobtype{
   pcl::PointCloud<pcl::PointXYZ> cloud;
   double tol;
};

//IndexableJob is a collection of vectors that
class IndexableJob{
	std::vector< pcl::PointCloud<pcl::PointXYZ> > clouds;
	std::vector<double> tolerances;

public:
   IndexableJob(std::vector< pcl::PointCloud<pcl::PointXYZ> > &c, std::vector<double> &t){
      clouds=c;
      tolerances=t;
   }
   int getTotalJobs(){
      return clouds.size()*tolerances.size();
   }

   void getJobStruct(int i, jobtype &j){
      j.cloud=clouds[i/tolerances.size()];
      j.tol=tolerances[i%tolerances.size()];
   }

};

class JobKernel{
   boost::mutex job_mutex;
   IndexableJob *job;
   int nextjob;
   int totaljobs;
public:
   JobKernel(IndexableJob *_job){
      job=_job;
      totaljobs=job->getTotalJobs();
      nextjob=0;
   }

   int getNextJob(jobtype &j){
      boost::mutex::scoped_lock  lock(job_mutex);
      if(nextjob<totaljobs)
         job->getJobStruct(nextjob++,j);
      else
         return 1;
      return 0;
   }
};


template <typename PointT>
bool checkcluster( pcl::PointCloud<PointT> &cloud,std::vector<int>  &inds, double tol,int pt, std::string s){
      for(uint i=0;i<inds.size();i++)
         if(pcl::euclideanDistance(cloud.points[pt],cloud.points[inds[i]]) > tol){
//            boost::mutex::scoped_lock  lock(print_mutex);  //already locked one level above
            ROS_ERROR("Point in %s's clustering is %f from point %d , should be less than %f ",s.c_str(),pcl::euclideanDistance(cloud.points[pt],cloud.points[inds[i]]),pt,tol);
            return false;
         }
      return true;
}

int clustercomp(std::vector<int>  &inds1, std::vector<int>  &inds2){
      if(inds1.size() != inds2.size())
         return -1;
      for(uint i=0;i<inds1.size();i++){
         bool found=false;
         for(uint j=0;j<inds2.size();j++)
            if(inds2[j]==inds1[i]){
               found=true;
               break;
            }
         if(!found)
            return i+1;
      }
      return 0;
}

template <typename PointT>
bool testclusters(pcl::PointCloud<PointT> &cloud,std::vector<int>  &inds1, std::vector<int>  &inds2, double tol,int pt, std::string s1, std::string s2 ){
//   if(inds1.size() != inds2.size()){
//      boost::mutex::scoped_lock  lock(print_mutex);
////      ROS_ERROR("trial: %f   index sizes %d (%s) and %d (%s) do not match!",tol,(int)inds1.size(),s1.c_str(),(int)inds2.size(),s2.c_str());
//   }
   if(inds1.size() > inds2.size()){
      boost::mutex::scoped_lock  lock(print_mutex);
      if(checkcluster(cloud,inds1,tol,pt,s1))
         ROS_ERROR("trial: %f   %s size %d > %d (%s), but %s has all valid points ",tol,s1.c_str(), (int)inds1.size(),(int)inds2.size(),s2.c_str(), s1.c_str());
      return false;
   }

   if(inds2.size() > inds1.size()){
      boost::mutex::scoped_lock  lock(print_mutex);
      if(checkcluster(cloud,inds2,tol,pt,s2))
         ROS_ERROR("trial: %f   %s size %d > %d (%s), but %s has all valid points ",tol,s2.c_str(), (int)inds2.size(),(int)inds1.size(),s1.c_str(), s2.c_str());
      return false;
   }

   if(clustercomp(inds1,inds2)){
      ROS_ERROR("trial: %f  the number of indices given by %s and %s are the same, but the values are different",tol,s2.c_str(),s1.c_str());
        return false;

   }

   return true;
}





template <typename PointT>
void testNNs(pcl::PointCloud<PointT> &_cloud, double cluster_tol, int iterations=1000){
   pcl::PointCloud<PointT> cloud;
   {
      boost::mutex::scoped_lock  lock(data_mutex);
      cloud=_cloud;
   }

   timeval t0=g_tick();
   pcl::KdTreeFLANN<PointT> tree;
   tree.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> > (cloud));
   double setupanntime= g_tock(t0); t0=g_tick();

   pcl::KdTreeFLANN<PointT> ftree;
   ftree.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> > (cloud));
   double setupflanntime= g_tock(t0); t0=g_tick();

   SplitCloud<PointT> sc(cloud,cluster_tol);
   double setupsctime= g_tock(t0); t0=g_tick();

   SplitCloud2<PointT> sc2(cloud,cluster_tol);
   double setupsc2time= g_tock(t0); t0=g_tick();

   std::vector<int> annindices,flannindices, naiveindices, scindices, sc2indices;
   std::vector<float> dists;
   double anntime=0,flanntime=0,naivetime=0, sctime=0, sc2dtime=0, sc2time=0;

   for(int trialnum=0; trialnum<iterations;trialnum++){
      srand(getUsec()); //ANN of FLANN resets seed at every search...
     int ind = rand()%cloud.points.size();
     annindices.clear(); flannindices.clear(); naiveindices.clear(); scindices.clear(); sc2indices.clear();
//     t0=g_tick(); tree.radiusSearch(cloud.points[ind],cluster_tol,annindices,dists);    anntime  +=g_tock(t0);
//     t0=g_tick(); ftree.radiusSearch(cloud.points[ind],cluster_tol,flannindices,dists); flanntime+=g_tock(t0);
//     t0=g_tick(); NNN(cloud,cloud.points[ind],naiveindices,cluster_tol); naivetime+=g_tock(t0);
     t0=g_tick(); sc.NNN(cloud.points[ind],scindices,cluster_tol); sctime+=g_tock(t0);
     t0=g_tick(); sc2.NNN(cloud.points[ind],sc2indices,cluster_tol); sc2time+=g_tock(t0);sc2indices.clear();
     t0=g_tick(); sc2.NNN(cloud.points[ind],sc2indices,dists,cluster_tol); sc2dtime+=g_tock(t0);
//     if(!testclusters(cloud,annindices,scindices,cluster_tol,ind, "ann", "splitcloud")) break;
//     if(!testclusters(cloud,annindices,sc2indices,cluster_tol,ind, "ann", "splitcloud2")) break;
   }
   float div=(float)iterations;
   {
      boost::mutex::scoped_lock  lock(print_mutex);
      printf("%d pts, %.2f tol, setup: A: %.3f, F: %.3f, SC: %.3f , SC2: %.3f ",(int)cloud.points.size(),cluster_tol, setupanntime,setupflanntime,setupsctime,setupsc2time);
      printf("| A: %.6f  F: %.6f  n: %.6f  sc: %.6f  sc2: %.6f  sc2d: %.6f\n", anntime/div,flanntime/div,naivetime/div,sctime/div,sc2time/div,sc2dtime/div);
   }
}


void jobthread(JobKernel *kernel,int trials){
   jobtype j;
   while(kernel->getNextJob(j)==0){
//      {
//          boost::mutex::scoped_lock  lock(print_mutex);
//          ROS_INFO("thread %d computing tolerance %f",threadnum,j.tol);
//      }
      testNNs(j.cloud,j.tol,trials);
   }
}


void LaunchThreads(std::vector<pcl::PointCloud<pcl::PointXYZ> > &clouds, std::vector<double> &tolerances, int numthreads, int trials){
   IndexableJob ij(clouds,tolerances);
   JobKernel kernel(&ij);
   boost::thread_group threads;
   for(int i=0;i<numthreads;i++){
      {
          boost::mutex::scoped_lock  lock(print_mutex);
//          ROS_INFO("Creating thread %d",i);
      }
      threads.create_thread(boost::bind(&jobthread,&kernel,trials));
   }
   threads.join_all();
}

bool randomDownsample(pcl::PointCloud<pcl::PointXYZ> &cloud, int numpts, pcl::PointCloud<pcl::PointXYZ> &cloudout){
   cloudout=cloud;
   if(numpts>(int)cloud.points.size()){
      ROS_WARN("requested number of points, %d is greater than the number of points in the cloud, %d",numpts,(int)cloud.points.size());
      return false;
   }
   //subtraction method:
   srand(getUsec());
//
//   for(int i=cloudout.points.size();i>numpts;i--){
//      cloudout.points.erase(cloudout.points.begin()+rand()%cloudout.points.size());
//   }
   std::vector<int> inds(cloud.points.size()), selected;
   for(uint i=0; i<cloud.points.size();i++)
      inds[i]=i;
   srand(getUsec());
   while((int)(inds.size()-selected.size())>numpts && (int)selected.size() < numpts){
     int ind = rand()%inds.size();
     if(inds[ind] != -1){
        selected.push_back(ind);
        inds[ind] = -1;
     }
   }
   if((int)selected.size()!=numpts){
      selected.clear();
      for(uint i=0; i<inds.size();i++){
         if(inds[i] != -1)
             selected.push_back(inds[i]);
      }
   }

   pcl::ExtractIndices<pcl::PointXYZ> extract;
   extract.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cloud));
   extract.setIndices (boost::make_shared<std::vector<int> > (selected));
   extract.filter (cloudout);
   return true;

}


void makeClouds(pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<pcl::PointCloud<pcl::PointXYZ> > &clouds){
   int ksizes[] = {1,5,10,20,30,40,50,60,70,80,90,100,120,140,160,180,200,240,260,280,300,350,400,500,600,700,800,900,1000,2000,3000,4000};
   for(int i=0;i<32;i++){
      clouds.push_back(pcl::PointCloud<pcl::PointXYZ>());
      if(!randomDownsample(cloud,ksizes[i]*1000,clouds[i])){
         break;
      }

      std::cout<<"cloud "<<i<<":  "<<getMem()<<std::endl;
   }
}




int main(int argc, char **argv) {
   if(argc<3){
      std::cout<<"Usage: filename.pcd <tolerance | 'all'> [number_of_trials]"<<std::endl;
      std::cout<<"    Using 'all' instead of tolerance will test a number of different tolerances"<<std::endl;
      return -1;
   }

   pcl::PointCloud<pcl::PointXYZ> cloud;
   pcl::io::loadPCDFile(argv[1],cloud);

   std::cout<<"Loaded PCD file:  "<<getMem()<<std::endl;


//   ROS_INFO("original cloud size: %d",cloud.size());

    int trials=1000;
    if(argc==4)
       trials=atoi(argv[3]);


    std::vector<pcl::PointCloud<pcl::PointXYZ> > clouds;
    clouds.push_back(cloud);
//    makeClouds(cloud,clouds);

//    cout<<"made clouds:  "<<getMem()<<endl;



    double ts[]={.01,.02,.03,.04,.05,.06,.07,.08,.09,.1,.2,.3,.4,.5,.6,.7,.8,.9,1.0};
    std::vector<double> tols;
    tols.assign(ts,ts+19);


    if(strcmp(argv[2],"all")==0)
       LaunchThreads(clouds,tols,4,trials);
    else
       testNNs(cloud,atof(argv[2]),trials);
    return 0;
}


