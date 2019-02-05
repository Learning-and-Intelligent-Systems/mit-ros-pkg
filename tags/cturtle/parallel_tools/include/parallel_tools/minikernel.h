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



#ifndef MINIKERNEL_H_
#define MINIKERNEL_H_


#include <iostream>
#include <vector>
//#include <fstream>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>


  boost::mutex data_mutex,print_mutex;

//  struct jobtype{
//     int jobnum;
//     pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
//  };


//  //IndexableJob is an abstract class that fills a jobstruct based on an index
//  template <typename type1, typename type2>
//  class IndexableJob{
//
//  public:
//     virtual int getTotalJobs()=0;
//     virtual void getJobStruct(int i, jobstruct &j)=0;
//  };


  template <typename type1, typename type2>
  struct jobtype{
     type1 t1;
     type2 t2;
  };


  //ArrayJob iterates through all the combinations of it's vectors
  template <typename type1, typename type2, typename jobstruct = jobtype<type1,type2> >
  class ArrayJob{
  protected:
     std::vector<type1> v1;
     std::vector<type2> v2;



  public:

     ArrayJob(std::vector<type1> _v1, std::vector<type2> _v2){
        v1=_v1; v2=_v2;
     }
     virtual int getTotalJobs(){
        return v1.size()*v2.size();
     }
     virtual void getJobStruct(int i, jobstruct &j){
        j.t1=v1[i%v1.size()];
        j.t2=v2[i/v1.size()];
     }
  };

  template <typename type1, typename type2, typename jobstruct = jobtype<type1,type2> >
  class PairedJob: public ArrayJob<type1,type2>{
  public:
     PairedJob(std::vector<type1> _v1, std::vector<type2> _v2):ArrayJob<type1,type2>(_v1,_v2){
//        this->v1=_v1; this->v2=_v2;
     }
     virtual int getTotalJobs(){
        return std::min(this->v1.size(),this->v2.size());
     }
     virtual void getJobStruct(int i, jobstruct &j){
        j.t1=this->v1[i];
        j.t2=this->v2[i];
     }
  };




//
//  //IndexableJob is a collection of vectors that
//  class IndexableJob{
//     vector< pcl::PointCloud<pcl::PointXYZ> > clouds;
//     vector<double> tolerances;
//
//  public:
//     IndexableJob(vector< pcl::PointCloud<pcl::PointXYZ> > &c){
//        clouds=c;
//     }
//     int getTotalJobs(){
//        return clouds.size()-1;
//     }
//
//     void getJobStruct(int i, jobtype &j){
//       j.jobnum=i;
//        j.cloud1=clouds[i];
//        j.cloud2=clouds[i+1];
//     }
//
//  };
//

  template <typename type1, typename type2, typename jobstruct = jobtype<type1,type2> >
  class JobKernel{
     boost::mutex job_mutex;
     ArrayJob<type1,type2> *job;
     int nextjob;
     int totaljobs;
  public:


     JobKernel(ArrayJob<type1,type2> *_job){
        job=_job;
        totaljobs=job->getTotalJobs();
        nextjob=0;
     }

     int getNextJob(jobstruct &j){
        boost::mutex::scoped_lock  lock(job_mutex);
        if(nextjob<totaljobs)
           job->getJobStruct(nextjob++,j);
        else
           return 1;
        return 0;
     }


       void jobthread(void (*jobfunc)(type1,type2)){
          jobstruct j;
          while(getNextJob(j)==0){
             jobfunc(j.t1,j.t2);
          }
       }



  };

namespace minikernel{
 enum IterType {PAIRED,ARRAY};

}

template <typename type1, typename type2>
 void LaunchThreads(std::vector<type1 > &v1,std::vector<type2 > &v2, int numthreads,void (*jobfunc)(type1,type2),minikernel::IterType it){
    ArrayJob<type1,type2> *ij;
    if(it==minikernel::PAIRED)
       ij = new PairedJob<type1,type2>(v1,v2);
    if(it==minikernel::ARRAY)
       ij = new ArrayJob<type1,type2>(v1,v2);

    JobKernel<type1,type2> kernel(ij);
    boost::thread_group threads;
    for(int i=0;i<numthreads;i++){
       threads.create_thread(boost::bind(&JobKernel<type1,type2>::jobthread,&kernel,jobfunc));
    }
    threads.join_all();
 }

//template <typename type1>
// void LaunchThreads(std::vector<type1 > &v1,int numthreads,void (*jobfunc)(type1,int)){
//   std::vector<int> v2(v1.size());
//
//    PairedJob<type1,int> ij(v1,v2);
//
//    JobKernel<type1,int> kernel(&ij);
//    boost::thread_group threads;
//    for(int i=0;i<numthreads;i++){
//       threads.create_thread(boost::bind(&JobKernel<type1,int>::jobthread,&kernel,jobfunc));
//    }
//    threads.join_all();
// }

//
//
//  void alignclouds(pcl::PointCloud<pcl::PointXYZ> c1, pcl::PointCloud<pcl::PointXYZ> c2, Eigen3::Matrix4f &alignment){
//      Eigen3::Matrix4f trans1 = icp2D(c1,c2,.05,2,500,10,50,.0002);
//      {
//            boost::mutex::scoped_lock  lock(data_mutex);
//            alignment=trans1;
//      }
//  }
//
//
//
//  void jobthread(JobKernel *kernel,std::vector<Eigen3::Matrix4f> &alignments){
//     jobtype j;
//     while(kernel->getNextJob(j)==0){
//        alignclouds(j.cloud1,j.cloud2,alignments[j.jobnum]);
//     }
//  }
//
//
//  void LaunchThreads(std::vector<pcl::PointCloud<pcl::PointXYZ> > &clouds, int numthreads,std::vector<Eigen3::Matrix4f> &alignments){
//     IndexableJob ij(clouds);
//     JobKernel kernel(&ij);
//     boost::thread_group threads;
//     for(int i=0;i<numthreads;i++){
////        {
////            boost::mutex::scoped_lock  lock(print_mutex);
////  //          ROS_INFO("Creating thread %d",i);
////        }
//        threads.create_thread(boost::bind(&jobthread,&kernel,alignments));
//     }
//     threads.join_all();
//  }
//
//
//
//
//




















#endif /* MINIKERNEL_H_ */
