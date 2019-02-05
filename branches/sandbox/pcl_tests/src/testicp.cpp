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



//#include "pcl/io/pcd_io.h"
//#include "pcl/registration/registration.h"
//#include "pcl/registration/icp.h"
//#include <sys/time.h>
//#include <list>
////
//#include <stdio.h>
//#include <stdlib.h>
//#include <time.h>
////#include "pcl/segmentation/extract_clusters.hpp"
////#include "ScanAnalyzer.h"
//#include "pcl_tf/transforms.h"

#include "pcl_tools/pcl_utils.h"

//  timeval g_tick(){
//     struct timeval tv;
//     gettimeofday(&tv, NULL);
//     return tv;
//  }
//
//  double g_tock(timeval tprev)
//  {
//     struct timeval tv;
//     gettimeofday(&tv, NULL);
//     return (double)(tv.tv_sec-tprev.tv_sec) + (tv.tv_usec-tprev.tv_usec)/1000000.0;
//  }
//
//  int getUsec(){
//     struct timeval tv;
//     gettimeofday(&tv, NULL);
//     return tv.tv_usec;
//  }
//
//  tf::Transform tfFromEigen(Eigen3::Matrix4f trans){
//   btMatrix3x3 btm;
//   btm.setValue(trans(0,0),trans(0,1),trans(0,2),
//              trans(1,0),trans(1,1),trans(1,2),
//              trans(2,0),trans(2,1),trans(2,2));
//   btTransform ret;
//   ret.setOrigin(btVector3(trans(0,3),trans(1,3),trans(2,3)));
//   ret.setBasis(btm);
//   return ret;
//  }
//
//  double getYaw(Eigen3::Matrix4f etrans){
//     tf::Transform tftrans=tfFromEigen(etrans);
//     return tf::getYaw(tftrans.getRotation());
//  }
//
//  //removes the yaw, pitch and z component from the transform
//  Eigen3::Matrix4f projectTo2D(Eigen3::Matrix4f etrans){
//     tf::Transform tftrans=tfFromEigen(etrans);
//     tftrans.setRotation(tf::createQuaternionFromYaw(tf::getYaw(tftrans.getRotation())));
//     Eigen3::Matrix4f out;
//     pcl::transformAsMatrix(tftrans,out);
//     out(2,3)=0.0;  //remove z component
//     return out;
//  }
//
////finds point in target cloud that are near points in ref_cloud
////this works by searching for nearest neighbors of a bunch of points in the ref cloud
////it is approximate, because we don't really check every point in ref_cloud
////template <typename PointT>
////int getCLosestPoints(pcl::PointCloud<PointT> &ref_cloud, pcl::PointCloud<PointT> &target_cloud, std::vector<int> &ref_pts, std::vector<int> &tgt_pts, double dist_thresh=.2, double cluster_tol=.2){
////   if(!ref_cloud.size() || !target_cloud.size() ){
////      ROS_ERROR("Not enough points in input clouds!");
////      return -1;
////   }
////
////   double smaller_tol = cluster_tol;
////   timeval t0=g_tick();
////   pcl::KdTreeFLANN<PointT> rtree,ttree;
////   rtree.setInputCloud(ref_cloud.makeShared());
////   ttree.setInputCloud(target_cloud.makeShared());
////   std::vector<int> minitree(ref_cloud.points.size(),-1), nearstatus(target_cloud.points.size(),0);
////   std::vector<int> indices,heads;
////   std::vector<float> dists;
////
////   //First 'downsample' the points into clusters slightly less than half the cluster tolerance
////   //minitree keeps track of who each point belongs to (which is an index of the 'head' array)
////   for(uint i=0; i<ref_cloud.points.size();i++){
////     if(minitree[i]==-1){
////       heads.push_back(i);
////        //find all the points in ref_cloud that are nearby
////        if(!rtree.radiusSearch(ref_cloud.points[i],smaller_tol,indices,dists)) { cout<<"radius search failed!"<<endl; return -1; }
////      for(uint j=0;j<indices.size();j++)
////         minitree[indices[j]]=0; //mark these points as 'searched'
////      minitree[i]=1;  //if we want to remember where we searched.
////     }
////   }
////
////
////
////   //now find the points in the target that are close to the ref:
////   indices.resize(1);
////   dists.resize(1);
////   ref_pts.clear();
////   tgt_pts.clear();
////   for(uint i=0; i<heads.size();i++){
////      if(ttree.nearestKSearch(ref_cloud.points[heads[i]],1,indices,dists) && dists[0] < dist_thresh){ //did find a nieghbor within the threshold
////         ref_pts.push_back(heads[i]);
////         tgt_pts.push_back(indices[0]);
////      }
//////      else{
//////         ROS_WARN("head %i closest point: %f",i,dists[0]);
//////      }
////   }
////   ROS_INFO("finding Closest Points took:  %f secs. found %d pts from %d guesses out of %d pts",g_tock(t0),tgt_pts.size(),heads.size(),target_cloud.points.size());
//////   std::cout<<"finding Closest Points took:  "<<g_tock(t0)<<"s  found "<<tgt_pts.size()<<" out of "<<<<target_cloud.points.size()<<" pts"<<std::endl;
////   return 0;
////}
//
//
////ref should map to target, but not necessarily the other way 'round.  also, target should not be downsampled
//template <typename PointT>
//int getCLosestPoints(pcl::PointCloud<PointT> &ref_cloud, pcl::PointCloud<PointT> &target_cloud, std::vector<int> &ref_pts, std::vector<int> &tgt_pts, double dist_thresh=.2, int num_pts=500){
//   timeval t0=g_tick();
//   pcl::KdTreeFLANN<PointT> ttree;
//   ttree.setInputCloud(target_cloud.makeShared());
////   ROS_INFO("step 1: initialization  %f secs. ",g_tock(t0));
//   t0=g_tick();
//   srand(getUsec());
//   std::vector<int> indices(1);
//   std::vector<float> dists(1);
//   ref_pts.clear();
//   tgt_pts.clear();
//   int ind;
//
//   std::vector<int> notused(ref_cloud.points.size(),1);
//   for(uint i=0;i<ref_cloud.points.size();i++)
//      notused[i]=i;
//   long tcount=0;
//   int indsleft=ref_cloud.points.size();
//   while(ref_pts.size() < num_pts && indsleft > 10){
//      ind=rand() %indsleft;
//      if(ttree.nearestKSearch(ref_cloud.points[notused[ind]],1,indices,dists) && dists[0] < dist_thresh)
//            if(fabs(ref_cloud.points[notused[ind]].z - target_cloud.points[indices[0]].z) < dist_thresh/10.0 ){
//         ref_pts.push_back(notused[ind]);
//         tgt_pts.push_back(indices[0]);
//      }
//      notused[ind]=notused[indsleft-1];
//      tcount++;
//      indsleft--;
//   }
////   ROS_INFO("finding Closest Points took:  %f secs. found %d pts from %d guesses out of %d pts",g_tock(t0),tgt_pts.size(),tcount,target_cloud.points.size());
////   ROS_INFO(" %f secs pre guess",g_tock(t0)/(double)tcount);
//   return 0;
//}
//
//
//
//template <typename PointT>
//Eigen3::Matrix4f gicp(pcl::PointCloud<PointT> &c1, pcl::PointCloud<PointT> &c2, double max_dist,
//      int small_transdiff_countreq=1, int num_pts=500, uint min_pts=10, uint max_iter=50, float transdiff_thresh=.0001 ){
//   timeval t0=g_tick(),t1=g_tick();
//   std::vector<int> c1pts,c2pts;
//   Eigen3::Matrix4f transformation_, final_transformation_=Eigen3::Matrix4f::Identity(),previous_transformation_,trans2d;
//   int small_transdiff_count=0;
//   for(uint i=0;i<max_iter;i++){
//      t0=g_tick();
//      getCLosestPoints(c1,c2,c1pts,c2pts,max_dist,num_pts);
//      if(c1pts.size() < min_pts){
//         ROS_ERROR("not enough correspondences");
//         return transformation_;
//      }
//      previous_transformation_ = final_transformation_;
//      pcl::estimateRigidTransformationSVD(c1,c1pts,c2,c2pts,transformation_);
//      // Tranform the data
//
//      transformPointCloud (c1, c1, projectTo2D(transformation_));
//      // Obtain the final transformation
//      final_transformation_ = transformation_ * final_transformation_;
//      trans2d=projectTo2D(final_transformation_);
//      final_transformation_=trans2d;
//      float transdiff=fabs ((final_transformation_ - previous_transformation_).sum ());
//      ROS_INFO("icp took:  %f secs. trans diff: %f,  total: %f, %f, %f",g_tock(t0),transdiff,trans2d(0,3),trans2d(1,3),getYaw(trans2d));
//      if(transdiff <transdiff_thresh) small_transdiff_count++;
//      else small_transdiff_count=0;
//      if(small_transdiff_count>small_transdiff_countreq)
//         break;
//   }
//   ROS_INFO("icp took:  %f secs. ",g_tock(t1));
//   std::cout<<final_transformation_<<std::endl;
//   return final_transformation_;
//}
//







int main(int argc, char **argv) {
   srand ( time(NULL) );
   if(argc<3){
      std::cout<<"USAGE "<<argv[0]<<" cloud1.pcd cloud2.pcd"<<std::endl;
      return -1;
   }

    pcl::PointCloud<pcl::PointXYZINormal> c1,c1b,c2;
    pcl::io::loadPCDFile(argv[1],c1b);
    pcl::io::loadPCDFile(argv[1],c1);
    pcl::io::loadPCDFile(argv[2],c2);
//    std::vector<int> c1pts,c2pts;
//    for(float f=.01;f<.3; f+=.01)
//    float f=.03;
//    getCLosestPoints(c1,c2,c1pts,c2pts,f,.2);
//    getCLosestPoints(c1,c2,c1pts,c2pts,f,.2);
//    getCLosestPoints(c1,c2,c1pts,c2pts,f,.2);

//    for(uint i=0;i<c1b.points.size();i++){
//       c1b.points[i].intensity=.1;
//    }
    c1b+=c2;
    pcl::io::savePCDFile("combinedcloud_before.pcd",c1b);

    icp2D(c1,c2,.05,2,500,10,50,.0002);
//    for(uint i=0;i<c1.points.size();i++){
//       c1.points[i].intensity=.1;
//    }
    c1+=c2;
    pcl::io::savePCDFile("combinedcloud.pcd",c1);

    return 0;
}
