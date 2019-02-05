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


template <typename PointT>
void testFLANN(pcl::PointCloud<PointT> &cloud, double cluster_tol, std::vector<int> searchinds){
   timeval t0=g_tick();
   pcl::KdTreeFLANN<PointT> ftree;
   ftree.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> > (cloud));
   double setupflanntime= g_tock(t0);

   std::vector<int> flannindices;
   std::vector<float> dists;
   t0=g_tick();
   for(uint trialnum=0; trialnum<searchinds.size();trialnum++){
       ftree.radiusSearch(cloud.points[searchinds[trialnum]],cluster_tol,flannindices,dists);
   }
   ROS_INFO("Flann had setup time of %f and search time of %f",setupflanntime,g_tock(t0));

}

//template <typename PointT>
//void testANN(pcl::PointCloud<PointT> &cloud, double cluster_tol, std::vector<int> searchinds){
//   timeval t0=g_tick();
//   pcl::KdTreeANN<PointT> tree;
//   tree.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> > (cloud));
//   double setupanntime= g_tock(t0);
//
//   std::vector<int> annindices;
//   std::vector<float> dists;
//   t0=g_tick();
//   for(uint trialnum=0; trialnum<searchinds.size();trialnum++){
//       tree.radiusSearch(cloud.points[searchinds[trialnum]],cluster_tol,annindices,dists);
//   }
//   ROS_INFO("ANN had setup time of %f and search time of %f",setupanntime,g_tock(t0));
//}


template <typename PointT>
void testNNN(pcl::PointCloud<PointT> &cloud, double cluster_tol, std::vector<int> searchinds){
   timeval t0=g_tick();
   SplitCloud2<PointT> sc2(cloud,cluster_tol);
   double setupnnntime= g_tock(t0);

   std::vector<int> nnnindices;
   t0=g_tick();
   for(uint trialnum=0; trialnum<searchinds.size();trialnum++){
      sc2.NNN(cloud.points[searchinds[trialnum]],nnnindices,cluster_tol);
   }
   ROS_INFO("NNN had setup time of %f and search time of %f",setupnnntime,g_tock(t0));
}








int main(int argc, char **argv) {
   if(argc<3){
      std::cout<<"Usage: filename.pcd <tolerance>"<<std::endl;
      return -1;
   }

   pcl::PointCloud<pcl::PointXYZ> cloud;
   pcl::io::loadPCDFile(argv[1],cloud);

   std::cout<<"Loaded PCD file:  "<<std::endl;

    int trials=1000;
    if(argc==4)
       trials=atoi(argv[3]);

    //generate test points
    std::vector<int> testpts;
    srand(getUsec());
    for(int i=0; i<trials;i++)
      testpts.push_back(rand()%cloud.points.size());

    testFLANN(cloud,atof(argv[2]),testpts);
//    testANN(cloud,atof(argv[2]),testpts);
    testNNN(cloud,atof(argv[2]),testpts);


    return 0;
}


