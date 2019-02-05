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



#ifndef NNN_H_
#define NNN_H_

#include "pcl/filters/extract_indices.h"
#include "pcl/io/pcd_io.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/features/feature.h"
#include "pcl/common/common.h"
#include <sys/time.h>
#include <list>
#include <fstream>


//performs radius search in a simplistic fashion
template <typename PointT>
void NNN(pcl::PointCloud<PointT> &cloud, PointT pt, std::vector<int> &inds, double radius){
   inds.clear();
   double r2=radius*radius;
   double smallerrad=radius/sqrt(3);
   double diffx,diffy,diffz;
   for(uint i=0;i<cloud.points.size(); i++){
      diffx=fabs(cloud.points[i].x - pt.x);
      diffy=fabs(cloud.points[i].y - pt.y);
      diffz=fabs(cloud.points[i].z - pt.z);
      if(diffx < radius && diffy < radius && diffz < radius){ //in outer box
         if(diffx < smallerrad && diffy < smallerrad && diffz < smallerrad) //also in inner box - include!
            inds.push_back(i);
         else//between the boxes: check for actual distance
            if(diffx*diffx+diffy*diffy+diffz*diffz < r2)
               inds.push_back(i);
      }
   }
}


class SplitCloud{
   double max_tol; //maximum tolerance that can be used in any algorithm and still maintain correctness
   double xdiv,ydiv,zdiv;
   std::vector<pcl::PointCloud<pcl::PointXYZ> > clouds;
   std::vector< std::vector<int> > cinds;
   int getIndex(pcl::PointXYZ &pt, double &x, double &y, double &z);
   void initClouds(pcl::PointCloud<pcl::PointXYZ> &cloud);

public:
   SplitCloud(pcl::PointCloud<pcl::PointXYZ> &cloud, double tol);
   pcl::PointCloud<pcl::PointXYZ> & get(pcl::PointXYZ &pt);
   //performs radius search in a simplistic fashion
   void NNN(pcl::PointXYZ pt, std::vector<int> &inds, double radius);
};


class SplitCloud2{
   double max_tol; //maximum tolerance that can be used in any algorithm and still maintain correctness
   double xdiv,ydiv,zdiv;
   double xdivs[8],ydivs[8],zdivs[8];
   double lx[64],ux[64],ly[64],uy[64],lz[64],uz[64];
   std::vector<pcl::PointCloud<pcl::PointXYZ> > clouds;
   pcl::PointCloud<pcl::PointXYZ> *_cloud;
   std::vector< std::vector<int> > cinds;
   std::vector< std::vector<int> > minds;

   //for searching over a subset of the indices
   std::vector< std::vector<int> > subinds;
   std::vector<int>  subindmap;

   // returns the index of the first split (0-8)
   int getMIndex(pcl::PointXYZ &pt);
   //fill a list of indices that indicate which octant the point is in
   //also, fills out the boundaries of the subdivisions
   void fillMInds(pcl::PointCloud<pcl::PointXYZ> &cloud);
   int getIndex(pcl::PointXYZ &pt);

   bool isIndex(pcl::PointXYZ &pt, int index);
   void initClouds(pcl::PointCloud<pcl::PointXYZ> &cloud);

public:
   SplitCloud2(pcl::PointCloud<pcl::PointXYZ> &cloud, double tol);

   pcl::PointCloud<pcl::PointXYZ> & get(pcl::PointXYZ &pt);

   //search over a subset of of the cloud
   void useInds(std::vector<int>  &inds);

   //performs radius search in a simplistic fashion
   void NNN(pcl::PointXYZ pt, std::vector<int> &inds, double radius, bool usesubset=false);

   //For heavily optimized system:
   //inds is assumed to be a vector of the same size as the initial point cloud
   //mark is the number to assign to those indices
   int AssignInds(pcl::PointXYZ pt, std::vector<int> &inds, double radius, int mark);

   //for unit testing:
   //this function checks to see if the indexed point is in the right hexiquadrant
   bool checkIndex(int index){
	   //get the point from the main cloud:
	   int ind=getIndex(_cloud->points[index]);
	   return isIndex(_cloud->points[index],ind);
   }

};







#endif /* NNN_H_ */
