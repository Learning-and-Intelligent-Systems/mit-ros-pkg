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
#include <Eigen/StdVector>
#include <sys/time.h>
#include <list>
#include <fstream>


//performs radius search in a simplistic fashion
template <typename PointT>
void NNN(const pcl::PointCloud<PointT> &cloud, PointT pt, std::vector<int> &inds, double radius){
   inds.clear();
   double r2=radius*radius;
   double smallerrad=radius/sqrt(3);
   double diffx,diffy,diffz;
   for(uint i=0;i<cloud.points.size(); i++){
      //find the distance between the cloud point and the reference in the three dimensions:
      diffx=fabs(cloud.points[i].x - pt.x);
      diffy=fabs(cloud.points[i].y - pt.y);
      diffz=fabs(cloud.points[i].z - pt.z);
      //first find whether the point is in an axis aligned cube around the point -   this is a very quick check
      if(diffx < radius && diffy < radius && diffz < radius){ //in outer box
         //if the point is also in a cube whose circumradius is the search radius, the point is close enough
         if(diffx < smallerrad && diffy < smallerrad && diffz < smallerrad) //also in inner box - include!
            inds.push_back(i);
         //if the point actually falls in between the two cubes, we do the more computationally intensive multiply...
         else//between the boxes: check for actual distance
            if(diffx*diffx+diffy*diffy+diffz*diffz < r2)
               inds.push_back(i);
      }//endif in outer box
   }//endfor all points in cloud
}

//performs radius search in a simplistic fashion
//this version saves the squares of the distances
template <typename PointT>
void NNN(const pcl::PointCloud<PointT> &cloud, PointT pt, std::vector<int> &inds, std::vector<float> &dists, double radius){
   inds.clear();
   double r2=radius*radius;
   double diffx,diffy,diffz;
   double sqrdist;
   for(uint i=0;i<cloud.points.size(); i++){
      //find the distance between the cloud point and the reference in the three dimensions:
     diffx=fabs(cloud.points[i].x - pt.x);
     diffy=fabs(cloud.points[i].y - pt.y);
     diffz=fabs(cloud.points[i].z - pt.z);
     //first find whether the point is in an axis aligned cube around the point -   this is a very quick check
     if(diffx < radius && diffy < radius && diffz < radius){ //in outer box
       //calculate actual distance to the point
       sqrdist=diffx*diffx+diffy*diffy+diffz*diffz;
       if(sqrdist < r2){
         inds.push_back(i);
         dists.push_back(sqrdist);
       }
     }//endif in outer box
   }//endfor all points in cloud
 }

//SplitCloud and Splitcloud2:
//Both of these classes define a form of KDtree, where instead of creating distinct sets of points, the
//branches contain all the points in regions which overlap by a fixed distance.
//By having overlapping regions, we guarantee that searches in the tree will not have to search more than one node,
//greatly increasing efficiency.  The fixed distance, stored in a variable max_tol, defines the maximum radius search
//that can be searched and still guarantee a correct result.

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b SplitCloud uses one (three dimensional) layer of branching, creating 8 overlapping octants.
 * \author Garratt Gallagher
 */
template <typename PointT>
class SplitCloud{

   double max_tol; //maximum tolerance that can be used in any algorithm and still maintain correctness
   double xdiv,ydiv,zdiv;
   std::vector<pcl::PointCloud<PointT>, Eigen::aligned_allocator<pcl::PointCloud<PointT> >  >  clouds;
 //  std::vector<pcl::PointCloud<PointT>  > clouds;
   std::vector< std::vector<int> > cinds;

   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief Finds which octant the point would be if the origin was at point x,y,z
     * \param pt the point we are investigating
     * \param x,y,z the origin
     * \return octant number
     */
   int getIndex(const PointT &pt, double &x, double &y, double &z){
      int ret=0;
      if(pt.z > z) ret +=4;
      if(pt.y > y) ret +=2;
      if(pt.x > x) ret +=1;
      return ret;
    }

   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief splits the cloud into 8 overlapping clouds representing each octant
     * \param cloud the incoming point cloud
     */
   void initClouds(const pcl::PointCloud<PointT> &cloud){
     cinds.resize(8);
     clouds.resize(8);
     double x,y,z;
     for(int index=0;index<8;index++){
        //each octant is bounded by 3 planes, and extends to infinity in 3 directions
        //here we push the boundary of the octant across the dividing line
        //to add the buffer in each direction
       x=xdiv + (max_tol * ((index%2)?-1.0:1.0));
       y=ydiv + (max_tol * ((index%4 >= 2)?-1.0:1.0));
       z=zdiv + (max_tol * ((index >= 4)?-1.0:1.0));
       for(uint i=0;i<cloud.points.size();i++){
         if(getIndex(cloud.points[i],x,y,z) == index)
            cinds[index].push_back(i);
       }
       pcl::copyPointCloud(cloud,cinds[index],clouds[index]);
   //         cout<<"cloud "<<index<<" size: "<<cinds[index].size()<<"    "<<x<<" "<<y<<" "<<z<<endl;
     }//end for each octant
   }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief Constructor.  Sets the cloud we search over, and the maximum radius search we will guarantee to be correct
     * \param cloud the incoming point cloud
     * \param tol the maximum radius search we will guarantee to be correct
     */
   SplitCloud(const pcl::PointCloud<PointT> &cloud, double tol){
     max_tol=tol;
     Eigen::Vector4f vec;
     //we need to find a good place to divide the cloud, so how about the centroid?
     pcl::compute3DCentroid(cloud,vec);
     //_div represents the actual dividing lines, which select in which octant to look
     xdiv=vec.x(); ydiv=vec.y(); zdiv=vec.z();
     //split up that cloud into subclouds
     initClouds(cloud);
   }

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	  /** \brief a way to access the point cloud that we search over.  Rarely used.
		* \param pt the reference point
		* \return the corresponding point cloud
		*/
   pcl::PointCloud<PointT> & get(PointT &pt){
     return clouds[getIndex(pt,xdiv,ydiv,zdiv)];
   }


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	  /** \brief The main function of SplitCloud - does a radius search, returning indices of the original cloud
		* \param pt the pint to search around
		* \param inds a vector of indices of the original cloud that gets filled out with the results of the search
		* \param radius the search radius
		*/
   //performs radius search in a simplistic fashion
   void NNN(const PointT &pt, std::vector<int> &inds, double radius){
     int index=getIndex(pt,xdiv,ydiv,zdiv);

     inds.clear();
     double r2=radius*radius;
     double smallerrad=radius/sqrt(3);
     double diffx,diffy,diffz;
     for(uint i=0;i<clouds[index].points.size(); i++){
        //find the distance between the cloud point and the reference in the three dimensions:
       diffx=fabs(clouds[index].points[i].x - pt.x);
       diffy=fabs(clouds[index].points[i].y - pt.y);
       diffz=fabs(clouds[index].points[i].z - pt.z);
       //first find whether the point is in an axis aligned cube around the point -   this is a very quick check
       if(diffx < radius && diffy < radius && diffz < radius){ //in outer box
          //if the point is also in a cube whose circumradius is the search radius, the point is close enough:
         if(diffx < smallerrad && diffy < smallerrad && diffz < smallerrad) //also in inner box - include!
            inds.push_back(cinds[index][i]);
         else//between the boxes: check for actual distance
            if(diffx*diffx+diffy*diffy+diffz*diffz < r2)
              inds.push_back(cinds[index][i]);
       }//endif in outer box
     }//endfor all points in cloud
   }

}; //end SplitCloud class

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b SplitCloud2 uses 2 (three dimensional) layer of branching, creating 64 overlapping hexiquadrants.
 * \author Garratt Gallagher
 */
template <typename PointT>
class SplitCloud2{

   double max_tol; //maximum tolerance that can be used in any algorithm and still maintain correctness
   double xdiv,ydiv,zdiv;
   double xdivs[8],ydivs[8],zdivs[8];
   double lx[64],ux[64],ly[64],uy[64],lz[64],uz[64];
   std::vector<pcl::PointCloud<PointT> > clouds;

//   std::vector<pcl::PointCloud<PointT> > clouds;
   pcl::PointCloud<PointT> *_cloud;
   std::vector< std::vector<int> > cinds;
   std::vector< std::vector<int> > minds;

   //for searching over a subset of the indices
   std::vector< std::vector<int> > subinds;
   std::vector<int>  subindmap;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief returns the index of the first split (0-8)
 * \param pt the point we are investigating
 * \return octant number
 */
   int getMIndex(PointT &pt){
     int ret=0;
     if(pt.z > zdiv) ret +=4;
     if(pt.y > ydiv) ret +=2;
     if(pt.x > xdiv) ret +=1;
     return ret;
   }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief fill a list of indices that indicate which hexiquadrant the point is in
* also, fills out the boundaries of the subdivisions
* \param cloud the incoming point cloud
*/
   void fillMInds(pcl::PointCloud<PointT> &cloud){
     minds.resize(8);
     for(uint i=0;i<cloud.points.size();i++){
         minds[getMIndex(cloud.points[i])].push_back(i);
     }
     Eigen::Vector4f vec;
     for(int i=0;i<8;i++){
       pcl::compute3DCentroid(cloud,minds[i],vec);
       xdivs[i]=vec.x(); ydivs[i]=vec.y(); zdivs[i]=vec.z();
     }

   }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief returns the full index of the point
* This is called when searching the cloud, so we don't care about buffer zones
 * \param pt the point we are investigating
 * \return hexiquadrant number
 */
   int getIndex(PointT &pt){
     int ret=0,index;
     if(pt.z > zdiv) ret +=4;
     if(pt.y > ydiv) ret +=2;
     if(pt.x > xdiv) ret +=1;
     index=ret;
     if(pt.z > zdivs[index]) ret +=32;
     if(pt.y > ydivs[index]) ret +=16;
     if(pt.x > xdivs[index]) ret +=8;
     return ret;
   }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Finds out if the point is in a given index
* This is a slightly more efficient, but much more complicated way to determine if a point is in the given hexiquadrant
* Remember, a point cloud be in any number of hexiquadrants (it could be all of them, if the max_tolerance is large enough)
 * \param pt the point we are investigating
 * \param index the index of the branch we are checking
 * \return true if the point should be in that hexiquadrant
 */
   bool isIndex(PointT &pt, int index){
     //check main index:

     if((pt.x > (xdiv + max_tol * ((index%2)?-1.0:1.0))) == (index%2 ==0)) return false;
     if((pt.y > (ydiv + max_tol * ((index%4 >=2 )?-1.0:1.0))) == (index%4 < 2 )) return false;
     if((pt.z > (zdiv + max_tol * ((index%8 >= 4)?-1.0:1.0))) == (index%8 < 4 )) return false;
     //by this point, the first 3 bits are correct
     if((pt.x > (xdivs[index%8] + max_tol * ((index%16 >= 8)?-1.0:1.0))) == (index%16 < 8)) return false;
     if((pt.y > (ydivs[index%8] + max_tol * ((index%32 >=16 )?-1.0:1.0))) == (index%32 < 16 )) return false;
     if((pt.z > (zdivs[index%8] + max_tol * ((index >= 32)?-1.0:1.0))) == (index < 32 )) return false;
     return true;
   }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief splits the cloud into 64 overlapping clouds representing each hexiquadrant
 * \param cloud the incoming point cloud
 */
   void initClouds(pcl::PointCloud<PointT> &cloud){
     PointT minpt,maxpt;
     pcl::getMinMax3D(cloud,minpt,maxpt);

     double minx=minpt.x-.1,maxx=maxpt.x+.1,miny=minpt.y-.1,maxy=maxpt.y+.1,minz=minpt.z-.1,maxz=maxpt.z+.1; //maximum limits of cloud, with a little extra
     //lower and upper bounds on cloud with tolerance subdivisions:
     for(int i=0;i<64;i++){
        //set hard limits
        lx[i]=minx; ux[i]=maxx; ly[i]=miny; uy[i]=maxy; lz[i]=minz; uz[i]=maxz;

        //apply first layer of limits:
        if(i%2 >= 1) //x bigger than xdiv
           lx[i] = xdiv - max_tol;
        else
           ux[i] = xdiv + max_tol;

        if(i%4 >= 2) //y bigger than xdiv
           ly[i] = ydiv - max_tol;
        else
           uy[i] = ydiv + max_tol;

        if(i%8 >= 4) //z bigger than xdiv
           lz[i] = zdiv - max_tol;
        else
           uz[i] = zdiv + max_tol;

        //now apply second layer of limits:
        if(i%16 >= 8) //x bigger than xdiv
           lx[i] = std::max(xdivs[i%8] - max_tol, lx[i]);
        else
           ux[i] = std::min(xdivs[i%8] + max_tol, ux[i]);
        if(i%32 >= 16) //y bigger than xdiv
           ly[i] = std::max(ydivs[i%8] - max_tol, ly[i]);
        else
           uy[i] = std::min(ydivs[i%8] + max_tol, uy[i]);
        if(i%64 >= 32) //z bigger than xdiv
           lz[i] = std::max(zdivs[i%8] - max_tol, lz[i]);
        else
           uz[i] = std::min(zdivs[i%8] + max_tol, uz[i]);

     }


     cinds.resize(64);
   //  clouds.resize(64);
     for(uint i=0;i<cloud.points.size();i++){
        for(int index=0;index<64;index++){
           if (cloud.points[i].x >= lx[index] && cloud.points[i].x <= ux[index] && cloud.points[i].y >= ly[index] && cloud.points[i].y <= uy[index] && cloud.points[i].z >= lz[index] && cloud.points[i].z <= uz[index] )
            cinds[index].push_back(i);
       }
     }
       _cloud=&cloud;
   }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Constructor.  Sets the cloud we search over, and the maximum radius search we will guarantee to be correct
        * \param cloud the incoming point cloud
        * \param tol the maximum radius search we will guarantee to be correct
        */
   SplitCloud2(pcl::PointCloud<PointT> &cloud, double tol){
     max_tol=tol;
     Eigen::Vector4f vec;
     pcl::compute3DCentroid(cloud,vec);
     xdiv=vec.x(); ydiv=vec.y(); zdiv=vec.z();
     fillMInds(cloud); //fill out divs
     initClouds(cloud);

   }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief a way to access the point cloud that we search over.  Rarely used.
	* \param pt the reference point
	* \return the corresponding point cloud
	*/
   pcl::PointCloud<PointT> & get(PointT &pt){
     return clouds[getIndex(pt)];
   }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Set a subset of points that will be searched over. Calling NNN with the usesubset variable set to true will take advantage of this subset
	* \param inds the indices into the original point cloud that will be searched over
	*/
   void useInds(std::vector<int>  &inds){
       subinds.resize(64);
        for(uint i=0;i<inds.size();i++){
           for(int index=0;index<64;index++){
              if (_cloud->points[inds[i]].x >= lx[index] && _cloud->points[inds[i]].x <= ux[index] && _cloud->points[inds[i]].y >= ly[index] && _cloud->points[inds[i]].y <= uy[index] && _cloud->points[inds[i]].z >= lz[index] && _cloud->points[inds[i]].z <= uz[index] )
               subinds[index].push_back(i);
          }
        }
        subindmap=inds;
   }



   //performs radius search in a simplistic fashion
   //usesubset: if UseInds() has been called, then use the inds.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief The main function of SplitCloud2 - does a radius search, returning indices of the original cloud
	* \param pt the point to search around
	* \param inds a vector of indices of the original cloud that gets filled out with the results of the search
	* \param radius the search radius
	* \param usesubset if UseInds() has been called, then only search over those points
	*/
   void NNN(PointT pt, std::vector<int> &inds, double radius, bool usesubset=false){
     //check if usesubinds is a bad idea:
     if(usesubset && !subindmap.size()){
        ROS_WARN("SplitCloud2::NNN : cannot use subindex if UseInds has not been called!");
        usesubset=false;
     }

     int index=getIndex(pt);

     inds.clear();
     double r2=radius*radius;
     double smallerrad=radius/sqrt(3);
     double diffx,diffy,diffz;

     //rather than calling an if statement for every point in the cloud, just duplicate the code.
     //this does speed up the code a bit...
     if(!usesubset){
        for(uint i=0;i<cinds[index].size(); i++){
            //find the distance between the cloud point and the reference in the three dimensions:
          diffx=fabs(_cloud->points[cinds[index][i]].x - pt.x);
          diffy=fabs(_cloud->points[cinds[index][i]].y - pt.y);
          diffz=fabs(_cloud->points[cinds[index][i]].z - pt.z);
          //first find whether the point is in an axis aligned cube around the point -   this is a very quick check
          if(diffx < radius && diffy < radius && diffz < radius){ //in outer box
              //if the point is also in a cube whose circumradius is the search radius, the point is close enough:
            if(diffx < smallerrad && diffy < smallerrad && diffz < smallerrad) //also in inner box - include!
               inds.push_back(cinds[index][i]);
            else//between the boxes: check for actual distance
               if(diffx*diffx+diffy*diffy+diffz*diffz < r2)
                 inds.push_back(cinds[index][i]);
          }//endif in outer box
        }//endfor all points in cloud
     }
     else{
        for(uint i=0;i<subinds[index].size(); i++){
            //find the distance between the cloud point and the reference in the three dimensions:
          diffx=fabs(_cloud->points[subindmap[subinds[index][i]]].x - pt.x);
          diffy=fabs(_cloud->points[subindmap[subinds[index][i]]].y - pt.y);
          diffz=fabs(_cloud->points[subindmap[subinds[index][i]]].z - pt.z);
          //first find whether the point is in an axis aligned cube around the point -   this is a very quick check
          if(diffx < radius && diffy < radius && diffz < radius){ //in outer box
              //if the point is also in a cube whose circumradius is the search radius, the point is close enough:
            if(diffx < smallerrad && diffy < smallerrad && diffz < smallerrad) //also in inner box - include!
               inds.push_back(subinds[index][i]);
            else//between the boxes: check for actual distance
               if(diffx*diffx+diffy*diffy+diffz*diffz < r2)
                 inds.push_back(subinds[index][i]);
          }//endif in outer box
        }//endfor all points in cloud
     }

   }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief a version of the main function of SplitCloud2 - does a radius search, returning indices of the original cloud
   * this version returns the distance to each point as well
	* \param pt the point to search around
	* \param inds a vector of indices of the original cloud that gets filled out with the results of the search
	* \param dists a vector of floats that gets filled out with the squares of distances to points found
	* \param radius the search radius
	* \param usesubset if UseInds() has been called, then only search over those points
	*/
    void NNN(PointT pt, std::vector<int> &inds, std::vector<float> &dists, double radius, bool usesubset=false){
      //check if usesubinds is a bad idea:
      if(usesubset && !subindmap.size()){
         ROS_WARN("SplitCloud2::NNN : cannot use subindex if UseInds has not been called!");
         usesubset=false;
      }

      int index=getIndex(pt);
      inds.clear();
      double r2=radius*radius;
      double diffx,diffy,diffz;
      double sqrdist;
      //rather than calling an if statement for every point in the cloud, just duplicate the code.
      //this does speed up the code a bit...
      if(!usesubset){
         for(uint i=0;i<cinds[index].size(); i++){
             //find the distance between the cloud point and the reference in the three dimensions:
           diffx=fabs(_cloud->points[cinds[index][i]].x - pt.x);
           diffy=fabs(_cloud->points[cinds[index][i]].y - pt.y);
           diffz=fabs(_cloud->points[cinds[index][i]].z - pt.z);
           //first find whether the point is in an axis aligned cube around the point -   this is a very quick check
           if(diffx < radius && diffy < radius && diffz < radius){ //in outer box
             sqrdist=diffx*diffx+diffy*diffy+diffz*diffz;
             if(sqrdist < r2){
               inds.push_back(cinds[index][i]);
               dists.push_back(sqrdist);
             }
           }//endif in outer box
         }//endfor all points in cloud
      }
      else{
         for(uint i=0;i<subinds[index].size(); i++){
             //find the distance between the cloud point and the reference in the three dimensions:
           diffx=fabs(_cloud->points[subindmap[subinds[index][i]]].x - pt.x);
           diffy=fabs(_cloud->points[subindmap[subinds[index][i]]].y - pt.y);
           diffz=fabs(_cloud->points[subindmap[subinds[index][i]]].z - pt.z);
           //first find whether the point is in an axis aligned cube around the point -   this is a very quick check
           if(diffx < radius && diffy < radius && diffz < radius){ //in outer box
              sqrdist=diffx*diffx+diffy*diffy+diffz*diffz;
              if(sqrdist < r2){
                 inds.push_back(subinds[index][i]);
                 dists.push_back(sqrdist);

              }
           }//endif in outer box
         }//endfor all points in cloud
      }

    }

    //For heavily optimized system:
    //inds is assumed to be a vector of the same size as the initial point cloud
    //mark is the number to assign to those indices
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief Just mark a set of indices, rather than returning the vector of found indices (for heavily optimized systems)
	* this version only finds points whose z component is within maxzoffset of the reference
	 * \param pt the point to search around
	 * \param inds a vector of indices of the original cloud that gets marked  with the results of the search.  This vector should already be the same size as the point cloud
	 * \param radius the search radius
	 * \param mark the integer label to apply to the indices that are within the search radius
	 * \param maxzoffset the max zdist matching points can have from the refs
	 */
    int AssignInds_filterz(PointT pt, std::vector<int> &inds, double radius, int mark, double maxzoffset){
      int index=getIndex(pt);
      double r2=radius*radius;
      double smallerrad=radius/sqrt(3);
      double diffx,diffy,diffz;
      int foundcount=0;
      for(uint i=0;i<cinds[index].size(); i++){
        diffz=fabs(_cloud->points[cinds[index][i]].z - pt.z);
        if(diffz > maxzoffset) continue;
        diffx=fabs(_cloud->points[cinds[index][i]].x - pt.x);
        diffy=fabs(_cloud->points[cinds[index][i]].y - pt.y);
        //first find whether the point is in an axis aligned cube around the point -   this is a very quick check
        if(diffx < radius && diffy < radius && diffz < radius){ //in outer box
          if(diffx < smallerrad && diffy < smallerrad && diffz < smallerrad){ //also in inner box - include!
             inds[cinds[index][i]]=mark;
             foundcount++;
          }
          else//between the boxes: check for actual distance
             if(diffx*diffx+diffy*diffy+diffz*diffz < r2){
               inds[cinds[index][i]]=mark;
               foundcount++;
             }
        }//endif in outer box
      }//endfor all points in cloud
      return foundcount;
    }

   //For heavily optimized system:
   //inds is assumed to be a vector of the same size as the initial point cloud
   //mark is the number to assign to those indices
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief Just mark a set of indices, rather than returning the vector of found indices (for heavily optimized systems)
	 * \param pt the point to search around
	 * \param inds a vector of indices of the original cloud that gets marked  with the results of the search.  This vector should already be the same size as the point cloud
	 * \param radius the search radius
	 * \param mark the integer label to apply to the indices that are within the search radius
	 */
   int AssignInds(PointT pt, std::vector<int> &inds, double radius, int mark){
     int index=getIndex(pt);
     double r2=radius*radius;
     double smallerrad=radius/sqrt(3);
     double diffx,diffy,diffz;
     int foundcount=0;
     for(uint i=0;i<cinds[index].size(); i++){
         //find the distance between the cloud point and the reference in the three dimensions:
       diffx=fabs(_cloud->points[cinds[index][i]].x - pt.x);
       diffy=fabs(_cloud->points[cinds[index][i]].y - pt.y);
       diffz=fabs(_cloud->points[cinds[index][i]].z - pt.z);
       //first find whether the point is in an axis aligned cube around the point -   this is a very quick check
       if(diffx < radius && diffy < radius && diffz < radius){ //in outer box
         if(diffx < smallerrad && diffy < smallerrad && diffz < smallerrad){ //also in inner box - include!
            inds[cinds[index][i]]=mark;
            foundcount++;
         }
         else//between the boxes: check for actual distance
            if(diffx*diffx+diffy*diffy+diffz*diffz < r2){
              inds[cinds[index][i]]=mark;
              foundcount++;
            }
       }//endif in outer box
     }//endfor all points in cloud
     return foundcount;
   }

   //for unit testing:
   //this function checks to see if the indexed point is in the right hexiquadrant
   bool checkIndex(int index){
	   //get the point from the main cloud:
	   int ind=getIndex(_cloud->points[index]);
	   return isIndex(_cloud->points[index],ind);
   }


};







#endif /* NNN_H_ */
