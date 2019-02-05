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
#include <Eigen/StdVector>


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <mapping_msgs/PolygonalMap.h>
#include <body_msgs/Hands.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_tools/pcl_utils.h>
#include <nnn/nnn.hpp>
#include <pcl_tools/segfast.hpp>


#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/ModelCoefficients.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/features/normal_3d.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <body_msgs/Hands.h>


float gdist(pcl::PointXYZ pt, const Eigen::Vector4f &v){
   return sqrt((pt.x-v(0))*(pt.x-v(0))+(pt.y-v(1))*(pt.y-v(1))+(pt.z-v(2))*(pt.z-v(2))); //
}

void flipvec(const Eigen::Vector4f &palm, const Eigen::Vector4f &fcentroid,Eigen::Vector4f &dir ){
   if((fcentroid-palm).dot(dir) <0)
      dir=dir*-1.0;

 }

geometry_msgs::Point32 eigenToMsgPoint32(const Eigen::Vector4f &v){
	geometry_msgs::Point32 p;
	p.x=v(0); p.y=v(1); p.z=v(2);
	return p;
}
geometry_msgs::Point eigenToMsgPoint(const Eigen::Vector4f &v){
	geometry_msgs::Point p;
	p.x=v(0); p.y=v(1); p.z=v(2);
	return p;
}

pcl::PointXYZ eigenToPclPoint(const Eigen::Vector4f &v){
   pcl::PointXYZ p;
   p.x=v(0); p.y=v(1); p.z=v(2);
   return p;
}


namespace handdetector{

enum FingerName {THUMB,INDEXF,MIDDLEF,RINGF,PINKY,UNKNOWN};

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b Finger is mostly an organizational tool. it holds all the information for one finger
 * \author Garratt Gallagher
 */
class Finger{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   pcl::PointCloud<pcl::PointXYZ> cloud;
   handdetector::FingerName fname;
   Eigen::Vector4f centroid, direction;
   Finger(pcl::PointCloud<pcl::PointXYZ> &cluster, Eigen::Vector4f &palmcenter){
      cloud=cluster;
      EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
      EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
      Eigen::Matrix3f cov;
      pcl::compute3DCentroid (cluster, centroid);
      pcl::computeCovarianceMatrixNormalized(cluster,centroid,cov);
      pcl::eigen33 (cov, eigen_vectors, eigen_values);
      direction(0)=eigen_vectors (0, 2);
      direction(1)=eigen_vectors (1, 2);
      direction(2)=eigen_vectors (2, 2);
      flipvec(palmcenter,centroid,direction);
   }

   geometry_msgs::Polygon getNormalPolygon(){
      geometry_msgs::Polygon p;
      p.points.push_back(eigenToMsgPoint32(centroid));
      p.points.push_back(eigenToMsgPoint32(centroid+direction*.1));
      return p;
   }


};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b HandProcessor does the heavy lifting for finding fingers.
 * \author Garratt Gallagher
 */
class HandProcessor{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    pcl::PointCloud<pcl::PointXYZ> full,digits,palm,digits2;
    std::vector<Finger,Eigen::aligned_allocator<Finger> > fingers;
    body_msgs::Hand handmsg;
    double distfromsensor;
    Eigen::Vector4f centroid,arm;
    int thumb;

//    HandProcessor(pcl::PointCloud<pcl::PointXYZ> &cloud){
//       full=cloud;
//        pcl::compute3DCentroid (full, centroid);
//        distfromsensor=centroid.norm();  //because we are in the sensor's frame
//        thumb=-1;
//        handmsg.thumb=thumb;
//        handmsg.stamp=cloud.header.stamp;
//    }
    //for re-initializing a handProcessor object, so we don't have to re-instantiate
    void Init(pcl::PointCloud<pcl::PointXYZ> &cloud,const Eigen::Vector4f &_arm){
      full=cloud;
        pcl::compute3DCentroid (full, centroid);
        distfromsensor=centroid.norm();  //because we are in the sensor's frame
        thumb=-1;
        handmsg.thumb=thumb;
        handmsg.stamp=cloud.header.stamp;
        digits=pcl::PointCloud<pcl::PointXYZ>();
        palm=pcl::PointCloud<pcl::PointXYZ>();
        arm=_arm;
        handmsg.arm=eigenToMsgPoint(arm);

    }

    void Init(const body_msgs::Hand &_handmsg){
       handmsg=_handmsg;

       pcl::fromROSMsg(_handmsg.handcloud,full);
        pcl::compute3DCentroid (full, centroid);
        distfromsensor=centroid.norm();  //because we are in the sensor's frame
        digits=pcl::PointCloud<pcl::PointXYZ>();
        palm=pcl::PointCloud<pcl::PointXYZ>();
        arm(0)=handmsg.arm.x;
        arm(1)=handmsg.arm.y;
        arm(2)=handmsg.arm.z;
    }

    //
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief filter the fingers from the palm.  This is done by performing radius searches to determine the point density.
     * when the density drops off, that's the finger!
      * \param nnthresh number of neighbors we expect to see on the palm in a radius search
      * \param tol the size of the search region when we are doing radius searches
      */
    void radiusFilter(int nnthresh, double tol){
//       //first remove points near the arm:
//       vector<int> tempinds;
//       NNN(full,eigenToPclPoint(arm),tempinds,.1);
//       getSubCloud(full, tempinds, full,false);



      timeval t0=g_tick();
      double t1,t2,t3;
      std::vector<int> inds,inds2,inds3;
      std::vector<int> searchinds;


       SplitCloud2<pcl::PointXYZ> sc2(full,tol);
       inds2.resize(full.points.size(),-1);
       t1=g_tock(t0);t0=g_tick();
       int label;

       //DEBUG:
//       find the number of points near a point at the center in x dist, but at y and z coord of 0 0
//       pcl::PointXYZ testpt;
//       testpt.x=0;
//       testpt.y=0;
//       testpt.z=centroid(2);
//       sc2.NNN(testpt,searchinds,tol);
//       if(searchinds.size()){
//          testpt=full.points[searchinds[0]];
//          sc2.NNN(testpt,searchinds,tol);
//       }
//       printf("%.02f, %.02f, %.02f searchinds.size() = %d \n",centroid(0),centroid(1),centroid(2),(int)searchinds.size());

       for(uint i=0;i<full.points.size();++i){
        if(inds2[i]==0) continue;
          sc2.NNN(full.points[i],searchinds,tol);
          //TODO: this is good for face-on, but not great for tilted hands
          if(searchinds.size()>(530-500*distfromsensor)){
             inds.push_back(i);

             if(searchinds.size()>(570-500*distfromsensor))
                label=0;
             else
                label=1;
             for(uint j=0;j<searchinds.size();++j)
                inds2[searchinds[j]]=label;
          }

       }

       t2=g_tock(t0);t0=g_tick();
       for(uint i=0;i<full.points.size();++i)
          if(inds2[i]==-1)
             inds3.push_back(i);

       getSubCloud(full, inds, palm,true);
       getSubCloud(full,inds3, digits,true);


      t3=g_tock(t0);
//    printf("radius: %05d %.03f, %.03f, %.03f   ",full.points.size(),t1,t2,t3);
       std::cout<<"dist:  "<<distfromsensor<<"  palm: "<<palm.points.size()<<"  digits: "<<digits.points.size()<<" "<<(575-500*distfromsensor)<<std::endl;
    }

    sensor_msgs::PointCloud2 getPalm(){
      sensor_msgs::PointCloud2 cloud;
      pcl::toROSMsg(palm,cloud);
      return cloud;
    }
    sensor_msgs::PointCloud2 getDigits(){
      sensor_msgs::PointCloud2 cloud;
      pcl::toROSMsg(digits,cloud);
      return cloud;
    }
    sensor_msgs::PointCloud2 getFull(){
      sensor_msgs::PointCloud2 cloud;
      pcl::toROSMsg(full,cloud);
      return cloud;
    }

    void addFingerDirs(mapping_msgs::PolygonalMap &pmap){
      for(uint i=0;i<fingers.size();++i)
         pmap.polygons.push_back(fingers[i].getNormalPolygon());

    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief runs a cluster segmentation to differentiate the fingers from each other
      * \param clustertol the max distance between a point on one finger and it's nearest neighbor
      * \param mincluster the fewest number of points allowed in a finger
      */
    void segFingers(double clustertol=.005, int mincluster=50){
       handmsg.palm.translation.x=centroid(0);
       handmsg.palm.translation.y=centroid(1);
       handmsg.palm.translation.z=centroid(2);
      if(digits.size()==0)
         return;
      std::vector< std::vector<int> > indclusts;
       extractEuclideanClustersFast2(digits,indclusts,clustertol,mincluster);
//       cout<<" clusters: "<<indclusts.size()<<endl;
       if(!indclusts.size()) return;
       pcl::PointCloud<pcl::PointXYZ> cluster;
       for(uint i=0;i<indclusts.size();++i){
             getSubCloud(digits,indclusts[i], cluster,true);
             fingers.push_back(Finger(cluster,centroid));
             //if it is actually the wrist, it is easily identified because the largest eigenvalue is perpendicular to the vector from the wrist
             //also, because we flip the 'normal' already, we are guaranteed this is positive:
//             if((fingers.back().centroid-centroid).dot(fingers.back().direction)/(fingers.back().centroid-centroid).norm() < .5 ){//a very conservative value...
            if((fingers.back().centroid-centroid).dot(centroid-arm)/((fingers.back().centroid-centroid).norm() * (centroid-arm).norm()) < 0.0 ){//a very conservative value...
                               fingers.pop_back();
             }
//             TODO: DEBUG
//             else{ //if it is a good finger, add it to digits2:
//                if(fingers.size()==1)
//                   digits2=cluster;
//                else
//                   digits2+=cluster;
//
//             }
//              cout<<indclusts[i].size()<<" ("<<(fingers.back().centroid-centroid).dot(fingers.back().direction)/(fingers.back().centroid-centroid).norm()<<")  ";

       }
//       cout<<endl;
       for(uint i=0;i<fingers.size();++i)
         handmsg.fingers.push_back(eigenToMsgPoint(fingers[i].centroid));

    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief locates the thumb, and then orders the fingers
      */
    void identfyFingers(){
      if(!fingers.size()) return;
      int farthest=0;
      thumb=0;
      if(fingers.size()>2){
         //try to identify the thumb based on distance:
         double biggest_dist=0;
         for(uint i=1;i<fingers.size();++i){
           double dist=10;
           for(uint j=1;j<fingers.size();++j){//find smallest distance to neighbor
             if(i!=j && (fingers[i].centroid-fingers[j].centroid).norm() < dist)
               dist=(fingers[i].centroid-fingers[j].centroid).norm();
           }
           if(i==1 || dist > biggest_dist){
             farthest=i;
//             std::cout<<"farthest = "<<farthest<<std::endl;
             biggest_dist=dist;
           }
         }
         thumb=farthest;
      }
      //add a point beyond the end of the thumb to mark it:
//        pcl::PointXYZ pt;
//        pt.x=fingers[thumb].centroid(0)+.1*fingers[thumb].direction(0);
//        pt.y=fingers[thumb].centroid(1)+.1*fingers[thumb].direction(1);
//        pt.z=fingers[thumb].centroid(2)+.1*fingers[thumb].direction(2);
//        digits.push_back(pt);
//        digits.width++;
        handmsg.thumb=thumb;
//        handmsg.palm.rotation.x=fingers[thumb].direction(0);
//        handmsg.palm.rotation.y=fingers[thumb].direction(1);
//        handmsg.palm.rotation.z=fingers[thumb].direction(2);
//        handmsg.palm.rotation.w=0.0;

//        Eigen::Vector4f minpt,maxpt;
//        pcl::getMinMax3D(digits,minpt,maxpt);
//        cout<<"hand size: "<<(maxpt-minpt).norm()<<" ";

    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief The method to rule them all: when processing a hand, just call this function.
      */
    void Process(){
      timeval t0=g_tick();
      double t1,t2,t3;
      radiusFilter(300,.02);
      t1=g_tock(t0);t0=g_tick();
      segFingers();
      t2=g_tock(t0);t0=g_tick();
       identfyFingers();
      t3=g_tock(t0);
//    printf("process: %.03f, %.03f, %.03f",t1,t2,t3);
//    std::cout<<"process: "<<setw(6)<<t1<<setw(6)<<",  "<<t2<<",  "<<t3;
    }


};



class HandAnalyzer
{

private:
  ros::NodeHandle n_;
  ros::Publisher cloudpub_[2],cloudpub2_[2],pmappub_,handspub_;
  ros::Subscriber sub_;
  mapping_msgs::PolygonalMap pmap;

public:

  HandAnalyzer(int p1=1, double p2=2.0)
  {
   handspub_ = n_.advertise<body_msgs::Hands> ("hands_pros", 1);
   pmappub_ = n_.advertise<mapping_msgs::PolygonalMap> ("finger_norms", 1);
   cloudpub_[0] = n_.advertise<sensor_msgs::PointCloud2> ("hand0_cloud", 1);
   cloudpub_[1] = n_.advertise<sensor_msgs::PointCloud2> ("hand1_cloud", 1);
   cloudpub2_[0] = n_.advertise<sensor_msgs::PointCloud2> ("hand0_cloud2", 1);
   cloudpub2_[1] = n_.advertise<sensor_msgs::PointCloud2> ("hand1_cloud2", 1);
    sub_=n_.subscribe("/hands", 1, &HandAnalyzer::handscb, this);
  }





  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Gets the direction of the hand.  Useful for determining where the hand is pointing
    */
  void getEigens(const body_msgs::Hand &h){
     pcl::PointCloud<pcl::PointXYZ> cloud;

     Eigen::Vector4f centroid, direction,armvector;
     pcl::fromROSMsg(h.handcloud,cloud);
     EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
       EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
       Eigen::Matrix3f cov;
       pcl::compute3DCentroid (cloud, centroid);
       pcl::computeCovarianceMatrixNormalized(cloud,centroid,cov);
       pcl::eigen33 (cov, eigen_vectors, eigen_values);
       direction(0)=eigen_vectors (0, 2);
       direction(1)=eigen_vectors (1, 2);
       direction(2)=eigen_vectors (2, 2);
       armvector(0)=h.arm.x; armvector(1)=h.arm.y; armvector(2)=h.arm.z;
       flipvec(armvector,centroid,direction);
       printf("Eigenvalues: %.02f, %.02f \n",eigen_values(0)/eigen_values(1),eigen_values(1)/eigen_values(2));

       //eigen eigen_values(1)/eigen_values(2) < .4 means closed fist, unless you are pointing at the kinect

       //make polygon
       geometry_msgs::Polygon p;
       p.points.push_back(eigenToMsgPoint32(centroid));
       p.points.push_back(eigenToMsgPoint32(centroid+direction));
       pmap.polygons.push_back(p);
       pmap.header=h.handcloud.header;
  }

  void ProcessHand(body_msgs::Hand &hand){
     HandProcessor hp;
     hp.Init(hand);
     hp.Process();

//     hp.radiusFilter(300,.02);
     hp.addFingerDirs(pmap);
     if(hand.left){
        cloudpub_[0].publish(hp.getPalm());
        cloudpub2_[0].publish(hp.getDigits());
     }
     else{
        cloudpub_[1].publish(hp.getPalm());
        cloudpub2_[1].publish(hp.getDigits());
     }
     //update the original message:
     hand=hp.handmsg;
     pmap.header=hand.handcloud.header;

  }



  void handscb(const body_msgs::HandsConstPtr &hands){
     body_msgs::Hands handsout=*hands;
     pmap.polygons.clear();
     pmap.header=hands->header;
     for(uint i=0;i<hands->hands.size();i++){
//        getEigens(hands->hands[i]);
//        if(hands->hands[i].left)
//           cloudpub_[0].publish(hands->hands[i].handcloud);
//        else
//           cloudpub_[1].publish(hands->hands[i].handcloud);
        ProcessHand(handsout.hands[i]);
        handsout.header=hands->hands[0].handcloud.header;

     }
     pmappub_.publish(pmap);
     handspub_.publish(handsout);
  }

} ;




int main(int argc, char **argv)
{
  ros::init(argc, argv, "finger_detector");
  ros::NodeHandle n;
  HandAnalyzer detector;
  ros::spin();
  return 0;
}
