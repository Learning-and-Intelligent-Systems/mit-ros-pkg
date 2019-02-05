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


class TimeEvaluator{
   timeval tmark;
   std::vector<double> times;
   std::vector<std::string> eventnames;
   std::string name;  //name is used to identify the evaluator as a whole


public:
   TimeEvaluator(std::string _name="Time Evaluator"){
      name=_name;
      //be default the clock starts running when TimeEvaluator is initialized

      gettimeofday(&tmark, NULL);

   }
   //records time diff;
   void mark(std::string _name=""){
      //Give the event a name:
      if(_name.size())
         eventnames.push_back(_name);
      else{
         int count=eventnames.size();
         char tname[10];
         sprintf(tname,"E%d",count);
         eventnames.push_back(std::string(tname));
      }
      //record the time since last event
      struct timeval tv;
      gettimeofday(&tv, NULL);
      times.push_back((double)(tv.tv_sec-tmark.tv_sec) + (tv.tv_usec-tmark.tv_usec)/1000000.0);
   }

   void print(){
      std::cout<<name;
      for(uint i=0;i<times.size();++i)
         std::cout<<"  "<<eventnames[i]<<": "<< std::setprecision (5) << times[i];
      std::cout<<std::endl;
   }




};



float gdist(pcl::PointXYZ pt, Eigen::Vector4f v){
   return sqrt((pt.x-v(0))*(pt.x-v(0))+(pt.y-v(1))*(pt.y-v(1))+(pt.z-v(2))*(pt.z-v(2))); //
}

void flipvec(Eigen::Vector4f palm, Eigen::Vector4f fcentroid,Eigen::Vector4f &dir ){
   if((fcentroid-palm).dot(dir) <0)
      dir=dir*-1.0;

 }

geometry_msgs::Point32 eigenToMsgPoint32(Eigen::Vector4f v){
	geometry_msgs::Point32 p;
	p.x=v(0); p.y=v(1); p.z=v(2);
	return p;
}
geometry_msgs::Point eigenToMsgPoint(Eigen::Vector4f v){
	geometry_msgs::Point p;
	p.x=v(0); p.y=v(1); p.z=v(2);
	return p;
}

pcl::PointXYZ eigenToPclPoint(Eigen::Vector4f v){
   pcl::PointXYZ p;
   p.x=v(0); p.y=v(1); p.z=v(2);
   return p;
}

//find the points that are ajoining a cloud, but not in it:
//cloud: the full cloud
//cloudpts a vector of indices into cloud that represents the cluster for which we want to find near points
//centroid: the centroid of the nearby pts
//return: true if points were found within 5cm
bool findNearbyPts(pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<int> &cloudpts, Eigen::Vector4f &centroid){
   std::vector<int> inds(cloud.size(),1); //a way of marking the points we have looked at
   // 1: not in the cluster  0: in the cluster, seen  -1: in the cluster, not seen
   std::vector<int> nearpts; //a way of marking the points we have looked at
   std::vector<int> temp;
   for(uint i=0;i<cloudpts.size(); ++i) inds[cloudpts[i]]=-1;
   for(uint i=0;i<cloudpts.size(); ++i){
      if(inds[cloudpts[i]]==-1){
         NNN(cloud,cloud.points[cloudpts[i]],temp, .05);
               mapping_msgs::PolygonalMap pmap;
               geometry_msgs::Polygon p;
         for(uint j=0;j<temp.size(); ++j){
            if(inds[temp[j]]==1){
               nearpts.push_back(temp[j]);
               inds[temp[j]]=2;
            }
            else
               inds[temp[j]]=-2;
         }
      }
   }
   //TODO: check if we are really just seeing the other hand:
   //       remove any points that do not have a point w/in 1cm
   if(nearpts.size())
   //now find the centroid of the nearcloud:
      pcl::compute3DCentroid(cloud,nearpts,centroid);
   else
      return false;
   return true;
}



bool getNearBlobs2(pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<pcl::PointCloud<pcl::PointXYZ> > &clouds, std::vector< Eigen::Vector4f> &nearcents ){
	pcl::PointCloud<pcl::PointXYZ> cloudout;
   pcl::PointXYZ pt,pt1,pt2; pt.x=pt.y=pt.z=0;
   std::vector<int> inds1,inds2,inds3(cloud.size(),1);
   std::vector<float> dists;
   Eigen::Vector4f centroid1,centroid2,nearcent1;
//   bool foundarm=false;

   //for debugging delays:
   TimeEvaluator te("getNearBlobs2: ");
//----------FIND FIRST HAND--------------------------

   //find closest pt to camera:
   NNN(cloud,pt,inds1,dists, 1.0);
   int ind=0; double smallestdist;
   for(uint i=0;i<dists.size(); ++i){
      if(dists[i]<smallestdist || i==0 ){
         ind=inds1[i];
         smallestdist=dists[i];
      }
   }
   smallestdist=sqrt(smallestdist);
   pt1=cloud.points[ind];

   te.mark("closest pt");

   //find points near that the closest point
   NNN(cloud,pt1,inds2, .1);

   //if there is nothing near that point, we're probably seeing noise.  just give up
   if(inds2.size() < 100){
	   std::cout<<"very few points ";
	   return false;
   }

   te.mark("nearby pts");
   //Iterate the following:
   //    find centroid of current cluster
   //    add a little height, to drive the cluster away from the arm
   //    search again around the centroid to redefine our cluster

   pcl::compute3DCentroid(cloud,inds2,centroid1);
   pt2.x=centroid1(0); pt2.y=centroid1(1)-.02; pt2.z=centroid1(2);
   NNN(cloud,pt2,inds2, .1);

   //in the middle of everything, locate where the arms is:
   std::vector<int> temp;
   NNN(cloud,pt2,temp, .15);
   //finding the arms is really reliable. we'll just throw out anytime when we can't find it.
   if(!findNearbyPts(cloud,temp,nearcent1))
      return false;


   te.mark("find arm");

   pcl::compute3DCentroid(cloud,inds2,centroid1);
   pt2.x=centroid1(0); pt2.y=centroid1(1)-.01; pt2.z=centroid1(2);
   NNN(cloud,pt2,inds2, .1);

   //save this cluster as a separate cloud.
   getSubCloud(cloud,inds2,cloudout);


   te.mark("save sub ");

   //-------Decide whether we are looking at potential hands:
   //try to classify whether this is actually a hand, or just a random object (like a face)
   //if there are many points at the same distance that we did not grab, then the object is not "out in front"
   for(uint i=0;i<inds2.size(); ++i) inds3[inds2[i]]=0; //mark in inds3 all the points in the potential hand
   pcl::compute3DCentroid(cloudout,centroid1);
   int s1,s2=0;
   s1=inds2.size();
   //search for all points in the cloud that are as close as the center of the potential hand:
   NNN(cloud,pt,inds2, centroid1.norm());
   for(uint i=0;i<inds2.size(); ++i){
      if(inds3[inds2[i]]) ++s2;
   }
   if(((float)s2)/((float)s1) > .3){
      std::cout<<"No hands detected ";
      return false;
   }


   te.mark("classify ");
//   te.print();
   //OK, we have decided that there is at least one hand.
   clouds.push_back(cloudout);
//   if(!foundarm) //if we never found the arm, use the centroid
    nearcents.push_back(nearcent1);

   //-----------------FIND SECOND HAND---------------------
   //find next smallest point
   smallestdist+=.3;
   smallestdist*=smallestdist;
//   double thresh=smallestdist;
   bool foundpt=false;
   for(uint i=0;i<dists.size(); ++i){
      //a point in the second had must be:
      //   dist to camera must be within 30 cm of the first hand's closest dist
      //   more than 20 cm from the center of the first hand
      //   more than 30 cm from the center of the arm

      if(dists[i]<smallestdist && inds3[i] && gdist(cloud.points[inds1[i]],centroid1) > .2  && gdist(cloud.points[inds1[i]],nearcent1) >.3){
//         printf("found second hand point %.03f  hand dist = %.03f, arm dist = %.03f \n",
//               dists[i],gdist(cloud.points[inds1[i]],centroid1),gdist(cloud.points[inds1[i]],nearcent1));
         ind=inds1[i];
         smallestdist=dists[i];
         foundpt=true;
      }
   }

   if(foundpt){
//	   cout<<" 2nd run: "<<thresh-smallestdist;
	   pcl::PointCloud<pcl::PointXYZ> cloudout2;
	   NNN(cloud,cloud.points[ind],inds2, .1);
	   pcl::compute3DCentroid(cloud,inds2,centroid2);
	   pt2.x=centroid2(0); pt2.y=centroid2(1)-.02; pt2.z=centroid2(2);
	   NNN(cloud,pt2,inds2, .1);
	   pcl::compute3DCentroid(cloud,inds2,centroid2);
	   pt2.x=centroid2(0); pt2.y=centroid2(1)-.01; pt2.z=centroid2(2);
	   NNN(cloud,pt2,inds2, .1);

	   //if too few points in the second hand, discard
	   if(inds2.size()<100) return true;

	   //check for overlapping points. if there are any, we don't want it!
//	   int overlap=0;
	   for(uint i=0;i<inds2.size(); ++i)
		   if(inds3[inds2[i]]==0)
		      return true;

	   NNN(cloud,pt2,temp, .15);
	   //finding the arms is really reliable. we'll just throw out anytime when we can't find it.
	   if(!findNearbyPts(cloud,temp,nearcent1))
	      return true;

	   getSubCloud(cloud,inds2,cloudout2);
	   clouds.push_back(cloudout2);
	   nearcents.push_back(nearcent1);

   }



   return true;

}


class HandDetector
{

private:
  ros::NodeHandle n_;
  ros::Publisher cloudpub_[2],handspub_;
  ros::Subscriber sub_;
  std::string fixedframe;

public:

  HandDetector()
  {
   handspub_ = n_.advertise<body_msgs::Hands> ("hands", 1);
   cloudpub_[0] = n_.advertise<sensor_msgs::PointCloud2> ("hand0_cloud", 1);
   cloudpub_[1] = n_.advertise<sensor_msgs::PointCloud2> ("hand1_cloud", 1);
    sub_=n_.subscribe("/camera/rgb/points", 1, &HandDetector::cloudcb, this);

  }




  void makeHand(pcl::PointCloud<pcl::PointXYZ> &cloud,Eigen::Vector4f &_arm,  body_msgs::Hand &handmsg){
    Eigen::Vector4f centroid;
    handmsg.thumb=-1; //because we have not processed the hand...
    handmsg.stamp=cloud.header.stamp;
    pcl::compute3DCentroid (cloud, centroid);
    handmsg.arm=eigenToMsgPoint(_arm);
    handmsg.state="unprocessed";
    handmsg.palm.translation.x=centroid(0);
    handmsg.palm.translation.y=centroid(1);
    handmsg.palm.translation.z=centroid(2);
    pcl::toROSMsg(cloud,handmsg.handcloud);
    handmsg.handcloud.header=cloud.header;
    //TODO: do tracking seq
  }


  void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan){
	  timeval t0=g_tick();
     sensor_msgs::PointCloud2 cloud2;
     pcl::PointCloud<pcl::PointXYZ> cloud;
     pcl::fromROSMsg(*scan,cloud);
     std::vector<Eigen::Vector4f> arm_center;
	   std::vector<pcl::PointCloud<pcl::PointXYZ> > initialclouds;
      std::cout<<" pre blob time:  "<<g_tock(t0)<<"  ";
	  	if(!getNearBlobs2(cloud,initialclouds,arm_center)){
	  	   std::cout<<" no hands detected "<<std::endl;
	  	   return;
	  	}
      std::cout<<" blob time:  "<<g_tock(t0)<<"  ";

	  	body_msgs::Hand hand1,hand2;
	  	body_msgs::Hands hands;

	  	// Publish hands
	  	if(initialclouds.size()==1){
	  	  makeHand(initialclouds[0],arm_center[0],hand1);
        hands.hands.push_back(hand1);
        cloudpub_[0].publish(hand1.handcloud);
        handspub_.publish(hands);
	  	}


      if(initialclouds.size()==2){
         makeHand(initialclouds[0],arm_center[0],hand1);
         makeHand(initialclouds[1],arm_center[1],hand2);
         //decide which is the left hand, which goes first:
         if(hand1.palm.translation.x < hand2.palm.translation.x){ //TODO: make sure this is right!
            hands.hands.push_back(hand1);
            hands.hands.push_back(hand2);
         }
         else{
            hands.hands.push_back(hand2);
            hands.hands.push_back(hand1);
         }
         cloudpub_[0].publish(hands.hands[0].handcloud);
         cloudpub_[1].publish(hands.hands[1].handcloud);
         handspub_.publish(hands);
      }


      std::cout<<" total time:  "<<g_tock(t0)<<std::endl;
  }

} ;




int main(int argc, char **argv)
{
  ros::init(argc, argv, "hand_detector");
  ros::NodeHandle n;
  HandDetector detector;
  ros::spin();
  return 0;
}
