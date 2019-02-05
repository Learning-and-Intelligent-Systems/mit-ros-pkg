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
#include <kinect_tools/Hands.h>
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


float gdist(pcl::PointXYZ pt, Eigen3::Vector4f v){
   return sqrt((pt.x-v(0))*(pt.x-v(0))+(pt.y-v(1))*(pt.y-v(1))+(pt.z-v(2))*(pt.z-v(2))); //
}

void flipvec(Eigen3::Vector4f palm, Eigen3::Vector4f fcentroid,Eigen3::Vector4f &dir ){
   if((fcentroid-palm).dot(dir) <0)
      dir=dir*-1.0;

 }

geometry_msgs::Point32 eigenToMsgPoint32(Eigen3::Vector4f v){
	geometry_msgs::Point32 p;
	p.x=v(0); p.y=v(1); p.z=v(2);
	return p;
}
geometry_msgs::Point eigenToMsgPoint(Eigen3::Vector4f v){
	geometry_msgs::Point p;
	p.x=v(0); p.y=v(1); p.z=v(2);
	return p;
}

pcl::PointXYZ eigenToPclPoint(Eigen3::Vector4f v){
   pcl::PointXYZ p;
   p.x=v(0); p.y=v(1); p.z=v(2);
   return p;
}

//find the points that are ajoining a cloud, but not in it:
//cloud: the full cloud
//cloudpts a vector of indices into cloud that represents the cluster for which we want to find near points
//centroid: the centroid of the nearby pts
//return: true if points were found within 5cm
bool findNearbyPts(pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<int> &cloudpts, Eigen3::Vector4f &centroid){
   std::vector<int> inds(cloud.size(),1); //a way of marking the points we have looked at
   // 1: not in the cluster  0: in the cluster, seen  -1: in the cluster, not seen
   std::vector<int> nearpts; //a way of marking the points we have looked at
   std::vector<int> temp;
   for(uint i=0;i<cloudpts.size(); ++i) inds[cloudpts[i]]=-1;
   for(uint i=0;i<cloudpts.size(); ++i){
      if(inds[cloudpts[i]]==-1){
         NNN(cloud,cloud.points[cloudpts[i]],temp, .02);
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

bool getNearBlobs2(pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<pcl::PointCloud<pcl::PointXYZ> > &clouds, std::vector< Eigen3::Vector4f> &nearcents ){
	pcl::PointCloud<pcl::PointXYZ> cloudout;
   pcl::PointXYZ pt,pt1,pt2; pt.x=pt.y=pt.z=0;
   std::vector<int> inds1,inds2,inds3(cloud.size(),1);
   std::vector<float> dists;
   Eigen3::Vector4f centroid1,centroid2,nearcent1;
//   bool foundarm=false;

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

   //find points near that the closest point
   NNN(cloud,pt1,inds2, .1);

   //if there is nothing near that point, we're probably seeing noise.  just give up
   if(inds2.size() < 100){
	   cout<<"very few points ";
	   return false;
   }

   //Iterate the following:
   //    find centroid of current cluster
   //    add a little height, to drive the cluster away from the arm
   //    search again around the centroid to redefine our cluster

   pcl::compute3DCentroid(cloud,inds2,centroid1);
   pt2.x=centroid1(0); pt2.y=centroid1(1); pt2.z=centroid1(2)+.01;
   NNN(cloud,pt2,inds2, .1);

   //in the middle of everything, locate where the arms is:
   std::vector<int> temp;
   NNN(cloud,pt2,temp, .15);
   //finding the arms is really reliable. we'll just throw out anytime when we can't find it.
   if(!findNearbyPts(cloud,temp,nearcent1))
      return false;

   pcl::compute3DCentroid(cloud,inds2,centroid1);
   pt2.x=centroid1(0); pt2.y=centroid1(1); pt2.z=centroid1(2);
   NNN(cloud,pt2,inds2, .1);

   //save this cluster as a separate cloud.
   getSubCloud(cloud,inds2,cloudout);

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
      cout<<"No hands detected ";
      return false;
   }

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
	   pt2.x=centroid2(0); pt2.y=centroid2(1); pt2.z=centroid2(2)+.01;
	   NNN(cloud,pt2,inds2, .1);
	   pcl::compute3DCentroid(cloud,inds2,centroid2);
	   pt2.x=centroid2(0); pt2.y=centroid2(1); pt2.z=centroid2(2);
	   NNN(cloud,pt2,inds2, .1);

	   //if too few points in the second hand, discard
	   if(inds2.size()<100) return true;

	   //check for overlapping points. if there are any, we don't want it!
	   int overlap=0;
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


namespace handdetector{

enum FingerName {THUMB,INDEXF,MIDDLEF,RINGF,PINKY,UNKNOWN};

}


class Finger{
public:
	pcl::PointCloud<pcl::PointXYZ> cloud;
	handdetector::FingerName fname;
	Eigen3::Vector4f centroid, direction;
	Finger(pcl::PointCloud<pcl::PointXYZ> &cluster, Eigen3::Vector4f &palmcenter){
		cloud=cluster;
		EIGEN_ALIGN16 Eigen3::Vector3f eigen_values;
		EIGEN_ALIGN16 Eigen3::Matrix3f eigen_vectors;
		Eigen3::Matrix3f cov;
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


class HandProcessor{
public:
    pcl::PointCloud<pcl::PointXYZ> full,digits,palm,digits2;
    vector<Finger> fingers;
    kinect_tools::Hand handmsg;
    double distfromsensor;
    Eigen3::Vector4f centroid,arm;
    int thumb;

//    HandProcessor(pcl::PointCloud<pcl::PointXYZ> &cloud){
//    	full=cloud;
//        pcl::compute3DCentroid (full, centroid);
//        distfromsensor=centroid.norm();  //because we are in the sensor's frame
//        thumb=-1;
//        handmsg.thumb=thumb;
//        handmsg.stamp=cloud.header.stamp;
//    }
    //for re-initializing a handProcessor object, so we don't have to re-instantiate
    void Init(pcl::PointCloud<pcl::PointXYZ> &cloud,Eigen3::Vector4f _arm){
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


    //filter the fingers from the palm
    void radiusFilter(int nnthresh, double tol){
       //first remove points near the arm:
       vector<int> tempinds;
       NNN(full,eigenToPclPoint(arm),tempinds,.1);
       getSubCloud(full, tempinds, full,false);



    	timeval t0=g_tick();
    	double t1,t2,t3;
       vector<int> inds,inds2,inds3;
       vector<int> searchinds;


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
          if(searchinds.size()>(500-500*distfromsensor)){
             inds.push_back(i);

             if(searchinds.size()>(550-500*distfromsensor))
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
//		printf("radius: %05d %.03f, %.03f, %.03f   ",full.points.size(),t1,t2,t3);
//       cout<<"dist:  "<<distfromsensor<<"  palm: "<<palm.points.size()<<"  digits: "<<digits.points.size()<<" "<<(575-500*distfromsensor)<<endl;
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

    void segFingers(double clustertol=.005, int mincluster=50){
       handmsg.palm.translation.x=centroid(0);
       handmsg.palm.translation.y=centroid(1);
       handmsg.palm.translation.z=centroid(2);
    	if(digits.size()==0)
    		return;
       vector< vector<int> > indclusts;
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
//            	 cout<<indclusts[i].size()<<" ("<<(fingers.back().centroid-centroid).dot(fingers.back().direction)/(fingers.back().centroid-centroid).norm()<<")  ";

       }
//       cout<<endl;
       for(uint i=0;i<fingers.size();++i)
    	   handmsg.fingers.push_back(eigenToMsgPoint(fingers[i].centroid));

    }

    void identfyFingers(){
    	if(!fingers.size()) return;
    	if(fingers.size()>2){
    		//try to identify the thumb based on distance:
		   double biggest_dist=0;
		   int farthest=0;
		   for(uint i=1;i<fingers.size();++i){
			  double dist=10;
			  for(uint j=1;j<fingers.size();++j){//find smallest distance to neighbor
				 if(i!=j && (fingers[i].centroid-fingers[j].centroid).norm() < dist)
					dist=(fingers[i].centroid-fingers[j].centroid).norm();
			  }
			  if(i==1 || dist > biggest_dist){
				 farthest=i;
				 biggest_dist=dist;
			  }
		   }
		   thumb=farthest;
    	}
    	//add a point beyond the end of the thumb to mark it:
        pcl::PointXYZ pt;
        pt.x=fingers[thumb].centroid(0)+.1*fingers[thumb].direction(0);
        pt.y=fingers[thumb].centroid(1)+.1*fingers[thumb].direction(1);
        pt.z=fingers[thumb].centroid(2)+.1*fingers[thumb].direction(2);
        digits.push_back(pt);
        digits.width++;
        handmsg.thumb=thumb;
//        handmsg.palm.rotation.x=fingers[thumb].direction(0);
//        handmsg.palm.rotation.y=fingers[thumb].direction(1);
//        handmsg.palm.rotation.z=fingers[thumb].direction(2);
//        handmsg.palm.rotation.w=0.0;

//        Eigen3::Vector4f minpt,maxpt;
//        pcl::getMinMax3D(digits,minpt,maxpt);
//        cout<<"hand size: "<<(maxpt-minpt).norm()<<" ";

    }


    void Process(){
    	timeval t0=g_tick();
    	double t1,t2,t3;
		radiusFilter(300,.02);
		t1=g_tock(t0);t0=g_tick();
		segFingers();
		t2=g_tock(t0);t0=g_tick();
	    identfyFingers();
		t3=g_tock(t0);
//		printf("process: %.03f, %.03f, %.03f",t1,t2,t3);
//		std::cout<<"process: "<<setw(6)<<t1<<setw(6)<<",  "<<t2<<",  "<<t3;
    }


};


class HandDetector
{

private:
  tf::TransformListener tl_;
  ros::NodeHandle n_;
  ros::Publisher cloudpub_[2],cloudpub2_[2],cloudpub3_,handspub_,pmappub_;
  ros::Subscriber sub_;
  std::string fixedframe;
  double currentdist;
  std::vector<pcl::PointCloud<pcl::PointXYZ> > initialclouds;
  std::vector<Eigen3::Vector4f> nearcent;
  boost::mutex cloud_mutex,hands_mutex; //for locking access to initialclouds
  boost::condition cond,cond1;  //allows threads to wait until data is available
  kinect_tools::Hands handsout;
  double ptimes[2];
  boost::thread_group threads;
  bool loopthreads;
  mapping_msgs::PolygonalMap pmap;

public:

  HandDetector(int p1=1, double p2=2.0):tl_(ros::Duration(120.0)),nearcent(2)
  {
   handspub_ = n_.advertise<kinect_tools::Hands> ("hands", 1);
   cloudpub_[0] = n_.advertise<sensor_msgs::PointCloud2> ("hand0_cloud", 1);
   cloudpub_[1] = n_.advertise<sensor_msgs::PointCloud2> ("hand1_cloud", 1);
   cloudpub2_[0] = n_.advertise<sensor_msgs::PointCloud2> ("hand0_cloud2", 1);
   cloudpub2_[1] = n_.advertise<sensor_msgs::PointCloud2> ("hand1_cloud2", 1);
   cloudpub3_ = n_.advertise<sensor_msgs::PointCloud2> ("hand_cloud3", 1);
   pmappub_ = n_.advertise<mapping_msgs::PolygonalMap> ("finger_norms", 1);
    sub_=n_.subscribe("/kinect/cloud", 1, &HandDetector::cloudcb, this);
    currentdist=0.0;
    ptimes[0]=ptimes[1]=0.0;
    loopthreads=true;
    threads.create_thread(boost::bind(&HandDetector::ProcessHand,this,0));
    //TODO: DEBUG:
    threads.create_thread(boost::bind(&HandDetector::ProcessHand,this,1));

  }

  void shutdown(){
	  {
	  	boost::mutex::scoped_lock  lock(cloud_mutex);
	  	loopthreads=false;
	  }
	  cond.notify_all();
	  threads.join_all(); //wait for all threads to return
  }

  //a loop that runs in a separate thread, just processing the hand data:
  void ProcessHand(int handnum){
	  double laststamp=0;
	  timeval t0;
	  while(true){
		  HandProcessor hp;
		  {

			 boost::mutex::scoped_lock  lock(cloud_mutex);
			 //wait until the data exists, and we have not seen it before
			 while(loopthreads && (initialclouds.size()<=handnum || initialclouds[handnum].header.stamp.toSec() <= laststamp +.001)){
//		        printf("thread %d waiting: last= %.03f: \n",handnum,laststamp);

			    cond.wait(lock);
			 }
//          printf("thread %d is processing: %d at %.03f\n",handnum,initialclouds[handnum].header.seq,initialclouds[handnum].header.stamp.toSec());

			 if(!loopthreads) return; //Cue to exit
			 t0=g_tick();
			 hp.Init(initialclouds[handnum],nearcent[handnum]);
			 laststamp=initialclouds[handnum].header.stamp.toSec();
		  }
		  hp.Process();
		  {
			boost::mutex::scoped_lock  lock(hands_mutex);
			handsout.hands.push_back(hp.handmsg);
			if(hp.digits.size()>10)
				cloudpub_[handnum].publish(hp.getDigits()); //publish the cloud here, might as well...

			//TODO: DEBUG:
         if(hp.palm.size()>10)
            cloudpub2_[handnum].publish(hp.getPalm()); //publish the cloud here, might as well...
         hp.addFingerDirs(pmap);
         //DEBUG

			ptimes[handnum]=g_tock(t0);
		  }
//		  printf("notifying main thread cloud: %d at %.03f:  handsout: %d \n",initialclouds[handnum].header.seq,initialclouds[handnum].header.stamp.toSec(),handsout.hands.size());
		  cond1.notify_one();//notify the publisher that data might be ready
	  }

  }

  void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan){
	  timeval t0=g_tick();
     sensor_msgs::PointCloud2 cloud2;
     pcl::PointCloud<pcl::PointXYZ> cloud;
     pcl::fromROSMsg(*scan,cloud);
     std::vector<Eigen3::Vector4f> arm_center;
     Eigen3::Vector4f centroid;
     int numclouds;
     bool resp;
	  {
	  	boost::mutex::scoped_lock  lock(cloud_mutex);
	  	initialclouds.clear();
	  	resp=getNearBlobs2(cloud,initialclouds,arm_center);
	  	numclouds=initialclouds.size();



	  	nearcent=arm_center;
      pmap.polygons.clear();

         for(uint i=0;i<initialclouds.size();++i){
//            //DEBUG:
//            sensor_msgs::PointCloud2 tempcloud;
//            pcl::toROSMsg(initialclouds[i],tempcloud);
//            tempcloud.header=scan->header;
//            cloudpub_[i].publish(tempcloud);

            geometry_msgs::Polygon p;
            pcl::compute3DCentroid (initialclouds[i], centroid);
            p.points.push_back(eigenToMsgPoint32(centroid));
            p.points.push_back(eigenToMsgPoint32(arm_center[i]));
            pmap.polygons.push_back(p);
         }
//         printf("cloud: %d at %.03f: %d clouds \n",scan->header.seq,scan->header.stamp.toSec(),numclouds);
      pmap.header=scan->header;
      handsout.hands.clear();
	  }
	  //DEBUG:
//	  resp=false;
     double inittime=g_tock(t0);
	  if(resp)
		  cond.notify_all();
	  else{
		  cout<<"  total time: "<<g_tock(t0)<<endl;
	      handsout.hands.clear();
	      handsout.header=scan->header;
	       handspub_.publish(handsout);
		  return;
	  }
//     if(getNearBlobs2(cloud,initialclouds) && initialclouds.size()){
//    	 cond.notify_all(); //tell the threads that they can process the data
//         timeval t1=g_tick();
//		 HandProcessor hp;
//		 hp.Init(initialclouds[0]);
//		 hp.Process();
//		 handsout.hands.push_back(hp.handmsg);
//		 ptime1=g_tock(t1);
//		 if(hp.digits.size())
//			 cloudpub_.publish(hp.getDigits());
//		 if(initialclouds.size()>1){
//			 t1=g_tick();
//			 HandProcessor hp2(initialclouds[1]);
//			 hp2.Process();
//	         handsout.hands.push_back(hp2.handmsg);
//	         ptime2=g_tock(t1);
//			 if(hp2.digits.size())
//				 cloudpub2_.publish(hp2.getDigits());
//		 }
//     }
	  {
	  	boost::mutex::scoped_lock  lock(hands_mutex);
//	  	handsout.hands.clear();
	    ptimes[0]=ptimes[1]=0.0;
	  	while(handsout.hands.size()<numclouds){
//         printf("waiting for cloud: %d at %.03f: %d clouds, have %d \n",scan->header.seq,scan->header.stamp.toSec(),numclouds,handsout.hands.size());
	  		cond1.wait(lock);  //wait until the hand processing is done
	  	}
//      printf("publishing results for cloud: %d at %.03f: %d clouds, have %d \n",scan->header.seq,scan->header.stamp.toSec(),numclouds,handsout.hands.size());
	  	handsout.header=scan->header;
	    handspub_.publish(handsout);

	      pmappub_.publish(pmap);
//	    printf("process: init: %.03f, hands: (%.03f, %.03f)    total:  %.03f ",inittime,ptimes[0],ptimes[1],g_tock(t0));
	     cout<<endl;//so the buffer gets flushed...
	  }
  }

} ;




int main(int argc, char **argv)
{
  ros::init(argc, argv, "hand_detector");
  ros::NodeHandle n;
  HandDetector detector;
  ros::spin();
  detector.shutdown(); //kill helper threads
  return 0;
}
