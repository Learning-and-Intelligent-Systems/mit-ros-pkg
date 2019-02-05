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

template <typename PointT>
void getSubCloud(pcl::PointCloud<PointT> &cloudin,  std::vector<int> &ind, pcl::PointCloud<PointT> &cloudout,bool use_positive=true){
   pcl::ExtractIndices<PointT> extract;
   // Extract the inliers
   extract.setInputCloud(cloudin.makeShared());
   extract.setIndices (boost::make_shared<std::vector<int> > (ind));
   extract.setNegative (!use_positive);
   extract.filter (cloudout);
//    ROS_INFO ("Subcloud representing the planar component: %d data points.", cloudout.width * cloudout.height);

}

template <typename PointT>
void mysegmentHeight(pcl::PointCloud<PointT> &cloudin, pcl::PointCloud<PointT> &cloudout,double lowest,double highest, bool use_positive=true){
   timeval t0=g_tick();
   std::vector<int> ind;
   for(uint i=0;i<cloudin.points.size(); i++){
      if(cloudin.points[i].y > lowest && cloudin.points[i].y < highest
    		  && cloudin.points[i].z<3.0 //z points out fo the camera

      ){ //|| cloudin.points[i].z <.1){
         ind.push_back(i);
      }
   }
   getSubCloud(cloudin,ind,cloudout,use_positive);
//   std::cout<<"segmentHeight took:  "<<g_tock(t0)<<"  for "<<cloudout.points.size()<<" indices."<<std::endl;
}


void getBiggestCluster(pcl::PointCloud<pcl::PointXYZ> &cloudin, pcl::PointCloud<pcl::PointXYZ> &cloudout){
   std::vector<std::vector<int> > clusters;
   extractEuclideanClustersFast2(cloudin, clusters, .4, 30);
   int biggest=0; uint biggestval=0;
   for(uint i=0;i<clusters.size(); i++){
//      std::cout<<"cluster "<<i<<"  has "<<clusters[i].size()<<std::endl;
      if(clusters[i].size()>biggestval){
         biggest=i;
         biggestval=clusters[i].size();
      }
   }

   std::cout<<"chose cluster "<<biggest<<"  with "<<clusters[biggest].size()<<std::endl;
   getSubCloud(cloudin,clusters[biggest],cloudout,true);

}



//
////find the points that are ajoining a cloud, but not in it:
////cloud: the full cloud
////cloudpts a vector of indices into cloud that represents the cluster for which we want to find near points
////centroid: the centroid of the nearby pts
////return: true if points were found within 5cm
//bool findNearbyPts(pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<int> &cloudpts, Eigen::Vector4f &centroid){
//   std::vector<int> inds(cloud.size(),1); //a way of marking the points we have looked at
//   // 1: not in the cluster  0: in the cluster, seen  -1: in the cluster, not seen
//   std::vector<int> nearpts; //a way of marking the points we have looked at
//   std::vector<int> temp;
//   for(uint i=0;i<cloudpts.size(); ++i) inds[cloudpts[i]]=-1;
//   for(uint i=0;i<cloudpts.size(); ++i){
//      if(inds[cloudpts[i]]==-1){
//         NNN(cloud,cloud.points[cloudpts[i]],temp, .05);
//               mapping_msgs::PolygonalMap pmap;
//               geometry_msgs::Polygon p;
//         for(uint j=0;j<temp.size(); ++j){
//            if(inds[temp[j]]==1){
//               nearpts.push_back(temp[j]);
//               inds[temp[j]]=2;
//            }
//            else
//               inds[temp[j]]=-2;
//         }
//      }
//   }
//   //TODO: check if we are really just seeing the other hand:
//   //       remove any points that do not have a point w/in 1cm
//   if(nearpts.size())
//   //now find the centroid of the nearcloud:
//      pcl::compute3DCentroid(cloud,nearpts,centroid);
//   else
//      return false;
//   return true;
//}

//, std::vector< Eigen::Vector4f> &nearcents std::vector<pcl::PointCloud<pcl::PointXYZ> > &clouds
bool getNearBlobs2(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &cloudout){
//	pcl::PointCloud<pcl::PointXYZ> cloudout;
   pcl::PointXYZ pt,pt1,pt2; pt.x=pt.y=pt.z=0;
   std::vector<int> inds1,inds2,inds3(cloud.size(),1);
   std::vector<float> dists;
   Eigen::Vector4f centroid1,centroid2,nearcent1;

   //find closest pt to camera:
   NNN(cloud,pt,inds1,dists, 2.0);
   int ind=0; double smallestdist;
   for(uint i=0;i<dists.size(); ++i){
      if(dists[i]<smallestdist || i==0 ){
         ind=inds1[i];
         smallestdist=dists[i];
      }
   }
//   smallestdist=sqrt(smallestdist);
   pt1=cloud.points[ind];


   //find points near that the closest point
   NNN(cloud,pt1,inds2, 1.0);
   getSubCloud(cloud,inds2,cloudout);
//   clouds.push_back(cloudout);

 return true;

}

bool updateLoc(pcl::PointXYZ &pt, pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &cloudout){
   std::vector<int> inds1;
   Eigen::Vector4f centroid;

//   //find closest pt to last loc:
//   NNN(cloud,pt,inds1,dists, 2.0);
//   int ind=0; double smallestdist;
//   for(uint i=0;i<dists.size(); ++i){
//      if(dists[i]<smallestdist || i==0 ){
//         ind=inds1[i];
//         smallestdist=dists[i];
//      }
//   }
////   smallestdist=sqrt(smallestdist);
//   pt1=cloud.points[ind];

   //find points near the last location:
   NNN(cloud,pt,inds1, 0.20);
   if(inds1.size()==0)
	   return false;
   pcl::compute3DCentroid(cloud,inds1,centroid);
   pt=eigenToPclPoint(centroid);
//    printf("  (%.02f %.02f %.02f) %05d ",pt.x, pt.y,pt.z,(int)inds1.size());

    getSubCloud(cloud,inds1,cloudout);
   return true;
}



//strategy:
/* transform cloud
 * remove floor
 * if not first:
 *   if object near last
 *     recenter, update filter
 *   else
 *     look for object to track
 *
 *
 */






class LegDetector
{

private:
  ros::NodeHandle n_;
  ros::Publisher cloudpub_[2],handspub_,cmdpub_;
  ros::Subscriber sub_;
  std::string fixedframe;
  pcl::PointXYZ objectloc;
  int lastseen;

public:

  LegDetector()
  {
   cloudpub_[0] = n_.advertise<sensor_msgs::PointCloud2> ("cloudout1", 1);
   cmdpub_ = n_.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
   cloudpub_[1] = n_.advertise<sensor_msgs::PointCloud2> ("cloutout2", 1);
    sub_=n_.subscribe("/camera/rgb/points", 1, &LegDetector::cloudcb, this);
    objectloc.z=1.0;
    lastseen=5;
  }

  void stop(){
    //publish stop command
  std::cout<<"stop"<<std::endl;
	  geometry_msgs::Twist cmd;
	  cmdpub_.publish(cmd);
  }

  void generateCommand(){
	  //z is dist in front -> x
	  //x is dist to side -> ang.z
	  geometry_msgs::Twist cmd;
	  cmd.linear.x = objectloc.z-.8;

	  cmd.angular.z = -2.0*objectloc.x*objectloc.z;
   printf(" %.02f %.02f \n",cmd.linear.x, cmd.angular.z);
	  cmdpub_.publish(cmd);

  }


  void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan){
	  timeval t0=g_tick();
     sensor_msgs::PointCloud2 cloud2;
     pcl::PointCloud<pcl::PointXYZ> cloud,cloudmid,cloudout;
     pcl::fromROSMsg(*scan,cloud);
     std::vector<Eigen::Vector4f> arm_center;
	   std::vector<pcl::PointCloud<pcl::PointXYZ> > initialclouds;
//       std::cout<<"pre: "<<g_tock(t0)<<"  ";
//      std::vector<std::vector<int> > clusters;

//      std::cout<<" cloud size  "<<cloud.points.size()<<"  ";
//      extractEuclideanClustersFast2(cloud, clusters, .2, 30);

      mysegmentHeight(cloud,cloudmid,-1,.30,true);
      if(cloudmid.points.size()==0) return;
      //assume we already found our object:
      if(!updateLoc(objectloc,cloudmid,cloudout)){
	  lastseen++;
	  if(lastseen>5)
	    stop();
    	  return;
      }
      lastseen=0;
      generateCommand();
//      getBiggestCluster(cloudmid,cloudout);

//      getNearBlobs2(cloud, cloudout);
      pcl::toROSMsg(cloudout,cloud2);
      cloud2.header=scan->header;

      cloudpub_[0].publish(cloud2);
//      std::cout<<" cloud size  "<<cloud.points.size()<<"  ";
//       std::cout<<" total time:  "<<g_tock(t0)<<std::endl;
  }

} ;




int main(int argc, char **argv)
{
  ros::init(argc, argv, "hand_detector");
  ros::NodeHandle n;
  LegDetector detector;
  ros::spin();
  detector.stop();
  return 0;
}
