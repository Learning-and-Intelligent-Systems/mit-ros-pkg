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



#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosbag/message_instance.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_tools/pcl_utils.h>
#include <parallel_tools/minikernel.h>

using namespace std;

double pitch_to_apply;
std::vector<pcl::PointCloud<pcl::PointXYZRGB> > outputclouds;
std::vector<pcl::PointCloud<pcl::PointXYZRGB> > inputclouds;

void applyTrans(int i, double pitch){
   pcl::PointCloud<pcl::PointXYZRGB> cloud;
   {
      boost::mutex::scoped_lock  lock(data_mutex);
      cloud=inputclouds[i];
   }
   tf::Transform tftrans;
   tftrans.setRotation(tf::createQuaternionFromRPY(0.0,pitch,0.0));
   Eigen3::Matrix4f out;
   pcl::transformAsMatrix(tftrans,out);
   pcl::PointCloud<pcl::PointXYZRGB> cloudout;

   transformPointCloud (cloud, cloudout, out);

   boost::mutex::scoped_lock  lock(data_mutex);
   outputclouds[i]=cloudout;
}


int readBag(std::string filename, std::vector<pcl::PointCloud<pcl::PointXYZRGB> > &scans){
      rosbag::Bag bag(filename);
      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      sensor_msgs::PointCloud2 cloud2;
      //check for SurveyScans in the bag:
      rosbag::TypeQuery query("sensor_msgs/PointCloud");
      rosbag::View view(bag,query,ros::TIME_MIN,ros::TIME_MAX);
      if(view.size() > 0){
         ROS_INFO("Found %d clouds messages. Reading...",view.size());
         boost::shared_ptr <sensor_msgs::PointCloud> scan;
         BOOST_FOREACH(rosbag::MessageInstance m, view){
            if(scan = m.instantiate<sensor_msgs::PointCloud>())
               sensor_msgs::convertPointCloudToPointCloud2(*scan,cloud2);
               pcl::fromROSMsg(cloud2,cloud);
               scans.push_back(cloud);
         }
         if(scans.size() == 0){
            ROS_ERROR("Error reading scans!");
            return -2;
         }
      }
      return 0;
   }

void addScans(std::vector<pcl::PointCloud<pcl::PointXYZRGB> > &scans,pcl::PointCloud<pcl::PointXYZRGB> &out){
//     ros::NodeHandle nh_;
//     ros::Publisher pub_points2_ = nh_.advertise<sensor_msgs::PointCloud2> ("from_pcd", 100);
//     sensor_msgs::PointCloud2 cloud2;
//   Eigen3::Matrix4f final_transformation_=Eigen3::Matrix4f::Identity();
//   pcl::PointCloud<pcl::PointXYZRGB> temp;
   out=scans[0];
   out.header.frame_id="/odom";
   for(uint i=1;i<scans.size();++i){
//        final_transformation_ = alignments[i-1];// * final_transformation_;
//          transformPointCloud (scans[i], temp, final_transformation_);
     scans[i].header.frame_id="/odom";
        out+=scans[i];
//        pcl::toROSMsg(out,cloud2);
////          for(int j=0;j<3;j++){
//          cout<<"publishing..."<<endl;
//          cloud2.header.frame_id="/odom";
//          pub_points2_.publish (cloud2);
//            sleep(1);
//          }
   }
}

int main(int argc, char **argv) {
   ros::init(argc, argv, "pcd_publisher");
   ros::NodeHandle nh_;
   srand ( time(NULL) );
   if(argc<2){
      std::cout<<"USAGE "<<argv[0]<<" bagin.bag pitch"<<std::endl;
      return -1;
   }
   std::vector<Eigen3::Matrix4f> alignments;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB> > clouds;
    pcl::PointCloud<pcl::PointXYZRGB> final;
    readBag(argv[1],clouds);
//    clouds.resize(3);
    for(int i=0;i<(int)clouds.size();i++){
       if(i%5==0)
          inputclouds.push_back(clouds[i]);
    }
//    alignments.resize(clouds2.size());
//    LaunchThreads(clouds,20,alignments);
//    addScans(clouds2,final,alignments);
//    pcl::io::savePCDFile("final.pcd",final);
    std::vector<int> v1(inputclouds.size());
    std::vector<double> v2(1,atof(argv[2]));
    for(int i=0;i<(int)inputclouds.size();i++)
       v1[i]=i;

    outputclouds.resize(inputclouds.size());

    LaunchThreads(v1,v2,20,&applyTrans,minikernel::ARRAY);
    addScans(outputclouds,final);

      sensor_msgs::PointCloud2 cloud_blob;
      pcl::toROSMsg(final,cloud_blob);
    ros::Publisher pub_points2_ = nh_.advertise<sensor_msgs::PointCloud2> ("from_pcd", 100);
    for(int i=0;i<3;i++){
      cout<<"publishing..."<<endl;
      cloud_blob.header.frame_id="/odom";
      pub_points2_.publish (cloud_blob);
      sleep(1);
    }


    return 0;
}








