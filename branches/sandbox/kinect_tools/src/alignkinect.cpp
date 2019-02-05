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

#include <sensor_msgs/point_cloud_conversion.h>
#include <kinect_tools/CloudWithViewpoint.h>


//a modified version of the main scan analyzer:


class ScanAnalyzer {
	std::vector<kinect_tools::CloudWithViewpoint> scans;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB> > clouds;
	std::vector<Eigen3::Matrix4f> alignments;
	ros::NodeHandle nh_;
	ros::Publisher pub_cloud;

public:
	ScanAnalyzer(std::string filename){
		pub_cloud = nh_.advertise<sensor_msgs::PointCloud2> ("scan_cloud", 100);
		timeval t0=g_tick();
		if(readBag(filename)){
			std::cerr<<"could not load file"<<std::endl;
			exit(-1);
		}
		std::cout<<"read in "<<scans.size()<<" clouds in "<<g_tock(t0)<<std::endl;
		convertToClouds();  //unpack the rosmsgs, generate normals and filter if needed.
		std::cout<<"converted clouds"<<std::endl;

	}

	void convertToClouds(){
		if(!scans.size())
			ROS_WARN("No scans in array!  Maybe you need to load a bagfile?");
		alignments.resize(scans.size());
		clouds.resize(scans.size());
		tf::Transform temp;
		for(uint i=0;i<scans.size();++i){
			pcl::fromROSMsg(scans[i].cloud,clouds[i]);
			tf::transformMsgToTF(scans[i].transform,temp);
			pcl::transformAsMatrix(temp,alignments[i]);
			pcl::transformPointCloud(clouds[i],clouds[i],alignments[i]);
		}
	}

	void convertCloud(kinect_tools::CloudWithViewpoint &cwv){
		tf::Transform tftemp;
		Eigen3::Matrix4f mtemp;
		pcl::PointCloud<pcl::PointXYZRGB> ctemp;
		pcl::fromROSMsg(cwv.cloud,ctemp);
		tf::transformMsgToTF(cwv.transform,tftemp);
		pcl::transformAsMatrix(tftemp,mtemp);
		if(ctemp.header.frame_id.compare("/odom"))  //if cloud is still in local frame:
			pcl::transformPointCloud(ctemp,ctemp,mtemp);
		ctemp.header.frame_id="/odom";
	}

	void writeBag(std::string filename){
		rosbag::Bag bag(filename,rosbag::bagmode::Write);
		for(uint i=0; i<scans.size();i++)
			bag.write("analyzed_scans",ros::Time::now(),scans[i]);
		bag.close();
	}

	double tdiff(kinect_tools::CloudWithViewpoint &c1, kinect_tools::CloudWithViewpoint &c2){
		return (c2.cloud.header.stamp-c1.cloud.header.stamp).toSec();
	}

	//Checks for both ScanAnalysis and SurveyScan messages
	int readBag(std::string filename){
		rosbag::Bag bag(filename);

		//check for SurveyScans in the bag:
		rosbag::TypeQuery query("kinect_tools/CloudWithViewpoint");
		rosbag::View view(bag,query,ros::TIME_MIN,ros::TIME_MAX);
		if(view.size() > 0){
			ROS_INFO("Found %d ScanAnalysis messages. Reading...",view.size());
			boost::shared_ptr <kinect_tools::CloudWithViewpoint> scan;
			BOOST_FOREACH(rosbag::MessageInstance m, view){
				if(scan = m.instantiate<kinect_tools::CloudWithViewpoint>()){
					cout<<"scan: "<<scan->cloud.header.stamp.toSec()<<std::endl;
					if(!scans.size() || tdiff(scans.back(),*scan) > .5)
						scans.push_back(*scan);

				}
			}
		}
		if(scans.size() == 0){
			std::cerr<<"no scans read."<<std::endl;
			return -2;
		}
		return 0;
	}


	void publishSummary(uint interval){
		pcl::PointCloud<pcl::PointXYZRGB> scloud;
		scloud=clouds[0];
		sensor_msgs::PointCloud2 cloudout;
		sensor_msgs::PointCloud cloudout1;
		pcl::toROSMsg(scloud,cloudout);
		sensor_msgs::convertPointCloud2ToPointCloud(cloudout,cloudout1);
		std::cout<<"original:"<<std::endl;
		for(uint i=0; i<clouds[0].cloud.channels.size();i+=interval)
			std::cout<<cloudout1.channels[i].name<<"  "<<cloudout1.channels[i].values[0]<<std::endl;

		std::cout<<"final:"<<std::endl;
		for(uint i=0; i<cloudout1.channels.size();i+=interval)
			std::cout<<cloudout1.channels[i].name<<"  "<<cloudout1.channels[i].values[0]<<std::endl;
		for(uint i=1; i<clouds.size();i+=interval)
			scloud+=clouds[i];
		pcl::toROSMsg(scloud,cloudout);
		cloudout.header.frame_id="/odom";
		for(int i=0;i<3;i++){
			pub_cloud.publish(cloudout);
			sleep(1.0);
		}
	}



};







//
//  void alignclouds(pcl::PointCloud<pcl::PointXYZ> c1, pcl::PointCloud<pcl::PointXYZ> c2, Eigen3::Matrix4f &alignment){
//      Eigen3::Matrix4f trans1 = icp2D(c1,c2,.05,2,500,10,50,.0002);
//      {
//            boost::mutex::scoped_lock  lock(data_mutex);
//            alignment=trans1;
//      }
//  }




//  void addScans(std::vector<pcl::PointCloud<pcl::PointXYZ> > &scans,pcl::PointCloud<pcl::PointXYZ> &out, std::vector<Eigen3::Matrix4f> &alignments){
//       ros::NodeHandle nh_;
//       ros::Publisher pub_points2_ = nh_.advertise<sensor_msgs::PointCloud2> ("from_pcd", 100);
//       sensor_msgs::PointCloud2 cloud2;
//     Eigen3::Matrix4f final_transformation_=Eigen3::Matrix4f::Identity();
//     pcl::PointCloud<pcl::PointXYZ> temp;
//     out=scans[0];
//     out.header.frame_id="/odom";
//     for(uint i=1;i<scans.size();++i){
////        final_transformation_ = alignments[i-1];// * final_transformation_;
////          transformPointCloud (scans[i], temp, final_transformation_);
//    	 scans[i].header.frame_id="/odom";
//          out+=scans[i];
//          pcl::toROSMsg(out,cloud2);
////          for(int j=0;j<3;j++){
//            cout<<"publishing..."<<endl;
//            cloud2.header.frame_id="/odom";
//            pub_points2_.publish (cloud2);
////            sleep(1);
////          }
//     }
//  }






//int readBag(std::string filename, std::vector<pcl::PointCloud<pcl::PointXYZ> > &scans){
//      rosbag::Bag bag(filename);
//      pcl::PointCloud<pcl::PointXYZ> cloud;
//      sensor_msgs::PointCloud2 cloud2;
//      //check for SurveyScans in the bag:
//      rosbag::TypeQuery query("sensor_msgs/PointCloud");
//      rosbag::View view(bag,query,ros::TIME_MIN,ros::TIME_MAX);
//      if(view.size() > 0){
//         ROS_INFO("Found %d clouds messages. Reading...",view.size());
//         boost::shared_ptr <sensor_msgs::PointCloud> scan;
//         BOOST_FOREACH(rosbag::MessageInstance m, view){
//            if(scan = m.instantiate<sensor_msgs::PointCloud>())
//               sensor_msgs::convertPointCloudToPointCloud2(*scan,cloud2);
//               pcl::fromROSMsg(cloud2,cloud);
//               scans.push_back(cloud);
//         }
//         if(scans.size() == 0){
//            ROS_ERROR("Error reading scans!");
//            return -2;
//         }
//      }
//      return 0;
//   }

int main(int argc, char **argv) {
   ros::init(argc, argv, "kinect_publisher");
//   srand ( time(NULL) );
   if(argc<2){
      std::cout<<"USAGE "<<argv[0]<<" bagin.bag bagout.bag"<<std::endl;
      return -1;
   }
   ScanAnalyzer sa(argv[1]);
   sa.publishSummary(2);

//   std::vector<Eigen3::Matrix4f> alignments;
//    std::vector<pcl::PointCloud<pcl::PointXYZ> > clouds,clouds2;
//    pcl::PointCloud<pcl::PointXYZ> final;
//    readBag(argv[1],clouds);
////    clouds.resize(3);
//    for(int i=0;i<(int)clouds.size();i++){
//       if(i%10==0)
//          clouds2.push_back(clouds[i]);
//    }
//    alignments.resize(clouds2.size());
////    LaunchThreads(clouds,20,alignments);
//    addScans(clouds2,final,alignments);
//    pcl::io::savePCDFile("final.pcd",final);

    return 0;
}
