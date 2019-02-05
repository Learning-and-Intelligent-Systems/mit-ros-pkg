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
#include <boost/foreach.hpp>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include "pcl/ModelCoefficients.h"

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/registration/registration.h"
#include "pcl/registration/icp.h"
#include <sys/time.h>
//#include "pcl_tf/transforms.h"
//
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/extract_indices.h"
//#include "pcl_tf/transforms.h"

#include "pcl_ros/transforms.h"

#include "pcl/filters/passthrough.h"
#include "pcl/features/normal_3d.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/kdtree/kdtree.h"
//#include "pcl/kdtree/kdtree_ann.h"
#include "pcl/kdtree/organized_data.h"
#include <list>
#include <fstream>
#include <vector>

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

  tf::Transform tfFromEigen(Eigen::Matrix4f trans){
   btMatrix3x3 btm;
   btm.setValue(trans(0,0),trans(0,1),trans(0,2),
              trans(1,0),trans(1,1),trans(1,2),
              trans(2,0),trans(2,1),trans(2,2));
   btTransform ret;
   ret.setOrigin(btVector3(trans(0,3),trans(1,3),trans(2,3)));
   ret.setBasis(btm);
   return ret;
  }


  std::vector<Eigen::Matrix4f> alignments;
  boost::mutex data_mutex,print_mutex;


  template <typename PointT>
  int getClosestPoints(pcl::PointCloud<PointT> &ref_cloud, pcl::PointCloud<PointT> &target_cloud, std::vector<int> &ref_pts, std::vector<int> &tgt_pts, double dist_thresh=.2, uint num_pts=500){
     timeval t0=g_tick();
     pcl::KdTreeFLANN<PointT> ttree;
     ttree.setInputCloud(target_cloud.makeShared());
  //   ROS_INFO("step 1: initialization  %f secs. ",g_tock(t0));
     t0=g_tick();
     srand(getUsec());
     std::vector<int> indices(1);
     std::vector<float> dists(1);
     ref_pts.clear();
     tgt_pts.clear();
     int ind;

     std::vector<int> notused(ref_cloud.points.size(),1);
     for(uint i=0;i<ref_cloud.points.size();i++)
        notused[i]=i;
     long tcount=0;
     int indsleft=ref_cloud.points.size();
     while(ref_pts.size() < num_pts && indsleft > 10){
        ind=rand() %indsleft;
        if(ttree.nearestKSearch(ref_cloud.points[notused[ind]],1,indices,dists) && dists[0] < dist_thresh)
              if(fabs(ref_cloud.points[notused[ind]].z - target_cloud.points[indices[0]].z) < dist_thresh ){
           ref_pts.push_back(notused[ind]);
           tgt_pts.push_back(indices[0]);
        }
        notused[ind]=notused[indsleft-1];
        tcount++;
        indsleft--;
     }
  //   ROS_INFO("finding Closest Points took:  %f secs. found %d pts from %d guesses out of %d pts",g_tock(t0),tgt_pts.size(),tcount,target_cloud.points.size());
  //   ROS_INFO(" %f secs pre guess",g_tock(t0)/(double)tcount);
     return 0;
  }



  template <typename PointT>
  Eigen::Matrix4f icp3Dt(pcl::PointCloud<PointT> &c1, pcl::PointCloud<PointT> &c2, double max_dist,
        int small_transdiff_countreq=1, int num_pts=500, uint min_pts=10, uint max_iter=50, float transdiff_thresh=.0001 ){
     timeval t0=g_tick(),t1=g_tick();
     std::vector<int> c1pts,c2pts;
     Eigen::Matrix4f transformation_, final_transformation_=Eigen::Matrix4f::Identity(),previous_transformation_,trans2d;
     int small_transdiff_count=0;
     for(uint i=0;i<max_iter;i++){
        t0=g_tick();
        getClosestPoints(c1,c2,c1pts,c2pts,max_dist,num_pts);
        if(c1pts.size() < min_pts){
           ROS_ERROR("not enough correspondences");
           return transformation_;
        }
        previous_transformation_ = final_transformation_;
        pcl::estimateRigidTransformationSVD(c1,c1pts,c2,c2pts,transformation_);
        // Tranform the data

        transformPointCloud (c1, c1, transformation_);
        // Obtain the final transformation
        final_transformation_ = transformation_ * final_transformation_;
  //      trans2d=projectTo2D(final_transformation_);
  //      final_transformation_=trans2d;
        float transdiff=fabs ((final_transformation_ - previous_transformation_).sum ());
//        ROS_INFO("icp took:  %f secs. trans diff: %f,  ",g_tock(t0),transdiff);
        if(transdiff <transdiff_thresh) small_transdiff_count++;
        else small_transdiff_count=0;
        if(small_transdiff_count>small_transdiff_countreq)
           break;
     }
     {
         boost::mutex::scoped_lock  lock(print_mutex);
     ROS_INFO("icp took:  %f secs. ",g_tock(t1));
     }
//     std::cout<<final_transformation_<<std::std::endl;
     return final_transformation_;
  }






  struct jobtype{
	  int jobnum;
     pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
  };

  //IndexableJob is a collection of vectors that
  class IndexableJob{
     std::vector< pcl::PointCloud<pcl::PointXYZ> > clouds;
     std::vector<double> tolerances;

  public:
     IndexableJob(std::vector< pcl::PointCloud<pcl::PointXYZ> > &c){
        clouds=c;
     }
     int getTotalJobs(){
        return clouds.size()-1;
     }

     void getJobStruct(int i, jobtype &j){
    	 j.jobnum=i;
        j.cloud1=clouds[i];
        j.cloud2=clouds[i+1];
     }

  };

  class JobKernel{
     boost::mutex job_mutex;
     IndexableJob *job;
     int nextjob;
     int totaljobs;
  public:
     JobKernel(IndexableJob *_job){
        job=_job;
        totaljobs=job->getTotalJobs();
        nextjob=0;
     }

     int getNextJob(jobtype &j){
        boost::mutex::scoped_lock  lock(job_mutex);
        if(nextjob<totaljobs)
           job->getJobStruct(nextjob++,j);
        else
           return 1;
        return 0;
     }
  };


  void alignclouds(pcl::PointCloud<pcl::PointXYZ> c1, pcl::PointCloud<pcl::PointXYZ> c2, int index){
		Eigen::Matrix4f trans1 = icp3Dt(c1,c2,.05,3,500);
		{
            boost::mutex::scoped_lock  lock(data_mutex);
            alignments[index]=trans1;
		}
  }



  void jobthread(JobKernel *kernel){
     jobtype j;
     while(kernel->getNextJob(j)==0){
        alignclouds(j.cloud1,j.cloud2,j.jobnum);
     }
  }


  void LaunchThreads(std::vector<pcl::PointCloud<pcl::PointXYZ> > &clouds, int numthreads){
     IndexableJob ij(clouds);
     JobKernel kernel(&ij);
     boost::thread_group threads;
     for(int i=0;i<numthreads;i++){
        {
            boost::mutex::scoped_lock  lock(print_mutex);
  //          ROS_INFO("Creating thread %d",i);
        }
        threads.create_thread(boost::bind(&jobthread,&kernel));
     }
     threads.join_all();
  }
















  void writePLY(pcl::PointCloud<pcl::PointXYZ> &cloud, std::string filename){
     std::ofstream outf;
     outf.open(filename.c_str(),std::ios::out);
     outf<<"ply\n"<<"format ascii 1.0\n"<<"element vertex "<<cloud.points.size()<<std::endl;
     outf<<"property float x\nproperty float y\nproperty float z\n";
     outf<<"element face 0\nproperty list uchar int vertex_indices\nend_header\n";
     for (int i = 0; i < cloud.points.size(); ++i) {
        outf<<cloud.points[i].x<<" "<<cloud.points[i].y<<" "<<cloud.points[i].z<<std::endl;
     }
     outf.close();

  }



int readBag(std::string filename, std::vector<pcl::PointCloud<pcl::PointXYZ> > &scans){
		rosbag::Bag bag(filename);
		pcl::PointCloud<pcl::PointXYZ> cloud;
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


void addScans(std::vector<pcl::PointCloud<pcl::PointXYZ> > &scans,pcl::PointCloud<pcl::PointXYZ> &out ){
	Eigen::Matrix4f final_transformation_=Eigen::Matrix4f::Identity();
	pcl::PointCloud<pcl::PointXYZ> temp;
	out=scans[0];
	for(uint i=1;i<scans.size();++i){
		final_transformation_ = alignments[i-1] * final_transformation_;
        transformPointCloud (scans[i], temp, final_transformation_);

        out+=temp;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "align_clouds");
	ros::NodeHandle nh_;

	std::vector<pcl::PointCloud<pcl::PointXYZ> > scans,scans2;
	pcl::PointCloud<pcl::PointXYZ> aligned;
	readBag("kinect_prelim_2010-11-15-22-03-58.bag",scans);
	alignments.resize(scans.size());



//	cout<<scans[0].size()<<endl;
//
//	scans2.assign(scans.begin(),scans.begin()+100);
//	LaunchThreads(scans2,8);
//	addScans(scans2,aligned);
	sensor_msgs::PointCloud2 cloud_blob;
	pcl::toROSMsg(scans[0],cloud_blob);
	writePLY(scans[50],"face50.ply");
	 ros::Publisher pub_points2_ = nh_.advertise<sensor_msgs::PointCloud2> ("from_pcd", 100);
	 for(int i=0;i<3;i++){
	   std::cout<<"publishing..."<<std::endl;
		 cloud_blob.header.frame_id="/kinect_depth";
		 cloud_blob.header.stamp=ros::Time::now();
		 pub_points2_.publish (cloud_blob);
		 sleep(1);
	 }

	return 0;
}









