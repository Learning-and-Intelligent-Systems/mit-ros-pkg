/*
 * cluster_furniture.cpp
 *
 *  Created on: Oct 13, 2010
 *      Author: garratt
 */
#include "ros/ros.h"

#include "pcl/segmentation/sac_segmentation.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "pcl/ModelCoefficients.h"

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include <tf/tf.h>

#include "pcl_tf/transforms.h"

#include "pcl/registration/registration.h"
#include "pcl/registration/icp.h"
#include "pcl/registration/icp_nl.h"
#include "furniture_ops/AlignClouds.h"
#include "furniture_ops/AlignClouds2.h"


  tf::Transform tfFromEigen(Eigen3::Matrix4f trans){
     btMatrix3x3 btm;
     btm.setValue(trans(0,0),trans(0,1),trans(0,2),
                trans(1,0),trans(1,1),trans(1,2),
                trans(2,0),trans(2,1),trans(2,2));
     btTransform ret;
     ret.setOrigin(btVector3(trans(0,3),trans(1,3),trans(2,3)));
     ret.setBasis(btm);
     return ret;
  }



class CloudAligner{


  private:
    ros::NodeHandle n_;
    ros::ServiceServer align_service;
    ros::Publisher path_pub;
    double time_per_scan,radians_per_scan,path_radius;
    std::string worldframe;

  public:
    CloudAligner()
    {
       align_service = n_.advertiseService("align_clouds", &CloudAligner::align_cb, this);
       align_service = n_.advertiseService("align_clouds2", &CloudAligner::align2_cb, this);

    }

    Eigen3::Matrix4f alignWithICP(pcl::PointCloud<pcl::PointXYZ> &snormcloud, pcl::PointCloud<pcl::PointXYZ> &tnormcloud){
      pcl::PointCloud<pcl::PointXYZ> aligned;
      pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
      icp.setInputCloud(snormcloud.makeShared());
      icp.setInputTarget(tnormcloud.makeShared());
      icp.setMaximumIterations (500);
      icp.setTransformationEpsilon (1e-6);
      icp.setMaxCorrespondenceDistance (0.1);
      icp.align(aligned); // is snormcloud, moved
      while(!icp.hasConverged()){
         cout<<"not converged.  "<<endl;
         icp.align(aligned);
      }
      return icp.getFinalTransformation();
    }

    bool align_cb(furniture_ops::AlignClouds::Request &req, furniture_ops::AlignClouds::Response &res){
       pcl::PointCloud<pcl::PointXYZ> pcloud1,pcloud2,pcloud1t;
       sensor_msgs::PointCloud2 cloud2;
       sensor_msgs::convertPointCloudToPointCloud2 (req.cloud1, cloud2);
       pcl::fromROSMsg(cloud2,pcloud1);
       sensor_msgs::convertPointCloudToPointCloud2 (req.cloud2, cloud2);
       pcl::fromROSMsg(cloud2,pcloud2);
       tf::Transform init_trans,finaltrans;
       tf::transformMsgToTF(req.initial_guess,init_trans);
       pcl::transformPointCloud(pcloud1,pcloud1t,init_trans);
       finaltrans=tfFromEigen(alignWithICP(pcloud1t,pcloud2));
       tf::transformTFToMsg(init_trans*finaltrans,res.transform);
       return true;
    }

    bool align2_cb(furniture_ops::AlignClouds2::Request &req, furniture_ops::AlignClouds2::Response &res){
      
       pcl::PointCloud<pcl::PointXYZ> pcloud1,pcloud2,pcloud1t;
       pcl::fromROSMsg(req.cloud1,pcloud1);
       pcl::fromROSMsg(req.cloud2,pcloud2);
       tf::Transform init_trans,finaltrans;
       tf::transformMsgToTF(req.initial_guess,init_trans);
       pcl::transformPointCloud(pcloud1,pcloud1t,init_trans);
       finaltrans=tfFromEigen(alignWithICP(pcloud1t,pcloud2));
       tf::transformTFToMsg(init_trans*finaltrans,res.transform);
       return true;
    }

};





int
  main (int argc, char** argv)
{
   ros::init(argc, argv, "cloud_aligner");
   ros::NodeHandle n;

   CloudAligner fc;
   ros::spin();

  return (0);
}



