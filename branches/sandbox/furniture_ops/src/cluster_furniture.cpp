/*
 * cluster_furniture.cpp
 *
 *  Created on: Oct 13, 2010
 *      Author: garratt
 */
#include "ros/ros.h"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include "pcl/ModelCoefficients.h"

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/extract_indices.h"

#include "pcl/filters/passthrough.h"
#include "pcl/features/normal_3d.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_ann.h"
#include "pcl/kdtree/organized_data.h"

  typedef pcl::PointWithViewpoint Point;
#include "furniture_ops/pcl_helpers.hpp"

#include "furniture_ops/GetFurnitureClusters.h"
#include "furniture_ops/GetNearestCluster.h"
#include "furniture_ops/CombineClouds.h"






class FurnitureClusterer{


  private:
    ros::NodeHandle n_;
    ros::ServiceServer nearest_service,cluster_service,combine_service;
    ros::Publisher path_pub;
    double time_per_scan,radians_per_scan,path_radius;
    std::string worldframe;

  public:
    FurnitureClusterer()
    {
     n_.param("world_frame", worldframe, std::string("/world"));

     cluster_service = n_.advertiseService("cluster_cloud", &FurnitureClusterer::cluster_cb, this);
     nearest_service = n_.advertiseService("nearest_cluster", &FurnitureClusterer::nearest_cb, this);
     combine_service = n_.advertiseService("combine_clouds", &FurnitureClusterer::combine_cb, this);
     //publish the path to our next destination
//     path_pub = n_.advertise<geometry_msgs::PoseArray> ("/proposed_path", 1);




    }

    geometry_msgs::Pose getObjectPose(pcl::PointCloud<pcl::PointXYZ> &cloud){
        geometry_msgs::Pose current_object_pose;
        //find centroid of the cloud:
        Eigen3::Vector4f centroid;
        pcl::compute3DCentroid(cloud,centroid);
        current_object_pose.position.x=centroid.x();
        current_object_pose.position.y=centroid.y();
        current_object_pose.position.z=centroid.z();
        return current_object_pose;
    }

    void cluster(const sensor_msgs::PointCloud2 &cloudin,  std::vector<pcl::PointCloud<pcl::PointXYZ> > &clusters){
       pcl::PointCloud<Point> cloud_filtered;
       pcl::PointCloud<pcl::PointXYZ> ncloud_filtered, cloud_downsampled,cloud_nooutliers,cloud_p, cloud_no_floor,cloud_pplheight;
       pcl::fromROSMsg(cloudin,ncloud_filtered);

//       getNormals(cloud_filtered,ncloud_filtered);

        ROS_INFO ("PointCloud before segmentation: %d data points.", cloud_filtered.width * cloud_filtered.height);
//        removeOutliers(ncloud_filtered,cloud_nooutliers);
//        downSample(cloud_nooutliers,cloud_downsampled);
        segmentFloor(ncloud_filtered,cloud_no_floor);
        segmentPplHeight(cloud_no_floor,cloud_pplheight);
        downSample(cloud_pplheight,cloud_downsampled);

        segfast(cloud_downsampled,clusters);

    }

    bool nearest_cb(furniture_ops::GetNearestCluster::Request &req, furniture_ops::GetNearestCluster::Response &res){
       std::vector<pcl::PointCloud<pcl::PointXYZ> > clusters;
       cluster(req.cloud,clusters);
       pcl::PointXYZ ref;
       ref.x=req.pose.pose.position.x;
       ref.y=req.pose.pose.position.y;
       ref.z=req.pose.pose.position.z;
       pcl::PointXYZ tmp;
       double tempd,closestd=getClosestPoint(clusters[0],ref,tmp);
       int closesti=0;
       for(uint i=0;i<clusters.size();i++){
          tempd=getClosestPoint(clusters[i],ref,tmp);
          if(tempd<closestd){
             closesti=i;
             closestd=tempd;
          }
       }
       pcl::toROSMsg(clusters[closesti],res.cloud);
       return true;
    }

    bool cluster_cb(furniture_ops::GetFurnitureClusters::Request &req, furniture_ops::GetFurnitureClusters::Response &res){
       std::vector<pcl::PointCloud<pcl::PointXYZ> > clusters;
       cluster(req.cloud,clusters);
       res.clouds.resize(clusters.size());
       res.cloud_centers.resize(clusters.size());
       for(uint i=0;i<clusters.size();i++){
          res.cloud_centers[i] = getObjectPose(clusters[i]);
          pcl::toROSMsg(clusters[i],res.clouds[i]);
       }
       //TODO:  make function that converts vector of clusters to a PointCloud2

       return true;
    }

    bool combine_cb(furniture_ops::CombineClouds::Request &req, furniture_ops::CombineClouds::Response &res){
       pcl::PointCloud<pcl::PointXYZ> cloud1,cloud2;
       pcl::fromROSMsg(req.cloud1,cloud1);
       pcl::fromROSMsg(req.cloud2,cloud2);
       cloud1+=cloud2;
       pcl::toROSMsg(cloud1,res.combined_cloud);
       res.cloud_center=getObjectPose(cloud1);

       return true;
    }


};




/* ---[ */
int
  main (int argc, char** argv)
{
   ros::init(argc, argv, "furniture_clusterer");
   ros::NodeHandle n;

   FurnitureClusterer fc;
   ros::spin();

  return (0);
}
/* ]--- */


