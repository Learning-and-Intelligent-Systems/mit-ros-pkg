/*
 * cluster_furniture.cpp
 *
 *  Created on: Oct 13, 2010
 *      Author: garratt
 */
#include "ros/ros.h"

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>

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


#include "tf/transform_listener.h"
#include "furniture_ops/PlanObservationPath.h"
#include "furniture_ops/PlanCirclePath.h"

//  template <typename PointT>
//  double pt2Ddist(PointT a,PointT b){
//     return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
//  }
//
//  template <typename PointT>
//  double pt3Ddist(PointT a,PointT b){
//     return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z));
//  }

  double ptDist(geometry_msgs::Pose a,geometry_msgs::Pose b){
     return sqrt((a.position.x-b.position.x)*(a.position.x-b.position.x)+
           (a.position.y-b.position.y)*(a.position.y-b.position.y)+
           (a.position.z-b.position.z)*(a.position.z-b.position.z));
  }

  double ptAngle(geometry_msgs::Pose a,geometry_msgs::Pose b){
     return atan2((a.position.y-b.position.y),(a.position.x-b.position.x));
  }

  double pt2DDist(geometry_msgs::Pose a,geometry_msgs::Pose b){
     return sqrt((a.position.x-b.position.x)*(a.position.x-b.position.x)+
           (a.position.y-b.position.y)*(a.position.y-b.position.y));
  }


class CirclePlanner{


  private:
     tf::TransformListener listener;
    ros::NodeHandle n_;
    ros::ServiceServer planner_service,circle_service;
    std::string worldframe;

  public:
    CirclePlanner():n_("~"),listener(ros::Duration(20.0))
    {
     n_.param("world_frame", worldframe, std::string("/world"));
     planner_service = n_.advertiseService("/plan_observation_path", &CirclePlanner::service_cb, this);
     circle_service = n_.advertiseService("/plan_circle", &CirclePlanner::circle_cb, this);
    }

    geometry_msgs::Pose getCurrentPose(){
       tf::StampedTransform transform;
       ros::Time t=ros::Time::now();
       ROS_DEBUG("transforming %s to base_link now in CirclePlanner::getCurrentPose()",worldframe.c_str());
       listener.waitForTransform(worldframe,"base_link",t,ros::Duration(.5));
       listener.lookupTransform(worldframe,"base_link",t,transform);
       geometry_msgs::TransformStamped ts;
       tf::transformStampedTFToMsg(transform,ts);
       geometry_msgs::Pose p;
       p.position.x=ts.transform.translation.x;
       p.position.y=ts.transform.translation.y;
       p.position.z=ts.transform.translation.z;
       p.orientation=ts.transform.rotation;
       return p;
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

    double getClosestdist(pcl::PointCloud<pcl::PointXYZ> &cloud, geometry_msgs::Pose current_pose){
           geometry_msgs::Pose closest;
           pcl::PointXYZ pt,ref(current_pose.position.x,current_pose.position.y,current_pose.position.z);
           return getClosestPoint(cloud,ref,pt);
//           closest.position.x=pt.x;
//           closest.position.y=pt.y;
//           closest.position.z=pt.z;
//           return closest;
        }

    bool service_cb(furniture_ops::PlanObservationPath::Request &req, furniture_ops::PlanObservationPath::Response &res){
       //first order: just go in a circle around object
       //go radians_per_scan around object, trying to keep path_radius meters away
       //plan in .1 radian (~5 degree) incriments
       //this all assumes that we have a holonomic base
       //the path starts at the current position, and moves to the target radius and angle linearly
       geometry_msgs::Pose current_object_pose, current_pose;

       //convert cloud to form we like:
       pcl::PointCloud<pcl::PointXYZ> cloud;
       pcl::fromROSMsg(req.object_cloud,cloud);
       current_pose=getCurrentPose();
       current_object_pose = getObjectPose(cloud);

       double current_radius=pt2DDist(current_pose,current_object_pose),radius;
       double current_angle=ptAngle(current_pose,current_object_pose),angle;
       double radians_per_scan=req.angle;
       //find estimated end pose:
       geometry_msgs::Pose estimated;
       estimated.position.x=req.radius*cos(radians_per_scan+current_angle)+current_object_pose.position.x;
       estimated.position.y=req.radius*sin(radians_per_scan+current_angle)+current_object_pose.position.y;
       estimated.position.z=current_object_pose.position.z;

       //now find out how close the object is to that position:
       double objdist=getClosestdist(cloud,estimated);
       //set the radius of our path to make us go req.radius away from this point:
       double path_radius=2*req.radius - objdist;

       double radius_incriment=(path_radius-current_radius)/radians_per_scan;

       std::cout<<"planPath: current_radius =  "<<current_radius<<std::endl;
       std::cout<<"planPath: current_angle =  "<<current_angle<<std::endl;
       std::cout<<"planPath: current_object_pose =  "<<current_object_pose<<std::endl;
       std::cout<<"planPath: current_pose =  "<<current_pose<<std::endl;

       geometry_msgs::Pose p;
       for(double i=0; i< radians_per_scan+.1; i+=.1){
          angle=i+current_angle;
          radius=current_radius+radius_incriment*i;
          p.position.x=radius*cos(angle)+current_object_pose.position.x;
          p.position.y=radius*sin(angle)+current_object_pose.position.y;
          p.orientation = tf::createQuaternionMsgFromYaw(angle+3.1415);
          res.path.poses.push_back(p);
          std::cout<<"planPath: increment =  "<<i<<"  angle "<<angle<<"  radius "<<radius<<"  p.position "<<p.position.x<<", "<<p.position.y<<std::endl;
       }
       //set header:
       res.path.header.stamp=ros::Time::now();
       res.path.header.frame_id=worldframe;

       return true;
    }

    //just go in a circle at a fixed radius
    bool circle_cb(furniture_ops::PlanCirclePath::Request &req, furniture_ops::PlanCirclePath::Response &res){
       geometry_msgs::Pose current_object_pose = req.object_pose;
       geometry_msgs::Pose current_pose=getCurrentPose();
       double radians_per_scan=req.angle;
       double path_radius=req.radius;
       double current_radius=ptDist(current_pose,current_object_pose);
        double current_angle=ptAngle(current_pose,current_object_pose);
        double radius_incriment=(path_radius-current_radius)/radians_per_scan;

        std::cout<<"planPath: current_radius =  "<<current_radius<<std::endl;
        std::cout<<"planPath: current_angle =  "<<current_angle<<std::endl;
        std::cout<<"planPath: current_object_pose =  "<<current_object_pose<<std::endl;
        std::cout<<"planPath: current_pose =  "<<current_pose<<std::endl;

        geometry_msgs::Pose p;
        double radius,angle;
        for(double i=0; i< radians_per_scan+.1; i+=.1){
           angle=i+current_angle;
           radius=current_radius+radius_incriment*i;
           p.position.x=radius*cos(angle)+current_object_pose.position.x;
           p.position.y=radius*sin(angle)+current_object_pose.position.y;
           p.orientation = tf::createQuaternionMsgFromYaw(angle+3.1415);
           res.path.poses.push_back(p);
           std::cout<<"planPath: increment =  "<<i<<"  angle "<<angle<<"  radius "<<radius<<"  p.position "<<p.position.x<<", "<<p.position.y<<std::endl;
        }
        //set header:
        res.path.header.stamp=ros::Time::now();
        res.path.header.frame_id=worldframe;

        return true;
    }




};




/* ---[ */
int
  main (int argc, char** argv)
{
   ros::init(argc, argv, "circle_planner");
   ros::NodeHandle n;

   CirclePlanner node;
   ros::spin();

  return (0);
}
/* ]--- */


