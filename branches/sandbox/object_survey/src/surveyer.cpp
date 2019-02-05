/*
 * surveyer.cpp
 *
 *  Created on: Oct 8, 2010
 *      Author: garratt
 */
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "geometry_msgs/PoseArray.h"
#include "polled_camera/GetPolledImage.h"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_assembler/AssembleScans.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

#include <simple_controller/SimpleMoveAction.h>
#include <actionlib/client/simple_action_client.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include "furniture_ops/PlanObservationPath.h"
#include "furniture_ops/PlanCirclePath.h"
#include "furniture_ops/GetFurnitureClusters.h"
#include "furniture_ops/GetNearestCluster.h"
#include "furniture_ops/CombineClouds.h"
#include <object_survey/SurveyResults.h>
#include <object_survey/CameraScan.h>
#include <object_survey/GetCameraScan.h>


int addViewPoint(sensor_msgs::PointCloud &cloud, tf::StampedTransform transform,sensor_msgs::PointCloud2 &output){
   sensor_msgs::ChannelFloat32 v1;
   v1.name="vp_x";
   v1.values=std::vector<float>(cloud.points.size(),transform.getOrigin().x());
   cloud.channels.push_back(v1);
   v1.name="vp_y";
   v1.values=std::vector<float>(cloud.points.size(),transform.getOrigin().y());
   cloud.channels.push_back(v1);
   v1.name="vp_z";
   v1.values=std::vector<float>(cloud.points.size(),transform.getOrigin().z());
   cloud.channels.push_back(v1);

   // Convert to the new point cloud format
   if (!sensor_msgs::convertPointCloudToPointCloud2 (cloud, output))
   {
     ROS_ERROR ("[cloud_grabber] Conversion from sensor_msgs::PointCloud to sensor_msgs::PointCloud2 failed!");
     return -1;
   }
   return 0;

}

double ptDist(geometry_msgs::Pose a,geometry_msgs::Pose b){
	return sqrt((a.position.x-b.position.x)*(a.position.x-b.position.x)+
			(a.position.y-b.position.y)*(a.position.y-b.position.y)+
			(a.position.z-b.position.z)*(a.position.z-b.position.z));
}

double ptAngle(geometry_msgs::Pose a,geometry_msgs::Pose b){
	return atan2((a.position.y-b.position.y),(a.position.x-b.position.x));
}




struct surveyscan{
   sensor_msgs::PointCloud2 cloud;
   tf::StampedTransform laser_transform,camera_transform,base_transform;
   std::vector<sensor_msgs::Image> images;
   sensor_msgs::CameraInfo caminfo;
   geometry_msgs::Pose object_pose;
};

class Surveyer{


private:
  ros::NodeHandle n_;
  tf::TransformListener listener;
  ros::ServiceClient assembler,path_planner,camera_scan_client;
  ros::ServiceClient cloud_combiner, nearest_cloud_finder,cloud_clusterer;
  ros::Publisher path_pub, cloud_pub,results_pub;
  double time_per_scan,radians_per_scan,path_radius;
  double object_height; //if positive, look at the x,y,z centroid of the object,
                        //otherwise look at the x,y centroid with z=object_height
  bool plan_circle;
  actionlib::SimpleActionClient<simple_controller::SimpleMoveAction> motion_client;
  std::string worldframe,bagfilename;
  object_survey::SurveyResults scans;
  object_survey::SurveyScan *currentscan;
  geometry_msgs::Pose current_object_pose;
  geometry_msgs::Pose current_pose;
  geometry_msgs::PoseArray current_path;
  sensor_msgs::PointCloud2 object_cloud;
  laser_geometry::LaserProjection projector_;

public:
  Surveyer():n_("~"),listener(ros::Duration(120.0)),motion_client("simple_mover", true)
  {

    updateParameters();
	//publish the path to our next destination
	path_pub = n_.advertise<geometry_msgs::PoseArray> ("/proposed_path", 1);
	//publish the cloud of the object we have gathered so far
   cloud_pub = n_.advertise<sensor_msgs::PointCloud2> ("/current_object", 1);
   results_pub = n_.advertise<object_survey::SurveyScan> ("/survey_results", 1);
	//start services
    //loc_client     = n_.serviceClient<target_nav::UpdateEstimate>("/checkerboard_localize");
    assembler      = n_.serviceClient<laser_assembler::AssembleScans>("/assemble_scans");
    camera_scan_client      = n_.serviceClient<object_survey::GetCameraScan>("/get_camera_scan");

    //cloud services:
    cloud_clusterer       = n_.serviceClient<furniture_ops::GetFurnitureClusters>("/cluster_cloud");
    nearest_cloud_finder  = n_.serviceClient<furniture_ops::GetNearestCluster>("/nearest_cluster");
    cloud_combiner        = n_.serviceClient<furniture_ops::CombineClouds>("/combine_clouds");


    if(plan_circle)
       path_planner     = n_.serviceClient<furniture_ops::PlanCirclePath>("/plan_circle");
    else
       path_planner     = n_.serviceClient<furniture_ops::PlanObservationPath>("/plan_observation_path");


    ROS_INFO("Waiting for services...");
    //ros::service::waitForService("/checkerboard_localize");
    ros::service::waitForService("/assemble_scans");
    ros::service::waitForService("/cluster_cloud");
    ros::service::waitForService("/get_camera_scan");
    ros::service::waitForService("/nearest_cluster");
    ros::service::waitForService("/combine_clouds");
    ros::service::waitForService("/plan_circle");
    ros::service::waitForService("/plan_observation_path");
    ROS_INFO("Services found.");
	currentscan=NULL;
  }

  void updateParameters(){
    n_.param("world_frame", worldframe, std::string("/odom_combined"));
    n_.param("bagfilename", bagfilename, std::string("default_bag"));
    n_.param("time_per_scan", time_per_scan, 30.0);  //default to 60 seconds per scan
    n_.param("radians_per_scan", radians_per_scan, .5);  //interval, in radians, at which to take scans
    n_.param("circle_path", plan_circle, false);  //whether to plan simple circle (true) or follow perimeter of object (false)
    n_.param("path_radius", path_radius, 1.0);  //distance at which to scan
    n_.param("object_height",object_height,-1.0);
    //if positive, look at the x,y,z centroid of the object,
    //otherwise look at the x,y centroid with z=object_height
  }


  void writeScanToBag(object_survey::SurveyScan &scan){

     try{
       rosbag::Bag bag(bagfilename,rosbag::bagmode::Append);
       bag.write("survey_results",ros::Time::now(),scan);
       bag.close();
     }
     catch(rosbag::BagIOException){
        rosbag::Bag bag(bagfilename,rosbag::bagmode::Write);
        bag.write("survey_results",ros::Time::now(),scan);
        bag.close();
     }
  }


  int planPath(){
     if(plan_circle){ //just moving in a circle
        ROS_INFO("Planning circle path");
        furniture_ops::PlanCirclePath srv;
        srv.request.radius=path_radius;
        srv.request.angle=radians_per_scan;
        srv.request.object_pose=current_object_pose;
        if (!path_planner.call(srv)){
          ROS_ERROR("Failed to call path planning service");
          return -1;
        }
        current_path = srv.response.path;
        ROS_INFO("Done planning circle path");
     }
     else{  //planning around the perimeter of the object
        ROS_INFO("Planning observation path");
        furniture_ops::PlanObservationPath srv;
        srv.request.radius=path_radius;
        srv.request.angle=radians_per_scan;
        srv.request.object_cloud=object_cloud;
        if (!path_planner.call(srv)){
          ROS_ERROR("Failed to call path planning service");
          return -1;
        }
        current_path = srv.response.path;
        ROS_INFO("Done planning observation path");
     }
     return 0;

  }

  int executePath(geometry_msgs::PoseArray &path){
     ROS_INFO("surveyer: Waiting for motion server...");
	  motion_client.waitForServer();
     ROS_INFO("surveyer: server found.");
	  simple_controller::SimpleMoveGoal moveaction;
	  moveaction.path=path;
	  motion_client.sendGoal(moveaction);
     ROS_INFO("surveyer: Waiting for server result...");
	  motion_client.waitForResult();
	  if (motion_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
	     std::cout<<"Goal position reached"<<std::endl;
		    return 0;
	  }
	  ROS_INFO("Current State: %s\n", motion_client.getState().toString().c_str());
	  return -1;
  }

//  int relocalize(bool scan=false){
//     ROS_INFO("Relocalizing ...");
//	  while(true){
//	  target_nav::UpdateEstimate srv;
//	  srv.request.scan=scan;
//	  if (!loc_client.call(srv)){
//	    ROS_ERROR("Failed to call Localization service");
//	    return -1;
//	  }
//	  if(srv.response.located==false){
//		 ROS_ERROR("Failed to relocalize");
//		 sleep(1);
//		 continue;
//	  }
//	  ROS_INFO("Localized successfully.");
//	  current_pose = srv.response.pose.pose;
//	  std::cout<<"relocalize: current_pose =  "<<current_pose<<std::endl;
//	  return 0;
//	}
//
//  }
  int takeCameraData(){
     std::vector<geometry_msgs::PointStamped> interest_points;
     //TODO: get interest points, where we want to get camera data

     //simplest case: point camera at centroid of object:
     geometry_msgs::PointStamped pt;
     pt.point=current_object_pose.position;
     if(object_height>=0) {
       pt.point.z = object_height;
     }
     pt.header.frame_id=worldframe;
     pt.header.stamp=ros::Time::now();
     interest_points.push_back(pt);

     for(uint i=0;i<interest_points.size();i++){
        object_survey::GetCameraScan srv;
        srv.request.focus_point=interest_points[i];
        if (!camera_scan_client.call(srv)){
           ROS_ERROR("Failed to call Camera Scan service");
           return -1;
        }
        currentscan->camera_scans.push_back(srv.response.scan);
      }
     return 0;
  }

  int takeData(){
	  //for debugging, just print message:
// 	  std::cout<<"just printing a take data message"<<std::endl;
// 	  sleep(.5);
// 	  return 0;
     ROS_INFO("Starting to take data.");
     //subscribe to the base laser scanner.  This subscriber will automatically unsubscribe
     //when the subscriber goes out of scope, i.e. when this procedure finishes
     ros::Subscriber sub = n_.subscribe("/base_scan",1,&Surveyer::scanCallback, this);
     scans.scans.push_back(object_survey::SurveyScan());
     currentscan=&scans.scans.back();
      //move head to look at object
     currentscan->object_pose = current_object_pose;
      ros::Time scan_start = ros::Time::now();
      ros::Time::sleepUntil(scan_start+ros::Duration(time_per_scan));
      //now save the laser data:
      laser_assembler::AssembleScans srv;
      srv.request.begin = scan_start;
      srv.request.end   = ros::Time::now();
      if (assembler.call(srv))
         printf("Got cloud with %u points\n", (uint)srv.response.cloud.points.size());
      else{
         printf("Service call failed\n");
         return -1;
      }
      tf::StampedTransform trans;

      ROS_DEBUG("transforming %s to %s delayed %f",srv.response.cloud.header.frame_id.c_str(),"/base_link",(ros::Time::now()-srv.response.cloud.header.stamp).toSec());
      listener.lookupTransform(srv.response.cloud.header.frame_id, "/base_link", ros::Time(0,0), trans);
      tf::transformStampedTFToMsg(trans,currentscan->base_transform);
      ROS_DEBUG("transforming %s to %s delayed %f",srv.response.cloud.header.frame_id.c_str(),"/laser_tilt_mount_link",(ros::Time::now()-srv.response.cloud.header.stamp).toSec());
      listener.lookupTransform(srv.response.cloud.header.frame_id, "/laser_tilt_mount_link", ros::Time(0,0), trans);
      tf::transformStampedTFToMsg(trans,currentscan->laser_transform);
      
      addViewPoint(srv.response.cloud,trans,currentscan->cloud);
      
      //signal that we are done recording data from this scan:
//      currentscan=NULL;
      ROS_INFO("done taking data.");

      return 0;

  }


  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    sensor_msgs::PointCloud cloud;
    projector_.transformLaserScanToPointCloud("/base_laser_link",*scan_in,cloud,listener);

    tf::StampedTransform trans;
    ROS_DEBUG("transforming %s to %s delayed %f",cloud.header.frame_id.c_str(),"/base_laser_link",(ros::Time::now()-cloud.header.stamp).toSec());
    ros::Time t = ros::Time::now();
    //while(!listener.canTransform(cloud.header.frame_id, "/base_laser_link", t)) t = ros::Time::now();
	listener.lookupTransform(cloud.header.frame_id, "/base_laser_link", t, trans);
    tf::transformStampedTFToMsg(trans,currentscan->base_laser_transform);

    sensor_msgs::PointCloud2 vp_cloud;
    addViewPoint(cloud,trans,vp_cloud);
    scans.scans.back().base_laser_scans.push_back(*scan_in);
    scans.scans.back().base_laser_clouds.push_back(vp_cloud);
  }


  int locateObject(bool use_prev){
     ROS_INFO("Locating object.");
     //assumes that scans has atleast one element:
     assert(scans.scans.size()>0);

     if(!use_prev){ //initial object locate - look at all objects and pick a suitable one
        furniture_ops::GetFurnitureClusters srv;
        srv.request.cloud=scans.scans.back().cloud;
        if (!cloud_clusterer.call(srv)){
          ROS_ERROR("Failed to call clustering service");
          return -1;
       }
        ROS_INFO("%d clusters found.",(int)srv.response.clouds.size());
        //find biggest cloud in front of robot (hackish, but a good way to start...)
        uint bestcloud=0,cloudsize=0;
        geometry_msgs::PointStamped p,pbase;
        for(uint i=0;i<srv.response.clouds.size();i++){
           p.point = srv.response.cloud_centers[i].position;
           p.header=scans.scans.back().cloud.header;
           ros::Time t=ros::Time::now();
           p.header.stamp=t;
           listener.waitForTransform("base_link",p.header.frame_id,t,ros::Duration(1.5));
           listener.transformPoint("base_link",p,pbase);
           //if cloud is between .5 and 2.5 meters in front
           //        and between -1 and 1 meters to the side
           if(abs(pbase.point.x - 1.5) < 1.0 && abs(pbase.point.y) < 1.0){
              if(srv.response.clouds[i].height*srv.response.clouds[i].width > cloudsize){
                 cloudsize=srv.response.clouds[i].height*srv.response.clouds[i].width;
                 bestcloud=i;
              } //if better cloud than prev seen
           }// if in expected location
        }//for each cloud
        ROS_INFO("picked cluster %u, at (%f,%f,%f)",bestcloud,srv.response.cloud_centers[bestcloud].position.x,srv.response.cloud_centers[bestcloud].position.y,srv.response.cloud_centers[bestcloud].position.z);
        current_object_pose=srv.response.cloud_centers[bestcloud];
        object_cloud=srv.response.clouds[bestcloud];

     }//if !useprev
     else{
        //already picked a cloud - now just get the cloud we want
       furniture_ops::GetNearestCluster srv;
       srv.request.cloud=scans.scans.back().cloud;
       srv.request.pose.pose=current_object_pose;
       if (!nearest_cloud_finder.call(srv)){
         ROS_ERROR("Failed to call nearest cloud finder service");
         return -1;
       }
       //now that we found a cloud, combine it with our current cloud
       furniture_ops::CombineClouds srv2;
       srv2.request.cloud1=object_cloud;
       srv2.request.cloud2=srv.response.cloud;
       if (!cloud_combiner.call(srv2)){
         ROS_ERROR("Failed to call cloud combiner service");
         return -1;
       }
       object_cloud=srv2.response.combined_cloud;
       current_object_pose=srv2.response.cloud_center;
     }
     ROS_INFO("current cloud has %i points",(int)(object_cloud.height*object_cloud.width));

     cloud_pub.publish(object_cloud);

	  return 0;
  }


  void mainLoop(){
    current_object_pose.position.x=1.0; current_object_pose.position.y=0.0;current_object_pose.position.z=0.0; //so the head knows where to go
      bool use_prev=false;
      //while(relocalize(true)) sleep(5);
      takeData();
      while(ros::ok()){
         if(locateObject(use_prev)){ ROS_ERROR("Error locating object. Exiting survey loop"); break;}
         use_prev=true;
         if(planPath()){ ROS_ERROR("Error planning path. Exiting survey loop"); break;}
         ROS_INFO("Path planned. %d waypoints. starting to execute.",(int)current_path.poses.size());
            path_pub.publish(current_path);
         //TODO: add path check to see if path is do-able first
         if(executePath(current_path)){ ROS_ERROR("Error executing path. Exiting survey loop"); break; }
         //if(relocalize()){ ROS_ERROR("Error relocalizing. Exiting survey loop"); break; }
	      sleep(5);
         takeData();
         if(locateObject(use_prev)){ ROS_ERROR("Error locating object. Exiting survey loop"); break;}

         if(takeCameraData()) return -1;   //take (possibly) multiple pictures, stereo clouds
         results_pub.publish(scans.scans.back());
         writeScanToBag(scans.scans.back());
      }
  }


};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_surveyer");
  ros::NodeHandle n;
  Surveyer surveyer;
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  surveyer.mainLoop();
  spinner.stop();
  return 0;
}
