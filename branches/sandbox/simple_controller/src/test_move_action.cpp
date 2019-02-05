/*
 * test_move_action.cpp
 *
 *  Created on: Oct 19, 2010
 *      Author: garratt
 */

/*
 * surveyer.cpp
 *
 *  Created on: Oct 8, 2010
 *      Author: garratt
 */
#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include <tf/transform_listener.h>

#include <simple_controller/SimpleMoveAction.h>
#include <actionlib/client/simple_action_client.h>



double ptDist(geometry_msgs::Pose a,geometry_msgs::Pose b){
   return sqrt((a.position.x-b.position.x)*(a.position.x-b.position.x)+
         (a.position.y-b.position.y)*(a.position.y-b.position.y)+
         (a.position.z-b.position.z)*(a.position.z-b.position.z));
}

double ptAngle(geometry_msgs::Pose a,geometry_msgs::Pose b){
   return atan2((a.position.y-b.position.y),(a.position.x-b.position.x));
}


class MoveClient{


private:
  tf::TransformListener listener;
  ros::NodeHandle n_;
  ros::ServiceClient path_planner;
  double time_per_scan,radians_per_scan,path_radius;
  bool plan_circle;
  ros::Subscriber image_subscribe,camera_info_subscribe;
  actionlib::SimpleActionClient<simple_controller::SimpleMoveAction> motion_client;
  std::string worldframe;
  geometry_msgs::Pose current_object_pose, current_pose;
  geometry_msgs::PoseArray current_path;

public:
  MoveClient():n_("~"),listener(ros::Duration(20.0)),motion_client("simple_mover", true)
  {

    updateParameters();
   //publish the path to our next destination

  }

  void updateParameters(){
    n_.param("world_frame", worldframe, std::string("/odom_combined"));
    n_.param("time_per_scan", time_per_scan, 30.0);  //default to 60 seconds per scan
    n_.param("radians_per_scan", radians_per_scan, .5);  //interval, in radians, at which to take scans
    n_.param("circle_path", plan_circle, false);  //whether to plan simple circle (true) or follow perimeter of object (false)
    n_.param("path_radius", path_radius, 1.0);  //distance at which to scan
  }


  int planPath(){

     current_pose=getCurrentPose();
   double current_radius=ptDist(current_pose,current_object_pose),radius;
   double current_angle=ptAngle(current_pose,current_object_pose),angle;
   double radius_incriment=(path_radius-current_radius)/radians_per_scan;

     std::cout<<"planPath: current_radius =  "<<current_radius<<std::endl;
     std::cout<<"planPath: current_angle =  "<<current_angle<<std::endl;
     std::cout<<"planPath: current_object_pose =  "<<current_object_pose<<std::endl;

   geometry_msgs::Pose p;
   for(double i=0; i< radians_per_scan+.1; i+=.1){
      angle=i+current_angle;
      radius=current_radius+radius_incriment*i;
      p.position.x=radius*cos(angle)+current_object_pose.position.x;
      p.position.y=radius*sin(angle)+current_object_pose.position.y;
      p.orientation = tf::createQuaternionMsgFromYaw(angle+3.1415);
      current_path.poses.push_back(p);
      std::cout<<"planPath: increment =  "<<i<<"  angle "<<angle<<"  radius "<<radius<<"  p.position "<<p.position.x<<", "<<p.position.y<<std::endl;
   }
   //set header:
   current_path.header.stamp=ros::Time::now();
   current_path.header.frame_id=worldframe;
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


  geometry_msgs::Pose getCurrentPose(){
     tf::StampedTransform transform;
     ros::Time t=ros::Time::now();
     ROS_DEBUG("transforming /world to base_link now in MoveClient::getCurrentPose()");
     listener.waitForTransform(worldframe,"base_link",t,ros::Duration(1.5));
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
  //just return a pose in front of the robot
  geometry_msgs::Pose getObjectPose(){
       tf::StampedTransform transform;
       ros::Time t=ros::Time::now();
       ROS_DEBUG("transforming /world to base_link now in MoveClient::getCurrentPose()");
       listener.waitForTransform(worldframe,"base_link",t,ros::Duration(1.5));
       geometry_msgs::PoseStamped ps,psout;
       ps.pose.orientation.w=1.0;
       ps.pose.position.x=1.0;
       ps.header.stamp=t;
       ps.header.frame_id="base_link";
       listener.transformPose(worldframe,ps, psout);
       return psout.pose;
    }


  void mainLoop(){
     current_object_pose=getObjectPose();

//   while(ros::ok()){
      if(planPath()){ ROS_ERROR("Error planning path. Exiting survey loop"); return;}
      ROS_INFO("Path planned. %d waypoints. starting to execute.",(int)current_path.poses.size());
      if(executePath(current_path)){ ROS_ERROR("Error executing path. Exiting survey loop"); return; }
//   }

  }


};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_surveyer");
  ros::NodeHandle n;
  MoveClient mc;
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  mc.mainLoop();
  spinner.stop();
  return 0;
}
