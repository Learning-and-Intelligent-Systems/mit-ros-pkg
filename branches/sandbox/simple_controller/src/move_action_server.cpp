/*
 * move_action_server.cpp
 *
 *  Created on: Oct 10, 2010
 *      Author: garratt
 */

#include <simple_controller/SimpleMoveAction.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <list>


typedef actionlib::SimpleActionServer<simple_controller::SimpleMoveAction> Server;

//signed min: returns the closest to zero, with the sign of 'a'
double smin(double a, double b){
   double s=a>0?1.0:-1.0;
   return s*std::min(fabs(a),fabs(b));
}
//signed max: returns the farthest to zero, with the sign of 'a'
double smax(double a, double b){
   double s=a>0?1.0:-1.0;
   return s*std::max(fabs(a),fabs(b));
}
double ptDist(geometry_msgs::PoseStamped &a, const geometry_msgs::Point32 &b){
   return sqrt((a.pose.position.x-b.x)*(a.pose.position.x-b.x)+(a.pose.position.y-b.y)*(a.pose.position.y-b.y));
}

class SimpleMover{
	ros::NodeHandle n_;
	ros::Publisher motor_cmd,path_pub;
	ros::Subscriber laser_sub;
	std::list<geometry_msgs::PoseStamped> path;
	std::string worldframe;
	Server server;
	std::string runstate;
	tf::TransformListener listener;
   double xy_tol, ang_tol, xy_vmax, ang_vmax,collision_threshold;

public:
	SimpleMover(std::string name):n_("~"),server(n_, "",boost::bind(&SimpleMover::execute, this,_1)),listener(ros::Duration(20.7))
	{
//	   server.registerGoalCallback(boost::bind(&SimpleMover::execute, this));
		path_pub = n_.advertise<geometry_msgs::PoseArray> ("/current_path", 1);
		motor_cmd = n_.advertise<geometry_msgs::Twist> ("/base_controller/command", 1);
		laser_sub=n_.subscribe("/base_scan_cloud", 1, &SimpleMover::lasercb, this);
		updateParameters();
      runstate="running";
	}

	void updateParameters(){
      n_.param("world_frame", worldframe, std::string("/world"));
      n_.param("position_tolerance", xy_tol, .05);
      n_.param("angular_tolerance", ang_tol, .05);
      n_.param("max_speed", xy_vmax, .14);
      n_.param("max_angular_speed", ang_vmax, .4);
      n_.param("collision_threshold", collision_threshold, .5);
	}


	void lasercb(const sensor_msgs::PointCloudConstPtr& cloud){
	   //for starters, naively check all path points for a point in the cloud w/in a threshold
	   sensor_msgs::PointCloud wcloud,blcloud;
	   if(path.size()>0){
	      std::string frame=cloud->header.frame_id;
	      ros::Time t = cloud->header.stamp;
	      ROS_DEBUG("transforming %s to %s delayed %f in %s",worldframe,frame,(ros::Time::now()-t).toSec(),"SimpleMover::lasercb()");
	      listener.waitForTransform(worldframe,cloud->header.frame_id,cloud->header.stamp,ros::Duration(1.5));
	      listener.transformPointCloud(worldframe,*cloud,wcloud);
         ROS_DEBUG("transforming %s to %s delayed %f in %s","base_laser_link",frame,(ros::Time::now()-t).toSec(),"SimpleMover::lasercb()");
	      listener.waitForTransform("/base_laser_link",cloud->header.frame_id,cloud->header.stamp,ros::Duration(1.5));
	      listener.transformPointCloud("/base_laser_link",*cloud,blcloud);
	   }
	   for(std::list<geometry_msgs::PoseStamped>::iterator it=path.begin();it !=path.end();it++){
	      for(uint j=0;j<wcloud.points.size();j++){
	         if(ptDist(*it, wcloud.points[j]) < collision_threshold && (fabs(blcloud.points[j].x) > .04 || fabs(blcloud.points[j].y) > .04)){
	            motor_cmd.publish(geometry_msgs::Twist()); //stop the robot
	            runstate="collision";
	            ROS_INFO("SimpleMover: Collision detected, distance= %f, x = %f, y = %f, (%f, %f)",ptDist(*it, wcloud.points[j]),wcloud.points[j].x,wcloud.points[j].y,fabs(blcloud.points[j].x),fabs(blcloud.points[j].y));

	            return;
	         }

	      }

	   }
	   runstate="running";
	}

	void addPath(const geometry_msgs::PoseArray _path){
		geometry_msgs::PoseStamped ps,psout;
      path.clear();
      ps.header=_path.header;
      ps.header.stamp=ros::Time::now();
      for(uint i=0;i<_path.poses.size();i++){
         ps.pose=_path.poses[i];
         ROS_DEBUG("transforming %s to %s delayed %d in %s","base_link",ps.header.frame_id,(ros::Time::now()-ps.header.stamp).toSec(),__PRETTY_FUNCTION__);
         listener.waitForTransform("/base_link",ps.header.frame_id,ps.header.stamp,ros::Duration(.5));
         listener.transformPose(worldframe,ps,psout);
         path.push_back(psout);
      }
	}

	geometry_msgs::PoseStamped getRelativePose(){
		geometry_msgs::PoseStamped rel_pose;
		path.front().header.stamp=ros::Time::now();
		listener.waitForTransform("/base_link",path.front().header.frame_id,path.front().header.stamp,ros::Duration(.5));
	    listener.transformPose("/base_link",path.front(),rel_pose);
		return rel_pose;
	}

	void normalizeVels(double &x,double &y){
		double norm=xy_vmax/sqrt(x*x+y*y);
		x*=norm;
		y*=norm;
	}

	void publishPath(){
	    geometry_msgs::PoseArray ppath;
	    ppath.header=path.front().header;
	    for(std::list<geometry_msgs::PoseStamped>::iterator it=path.begin();it !=path.end();it++)
		ppath.poses.push_back(it->pose);
	    path_pub.publish(ppath);	  
	}

	void execute(const simple_controller::SimpleMoveGoalConstPtr& goal)
	{
	   double angdiff;
	   addPath(goal->path); //convert the poseArray to a vector of poses in the world frame
	   geometry_msgs::PoseStamped rel_pose;
	   geometry_msgs::Twist cmd;
	   ros::Rate r(30.0);
	   int waypointsreached=0,totalwaypoints=path.size();
//      ROS_INFO("SimpleMover: distance thresholds: (%f, %f, %f)",xy_tol,xy_tol,ang_tol);
	   while(path.size()>0){
	      //get relative pose of next waypoint
	     rel_pose=getRelativePose();
         angdiff=tf::getYaw(rel_pose.pose.orientation);

	     //check if reached waypoint:
         if( fabs(rel_pose.pose.position.x) < xy_tol && fabs(rel_pose.pose.position.y) < xy_tol && (path.size() > 1 || fabs(angdiff) < ang_tol)){
            ROS_INFO("Reached %d out of %d waypoints",++waypointsreached,totalwaypoints);
            path.pop_front();
            continue;
         }

         ROS_DEBUG("SimpleMover: distance to next waypoint: (%f, %f, %f)",rel_pose.pose.position.x,rel_pose.pose.position.y,angdiff);
         //convert the diffs to a velocity command:
	     cmd.angular.z=smin(angdiff*4.0,ang_vmax);
	     cmd.linear.x=smin(rel_pose.pose.position.x*4.0,xy_vmax);
         cmd.linear.y=smin(rel_pose.pose.position.y*4.0,xy_vmax);
         //normalize to constant speed:
//         if(path.size()>1)
//        	 normalizeVels(cmd.linear.x,cmd.linear.y);
//         std::cout<<"vel: "<<sqrt(cmd.linear.x*cmd.linear.x+cmd.linear.y*cmd.linear.y)<<"  ("<<cmd.linear.x<<", "<<cmd.linear.y<<")  avel: "<<cmd.angular.z<<std::endl;

         if(runstate.compare("running")){
            ROS_INFO("SimpleMover: Collision detected, paused");
         }
         else{
            ROS_INFO("SimpleMover: Publishing motor commands (%f, %f, %f)",cmd.linear.x,cmd.linear.y,cmd.angular.z);
            motor_cmd.publish(cmd);
         }
         r.sleep();
	 publishPath();
	   }
	   //done with waypoints:
	     cmd.angular.z=cmd.linear.x=cmd.linear.y=0;
         motor_cmd.publish(cmd);



	  server.setSucceeded();
	}

};








int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_action_server");
//  ros::NodeHandle n;
  SimpleMover mover(ros::this_node::getName());
  ros::spin();
  return 0;
}
