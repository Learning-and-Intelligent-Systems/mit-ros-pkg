#include "ros/ros.h"
#include "opencupboard_msgs/cabinet_table.h"
#include "opencupboard_msgs/cabinet_table_result.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include "pcl/segmentation/extract_clusters.h"
#include <pcl/kdtree/tree_types.h>
#include <object_manipulation_msgs/FindClusterBoundingBox.h>
#include "opencupboard_action/processing_tools.hpp"

tf::TransformListener *listener;

/**
 *\brief Finds handle and returns its pose
 *
 *finds handle on a given table by looking for the closest point
 *@param req the detected table
 *@param res the pose of the detected handle (by reference)
 *@returns True
 *@todo change so that it returns the handle as the cluster with 
 * the best parameters. currently still just uses the closest point
 */
bool add(opencupboard_msgs::cabinet_table_result::Request &req,
         opencupboard_msgs::cabinet_table_result::Response &res )
{
  ros::NodeHandle nh;

  std::string orien;
  geometry_msgs::Point32 lowest, highest, handle;
  lowest.x = 1000;
  lowest.y = 1000;
  lowest.z = 1000;
  highest.x = -1000;
  highest.y = -1000;
  highest.z = -1000;

  geometry_msgs::Pose handle_pose;      
  object_manipulation_msgs::ClusterBoundingBox final_box;

  // refine handle options using clusters
  for (unsigned int i=0;i<req.detection.clusters.size();i++)
    {
      // find the cluster's bounding box 
      ros::ServiceClient client = nh.serviceClient
	<object_manipulation_msgs::FindClusterBoundingBox>
	("find_cluster_bounding_box");
      object_manipulation_msgs::FindClusterBoundingBox srv;
      
      srv.request.cluster = req.detection.clusters[i];
      if (client.call(srv))
	{
	  object_manipulation_msgs::ClusterBoundingBox bounding_box, transformed_box;
	  bounding_box.pose_stamped = srv.response.pose;
	  bounding_box.dimensions = srv.response.box_dims;

	  make_marker(srv.response.pose, visualization_msgs::Marker::CUBE,
		      srv.response.pose.pose.position.x,
		      srv.response.pose.pose.position.y,
		      srv.response.pose.pose.position.z,
		      srv.response.box_dims.x,
		      srv.response.box_dims.y,
		      srv.response.box_dims.z, 1.0,0.0,1.0,
		      srv.response.pose.pose.orientation.x,
		      srv.response.pose.pose.orientation.y,
		      srv.response.pose.pose.orientation.z,
		      srv.response.pose.pose.orientation.w);
	  
  	  // convert box from box frame into table frame
	  double half_x = (srv.response.box_dims.x)/2;
  	  double half_y = (srv.response.box_dims.y)/2;
  	  double half_z = (srv.response.box_dims.z)/2;
  	  geometry_msgs::Point32 box_max,new_max, world_max;
  	  box_max.x = half_x;
  	  box_max.y = half_y;
  	  box_max.z = half_z;
  	  inverse_transform_point(srv.response.pose.pose,box_max,world_max); 
	  transform_point(req.detection.table.pose.pose, world_max, new_max);
  	  geometry_msgs::Point32 box_min,new_min, world_min;
  	  box_min.x = -1.0*half_x;
  	  box_min.y = -1.0*half_y;
  	  box_min.z = -1.0*half_z;
  	  inverse_transform_point(srv.response.pose.pose,box_min,world_min);
	  transform_point(req.detection.table.pose.pose, world_min, new_min);

	  if (world_min.x > world_max.x)
	    {
	      float temp = world_min.x;
	      world_min.x = world_max.x;
	      world_max.x = temp;
	    }
	  if (world_min.y > world_max.y)
	    {
	      float temp = world_min.y;
	      world_min.y = world_max.y;
	      world_max.y = temp;
	    }
	  if (world_min.z > world_max.z)
	    {
	      float temp = world_min.z;
	      world_min.z = world_max.z;
	      world_max.z = temp;
	    }
	  
	  geometry_msgs::Point32 cluster_middle;
	  cluster_middle.x = (world_max.x+world_min.x)/2;
	  cluster_middle.y = (world_max.y+world_min.y)/2;
	  cluster_middle.z = (world_max.z+world_min.z)/2;

	  transformed_box.pose_stamped.header.frame_id = req.detection.table.pose.header.frame_id;
	  transformed_box.pose_stamped.pose.position.x = cluster_middle.x;
	  transformed_box.pose_stamped.pose.position.y = cluster_middle.y;
	  transformed_box.pose_stamped.pose.position.z = cluster_middle.z;
	  transformed_box.dimensions.x = world_max.x-world_min.x;
	  transformed_box.dimensions.y = world_max.y-world_min.y;
	  transformed_box.dimensions.z = world_max.z-world_min.z;

	  // find the bounds of the table dims
	  geometry_msgs::Point32 table_orig_max, table_orig_min, table_max, table_min;
	  table_orig_max.x = req.detection.table.x_max;
	  table_orig_max.y = req.detection.table.y_max;
	  table_orig_max.z = 0.25;
	  table_orig_min.x = req.detection.table.x_min;
	  table_orig_min.y = req.detection.table.y_min;
	  table_orig_min.z = 0.0;

	  inverse_transform_point(req.detection.table.pose.pose, table_orig_max, table_max);
	  inverse_transform_point(req.detection.table.pose.pose, table_orig_min, table_min);

	  // check that the handle is within the table dims
	  // if ((cluster_middle.x < table_max.x) && (cluster_middle.x > table_min.x) && 
	  //     (cluster_middle.y < table_max.y) && (cluster_middle.y > table_min.y) && 
	  //     (cluster_middle.z < table_max.z) && (cluster_middle.z > table_min.z))
	  if (1==1)
	    {
	      
	      float handle_dim_min = 0.007; // so ignores tiny blips
	      float handle_dim_max = 0.07; // max the hand can open

	      // checks the dims of the handle to fit in the hand: x or y .002-.07
	      if ((((world_max.x-world_min.x)<handle_dim_max && 
		    (world_max.x-world_min.x)>handle_dim_min) || 
		   ((world_max.y-world_min.y)<handle_dim_max && 
		    (world_max.y-world_min.y)>handle_dim_min)) && 
		  ((world_max.z-world_min.z)>0))
		{   
		  
		  // checks handle position relative to the door: 'top','bot','left','right'
		  std::string handle_pos;
		  nh.param<std::string>("handle_position", handle_pos, "top");

		  if (handle_pos == "top")
		    {
		      if (cluster_middle.z > highest.z)
			{
			  if ((world_max.z-world_min.z) > (world_max.y-world_min.y)) // vertical
			    {
			      orien = "vert";
			    } 
			  else 
			    {
			      orien = "hor";
			    }
			  highest = cluster_middle;
			  handle = highest;
			  final_box = transformed_box;
			  ROS_INFO("best TOP handle");
			}
		      else
			{
			  ROS_INFO("not best");
			}

		    }
		  else if (handle_pos == "bot")
		    {
		      if (cluster_middle.z < lowest.z)
			{
			  if ((world_max.z-world_min.z) > (world_max.y-world_min.y)) // vertical
			    {
			      orien = "vert";
			    } 
			  else 
			    {
			      orien = "hor";
			    }
			  lowest = cluster_middle;
			  handle = lowest;
			  final_box = transformed_box;		      
			  ROS_INFO("best BOT handle");
			}
		      else
			{
			  ROS_INFO("not best");
			}
		    }
		  else if (handle_pos == "left")
		    {
		      if (cluster_middle.y > highest.y)
			{
			  if ((world_max.z-world_min.z) > (world_max.y-world_min.y)) // vertical
			    {
			      orien = "vert";
			    } 
			  else 
			    {
			      orien = "hor";
			    }
			  highest = cluster_middle;
			  handle = highest;
			  final_box = transformed_box;
			  ROS_INFO("best LEFT handle");
			}
		      else
			{
			  ROS_INFO("not best");
			}
		    }
		  else if (handle_pos == "right")
		    {
		      if (cluster_middle.y < lowest.y)
			{
			  if ((world_max.z-world_min.z) > (world_max.y-world_min.y)) // vertical
			    {
			      orien = "vert";
			    } 
			  else 
			    {
			      orien = "hor";
			    }
			  lowest = cluster_middle;
			  handle = lowest;
			  final_box = transformed_box;
			  ROS_INFO("best RIGHT handle");
			}
		      else
			{
			  ROS_INFO("not best");
			}
		    }
		}
	      else
		{
		  ROS_INFO("cluster dims wrong");
		}
	    }
	  else 
	    {
	      ROS_INFO("cluster not within table");
	    }
	}
      else
	{
	  ROS_INFO("could not find bounding box");
	}
    }

  geometry_msgs::Point32 transformed_handle;
  transformed_handle = handle;
  make_marker(req.detection.table.pose,visualization_msgs::Marker::SPHERE,
	      transformed_handle.x,
	      transformed_handle.y,
	      transformed_handle.z,
	      0.1,0.1,0.1,0.0,1.0,1.0);
  
  handle_pose.position.x = transformed_handle.x;
  handle_pose.position.y = transformed_handle.y;
  handle_pose.position.z = transformed_handle.z;

  // find handle orientation
  if (orien=="vert") // vertical
    {
      ROS_INFO("VERTICAL");
      handle_pose.orientation.x = 0.0;
      handle_pose.orientation.y = 0.0;
      handle_pose.orientation.z = 0.0;
      handle_pose.orientation.w = 1.0;
    }
  else // horizontal
    {
      ROS_INFO("HORIZONTAL");
      handle_pose.orientation.x = 0.707;
      handle_pose.orientation.y = 0.0;
      handle_pose.orientation.z = 0.0;
      handle_pose.orientation.w = 0.707;
    }

  // refines if params are set

  // handle orientation: 'horizontal','vertical'
  if (nh.hasParam("handle_orientation"))
    {
      std::string handle_orientation;
      nh.getParam("handle_orientation", handle_orientation); 
      
      if (handle_orientation == "horizontal")
	{
	  handle_pose.orientation.x = 0.707;
	  handle_pose.orientation.y = 0.0;
	  handle_pose.orientation.z = 0.0;
	  handle_pose.orientation.w = 0.707;
	}
      else if (handle_orientation == "vertical")
	{
	  handle_pose.orientation.x = 0.0;
	  handle_pose.orientation.y = 0.0;
	  handle_pose.orientation.z = 0.0;
	  handle_pose.orientation.w = 1.0;
	}
    }

  // handle size
  if (nh.hasParam("max_x"))
    {
      double max_x_param;
      nh.getParam("max_x", max_x_param);
      if (final_box.dimensions.x > max_x_param)
	{
	  ROS_INFO("handle size not within params");
	  handle_pose.position.x = 0.0;
	  handle_pose.position.y = 0.0;
	  handle_pose.position.z = 0.0;	  
	}
    }
  if (nh.hasParam("min_x"))
    {
      double min_x_param;
      nh.getParam("min_x", min_x_param);
      if (final_box.dimensions.x < min_x_param)
	{
	  ROS_INFO("handle size not within params");
	  handle_pose.position.x = 0.0;
	  handle_pose.position.y = 0.0;
	  handle_pose.position.z = 0.0;	  
	}
    }
  if (nh.hasParam("max_y"))
    {
      double max_y_param;
      nh.getParam("max_y", max_y_param);
      if (final_box.dimensions.y > max_y_param)
	{
	  ROS_INFO("handle size not within params");
	  handle_pose.position.x = 0.0;
	  handle_pose.position.y = 0.0;
	  handle_pose.position.z = 0.0;	  
	}
    }
  if (nh.hasParam("min_y"))
    {
      double min_y_param;
      nh.getParam("min_y", min_y_param);
      if (final_box.dimensions.y < min_y_param)
	{
	  ROS_INFO("handle size not within params");
	  handle_pose.position.x = 0.0;
	  handle_pose.position.y = 0.0;
	  handle_pose.position.z = 0.0;	  
	}
    }
  if (nh.hasParam("max_z"))
    {
      double max_z_param;
      nh.getParam("max_z", max_z_param);
      if (final_box.dimensions.z > max_z_param)
	{
	  ROS_INFO("handle size not within params");
	  handle_pose.position.x = 0.0;
	  handle_pose.position.y = 0.0;
	  handle_pose.position.z = 0.0;	  
	}
    }
  if (nh.hasParam("min_z"))
    {
      double min_z_param;
      nh.getParam("min_z", min_z_param);
      if (final_box.dimensions.z < min_z_param)
	{
	  ROS_INFO("handle size not within params");
	  handle_pose.position.x = 0.0;
	  handle_pose.position.y = 0.0;
	  handle_pose.position.z = 0.0;	  
	}
    }

  // handle absolute position (in base_link)
  double tol = 0.05;
  if (nh.hasParam("handle_x"))
    {
      double handle_x;
      nh.getParam("handle_x", handle_x);
      if ((handle_pose.position.x > (handle_x+tol)) ||
	  (handle_pose.position.x < (handle_x-tol)))
	{
	  ROS_INFO("handle position not within params");
	  handle_pose.position.x = 0.0;
	  handle_pose.position.y = 0.0;
	  handle_pose.position.z = 0.0;
	}
    }
  if (nh.hasParam("handle_y"))
    {
      double handle_y;
      nh.getParam("handle_y", handle_y);
      if ((handle_pose.position.y > (handle_y+tol)) ||
	  (handle_pose.position.y < (handle_y-tol)))
	{
	  ROS_INFO("handle position not within params");
	  handle_pose.position.x = 0.0;
	  handle_pose.position.y = 0.0;
	  handle_pose.position.z = 0.0;
	}
    }
  if (nh.hasParam("handle_z"))
    {
      double handle_z;
      nh.getParam("handle_z", handle_z);
      if ((handle_pose.position.z > (handle_z+tol)) ||
	  (handle_pose.position.z < (handle_z-tol)))
	{
	  ROS_INFO("handle position not within params");
	  handle_pose.position.x = 0.0;
	  handle_pose.position.y = 0.0;
	  handle_pose.position.z = 0.0;
	}
    }

  // response is the cluster that best fits a handle dude
  res.pose.header.frame_id = req.detection.table.pose.header.frame_id;
  res.pose.header.stamp = ros::Time::now();
  res.pose.pose = handle_pose;
  res.box = final_box;

  return true;
}

/**
 *\brief publishes handle
 *
 *initializes the node, transform listener, and publishes handle and marker
 *@param argc 
 *@param argv 
 *@returns 0
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "cabinet_table_result_server");
  ros::NodeHandle n;

  listener = new tf::TransformListener();
  //  visPub =  n.advertise<visualization_msgs::Marker>("visualization_marker",1);
  marker_init();
  // visPub = n.advertise<visualization_msgs::Marker>("/handle_marker",0);
  ros::ServiceServer service = n.advertiseService("cabinet_table_result", add);
  ROS_INFO("Ready to find cabinet handle.");
  ros::spin();

  delete listener;
  return 0;
}
