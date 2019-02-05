k
#include "ros/ros.h"
#include "tabletop_for_cabinet/cabinet_table.h"
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

//#include <pcl/kdtree/kdtree_ann.h>

#include "pcl/segmentation/extract_clusters.h"
#include <pcl/kdtree/tree_types.h>
#include <object_manipulation_msgs/FindClusterBoundingBox.h>

ros::Publisher visPub;

visualization_msgs::Marker marker,box;

tf::TransformListener *listener;

/**
 *\brief Converts the given point from the frame of the given pose.
 *
 *Uses conversion matrix to inverse transform a point when the tf is not 
 *publicly published(e.g. to convert a point from the frame of a table
 *to the frame the table is defined in, give the pose of the detected table)
 *@param pose desired pose to transform from
 *@param point originial point 
 *@param new_point converted point (by reference)
 */
void inverse_transform_point(geometry_msgs::Pose pose,
			     const geometry_msgs::Point32 &point,
			     geometry_msgs::Point32 &new_point)
{
  double x = pose.orientation.x, y = pose.orientation.y, z = pose.orientation.z, w = pose.orientation.w;
  double pose_x = pose.position.x, pose_y = pose.position.y, pose_z = pose.position.z;
  
  // Conversion matrix from table frame
  double m00 = (1.0-2.0*((y*y)+(z*z)));
  double m01 = (2.0*((x*y)-(w*z)));
  double m02 = (2.0*((x*z)+(w*y)));
  double m10 = (2.0*((x*y)+(w*z)));
  double m11 = (1.0-2.0*((x*x)+(z*z)));
  double m12 = (2.0*((y*z)-(w*x)));
  double m20 = (2.0*((x*z)-(w*y)));
  double m21 = (2.0*((y*z)+(w*x)));
  double m22 = (1.0-2.0*((x*x)+(y*y)));
  new_point.x = ((m00*point.x)+(m01*point.y)+(m02*point.z)) + pose_x;
  new_point.y = ((m10*point.x)+(m11*point.y)+(m12*point.z)) + pose_y;
  new_point.z = ((m20*point.x)+(m21*point.y)+(m22*point.z)) + pose_z;
}

/**
 *\brief Converts the given point into the frame of the given pose.
 *
 *Uses conversion matrix to transform a point when the tf is not 
 *publicly published(e.g. to convert a point into the frame of a table
 *from the frame the table is defined in, give the pose of the detected table)
 *@param pose desired pose to transform from
 *@param point originial point 
 *@param new_point converted point (by reference)

 */
void transform_point(geometry_msgs::Pose pose,
		     const geometry_msgs::Point32 &point,
		     geometry_msgs::Point32 &new_point) 
{
  double x = pose.orientation.x, y = pose.orientation.y, z = pose.orientation.z, w = pose.orientation.w;
  double pose_x = pose.position.x, pose_y = pose.position.y, pose_z = pose.position.z;
  
  // Conversion matrix to table frame
  double m00 = (1.0-2.0*((y*y)+(z*z)));
  double m01 = (2.0*((x*y)-(w*z)));
  double m02 = (2.0*((x*z)+(w*y)));
  double m10 = (2.0*((x*y)+(w*z)));
  double m11 = (1.0-2.0*((x*x)+(z*z)));
  double m12 = (2.0*((y*z)-(w*x)));
  double m20 = (2.0*((x*z)-(w*y)));
  double m21 = (2.0*((y*z)+(w*x)));
  double m22 = (1.0-2.0*((x*x)+(y*y)));
  double tx = point.x-pose_x;
  double ty = point.y-pose_y;
  double tz = point.z-pose_z;
  new_point.x = ((m00*tx)+(m10*ty)+(m20*tz));
  new_point.y = ((m01*tx)+(m11*ty)+(m21*tz));
  new_point.z = ((m02*tx)+(m12*ty)+(m22*tz));
}

/**
 *\brief Finds the clusters in a given point cloud.
 *
 *Uses pcl's extractEuclideanClusters to find a vector of the clusters
 *@param cloud full cloud you where clusters will be found
 *@param clusters vector of the detected clusters (by reference)
 */
void create_clusters(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, 
		     std::vector<pcl::PointIndices> &clusters )
{
  pcl::KdTree<pcl::PointXYZ>::Ptr clusters_tree_;

  //  clusters_tree_ = boost::make_shared<pcl::KdTreeANN<pcl::PointXYZ> > ();

  //sets the tolerance for the size of the clusters
  double cluster_tolerance_ = 0.05;

  pcl::initTree(0, clusters_tree_);
  clusters_tree_->setInputCloud(cloud);
  pcl::extractEuclideanClusters(*cloud,clusters_tree_,cluster_tolerance_,clusters);
}

/**
 *\brief Filters cloud to only include objects within the boundaries of the table
 *
 *@param table
 *@param cloud cloud you are filtering
 *@param object_cloud cloud of only the points within the table (by reference)
 */
void find_object_cloud(tabletop_object_detector::Table table, 
		       sensor_msgs::PointCloud cloud,
		       pcl::PointCloud<pcl::PointXYZ> &object_cloud)
{
  object_cloud.header.frame_id = cloud.header.frame_id;
  for (int i=0;i<cloud.points.size();i+=2)
    {
      if (!isnan(cloud.points[i].z))
	{	
	  // transform into the frame of the table
	  geometry_msgs::Point32 new_point;
	  transform_point(table.pose.pose,cloud.points[i],new_point);
	  
	  if (new_point.x>table.x_min && new_point.x<table.x_max && 
	      new_point.y>table.y_min && new_point.y<table.y_max && 
	      new_point.z>0.005)
	    {
	      pcl::PointXYZ pcl_point;
	      pcl_point.x = cloud.points[i].x;
	      pcl_point.y = cloud.points[i].y;
	      pcl_point.z = cloud.points[i].z;
	      object_cloud.push_back(pcl_point);
	    }
	}
    }
  ROS_INFO("in func object_cloud size=%d", object_cloud.points.size());
}

/**
 *\brief publishes a marker in rviz
 *
 *Publishes a purple sphere marker in rviz at specified coordinates
 *@param pose pose of frame to publish the marker in
 *@param type type of marker. must be in all caps (visualization_msgs::Marker::SPHERE,etc)
 *@param x (should be in baselink)
 *@param y (should be in baselink)
 *@param z (should be in baselink)
 */
void make_marker(geometry_msgs::PoseStamped pose,
		 int type, 
		 float x, float y, float z )
{
      marker.header.frame_id = pose.header.frame_id;
      marker.header.stamp = ros::Time::now();
      marker.ns = "mysphere";
      // doesn't matter as long as its different for every marker
      marker.id = x+y+z;
      marker.type = type;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = z;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.r = 1;
      marker.color.g = 0;
      marker.color.b = 1;
      marker.color.a = 1;
      //marker.lifetime = ros::Duration(30);
      
      visPub.publish(marker);
}

void find_closest_point(geometry_msgs::Pose pose,
			pcl::PointCloud<pcl::PointXYZ> cloud,
			geometry_msgs::Point32 &closestPoint)
{
  // initialize closestPoint to be very large
  closestPoint.x = 0.0;
  closestPoint.y = 0.0;
  closestPoint.z = -1000;
  // only goes through every other point for speed
  for (int i=0;i<cloud.points.size();i+=2)
    {
      if (!isnan(cloud.points[i].z))
	{
	  geometry_msgs::Point32 new_point;
	  new_point.x = cloud.points[i].x;
	  new_point.y = cloud.points[i].y;
	  new_point.z = cloud.points[i].z;
	  if (new_point.z>closestPoint.z)
	    {
	      closestPoint = new_point;
	    }
	}
    }
}


bool add(tabletop_for_cabinet::cabinet_table::Request  &req,
         tabletop_for_cabinet::cabinet_table::Response &res )
{
  // Get pointcloud
  ros::NodeHandle nh;
  // recent_cloud in recent_cloud->header.frame_id
  sensor_msgs::PointCloud::ConstPtr recent_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud>
    ("narrow_stereo_textured/points",nh, ros::Duration(3.0));
  if (!recent_cloud)
    {
      ROS_ERROR("object_processing_server: No PointCloud");
      // res.pose = res.pose.NO_CLOUD_RECEIVED;
      return true;
    }

  // transform cloud into the frame of the table
  sensor_msgs::PointCloud cloud;
  std::string to_table_frame = req.table.pose.header.frame_id;
  listener->transformPointCloud(to_table_frame,*recent_cloud, cloud);
  // cloud in cloud.header.frame_id
  ROS_INFO("orig cloud size=%d",cloud.points.size());

  // // filter cloud to only include things within the boundary of the table
  // pcl::PointCloud<pcl::PointXYZ> object_cloud;
  // find_object_cloud(req.table, cloud, object_cloud);
  // // object_cloud in object_cloud.header.frame_id = cloud.header.frame_id
  // ROS_INFO("object_cloud size=%d", object_cloud.size());

  // //closest point turd monkey
  // geometry_msgs::Point32 closestPoint, dude;
  // find_closest_point(req.table.pose.pose,object_cloud,closestPoint);
  // //  inverse_transform_point(req.table.pose.pose,closestPoint,dude);
  // make_marker(req.table.pose, visualization_msgs::Marker::SPHERE,
  // 	      closestPoint.x,closestPoint.y,closestPoint.z);
  // ROS_INFO("closest.x=%f y=%f z=%f", closestPoint.x,closestPoint.y,closestPoint.z);
  // ROS_INFO("closest.x=%f y=%f z=%f", dude.x,dude.y,dude.z);


  //transform object cloud and publish it
  marker.header.frame_id = cloud.header.frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "clouds";
  marker.id = 3;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;

  for (size_t i =0; i<cloud.points.size();i++)
    {
      geometry_msgs::Point p;
      p.x = cloud.points[i].x;
      p.y = cloud.points[i].y;
      p.z = cloud.points[i].z;
      marker.points.push_back(p);
    }

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 1;
  marker.color.g = 0.4;
  marker.color.b = 0.4;
  marker.color.a = 1;
  //marker.lifetime = ros::Duration(30);
      
  visPub.publish(marker);


  // // find clusters on the table
  // pcl::PointCloud<pcl::PointXYZ>::ConstPtr object_cloud_ptr = 
  //   boost::make_shared<const pcl::PointCloud<pcl::PointXYZ> > (object_cloud);
  // std::vector<pcl::PointIndices> clusters;
  // create_clusters(object_cloud_ptr, clusters);
  // ROS_INFO("Number of clusters found: %d",clusters.size());

  // // create pointclouds of clusters to return
  // std::vector<sensor_msgs::PointCloud> cluster_clouds;
  // for (int i=0;i<clusters.size();i++) {
  //   sensor_msgs::PointCloud new_cloud;
  //   new_cloud.header = req.table.pose.header;
  //   for (int j=0;j<clusters[i].indices.size();j++) {
  //     geometry_msgs::Point32 new_point;
  //     new_point.x = object_cloud.points[clusters[i].indices[j]].x;
  //     new_point.y = object_cloud.points[clusters[i].indices[j]].y;
  //     new_point.z = object_cloud.points[clusters[i].indices[j]].z;
  //     new_cloud.points.push_back(new_point);
  //   }
  //   cluster_clouds.push_back(new_cloud);
  // }

  // // define the table and the clusters associated with it as the result
  // res.detection.table = req.table;
  // res.detection.clusters = cluster_clouds;

  return true;
}

/**
 * Uses tabletop detection to find the cabinet door and the clusters on the door
 *
 *@param argc
 *@param argv
 *@returns 0 
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "cabinet_table_server");
  ros::NodeHandle n;

  listener = new tf::TransformListener();

  visPub = n.advertise<visualization_msgs::Marker>("/handle_marker",0);
  ros::ServiceServer service = n.advertiseService("cabinet_table", add);
  ROS_INFO("Ready to find cabinet door.");
  ros::spin();

  delete listener;
  return 0;
}
