#include <vector>
#include <iostream>

#include "util.h"

#include <tf/transform_listener.h>




//---------------------------- HELPER FUNCTIONS ---------------------------//

/* convert a tf transform to an eigen3 transform */
void transformTFToEigen3(const tf::Transform &t, Eigen3::Affine3f &k)
{
  for(int i=0; i<3; i++)
    {
      k.matrix()(i,3) = t.getOrigin()[i];
      for(int j=0; j<3; j++)
	{
	  k.matrix()(i,j) = t.getBasis()[i][j];
	}
    }
  // Fill in identity in last row
  for (int col = 0 ; col < 3; col ++)
    k.matrix()(3, col) = 0;
  k.matrix()(3,3) = 1;
}



int main(int argc, char **argv) {
  // init ROS
  ros::init(argc, argv, "table_detector");
  ros::NodeHandle nh;
  ros::Publisher pub_hulls, pub_clouds, pub_poses;
	
  pub_hulls = nh.advertise<furniture::All_Hulls>("convex_hulls", 1);
  pub_clouds = nh.advertise<furniture::Table_clouds>("table_clouds", 1);
  pub_poses = nh.advertise<furniture::Table_Poses>("table_poses", 1);
  ros::Publisher pub_marker = nh.advertise<visualization_msgs::Marker>("convex_hulls_markers", 10);
  
  tf::TransformListener tfListener;
  
  // Parameters
  double z_min, z_max, plane_dist, plane_angle, cluster_dist, normals_angle;
  int k_knn, plane_min_size, cluster_min_size;
  
  nh.param("table_detector/z_min", z_min, 0.4);
  nh.param("table_detector/z_max", z_max, 1.1);
  nh.param("table_detector/k_knn", k_knn, 20);
  nh.param("table_detector/plane_distance_tolerance", plane_dist, 0.03);
  nh.param("table_detector/plane_angle_tolerance", plane_angle, 0.087);
  nh.param("table_detector/plane_min_size", plane_min_size, 300);
  nh.param("table_detector/cluster_min_size", cluster_min_size, 300);
  nh.param("table_detector/cluster_distance_tolerance", cluster_dist, 0.3);
  nh.param("table_detector/normals_angle_tolerance", normals_angle, 0.25);
	
  // ROS point clouds
  sensor_msgs::PointCloud new_cloud;
  furniture::Table_clouds table_clouds;

  // Markers
  vector<visualization_msgs::Marker> markers;
  
  // Convex Hull
  furniture::All_Hulls hulls;		

  // Table model poses
  furniture::Table_Poses table_poses;
  
  ROS_INFO("Initialized node");
  
  while (nh.ok()) {
    
    // Spin until we get a PointCloud message
    sensor_msgs::PointCloudConstPtr cloud_blob_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud>("/points");
    //sensor_msgs::PointCloudConstPtr cloud_blob_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud>("/tilt_laser_cloud");
    ROS_INFO("Received point cloud with %ld points", cloud_blob_ptr->points.size());
    
    // Transform point cloud to base_footprint coordinates
    try {
      
      // Supercloud made for testing
      pcl::PointCloud<pcl::PointXYZ> supercloud; 
      supercloud.header.frame_id = "/base_footprint";
      
      tfListener.transformPointCloud("/base_footprint", *cloud_blob_ptr, new_cloud);
      
      // Convert to PCL
      pcl::PointCloud<pcl::PointXYZ> initial_cloud;
      convert_ROS_to_PCL(new_cloud, initial_cloud);
  
      find_tables(initial_cloud, table_clouds, hulls, z_min, z_max, k_knn, plane_dist,
		  plane_angle, plane_min_size, cluster_min_size, cluster_dist, normals_angle);


      // get sensor pose
      tf::StampedTransform tf_transform;
      Eigen3::Affine3f sensor_pose;
      try {
	//tfListener->lookupTransform(msg.header.frame_id, target_frame, msg.header.stamp, tf_transform);
	tfListener.lookupTransform("/base_footprint", cloud_blob_ptr->header.frame_id, cloud_blob_ptr->header.stamp, tf_transform);
	transformTFToEigen3(tf_transform, sensor_pose);

	// fit table models
	fit_table_models(initial_cloud, sensor_pose, hulls, table_poses);
      }
      catch (tf::TransformException& ex) {
	ROS_WARN("[point_cloud_callback] TF exception:\n%s", ex.what());
      }

      create_markers(hulls, markers);			
      
      //Publishing hulls and corresponding clouds
      pub_hulls.publish(hulls);
      pub_clouds.publish(table_clouds);
      pub_poses.publish(table_poses);
      
      // Publishing markers
      for (unsigned int i = 0; i < markers.size(); i++) {
	pub_marker.publish(markers[i]);
      }
      ROS_INFO("Published %ld markers", markers.size());
      
      // Prepare the variables for next message
      markers.clear();
      hulls.hulls.clear();
      hulls.ids.clear();
      table_clouds.clouds.clear();
      table_clouds.ids.clear();
      cout << endl;
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
    }	
  }
}
