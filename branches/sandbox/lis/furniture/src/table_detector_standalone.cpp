#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/transform_listener.h>

#include <Eigen/Core>

#include <furniture/All_Hulls.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>

#include <visualization_msgs/Marker.h>

using namespace std;


int main(int argc, char **argv) {
  // init ROS
  ros::init(argc, argv, "table_detector");
  ros::NodeHandle nh;
  ros::Publisher pub_tabletop, pub_hulls;
  pub_tabletop = nh.advertise<sensor_msgs::PointCloud2>("aggregated_planes", 1);
  pub_hulls = nh.advertise<furniture::All_Hulls>("convex_hulls", 1);
  tf::TransformListener tfListener;
  
  // Point clouds
  sensor_msgs::PointCloud new_cloud;
  sensor_msgs::PointCloud2 new_cloud2, tableROS;

  // PCL objects
  pcl::PointCloud<pcl::PointXYZ> cloud, cluster_cloud, cloud_downsampled;
  pcl::PointIndices inliers;
  pcl::ModelCoefficients coefficients;
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster;
  
  // PCL Filtering
  grid.setFilterFieldName ("z");
  grid.setLeafSize (0.005, 0.005, 0.005);
  grid.setFilterLimits (0.4, 1.1); //assuming there might be very low and very high tables
  
  // PCL Segmentation
  Eigen::Vector3f v = Eigen::Vector3f(0.0, 0.0, 1.0); // z-axis is normal
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE); //Changed name from SACMODEL_ORIENTED_PLANE... Thank you, Radu...
  seg.setAxis(v);
  seg.setEpsAngle(0.087); //approximatelly 5 degrees tolerance
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.03);
  
  // PCL Extracting indices
  extract.setNegative (false); //extract indices, NOT everything but indices
  
  // PCL Clustering
  // Parameters shamelessly stolen from table_boundary_node.cpp in package table_boundary_detector
  cluster.setClusterTolerance (0.07);
  cluster.setMinClusterSize (1000); //testing!
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ> ());
  cluster.setSearchMethod (kdtree);
  vector<pcl::PointIndices> cluster_indices;
  
  // PCL Projection
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  pcl::PointCloud<pcl::PointXYZ> cloud_projected;
  proj.setModelType (pcl::SACMODEL_PLANE);
  
  // PCL Convex Hull
  pcl::PointCloud<pcl::PointXYZ> cloud_hull;
	pcl::ConvexHull<pcl::PointXYZ> chull;
  furniture::All_Hulls hulls;
  
  //Markers
  vector<visualization_msgs::Marker> markers;
  ros::Publisher pub_marker = nh.advertise<visualization_msgs::Marker>("convex_hulls_markers", 10);
  
  int tableIDs = 0;
  
  cout << "Initialized node" << endl;
  
  while (nh.ok()) {
    
    // Spin until we get a PointCloud message
    sensor_msgs::PointCloud2ConstPtr cloud_blob_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/points2");
    //cout << "Received point cloud with " << cloud_blob_ptr->points.size() << " points" << endl;
    cout << "Received PointCloud2" << endl;
    sensor_msgs::PointCloud cloud1;
    sensor_msgs::convertPointCloud2ToPointCloud(*cloud_blob_ptr, cloud1);
    // Transform point cloud to base_footprint coordinates
    try {
      pcl::PointCloud<pcl::PointXYZ> supercloud; //supercloud made for testing
      supercloud.header.frame_id = "/base_footprint";
      
      tfListener.transformPointCloud("/base_footprint", cloud1, new_cloud);
      // Convert to PCL
      sensor_msgs::convertPointCloudToPointCloud2(new_cloud, new_cloud2);
      pcl::fromROSMsg(new_cloud2, cloud);
      
      cout << "Converted to PCL" << endl;
      
      // Downsample the data    
      grid.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud));
      grid.filter (cloud_downsampled);
      cout << "Downsampled point cloud to " << cloud_downsampled.points.size() << " points" << endl;
      
      // Clustering data
      // Possible issue: distinguish two tables and one table where the top is divided in two by the object on it (look at the table_with_computer bag files).
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_ptr = boost::make_shared <const pcl::PointCloud<pcl::PointXYZ> > (cloud_downsampled);
      cluster.setInputCloud (cloud_ptr);
      cluster.extract(cluster_indices);
      
      cout << "Found " << cluster_indices.size() << " clusters" << endl;
      
      // Process each cluster
      
      for(unsigned int i = 0; i < cluster_indices.size(); i++) {
	cout << "Processing cluster " << i <<" with size: "<< cluster_indices[i].indices.size() << endl;
	
	// Extracting points from i-th cluster
	extract.setInputCloud (cloud_ptr);
	extract.setIndices (boost::make_shared<const pcl::PointIndices> (cluster_indices[i]));
	extract.filter (cluster_cloud);
	
	// Fit a plane through the data
	seg.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cluster_cloud));
	seg.segment (inliers, coefficients);
	cout << "Segmented the plane of size " << inliers.indices.size() << endl;
	if (inliers.indices.size() > 800) {
	  
	  // Extract indices to a separate PointCloud
	  extract.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cluster_cloud));
	  extract.setIndices (boost::make_shared<pcl::PointIndices> (inliers));
	  extract.filter (cluster_cloud);
	  cout << "Extracted plane with the size " << cluster_cloud.size() << endl;
	  
	  // Find convex hull	
	  // Project the model inliers 
	  proj.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cluster_cloud));
	  proj.setModelCoefficients (boost::make_shared<pcl::ModelCoefficients> (coefficients));
	  proj.filter (cloud_projected);
	  
	  // Create a Convex Hull representation of the projected inliers
	  // TODO: Make vector of convex_hulls and publish the vector. Used to have problems with making a vector of convex hulls. Check up on that.
	  chull.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cloud_projected));
	  chull.reconstruct (cloud_hull);
	  cout << "Found convex hull with size: " << cloud_hull.points.size() << endl;
	  
	  // Create marker out of convex hull
	  visualization_msgs::Marker currentHullMarker;
	  currentHullMarker.header.frame_id = cluster_cloud.header.frame_id;
	  currentHullMarker.header.stamp = cluster_cloud.header.stamp;
	  currentHullMarker.ns = "Convex hull";
	  currentHullMarker.action = visualization_msgs::Marker::ADD;
	  currentHullMarker.pose.orientation.w = 1.0;
	  currentHullMarker.id = i;
	  currentHullMarker.type = visualization_msgs::Marker::LINE_STRIP;
	  currentHullMarker.scale.x = 0.01; // thickness
	  //currentHullMarker.scale.y = 1;
	  //currentHullMarker.scale.z = 1;
	  currentHullMarker.color.b = 1.0; // color = blue
	  currentHullMarker.color.a = 1.0; // alpha
	  
	  geometry_msgs::PolygonStamped currentHull;
	  currentHull.header=cluster_cloud.header;
	  for (unsigned int j = 0; j < cloud_hull.points.size(); j++) {
	    geometry_msgs::Point tmp; // the ugliest way to do this... too tired to think
	    geometry_msgs::Point32 tmp32;
	    tmp.x = cloud_hull.points[j].x;
	    tmp.y = cloud_hull.points[j].y;
	    tmp.z = cloud_hull.points[j].z;
	    tmp32.x = cloud_hull.points[j].x;
	    tmp32.y = cloud_hull.points[j].y;
	    tmp32.z = cloud_hull.points[j].z;
	    currentHullMarker.points.push_back(tmp);
	    currentHull.polygon.points.push_back(tmp32);
	  }
	  
	  // for testing, assembling all of the clusters into a supercloud to see whether I detect everything.
	  supercloud += cluster_cloud;
	  markers.push_back(currentHullMarker);
	  hulls.hulls.push_back(currentHull);
	  hulls.ids.push_back(tableIDs++);
	} else {
	  cout << "Segmented plane too small, discarding" << endl;
	}
      }
      cluster_indices.clear();
      
      //Publishing hulls
      pub_hulls.publish(hulls);
      
      // Publishing supercloud
      pcl::toROSMsg(supercloud, tableROS);
      pub_tabletop.publish(tableROS);
      
      // Publishing markers
      for (unsigned int i = 0; i < markers.size(); i++) {
	pub_marker.publish(markers[i]);
      }
      cout << "Published " << markers.size() << " markers" << endl;
      
      markers.clear();
      hulls.hulls.clear();
      hulls.ids.clear();
      cout << endl;
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
    }	 
  }  
}
