#include "util.h"

double angle3D(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2, geometry_msgs::Point32 p3) {
  double x1 = p1.x - p2.x;
  double x2 = p3.x - p2.x;
  double y1 = p1.y - p2.y;
  double y2 = p3.y - p2.y;
  double z1 = p1.z - p2.z;
  double z2 = p3.z - p2.z;
  double len1 = sqrt(x1*x1 + y1*y1 + z1*z1); 
  double len2 = sqrt(x2*x2 + y2*y2 + z2*z2);
  x1 /= len1; y1 /= len1; z1 /= len1;
  x2 /= len2; y2 /= len2; z2 /= len2;
  double dotProduct = x1*x2 + y1*y2 + z1*z2;
  return acos(dotProduct);
}

bool flat(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2, geometry_msgs::Point32 p3) {
  return fabs(angle3D(p1, p2, p3)-PI) < 0.15;
}

void convert_ROS_to_PCL(sensor_msgs::PointCloud cloud, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud) {
  sensor_msgs::PointCloud2 cloud2;
  sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);
  pcl::fromROSMsg(cloud2, pcl_cloud);
  ROS_INFO("Converted to PCL");
}

void convert_PCL_to_ROS(pcl::PointCloud<pcl::PointXYZ> pcl_cloud, sensor_msgs::PointCloud &cloud) {
  sensor_msgs::PointCloud2 cloud2;
  pcl::toROSMsg(pcl_cloud, cloud2);
  sensor_msgs::convertPointCloud2ToPointCloud(cloud2, cloud);
}

void estimate_normals(pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointCloud<pcl::Normal> &cloud_normals, int k_knn) {
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_est;
  normal_est.setKSearch (k_knn);  
  normal_est.setSearchMethod (boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ());
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_ptr = boost::make_shared<const pcl::PointCloud<pcl::PointXYZ> > (cloud);
  normal_est.setInputCloud(cloud_ptr);
  normal_est.compute(cloud_normals);
  ROS_INFO("Normals found");
}

void remove_non_vertical(sensor_msgs::PointCloud cloud, pcl::PointCloud<pcl::Normal> cloud_normals, sensor_msgs::PointCloud &cloud_horizontal, double normals_angle) {
  /*for (unsigned int i = 0; i < cloud_normals.points.size();) {
      if (acos(cloud_normals.points[i].normal[2]) > 0.25) {
      cloud_normals.points.erase(cloud_normals.points.begin() + i);
      cloud.points.erase(cloud.points.begin() + i);
      } else {
        i++;
      }
    }
    cloud_downsampled_normals.width = cloud_downsampled_normals.points.size();*/
  
  // Making a new point cloud because push_back() is faster than erasing a particular element from vector (the approach above).
  cloud_horizontal.header = cloud.header;
  for (unsigned int i = 0; i < cloud_normals.points.size(); i++) {
    double tmp = acos(cloud_normals.points[i].normal[2]);
    if (tmp < normals_angle || tmp > (PI-normals_angle)) { // normal vector pointing upwards or downwards
      cloud_horizontal.points.push_back(cloud.points[i]);
    }
  }
  ROS_INFO("Non-vertical surfaces removed. %ld points left", cloud.points.size());
}

void flatten_plane(pcl::PointCloud<pcl::PointXYZ> cloud, pcl::ModelCoefficients::Ptr plane_coefficients, pcl::PointCloud<pcl::PointXYZ> &cloud_projected) {
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cloud));
  proj.setModelCoefficients (plane_coefficients);
  proj.filter (cloud_projected);
}

void find_convex_hull(pcl::PointCloud<pcl::PointXYZ> cloud_projected, geometry_msgs::PolygonStamped &current_hull) {
  pcl::PointCloud<pcl::PointXYZ> cloud_hull;
  pcl::ConvexHull<pcl::PointXYZ> chull;

  // Create a Convex Hull representation of the projected inliers
  chull.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cloud_projected));
  chull.reconstruct (cloud_hull);
  ROS_INFO("Found convex hull with size: %ld", cloud_hull.points.size());
  
  // Create a polygon out of a PointCloud and simplify the hull
  current_hull.header=cloud_projected.header;
  for (unsigned int j = 0; j < cloud_hull.points.size(); j++) {
    geometry_msgs::Point32 tmp32;
    tmp32.x = cloud_hull.points[j].x;
    tmp32.y = cloud_hull.points[j].y;
    tmp32.z = cloud_hull.points[j].z;
    
    // Simplifying convex hull (if 3 points are almost colinear, the middle one can be ignored)
    int hull_size = current_hull.polygon.points.size();
    if (current_hull.polygon.points.size()>1 && flat(current_hull.polygon.points[hull_size-2], current_hull.polygon.points[hull_size-1], tmp32)) {
      current_hull.polygon.points.pop_back();
    }
    current_hull.polygon.points.push_back(tmp32);
  }
  int hull_size = current_hull.polygon.points.size();
  if (flat(current_hull.polygon.points[hull_size-2], current_hull.polygon.points[hull_size-1], current_hull.polygon.points[0])) {
    current_hull.polygon.points.pop_back();
  }
  ROS_INFO("Reduced convex hull to size: %ld", current_hull.polygon.points.size());
}

void create_markers(furniture::All_Hulls hulls, vector<visualization_msgs::Marker> &markers) {
  visualization_msgs::Marker current_hullMarker;
  current_hullMarker.ns = "Convex hull";
  current_hullMarker.action = visualization_msgs::Marker::ADD;
  current_hullMarker.pose.orientation.w = 1.0;
  current_hullMarker.type = visualization_msgs::Marker::LINE_STRIP;
  current_hullMarker.scale.x = 0.01; // thickness
  current_hullMarker.color.b = 1.0; // color = blue
  current_hullMarker.color.a = 1.0; // alpha
  
  for (unsigned i = 0; i < hulls.hulls.size(); i++) {
    current_hullMarker.points.clear();
    current_hullMarker.id = hulls.ids[i];
    current_hullMarker.header.frame_id = hulls.hulls[i].header.frame_id;
    current_hullMarker.header.stamp = hulls.hulls[i].header.stamp;
    for (unsigned j = 0; j < hulls.hulls[i].polygon.points.size(); j++) {
      geometry_msgs::Point tmp;
      tmp.x = hulls.hulls[i].polygon.points[j].x;
      tmp.y = hulls.hulls[i].polygon.points[j].y;
      tmp.z = hulls.hulls[i].polygon.points[j].z;
      current_hullMarker.points.push_back(tmp);
    }
    markers.push_back(current_hullMarker);
  }
}

void find_tables(const pcl::PointCloud<pcl::PointXYZ> &initial_cloud, furniture::Table_clouds &table_clouds,
		 furniture::All_Hulls &hulls, double z_min, double z_max, int k_knn, double plane_dist, double plane_angle,
		 int plane_min_size, int cluster_min_size, double cluster_dist, double normals_angle)
{
  int tableIDs = 0;
  
  // ROS PointClouds
  sensor_msgs::PointCloud tableROS, cloud_downsampled_ROS, cloud_horizontal_ROS;
  
  // PCL objects
  pcl::PointCloud<pcl::PointXYZ> cluster_cloud, cloud_downsampled, cloud_projected, cloud_horizontal;
  pcl::PointCloud<pcl::Normal> cloud_downsampled_normals;
  vector<pcl::PointIndices> cluster_indices;
  pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients);
  
  // Downsample the data
  downsample_cloud(initial_cloud, cloud_downsampled, z_min, z_max);
  
  // Find normals for the cluster
  estimate_normals(cloud_downsampled, cloud_downsampled_normals, k_knn);
  
  convert_PCL_to_ROS(cloud_downsampled, cloud_downsampled_ROS);
  
  remove_non_vertical(cloud_downsampled_ROS, cloud_downsampled_normals, cloud_horizontal_ROS, normals_angle);
  
  // Convert back to PCL
  convert_ROS_to_PCL(cloud_horizontal_ROS, cloud_horizontal);
  
  // Project everything to a horizontal plane so the clustering is not affected by the distances on the z-axis.
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;
  flatten_plane(cloud_horizontal, coefficients, cloud_projected);
  
  // Clustering data
  cluster_data(cloud_projected, cluster_indices, cluster_min_size, cluster_dist);

  pcl::PointIndices::Ptr indices (new pcl::PointIndices);
  // Process each cluster
  for(unsigned int i = 0; i < cluster_indices.size(); i++) {
    ROS_INFO("Processing cluster %u with size: %ld", i, cluster_indices[i].indices.size());

    // Extracting points from i-th cluster
    *indices = cluster_indices[i];
    extract_indices(cloud_horizontal, indices, cluster_cloud);
    

    // Fit a plane through the data
    segment_plane(cluster_cloud, plane_inliers, plane_coefficients, plane_dist, plane_angle, plane_min_size);
    
    if (plane_inliers->indices.size() > unsigned(plane_min_size)) {
      
      // Extract indices to a separate PointCloud
      extract_indices(cluster_cloud, plane_inliers, cluster_cloud);
      
      // Project the model inliers
      flatten_plane(cluster_cloud, plane_coefficients, cloud_projected);
      
      // Find convex hull	
      geometry_msgs::PolygonStamped current_hull;
      find_convex_hull(cloud_projected, current_hull);					
      
      // Going back to the regular PointCloud message
      convert_PCL_to_ROS(cloud_projected, tableROS);
      
      // Adding data
      table_clouds.clouds.push_back(tableROS);
      table_clouds.ids.push_back(tableIDs);
      hulls.hulls.push_back(current_hull);
      hulls.ids.push_back(tableIDs++);
      
    } else {
      ROS_WARN("Segmented plane too small, discarding");
    }
  }
}

/* convert a tf transform to an eigen3 transform */
void transformTFToEigen(const tf::Transform &t, Eigen::Affine3f &k)
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

void fit_table_models(const pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Affine3f sensor_pose,
		      const furniture::All_Hulls &hulls, furniture::Table_Poses &table_poses)
{
  // create range image from point cloud
  pcl::RangeImage range_image;
  pcl::RangeImage::CoordinateFrame frame = pcl::RangeImage::CAMERA_FRAME;
  range_image.createFromPointCloud(cloud, .2*M_PI/180, 2*M_PI, M_PI, sensor_pose, frame, 0, 0, 0);
  
  //TODO: copy optimization library from cardboard, create an optimizer
}
