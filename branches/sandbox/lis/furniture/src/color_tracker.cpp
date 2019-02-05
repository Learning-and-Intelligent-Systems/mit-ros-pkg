#include <ros/ros.h>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <visualization_msgs/MarkerArray.h>

#include <furniture/color_util.h>
#include "util.h"
#include <furniture/Table_Color_Model.h>

using namespace::std;

double l_low_weight = 0.005, a_weight = 1.0, b_weight = 1.0;
double l_high_weight = 0.03;

int64_t skipped = 0;

double variance = 60.0;

double gaussian_mean = 3.0;

vector<furniture::Table_Color_Model> models;

double l2_norm_distance_squared(pcl::PointXYZRGB p, int m_idx) {
  uint32_t lab = *reinterpret_cast<int*>(&p.rgb);
  int l = (int) (lab >> 16) & 0x0000ff;
  int a = (int) (lab >> 8)  & 0x0000ff;
  int b = (int) (lab)       & 0x0000ff;

  a = (int) (a * 220.0 / 255.0);
  b = (int) (b * 220.0 / 255.0);

  a -= 110;
  b -= 110;
  
  /*if (b > 15 && m_idx == 2) { // hack to get rid of the outliers in the black table
    ++skipped;
    return 0; 
    }*/
  furniture::Table_Color_Model *m = &models[m_idx];
    
  double l_weight;
  if (m->a_mean < 8 && m->b_mean < 8) 
    l_weight = l_high_weight;
  else 
    l_weight = l_low_weight;

  return l_weight * (l - m->l_mean) * (l - m->l_mean) + a_weight * (a - m->a_mean) * (a - m->a_mean) + b_weight * (b - m->b_mean) * (b - m->b_mean);
}

// NOTE(sanja): I used this for some testing
void add_to_variance(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, int m_idx, double &variance) {
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    // Expected distance is 0, so just add to variance. There is no root from the distance because the variance goes squared, and it is faster this way.
    variance += l2_norm_distance_squared(cloud.points[i], m_idx);
  }
}

double cloud_avg_dist(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, int m_idx) {
  double avg = 0;
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    avg += sqrt(l2_norm_distance_squared(cloud.points[i], m_idx));
  }
  avg /= cloud.points.size() - skipped;
  return avg;
}

double get_probability(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, int m_idx) {
  double x = cloud_avg_dist(cloud, m_idx);
  ROS_WARN("Avg dist from model %d: %lf", m_idx, x);
  // double prob = 1 + erf(-fabs(x - gaussian_mean) / (sqrt(2 * models[m_idx].variance)));
  x -= gaussian_mean;
  double prob = exp(-x*x / (2 * models[m_idx].variance));
  return prob;
}

void get_coordinates(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, geometry_msgs::Point &point) {
  point.x = 0;
  point.y = 0;
  point.z = 0;
  
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    point.x += cloud.points[i].x;
    point.y += cloud.points[i].y;
    point.z += cloud.points[i].z;
  }

  point.x /= cloud.points.size();
  point.y /= cloud.points.size();
  point.z /= cloud.points.size();
  point.z += 0.1;
}

visualization_msgs::Marker create_marker(geometry_msgs::Point position, double max_prob, size_t max_prob_idx) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/base_footprint";
  marker.header.stamp = ros::Time::now();
  marker.pose.position = position;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.08;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  stringstream ss;
  if (max_prob < 0.20) {
    ss << "Unknown";
  } else {
    ss << setprecision(3);
    ss << "Table " << max_prob_idx << " with prob " << max_prob;
  }
  marker.text = ss.str();
  return marker;
}

void init_models() {
  // Different variance for each model because texture/reflectivity determine sensitivity to illumination changes
  furniture::Table_Color_Model *m = new furniture::Table_Color_Model;

  m->id = 0; // Wooden table
  m->l_mean = 83;
  m->a_mean = 7;
  m->b_mean = 12;
  m->variance = variance;
  models.push_back(*m);

  m = new furniture::Table_Color_Model;
  m->id = 1; // Awkward shaped white table
  m->l_mean = 91;
  m->a_mean = 4;
  m->b_mean = 0;
  m->variance = variance;
  models.push_back(*m);

  m = new furniture::Table_Color_Model;
  m->id = 2; // Round, dark table
  // NOTE(sanja): The values are thrown off a bit because the table doesn't have uniform color (there is a wooden frame)
  m->l_mean = 70;
  m->a_mean = 7;
  m->b_mean = 0;
  m->variance = variance;
  models.push_back(*m);  
}

int main(int argc, char **argv) {
  // init ROS
  ros::init(argc, argv, "color_tracker");

  ros::NodeHandle nh;

  ros::Publisher pub, pub_cloud, pub1, pub2;
  pub = nh.advertise<visualization_msgs::MarkerArray>("table_markers", 1);
  pub1 = nh.advertise<visualization_msgs::Marker>("table1", 1);
  pub2 = nh.advertise<visualization_msgs::Marker>("table2", 1);
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("supercloud", 1);

  // Parameters
  double z_min, z_max, plane_dist, plane_angle, cluster_dist, normals_angle;
  int k_knn, plane_min_size, cluster_min_size;

  // int64_t total_points = 0;
  // double var[3] = {};

  sensor_msgs::PointCloud2 tables;
  
  tf::TransformListener tfListener;
  ros::Rate rate(15.0);

  nh.param("color_tracker/z_min", z_min, 0.5);
  nh.param("color_tracker/z_max", z_max, 0.9);
  nh.param("color_tracker/k_knn", k_knn, 20);
  nh.param("color_tracker/plane_distance_tolerance", plane_dist, 0.03);
  nh.param("color_tracker/plane_angle_tolerance", plane_angle, 0.087);
  nh.param("color_tracker/plane_min_size", plane_min_size, 2000);
  nh.param("color_tracker/cluster_min_size", cluster_min_size, 2000);
  nh.param("color_tracker/cluster_distance_tolerance", cluster_dist, 0.05);
  nh.param("color_tracker/normals_angle_tolerance", normals_angle, 0.25);

  init_models();

  ROS_INFO("Initialized node");

  while (nh.ok()) {
    ROS_INFO("Entered loop");
    // Spin until we get a PointCloud message
    sensor_msgs::PointCloud2ConstPtr cloud_blob_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/rgb/points");
    // Transform point cloud to base_footprint coordinates
    try {
      // get sensor pose
      tf::StampedTransform tf_transform;
      const string s = "/base_footprint";
      if (!tfListener.waitForTransform(s, "/openni_rgb_optical_frame", cloud_blob_ptr->header.stamp, ros::Duration(1.0))) {
	cout << cloud_blob_ptr->header.stamp << endl;
	cout << ros::Time::now() << endl;
	cout << ros::WallTime::now() << endl;
	ROS_ERROR("No transform...");
	continue;
      }

      // Transform point cloud
      tfListener.lookupTransform(s, "/openni_rgb_optical_frame", cloud_blob_ptr->header.stamp, tf_transform);
      Eigen::Affine3f sensor_pose;
      transformTFToEigen(tf_transform, sensor_pose);
      // ROS_INFO("Got cloud with %d points", cloud_blob_ptr->height * cloud_blob_ptr->width);

      pcl::PointCloud<pcl::PointXYZRGB> supercloud; //supercloud made for testing
      supercloud.header.frame_id = "/base_footprint";
 
      // Convert to PCL
      pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
      pcl::fromROSMsg(*cloud_blob_ptr, pcl_cloud);
      // ROS_INFO("Converted to PCL of size %ld", pcl_cloud.size());

      pcl::transformPointCloud(pcl_cloud, pcl_cloud, sensor_pose);
      pcl_cloud.header.frame_id = s; // PCL transform doesn't change the point cloud

      downsample_cloud(pcl_cloud, pcl_cloud, z_min, z_max);

      // Segment out the plane
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

      segment_plane(pcl_cloud, inliers, coefficients, plane_dist, plane_angle, plane_min_size);
      
      if (inliers->indices.size() == 0) continue;

      extract_indices(pcl_cloud, inliers, pcl_cloud);

      // Clustering
      std::vector<pcl::PointIndices> cluster_indices;
      cluster_data(pcl_cloud, cluster_indices, cluster_min_size, cluster_dist);

      visualization_msgs::MarkerArray marker_array;

      for (size_t i = 0; i < cluster_indices.size(); ++i) {
	
	*inliers = cluster_indices[i];
	ROS_INFO("Cluster %ld with %ld points", i, inliers->indices.size());
	pcl::PointCloud<pcl::PointXYZRGB> cluster_cloud;
	extract_indices(pcl_cloud, inliers, cluster_cloud);

	supercloud += cluster_cloud;
	
	// Convert to lab
	furniture::convert_rgb_to_lab(cluster_cloud);
	
	skipped = 0;
	double max_prob = 0;
	size_t max_prob_idx = -1;
	for (size_t j = 0; j < models.size(); ++j) {
	  // add_to_variance(pcl_cloud, i, var[i]);
	  // ROS_WARN("New variance for model %d: %lf", i, var[i]/(total_points - skipped));
	  // ROS_INFO("Probability of a model %ld: %lf", j, get_probability(pcl_cloud, j));
	  double tmp = get_probability(cluster_cloud, j);
	  if (tmp > max_prob) {
	    max_prob = tmp;
	    max_prob_idx = j;
	  }
	}
	geometry_msgs::Point marker_point;
	get_coordinates(cluster_cloud, marker_point);
	marker_array.markers.push_back(create_marker(marker_point, max_prob, max_prob_idx));
	// total_points += pcl_cloud.points.size();
      }
      if (marker_array.markers.size() > 0)
	pub1.publish(marker_array.markers[0]);
      if (marker_array.markers.size() > 1)
	pub2.publish(marker_array.markers[1]);
      pub.publish(marker_array);
      // Publishing supercloud
      pcl::toROSMsg(supercloud, tables);
      pub_cloud.publish(tables);

    } catch (tf::TransformException& ex) {
      ROS_WARN("[point_cloud_callback] TF exception:\n%s", ex.what());
    }
  }
}

