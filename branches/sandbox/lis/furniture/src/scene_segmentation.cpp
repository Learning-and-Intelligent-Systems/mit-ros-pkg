#include <furniture/common.h>
#include <furniture/segmentation_util.h>
#include <furniture/pcl_common.h>
#include <furniture/color_util.h>

#include <cstdlib>
#include <ctime>
#include <iostream>

using namespace std;

const long SZ = 310000;
const long W = 640;

long rank[SZ];
long setSize[SZ];
long prev[SZ];
long avgColor[SZ][3];
double avgNorm[SZ][3];

uint8_t recolorInfo[SZ][3];

double color_thresh, norm_thresh, dist_thresh;

long findSet(long k) {
  if (prev[k] != k)
    prev[k] = findSet(prev[k]);
  return prev[k];
}

void makeUnion(long setK, long setL) {
  if (setK == setL) return;
  if (rank[setK] > rank[setL]) {
    prev[setL] = setK;
    for (int i = 0; i < 3; ++i) {
      avgColor[setK][i] += avgColor[setL][i];
      avgNorm[setK][i] += avgNorm[setL][i];
    }
    setSize[setK] += setSize[setL];
  } else {
    prev[setK] = setL;
    for (int i = 0; i < 3; ++i) {
      avgColor[setL][i] += avgColor[setK][i];
      avgNorm[setL][i] += avgNorm[setK][i];
    }
    setSize[setL] += setSize[setK];
    if (rank[setK] == rank[setL])
      ++rank[setL];
  }
}

double l2DistSq(double p1[3], double p2[3]) {
  return (p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]) + (p1[2] - p2[2]) * (p1[2] - p2[2]);
}

double normDiff(double p1[3], double p2[3]) {
  double scalar = p1[0] * p2[0] + p1[1] * p2[1] + p1[2] * p2[2];
  double intensity = (p1[0] * p1[0] + p1[1] * p1[1] + p1[2] * p1[2]) * (p2[0] * p2[0] + p2[1] * p2[1] + p2[2] * p2[2]);
  return acos(scalar / sqrt(intensity));
}

bool dist(long setA, long setB) {
  double p1[3], p2[3];
  //ROS_INFO("Sets: %d %d", setA, setB);
  //ROS_INFO("Avgs: %ld %ld %ld %ld %ld %ld", avgColor[setA][0], avgColor[setA][1], avgColor[setA][2], avgColor[setB][0], avgColor[setB][1], avgColor[setB][2]);
  //ROS_INFO("Sizes: %ld %ld", setSize[setA], setSize[setB]);
  for (long i = 0; i < 3; ++i) {
    p1[i] = avgColor[setA][i] / setSize[setA];
    p2[i] = avgColor[setB][i] / setSize[setB];
  }
  if (l2DistSq(p1, p2) > color_thresh) return false;
  for (long i = 0; i < 3; ++i) {
    p1[i] = avgNorm[setA][i] / setSize[setA];
    p2[i] = avgNorm[setB][i] / setSize[setB];
  }
  //cout << p1[0] << " " << p1[1] << " " << p1[2] << endl;
  //cout << p2[0] << " " << p2[1] << " " << p2[2] << endl;
  //cout << normDiff(p1, p2) << endl << endl;
  if (normDiff (p1, p2) > norm_thresh) return false;
  return true;
}

//void disjoint_sets(pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> &kdTree, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, const double color_thresh, const double norm_thresh) {
void disjoint_sets(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud) {
  /*int k = 3;
  std::vector<int> nearest(k);
  std::vector<float> nearestDistSq(k);*/
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    prev[i] = i;
    rank[i] = 0;
    setSize[i] = 1;
    if (furniture::validPoint(cloud.points[i])) {
      int c[3];
      furniture::get_color(cloud.points[i], c[0], c[1], c[2]);
      for (int j = 0; j < 3; ++j) {
	avgColor[i][j] = c[j];
      }
      if (furniture::validNormals(cloud.points[i])) {
	  avgNorm[i][0] = cloud.points[i].normal_x;
	  avgNorm[i][1] = cloud.points[i].normal_y;
	  avgNorm[i][2] = cloud.points[i].normal_z;
      } else {
	avgNorm[i][0] = 0;
	avgNorm[i][1] = 0;
	avgNorm[i][2] = 0;
      }
      // ROS_INFO("Init color: %ld %ld %ld", avgColor[i][0], avgColor[i][1], avgColor[i][2]);
      // ROS_INFO("Init norm: %lf %lf %lf", avgNorm[i][0], avgNorm[i][1], avgNorm[i][2]);
    } else {
      for (int j = 0; j < 3; ++j) {
	avgColor[i][j] = 0;
	avgNorm[i][j] = 0;
      } 
    }
  }
  //double p1[3];
  //double p2[3];
  pcl::OrganizedDataIndex<pcl::PointXYZRGBNormal> org_index;
  int k = 20;
  std::vector<int> nearest(k);
  std::vector<float> nearestDistSq(k); // PCL wants floats...
  org_index.setInputCloud(cloud.makeShared());
  
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    if (!furniture::validPoint(cloud.points[i]) || !furniture::validNormals(cloud.points[i])) continue;
    //ROS_INFO("Processing %d out of %d", (int) i, (int) cloud.points.size());
    //kdTree.nearestKSearch(i, k, nearest, nearestDistSq);
    //for (size_t j = 0; j < nearest.size(); ++j) {
    org_index.radiusSearch(i, dist_thresh, nearest, nearestDistSq, k);
    for (int j = 0; j < (int) nearest.size(); ++j) {
      if (!furniture::validPoint(cloud.points[nearest[j]]) && !furniture::validNormals(cloud.points[nearest[j]])) continue;
      long setA = findSet(i); // cannot move out of loop, set might change, and if it doesn't, findSet is very cheap.
      long setB = findSet(nearest[j]);
      if (setA == setB) continue;
      if (dist(setA, setB)) {
	makeUnion(setA, setB);
      }
    }
    nearest.clear();
    nearestDistSq.clear();

    /* p1[0] = cloud.points[i].x;
    p1[1] = cloud.points[i].y;
    p1[2] = cloud.points[i].z;
    for (long dW = -2; dW <= 2; ++dW) {
      for (long di = -2; di <= 2; ++di) {
	long j = i + dW * W + di;
	if (j < 0) continue;
	if (j > (long) cloud.points.size()) continue;
	if (!furniture::validPoint(cloud.points[j])) continue;
	p2[0] = cloud.points[j].x;
	p2[1] = cloud.points[j].y;
	p2[2] = cloud.points[j].z;
	//if (nearestDistSq[j] > 0.0064) {
	if (l2DistSq(p1, p2) < dist_thresh) {
	  long setA = findSet(i);
	  long setB = findSet(j);
	  // int setB = findSet(nearest[j]);
	  if (setA == setB) continue;
	  if (dist(setA, setB)) {
	    makeUnion(setA, setB);
	  }  
	}
      }
      }*/
    //nearest.clear();
    //nearestDistSq.clear();
  }
}

void recolor_cloud(const pcl::PointCloud<pcl::PointXYZRGBNormal> &input, pcl::PointCloud<pcl::PointXYZRGBNormal> &output) {
  for (size_t i = 0; i < input.points.size(); ++i) {
    if (prev[i] == (int) i) {
      recolorInfo[i][0] = rand() & 0xFF; // mod 256 - to get 0-255 value
      recolorInfo[i][1] = rand() & 0xFF;
      recolorInfo[i][2] = rand() & 0xFF;
    }
  }
  for (size_t i = 0; i < input.points.size(); ++i) {
    furniture::set_color(output.points[i], recolorInfo[prev[i]][0], recolorInfo[prev[i]][1], recolorInfo[prev[i]][2]);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "scene_segmentation");

  ros::NodeHandle nh;

  tf::TransformListener tfListener;
  ros::Rate rate(15.0);

  ros::Publisher pub_scene;
  pub_scene = nh.advertise<sensor_msgs::PointCloud2>("segmented_scene", 1);

  // Parameters
  bool true_color;
  
  nh.param("scene_segmentation/true_color", true_color, false);
  nh.param("scene_segmentation/color_threshold", color_thresh, 1000.0);
  nh.param("scene_segmentation/normal_threshold", norm_thresh, 1000.0);
  nh.param("scene_segmentation/distance_threshold", dist_thresh, 1000.0);
  //dist_thresh *= dist_thresh;
  norm_thresh *= PI / 180;

  srand(time(NULL));

  while (nh.ok()) {
    // Spin until we get a PointCloud message
    sensor_msgs::PointCloud2ConstPtr cloud_blob_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/rgb/points");
    // Transform point cloud to base_footprint coordinates
    try {
      ROS_INFO("Starting!");
      const string s = "/openni_rgb_optical_frame";
      // get sensor pose
      /*tf::StampedTransform tf_transform;
      const string s = "/base_footprint";
      if (!tfListener.waitForTransform(s, "/openni_rgb_optical_frame", cloud_blob_ptr->header.stamp, ros::Duration(1.0))) {
	continue;
      }

      // Prepare transfort
      tfListener.lookupTransform(s, "/openni_rgb_optical_frame", cloud_blob_ptr->header.stamp, tf_transform);
      Eigen::Affine3f sensor_pose;
      furniture::transformTFToEigen(tf_transform, sensor_pose);
      */
      // Preprocess PCL
      pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
      pcl::PointCloud<pcl::PointXYZRGBNormal> pcl_cloud_normals;
      pcl::fromROSMsg(*cloud_blob_ptr, pcl_cloud);
      sensor_msgs::PointCloud cloud1;
      sensor_msgs::convertPointCloud2ToPointCloud(*cloud_blob_ptr, cloud1);
      //pcl::transformPointCloud(pcl_cloud, pcl_cloud, sensor_pose);
      pcl_cloud.header.frame_id = s; // PCL transform doesn't change the point cloud header
      //furniture::downsample_cloud(pcl_cloud, pcl_cloud, 0.1, 10); // limits to get rid of the floor
      //int k = 25;
      //double radius = 0.05;
      //furniture::compute_normals(pcl_cloud, pcl_cloud_normals, k);
      furniture::normals_integral_image(pcl_cloud, pcl_cloud_normals);
      // Convert to lab
      furniture::convert_rgb_to_lab(pcl_cloud_normals);

      disjoint_sets(pcl_cloud_normals);
      for (size_t i = 0; i < pcl_cloud_normals.points.size(); ++i) {
	findSet(i);
      }
      pcl::PointCloud<pcl::PointXYZRGBNormal> output_cloud = pcl_cloud_normals;
      recolor_cloud(pcl_cloud_normals, output_cloud);
      sensor_msgs::PointCloud2 ros_cloud;
      pcl::toROSMsg(output_cloud, ros_cloud);
      pub_scene.publish(ros_cloud);
      ROS_INFO("Done!");
    } catch (tf::TransformException& ex) {
      ROS_WARN("[point_cloud_callback] TF exception:\n%s", ex.what());
    }
         
 }


}


