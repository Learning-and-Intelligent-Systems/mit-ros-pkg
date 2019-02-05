#include <ros/ros.h>
#include <furniture/All_Hulls.h>
#include <furniture/Table_Polygons.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/segmentation/extract_polygonal_prism_data.hpp>
#include <pcl_ros/transforms.h>
//#include <pcl_visualization/pcl_visualizer.h>
//#include <pcl_visualization/range_image_visualizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>


#include <string>
#include <vector>
#include <list>

#include <math.h>
#include <stdlib.h>


// eigen
#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <Eigen/LU>

using namespace std;
using namespace Eigen;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudXYZN;
typedef pcl::KdTree<pcl::PointXYZ> KdTreeXYZ;
ros::Publisher pub_chair_back;
ros::Publisher pub_chair_bottom;
tf::TransformListener *tf_listener;

typedef struct {
  PointCloudXYZN back;
  PointCloudXYZN bottom;
} ChairCloud;


/* convert a tf transform to an eigen transform */
void transformTFToEigen(const tf::Transform &t, Affine3f &k)
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

  void compute_normals(const PointCloudXYZ &cloud, PointCloudXYZN &cloud_with_normals, float radius)
  {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimation;
    KdTreeXYZ::Ptr kdtree = boost::make_shared< pcl::KdTreeFLANN<pcl::PointXYZ> >();

    kdtree->setEpsilon(0.);
    normal_estimation.setSearchMethod(kdtree);
    //normal_estimation.setKSearch(10);
    normal_estimation.setRadiusSearch(radius);
    normal_estimation.setInputCloud(cloud.makeShared());
    normal_estimation.setViewPoint(cloud.sensor_origin_(0), cloud.sensor_origin_(1), cloud.sensor_origin_(2));
    normal_estimation.compute(cloud_with_normals);

    for (uint i = 0; i < cloud.points.size(); i++) {
      cloud_with_normals.points[i].x = cloud.points[i].x;
      cloud_with_normals.points[i].y = cloud.points[i].y;
      cloud_with_normals.points[i].z = cloud.points[i].z;
    }
  }

/* adds to cloud_out the points in cloud_in that are within some height window */
static void filter_cloud(const PointCloudXYZ &cloud_in, PointCloudXYZ &cloud_out)
{
  cloud_out.height = 1;
  cloud_out.is_dense = false;
  cloud_out.points.resize(0);

  for (size_t i = 0; i < cloud_in.points.size(); i++) {
    if (cloud_in.points[i].z > .3 && cloud_in.points[i].z < 1.5)
      cloud_out.push_back(cloud_in.points[i]);
  }

  cloud_out.width = cloud_out.points.size();
}

PointCloudXYZN extract_back ( PointCloudXYZN cloud ){
  int i, j, r, iter = 80, n = cloud.points.size();
  double thresh = .1;

  double x_min = 0, y_min = 0, a_min = 0, b_min = 0, dmin = 1000*n;
  for (i = 0; i < iter; i++) {
    r = rand() % n; 
    double x1 = cloud.points[r].x;
    double y1 = cloud.points[r].y;

    r = rand() % n; 
    double b = cloud.points[r].x - x1;
    double a = y1 - cloud.points[r].y;
    
    a = a/sqrt(a*a + b*b + .001);
    b = b/sqrt(a*a + b*b + .001);

    //double c1 = y1*(x2-x1) + x1*(y1-y2);
    //double q = -x1/c + y1;

    double d = 0.0;
    for (j = 0; j < n; j++) {
      double x = cloud.points[j].x;
      double y = cloud.points[j].y;
      //double dz = fabs(c - (x - x2)/(y-y2+.0000000001));
      double dz = fabs( a*(x-x1) + b*(y-y1)  );
      if (dz > thresh)
	dz = thresh;
      d += dz;
    }

    if (d < dmin) {
      //printf("hoyo %d", dmin);
      dmin = d;
      x_min = x1;
      y_min = y1;
      a_min = a;
      b_min = b;
    }
  }

  PointCloudXYZN back;
  for (j=0; j<n; j++) {
    double x = cloud.points[j].x;
    double y = cloud.points[j].y;
    if (fabs( a_min*(x-x_min) + b_min*(y-y_min)) < thresh ){ //fabs(c_min - (x - x_min)/(y-y_min+.0000000001)) < thresh){
      //printf("hi");
      back.points.push_back(cloud.points[j]);
    }
  }
  printf("back points %d", back.points.size());
  back.width = back.points.size();
  back.height = 1;

  return back;

}

PointCloudXYZN extract_bottom ( PointCloudXYZN &cloud ){

  int i, j, iter = 70, n = cloud.points.size();
  double thresh = .04;

  double zmin = 0, dmin = 1000*n;
  for (i = 0; i < iter; i++) {

    double foo = cloud.points[rand() % n].z;

    double d = 0.0;
    for (j = 0; j < n; j++) {
      double dz = fabs(foo - cloud.points[j].z);
      if (dz > thresh)
	dz = thresh;
      d += dz;
    }

    if (d < dmin) {
      dmin = d;
      zmin = foo;
    }
  }


  PointCloudXYZN bottom;
  for (j=0; j<n; j++) {
    if (fabs(zmin - cloud.points[j].z) < thresh){
      bottom.points.push_back(cloud.points[j]);
    }
  }

  bottom.width = bottom.points.size();
  bottom.height = 1;

  return bottom;

}

// Chair objects needs to be defined (ordered pair of polygons)
ChairCloud *extract_chair(PointCloudXYZ clu){
  PointCloudXYZN cwn;
  compute_normals(clu, cwn, .03);
  PointCloudXYZN bottom, back;
  double threshold = .4;

  for (unsigned int i = 0; i < cwn.points.size(); i++){
    if (fabs(cwn.points[i].normal_z) < threshold)
    {
      back.points.push_back(cwn.points[i]);
    }
    else// if (fabs(cwn.points[i].normal_z) > 1 - 2*threshold)
      {
	bottom.points.push_back(cwn.points[i]);
      }
  }

  if (bottom.points.size() < 10 || back.points.size()< 10){
    return NULL;
  }
  
  bottom = extract_bottom(bottom);
  back = extract_back(back);  
  // If back and bottom could conceivably be part of the same chair return them, otherwise retrun null.
  double mean = 0;
  for (unsigned int i = 0; i < bottom.points.size(); i++)
    mean += bottom.points[i].z;
  mean /= bottom.points.size();

  double tot = 0;
  for (unsigned int i = 0; i < back.points.size(); i++)
    if (back.points[i].z < mean)
      tot += 1;

  if (tot/back.points.size() > .13) //less than 13% of back points can be below bottom
    return NULL;

  ChairCloud *chair = new ChairCloud();
  chair->bottom = bottom;
  chair->back = back;
  return chair;
}

//Chair models is to be defined. It is something that has a parameter 'models' whitch is a vector of chair descriptions
//External API
vector<ChairCloud> *find_chairs(sensor_msgs::PointCloud2 &msg, string target_frame, tf::TransformListener *tf_listener)
{
  vector<ChairCloud> *chair_models = new vector<ChairCloud>();
  //-> Preprocessing: 

  // get the cloud in PCL format in the right coordinate frame
  PointCloudXYZ cloud, cloud2, full_cloud;
  pcl::fromROSMsg(msg, cloud);
  if (!pcl_ros::transformPointCloud(target_frame, cloud, cloud2, *tf_listener)) 
  {
    ROS_WARN("Can't transform point cloud; aborting object detection.");
    return NULL;
  }
  full_cloud = cloud2;

  // lookup the transform from target frame to sensor
  tf::StampedTransform tf_transform;
  Eigen::Affine3f sensor_pose;
  try {
    //tf_listener->lookupTransform(msg.header.frame_id, target_frame, msg.header.stamp, tf_transform);
    tf_listener->lookupTransform(target_frame, msg.header.frame_id, msg.header.stamp, tf_transform);
    transformTFToEigen(tf_transform, sensor_pose);
  }
  catch (tf::TransformException& ex) {
    ROS_WARN("[point_cloud_callback] TF exception:\n%s", ex.what());
    return NULL;
  }

  //pcl::RangeImage full_range_image;
  //pcl::RangeImage::CoordinateFrame frame = pcl::RangeImage::CAMERA_FRAME;
  //full_range_image.createFromPointCloud(full_cloud, .2*M_PI/180, 2*M_PI, M_PI, sensor_pose, frame, 0, 0, 0);

  ROS_INFO("Got PointCloud2 msg with %lu points\n", cloud2.points.size());

  // <END Preprocessing

  filter_cloud(cloud2, cloud);
  //ROS_INFO("Points of interest number ", cloud.points.size());

  //->Clustering
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster;
  KdTreeXYZ::Ptr clusters_tree = boost::make_shared< pcl::KdTreeFLANN<pcl::PointXYZ> > ();
  clusters_tree->setEpsilon (.0001);
  cluster.setClusterTolerance (0.15);
  cluster.setMinClusterSize (100);
  cluster.setSearchMethod (clusters_tree);
  vector<pcl::PointIndices> cluster_indices;
  cluster.setInputCloud (cloud.makeShared());  //cloud2
  cluster.extract(cluster_indices);
  ROS_INFO("Found %lu clusters.", cluster_indices.size());

  // <END Clustering

  //int max = 0;

  PointCloudXYZN allBacks;
  PointCloudXYZN allBottoms;  

  for (size_t i = 0; i < cluster_indices.size(); i++) {
    PointCloudXYZ cluster;
    pcl::copyPointCloud (cloud, cluster_indices[i], cluster);
    ChairCloud* c = extract_chair(cluster);

    if (c != NULL){
      printf("publishing chair clouds\n");

      // publish c to rviz
      //sensor_msgs::PointCloud2 chair_back, chair_bottom;
      //pcl::toROSMsg(c->back, chair_back);
      //pcl::toROSMsg(c->bottom, chair_bottom);

      printf("chair back has %i points\n", c->back.points.size());
      printf("chair bottom has %i points\n", c->bottom.points.size());

      //chair_back.header.frame_id = "base_footprint";
      //chair_bottom.header.frame_id = "base_footprint";

      for(unsigned int q = 0; q < (c->bottom.points.size()); q++)
	allBottoms.points.push_back(c->bottom.points[q]);

    for(unsigned int q = 0; q < (c->back.points.size()); q++)
      allBacks.points.push_back(c->back.points[q]);

      //if (c->back.points.size() + c->bottom.points.size() > max){
      //max = c->back.points.size() + c->bottom.points.size();
      //pub_chair_back.publish(chair_back);
      //pub_chair_bottom.publish(chair_bottom);
      //}
      chair_models->push_back(*c);
    }
  }
  allBottoms.width = allBottoms.points.size();
  allBottoms.height = 1;
  allBacks.width = allBacks.points.size();
  allBacks.height = 1;

  sensor_msgs::PointCloud2 chair_back, chair_bottom;
  pcl::toROSMsg(allBacks, chair_back);
  pcl::toROSMsg(allBottoms, chair_bottom);

  chair_back.header.frame_id = "base_footprint";
  chair_bottom.header.frame_id = "base_footprint";

  pub_chair_back.publish(chair_back);
  pub_chair_bottom.publish(chair_bottom);

  return chair_models;
}

void callb(const sensor_msgs::PointCloud::ConstPtr& msg)
{
  //debug this
  sensor_msgs::PointCloud2 cloud2; 
  sensor_msgs::convertPointCloudToPointCloud2(*msg, cloud2); 	
  find_chairs(cloud2, "base_footprint", tf_listener);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "chair_detector_standalone");
  ros::NodeHandle n;
  tf_listener = new tf::TransformListener();
  pub_chair_back = n.advertise<sensor_msgs::PointCloud2>("chair_back_cloud", 1);
  pub_chair_bottom = n.advertise<sensor_msgs::PointCloud2>("chair_bottom_cloud", 1);
  ros::Subscriber sub = n.subscribe("tilt_laser_cloud",1, callb);
  
  srand(854373);

  ros::spin();
  return 0;

}
