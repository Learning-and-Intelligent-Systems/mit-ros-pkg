#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include <ros/ros.h>
#include <furniture/All_Hulls.h>
#include <furniture/Table_Polygons.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
//#include <pcl/features/feature.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/features/principal_curvatures.h>
//#include <pcl/features/fpfh.h>
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
typedef pcl::KdTree<pcl::PointXYZ> KdTreeXYZ;
ros::Publisher pub_chair_back;
ros::Publisher pub_chair_bottom;
ros::Publisher pub_chair;
tf::TransformListener *tf_listener;

typedef struct {
  float d1; // (with d2) vector perpendicular to a regression line
  float d2; // through the projection of the points onto the x-y plane
  float meanX; //average value of X
  float meanY; //average value of Y
  float meanZ; //average value of Z
  float dz; //spread in Z direction
  float spread; //largest spread in X-Y plane
  PointCloudXYZ cloud; //the cloud itself
} Feature; //A set of numbers with all relevant information about a point cloud

typedef struct {
  Feature back;
  Feature bottom;
} ChairCloud;

typedef struct { 
  int back;
  int bottom;
  double weight;
} Match;

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
    //KdTreeXYZ::Ptr kdtree = boost::make_shared< pcl::KdTreeFLANN<pcl::PointXYZ> >();
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ> ());

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
    if (cloud_in.points[i].z > .2 && cloud_in.points[i].z < 1.5)
      cloud_out.push_back(cloud_in.points[i]);
  }
  //TODO: downsample cloud
  cloud_out.width = cloud_out.points.size();
}
vector<Feature> *extract_backs(PointCloudXYZ cloud){
  
  vector<Feature> *backs = new vector<Feature>;
  int i, j, r, iter = 80, n = cloud.points.size();
  double thresh = .05, wthresh = .4;
  
  while(n > 30){
    double x_min = 0, y_min = 0, z_min = 0, a_min = 0, b_min = 0, dmin = 1000*n;

  for (i = 0; i < iter; i++) {
    r = rand() % n; 
    double x1 = cloud.points[r].x;
    double y1 = cloud.points[r].y;
    double z1 = cloud.points[r].z;

    r = rand() % n; 
    double b = cloud.points[r].x - x1;
    double a = y1 - cloud.points[r].y;
    
    if (a==0&&b==0){ //picked same point
      i--;
      continue;
    }
    
    a = a/sqrt(a*a + b*b);
    b = b/sqrt(a*a + b*b);

    //double c1 = y1*(x2-x1) + x1*(y1-y2);
    //double q = -x1/c + y1;

    double d = 0.0;
    for (j = 0; j < n; j++) {
      double x = cloud.points[j].x;
      double y = cloud.points[j].y;
      double z = cloud.points[j].z;
      //double dz = fabs(c - (x - x2)/(y-y2+.0000000001));
      double dz = fabs( a*(x-x1) + b*(y-y1)  );
      if (dz > thresh || sqrt((x-x1)*(x-x1) + (y-y1)*(y-y1) + (z-z1)*(z-z1)) > wthresh)
	dz = thresh;
      d += dz;
    }

    if (d < dmin) {
      //printf("hoyo %d", dmin);
      dmin = d;
      x_min = x1;
      y_min = y1;
      z_min = z1;
      a_min = a;
      b_min = b;
    }
  }

  PointCloudXYZ back, other;
  for (j=0; j<n; j++) {
    double x = cloud.points[j].x;
    double y = cloud.points[j].y;
    double z = cloud.points[j].z;
    double distance = sqrt((x-x_min)*(x-x_min) + (y-y_min)*(y-y_min) + (z-z_min)*(z-z_min));
    if (fabs( a_min*(x-x_min) + b_min*(y-y_min)) < thresh && distance < wthresh){ //fabs(c_min - (x - x_min)/(y-y_min+.0000000001)) < thresh){
      //printf("hi");
      back.points.push_back(cloud.points[j]);
    }
    else
      other.points.push_back(cloud.points[j]);
  }

  back.width = back.points.size();
  back.height = 1;

  double max = -9999, min = 9999, aveX = 0, aveY = 0, aveZ = 0, spr = 0;
  for( unsigned int i = 0; i < back.points.size(); i++){
    double z = back.points[i].z;
    double y = back.points[i].y;
    double x = back.points[i].x;
    aveX += x;
    aveY += y;
    aveZ += z;
    if ( (x-x_min)*(x-x_min) + (y-y_min)*(y-y_min) > spr )
      spr = (x-x_min)*(x-x_min) + (y-y_min)*(y-y_min);
    if (z > max)
      max = z;
    if (z < min)
      min = z;
  } 
  if (back.points.size() > 20 && max - min > .2){
    Feature feat;
    feat.d1 = a_min;
    feat.d2 = b_min;
    feat.cloud = back;
    feat.meanX = aveX/back.points.size();
    feat.meanY = aveY/back.points.size();
    feat.meanZ = aveZ/back.points.size();
    feat.dz = max - min;
    feat.spread = sqrt(spr);
    feat.cloud = back;
    backs->push_back(feat);
    cloud = other;
    n = cloud.points.size();
  }
  else
    return backs;
  }
  return backs;
}
vector<Feature> *extract_bottoms(PointCloudXYZ cloud){
  vector<Feature> *bottoms = new vector<Feature>;
  int iter = 70, n = cloud.points.size();
  double thresh = .04, wthresh = .08;

  while (n > 30){
    double zmin = 0, dmin = 1000*n, xmin=0, ymin=0;
    for (int i = 0; i < iter; i++) {

      double r = rand() % n;
      double foo = cloud.points[r].z;
      double x = cloud.points[r].x;
      double y = cloud.points[r].y;

      double d = 0.0;
      for (int j = 0; j < n; j++) {
	double dz = fabs(foo - cloud.points[j].z);
	double tmp = (cloud.points[j].x-x)*(cloud.points[j].x-x) + (cloud.points[j].y-y)*(cloud.points[j].y-y);
	if (dz > thresh || tmp > wthresh)
	  dz = thresh;
	d += dz;
      }

      if (d < dmin) {
	dmin = d;
	zmin = foo;
	xmin = x;
	ymin = y;
      }
    }

    PointCloudXYZ bottom, remains;

    for (int j=0; j<n; j++) {
      if (fabs(zmin - cloud.points[j].z) < thresh && (cloud.points[j].x-xmin)*(cloud.points[j].x-xmin) + (cloud.points[j].y-ymin)*(cloud.points[j].y-ymin) < wthresh){
	bottom.points.push_back(cloud.points[j]);
      }
      else {
	remains.push_back(cloud.points[j]);		  
      }
    }
    bottom.width = bottom.points.size();
    bottom.height = 1;
    double xmx = -9999, ymx = -9999, zmx = -9999, xmn = 9999, ymn = 9999, zmn = 9999, aveX = 0, aveY = 0, aveZ = 0, spr = 0;
    for (unsigned int j = 0; j < bottom.points.size(); j++){
      double x = bottom.points[j].x;
      double y = bottom.points[j].y;
      double z = bottom.points[j].z;
      aveX += x;
      aveY += y;
      aveZ += z;
      if ( (x-xmin)*(x-xmin) + (y-ymin)*(y-ymin) > spr )
	spr = (x-xmin)*(x-xmin) + (y-ymin)*(y-ymin);
      if (x > xmx) 
	xmx = x;
      if (x < xmn)
	xmn = x;
      if (y > ymx)
	ymx = y;
      if (y < ymn)
	ymn = y;
      if (z > zmx)
	zmx = z;
      if (z < zmn)
	zmn = z;
    }


    if (bottom.points.size() > 20 && xmx-xmn > .1 && ymx-ymn > .1){
    Feature feat;
    feat.d1 = 0;
    feat.d2 = 0; //does not matter for bottoms
    feat.cloud = bottom;
    feat.meanX = aveX/bottom.points.size();
    feat.meanY = aveY/bottom.points.size();
    feat.meanZ = aveZ/bottom.points.size();
    feat.dz = zmx - zmn;
    feat.spread = sqrt(spr);
    feat.cloud = bottom;
    bottoms->push_back(feat);
      cloud = remains;
      n = cloud.points.size();
    }
    else
     return bottoms;
    }
    return bottoms;
  }

double stdErr(double val, double ave, double var){
  return (val-ave)*(val-ave)/var;
}

double scoreFeatures(Feature *backptr, Feature *bottomptr){
  Feature back = *backptr;
  Feature bottom = *bottomptr;
  double score = 0;
  /* Computed values for average chair
average     |  variance          |  feature
0.443571428571  |  0.0050873877551  |  bottom height
0.209782262617  |  0.00373997371985  |  back bottom difference in xy plane
0.123714285714  |  0.00223363265306  |  height difference b/n back and bottom
0.476714285714  |  0.0100547755102  |  spread of the back in xy
0.281857142857  |  1.26530612245e-05  |  spread of the bottom in xy
0.441571428571  |  0.00420853061224  |  spread of back in z
0.0704285714286  |  3.51020408163e-05  |  spread of bottom in z
   */

  score += stdErr( bottom.meanZ, 0.443571428571,  0.0150873877551);
  score += stdErr( sqrt( (back.meanX-bottom.meanX)*(back.meanX-bottom.meanX)+(back.meanY-bottom.meanY)*(back.meanY-bottom.meanY) ), 0.309782262617,  0.00373997371985);
  score += stdErr( back.meanZ - bottom.meanZ, 0.223714285714, 0.00223363265306);
  score += stdErr( back.spread , 0.476714285714, 0.0100547755102);
  //score += stdErr( bottom.spread, 0.281857142857, .0000126530612245);
  score += stdErr( back.dz, 0.441571428571, 0.00420853061224);
  score += stdErr( bottom.dz, 0.0704285714286,  .000351020408163);

  return score;

}

//Matching is currently done greedily, that needs to change for this to work properly.
vector<ChairCloud> *extract_chair(PointCloudXYZ cloud){

  vector<ChairCloud> *chairs = new vector<ChairCloud>();
  vector<Feature> backs = *extract_backs(cloud);
  vector<Feature> bottoms = *extract_bottoms(cloud);
  //printf("Finished extracting planes \n");
  /*printf("The %i backs: \n", backs.size());
  for (unsigned int i = 0; i <backs.size(); i ++){
    Feature f = backs[i];   
    printf("\t %.3f %.3f %.3f %.3f %.3f %.3f %.3f %i\n", f.d1, f.d2, f.meanX, f.meanY, f.meanZ, f.dz, f.spread, f.cloud.size());
  }
  printf("The %i bottoms: \n", bottoms.size());
  for (unsigned int i = 0; i <bottoms.size(); i ++){
    Feature f = bottoms[i];   
    printf("\t %.3f %.3f %.3f %.3f %.3f %.3f %.3f %i\n", f.d1, f.d2, f.meanX, f.meanY, f.meanZ, f.dz, f.spread, f.cloud.size());
    }*/
											
  vector<Match> allPairs;
  if (backs.size() == 0 || bottoms.size() == 0)
    return chairs;

  int back=0, bottom=0;
  double bestw=9999999;
  for (unsigned int i = 0; i < backs.size(); i++){
    for (unsigned int j = 0; j < bottoms.size(); j++){
      //double w = score(backs[i], bottoms[j]);
      //if (w > bestw){
      //bestw = w;
      // back = i;
      //bottom = j;
      //}
      Match m;
      m.back = i;
      m.bottom = j;
      m.weight = scoreFeatures(&backs[i], &bottoms[j]);
      allPairs.push_back(m);
    }
  }
  //printf("The highest score was %g", bestw);
  //printf("the combo was %i ", back);
  //printf("and %i \n", bottom);
  for (unsigned int k = 0; k < min(backs.size(),bottoms.size()); k++){
    for (unsigned int i = 0; i < allPairs.size(); i++){
      if (allPairs[i].weight < bestw){
	bestw = allPairs[i].weight;
	back = allPairs[i].back;
	bottom = allPairs[i].bottom;
      }
    }
    // printf( "best: %g back: %i, bottom: %i", bestw
    if (bestw < 6){
      ChairCloud c;
      c.back = backs[back];
      c.bottom = bottoms[bottom];
      chairs->push_back(c);
      //printf("Matched pair: back %i, bottom %i", back, bottom);
      for (unsigned int i = 0; i < allPairs.size(); i++)
	if (allPairs[i].back == back || allPairs[i].bottom == bottom)
	  allPairs[i].weight = 99999;
      printf("New match: score=%.3f a=%.3f b= %.3f angle=%.3f bspr=%.2f \n",bestw,backs[back].d1,backs[back].d2, atan2(backs[back].d2/backs[back].d1,1) - atan2(backs[back].meanY-bottoms[bottom].meanY , backs[back].meanX-bottoms[bottom].meanX), bottoms[bottom].spread );
      bestw = 999999;
    }
    else
      break;
  }
  printf("\n %i chairs found in cluster out of a possible %i\n", chairs->size(), min(backs.size(), bottoms.size()));
  return chairs;
}
 

//Chair models is to be defined. It is something that has a parameter 'models' whitch is a vector of chair descriptions
//External API
vector<ChairCloud> *find_chairs(const sensor_msgs::PointCloud2 &msg, string target_frame, tf::TransformListener *tf_listener)
{
  printf("Starting \n");
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
  //KdTreeXYZ::Ptr clusters_tree = boost::make_shared< pcl::KdTreeFLANN<pcl::PointXYZ> > ();
  pcl::search::KdTree<pcl::PointXYZ>::Ptr clusters_tree (new pcl::search::KdTree<pcl::PointXYZ> ());
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

  PointCloudXYZ allBacks;
  PointCloudXYZ allBottoms; 

  for (size_t i = 0; i < cluster_indices.size(); i++) {

    //if (i != 0)
    //  continue;
 
    PointCloudXYZ cluster;
    pcl::copyPointCloud (cloud, cluster_indices[i], cluster);
    vector<ChairCloud> vc = *extract_chair(cluster);

    for (unsigned int tmp = 0; tmp < vc.size(); tmp++){
    //  if (tmp != 0)
    //   continue;
      ChairCloud c = vc[tmp];
  //if (c != NULL){
      printf("publishing chair clouds\n");

      PointCloudXYZ singleChair;
      // publish c to rviz
      //sensor_msgs::PointCloud2 chair_back, chair_bottom;
      //pcl::toROSMsg(c->back, chair_back);
      //pcl::toROSMsg(c->bottom, chair_bottom);

      printf("chair back has %i points\n", c.back.cloud.points.size());
      printf("chair bottom has %i points\n", c.bottom.cloud.points.size());

      //chair_back.header.frame_id = "base_footprint";
      //chair_bottom.header.frame_id = "base_footprint";

      for(unsigned int q = 0; q < (c.bottom.cloud.points.size()); q++){
        //c.bottom.cloud.points[q].z += tmp/2.0;
	allBottoms.points.push_back(c.bottom.cloud.points[q]);
	singleChair.points.push_back(c.bottom.cloud.points[q]);
      }
      for(unsigned int q = 0; q < (c.back.cloud.points.size()); q++){
	//c.back.cloud.points[q].z += tmp/2.0;
        allBacks.points.push_back(c.back.cloud.points[q]);
	singleChair.points.push_back(c.back.cloud.points[q]);
      }

      //if (c->back.points.size() + c->bottom.points.size() > max){
      //max = c->back.points.size() + c->bottom.points.size();
      //pub_chair_back.publish(chair_back);
      //pub_chair_bottom.publish(chair_bottom);
      //}
      sensor_msgs::PointCloud2 chair_full;
      chair_models->push_back(c);
      singleChair.width = singleChair.points.size();
      singleChair.height = 1;
      pcl::toROSMsg(singleChair, chair_full);
      chair_full.header.frame_id = "base_footprint";
      pub_chair.publish(chair_full);
  //}
    
    }
  }
  allBottoms.width = allBottoms.points.size();
  allBottoms.height = 1;
  allBacks.width = allBacks.points.size();
  allBacks.height = 1;

  sensor_msgs::PointCloud2 chair_back, chair_bottom;
  //This publishes all the backs in one cloud and all the botoms in another cloud. To make this useful we need to publish chairs separately
  pcl::toROSMsg(allBacks, chair_back);
  pcl::toROSMsg(allBottoms, chair_bottom);

  chair_back.header.frame_id = "base_footprint";
  chair_bottom.header.frame_id = "base_footprint";

  pub_chair_back.publish(chair_back);
  pub_chair_bottom.publish(chair_bottom);
  printf("Found and published %i chair models \n\n", chair_models->size());
  return chair_models;
}

void callb(const sensor_msgs::PointCloud::ConstPtr& msg)
{
  //debug this
  sensor_msgs::PointCloud2 cloud2; 
  sensor_msgs::convertPointCloudToPointCloud2(*msg, cloud2); 	
  find_chairs(cloud2, "base_footprint", tf_listener);
}
void callb2(const sensor_msgs::PointCloud2::ConstPtr& msg)
{	
  find_chairs(*msg, "base_footprint", tf_listener);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "chair_detector_standalone");
  ros::NodeHandle n;
  tf_listener = new tf::TransformListener();
  pub_chair_back = n.advertise<sensor_msgs::PointCloud2>("chair_back_cloud", 1);
  pub_chair_bottom = n.advertise<sensor_msgs::PointCloud2>("chair_bottom_cloud", 1);
  pub_chair = n.advertise<sensor_msgs::PointCloud2>("chair_complete_cloud", 1);
  ros::Subscriber sub = n.subscribe("pc1",1, callb); //remap to tilt_laser_cloud
  ros::Subscriber sub2 = n.subscribe("pc2",1, callb2); //remap to camera_rgb_points
  srand(85653);

  ros::spin();
  return 0;

}
