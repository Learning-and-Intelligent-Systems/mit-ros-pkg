#include <ros/ros.h>
#include "std_msgs/String.h"
#include <vector>
#include <iostream>
#include <tf/transform_listener.h>
#include <math.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <visualization_msgs/Marker.h>
#include <furniture/Add_Table.h>
#include <furniture/Table_Poses.h>
#include <furniture/Table_Polygons.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_ros/transforms.h>

using namespace Eigen;
using namespace std;

#define foreach BOOST_FOREACH

vector<furniture::Add_Table> added_tables;
vector<float> max_scores;
int current_table_id;

ros::Publisher pub_marker;
ros::Publisher pub_poses;
ros::Publisher pub_poly;
ros::Publisher pub_downsample;

tf::TransformListener *tf_listener;
tf::StampedTransform tf_transform;

// Parameters
double step_size;
double downsample;
string sensor_frame;
double zTolerance; // specifies the distance _under_ the predicted height we wish to optimize over
double multiplier; // weight for reward for points included in prediction
  

geometry_msgs::Point32 point_to_poly(float x, float y, float z)
{
  geometry_msgs::Point32 p;
  p.x = x;
  p.y = y;
  p.z = z;

  return p;
}
  
void init_rand()
{
  static int first = 1;
  if (first) {
    first = 0;
    srand (time(NULL));
  }
}
  
  // returns a random double in [0,1]
double frand()
{
  init_rand();
  
  return fabs(rand()) / (double)RAND_MAX;
}

geometry_msgs::Polygon poly_from_vector( VectorXf vec , unsigned int table){
  geometry_msgs::Polygon nP;
  float sine = sin(vec[2]);
  float cose  = cos(vec[2]);
  foreach (geometry_msgs::Point32 p, added_tables[table].poly.points ){
    float x = p.x;
    float y = p.y;
    float x2 = x*cose - y*sine + vec[0];
    float y2 = x*sine + y*cose + vec[1] ;
    nP.points.push_back(point_to_poly(x2, y2, p.z));
  }
  return nP;
}
 double erfinv(double x)
  {
    if (x < 0)
      return -erfinv(-x);
    
    double a = .147;
    
    double y1 = (2/(M_PI*a) + log(1-x*x)/2.0);
    double y2 = sqrt(y1*y1 - (1/a)*log(1-x*x));
    double y3 = sqrt(y2 - y1);
    
    return y3;
  }
double normrand(double mu, double sigma)
{
  double u = frand();
    
  return mu + sigma*sqrt(2.0)*erfinv(2*u-1);
}

geometry_msgs::Point32 extend_along(float x, float y, float z, float xA, float yA, float zA, float tol){
  
  //float m_x = (x - xA)/(y - yA);
  //float m_z = (z - zA)/(y - yA);
  float ny;
  float nx;
  float nz;
  if (y > yA){
    ny = y + tol;
  }
  else{
    ny = y - tol;
  }
  if (x > xA){
    //nx = x + tol*m_x;
    nx = x + tol;
  }
  else{
    // nx = x -  tol*m_x;
    nx = x - tol;
  }
  if (z > zA){
    nz = z;// + tol*m_z;
  }else{
    nz = z;// - tol*m_z;
  }
  
  geometry_msgs::Point32 p;
  p.x = nx;
  p.y = ny;
  p.z = nz;
  
  // cout << p.x << p.y << p.z << '   '<<x <<y<<z <<  endl;
  return p;
}

geometry_msgs::Polygon outerEdge(geometry_msgs::Polygon poly){
  float x_sum = 0;
  float y_sum = 0;
  float z_sum = 0;
  int counter = 0;
  foreach(geometry_msgs::Point32 point  , poly.points){
    counter += 1;
    x_sum += point.x;
    y_sum += point.y;
    z_sum += point.z;
  }
  x_sum /= counter;
  y_sum /= counter;
  z_sum /= counter;

  geometry_msgs::Polygon returnPoly;
  foreach(geometry_msgs::Point32 point  , poly.points){
    returnPoly.points.push_back(extend_along(point.x,point.y,point.z,x_sum,y_sum,z_sum,.05));
  }
  return returnPoly;

}


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

static bool isXYPointIn2DXYPolygon (const geometry_msgs::Point32 &point, const geometry_msgs::Polygon &polygon)
{
  bool in_poly = false;
  double x1, x2, y1, y2;

  int nr_poly_points = polygon.points.size();
  double xold = polygon.points[nr_poly_points - 1].x;
  double yold = polygon.points[nr_poly_points - 1].y;
  for (int i = 0; i < nr_poly_points; i++)
  {
    double xnew = polygon.points[i].x;
    double ynew = polygon.points[i].y;

    if (xnew > xold)
    {
      x1 = xold;
      x2 = xnew;
      y1 = yold;
      y2 = ynew;
    }
    else
    {
      x1 = xnew;
      x2 = xold;
      y1 = ynew;
      y2 = yold;
    }

    if ( (xnew < point.x) == (point.x <= xold) && (point.y - y1) * (x2 - x1) < (y2 - y1) * (point.x - x1) )
    {
      in_poly = !in_poly;
    }
    xold = xnew;
    yold = ynew;
  }
  return (in_poly);
}

float smooth_cost(const geometry_msgs::Point32 point, float table_height) {
  // cost function is capped parabola over dz; points found below predicted height are high cost, above are medium cost, and at correct height no cost
  float dz = point.z - table_height;
  if ( dz > zTolerance/2 ) {
    dz = zTolerance/2;
  } else if ( dz < -zTolerance ){
    dz = -zTolerance;
  }
  return dz*dz;
}

vector<geometry_msgs::Point32> interpBtwn2Points (const geometry_msgs::Point32 p, const geometry_msgs::Point32 q, float steps)
{
  vector<geometry_msgs::Point32> interp_line;
  float alpha = 1/steps;
  for (int i = 0; i*alpha <= 1; i++) {
    geometry_msgs::Point32 u;
    u.x = i*alpha*p.x + (1 - i*alpha)*q.x;
    u.y = i*alpha*p.y + (1 - i*alpha)*q.y;
    interp_line.push_back (u);
  }
  return interp_line;
}

float tables_overlap (const geometry_msgs::Polygon &table_poly)
{
  // compares different tables we simultaneously track that are at the same height; penalizes for points along current poly edge that lie in another table poly
  float overlap_cost = 0.0;
  for (unsigned int i = 0; i < added_tables.size() - 1; i++) {
    if (current_table_id != added_tables[i].id) {
      float dz = table_poly.points[0].z - added_tables[i].poly.points[0].z;
      if (abs(dz) <= zTolerance) {
	for (unsigned int j = 0; j < table_poly.points.size() - 1; j++) {
	  for (unsigned int k = 0; k < table_poly.points.size() - 1; k++) {
	    if (j != k) {
	      vector<geometry_msgs::Point32> interp_line = interpBtwn2Points (table_poly.points[j], table_poly.points[k], 10);
	      foreach (geometry_msgs::Point32 interp_pt, interp_line) {
		VectorXf pose_vec = Vector3f(added_tables[i].pose.x, added_tables[i].pose.y, added_tables[i].pose.theta);
		if (isXYPointIn2DXYPolygon (interp_pt, poly_from_vector(pose_vec, i))) {
		++overlap_cost;
		}
	      }
	    }
	  }
	}
      }
    }
  }
  return 2.0*overlap_cost*overlap_cost;
}

bool inTableView(const geometry_msgs::Point32 point, float scanner_x, float scanner_y, float scanner_z, float table_height, const geometry_msgs::Polygon &table_poly)
{
  float m_x = (scanner_x - point.x)/(scanner_z - point.z);
  float m_y = (scanner_y - point.y)/(scanner_z - point.z);
  float table_y = m_y * (table_height - scanner_z) + scanner_y;
  float table_x = m_x * (table_height - scanner_z) + scanner_x;

  geometry_msgs::Point32 pt;
  pt.x = table_x;
  pt.y = table_y;
  pt.z = table_height;
  
  return isXYPointIn2DXYPolygon(pt, table_poly);
}

float evaluate(geometry_msgs::Polygon new_poly, const sensor_msgs::PointCloud &msg) {
  //geometry_msgs::Polygon new_big = outerEdge(new_poly);

  Affine3f sensor_pose;
  transformTFToEigen(tf_transform, sensor_pose);

  // x,y,z origin of sensor in sensor frame
  float scanner_x = sensor_pose.translation()[0];
  float scanner_y = sensor_pose.translation()[1];
  float scanner_z = sensor_pose.translation()[2];
  //cout << "scanner_x: " << scanner_x << " scanner_y: " << scanner_y << " scanner_z: " << scanner_z << endl;

  // iterating over input for scoring
  float totalScore = 0;
  unsigned int all_in_view = 0;
  foreach (geometry_msgs::Point32 point, msg.points){
    if (inTableView(point, scanner_x, scanner_y, scanner_z, new_poly.points[0].z, new_poly)){
      all_in_view++;
      totalScore += smooth_cost(point, new_poly.points[0].z); //table must be parallel to ground
    }
  }

  //Overlap cost has to be calculated seperate from totalscore because it doesn't iterate over points from the input
  //The second term rewards points that are in the predicted polygon; it depends on zTolerance because totalscore depends closely on zTolerance; both zTolerance and multiplier are launchable parameters
  totalScore += tables_overlap(new_poly) - (all_in_view * zTolerance * multiplier);

  //cout<<totalscore<<endl;

  return totalScore;
}

VectorXf optimize(unsigned int table, float step_size_, float learning_rate_, int max_iter_, const sensor_msgs::PointCloud &msg)
  {
    float gradient_step_size_ = step_size_;

    //cout << "gradient_step_size_ = " << gradient_step_size_ << endl;
    
    VectorXf x = Vector3f(added_tables[table].pose.x, added_tables[table].pose.y, added_tables[table].pose.theta);
    float f = evaluate(poly_from_vector(x,table), msg);
    VectorXf noise_ = Vector3f(.01, .01, .03);
    float eta = step_size_;

    VectorXf xmin = x;
    float fmin = f;

    for (int i = 0; i < max_iter_; i++) {

      //printf("xmin = [%.2f, %.2f, %.2f], fmin = %.2f\n", xmin[0], xmin[1], xmin[2], fmin);

      // pick a random gradient direction
      VectorXf gdir(x.size());
      for (int j = 0; j < x.size(); j++)
	gdir(j) = normrand(0.0, noise_[j]);
      gdir.normalize();
      VectorXf x2 = x;
      for (int j = 0; j < x.size(); j++)
	x2(j) += gradient_step_size_ * noise_[j] * gdir(j);
      float f2 = evaluate(poly_from_vector(x2,table), msg);
      float df = (fabs(f2-f) < .000000001) ? 0 : (f2-f) / fabs(f2-f);
      VectorXf dx = df * ((x2-x) / (x2-x).norm());

      x2 = x;
      for (int j = 0; j < x2.size(); j++)
	x2(j) += normrand(0.0, noise_[j]);  // initialize x2 at x plus random noise

      //printf("(1) x2 = [%.2f, %.2f, %.2f], f2 = %.2f\n", x2[0], x2[1], x2[2], f2);

      x2 -= eta*dx;  // take a step along the chosen gradient direction

      //printf("(2) x2 = [%.2f, %.2f, %.2f], f2 = %.2f\n", x2[0], x2[1], x2[2], f2);

      //TODO: lineSearch

      f2 = evaluate(poly_from_vector(x2,table), msg);
      float accept_prob = eta / step_size_;

      if (f2 < f || frand() < accept_prob) {  // annealing (randomized acceptance)
	x = x2;
	f = f2;
	if (f < fmin) {
	  xmin = x;
	  fmin = f;
	}
      }

      eta *= learning_rate_;
    }
    return xmin;
}

void create_markers(int tablenum, geometry_msgs::Polygon polygon, ros::Time timestamp, vector<visualization_msgs::Marker> &markers) {
  visualization_msgs::Marker current_hullMarker;
  stringstream name;
  name  << "Table" << tablenum;
  current_hullMarker.ns = name.str();
  current_hullMarker.action = visualization_msgs::Marker::ADD;
  current_hullMarker.pose.orientation.w = 1.0;
  current_hullMarker.type = visualization_msgs::Marker::LINE_STRIP;
  current_hullMarker.scale.x = 0.01; // thickness
  current_hullMarker.color.b = 1.0/current_table_id; // color = blue
  current_hullMarker.color.a = 1.0; // alpha
  
  //  for (unsigned i = 0; i < hulls.hulls.size(); i++) {
  //current_hullMarker.points.clear();
    current_hullMarker.id = 0; // hulls.ids[i];
    current_hullMarker.header.frame_id = "/base_footprint";
    current_hullMarker.header.stamp = timestamp;
    geometry_msgs::Point tmp;
    for (unsigned j = 0; j < polygon.points.size(); j++) {
      tmp.x = polygon.points[j].x;
      tmp.y = polygon.points[j].y;
      tmp.z = polygon.points[j].z;
      current_hullMarker.points.push_back(tmp);
    }
    tmp.x = polygon.points[0].x;
    tmp.y = polygon.points[0].y;
    tmp.z = polygon.points[0].z;
    current_hullMarker.points.push_back(tmp);
    markers.push_back(current_hullMarker);
} 

void convert_ROS_to_PCL(sensor_msgs::PointCloud cloud, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud) {
  sensor_msgs::PointCloud2 cloud2;
  sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);
  pcl::fromROSMsg(cloud2, pcl_cloud);
}
void convert_PCL_to_ROS(pcl::PointCloud<pcl::PointXYZ> &pcl_cloud, sensor_msgs::PointCloud &cloud) {
  sensor_msgs::PointCloud2 cloud2;
  pcl::toROSMsg(pcl_cloud, cloud2);
  sensor_msgs::convertPointCloud2ToPointCloud(cloud2, cloud);
  //ROS_INFO("Converted %ld points to ROS", cloud.points.size());
}

pcl::PointCloud<pcl::PointXYZ> downsample_cloud (pcl::PointCloud<pcl::PointXYZ> cloud) {
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;    
    sensor_msgs::PointCloud msg_downsampled;

    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setInputCloud (cloud.makeShared());

    //grid.setFilterFieldName ("z");
    //grid.setFilterLimits (0.4, 1.1); //assuming there might be very low and very high tables

    grid.setLeafSize (downsample, downsample, downsample);
    grid.filter (cloud_filtered);
    ROS_INFO("Downsampled point cloud from %ld to %ld points", cloud.points.size(), cloud_filtered.points.size());

    convert_PCL_to_ROS(cloud_filtered, msg_downsampled);
    //ROS_INFO("%ld points in downsampled msg", msg_downsampled.points.size());            
    pub_downsample.publish(msg_downsampled);
    return cloud_filtered;
}

void chatterCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
  if (added_tables.size() == 0){
    ROS_INFO("Not tracking any tables, ignoring point cloud");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  //const_cloud_msg = msg; //&initial_msg;

  convert_ROS_to_PCL(*msg, cloud);

  if (downsample > 0) {
    cloud = downsample_cloud (cloud);
  }

  furniture::Table_Poses nPoses;
  furniture::Table_Polygons nPols;

  if (!tf_listener->waitForTransform("/base_footprint", msg->header.frame_id, msg->header.stamp, ros::Duration(3.0))) {
    ROS_WARN("waitForTransform(%s, %s) failed\n", "/base_footprint", msg->header.frame_id.c_str());
    return;
  }

  //sensor_msgs::PointCloud new_cloud;
  //tf_listener->transformPointCloud("/base_footprint", cloud, new_cloud);

  pcl::PointCloud<pcl::PointXYZ> new_cloud;
  sensor_msgs::PointCloud new_msg;
  pcl_ros::transformPointCloud("/base_footprint", cloud, new_cloud, *tf_listener);

  convert_PCL_to_ROS(new_cloud, new_msg);

  ROS_INFO("I heard: [%s]", msg->header.frame_id.c_str());

  if (!tf_listener->waitForTransform("/base_footprint", sensor_frame, msg->header.stamp, ros::Duration(3.0)))
    {
      ROS_WARN("waitForTransform2 failed\n , /base_footprint");
      return;
    }
  tf_listener->lookupTransform("/base_footprint", sensor_frame, msg->header.stamp, tf_transform);

  for (unsigned int i = 0; i < added_tables.size(); i++) {
    VectorXf newPose = optimize(i, step_size, .95, 500, new_msg);
    geometry_msgs::Polygon optimal = poly_from_vector(newPose, i);

    current_table_id = added_tables[i].id;
    
    float score = evaluate(optimal, new_msg);
    cout << score << endl;
    float threshold = max_scores[i]*.2;
  
    //    if (score < threshold) {
      added_tables[i].pose.x = newPose[0];
      added_tables[i].pose.y = newPose[1];
      added_tables[i].pose.theta = newPose[2];
      nPoses.ids.push_back(added_tables[i].id);
      nPols.ids.push_back(added_tables[i].id);
      nPoses.poses.push_back(added_tables[i].pose);
      nPols.polygons.push_back(added_tables[i].poly);
      ROS_INFO("Table %i scored", added_tables[i].id);
      step_size = .03;
    // } else {
    //   ROS_INFO("Table %i lost", added_tables[i].id);
    //   newPose[0]=added_tables[i].pose.x;
    //   newPose[1]=added_tables[i].pose.y;
    //   newPose[2]=added_tables[i].pose.theta;
    // }
    vector<visualization_msgs::Marker> markers;
    create_markers(added_tables[i].id,poly_from_vector(newPose, i), new_msg.header.stamp, markers);

    //create_markers(-1,optimal, new_cloud.header.stamp, markers);
    //create_markers(-1,outerEdge(optimal), new_cloud.header.stamp, markers);
    for (unsigned int j = 0; j < markers.size(); j++) {
      pub_marker.publish(markers[j]);
      //cout << markers[i] << endl;
    }
    if (nPols.polygons.size() > 0){
      pub_poly.publish(nPols);
      pub_poses.publish(nPoses);
    }
  }

  //vector<visualization_msgs::Marker> bigMarkers;
  //create_markers(outerEdge(optimal), new_cloud.header.stamp, bigMarkers);
  //for (unsigned int i = 0; i < bigMarkers.size(); i++) {
  //  pub_marker.publish(bigMarkers[i]);

    //cout << markers[i] << endl;

  //}
  //cout<< evaluate(optimal, new_cloud)<<endl;
}

float area(geometry_msgs::Polygon table_poly) {
  int points = table_poly.points.size()-1;
  float area = 0;
  for (int i = 0; i < points; i++) {
    area += table_poly.points[i].x*table_poly.points[i+1].y - table_poly.points[i].y*table_poly.points[i+1].x;
  }
  area += table_poly.points[points].x*table_poly.points[0].y - table_poly.points[points].y*table_poly.points[0].x;
  return -abs(area/2);
}

void tableCallback(const furniture::Add_Table::ConstPtr& at) {
  ROS_INFO("Recieved a table model");
  furniture::Add_Table message = *at;
  for(unsigned int i = 0; i < added_tables.size(); i++){
    if (added_tables[i].id == message.id){
      ROS_INFO("Overwriting model for table with id: %i", message.id);
      //geometry_msgs::Polygon table = added_tables[i].poly;
        if (message.poly.points.size() == 0) {
	  //added_tables.erase(i);
          //max_scores.erase(i);
	}
        else {
	  added_tables[i].poly = message.poly;
	  added_tables[i].pose = message.pose;
	  max_scores[i] = area(message.poly);
	}
        return;
    }
  }
  added_tables.push_back(message);
  max_scores.push_back(area(message.poly));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "table_tracker");
  ros::NodeHandle n;

  n.param("/table_tracker/step_size", step_size, 0.3);
  n.param("/table_tracker/downsample", downsample, 0.0);
  n.getParam("/table_tracker/sensor_frame", sensor_frame);
  n.param("/table_tracker/zTolerance", zTolerance, 0.1);
  n.param("/table_tracker/multiplier", multiplier, 0.0075);

  pub_marker = n.advertise<visualization_msgs::Marker>("convex_hulls_markers", 10);
  pub_poses = n.advertise<furniture::Table_Poses>("table_poses", 1);
  pub_poly = n.advertise<furniture::Table_Polygons>("table_polygons", 1);
  pub_downsample = n.advertise<sensor_msgs::PointCloud>("downsampled_cloud", 10);


  tf_listener = new tf::TransformListener();
  ROS_INFO("Table tracking initiated");

  ros::Subscriber add = n.subscribe("add_table", 10, tableCallback);
  ros::Subscriber sub = n.subscribe("/points", 1, chatterCallback);

  ros::spin();
  return 0;
}
