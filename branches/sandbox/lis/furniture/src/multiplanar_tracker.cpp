#include <ros/ros.h>
#include "std_msgs/String.h"
#include <vector>
#include <iostream>
#include <tf/transform_listener.h>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
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
#include <pcl/features/normal_3d_omp.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>

#include <furniture/Add_Multiplanar.h>
#include <furniture/Multiplanar_Poses.h>
#include <furniture/Pose_2D_With_Z.h>
#include <furniture/Multiplanar_Library.h>
#include <furniture/Multiplanar_Model.h>

using namespace Eigen;
using namespace std;

const double PI = M_PI;

vector<furniture::Add_Multiplanar> added_predictions;
geometry_msgs::Point32 sensorOrig;

ros::Publisher pub_markers, pub_poses, pub_library, pub_downsample, pub_segment;

tf::TransformListener *tf_listener;
tf::StampedTransform tf_transform;

//Parameters
double step_size, downsample, inclMultiplier, distTolerance, zNoise, xyNoise, thetaNoise, epsilon, normMultiplier;
int k_knn;
string sensor_frame;
bool segmentCloud;

geometry_msgs::Point32 point_from_coord (float x, float y, float z) {
  geometry_msgs::Point32 p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

furniture::Multiplanar_Model model_with_vector (VectorXf vec, furniture::Multiplanar_Model initModel) {
  // Transforms model points to global coord frame by translating and rotating by pose
  // Vector arg needs to be (X, Y, THETA, Z)
  furniture::Multiplanar_Model newModel;
  //cout<<"initModel polygons: "<<initModel.polygons.size()<<endl;
  for (unsigned int j = 0; j < initModel.polygons.size(); j++) {
    geometry_msgs::Polygon newPoly;
    float sinTheta = sin(vec[2]);
    float cosTheta  = cos(vec[2]);
    for (unsigned int i = 0; i < initModel.polygons[j].points.size(); i++) {
      geometry_msgs::Point32 p = initModel.polygons[j].points[i];
      float newX = p.x*cosTheta - p.y*sinTheta + vec[0];
      float newY = p.x*sinTheta + p.y*cosTheta + vec[1];
      float newZ = p.z + vec[3];
      newPoly.points.push_back (point_from_coord (newX, newY, newZ));
    }
    newModel.polygons.push_back (newPoly);
  }
  return newModel;
}

float point_plane_dist (geometry_msgs::Point32 point, VectorXf planeEq) {
  float a = planeEq[0];
  float b = planeEq[1];
  float c = planeEq[2];
  float d = planeEq[3];
  return (a*point.x + b*point.y + c*point.z + d) / (sqrt (a*a + b*b + c*c));
}

VectorXf plane_equation (Vector3f vecFirst, Vector3f vecSec, geometry_msgs::Point32 vertex) {
  // Finds normal vector to two rays and then determines the constant in plane eq using other normal coeffs
  Vector3f vecNormal = vecFirst.cross(vecSec);
  float coeffD = -(vertex.x * vecNormal[0]) - (vertex.y * vecNormal[1]) - (vertex.z * vecNormal[2]);
  VectorXf plane_coeffs = Vector4f (vecNormal[0], vecNormal[1], vecNormal[2], coeffD);

  //Makes plane normal point towards the sensor
  //A negative dist implies that the normal is NOT pointing towards the sensor; recurses with vectors swapped
  float pointPlaneDist = point_plane_dist (sensorOrig, plane_coeffs);
  if (pointPlaneDist < 0) {
    plane_equation (vecSec, vecFirst, vertex);
  }

  //cout<<"plane equation: "<<plane_coeffs<<endl;
  return plane_coeffs;
}

void remove_dominant_coord (VectorXf initEq, VectorXf &planeEq2D) {
  // Records the index of the dominant coeff
  VectorXf coeffsABC = Vector3f (initEq[0], initEq[1], initEq[2]);
  unsigned int maxCoeffPlace = 0;
  for (unsigned int j = 0; j < coeffsABC.size(); j++) {
    if (abs(coeffsABC[j]) > abs(coeffsABC[maxCoeffPlace])) {
      maxCoeffPlace = j;
    }
  }
  // From a given 3D plane eq, constructs a 2D plane eq with the dominant coeff removed
  // The last element is the index of the dominant coeff
  vector<double> twoDVec;
  for (unsigned int k = 0; k < coeffsABC.size(); k++) {
    if (k != maxCoeffPlace) {
      twoDVec.push_back (coeffsABC[k]);
    }
  }
  twoDVec.push_back (maxCoeffPlace);
  planeEq2D = Vector3f (twoDVec[0], twoDVec[1], twoDVec[2]);
  //cout<<"2D plane eq: "<<planeEq2D<<endl;
}

static bool isXYPointIn2DXYPolygon (const geometry_msgs::Point32 &point, const geometry_msgs::Polygon &polygon) {
  bool in_poly = false;
  double x1, x2, y1, y2;
  int nr_poly_points = polygon.points.size();
  double x_nth = polygon.points[nr_poly_points - 1].x;
  double y_nth = polygon.points[nr_poly_points - 1].y;

  for (int i = 0; i < nr_poly_points; i++) {
    double x_i = polygon.points[i].x;
    double y_i = polygon.points[i].y;

    if (x_i > x_nth) {
      x1 = x_nth;
      x2 = x_i;
      y1 = y_nth;
      y2 = y_i;
    } else {
      x1 = x_i;
      x2 = x_nth;
      y1 = y_i;
      y2 = y_nth;
    }

    if ((x_i < point.x) == (point.x <= x_nth) && (point.y - y1) * (x2 - x1) < (y2 - y1) * (point.x - x1)) {
      in_poly = !in_poly;
    }
    x_nth = x_i;
    y_nth = y_i;
  }
  return (in_poly);
}

bool inModelView (const geometry_msgs::Point32 point, furniture::Multiplanar_Model model, vector<VectorXf> planeEqs, vector<VectorXf> twoDEqs, vector<geometry_msgs::Polygon> twoDPolys, vector<unsigned int> &intersectPolyInds) {
  bool isInView = false;
  for (unsigned int i = 0; i < model.polygons.size(); i++) {
    VectorXf currentPlaneEq = planeEqs[i];
    geometry_msgs::Point32 intersectPt;
    float A = currentPlaneEq[0];
    float B = currentPlaneEq[1];
    float C = currentPlaneEq[2];
    float D = currentPlaneEq[3];

    // Value of t is derived by substituting in the plane eq from the parametric equations for each axes
    float t = -(D + A*sensorOrig.x + B*sensorOrig.y + C*sensorOrig.z) / (A*(point.x-sensorOrig.x) + B*(point.y-sensorOrig.y) + C*(point.z-sensorOrig.z));
    // Finds the intersection of the ray from sensor to point with plane of face of model
    intersectPt.x = sensorOrig.x + t*(point.x-sensorOrig.x);
    intersectPt.y = sensorOrig.y + t*(point.y-sensorOrig.y);
    intersectPt.z = sensorOrig.z + t*(point.z-sensorOrig.z);

    // Uses the 2D plane eq to remove the same dominant coeff in the intersect pt
    VectorXf curr2DEq = twoDEqs[i];
    geometry_msgs::Point32 domLessInterPt;
    if (curr2DEq[2] == 0) {
      domLessInterPt = point_from_coord (intersectPt.y, intersectPt.z, intersectPt.x);
    } else if (curr2DEq[2] == 1) {
      domLessInterPt = point_from_coord (intersectPt.x, intersectPt.z, intersectPt.y);
    } else {
      domLessInterPt = intersectPt;
    }

    //cout<<"2D intersection pt: \n"<<domLessInterPt<<endl;

    if (isXYPointIn2DXYPolygon (domLessInterPt, twoDPolys[i])) {
      // intersectPolyInds records the index of all faces the ray from sensor to point intersects
      intersectPolyInds.push_back (i);
      isInView = true;
    }
  }
  return isInView;
}

void convert_ROS_to_PCL(sensor_msgs::PointCloud cloud, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud) {
  sensor_msgs::PointCloud2 cloud2;
  sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);
  pcl::fromROSMsg(cloud2, pcl_cloud);
  //cout<<"converted ROS to PCL"<<endl;
}

void convert_PCL_to_ROS(pcl::PointCloud<pcl::PointXYZ> &pcl_cloud, sensor_msgs::PointCloud &cloud) {
  sensor_msgs::PointCloud2 cloud2;
  pcl::toROSMsg(pcl_cloud, cloud2);
  sensor_msgs::convertPointCloud2ToPointCloud(cloud2, cloud);
  //cout<<"converted PCL to ROS"<<endl;
}

void estimate_normals (pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointCloud<pcl::Normal> &cloud_normals, int k_knn) {
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_est;
  normal_est.setKSearch (k_knn);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ> ());
  normal_est.setSearchMethod (kdtree);
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_ptr = boost::make_shared<const pcl::PointCloud<pcl::PointXYZ> > (cloud);
  normal_est.setNumberOfThreads(4);
  normal_est.setInputCloud(cloud_ptr);
  normal_est.compute(cloud_normals);
  //ROS_INFO("Normals found");
}

VectorXf closest_plane_to_point (geometry_msgs::Point32 point, vector<VectorXf> planeCoeffs, vector<unsigned int> polyInds) {
  float pointPlaneDist = point_plane_dist (point, planeCoeffs[0]);
  unsigned int closestPlaneIndex;
  closestPlaneIndex = polyInds[0];
  for (unsigned int i = 0; i < polyInds.size(); i++) {
    float nextDist = point_plane_dist (point, planeCoeffs[polyInds[i]]);
    if (pointPlaneDist < nextDist) {
      pointPlaneDist = nextDist;
      closestPlaneIndex = polyInds[i];
    }
  }
  return planeCoeffs[closestPlaneIndex];
}

float fit_normals_cost (pcl::Normal pointNormal, VectorXf plane) {
  float normLength = sqrt(pointNormal.normal[0]*pointNormal.normal[0] + pointNormal.normal[1]*pointNormal.normal[1] + pointNormal.normal[2]*pointNormal.normal[2]);
  VectorXf ptUnitNormal = Vector3f (pointNormal.normal[0]/normLength, pointNormal.normal[1]/normLength, pointNormal.normal[2]/normLength);
  float planeNormLength = sqrt(plane[0]*plane[0] + plane[1]*plane[1] + plane[2]*plane[2]);
  VectorXf planeUnitNorm = Vector3f (plane[0]/planeNormLength, plane[1]/planeNormLength, plane[2]/planeNormLength);
  float dp = abs(planeUnitNorm.dot(ptUnitNormal));
  // cout<<"\n pointNormal: "<<pointNormal.normal[0]<<" "<<pointNormal.normal[1]<<" "<<pointNormal.normal[2]<<endl;
  // cout<<"\n point unit normal: "<<ptUnitNormal<<"\n plane unit normal: "<<planeUnitNorm<<endl;
  // cout<<"\n dot product: "<<dp<<endl;
  // We want dot product = 1, so to make it a cost, we subtract from 1
  return (1 - dp);
}

float model_cost (const geometry_msgs::Point32 point, VectorXf plane) {
  float totalCost = 0;
  //Because only those points that are inModelView have their costs calculated, we can use the distance from that point to the entire plane the polygon exists in.
  float pointPlaneDist = point_plane_dist (point, plane);
  if (pointPlaneDist > distTolerance/2) {
    pointPlaneDist = distTolerance/2;
  } else if (pointPlaneDist < -distTolerance) {
    pointPlaneDist = -distTolerance;
  }
  totalCost += pointPlaneDist*pointPlaneDist;
  return totalCost;
}

void transformTFToEigen (const tf::Transform &t, Affine3f &k) {
  for(int i=0; i<3; i++) {
    k.matrix()(i,3) = t.getOrigin()[i];
    for(int j=0; j<3; j++) {
      k.matrix()(i,j) = t.getBasis()[i][j];
    }
  }
  // Fill in identity in last row
  for (int col = 0 ; col < 3; col ++)
    k.matrix()(3, col) = 0;
  k.matrix()(3,3) = 1;
}

float evaluate_model (furniture::Multiplanar_Model model, const sensor_msgs::PointCloud &msg, double alpha, double beta) {
  float totalScore = 0;
  unsigned int all_in_view = 0;

  // // Moved to sensorCallback
  // Affine3f sensor_pose;
  // transformTFToEigen (tf_transform, sensor_pose);
  // // x,y,z origin of sensor in sensor frame
  // float sensor_x = sensor_pose.translation()[0];
  // float sensor_y = sensor_pose.translation()[1];
  // float sensor_z = sensor_pose.translation()[2];

  // geometry_msgs::Point32 sensorOrig = point_from_coord (sensor_x, sensor_y, sensor_z);

  vector<VectorXf> plane2DEqList, planeCoeffsList, domLessEqsList;
  vector<geometry_msgs::Polygon> domLessPolysList;

  for (unsigned int i = 0; i < model.polygons.size(); i++) {
    VectorXf planeEqCoeffs, plane2DEq;
    geometry_msgs::Polygon face = model.polygons[i];

    //Two vectors derived from the polygon will be crossed to find the eq of the plane the polygon is in
    Vector3f vecFirst = Vector3f (face.points[0].x - face.points[1].x, face.points[0].y - face.points[1].y, face.points[0].z - face.points[1].z);
    Vector3f vecSec = Vector3f (face.points[0].x - face.points[2].x, face.points[0].y - face.points[2].y, face.points[0].z - face.points[2].z);
    planeEqCoeffs = Vector4f (plane_equation (vecFirst, vecSec, face.points[0]));

    //cout << "planeEqCoeffs: " << planeEqCoeffs << endl;
    planeCoeffsList.push_back (planeEqCoeffs);


    VectorXf domLessEq;
    remove_dominant_coord (planeEqCoeffs, domLessEq);
    //VectorXf currentPlaneEq = planeCoeffs[i];
    geometry_msgs::Polygon domLessPoly = model.polygons[i];

    if (domLessEq[2] == 0) {
      for (unsigned int k = 0; k < model.polygons[i].points.size(); k++) {
    	domLessPoly.points[k] = point_from_coord (model.polygons[i].points[k].y, model.polygons[i].points[k].z, model.polygons[i].points[k].x);
      }
    } else if (domLessEq[2] == 1) {
      for (unsigned int k = 0; k < model.polygons[i].points.size(); k++) {
    	domLessPoly.points[k] = point_from_coord (model.polygons[i].points[k].x, model.polygons[i].points[k].z, model.polygons[i].points[k].y);
      }
    } else {
      //domLessInterPt = point;
    }
    domLessEqsList.push_back (domLessEq);
    domLessPolysList.push_back (domLessPoly);
    //cout<<"2D polygons: "<<domLessPoly<<endl;

  }

  //DEBUG
  // vector<unsigned int> polyInds;
  // inModelView (point_from_coord (.8, 0.0, .8), sensorOrig, model, planeCoeffsList, domLessEqsList, domLessPolysList, polyInds);
  // return model_cost (point_from_coord (.8, 0, .8), planeCoeffsList, polyInds);

  //float score = model_cost (point_from_coord (1.55, 0.0, 0.0), model, planeCoeffsList);
  //score += model_cost (point_from_coord (1.55, .4, .3), model, planeCoeffsList);
  //cout<<"Score: "<<score<<endl;
  //return score;

  pcl::PointCloud<pcl::PointXYZ> cloudInit;
  pcl::PointCloud<pcl::Normal> cloudNormals;
  convert_ROS_to_PCL (msg, cloudInit);
  estimate_normals (cloudInit, cloudNormals, k_knn);

  for (unsigned int j = 0; j < msg.points.size(); j++) {
    vector<unsigned int> polyInds;
    // inModelView uses complete unaltered plane eq to determine ray-poly intersection; passes domLess eq and point to XYpt in XYplane
    if (inModelView (msg.points[j], model, planeCoeffsList, domLessEqsList, domLessPolysList, polyInds)) {
      all_in_view++;
      VectorXf closestPlane = planeCoeffsList[polyInds[0]];
      if (polyInds.size() > 1) {
	VectorXf closestPlane = closest_plane_to_point (sensorOrig, planeCoeffsList, polyInds);
      }
      //cout<<"ClosestPlane: "<<closestPlane<<endl;
      totalScore += model_cost (msg.points[j], closestPlane);
      totalScore += beta*fit_normals_cost (cloudNormals.points[j], closestPlane);
    }
  }

  totalScore -= alpha*all_in_view;
  return totalScore;
}

void init_rand() {
  static int first = 1;
  if (first) {
    first = 0;
    srand (time(NULL));
  }
}
  
double frand() {
  // returns a random double in [0,1]
  init_rand();
  return fabs(rand()) / (double)RAND_MAX;
}

double erfinv (double x) {
  if (x < 0)
    return -erfinv(-x);
    
  double a = .147;
  double y1 = (2/(M_PI*a) + log(1-x*x)/2.0);
  double y2 = sqrt(y1*y1 - (1/a)*log(1-x*x));
  double y3 = sqrt(y2 - y1);
  return y3;
}

double normrand (double mu, double sigma) {
  double u = frand();
  return mu + sigma*sqrt(2.0)*erfinv(2*u-1);
}

VectorXf optimize (unsigned int index, float step_size_, float learning_rate_, int max_iter_, const sensor_msgs::PointCloud &msg) {
    float gradient_step_size_ = step_size_;
    furniture::Pose_2D_With_Z current_pose = added_predictions[index].pose;
    furniture::Multiplanar_Model current_model = added_predictions[index].model;

    //cout << "current_pose: " << current_pose << endl;
    //cout << "current_model: " << current_model << endl;

    //cout << "gradient_step_size_ = " << gradient_step_size_ << endl;
    
    VectorXf x = Vector4f (current_pose.x, current_pose.y, current_pose.theta, current_pose.z);
    //cout<<"number of polys after model_with_vector "<<model_with_vector(x, current_model).polygons.size()<<endl;
    float f = evaluate_model (model_with_vector (x, current_model), msg, inclMultiplier, normMultiplier);

    //DEBUG
    // cout << endl << "score: " << evaluate_model (model_with_vector (x, current_model), msg, inclMultiplier, normMultiplier) << endl;
    // cout << "raw score: " << evaluate_model (model_with_vector (x, current_model), msg, 0.0, 0.0) << endl;
    // cout<<"x vector: "<<x<<endl;
    // return x;

    VectorXf noise_ = Vector4f(xyNoise, xyNoise, thetaNoise, zNoise);
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
      float f2 = evaluate_model( model_with_vector (x2, current_model), msg, inclMultiplier, normMultiplier);
      float df = (fabs(f2-f) < .000000001) ? 0 : (f2-f) / fabs(f2-f);
      VectorXf dx = df * ((x2-x) / (x2-x).norm());

      x2 = x;
      for (int j = 0; j < x2.size(); j++)
	x2(j) += normrand(0.0, noise_[j]);  // initialize x2 at x plus random noise

      //printf("(1) x2 = [%.2f, %.2f, %.2f], f2 = %.2f\n", x2[0], x2[1], x2[2], f2);

      x2 -= eta*dx;  // take a step along the chosen gradient direction

      //printf("(2) x2 = [%.2f, %.2f, %.2f], f2 = %.2f\n", x2[0], x2[1], x2[2], f2);

      //TODO: lineSearch

      f2 = evaluate_model (model_with_vector (x2, current_model), msg, inclMultiplier, normMultiplier);
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
    ROS_INFO("Score: %f", evaluate_model (model_with_vector (xmin, current_model), msg, inclMultiplier, normMultiplier));
    ROS_INFO("Raw Cost: %f", evaluate_model (model_with_vector (xmin, current_model), msg, 0.0, 0.0));
    ROS_INFO("xmin x: %f, y: %f, z: %f, theta: %f", xmin[0], xmin[1], xmin[3], xmin[2]);

    // cout << "score: " << evaluate_model (model_with_vector (xmin, current_model), msg, inclMultiplier, normMultiplier) << endl;
    // cout << "raw score: " << evaluate_model (model_with_vector (xmin, current_model), msg, 0.0, 0.0) << endl;
    // cout << "xmin x: " << xmin[0] << " y: " << xmin[1] << " z: " << xmin[3] << " theta: " << xmin[2] << endl;

    return xmin;
}

void create_model_markers (string modelID, furniture::Multiplanar_Model model, ros::Time timestamp, vector<visualization_msgs::Marker> &markers) {
  for (unsigned int i = 0; i < model.polygons.size(); i++) {
    geometry_msgs::Polygon polygon = model.polygons[i];
    stringstream name;
    name  << "Model" << modelID << "-" << i;;
    visualization_msgs::Marker current_hullMarker;
    current_hullMarker.ns = name.str();
    current_hullMarker.action = visualization_msgs::Marker::ADD;
    current_hullMarker.pose.orientation.w = 1.0;
    current_hullMarker.type = visualization_msgs::Marker::LINE_STRIP;
    current_hullMarker.scale.x = 0.01; // thickness
    current_hullMarker.color.b = 0.0; // color = blue
    current_hullMarker.color.a = 1.0; // alpha

    current_hullMarker.id = 0;
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
    ROS_INFO("Markers for model with ID \"%s-%i\" created", modelID.c_str(), i);
  }
}

pcl::PointCloud<pcl::PointXYZ> downsample_cloud (pcl::PointCloud<pcl::PointXYZ> cloud) {
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;    
    sensor_msgs::PointCloud msg_downsampled;

    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setInputCloud (cloud.makeShared());

    grid.setLeafSize (downsample, downsample, downsample);
    grid.filter (cloud_filtered);
    ROS_INFO("Downsampled point cloud from %ld to %ld points", cloud.points.size(), cloud_filtered.points.size());

    convert_PCL_to_ROS(cloud_filtered, msg_downsampled);
    pub_downsample.publish(msg_downsampled);

    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ> segment_cloud (pcl::PointCloud<pcl::PointXYZ> cloud, furniture::Multiplanar_Model model) {
  // No z_min because we want negative information for horizontal planes
  float x_max, x_min, y_max, y_min, z_max;
  x_min = x_max = model.polygons[0].points[0].x;
  y_max = y_min = model.polygons[0].points[0].y;
  z_max = model.polygons[0].points[0].z;

  for (unsigned int i = 0; i < model.polygons.size(); i++) {
    for (unsigned int j = 0; j < model.polygons[i].points.size(); j++) {
      //cout<<"point "<<j<<" z coord: "<<model.polygons[i].points[j].z<<endl;
      x_max = max (x_max, model.polygons[i].points[j].x);
      x_min = min (x_min, model.polygons[i].points[j].x);
      y_max = max (y_max, model.polygons[i].points[j].y);
      y_min = min (y_min, model.polygons[i].points[j].y);
      z_max = max (z_max, model.polygons[i].points[j].z);
    }
  }

  x_max -= sensorOrig.x;
  x_min -= sensorOrig.x;
  y_max -= sensorOrig.y;
  y_min -= sensorOrig.y;
  z_max -= sensorOrig.z;

  float length = x_max - x_min;
  float width = y_max - y_min;

  // Thetas 1 and 2 are off of the x axis of the robot
  float theta_1, theta_2, theta_high;
  theta_1 = atan ((y_max + epsilon*width) / (x_min - epsilon*length));
  theta_2 = atan ((y_min - epsilon*width) / (x_min - epsilon*length));
  theta_high = atan ((z_max + epsilon*z_max) / (x_max + epsilon*length));


  //cout << "z_max: " << z_max << endl; //" z_min: " << z_min << endl;
  //cout << "x_max: " << x_max <<endl; //<< " x_min: " << x_min << " y_max: " << y_max << " y_min: " << y_min << endl;
  // cout << "length: " << length << " width: " << width << endl;
  //cout << "theta_1: " << theta_1 << " theta_2: " << theta_2 << " diff: "<<(theta_1-theta_2)<< endl;
  //cout << "theta_high: " << theta_high << endl; //" theta_low: " << theta_low << endl;

  if (theta_1 < theta_2) {
    float tmp = theta_1;
    theta_1 = theta_2;
    theta_2 = tmp;
  }

  if ((theta_1 - theta_2) < 0.2) {
    theta_1 += 0.1;
    theta_2 -= 0.1;
  }

  if (theta_high < 0.0) {
    theta_high += 0.2;
  }

  //cout<<"after diff: "<<theta_1-theta_2<<endl;

  pcl::PointCloud<pcl::PointXYZ> boundedCloud;
  boundedCloud.header.frame_id = cloud.header.frame_id;
  for (unsigned int k = 0; k < cloud.points.size(); k++) {
    float xyAngle = atan ((cloud.points[k].y - sensorOrig.y) / (cloud.points[k].x - sensorOrig.x));
    // Only points within the two angle values are added to the new cloud
    if (xyAngle < theta_1 && xyAngle > theta_2) {
      float zAngle = atan ((cloud.points[k].z - sensorOrig.z) / (cloud.points[k].x - sensorOrig.x));
      if (zAngle < theta_high) {
	boundedCloud.push_back (cloud.points[k]);
      }
    }
  }
  sensor_msgs::PointCloud boundedCloudMsg;
  convert_PCL_to_ROS (boundedCloud, boundedCloudMsg);
  pub_segment.publish (boundedCloudMsg);

  ROS_INFO("Point cloud segmented from %ld to %ld points", cloud.points.size(), boundedCloud.points.size());

  return boundedCloud;
}

void sensorCallback(const sensor_msgs::PointCloud& msg) {
  ROS_INFO("I heard: [%s]", msg.header.frame_id.c_str());

  if (added_predictions.size() == 0) {
    ROS_INFO("Not tracking any models, ignoring point cloud");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  convert_ROS_to_PCL(msg, cloud);
  if (downsample > 0) {
    cloud = downsample_cloud (cloud);
  }

  if (!tf_listener->waitForTransform("/base_footprint", msg.header.frame_id, msg.header.stamp, ros::Duration(3.0))) {
    ROS_WARN("waitForTransform(%s, %s) failed\n", "/base_footprint", msg.header.frame_id.c_str());
    return;
  } else if (!tf_listener->waitForTransform("/base_footprint", sensor_frame, msg.header.stamp, ros::Duration(3.0))) {
      ROS_WARN("waitForTransform2 failed\n , /base_footprint");
      return;
  } else {
    tf_listener->lookupTransform("/base_footprint", sensor_frame, msg.header.stamp, tf_transform);
  }

  furniture::Multiplanar_Poses nPoses;
  furniture::Multiplanar_Library nModels;

  // Moved from evaluate function
  Affine3f sensor_pose;
  transformTFToEigen (tf_transform, sensor_pose);
  // x,y,z origin of sensor in sensor frame
  float sensor_x = sensor_pose.translation()[0];
  float sensor_y = sensor_pose.translation()[1];
  float sensor_z = sensor_pose.translation()[2];
  sensorOrig = point_from_coord (sensor_x, sensor_y, sensor_z);
  //cout<<"sensorOrig: "<<sensorOrig<<endl;

  for (unsigned int i = 0; i < added_predictions.size(); i++) {
    furniture::Multiplanar_Model initModel = added_predictions[i].model;
    furniture::Pose_2D_With_Z currentPose = added_predictions[i].pose;
    VectorXf currPoseVec = Vector4f (currentPose.x, currentPose.y, currentPose.theta, currentPose.z);

    pcl::PointCloud<pcl::PointXYZ> new_cloud;
    pcl_ros::transformPointCloud("/base_footprint", cloud, new_cloud, *tf_listener);
    if (segmentCloud == true) {
      new_cloud = segment_cloud (new_cloud, model_with_vector (currPoseVec, initModel));
    }

    sensor_msgs::PointCloud new_msg;
    convert_PCL_to_ROS (new_cloud, new_msg);

    VectorXf optPoseVec = optimize (i, step_size, .95, 500, new_msg);
    string id = added_predictions[i].id;

    currentPose.x = optPoseVec[0];
    currentPose.y = optPoseVec[1];
    currentPose.theta = optPoseVec[2];
    currentPose.z = optPoseVec[3];

    added_predictions[i].pose = currentPose;
    nPoses.ids.push_back (id);
    nPoses.poses.push_back (added_predictions[i].pose);

    // model isn't updated, does this still need to be here?
    nModels.ids.push_back(id);
    nModels.models.push_back(initModel);

    ROS_INFO("Model with ID \"%s\" scored and updated", id.c_str());

    vector<visualization_msgs::Marker> markers;
    create_model_markers (id.c_str(), model_with_vector (optPoseVec, initModel), new_msg.header.stamp, markers);
    for (unsigned int j = 0; j < markers.size(); j++) {
      pub_markers.publish(markers[j]);
    }
    pub_library.publish(nModels);
    pub_poses.publish(nPoses);
    cout<<endl;
  }
}

void predictionCallback (const furniture::Add_Multiplanar::ConstPtr& add_msg) {
  ROS_INFO("Recieved a multiplanar model and pose");
  furniture::Add_Multiplanar message = *add_msg;
  if (added_predictions.size() == 0) {
      added_predictions.push_back (message);
  } else {
    for (unsigned int i = 0; i < added_predictions.size(); i++){
      if (added_predictions[i].id.compare (message.id) == 0) {      
	ROS_INFO("Overwriting prediction for model with id: %s", message.id.c_str());
	added_predictions[i].model = message.model;
	added_predictions[i].pose = message.pose;
      } else {
	added_predictions.push_back (message);
	ROS_INFO("Added prediction for model with ID \"%s\"", message.id.c_str());
      }
    }
  }
}

void sensorCallback2(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  sensor_msgs::PointCloud cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(*msg, cloud);
  sensorCallback(cloud);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "multiplanar_tracker");
  ros::NodeHandle n;


  n.getParam("/multiplanar_tracker/sensor_frame", sensor_frame);
  n.param("/multiplanar_tracker/normMultiplier", normMultiplier, 1.0);
  n.param("/multiplanar_tracker/inclMultiplier", inclMultiplier, 0.05);
  n.param("/multiplanar_tracker/downsample", downsample, 0.05);
  n.param("/multiplanar_tracker/epsilon", epsilon, 0.25);
  n.param("/multiplanar_tracker/distTolerance", distTolerance, 0.25);
  n.param("/multiplanar_tracker/k_knn", k_knn, 10);

  n.param("/multiplanar_tracker/zNoise", zNoise, 0.0005);
  n.param("/multiplanar_tracker/xyNoise", xyNoise, 0.005);
  n.param("/multiplanar_tracker/thetaNoise", thetaNoise, .01);
  n.param("/multiplanar_tracker/segmentCloud", segmentCloud, true);
  n.param("/multiplanar_tracker/step_size", step_size, 0.3);

  pub_markers = n.advertise<visualization_msgs::Marker>("convex_hulls_markers", 10);
  pub_poses = n.advertise<furniture::Multiplanar_Poses>("multiplanar_poses", 1);
  pub_library = n.advertise<furniture::Multiplanar_Library>("multiplanar_library", 1);
  pub_downsample = n.advertise<sensor_msgs::PointCloud>("downsampled_cloud", 10);
  pub_segment = n.advertise<sensor_msgs::PointCloud>("segmented_cloud", 10);

  tf_listener = new tf::TransformListener();
  ROS_INFO("Model tracking initiated");

  ros::Subscriber add = n.subscribe ("add_multiplanar", 10, predictionCallback);
  ros::Subscriber sens = n.subscribe ("/points", 1, sensorCallback);
  ros::Subscriber kinect = n.subscribe("/points2",1, sensorCallback2); //remap to camera_rgb_points

  ros::spin();
  return 0;
}
