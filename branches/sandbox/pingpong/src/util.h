#ifndef PINGPONG_UTIL_H
#define PINGPONG_UTIL_H


#include <vector>
#include <image_geometry/stereo_camera_model.h>
//#include <sensor_msgs/Image.h>
#include <tf/transform_datatypes.h>
#include "optimization.h"
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <bingham.h>

typedef unsigned char uchar;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef struct {
  uint x;
  uint y;
  uint w;
  uint h;
} rect_t;

typedef struct {
  uchar *data;
  int width;
  int height;
} image_t;

typedef struct {
  std::vector<float> cluster_x;
  std::vector<float> cluster_y;
  std::vector<uint> cluster_size;
} connected_components_t;

typedef struct {
  double t;            // observed time
  double x, y, z;      // tracked position
  double vx, vy, vz;   // tracked velocity
  double wx, wy, wz;   // tracked spin (axis-angle)
  double cov[81];      // covariance in position and velocity (and spin)
  bingham_t B;         // orientation distribution (from logo observations)
  bingham_t dB;        // spin distribution (from logo observations)
  double w;            // probability weight (for multi-hypothesis tracking)
  double r;            // observed radius (in image)
  double ox, oy, oz;   // observed position
  double oq[4];        // observed orientation
  uchar image[32*32];  // left ball image
  //double circle_score;
  //double lx, ly, rx, ry;
} ball_t;


// constants
#define RED CV_RGB(255,0,0)
#define GREEN CV_RGB(0,255,0)
#define BLUE CV_RGB(0,0,255)
#define MAGENTA CV_RGB(255,0,255)
//#define TABLE_LENGTH 2.74
#define TABLE_LENGTH 2.74 //2.76
#define TABLE_WIDTH 1.525
#define TABLE_EDGE_WIDTH .02
#define BALL_RADIUS .02


int connected_components_sse(connected_components_t &C, uchar *I, int w, int h);
int connected_components(ushort *C, uchar *I, int w, int h);
//uint *connected_components(const image_t &I, rect_t &roi);
void crop_image(uchar *dst, uchar *src, int src_width, int src_height, int x, int y, int w, int h);

void resize_image(uchar *dst, uchar *src, int src_width, int src_height, int w, int h);
void compute_image_alignment(int &dx, int &dy, uchar *A, uchar *B, int w, int h, int r);
void shift_image(uchar *dst, uchar *src, int w, int h, int dx, int dy);
void integral_image(uint *dst, uchar *src, int w, int h);
void blur_image(uchar *dst, uchar *src, int w, int h, int r);
void image_centroid(int &mx, int &my, uchar *I, int w, int h);
void image_find_local_mode(int &mx, int &my, uchar *I, int w, int h, int r, int rmax=100000);

void edge_image(sensor_msgs::Image &E, const sensor_msgs::Image &I);
void distance_transform(sensor_msgs::Image &D, const sensor_msgs::Image &I);
rect_t roi_from_points(std::vector<cv::Point2d> points, uint roi_padding, rect_t max_roi);

void table_corners_from_transform(cv::Point3d corners[], const tf::Transform &table_camera_transform);
double table_fitness(const tf::Transform &table_camera_transform,
		     const sensor_msgs::Image &D_left, const sensor_msgs::Image &D_right,
		     const image_geometry::StereoCameraModel &stereo_camera_model);
double fit_table(tf::Transform &table_camera_transform,
		 const sensor_msgs::Image &left, const sensor_msgs::Image &right,
		 const image_geometry::StereoCameraModel &stereo_camera_model);

ball_t ball_dynamics(const ball_t &b, double dt);
std::vector<ball_t> predict_ball_trajectory(const ball_t &b0, int n, double dt, bool use_qbf=0);
void filter_ball_init(ball_t &ball);
void filter_ball(ball_t &ball, ball_t &prev_ball);


float dist_from_plane(const pcl::PointXYZ& point, const Eigen::Vector4f& plane);

Eigen::Vector3f project_to_plane(const pcl::PointXYZ& point, const Eigen::Vector4f& plane);

void quaternion_to_axis_angle(double *a, double *q);



bool in_rectangle(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& p3,const Eigen::Vector3f& p4, const Eigen::Vector3f& p);

//--------------------- Transform Optimization Helpers and Classes -----------------------//


tf::Transform vector_to_transform(Eigen::VectorXf x);
Eigen::VectorXf transform_to_vector(tf::Transform T);


//---------------------  TransformTester class --------------------//

class TransformTester {
 public:
  TransformTester() {}
  virtual double test(const tf::Transform &T) = 0;
};


//---------------------  TransformOptimizer class --------------------//

class TransformOptimizer : public GradientFreeRandomizedGradientOptimizer {
 private:
  TransformTester *transform_tester_;
 public:
  TransformOptimizer(TransformTester *T);
  float evaluate(VectorXf x);
  tf::Transform optimizeTransform(const tf::Transform &initial_transform);
};


//---------------------  TableCameraTransformTester class --------------------//

class TableCameraTransformTester : public TransformTester {
 public:
  const sensor_msgs::Image &L;
  const sensor_msgs::Image &R;
  const image_geometry::StereoCameraModel &S;

  TableCameraTransformTester(const sensor_msgs::Image &left, const sensor_msgs::Image &right,
			     const image_geometry::StereoCameraModel &stereo_camera_model);
  double test(const tf::Transform &T);
};



//----------------- TablePositionOptimizer class ---------------------------//

class TablePositionOptimizer : public GradientFreeRandomizedGradientOptimizer {
 private:
  Eigen::Vector4f plane_equation;
  pcl::PointCloud<pcl::PointXYZ> plane_cloud;
  Eigen::Vector3f origin, faux_x_axis, faux_y_axis, faux_z_axis;
  float a,b,c,d;
 public:
  pcl::PointCloud<pcl::PointXYZ> pointsInPlane;
  TablePositionOptimizer(float step_size, VectorXf noise, float gradient_step_size,Eigen::Vector4f peq, pcl::PointCloud<pcl::PointXYZ> pc);
  float evaluate(VectorXf x);
  std::vector<pcl::PointXYZ> corners(Eigen::Vector3f solution);
  void FillPointsInPlane(VectorXf x);
};


#endif
