#ifndef CHAOS_UTIL_H_
#define CHAOS_UTIL_H_


#include <chaos/common.h>
#include "ply.h"




namespace chaos {


  double get_time_ms();

  void init_rand();
  
  // returns a random double in [0,1]
  double frand();
  
  // approximation to the inverse error function
  double erfinv(double x);
  
  // generate a random sample from a normal distribution
  double normrand(double mu, double sigma);

  // compute the pdf of a normal random variable
  double normpdf(double x, double mu, double sigma);
  
  Matrix4f pose_to_affine_matrix(geometry_msgs::Pose pose);
  
  Matrix4f pose_to_affine_matrix(Vector3f t, Quaternionf q);
  
  geometry_msgs::Pose affine_matrix_to_pose(Matrix4f A);
  
  double sum(vector<double> v);
  
  // calculate the area of a triangle
  double triangle_area(Vector3f x, Vector3f y, Vector3f z);
  
  // convert a PlyVertex into an array
  Vector3f vertex_to_eigen(PlyVertex *V);
  
  // calculate the orientation (normal) of a triangle in 3F
  Vector3f triangle_orientation(Vector3f x, Vector3f y, Vector3f z);

  // dilate finite ranges
  void dilate_range_image(const pcl::RangeImage &range_image_in, pcl::RangeImage &range_image_out);

  // gaussian blur a matrix
  MatrixXf blur_matrix(const MatrixXf &A, float sigma);

  // get the matrix of far ranges in a range image
  MatrixXf get_far_range_matrix(const pcl::RangeImage &range_image);

  // visualize a point cloud with normals
  void visualize_point_cloud_normals(const PointCloudXYZN &cloud_normals);

  // visualize a range image
  void visualize_range_image(const pcl::RangeImage &range_image);

  // visualize a range image point cloud with normals
  void visualize_range_image_with_normals(const pcl::RangeImage &range_image, const pcl::RangeImage &range_image_normals);

  void compute_normals(const PointCloudXYZ &cloud, PointCloudXYZN &cloud_with_normals, float range);

  void compute_principal_curvatures(const PointCloudXYZ &cloud, const PointCloudXYZN &cloud_with_normals,
				    pcl::PointCloud<pcl::PrincipalCurvatures> &principal_curvatures_cloud,
				    float radius);

  void compute_fpfhs(const PointCloudXYZ &cloud, const PointCloudXYZN &cloud_with_normals,
		     pcl::PointCloud<pcl::FPFHSignature33> &fpfh_cloud,
		     float radius);

  void compute_range_image_normals(const pcl::RangeImage &range_image, const PointCloudXYZN &normals, pcl::RangeImage &range_image_normals);

  void point_cloud_normals_to_points_and_normals(const PointCloudXYZN &cloud_normals, PointCloudXYZ &cloud, PointCloudN &normals);

  void range_image_with_normals_to_points_and_normals(const pcl::RangeImage &range_image, const pcl::RangeImage &range_image_normals,
						      PointCloudXYZ &cloud, PointCloudN &normals);
}  // namespace chaos



#endif
