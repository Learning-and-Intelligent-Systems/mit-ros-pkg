#ifndef CARDBOARD_FITNESS_H_
#define CARDBOARD_FITNESS_H_


#include <cardboard/common.h>
#include <cardboard/transforms.h>



namespace cardboard {

  float range_image_fitness(const pcl::RangeImage &range_image, const MatrixXf &cost_map, const PointCloudXYZ &cloud, float out_of_bounds_cost);
  float range_image_fitness(const pcl::RangeImage &range_image, const MatrixXf &cost_map, const PointCloudXYZ &cloud, float out_of_bounds_cost, bool debug);  //dbug
  float range_image_normal_fitness(const pcl::RangeImage &range_image, const PointCloudXYZN &cloud_normals);
  float range_image_normal_fitness(const pcl::RangeImage &range_image, const PointCloudXYZN &cloud_normals, bool debug);  //dbug

  // TODO: make these templated
  float range_cloud_fitness(KdTreeXYZ &kdtree, const pcl::RangeImage &range_cloud);
  float range_cloud_fitness(const DistanceTransform3D &distance_transform, const pcl::RangeImage &range_cloud);
  float point_cloud_fitness(KdTreeXYZ &kdtree, const PointCloudXYZ &point_cloud);
  float point_cloud_fitness(const DistanceTransform3D &distance_transform, const PointCloudXYZ &point_cloud);


}  // namespace cardboard


#endif

