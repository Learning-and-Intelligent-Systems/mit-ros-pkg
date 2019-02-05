
#include <stdio.h>
#include <pcl/features/feature.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>

#include "chaos/transforms.h"

#include <Eigen3/Core>


using namespace std;
using namespace Eigen3;
using namespace chaos;


void test_one_point_small()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 1;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(1);
  cloud.points[0] = pcl::PointXYZ(1.5, 1.5, 1.5);

  DistanceTransform3D D(cloud, 1.0, 1.5);
  cout << D;
}

void test_one_point_small_offset()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 1;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(1);
  cloud.points[0] = pcl::PointXYZ(1.01, 1.5, 1.5);

  DistanceTransform3D D(cloud, Vector3f(0,0,0), Vector3f(3,3,3), 1.0);
  cout << D;
}

void test_one_point_medium()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 1;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(1);
  cloud.points[0] = pcl::PointXYZ(3.5, 3.5, 3.5);

  DistanceTransform3D D(cloud, 1.0, 3.5);
  cout << D;
}

void test_one_point_large()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 1;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(1);
  cloud.points[0] = pcl::PointXYZ(9.5, 9.5, 9.5);

  DistanceTransform3D D(cloud, 1.0, 9.5);
  cout << D;

  // compare to ground truth
  printf("Errors:\n");
  for (float x = -0.5; x < 20.0; x += 1.0) {
    printf("[x = %.2f]\n", x);
    for (float y = -0.5; y < 20.0; y += 1.0) {
      for (float z = -0.5; z < 20.0; z += 1.0) {
	float dx = 9.5 - x;
	float dy = 9.5 - y;
	float dz = 9.5 - z;
	float d_truth = sqrt(dx*dx + dy*dy + dz*dz);
	printf("%.4f ", d_truth - D.getDistance(pcl::PointXYZ(x,y,z)));
      }
      printf("\n");
    }
    printf("\n");
  }
}

void test_three_points_small()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 3;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(3);
  cloud.points[0] = pcl::PointXYZ(1.5, 0.5, 1.5);
  cloud.points[1] = pcl::PointXYZ(1.5, 1.5, 1.5);
  cloud.points[2] = pcl::PointXYZ(1.5, 2.5, 1.5);

  DistanceTransform3D D(cloud, Vector3f(0,0,0), Vector3f(3,3,3), 1.0);
  cout << D;
}

int main(int argc, char *argv[])
{
  //test_one_point_small();
  //test_one_point_medium();
  //test_one_point_small_offset();
  //test_three_points_small();
  test_one_point_large();

  return 0;
}
