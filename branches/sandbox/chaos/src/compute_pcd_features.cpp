
#include "util.h"

using namespace std;

void usage(int argc, char** argv)
{
  printf("usage: %s <pcd_xyz_in> <pcd_xyzn_out> <pcd_pcs_out> <pcd_fpfh_out>\n", argv[0]);
  exit(1);
}

int main(int argc, char** argv)
{
  if (argc < 5)
    usage(argc, argv);

  PointCloudXYZ cloud;
  pcl::io::loadPCDFile(argv[1], cloud);

  PointCloudXYZN cloud_with_normals;
  chaos::compute_normals(cloud, cloud_with_normals, .015);
  pcl::io::savePCDFileASCII(argv[2], cloud_with_normals);

  pcl::PointCloud<pcl::PrincipalCurvatures> pcs_cloud;
  chaos::compute_principal_curvatures(cloud, cloud_with_normals, pcs_cloud, .015);
  pcl::io::savePCDFileASCII(argv[3], pcs_cloud);

  pcl::PointCloud<pcl::FPFHSignature33> fpfh_cloud;
  chaos::compute_fpfhs(cloud, cloud_with_normals, fpfh_cloud, .03);
  pcl::io::savePCDFileASCII(argv[4], fpfh_cloud);

  return 0;
}

