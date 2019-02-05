
#include <cardboard/util.h>

using namespace std;

/*
struct PointFeatures
{
  float x;
  float y;
  float z;
  float nx;
  float ny;
  float nz;
  float curvature;
  float pcx;
  float pcy;
  float pcz;
  float pc1;
  float pc2;
  float f1;
  float f2;
  float f3;
  float f4;
  float f5;
  float f6;
  float f7;
  float f8;
  float f9;
  float f10;
  float f11;
  float f12;
  float f13;
  float f14;
  float f15;
  float f16;
  float f17;
  float f18;
  float f19;
  float f20;
  float f21;
  float f22;
  float f23;
  float f24;
  float f25;
  float f26;
  float f27;
  float f28;
  float f29;
  float f30;
  float f31;
  float f32;
  float f33;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointFeatures,
				   (float, x, x)
				   (float, y, y)
				   (float, z, z)
				   (float, nx, nx)
				   (float, ny, ny)
				   (float, nz, nz)
				   (float, curvature, curvature)
				   (float, pcx, pcx)
				   (float, pcy, pcy)
				   (float, pcz, pcz)
				   (float, pc1, pc1)
				   (float, pc2, pc2)
				   (float, f1, f1)
				   (float, f2, f2)
				   (float, f3, f3)
				   (float, f4, f4)
				   (float, f5, f5)
				   (float, f6, f6)
				   (float, f7, f7)
				   (float, f8, f8)
				   (float, f9, f9)
				   (float, f10, f10)
				   (float, f11, f11)
				   (float, f12, f12)
				   (float, f13, f13)
				   (float, f14, f14)
				   (float, f15, f15)
				   (float, f16, f16)
				   (float, f17, f17)
				   (float, f18, f18)
				   (float, f19, f19)
				   (float, f20, f20)
				   (float, f21, f21)
				   (float, f22, f22)
				   (float, f23, f23)
				   (float, f24, f24)
				   (float, f25, f25)
				   (float, f26, f26)
				   (float, f27, f27)
				   (float, f28, f28)
				   (float, f29, f29)
				   (float, f30, f30)
				   (float, f31, f31)
				   (float, f32, f32)
				   (float, f33, f33)
				   )



pcl::PointCloud<PointFeatures> = merge_fields(PointCloudXYZN cloud_with_normals,
					       pcl::PointCloud<pcl::PrincipalCurvatures> pcs_cloud,
					       pcl::PointCloud<pcl::FPFHSignature33> fpfh_cloud)
{
  pcl::PointCloud<PointFeatures> cloud;
  cloud.width = cloud_with_normals.width;
  cloud.height = cloud_with_normals.height;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  for (int i = 0; i < cloud.points.size(); i++) {
    cloud.points[i].x = cloud_with_normals.points[i].x;
    cloud.points[i].y = cloud_with_normals.points[i].y;
    cloud.points[i].z = cloud_with_normals.points[i].z;
    cloud.points[i].nx = cloud_with_normals.points[i].normal_x;
    cloud.points[i].ny = cloud_with_normals.points[i].normal_y;
    cloud.points[i].nz = cloud_with_normals.points[i].normal_z;
    cloud.points[i].curvature = cloud_with_normals.points[i].curvature;

    cloud.points[i].pcx = pcs_cloud.points[i].principal_curvature_x;
    cloud.points[i].pcy = pcs_cloud.points[i].principal_curvature_y;
    cloud.points[i].pcz = pcs_cloud.points[i].principal_curvature_z;
    cloud.points[i].pc1 = pcs_cloud.points[i].pc1;
    cloud.points[i].pc2 = pcs_cloud.points[i].pc2;

    cloud.points[i].f1 = fpfh_cloud.points[i].histogram[0];
    cloud.points[i].f2 = fpfh_cloud.points[i].histogram[1];
    cloud.points[i].f3 = fpfh_cloud.points[i].histogram[2];
    cloud.points[i].f4 = fpfh_cloud.points[i].histogram[3];
    cloud.points[i].f5 = fpfh_cloud.points[i].histogram[4];
    cloud.points[i].f6 = fpfh_cloud.points[i].histogram[5];
    cloud.points[i].f7 = fpfh_cloud.points[i].histogram[6];
    cloud.points[i].f8 = fpfh_cloud.points[i].histogram[7];
    cloud.points[i].f9 = fpfh_cloud.points[i].histogram[8];
    cloud.points[i].f10 = fpfh_cloud.points[i].histogram[9];
    cloud.points[i].f11 = fpfh_cloud.points[i].histogram[10];
    cloud.points[i].f12 = fpfh_cloud.points[i].histogram[11];
    cloud.points[i].f13 = fpfh_cloud.points[i].histogram[12];
    cloud.points[i].f14 = fpfh_cloud.points[i].histogram[13];
    cloud.points[i].f15 = fpfh_cloud.points[i].histogram[14];
    cloud.points[i].f16 = fpfh_cloud.points[i].histogram[15];
    cloud.points[i].f17 = fpfh_cloud.points[i].histogram[16];
    cloud.points[i].f18 = fpfh_cloud.points[i].histogram[17];
    cloud.points[i].f19 = fpfh_cloud.points[i].histogram[18];
    cloud.points[i].f20 = fpfh_cloud.points[i].histogram[19];
    cloud.points[i].f21 = fpfh_cloud.points[i].histogram[20];
    cloud.points[i].f22 = fpfh_cloud.points[i].histogram[21];
    cloud.points[i].f23 = fpfh_cloud.points[i].histogram[22];
    cloud.points[i].f24 = fpfh_cloud.points[i].histogram[23];
    cloud.points[i].f25 = fpfh_cloud.points[i].histogram[24];
    cloud.points[i].f26 = fpfh_cloud.points[i].histogram[25];
    cloud.points[i].f27 = fpfh_cloud.points[i].histogram[26];
    cloud.points[i].f28 = fpfh_cloud.points[i].histogram[27];
    cloud.points[i].f29 = fpfh_cloud.points[i].histogram[28];
    cloud.points[i].f30 = fpfh_cloud.points[i].histogram[29];
    cloud.points[i].f31 = fpfh_cloud.points[i].histogram[30];
    cloud.points[i].f32 = fpfh_cloud.points[i].histogram[31];
    cloud.points[i].f33 = fpfh_cloud.points[i].histogram[32];
  }

  return cloud;
}
*/

void usage(int argc, char** argv)
{
  printf("usage: %s <pcd_xyz_in> <pcd_features_out>\n", argv[0]);
  exit(1);
}

int main(int argc, char** argv)
{
  if (argc < 5)
    usage(argc, argv);

  PointCloudXYZ cloud;
  pcl::io::loadPCDFile(argv[1], cloud);

  PointCloudXYZN cloud_with_normals;
  cardboard::compute_normals(cloud, cloud_with_normals, .015);
  pcl::io::savePCDFileASCII(argv[2], cloud_with_normals);

  pcl::PointCloud<pcl::PrincipalCurvatures> pcs_cloud;
  cardboard::compute_principal_curvatures(cloud, cloud_with_normals, pcs_cloud, .015);
  pcl::io::savePCDFileASCII(argv[3], pcs_cloud);

  pcl::PointCloud<pcl::FPFHSignature33> fpfh_cloud;
  cardboard::compute_fpfhs(cloud, cloud_with_normals, fpfh_cloud, .03);
  pcl::io::savePCDFileASCII(argv[4], fpfh_cloud);

  //pcl::PointCloud<PointFeatures> = merge_fields(cloud_with_normals, pcs_cloud, fpfh_cloud);

  return 0;
}

