#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <boost/foreach.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


using namespace std;
using namespace Eigen;



struct PointXYZRedGreenBlue
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  int red;
  int green;
  int blue;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRedGreenBlue,
				   (float, x, x)
				   (float, y, y)
				   (float, z, z)
				   (int, red, red)
				   (int, green, green)
				   (int, blue, blue)
				   )


pcl::PointCloud<pcl::PointXYZRGB> RedGreenBlue_to_RGB(const pcl::PointCloud<PointXYZRedGreenBlue> &cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud2;
  cloud2.width = cloud.width;
  cloud2.height = cloud.height;
  cloud2.is_dense = false;
  cloud2.points.resize(cloud.width * cloud.height);
  
  for (uint i = 0; i < cloud.points.size(); i++) {
    cloud2.points[i].x = cloud.points[i].x;
    cloud2.points[i].y = cloud.points[i].y;
    cloud2.points[i].z = cloud.points[i].z;
    
    int r = cloud.points[i].red;
    int g = cloud.points[i].green;
    int b = cloud.points[i].blue;
    int rgbi = b;
    
    
    rgbi += (g << 8);
    rgbi += (r << 16);
    if (rgbi == 0){
      std::cout<<"zero "<<r<<" "<<g<<" "<<b<<" "<<std::endl;
    }
    float rgbf; // = *(float*)(&rgbi);
    //memset(&rgbf, 0, sizeof(float));
    memcpy(&rgbf, (float*)(&rgbi), 3);
    if (rgbf == 0){
      //std::cout<<"zero "<<r<<" "<<g<<" "<<b<<" "<<std::endl;
      printf("zero!\n");
      printf("  0x%x\n", rgbi);
      printf("  0x%x\n", rgbf);
    }
    
    cloud2.points[i].rgb = rgbf;
  }

  return cloud2;
}


pcl::PointCloud<PointXYZRedGreenBlue> RGB_to_RedGreenBlue(const pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
  pcl::PointCloud<PointXYZRedGreenBlue> cloud2;
  cloud2.width = cloud.width;
  cloud2.height = cloud.height;
  cloud2.is_dense = false;
  cloud2.points.resize(cloud.width * cloud.height);
  
  for (uint i = 0; i < cloud.points.size(); i++) {
    cloud2.points[i].x = cloud.points[i].x;
    cloud2.points[i].y = cloud.points[i].y;
    cloud2.points[i].z = cloud.points[i].z;
    
    float rgbf = cloud.points[i].rgb;
    int rgbi = *(int*)&rgbf;
    
    cloud2.points[i].red = (rgbi >> 16) & 0xFF;
    cloud2.points[i].green = (rgbi >> 8) & 0xFF;
    cloud2.points[i].blue = rgbi & 0xFF;
  }

  return cloud2;
}


int main(int argc, char** argv)
{
 pcl::PointCloud<PointXYZRedGreenBlue> RedGreenBlue;
 pcl::PointCloud<pcl::PointXYZRGB> RGB;

 if (argc < 4) {
   printf("usage: %s <1|3> <pcd_in> <pcd_out>\n", argv[0]);
   return 1;
 }

 if (argv[1][0] == '1') {  // RedGreenBlue -> RGB
   pcl::io::loadPCDFile(argv[2], RedGreenBlue);
   RGB = RedGreenBlue_to_RGB(RedGreenBlue);
   pcl::io::savePCDFile(argv[3], RGB);
 }
 else if (argv[1][0] == '3') {  // RGB -> RedGreenBlue
   pcl::io::loadPCDFile(argv[2], RGB);
   RedGreenBlue = RGB_to_RedGreenBlue(RGB);
   pcl::io::savePCDFile(argv[3], RedGreenBlue);
 }
 else {
   printf("usage: %s <1|3> <pcd_in> <pcd_out>\n", argv[0]);
   return 1;
 }

 return 0;
}
 

 
