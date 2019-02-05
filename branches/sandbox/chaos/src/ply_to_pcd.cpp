
#include <stdio.h>
#include "ply.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"



void usage(char *argv[])
{
  printf("usage: %s <ply_in> <pcd_out>\n", argv[0]);
  exit(1);
}


int main(int argc, char *argv[])
{
  if (argc < 3)
    usage(argv);

  // read in PLY
  int num_vertices;
  PlyVertex *vertices = ply_read_vertices(argv[1], &num_vertices);

  // convert to point cloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = num_vertices;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = vertices[i].x;
    cloud.points[i].y = vertices[i].y;
    cloud.points[i].z = vertices[i].z;
  }

  // write to PCD file
  pcl::io::savePCDFileASCII (argv[2], cloud);
  printf("Saved %d data points to %s.\n", (int)cloud.points.size(), argv[2]);

  return 0;
}
