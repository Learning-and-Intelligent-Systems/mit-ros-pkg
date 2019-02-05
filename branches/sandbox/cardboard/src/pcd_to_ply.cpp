
#include <cardboard/common.h>
#include <fstream>

using namespace std;

void writePLY(const char *filename, PointCloudXYZ &cloud)
{
  std::ofstream outf;
  outf.open(filename, ios::out);
  outf<<"ply\n"<<"format ascii 1.0\n"<<"element vertex "<<cloud.points.size()<<endl;
  outf<<"property float x\nproperty float y\nproperty float z\n";
  outf<<"element face 0\n";
  outf<<"element face 0\nproperty list uchar int vertex_indices\nend_header\n";
  for (int i = 0; i < cloud.points.size(); i++)
    outf << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << endl;
  outf.close();
}

void usage(int argc, char** argv)
{
  printf("usage: %s <pcd_in> <ply_out>\n", argv[0]);
  exit(1);
}

int main(int argc, char** argv)
{
  if (argc < 3)
    usage(argc, argv);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::io::loadPCDFile(argv[1], cloud);
  writePLY(argv[2], cloud);

  return 0;
}

