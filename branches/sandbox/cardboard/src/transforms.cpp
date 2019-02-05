
#include <math.h>
#include <queue>

#include <pcl/features/feature.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/kdtree/kdtree.h>
//#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <cardboard/transforms.h>



namespace cardboard {



  void DistanceTransform3D::init(const pcl::PointCloud<pcl::PointXYZ> &cloud, Vector3f gridMin, Vector3f gridMax, float resolution)
  {
    resolution_ = resolution;
    dims_ = ((gridMax - gridMin) / resolution).cast<int>();
    gridMin_ = gridMin;
    gridMax_ = gridMin_ + resolution_*(dims_.cast<float>());  // round gridMax down so there aren't any fractional cells

    gridData_ = new float[dims_(0)*dims_(1)*dims_(2)];
    for (int i = 0; i < dims_(0)*dims_(1)*dims_(2); i++)
      gridData_[i] = INFINITY;

    outOfBoundsDist_ = NAN;

    //printf("Created new distance transform with dims (%dx%dx%d)\n", dims_(0), dims_(1), dims_(2));  //dbug

    setPointCloud(cloud);  // compute distance transform
  }

  void DistanceTransform3D::init(const pcl::PointCloud<pcl::PointXYZ> &cloud, float resolution, float padding)
  {
    Vector4f pmin, pmax;
    pcl::getMinMax3D(cloud, pmin, pmax);

    init(cloud, pmin.topRows(3) - Vector3f::Constant(padding), pmax.topRows(3) + Vector3f::Constant(padding), resolution);
  }

  DistanceTransform3D::DistanceTransform3D()
  {
    gridData_ = NULL;
  }

  DistanceTransform3D::DistanceTransform3D(const pcl::PointCloud<pcl::PointXYZ> &cloud, Vector3f gridMin, Vector3f gridMax, float resolution)
  {
    init(cloud, gridMin, gridMax, resolution);
  }


  DistanceTransform3D::DistanceTransform3D(const pcl::PointCloud<pcl::PointXYZ> &cloud, float resolution, float padding)
  {
    init(cloud, resolution, padding);
  }


  DistanceTransform3D::~DistanceTransform3D()
  {
    if (gridData_)
      delete[] gridData_;
  }

  
  /* Compute the distance transform of the given cloud */
  void DistanceTransform3D::setPointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud)
  {
    float *G = gridData_;
    std::queue<int> Q[3];  // queues for the three waves
    int current_wave = 0;
    int x, y, z;

    // use intra-voxel distance for immediate neighbors of point cloud points
    for (size_t i = 0; i < cloud.points.size(); i++) {
      pointToIndex3D(cloud.points[i], x, y, z);
      if (inGrid(x,y,z)) {
	G[getIndex(x,y,z)] = 0.0;
	std::vector< std::pair<int,float> > neighbors = getNeighbors(x,y,z);
	for (size_t j = 0; j < neighbors.size(); j++) {
	  int n = neighbors[j].first;
	  float d = pcl::euclideanDistance(cloud.points[i], indexToPoint(n));
	  if (d < G[n]) {
	    int w = (int)(d/resolution_);  // n's next wave number
	    if (isinf(G[n]) || w < (int)(G[n]/resolution_))  // if n's next wave number is lower than n's current wave number,
	      Q[w%3].push(n);                               // add n to wave w
	    G[n] = d;  // update G with the new distance
	  }
	}
      }
    }

    // use a wavefront algorithm (with 3 waves) to fill in the rest of the distances
    while (Q[0].size() + Q[1].size() + Q[2].size() > 0) {  // while there are still cells in waves

      while (!Q[current_wave%3].empty()) {  // process current wave
	int i = Q[current_wave%3].front();  // get current node
	Q[current_wave%3].pop();
	if ((int)(G[i]/resolution_) < current_wave)  // check if we already processed node i in a previous wave
	  continue;
	std::vector< std::pair<int,float> > neighbors = getNeighbors(i);  // get current node's neighbors
	for (size_t j = 0; j < neighbors.size(); j++) {
	  int n = neighbors[j].first;
	  float d = G[i] + neighbors[j].second;
	  if (d < G[n]) {
	    int w = (int)(d/resolution_);  // n's next wave number
	    if (isinf(G[n]) || w < (int)(G[n]/resolution_))  // if n's next wave number is lower than n's current wave number,
	      Q[w%3].push(n);                               // add n to wave w
	    G[n] = d;  // update G with the new distance
	  }
	}
      }

      current_wave++;  // go to the next wave
    }

    // compute max dist
    maxDist_ = 0.0;
    for (int i = 0; i < dims_(0)*dims_(1)*dims_(2); i++)
      if (gridData_[i] > maxDist_)
	maxDist_ = gridData_[i];
  }


  void DistanceTransform3D::setOutOfBoundsDistance(float d)
  {
    outOfBoundsDist_ = d;
  }


  float DistanceTransform3D::getDistance(const pcl::PointXYZ &point) const
  {
    int x = (int)((point.x - gridMin_(0)) / resolution_);
    int y = (int)((point.y - gridMin_(1)) / resolution_);
    int z = (int)((point.z - gridMin_(2)) / resolution_);

    if (!inGrid(x,y,z)) {
      if (!isnan(outOfBoundsDist_))
	return outOfBoundsDist_;
      else  // by default, if point is out of the grid, return the max dist of any point in the grid
	return maxDist_;
    }

    return gridData_[getIndex(x,y,z)];
  }


  std::vector<float> DistanceTransform3D::getDistances(const pcl::PointCloud<pcl::PointXYZ> &cloud) const
  {
    std::vector<float> distances(cloud.points.size());
    for (size_t i = 0; i < distances.size(); i++)
      distances[i] = getDistance(cloud.points[i]);
    return distances;
  }


  //------------------ PRIVATE METHODS ------------------//

  std::vector< std::pair<int,float> > DistanceTransform3D::getNeighbors(int x, int y, int z)
  {
    std::vector< std::pair<int,float> > neighbors(26);  // 26-connected (8+9+9)
    
    static const int dx[26] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,
			        0, 0, 0, 0,    0, 0, 0, 0,
			        1, 1, 1, 1, 1, 1, 1, 1, 1};
    static const int dy[26] = {-1,-1,-1, 0, 0, 0, 1, 1, 1,
			       -1,-1,-1, 0,    0, 1, 1, 1,
			       -1,-1,-1, 0, 0, 0, 1, 1, 1};
    static const int dz[26] = {-1, 0, 1,-1, 0, 1,-1, 0, 1,
			       -1, 0, 1,-1,    1,-1, 0, 1,
			       -1, 0, 1,-1, 0, 1,-1, 0, 1};

    const float S1 = resolution_;
    const float S2 = sqrt(2)*resolution_;
    const float S3 = sqrt(3)*resolution_;
    const float d[26] = {S3,S2,S3,S2,S1,S2,S3,S2,S3,
			 S2,S1,S2,S1,   S1,S2,S1,S2,
			 S3,S2,S3,S2,S1,S2,S3,S2,S3};

    int n = 0;
    for (int i = 0; i < 26; i++) {
      int x2 = x + dx[i];
      int y2 = y + dy[i];
      int z2 = z + dz[i];
      if (inGrid(x2, y2, z2))
	neighbors[n++] = std::pair<int,float>(getIndex(x2, y2, z2), d[i]);
    }
    neighbors.resize(n);

    return neighbors;
  }

}
