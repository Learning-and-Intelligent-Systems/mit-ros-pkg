#ifndef CHAOS_TRANSFORMS_H_
#define CHAOS_TRANSFORMS_H_

#include <chaos/common.h>



namespace chaos {

  /* DistanceTransform3D class -- computes an approximate 3D distance transform of a point cloud */
  class DistanceTransform3D {

  private:
    Vector3f gridMin_;  // grid lower bounds
    Vector3f gridMax_;  // grid upper bounds
    float resolution_;  // grid cell resolution
    Vector3i dims_;     // grid dimensions
    float *gridData_;   // distances

    float outOfBoundsDist_;
    float maxDist_;


  public:
    void init(const pcl::PointCloud<pcl::PointXYZ> &cloud, Vector3f gridMin, Vector3f gridMax, float resolution);
    void init(const pcl::PointCloud<pcl::PointXYZ> &cloud, float resolution, float padding);
    DistanceTransform3D();
    DistanceTransform3D(const pcl::PointCloud<pcl::PointXYZ> &cloud, Vector3f gridMin, Vector3f gridMax, float resolution);
    DistanceTransform3D(const pcl::PointCloud<pcl::PointXYZ> &cloud, float resolution, float padding);
    ~DistanceTransform3D();
    void setPointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud);  // compute a new transform
    void setOutOfBoundsDistance(float d);
    float getDistance(const pcl::PointXYZ &point) const;
    std::vector<float> getDistances(const pcl::PointCloud<pcl::PointXYZ> &cloud) const;

    inline float *getGridData() const {
      return gridData_;
    }

    inline Vector3i getGridDims() const {
      return dims_;
    }

    inline int getIndex(int x, int y, int z) const {
      return x*dims_(1)*dims_(2) + y*dims_(2) + z;
    }

    inline void getIndex3D(int i, int &x, int &y, int &z) const {
      z = i % dims_(2);
      i = (i - z) / dims_(2);
      y = i % dims_(1);
      x = (i - y) / dims_(1);
    }

    inline void pointToIndex3D(const pcl::PointXYZ &point, int &x, int &y, int &z) const {
      x = (int)((point.x - gridMin_(0)) / resolution_);
      y = (int)((point.y - gridMin_(1)) / resolution_);
      z = (int)((point.z - gridMin_(2)) / resolution_);
    }

    inline pcl::PointXYZ indexToPoint(int i) const {  // returns the cell center
      int x, y, z;
      getIndex3D(i, x, y, z);
      return pcl::PointXYZ(gridMin_(0) + (x+.5)*resolution_,
			   gridMin_(1) + (y+.5)*resolution_,
			   gridMin_(2) + (z+.5)*resolution_);
    }

    inline bool inGrid(int x, int y, int z) const {
      return x >= 0 && x < dims_(0) && y >= 0 && y < dims_(1) && z >= 0 && z < dims_(2);
    }

    std::vector< std::pair<int,float> > getNeighbors(int x, int y, int z);

    inline std::vector< std::pair<int,float> > getNeighbors(int i) {
      int x, y, z;
      getIndex3D(i, x, y, z);
      return getNeighbors(x,y,z);
    }

  };

  inline std::ostream& operator<<(std::ostream& os, const DistanceTransform3D& D) {
    float *gridData = D.getGridData();
    Vector3i dims = D.getGridDims();
    for (int x = 0; x < dims(0); x++) {
      os << "[x = " << x << "]" << endl;
      for (int y = 0; y < dims(1); y++) {
	for (int z = 0; z < dims(2); z++)
	  os << gridData[D.getIndex(x,y,z)] << " ";
	os << endl;
      }
      os << endl;
    }
    return os;
  }

}

#endif

