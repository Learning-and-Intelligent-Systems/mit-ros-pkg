#ifndef INFINITY_MAPDB_POINTCLOUD_H
#define INFINITY_MAPDB_POINTCLOUD_H 1

#include <sys/types.h>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <isam/Point3d.h>

class PointCloud
{
public:
    PointCloud(int64_t utime) : m_utime(utime) {}
    void addPoint(float x, float y, float z) {
        m_points.push_back(isam::Point3d(x,y,z));
    }	
    const std::vector<isam::Point3d> & points() const { return m_points; }
    size_t size() { return m_points.size(); }
    int64_t utime() { return m_utime; }
private:
    std::vector<isam::Point3d> m_points;
    int64_t m_utime;
};
typedef boost::shared_ptr<PointCloud> PointCloudPtr;
    
#endif
