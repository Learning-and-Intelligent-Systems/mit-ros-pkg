#ifndef __DARRT_SUPPORT_SURFACE_HH__
#define __DARRT_SUPPORT_SURFACE_HH__

#include "darrt/utils.hh"

#include <arm_navigation_msgs/CollisionObject.h>
#include <geometry_msgs/Point.h>

namespace gm = geometry_msgs;
namespace an = arm_navigation_msgs;

namespace darrt {

  class SupportSurface {
  public:
    SupportSurface(std::string name) {name_ = name;}
    virtual bool supporting(const an::CollisionObject &obj,
			    const gm::Pose &pose,
			    double eps=DIST_EPS) const;
    //is the point actually on the surface (in 3D, no projection)
    virtual bool on_surface(const gm::Point &p, 
			    double eps=DIST_EPS) const=0;
    virtual gm::Point randomPointOnSurface() const=0;
    virtual double distance(const gm::Point &p) const=0;
    std::string name() const {return name_;}
    //uses a projection.  p1 should be on the
    virtual gm::Point last_point_on_surface
    (const gm::Point &p1, const gm::Point &p2) const=0;
    virtual gm::Point last_point_on_surface
    (const gm::Point &p1, const gm::Point &p2, double offset) const=0;
    virtual std::string str() const {return name_;}
  protected:
    std::string name_;
  };

  typedef std::vector<const SupportSurface *> SupportSurfaceList;

  class MeshSurface : public SupportSurface {
    static const double Z_DIFF = 0.03;
  public:
    MeshSurface(std::string name) :
      SupportSurface(name) {}
    MeshSurface(std::string name, const gm::Pose &pose, 
		const an::Shape &mesh); 
    bool on_surface(const gm::Point &p, 
		    double eps=DIST_EPS) const;
    virtual gm::Point randomPointOnSurface() const;
    double distance(const gm::Point &p) const;
    gm::Point last_point_on_surface
    (const gm::Point &p1, const gm::Point &p2) const;
    virtual gm::Point last_point_on_surface
    (const gm::Point &p1, const gm::Point &p2, double offset) const;
    void set_pose_and_mesh(const gm::Pose &pose, const an::Shape &mesh);
    const gm::Pose &get_pose() const {return pose_;}
    const an::Shape &get_mesh() const {return mesh_;}
  protected:
    gm::Pose pose_;
    an::Shape mesh_;
    double xmin_, xmax_, ymin_, ymax_;
  };
}

#endif //utils.hh
