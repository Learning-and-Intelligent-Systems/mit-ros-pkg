#ifndef __OBJECT_TYPES__HH__
#define __OBJECT_TYPES__HH__

#include "darrt/support_surface.hh"

#include <arm_navigation_msgs/CollisionObject.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>

namespace darrt {

  class DARRTObject : public an::CollisionObject {
  public:
    DARRTObject(const std::vector<std::string> support_surfaces) : 
      an::CollisionObject(), support_surfaces_(support_surfaces) {}
    void copyCollisionObject(const an::CollisionObject &obj);
    virtual gm::Pose sampleStablePose(const SupportSurface *surface) const;
    virtual double distance(const gm::Pose &p1, const gm::Pose &p2) const;
    virtual bool nearPoses(const gm::Pose &p1, const gm::Pose &p2,
			   double deps=DIST_EPS, double aeps=ANGLE_EPS) const;
    virtual bool canSupport(const SupportSurface *surface) const;
    virtual ~DARRTObject() {}
  protected:
    std::vector<std::string> support_surfaces_;
  };

  class FixedObject : public DARRTObject {
  public:
    FixedObject(const std::vector<std::string> support_surfaces,
		const gm::Pose starting_pose) : 
      DARRTObject(support_surfaces), starting_pose_(starting_pose) {}
    virtual gm::Pose sampleStablePose(const SupportSurface *surface) const
    {return starting_pose_;}
    virtual double distance(const gm::Pose &p1, const gm::Pose &p2) const
    {return 0;}
    virtual bool nearPoses(const gm::Pose &p1, const gm::Pose &p2,
			   double deps=DIST_EPS, double aeps=ANGLE_EPS) const
    {return true;}
  protected:
    gm::Pose starting_pose_;
  };

  class RoundObject : public DARRTObject {
  public:
    RoundObject(const std::vector<std::string> &support_surfaces) : 
      DARRTObject(support_surfaces) {}
    virtual double distance(const gm::Pose &p1, const gm::Pose &p2) const;
    virtual bool nearPoses(const gm::Pose &p1, const gm::Pose &p2,
			   double deps=DIST_EPS, double aeps=ANGLE_EPS) const;
  };

  class SpatulaObject : public DARRTObject {
  public:
    SpatulaObject(std::string spatid, std::string frame_id,
		  const std::vector<std::string> &support_surfaces,
		  const std::vector<double> &paddle_dimensions, 
		  const std::vector<double> &handle_dimensions,
		  double angle, gm::Pose starting_pose);
    void getGrasps(std::vector<gm::Transform> &grasps) const;
    double angle() const {return angle_;}
    virtual gm::Pose sampleStablePose(const SupportSurface *surface) const;
  protected:
    double angle_;
    an::Shape paddle_, handle_;
    gm::Pose starting_pose_;
  };
}
#endif //object_types.hh
