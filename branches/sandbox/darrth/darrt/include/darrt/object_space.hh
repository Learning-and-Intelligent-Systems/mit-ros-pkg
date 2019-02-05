#ifndef __DARRT_OBJECT_SPACE_HH__
#define __DARRT_OBJECT_SPACE_HH__

#include "darrt/collision_aware_types.hh"
#include "darrt/space.hh"
#include "darrt/support_surface.hh"
#include "darrt/types.hh"
#include "darrt/object_types.hh"
#include "darrt/ompl_ros_conversions.h"

#include <ompl/base/spaces/SE3StateSpace.h>

#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/OrderedCollisionOperations.h>

namespace an = arm_navigation_msgs;
namespace gm = geometry_msgs;
namespace orc = ompl_ros_conversions;

namespace darrt {
    
  class ObjectStateSpace : public ob::SE3StateSpace, public Displayable,
			   public CollisionAwareStateSpace, 
			   public Approximable {
  public:
    class StateType : public SE3StateSpace::StateType, 
		      public CollisionAwareState {
    public:
      //an object is either sitting on a surface
      //or rigidly attached to the robot
      const SupportSurface *support_surface;
      //at this point perhaps we should just store the grasp...
      std::string attach_link;
      std::vector<std::string> touch_links;
      StateType() : SE3StateSpace::StateType(), CollisionAwareState() 
      {support_surface=NULL; attach_link="";}
    };

    ObjectStateSpace(const DARRTObject &obj);
    virtual visualization_msgs::MarkerArray displayable_state
    (const ob::State *state, std::string ns, int id, double scale=0.05,
     ColorPalate p=PRIMARYPALATE, double alpha=0.5, bool minimal=false) const;

    virtual ob::State *allocState() const;
    virtual void copyState(ob::State *destination, const ob::State *source) 
      const;

    virtual bool near_states(const ob::State *s1, const ob::State *s2,
			     double deps=DIST_EPS, double aeps=ANGLE_EPS) const;
    virtual double between(const ob::State *state, const ob::State *source,
			   const ob::State *destination,
			   double deps=DIST_EPS, double aeps=ANGLE_EPS) const;

    const DARRTObject &object() const {return object_;}
    virtual bool object_position(const ob::State *state, gm::Pose &pose) const;
    virtual bool set_object_position(ob::State *state, const gm::Pose &pose) 
      const;
    virtual std::string state_string(const ob::State *state) const
    {std::stringstream ss; printState(state, ss); return ss.str();}
    virtual void printState(const ob::State *state, 
			    std::ostream &out = std::cout) const;
    virtual bool update_model
    (const ob::State *new_state, EnvironmentInterface *env) const;

    virtual double distance(const ob::State *source, 
			    const ob::State *destination) const;

  protected:
    const DARRTObject &object_;

  };

   //has to be able to move objects about
  class ObjectStateValidityChecker : public CollisionAwareStateValidityChecker {
  public:
    ObjectStateValidityChecker
    (ob::SpaceInformation *si, 
     collision_space::EnvironmentModel::AllowedCollisionMatrix 
     &default_allowed_collisions,
     bool update_object_location=true, bool check_robot_collisions=true);
    
    bool isValid(const ob::State *state) const;
    void setCheckRobotCollisions(bool cr) {check_robot_collisions_=cr;}
  protected:
    ros::Publisher collision_pub_;
    bool update_model_, check_robot_collisions_;
    const ObjectStateSpace *space_;
  };

  //i can't get ClassFoward(ObjectStateValidityChecker); to compile
  typedef boost::shared_ptr<ObjectStateValidityChecker> 
  ObjectStateValidityCheckerPtr;   

}

#endif //object_space.hh
