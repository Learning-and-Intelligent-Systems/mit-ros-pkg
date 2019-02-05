#ifndef __DARRT_CA_TYPES_HH__
#define __DARRT_CA_TYPES_HH__

#include "darrt/environment_interface.hh"
#include "darrt/primitive.hh"

#include <arm_navigation_msgs/OrderedCollisionOperations.h>
#include <arm_navigation_msgs/RobotState.h>

#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>

namespace ob = ompl::base;
namespace an = arm_navigation_msgs;

namespace darrt {

  class CollisionAwareStateSpace {
  public:
    virtual bool update_model(const ob::State *state, EnvironmentInterface *env)
      const=0;
  };

  class CollisionAwareState {
  public:
    CollisionAwareState() {control_ = NULL; type_ = "";}
    
    an::OrderedCollisionOperations allowed_collisions;    
    
    bool add_allowed_collisions(const an::OrderedCollisionOperations &ops);
    bool add_allowed_collision(an::CollisionOperation op)
    {allowed_collisions.collision_operations.push_back(op); return true;}
    bool set_allowed_collisions(const an::OrderedCollisionOperations &ops)
    {allowed_collisions.collision_operations.clear(); 
      add_allowed_collisions(ops); return true;}
    virtual void set_control(const Primitive *control, std::string type) 
    {control_ = control; type_ = type;}
    const Primitive *control() const {return control_;}
    std::string type() const {return type_;}

  protected:
    const Primitive *control_;
    std::string type_;
  };

  class CollisionAwareStateValidityChecker : public ob::StateValidityChecker {
  public:
    CollisionAwareStateValidityChecker
    (ob::SpaceInformation *si,
     collision_space::EnvironmentModel::AllowedCollisionMatrix
     &default_allowed_collisions);

    virtual bool setup(EnvironmentInterface *env)
    {environment_interface_ = env; return true;}

  protected:
    virtual bool reconcile_collision_model(const ob::State *ompl_state) const;
    virtual bool revert_collision_model() const;
    EnvironmentInterface *environment_interface_;
    collision_space::EnvironmentModel::AllowedCollisionMatrix
    default_allowed_collisions_;
    const CollisionAwareStateSpace *caspace_;
  };

  class CollisionAwarePrimitiveInstance : public PrimitiveInstance {
  public:
    CollisionAwarePrimitiveInstance(const Primitive *prim,
				    std::string type,
				    const ob::State *source,
				    const ob::State *destination);
    virtual std::string str() const;
    bool add_allowed_collisions(const an::OrderedCollisionOperations &ops);
    bool add_allowed_collision(an::CollisionOperation op);
    bool set_allowed_collisions(const an::OrderedCollisionOperations &ops);
    an::OrderedCollisionOperations allowed_collisions;
  };

}

#endif //collision_aware_types.hh
