#ifndef __DARRT_CONTROL_HH__
#define __DARRT_CONTROL_HH__

#include "darrt/primitive.hh"
#include "darrt/space.hh"
#include "darrt/types.hh"

//ROS includes
#include <ros/ros.h>
#include <arm_navigation_msgs/OrderedCollisionOperations.h>
#include <arm_navigation_msgs/PlanningScene.h>
#include <arm_navigation_msgs/RobotState.h>
#include <geometry_msgs/PoseStamped.h>
#include <planning_environment/models/collision_models_interface.h>
#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>

//OMPL includes
#include <ompl/base/State.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/PathControl.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <rrt_connect/BiDirectedControlSampler.h>

//boost includes
#include <boost/concept_check.hpp>

//shorthand for ompl::base
namespace ob = ompl::base;
namespace oc = ompl::control;

//this does not override interpolate because it is assumed that the
//state propagator will only return straight line paths between points

namespace darrt {
    
  void convert_to_primitive_path(const oc::PathControl &path, 
				 PrimitivePath &prim_path);

  class DARRTControlSpace : public oc::ControlSpace {
  public:
    class ControlType : public oc::Control {
    public:
      PIList clist_;
      int *curr_turn;
      ControlType() {curr_turn = new int(); (*curr_turn) = 0;}
      virtual ~ControlType() {delete curr_turn;}
    protected:

    };
    
    DARRTControlSpace(const ob::StateSpacePtr &stateSpace,
		      const PrimitiveList &primitives,
		      int max_depth=-1) :
      oc::ControlSpace(stateSpace), primitives_(primitives), 
      max_depth_(max_depth){paused_ = false;}
    
    virtual oc::Control *allocControl() const;
    
    virtual void freeControl(oc::Control *c) const;
    
    virtual void copyControl(oc::Control *destination, 
			     const oc::Control *source) const;
    
    virtual bool equalControls(const oc::Control *control1,
			       const oc::Control *control2) const;

    virtual void nullControl(oc::Control *control) const;

    std::string control_string(const oc::Control *control) const;

    void printControl(const oc::Control *control, std::ostream &out) const
    {out << control_string(control);}

    void printSettings(std::ostream &out) const;

    virtual unsigned int getDimension() const 
    {return stateSpace_.get()->getDimension();}
    
    //it's annoying that this is here
    virtual oc::ControlSamplerPtr allocDefaultControlSampler() const
    {ROS_ERROR("Default control sampler allocated shouldn't be used.  Use directed control instead"); return oc::ControlSamplerPtr();}

    virtual void freeControlData(oc::Control *control) const;

    const PrimitiveList &primitives() const {return primitives_;}

    int max_depth() const {return max_depth_;}
    void set_max_depth(int md) {max_depth_ = md;}

    void waitForUnpause() const;
    void pause() {paused_ = true;}
    void unpause() {paused_ = false;}
    bool paused() const {return paused_;}

  protected:
    PrimitiveList primitives_;
    int max_depth_;
    bool paused_;
  };
  
  class DARRTDirectedControlSampler : public oc::BiDirectedControlSampler {
  public:
    DARRTDirectedControlSampler(const oc::SpaceInformation *si);

    unsigned int sampleTo(oc::Control *control, 
			  const ob::State *source,
			  const ob::State *target);
    unsigned int sampleTo(oc::Control *control,  
			  const oc::Control *previous,
			  const ob::State *source,
			  const ob::State *target);
    int sampleFrom(oc::Control *control,
		   const ob::State *target,
		   const ob::State *source);
    int sampleFrom(oc::Control *control,
		   const oc::Control *previous,
		   const ob::State *target,
		   const ob::State *source)
    {return sampleFrom(control, target, source);}

    unsigned int sampleDirectlyTo(oc::Control *control, 
				  const ob::State *source, 
				  const ob::State *target);

    ~DARRTDirectedControlSampler() {si_->freeState(projected_state_);}
    
  protected:
    ob::StateSamplerPtr sampler_ptr_;
    ProjectibleStateSampler *sampler_;
    ProjectibleState *projected_state_;

    unsigned int sampleToRecur(oc::Control *control,
			       const ob::State *source,
			       const ob::State *target,
			       unsigned int depth);
  };
  
  oc::DirectedControlSamplerPtr alloc_darrt_sampler
  (const oc::SpaceInformation *si);


  class DARRTStatePropagator : public oc::StatePropagator {
  public:
    DARRTStatePropagator(oc::SpaceInformation *si) :
      oc::StatePropagator(si) {}
    DARRTStatePropagator(const oc::SpaceInformationPtr &si) :
      oc::StatePropagator(si) {}
    bool init();
    void propagate(const ob::State *state, const oc::Control *control,
		   const double duration, ob::State *result) const;
    bool canPropagateBackward() const {return true;}
  };
  
  


}

#endif //control.hh
