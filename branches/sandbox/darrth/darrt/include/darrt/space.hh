#ifndef __DARRT_SPACE_HH__
#define __DARRT_SPACE_HH__

#include <boost/shared_ptr.hpp>
#include <map>
#include <set>

#include "darrt/collision_aware_types.hh"
#include "darrt/primitive.hh"
#include "darrt/state_transformer.hh"
#include "darrt/support_surface.hh"
#include "darrt/types.hh"
#include "darrt/utils.hh"

#include "darrt_msgs/DARRTState.h"

//ROS includes
#include <ros/ros.h>

#include <arm_navigation_msgs/PlanningScene.h>
#include <arm_navigation_msgs/RobotState.h>
#include <geometry_msgs/PoseStamped.h>

#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>

//OMPL includes
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>

//boost includes
#include <boost/concept_check.hpp>

#define ROBOT_RADIUS 0.3

//shorthand for ompl::base
namespace ob = ompl::base;
namespace oc = ompl::control;
namespace an = arm_navigation_msgs;

//this does not override interpolate because it is assumed that the
//state propagator will only return straight line paths between points

namespace darrt {

  enum ThingType {OBJECT, ROBOT, UNKNOWN};

  class ProjectibleState : public ob::CompoundState {
  protected:
    DARRTProjectionFunction function_;
    bool valid_;
  public:
    ProjectibleState() : ob::CompoundState() {function_=NULL; valid_=true;}
    void setProjectionFunction(const DARRTProjectionFunction &f) {function_ = f;}
    const DARRTProjectionFunction &getProjectionFunction() const {return function_;}
    void set_invalid() {valid_ = false;}
    void set_valid() {valid_ = true;}
    bool valid() const {return valid_;}
  };

  class DARRTStateSpace : public ob::CompoundStateSpace, public Displayable,
			  public Approximable, public CollisionAwareStateSpace {

  public:

    class StateType : public ProjectibleState, public CollisionAwareState {
    public:
      StateType() : ProjectibleState(), CollisionAwareState() {}
      virtual void set_control(const Primitive *control, std::string type);
    };
    
    DARRTStateSpace(const PrimitiveList &primitives,
		    const SupportSurfaceList &support_surfaces,
		    const std::vector< std::pair<DARRTProjectionFunction, 
		    double> > &projection_functions);
    DARRTStateSpace(const std::vector<ob::StateSpacePtr> &components,
		    int robot_index, const PrimitiveList &plist,
		    const SupportSurfaceList &support_surfaces, 
		    const std::vector< std::pair<DARRTProjectionFunction, 
		    double> > &projection_functions);
    virtual void freeState(ob::State *s) const;
    virtual void addSubspace(const ob::StateSpacePtr &component,
			     double weight);
    virtual void addSubspace(const ob::StateSpacePtr &component,
			     bool robot_space);
    void add_support_surface(const SupportSurface *surface)
    {support_surfaces_.push_back(surface);}
    void set_support_surfaces(const SupportSurfaceList &support_surfaces)
    {support_surfaces_ = support_surfaces;}
    virtual ob::State *allocState() const;
    virtual void copyState(ob::State *destination, const ob::State *source)
      const;
    virtual double distance(const ob::State *source, 
			    const ob::State *destination) const;
    virtual bool equalStates(const ob::State *state1, const ob::State *state2)
      const;
    virtual bool near_states(const ob::State *s1, const ob::State *s2,
			     double deps=DIST_EPS, double aeps=ANGLE_EPS) const;
    

    virtual bool satisfiesBounds(const ob::State *state) const;

    virtual ob::State *get_state(unsigned int ind, ob::State *state) const;
    virtual const ob::State *get_state(unsigned int ind,
				       const ob::State *state) const;

    virtual ob::State *robot_state(ob::State *state) const;
    virtual const ob::State *robot_state(const ob::State *state) const;
    virtual ob::StateSpace *robot_state_space();
    virtual const ob::StateSpace *robot_state_space() const;
    virtual double robot_distance(const ob::State *destination,
				  const ob::State *source) const;
    virtual bool robot_states_match(const ob::State *s1, const ob::State *s2)
      const;
    virtual bool near_robot_states(const ob::State *s1, 
				   const ob::State *s2, 
				   double deps = DIST_EPS, 
				   double aeps = ANGLE_EPS) const;
    
    virtual ob::State *object_state(unsigned int ind, ob::State *state) const;
    virtual const ob::State *object_state(unsigned int ind,
					  const ob::State *state) const;
    virtual ob::StateSpace *object_state_space(unsigned int ind);
    virtual const ob::StateSpace *object_state_space(unsigned int ind) const;
    virtual bool object_states_match
    (unsigned int ind, const ob::State *s1, const ob::State *s2) const;
    virtual bool object_states_match
    (const ob::State *s1, const ob::State *s2) const;

    virtual bool near_object_states(unsigned int ind, const ob::State *s1, 
				    const ob::State *s2, 
				    double deps = DIST_EPS, 
				    double aeps = ANGLE_EPS) const;

    virtual bool near_object_states(const ob::State *s1, 
				    const ob::State *s2, 
				    double deps = DIST_EPS, 
				    double aeps = ANGLE_EPS) const;

    virtual std::string state_string(const ob::State *state) const;
    virtual void printState(const ob::State *state, 
			    std::ostream &out = std::cout) const {
      out << state_string(state);
    }
    virtual visualization_msgs::MarkerArray displayable_state
    (const ob::State *state, std::string ns, int id, double scale=0.05,
     ColorPalate p=PRIMARYPALATE, double alpha=0.5, bool minimal=false) const;

    virtual bool update_model
    (const ob::State *new_state, EnvironmentInterface *env) const;

    virtual ob::StateSamplerPtr allocDefaultStateSampler() const;
    const an::OrderedCollisionOperations &allowed_collisions
    (const ob::State *state) const;
    bool disable_object_collisions(an::OrderedCollisionOperations &ops) const;
    bool disable_object_collisions(ob::State *state) const;

    const std::vector<unsigned int> &object_indexes() const 
    {return object_indexes_;}
    int robot_index() const {return robot_index_;}

    ThingType space_type(unsigned int ind) const;

    const PrimitiveList &primitives() const {return plist_;}

    const SupportSurfaceList &support_surfaces() const 
    {return support_surfaces_;}

    void convert_ompl_state_to_darrt_state
    (const ob::State *ompl_state, darrt_msgs::DARRTState &darrt_state) const;

    void convert_darrt_state_to_ompl_state
    (const darrt_msgs::DARRTState &darrt_state, ob::State *ompl_state) const;

    const ob::GoalSampleableRegion *goal() const {return goal_;}
    void set_goal(ob::GoalSampleableRegion *goal);
    void reset_goal();
    std::vector<const TransferPrimitive *> transfer_primitives() const
    {return tplist_;}
    const std::vector< std::pair<DARRTProjectionFunction, double> > &projection_functions() const 
    {return projection_functions_;}
    void set_projection_functions(const std::vector< std::pair<DARRTProjectionFunction, double> > &projection_functions) 
    {projection_functions_ = projection_functions;}
    //helpful for the object solver
    void setSubspaceDistance(int s) {subspace_distance_ = s;}
    int subspaceDistance() {return subspace_distance_;}

  protected:
    ros::Publisher viz_pub_;
    PrimitiveList plist_;
    std::vector<const TransferPrimitive *> tplist_;
    int robot_index_, subspace_distance_;
    std::vector<unsigned int> object_indexes_;
    ob::GoalSampleableRegion *goal_;
    StateTransformer transformer_;
    SupportSurfaceList support_surfaces_;
    std::vector< std::pair<DARRTProjectionFunction, double> > projection_functions_;
  };



  class DARRTStateValidityChecker : public CollisionAwareStateValidityChecker {
  public:

    DARRTStateValidityChecker
    (ompl::base::SpaceInformation *si, 
     EnvironmentInterface *environment_interface,
     collision_space::EnvironmentModel::AllowedCollisionMatrix
     &default_allowed_collisions,
     const std::vector<ob::StateValidityCheckerPtr> &state_validity_checkers);
    
    ~DARRTStateValidityChecker(){}
   
    virtual bool isValid(const ompl::base::State *ompl_state) const;

    const std::vector<ob::StateValidityCheckerPtr> &state_validity_checkers() 
      const {return state_validity_checkers_;}
    const ob::StateValidityCheckerPtr state_validity_checker(unsigned int ind) 
      const {return state_validity_checkers_[ind];}
    ob::StateValidityCheckerPtr state_validity_checker(unsigned int ind)
    {return state_validity_checkers_[ind];}

  protected:
    std::vector<ob::StateValidityCheckerPtr> state_validity_checkers_;
    const DARRTStateSpace *space_;
    ros::Publisher collision_pub_;
  };


  class ProjectibleStateSampler : public ob::CompoundStateSampler {
  public:
    ProjectibleStateSampler(const ob::StateSpace *space, 
			    const std::vector< std::pair<
			    DARRTProjectionFunction, double> > &functions,
			    ob::GoalSampleableRegion *goal=NULL);
    virtual void sampleUniform(ob::State *state);
    ob::StateSamplerPtr sampler(unsigned int i) const {return samplers_[i];}

  protected:
    ob::GoalSampleableRegion *goal_;
    bool first_sample_;
    std::vector< std::pair<DARRTProjectionFunction, double> > functions_;
    unsigned int sample_;
  };




}

#endif //space.hh
