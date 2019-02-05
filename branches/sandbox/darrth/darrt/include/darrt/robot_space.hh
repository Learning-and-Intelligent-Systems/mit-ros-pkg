#ifndef __ARM_PUSHING_HH__
#define __ARM_PUSHING_HH__

#include "darrt/collision_aware_types.hh"
#include "darrt/environment_interface.hh"
#include "darrt/space.hh"
#include "darrt/types.hh"

#include <arm_navigation_msgs/GetMotionPlan.h>
#include <planning_environment/util/kinematic_state_constraint_evaluator.h>



namespace an=arm_navigation_msgs;

namespace darrt {

  //for a robot chain
  //really, we should have one for the
  //whole robot: base + chain
  //but we'll get there

  class RobotStateSpace : public ob::CompoundStateSpace, public Approximable, 
			  public CollisionAwareStateSpace {
  public:
    virtual void convert_ompl_to_robot_state(const ob::State *ompl_state, 
					     an::RobotState &robot_state) const=0;
    virtual void convert_robot_to_ompl_state(const an::RobotState &robot_state,
					     ob::State *ompl_state) const=0;
    virtual void convert_ompl_to_kinematic_state(const ob::State *ompl_state, 
						 pm::KinematicState &kstate) const=0;
    virtual void convert_kinematic_to_ompl_state(const pm::KinematicState &kstate,
						 ob::State *ompl_state) const=0;
  };

  class CompoundRobotStateSpace : public RobotStateSpace, public Displayable {
  public:
    typedef ob::CompoundState StateType;
    CompoundRobotStateSpace
    (const ros::NodeHandle &node_handle, EnvironmentInterface *env);
    
    double distance(const ob::State *destination, const ob::State *source) const;
    ob::State *allocState() const;

    void addSubspace(std::string group_name, const ob::RealVectorBounds &bounds);

    ob::State *get_state(unsigned int ind, ob::State *state) const;
    const ob::State *get_state(unsigned int ind, const ob::State *state) const;
    RobotStateSpace *get_subspace(unsigned int ind);
    const RobotStateSpace *get_subspace(unsigned int ind) const;
    
    std::string state_string(const ob::State *state) const
    {std::stringstream ss; printState(state, ss); return ss.str();}
    void printState(const ob::State *state,
		    std::ostream &out = std::cout) const;

    visualization_msgs::MarkerArray displayable_state
    (const ob::State *state, std::string ns, int id, double scale=0.05,
     ColorPalate p=PRIMARYPALATE, double alpha=0.5, bool minimal=false) const;

    bool near_states(const ob::State *s1, const ob::State *s2,
			     double deps=DIST_EPS, double aeps=ANGLE_EPS) const;

    double between(const ob::State *state, const ob::State *source,
		   const ob::State *destination,
		   double eps=DIST_EPS, double aeps=ANGLE_EPS) const;

    bool update_model(const ob::State *new_state, EnvironmentInterface *env) const;

    void convert_ompl_to_robot_state(const ob::State *ompl_state, 
				     an::RobotState &robot_state) const;
    void convert_robot_to_ompl_state(const an::RobotState &robot_state,
				     ob::State *ompl_state) const;
    void convert_ompl_to_kinematic_state(const ob::State *ompl_state, 
					 pm::KinematicState &kstate) const;
    void convert_kinematic_to_ompl_state(const pm::KinematicState &kstate,
					 ob::State *ompl_state) const;
    const EnvironmentInterface *environment_interface() const {return environment_interface_;}
    EnvironmentInterface *environment_interface() {return environment_interface_;}

    ~CompoundRobotStateSpace() {delete display_id_;}
  protected:
    ros::NodeHandle node_;
    int *display_id_;
    EnvironmentInterface *environment_interface_;
  };

  class RobotStateValidityChecker : public CollisionAwareStateValidityChecker {
  public:
    RobotStateValidityChecker
    (ob::SpaceInformation *si, 
     collision_space::EnvironmentModel::AllowedCollisionMatrix 
     &default_allowed_collision_matrix, bool update_model=true);
    
    bool init(const an::GetMotionPlan::Request &request);
    bool isValid(const ob::State *state) const;

  protected:
    an::Constraints get_physical_constraints
    (const an::Constraints &constraints);

    const RobotStateSpace *space_;
    planning_environment::KinematicConstraintEvaluatorSet 
    path_constraint_evaluator_set_;
    ros::Publisher collision_pub_;
    bool update_model_;
  };
}

#endif //robot_space.hh
