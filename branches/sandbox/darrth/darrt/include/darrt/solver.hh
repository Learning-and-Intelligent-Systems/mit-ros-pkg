#ifndef __DARRT_SOLVER__HH__
#define __DARRT_SOLVER__HH__

#include <string>
#include <vector>

//local includes
#include "darrt/environment_interface.hh"
#include "darrt/goal.hh"
#include "darrt/primitive.hh"
#include "darrt/utils.hh"

//ros includes
#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/OrderedCollisionOperations.h>
#include <arm_navigation_msgs/RobotState.h>
#include <arm_navigation_msgs/RobotTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

//ompl includes
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/State.h>
#include <ompl/control/SimpleSetup.h>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace an = arm_navigation_msgs;

//this should acquire unit tests
//hahahhahaa

namespace darrt {

  class InterruptableSolver {
  public:
    enum Phase {READY, SOLVING, WAITING};
    InterruptableSolver() {phase_ = READY;}
    bool cancelled() const 
    {return phase_ == WAITING || !ros::ok();}
    virtual bool cancel(ros::WallDuration timeout=ros::WallDuration(0));
    virtual bool reset(ros::WallDuration timeout=ros::WallDuration(0));
 
  protected:
    virtual ob::PlannerTerminationCondition terminate_after(double time) const;
    Phase phase_;
  };    

  class DARRTSolver : public InterruptableSolver {
  public:

    DARRTSolver(ros::NodeHandle n);
    
    virtual bool configure(const Goal &goal);
    virtual bool display_solution(bool step=false, unsigned int resolution=1) const;
    virtual bool display_solution(const oc::PathControl &path, bool step=false,
				  int offset=0, unsigned int resolution=1) const;
    virtual bool execute_solution() const;
    virtual bool execute_solution(const oc::PathControl &path) const;
    virtual bool plan(const Goal &goal)
    {return plan(goal, NULL);}
    virtual bool plan(const Goal &goal, float &planning_time)
    {return plan(goal, NULL, planning_time);}
    virtual bool plan(const Goal &goal, const ob::State *starting_state)
    {float pt; return plan(goal, starting_state, pt);} 
    virtual bool plan(const Goal &goal, const ob::State *starting_state, 
		      float &planning_time);
    EnvironmentInterface *environment_interface() 
    {return environment_interface_;}
    const EnvironmentInterface *environment_interface() const
    {return environment_interface_;}
    const oc::PathControl *getSolutionPath() const;
    const oc::SpaceInformation *space_information() const;
    const oc::SpaceInformationPtr getSpaceInformation() const;
    virtual bool reset(ros::WallDuration timeout=ros::WallDuration(0));
    const ob::State *starting_state() const {return starting_state_;}
    void sample_goal_state(ob::State *goal) const 
    {return goal_->as<ob::GoalSampleableRegion>()->sampleGoal(goal);}

    //moved into public for testing
    virtual bool initialize_environment(an::RobotState &starting_state, 
					const std::vector<const DARRTObject *> &moveable_objects);
    //virtual bool setup_planning_scene(const Goal &goal);
    virtual bool setup_problem(const an::RobotState &starting_state,
			       const Goal &goal);

    virtual ~DARRTSolver();

  protected:
    void listen();

    virtual bool setup_space(const Goal &goal);
    virtual bool setup_state_validity_checker(const Goal &goal);
    virtual bool check_object_headers(const Goal &goal);



    ompl::control::SimpleSetup *solver_;
    EnvironmentInterface *environment_interface_;
    arm_navigation_msgs::OrderedCollisionOperations default_collisions_;
    ros::NodeHandle node_;
    ob::State *starting_state_;
    ob::GoalPtr goal_;
    Params info_;
    bool configured_;
    bool cancelled_;
    ros::ServiceClient set_planning_scene_, get_planning_scene_, get_robot_state_;
    std::vector<Primitive *> primitives_;
  };

  
}

#endif //solver.hh
