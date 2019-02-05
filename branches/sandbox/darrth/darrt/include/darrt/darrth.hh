#ifndef __DARRT_DARRTH_SOLVER_HH__
#define __DARRT_DARRTH_SOLVER_HH__

#include "darrt/goal.hh"
#include "darrt/solver.hh"
#include "darrt/object_solver.hh"

namespace darrt {
  
  class DARRTHSolver : public InterruptableSolver {
  public:
    DARRTHSolver(ros::NodeHandle n);
    bool configure(const Goal &goal);
    bool plan(const Goal &goal) {float t; return plan(goal, t);}
    bool plan(const Goal &goal, float &time) {float t; std::vector<float> v; return plan(goal, time, t, v);}
    bool plan(const Goal &goal, float &time, float &object_time, std::vector<float> &subgoal_times);
    bool display_solution(bool step) const;
    bool execute_solution() const;
    virtual ~DARRTHSolver();
    const DARRTSolver &darrt_solver() const {return darrt_solver_;}
    DARRTSolver &darrt_solver() {return darrt_solver_;}
    virtual bool cancel(ros::WallDuration timeout=ros::WallDuration(0));
    virtual bool reset(ros::WallDuration timeout=ros::WallDuration(0));
    const EnvironmentInterface *environemt_interface() const 
    {return environment_interface_;}
    EnvironmentInterface *environment_interface() {return environment_interface_;}
    const oc::PathControl *getSolutionPath();

  protected:
    DARRTSolver darrt_solver_;
    ObjectSolver object_solver_;
    bool configured_;
    Params info_;
    std::vector<const DARRTObject *> objects_;
    std::string goal_object_;
    EnvironmentInterface *environment_interface_;

    std::vector<oc::PathControl> path_;
    oc::PathControl *full_path_;

    virtual bool object_path_to_subgoals
    (const Goal &goal, std::vector<Goal> &subgoals) const;
  };
}

#endif //darrth.hh
