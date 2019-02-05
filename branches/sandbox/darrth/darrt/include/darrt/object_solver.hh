#ifndef __DARRT_OBJECT_SOLVER_HH__
#define __DARRT_OBJECT_SOLVER_HH__

#include "darrt/solver.hh"

namespace darrt {

  class ObjectSolver : public DARRTSolver {
  public:
    ObjectSolver(ros::NodeHandle n) : DARRTSolver(n) {}
    virtual bool setup_state_validity_checker(const Goal &goal);
    virtual bool display_solution(bool step=false, unsigned int resolution=1) const;
  };
}

#endif //object_solver.hh
