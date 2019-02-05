#ifndef PINGPONG_OPTIMIZER_H
#define PINGPONG_OPTIMIZER_H

#include <iostream>
#include <bingham/util.h>
#include "dynamics.h"
#include <eigen2/Eigen/Core>
#include <eigen2/Eigen/LU>

USING_PART_OF_NAMESPACE_EIGEN;


//-----------------TrajectoryOptimizer class---------------//

class TrajectoryOptimizer {

 public:
  TrajectoryOptimizer(int njoints);

  // testing purposes
  void test();

  void find_opt(MatrixXf optimal, ArmDynamics arm, VectorXf x_init, VectorXf x_desired, double T, double dt);

};

#endif
