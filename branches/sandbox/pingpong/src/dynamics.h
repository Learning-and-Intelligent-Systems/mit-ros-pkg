#ifndef PINGPONG_DYNAMICS_H
#define PINGPONG_DYNAMICS_H

//#include <flann/flann.h>
#include <fastwam/SwingData.h>
//#include <pingpong/SwingData.h>
#include "gravity_table.h"

//typedef int flann_index_t;
//typedef int FLANNParameters;


struct nn_map_t {
  //flann_index_t index;
  //FLANNParameters params;
  double *X;    // input data  (n-by-dx)
  double *Y;    // output data (n-by-dy)
  int n;        // number of stored data rows
  int dx;       // length of each input row
  int dy;       // length of each output row
};

struct arm_dynamics_signal_t {
  double **Q;
  double **dQ;
  double **ddQ;
  double **U;
  int n;
  int d;
};


//-------------------------- ArmDynamics class --------------------------//

class ArmDynamics {
 public:
  ArmDynamics(int njoints) {
    full_maps = NULL;
    lwr_maps = NULL;
    num_joints = njoints;
  }

  int getNumJoints() {
    return num_joints;
  }

  bool isEmpty() {
    return (full_maps==NULL);
  }

  /*
   * File I/O
   */
  bool save(const char *base_name);
  bool load(const char *base_name);

  /*
   * Add swing data to the ArmDynamics model.
   */
  void addSwingData(const fastwam::SwingData &swing);
  //void addSwingData(const pingpong::SwingData &swing);

  /*
   * Computes joint accelerations and gradients w.r.t. (q,dq,u).
   * 'ddq' and 'gradients' must be pre-allocated.  'ddq' should be a length d array and
   * 'gradients' should be a (d-by-2d+2) matrix, which can be created with new_matrix2(d, 2*d+2).
   * Upon return:
   *   ddq[0,...,j-1] will contain the joint accelerations,
   *   gradients[i][0] will contain the partial derivatives of ddq[i] w.r.t. 1,
   *   gradients[i][1,...,j] will contain the partial derivatives of ddq[i] w.r.t. q,
   *   gradients[i][j+1,...,2*j] will contain the partial derivatives of ddq[i] w.r.t. dq, and
   *   gradients[i][2*j+1] will contain the partial derivative of ddq[i] w.r.t. u[i].
   */
  void dynamics(double *ddq, double **gradients, double *q, double *dq, double *u);
  

 protected:
  nn_map_t *full_maps;
  nn_map_t *lwr_maps;
  int num_joints;

  void addData(double ***X, double **Y, int n, int dx);
};



//------------------------------------------ Gravity Compensation -------------------------------------------------//

void get_gravity_torques(double *torques, double *joint_angles);
double **get_gravity_torques_for_trajectory(double **Q, int n);



//------------------------------------------ Swing Torques -------------------------------------------------//


typedef struct {
  double t;  // t=0 means no hit
  double pos[3];
  double normal[3];
  double vel[3];
  double pos_offset[3];
} hit_t;

typedef struct {
  int n;
  double *T;    // duration (time to hit)
  double **B;   // ball hit params (x,y,z)
  double **PP;  // paddle positions (at t_hit)
  double **PV;  // paddle velocities (at t_hit)
  double **PN;  // paddle normals (at t_hit)
  double ***U;  // swing torques
  double *E;    // swing errors
  int *N;       // swing lengths
} swing_table_t;

typedef struct {
  int n;
  double **U;
} swing_torques_t;


//void update_swing_torques(const fastwam::SwingData &swing, double *p_hit);
swing_torques_t get_swing_torques(swing_table_t *table, hit_t hit, double t0);
swing_table_t load_swing_table(char *path);





#endif
