#include <eigen2/Eigen/Core>
#include <eigen2/Eigen/LU>
#include <eigen2/Eigen/Array>
#include <iostream>
#include <bingham/util.h>
#include "dynamics.h"
#include "optimizer.h"

using namespace std;
USING_PART_OF_NAMESPACE_EIGEN;

//-----GLOBAL VARIABLES--------//
int num_joints;

//-----CONSTANTS--------------//
float DELTA = .001;
float TOLERANCE = 0.00000000000000000000001;


//for debugging use
bool print_traj = 0; //for each iteration
bool print_dJdalpha = 0;
bool print_alphain = 0;
bool print_subalphas = 0;
bool print_dfdxlist = 0;


TrajectoryOptimizer::TrajectoryOptimizer(int njoints) {
  num_joints = njoints;
}


void print(MatrixXf m) {
  for(int i = 0; i < m.col(0).size(); i++) {
    for(int j = 0; j < m.row(0).size(); j++) {
      cout << std::setprecision(4) << m(i, j)<<" ";
    }
    cout<<"\n\n";
  }
}

MatrixXf get_Q() {
  MatrixXf Q = 10000*MatrixXf::Identity(2*num_joints, 2*num_joints);
  Q.block(0, 0, num_joints, num_joints) = MatrixXf::Zero(num_joints, num_joints);
  return Q;
  //return 1 * MatrixXf::Identity(2*num_joints, 2*num_joints);
}

MatrixXf get_R() {
  return 0 * MatrixXf::Identity(num_joints, num_joints);
}

//cost function
double cost(VectorXf x_desired, MatrixXf alpha_in, MatrixXf trajectory, double dt, int N) {

  MatrixXf cost = MatrixXf::Zero(1, 1);
  MatrixXf R = get_R() * dt;
   for(int i = 0; i < N; i++) {
    VectorXf torque = alpha_in.col(i); 
    cost += torque.transpose() * R * torque;
  }
  // end state error cost
  MatrixXf error = trajectory.col(N) - x_desired;
  cost += error.transpose() * get_Q() * error;

  return cost(0, 0);

}


MatrixXf compute_gradients(MatrixXf trajectory, MatrixXf alpha, MatrixXf *dfdx_list, MatrixXf *dfdu_list, VectorXf x_desired, double dt, int N) {

  MatrixXf R = get_R() * dt;

  MatrixXf dfdx = MatrixXf::Zero(2*num_joints, 2*num_joints);
  //dfdx.block(0, 0, num_joints, num_joints) = MatrixXf::Zero(num_joints, num_joints);
  dfdx.block(0, num_joints, num_joints, num_joints) = MatrixXf::Identity(num_joints, num_joints);
  MatrixXf dfdu(2*num_joints, num_joints);
  dfdu.block(0, 0, num_joints, num_joints) = MatrixXf::Zero(num_joints, num_joints);
  MatrixXf dgdu = 2 * alpha.col(N-1).transpose() * R;
  MatrixXf dudalpha = MatrixXf::Zero(num_joints, num_joints * N);
  MatrixXf dgdx = MatrixXf::Zero(1, 2*num_joints);

  dfdu.block(num_joints, 0, num_joints, num_joints) = dfdu_list[N];

  dgdu = 2 * alpha.col(N-1).transpose() * R;
  dudalpha = MatrixXf::Zero(num_joints, num_joints * N);
  dudalpha.block(0, num_joints * (N-1), num_joints, num_joints) = MatrixXf::Identity(num_joints, num_joints);

  //start from N
  dfdx.block(num_joints, 0, num_joints, 2*num_joints) = dfdx_list[N];

  MatrixXf F_x = dfdx;
  MatrixXf G_x = dgdx;

  MatrixXf F_alpha, G_alpha;
  F_alpha = dfdu * dudalpha;
  G_alpha = dgdu * dudalpha;

  MatrixXf Q = get_Q();
  MatrixXf y = -2 * Q * (trajectory.col(N) - x_desired);

  MatrixXf dJdalpha = MatrixXf::Zero(num_joints * N, 1);
  dJdalpha = (G_alpha.transpose() - F_alpha.transpose() * y) * dt;

  for(int i = N-1; i > -1; i--) {

    dfdu.block(num_joints, 0, num_joints, num_joints) = dfdu_list[i];
    dgdu = 2 * alpha.col(i).transpose() * R;
    dudalpha = MatrixXf::Zero(num_joints, num_joints *N);
    dudalpha.block(0, num_joints * i, num_joints, num_joints) = MatrixXf::Identity(num_joints, num_joints);

    F_alpha = dfdu * dudalpha;
    G_alpha = dgdu * dudalpha;

    //dJdalpha += (G_alpha.transpose() - F_alpha.transpose() * y)*dt;

    dfdx.block(num_joints, 0, num_joints, 2*num_joints) = dfdx_list[i];
    F_x = dfdx;

    y = y + (F_x.transpose() * y - G_x.transpose())*dt;
    //y = (MatrixXf::Identity(2*num_joints, 2*num_joints) - F_x.transpose()).inverse() * (y - G_x.transpose()*dt);
    //y = (MatrixXf::Identity(2*num_joints, 2*num_joints) - F_x.transpose()).inverse() * (y - G_x.transpose() * dt);
    dJdalpha += (G_alpha.transpose() - F_alpha.transpose() * y) * dt;

  }

  MatrixXf dJdalpha_reshape(num_joints, N);
  for(int i = 0; i < N; i++) {
    dJdalpha_reshape.col(i) = dJdalpha.block(num_joints * i, 0, num_joints, 1);
  }
  return dJdalpha_reshape; 

}
/*

MatrixXf compute_gradients(MatrixXf trajectory, MatrixXf alpha, MatrixXf *dfdx_list, MatrixXf *dfdu_list, VectorXf x_desired, double dt, int N) {

  MatrixXf R = get_R() * dt;

  MatrixXf dfdx(2*num_joints, 2*num_joints);
  dfdx.block(0, 0, num_joints, num_joints) = MatrixXf::Zero(num_joints, num_joints);
  dfdx.block(0, num_joints, num_joints, num_joints) = MatrixXf::Identity(num_joints, num_joints);
  MatrixXf dfdu(2*num_joints, num_joints);
  dfdu.block(0, 0, num_joints, num_joints) = MatrixXf::Zero(num_joints, num_joints);
  MatrixXf dgdu = 2 * alpha.col(N-1).transpose() * R;
  MatrixXf dudalpha = MatrixXf::Zero(num_joints, num_joints * N);
  MatrixXf dgdx = MatrixXf::Zero(1, 2*num_joints);
 
  //start from N
  dfdx.block(num_joints, 0, num_joints, 2*num_joints) = dfdx_list[N];

  MatrixXf F_x = dfdx;
  MatrixXf G_x = dgdx;

  MatrixXf F_alpha, G_alpha;

  MatrixXf Q = get_Q();
  MatrixXf y = -2 * Q * (trajectory.col(N) - x_desired);
  y = y + (F_x.transpose() * y - G_x.transpose())*dt;

  MatrixXf dJdalpha = MatrixXf::Zero(num_joints * N, 1);

  for(int i = N-1; i > -1; i--) {

    dfdu.block(num_joints, 0, num_joints, num_joints) = dfdu_list[i];
    dgdu = 2 * alpha.col(i).transpose() * R;
    dudalpha = MatrixXf::Zero(num_joints, num_joints *N);
    dudalpha.block(0, num_joints * i, num_joints, num_joints) = MatrixXf::Identity(num_joints, num_joints);

    F_alpha = dfdu * dudalpha;
    G_alpha = dgdu * dudalpha;

    dJdalpha += (G_alpha.transpose() - F_alpha.transpose() * y)*dt;

    dfdx.block(num_joints, 0, num_joints, 2*num_joints) = dfdx_list[i];
    F_x = dfdx;

    //y = y + (F_x.transpose() * y - G_x.transpose())*dt;
    y = (MatrixXf::Identity(2*num_joints, 2*num_joints) - F_x.transpose()).inverse() * (y - G_x.transpose()*dt);


  }

  MatrixXf dJdalpha_reshape(num_joints, N);
  for(int i = 0; i < N; i++) {
    dJdalpha_reshape.col(i) = dJdalpha.block(num_joints * i, 0, num_joints, 1);
  }
  return dJdalpha_reshape; 

}
*/



/*
void simulate_trajectory(ArmDynamics arm, MatrixXf x_init, MatrixXf alpha, double dt, int N, MatrixXf &trajectory, MatrixXf *dfdx_list, MatrixXf *dfdu_list) {

  trajectory.col(0) = x_init;

  for(int i = 1; i < N + 1; i++) {
    trajectory.block(0, i, num_joints, 1) = trajectory.block(0, i-1, num_joints, 1) + trajectory.block(num_joints, i-1, num_joints, 1) * dt;
    trajectory.block(num_joints, i, num_joints, 1) = trajectory.block(num_joints, i-1, num_joints, 1) + alpha.col(i-1) * dt;
  }

  MatrixXf dfdx = MatrixXf::Zero(num_joints, 2*num_joints);
  MatrixXf dfdu = MatrixXf::Identity(num_joints, num_joints);

  for(int i = 0; i < N + 1; i++) {
    dfdx_list[i] = dfdx;
    dfdu_list[i] = dfdu;
  }

}
*/


// trajectory(2 * num_joints, alpha.row(0).size() + 1), gradients(x, x)
void simulate_trajectory(ArmDynamics arm, MatrixXf x_init, MatrixXf alpha, double dt, int N, MatrixXf &trajectory, MatrixXf *dfdx_list, MatrixXf *dfdu_list) {

  double q[num_joints], dq[num_joints], ddq[num_joints], u[num_joints];
  double **gradients = new_matrix2(num_joints, 2*num_joints+1);

  // initiate
  for (int i = 0; i < num_joints; i++) {
    q[i] = x_init(i, 0);
    dq[i] = x_init(num_joints + i, 0);
    trajectory(i, 0) = q[i];
    trajectory(num_joints + i, 0) = dq[i];
  }

  // moved from inside first for loop
  MatrixXf dfdx(num_joints, 2*num_joints);
  MatrixXf dfdu = MatrixXf::Identity(num_joints, num_joints);

  //simulate forward
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < num_joints; j++) {
      u[j] = alpha(j, i);
    }
    arm.dynamics(ddq, gradients, q, dq, u);
    
    for (int j = 0; j < num_joints; j++) {
      q[j] += dt * dq[j];
      dq[j] += dt * ddq[j];
      trajectory(j, i + 1) = q[j];
      trajectory(j + num_joints, i + 1) = dq[j];
      //update gradients  
      for(int k = 0; k < 2*num_joints; k++)
	dfdx(j, k) = gradients[j][k]; //partial derivative of ddq[j] w.r.t. q[k], dq[k]
      dfdu(j, j) = gradients[j][2*num_joints];
    }
 
    dfdx_list[i] = dfdx;
    dfdu_list[i] = dfdu;
  }

  // added this part
  arm.dynamics(ddq, gradients, q, dq, u);
  for (int j = 0; j < num_joints; j++) {
    for (int k = 0; k < 2*num_joints; k++)
      dfdx(j, k) = gradients[j][k];
    dfdu(j, j) = gradients[j][2*num_joints];
  }
  dfdx_list[N] = dfdx;
  dfdu_list[N] = dfdu;

  free_matrix2(gradients);

}



void function(ArmDynamics arm, MatrixXf x_init, MatrixXf x_desired, double dt, int N, MatrixXf alpha_in, MatrixXf &dJdalpha, double &J, MatrixXf &trajectory) {
  /*
  for(int i = 0; i < N; i++) {
    trajectory.col(i) = x;
    u = alpha.col(i);
    u_tape.col(i) = u;
    x = x + dynamics(x, u, N) * dt;
  }
  */

  MatrixXf dfdx[N+1]; // changed from N
  MatrixXf dfdu[N+1];

  simulate_trajectory(arm, x_init, alpha_in, dt, N, trajectory, dfdx, dfdu);

  if (print_traj) {
    printf("trajectory \n");
    print(trajectory);
  }

  if (print_dfdxlist) {
    for(int i = 0; i < N; i++) 
      print(dfdx[i]);
  }

  J = cost(x_desired, alpha_in, trajectory, dt, N);
  //printf("J %.2f\n", J);

  dJdalpha = compute_gradients(trajectory, alpha_in, dfdx, dfdu, x_desired, dt, N);
  if (print_dJdalpha) {
    printf("dJdalpha\n");
    print(dJdalpha);
  }

}

/*
void test() {

  num_joints = 1;
  ArmDynamics arm(num_joints);

  if (arm.load("data/arm3"))
    printf("done\n");
  else
  printf("FAILED\n");

  MatrixXf dfdx[11];
  MatrixXf dfdu[11];

  //MatrixXf x_init = 4*MatrixXf::Ones(2*num_joints, 1);
  //MatrixXf x_desired = 10*MatrixXf::Ones(2*num_joints, 1);
  MatrixXf dJdalpha(num_joints, 10);
  MatrixXf trajectory(2*num_joints, 11);
  double J;

  MatrixXf x_init(2*num_joints, 1);
  x_init(0, 0) = 0;
  x_init(1, 0) = 1.56;

  MatrixXf x_desired(2*num_joints, 1);
  x_desired(0, 0) = 1.56;
  x_desired(1, 0) = 1.56;

  //function(arm, x_init, x_desired, 0.01, 10, alpha, dJdalpha, J, trajectory);

  MatrixXf alpha = MatrixXf::Ones(num_joints, 10);
  simulate_trajectory(arm, x_init, alpha, 0.1, 10, trajectory, dfdx, dfdu);
  printf("zero\n");
  print(trajectory);

  alpha = 10*MatrixXf::Ones(2*num_joints, 10);
  simulate_trajectory(arm, x_init, alpha, 0.1, 10, trajectory, dfdx, dfdu);
  printf("one\n");
  print(trajectory);

}
*/

// cap off torques at 10
MatrixXf cap_torques(MatrixXf torques) {

  MatrixXf capped_torques = torques;

  int N = torques.row(0).size();
  for(int i = 0; i < N; i++) {
    for(int j = 0; j < num_joints; j++) {
      if(torques(j, i) > 10) 
	capped_torques(j, i) = 10;
    }
  }

  return capped_torques;

}



void TrajectoryOptimizer::find_opt(MatrixXf optimal, ArmDynamics arm, VectorXf x_init, VectorXf x_desired, double T, double dt) {

  int N = T / dt;
  //printf("N %d\n", N);

  //MatrixXf alpha_in = 5*MatrixXf::Ones(num_joints, N);
  MatrixXf alpha_in = 0.01*MatrixXf::Ones(num_joints, N);
  for(int i = 0; i < N; i++) {
    alpha_in(1, i) = 4.0;
  }

  MatrixXf dJdalpha(num_joints, N);
  double J;
  MatrixXf trajectory(2*num_joints, N + 1);

  function(arm, x_init, x_desired, dt, N, alpha_in, dJdalpha, J, trajectory);

  double delta = DELTA;
  double previous_J = 0;
  int iteration = 0;
  while(fabs(J - previous_J) > TOLERANCE) {
    //printf("iteration %d\n", iteration);
    previous_J = J;
    /*
    MatrixXf alpha_min = alpha_in - delta * dJdalpha;
    double min_cost = cost(x_init, x_desired, alpha_min, dt);
    //double min_delta = delta; 
    //line search
    for(int k = 0; k < 3; k++) {
      MatrixXf alpha_temp = alpha_in - LINE_SEARCH[k] * delta * dJdalpha;
      double cost_temp = cost(x_init, x_desired, alpha_temp, dt);
      if(cost_temp < min_cost) {
	min_cost = cost_temp;
	alpha_min = alpha_temp;
      }
    }
    alpha_in = alpha_min;
    */
    alpha_in = alpha_in - delta * dJdalpha;
    if (print_alphain) {
      printf("alpha in \n");
      print(alpha_in);
    }

    function(arm, x_init, x_desired, dt, N, alpha_in, dJdalpha, J, trajectory);
    iteration++;
    //printf("fabs %.2f\n", fabs(J - previous_J));
    printf("cost: %.6f\n", J);
  }

  printf("theoretical trajectory: \n");
  print(trajectory);
  printf("torques: \n");
  print(alpha_in);
  optimal = cap_torques(alpha_in);

}

//for debug
void TrajectoryOptimizer::test() {

  num_joints = 3;
  ArmDynamics arm(num_joints);

  if (arm.load("data/arm3"))
    printf("done\n");
  else
  printf("FAILED\n");

  VectorXf x_init = VectorXf::Zero(2*num_joints);

  VectorXf x_desired = VectorXf::Zero(2*num_joints);
  x_desired(1) = 0.5;
  x_desired(4) = .2;

  double T = .2;
  double dt = 0.01;

  MatrixXf torques;

  find_opt(torques, arm, x_init, x_desired, T, dt);

}

/*

void test() {

  num_joints = 1;
  ArmDynamics arm(num_joints);

  if (arm.load("data/arm3"))
    printf("done\n");
  else
  printf("FAILED\n");

  MatrixXf x_init(2*num_joints, 1);
  x_init(0, 0) = 0.001;
  x_init(1, 0) = 0.001;

  MatrixXf x_desired(2*num_joints, 1);
  x_desired(0, 0) = 0.01;
  x_desired(1, 0) = 0.01;
  MatrixXf x_init = MatrixXf::Identity(2*num_joints, 1);
  MatrixXf x_desired  = 2*MatrixXf::Identity(2*num_joints, 1);

  double T = 1.0;
  double dt = 0.1;

  MatrixXf torques;

  find_opt(torques, arm, x_init, x_desired, T, dt);

}

*/

/*
int main() {
  num_joints = 3;
  ArmDynamics arm(num_joints);
  arm.load("data/arm3");
  //  MatrixXf x_init = 2*MatrixXf::Ones(2*num_joints, 1);
//

//  test();

  //  MatrixXf x_desired = 3 * MatrixXf::Ones(2*num_joints, 1);
  //printf("torques:\n");
  //find_opt(arm, x_init, x_desired, 0.7, 0.1);
  //test();
  // will almost certainly fail
  //MatrixXf commands = find_opt(x_init, x_desired, 1, 0.1);
  //print(commands);
//  cout<<"all's good\n";
//}
*/
