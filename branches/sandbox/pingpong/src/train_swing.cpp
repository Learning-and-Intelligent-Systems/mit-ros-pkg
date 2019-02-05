#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <fastwam/MoveTo.h>
#include <fastwam/JointTrajectory.h>
#include <fastwam/TorqueTrajectory.h>
#include <fastwam/PaddleOrientationTrajectory.h>
#include <bingham/util.h>
#include "dynamics.h"
#include "kinematics.h"
#include "optimization.h"

// warning: keep this at the bottom of the includes!
#include <lapacke.h>


void moveto(double *ja);
void torque_swing(const fastwam::SwingData &swing, double noise, int smoothing);
void torque_swing_with_paddle_orientation(const fastwam::SwingData &swing, double *paddle_orientation);
void ros_sleep(int seconds);
void wait_for_swing();


ros::Publisher wam_moveto_pub;
ros::Publisher wam_joint_traj_pub;
ros::Publisher wam_torque_traj_pub;
ros::Publisher paddle_orient_traj_pub;
ArmDynamics *arm;
char *last_swing_name = NULL;
fastwam::SwingData last_swing;
bool got_swing = false;

double *paddle_normal_;




//-----------------------------  SWING OPTIMIZATION  --------------------------//


/*
 * Use gradient descent to find a set of wrist joint angles that results in the desired paddle orientation
 */
double *wrist_ik(double *joint_angles, double *paddle_orientation)
{
  double q[7];  // tmp joint angles
  double *w = &q[4];
  //double *w0 = &joint_angles[4];
  //double *ja = joint_angles;
  memcpy(q, joint_angles, 7*sizeof(double));
  double dfdq[7];
  double *dfdw = &dfdq[4];

  double pdes[6] = {0,0,0, paddle_orientation[0], paddle_orientation[1], paddle_orientation[2]};  // desired paddle state

  double *paddle = joints_to_paddle(q);  // paddle position + normal
  double fmax = dot(&paddle[3], paddle_orientation, 3);
  double wmax[3] = {w[0], w[1], w[2]};  // best wrist angles so far
  free(paddle);

  double s = .1;  // step size
  double a[3] = {.618, 1, 1.618};  // line search

  int max_iter = 50;
  for (int iter = 0; iter < max_iter; iter++) {
    
    // compute the gradient of f w.r.t w
    double **dPNdq = paddle_jacobian(q);
    vec_matrix_mult(dfdq, pdes, dPNdq, 6, 7);
    free_matrix2(dPNdq);

    // take steps in the direction of the gradient and pick the one with max f value
    double f[3];
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++)
	w[j] += a[i]*s*dfdw[j];
      paddle = joints_to_paddle(q);
      f[i] = dot(&paddle[3], paddle_orientation, 3);
      free(paddle);
      for (int j = 0; j < 3; j++)
	w[j] -= a[i]*s*dfdw[j];
    }
    int i = find_max(f,3);

    s *= a[i];  // update step size

    // take the step in wrist angle space
    for (int j = 0; j < 3; j++)
      w[j] += s*dfdw[j];

    // update best wrist angles solution
    if (f[i] > fmax) {
      fmax = f[i];
      memcpy(wmax, w, 3*sizeof(double));
    }

    // check for convergence
    if (f[i] > .999) {
      //printf("(iter=%d) ", iter);  //dbug
      break;
    }
  }

  //printf("fmax = %f, wmax = [%.2f, %.2f, %.2f], ja = [%.2f, %.2f, %.2f]\n", fmax, wmax[0], wmax[1], wmax[2], ja[4], ja[5], ja[6]);  //dbug

  double *wrist_angles;
  safe_calloc(wrist_angles, 3, double);
  memcpy(wrist_angles, wmax, 3*sizeof(double));

  return wrist_angles;
}



/*
 * Fills in swing.joint_angles and gradients
 */
void simulate_trajectory(ArmDynamics &arm, fastwam::SwingData &swing, double ***gradients, double *paddle_orientation)
{
  int num_joints = 7;  //arm.getNumJoints();
  double q[num_joints], dq[num_joints], ddq[num_joints], u[num_joints];

  // get initial arm state (q,dq)
  for (int j = 0; j < num_joints; j++)
    q[j] = swing.joint_angles[j];
  //memcpy(q, init_joint_angles, num_joints * sizeof(double));
  memset(dq, 0, num_joints * sizeof(double));

  // simulate trajectory
  int NT = swing.t.size();
  double **Q = new_matrix2(NT, num_joints);
  memcpy(Q[0], q, num_joints * sizeof(double));

  int d = num_joints;
  for (int i = 1; i < NT; i++) {

    // use arm dynamics to update first 4 joints

    for (int j = 0; j < 4; j++)
      u[j] = swing.joint_torques[(i-1)*d+j];

    if (gradients)
      arm.dynamics(ddq, gradients[i-1], q, dq, u);
    else
      arm.dynamics(ddq, NULL, q, dq, u);

    double dt = swing.t[i] - swing.t[i-1];
    for (int j = 0; j < 4; j++) {
      q[j] += dt*dq[j];
      dq[j] += dt*ddq[j];
    }

    // find wrist angles to match paddle orientation
    double *wrist_angles = wrist_ik(q, paddle_orientation);
    memcpy(&q[4], wrist_angles, 3*sizeof(double));

    memcpy(Q[i], q, num_joints * sizeof(double));
  }

  swing.joint_angles.clear();
  for (int i = 0; i < NT; i++)
    for (int j = 0; j < num_joints; j++)
      swing.joint_angles.push_back(Q[i][j]);

  free_matrix2(Q);
}


VectorXf swing_to_vector(const fastwam::SwingData &swing)
{
  //printf("swing_to_vector()\n");  //dbug

  int n = swing.joint_torques.size();
  VectorXf x = VectorXf::Zero(n);
  for (int i = 0; i < n; i++)
    x(i) = swing.joint_torques[i];

  return x;
}




/**************  SwingTester  **************/

class SwingTester {
public:
  SwingTester() {}
  virtual double test(fastwam::SwingData &swing) = 0;
};



/**************  GradientSwingTester  **************/

class GradientSwingTester : public SwingTester {
public:
  GradientSwingTester() {}
  //virtual double test(fastwam::SwingData &swing) = 0;
  virtual VectorXf gradient(fastwam::SwingData &swing, double ***dfdxu) = 0;
};


/**************  LinearGradientSwingTester  **************/

class LinearGradientSwingTester : public GradientSwingTester {

protected:
  double v0_des[3];  // line starting point
  double v_des[3];   // line velocity vector
  double s_des;      // paddle speed
  double n_des[3];   // paddle normal
  fastwam::SwingData last_swing;

  double line_weight;
  double speed_weight;
  double normal_weight;
  //double u_weight;
  double test_delay;

  bool swing_equals(const fastwam::SwingData &s1, const fastwam::SwingData &s2)
  {
    if (s1.t.size() != s2.t.size() || s1.joint_torques.size() != s2.joint_torques.size())
      return false;

    for (uint i = 0; i < s1.joint_torques.size(); i++)
      if (s1.t[i] != s2.t[i] || s1.joint_torques[i] != s2.joint_torques[i])
	return false;

    return true;
  }

public:

  LinearGradientSwingTester(double *line_start, double *line_velocity, double *paddle_normal)
  {
    memcpy(v0_des, line_start, 3*sizeof(double));
    memcpy(v_des, line_velocity, 3*sizeof(double));
    s_des = norm(v_des, 3);  // desired speed
    normalize(v_des, v_des, 3);
    memcpy(n_des, paddle_normal, 3*sizeof(double));

    line_weight = 1;
    speed_weight = .01; //.01;
    normal_weight = .01;
    test_delay = .1; //.1;
  }

  double test(fastwam::SwingData &swing)
  {
    //printf("test()\n"); //dbug

    //if (!swing_equals(swing, last_swing)) {
    //  simulate_trajectory(*arm, swing, NULL);
    //  last_swing = swing;
    //}
    
    int n = swing.t.size() - 2;
    int m = 7; //arm->getNumJoints();
    
    double g[n];
    
    //dbug
    //double swing_speed_err[n]
    double line_cost = 0, speed_cost = 0, normal_cost = 0;
    double line_costs[n], speed_costs[n], normal_costs[n];

    for (int i = 0; i < n; i++) {

      if (swing.t[i] - swing.t[0] < test_delay) {
	g[i] = 0;
	line_costs[i] = speed_costs[i] = normal_costs[i] = 0;
	continue;
      }

      double q[m], q2[m];
      for (int j = 0; j < m; j++) {
	q[j] = swing.joint_angles[i*m+j];
	q2[j] = swing.joint_angles[(i+1)*m+j];
      }
      double dt = swing.t[i+1] - swing.t[i];
      double dt2 = dt*dt;

      double *paddle_pose = joints_to_paddle(q);
      double *paddle_pose2 = joints_to_paddle(q2);
      double *P = paddle_pose;
      double *N = &paddle_pose[3];
      double *P2 = paddle_pose2;

      double dP[3];
      sub(dP, P, v0_des, 3);
      double d = dot(v_des, dP, 3);
      double line_err = dot(dP, dP, 3) - d*d;

      d = dist(P2, P, 3) / dt - s_des;
      double speed_err = d*d;

      //sub(dP, P2, P, 3);
      //double sv = dot(v_des, dP, 3) / dt;
      //double sh2 = dot(dP, dP, 3) / dt2 - sv*sv;
      //double speed_err = (sv - s_des)*(sv - s_des) + sh2;

      double normal_err = dist2(N, n_des, 3);

      //double u_err = U(i,:)*U(i,:)';

      // cost at time i
      g[i] = line_weight*line_err + speed_weight*speed_err + normal_weight*normal_err; // + u_weight*u_err;

      //dbug
      line_cost += line_weight*line_err;
      speed_cost += speed_weight*speed_err;
      normal_cost += normal_weight*normal_err;
      line_costs[i] = line_weight*line_err;
      speed_costs[i] = speed_weight*speed_err;
      normal_costs[i] = normal_weight*normal_err;

      free(paddle_pose);
      free(paddle_pose2);
    }

    double cost = sum(g,n);

    //dbug
    printf("line_costs = ["); for (int i = 0; i < n; i++) printf("%.3f ", line_costs[i]); printf("]\n");
    printf("speed_costs = ["); for (int i = 0; i < n; i++) printf("%.3f ", speed_costs[i]); printf("]\n");
    printf("normal_costs = ["); for (int i = 0; i < n; i++) printf("%.3f ", normal_costs[i]); printf("]\n");
    printf(" --> total cost = %.2f (%.2f, %2f, %2f)\n\n", cost, line_cost, speed_cost, normal_cost);  //dbug

    return cost;
  }

  VectorXf gradient(fastwam::SwingData &swing, double ***dfdxu)
  {
    //printf("gradient()\n");

    int NT = swing.t.size() - 1;
    int num_joints = 7; //arm->getNumJoints();

    // state gradients (dfdx, dfdu)
    //double ***gradients = new_matrix3(NT, num_joints, 2*num_joints + 2);

    //simulate_trajectory(*arm, swing, gradients);
    //last_swing = swing;

    int UD = num_joints;
    int XD = 2*num_joints + 1;
    int N = NT-1;
    double y[XD];
    memset(y, 0, XD*sizeof(double));

    double **dfdx = new_matrix2(XD,XD);  // [zeros(1,nx); zeros(nq,1+nq), eye(nq); A(:,:,i)];
    for (int j = 0; j < UD; j++)
      dfdx[1+j][UD+1+j] = 1;

    double **dfdu = new_matrix2(XD,UD);  // [zeros(1+nq,nq); B(:,:,i)];

    double dJdalpha[UD*N];

    for (int n = N-1; n >= 0; n--) {

      // compute dfdx & dfdu at time n
      for (int j = 0; j < 4; j++) {
	memcpy(dfdx[1+UD+j], dfdxu[n][j], 9*sizeof(double));
	dfdu[1+UD+j][j] = dfdxu[n][j][9];
      }
      //for (int j = 0; j < UD; j++) {
      //  memcpy(dfdx[1+UD+j], dfdxu[n][j], XD*sizeof(double));
      //  dfdu[1+UD+j][j] = dfdxu[n][j][XD];
      //}

      // compute dgdx & dgdu at time n
      double dgdx[XD], dgdu[UD];
      memset(dgdx, 0, XD*sizeof(double));
      memset(dgdu, 0, UD*sizeof(double));
      
      double q[UD], q2[UD];
      for (int j = 0; j < UD; j++) {
	q[j] = swing.joint_angles[n*UD+j];
	q2[j] = swing.joint_angles[(n+1)*UD+j];
      }
      double dt = swing.t[n+1] - swing.t[n];

      double *paddle_pose = joints_to_paddle(q);
      double *paddle_pose2 = joints_to_paddle(q2);
      double *P = paddle_pose;
      double *N = &paddle_pose[3];
      double *P2 = paddle_pose2;
      
      double **dPNdq = paddle_jacobian(q);
      double **dPNdq2 = paddle_jacobian(q2);
      double **dPdq = dPNdq;
      double **dNdq = &dPNdq[3];
      double **dPdq2 = dPNdq2;

      // line_err_gradient = [0, 2*(P - v0_des)'*(eye(3) - v_des*v_des')*dPdq, zeros(1,7)];
      double line_err_gradient[XD];
      memset(line_err_gradient, 0, XD*sizeof(double));
      double x[3] = {2*(P[0] - v0_des[0]), 2*(P[1] - v0_des[1]), 2*(P[2] - v0_des[2])};
      double **A = new_identity_matrix2(3);
      double **V = new_matrix2(3,3);
      outer_prod(V, v_des, v_des, 3, 3);
      sub(A[0], A[0], V[0], 9);
      vec_matrix_mult(x, x, A, 3, 3);
      vec_matrix_mult(&line_err_gradient[1], x, dPdq, 3, UD);

      // speed_err_gradient = [0, 2*(1 - dt*s_des/norm(P2-P))*(P2-P)'*[dPdq2 - dPdq, dt*dPdq2]] / dt^2;
      double speed_err_gradient[XD];
      memset(speed_err_gradient, 0, XD*sizeof(double));
      if (dist(P2, P, 3) > 1e-16) {
	sub(x, P2, P, 3);
	mult(x, x, 2*(1 - dt*s_des/dist(P2,P,3)), 3);
	//for (int j = 0; j < 3; j++)
	//  x[j] = 2*(P2[j] - P[j] - dt*s_des*v_des[j]);
	vec_matrix_mult(&speed_err_gradient[1], x, dPdq, 3, UD);
	vec_matrix_mult(&speed_err_gradient[1+UD], x, dPdq2, 3, UD);
	sub(&speed_err_gradient[1], &speed_err_gradient[1+UD], &speed_err_gradient[1], UD);
	mult(&speed_err_gradient[1], &speed_err_gradient[1], 1.0/(dt*dt), UD);
	mult(&speed_err_gradient[1+UD], &speed_err_gradient[1+UD], 1.0/dt, UD);
      }

      // normal_err_gradient = [0, 2*(N - n_des)'*dNdq, zeros(1,7)];
      double normal_err_gradient[XD];
      memset(normal_err_gradient, 0, XD*sizeof(double));
      sub(x, N, n_des, 3);
      mult(x, x, 2.0, 3);
      vec_matrix_mult(&normal_err_gradient[1], x, dNdq, 3, UD);

      // u_err_gradient = 2*U(i,:);

      // cost gradient at time n
      if (swing.t[n] - swing.t[0] >= test_delay) {
	for (int j = 0; j < XD; j++)
	  dgdx[j] = line_weight*line_err_gradient[j] + speed_weight*speed_err_gradient[j] + normal_weight*normal_err_gradient[j];

	// dgdu = u_weight*u_err_gradient;
      }


      // adjoint equations:
      //   y = (eye(XD) - F_x'.*dt) \ (y - G_x'.*dt);
      //   dJdalpha = dJdalpha + (G_alpha'-F_alpha'*y).*dt; %ADD this step's contribution to dJdalpha

      free_matrix2(A);
      A = new_matrix2(XD, XD);
      transpose(A, dfdx, XD, XD);
      mult(A[0], A[0], -dt, XD*XD);
      for (int j = 0; j < XD; j++)
	A[j][j] += 1.0;
      double b[XD];
      mult(b, dgdx, -dt, XD);
      add(b, b, y, XD);
      
      int pivots[XD];
      LAPACKE_dgesv(LAPACK_COL_MAJOR, XD, 1, A[0], XD, pivots, b, XD);
      memcpy(y, b, XD*sizeof(double));

      vec_matrix_mult(b, y, dfdu, XD, UD);
      sub(b, dgdu, b, UD);
      mult(b, b, dt, UD);
      memcpy(&dJdalpha[n*UD], b, UD*sizeof(double));


      free(paddle_pose);
      free(paddle_pose2);
      free_matrix2(dPNdq);
      free_matrix2(dPNdq2);
      free_matrix2(A);
      free_matrix2(V);
    }

    free_matrix2(dfdx);
    free_matrix2(dfdu);

    //dbug
    //printf("dCdU = [");
    //for (int i = 0; i < UD*N; i++)
    //  printf("%.2f ", dJdalpha[i]);
    //printf("]\n");

    // convert dJdalpha to VectorXf
    VectorXf dCdU = VectorXf::Zero(UD*(N+1));
    for (int i = 0; i < UD*N; i++)
      dCdU(i) = dJdalpha[i];

    return dCdU;
  }
};








class GradientSwingOptimizer : public GradientOptimizer {
 private:
  GradientSwingTester *swing_tester_;
  fastwam::SwingData swing_;
 public:
  GradientSwingOptimizer(GradientSwingTester *T);
  float evaluate(VectorXf x);
  VectorXf gradient(VectorXf x);
  fastwam::SwingData optimizeSwing(const fastwam::SwingData &initial_swing);
};

GradientSwingOptimizer::GradientSwingOptimizer(GradientSwingTester *T) :
  GradientOptimizer(.1),  //.01
  swing_tester_(T)
{
}

float GradientSwingOptimizer::evaluate(VectorXf x)
{
  int n = swing_.joint_torques.size();
  for (int i = 0; i < n; i++)
    swing_.joint_torques[i] = x(i);

  simulate_trajectory(*arm, swing_, NULL, paddle_normal_);

  return swing_tester_->test(swing_);
}

VectorXf GradientSwingOptimizer::gradient(VectorXf x)
{
  int n = swing_.joint_torques.size();
  for (int i = 0; i < n; i++)
    swing_.joint_torques[i] = x(i);

  int num_joints = arm->getNumJoints();
  double ***dfdxu = new_matrix3(n, num_joints, 2*num_joints + 2);

  simulate_trajectory(*arm, swing_, dfdxu, paddle_normal_);

  VectorXf g = swing_tester_->gradient(swing_, dfdxu);

  free_matrix3(dfdxu);

  return g;
}

fastwam::SwingData GradientSwingOptimizer::optimizeSwing(const fastwam::SwingData &initial_swing)
{
  swing_ = initial_swing;
  VectorXf x = optimize(swing_to_vector(initial_swing));

  int n = initial_swing.joint_torques.size();
  for (int i = 0; i < n; i++)
    swing_.joint_torques[i] = x(i);

  return swing_;
}





//-------------------------------  ARM CONTROL  ------------------------------------//




void moveto(double *ja)
{
  fastwam::MoveTo msg;
  for (int j = 0; j < 7; j++)
    msg.joint_angles.push_back(ja[j]);
  wam_moveto_pub.publish(msg);
}


void jacobian_swing(double *joint_angles_start, double *dp)
{
  double t0 = ros::Time::now().toSec() + 0.01;
  double dt = .02;

  fastwam::JointTrajectory msg;

  //double dp[6] = {.01, 0, .005, 0, 0, 0};
  //double dp[6] = {.02, 0, .007, 0, 0, 0};
  //double w[7] = {1,1,.1,1,.3,1,1};
  double w[7] = {1,1,.1,1,.1,.1,.1};
  double ja[7];
  memcpy(ja, joint_angles_start, 7*sizeof(double));
  for (int j = 0; j < 30; j++) {
    double *dj = paddle_to_joints_displacement_weighted(ja, dp, w);
    for (int k = 0; k < 4/*7*/; k++)
      ja[k] += dj[k];
    free(dj);

    double *wrist_angles = wrist_ik(ja, paddle_normal_);
    memcpy(&ja[4], wrist_angles, 3*sizeof(double));
    free(wrist_angles);

    //if (j >= 10) {
    msg.times.push_back(t0 + dt*j);
    for (int k = 0; k < 7; k++)
      msg.joint_angles.push_back(ja[k]);
    //}
  }
  //    msg.times.push_back(t0 + dt*100);
  //    for (int k = 0; k < 7; k++)
  //      msg.joint_angles.push_back(ja[k]);

  wam_joint_traj_pub.publish(msg);
  got_swing = false;
}


void torque_swing(const fastwam::SwingData &swing, double noise, int smoothing)
{
  int d = 7;
  fastwam::TorqueTrajectory msg;
  msg.times = swing.t;
  for (int j = 0; j < d; j++)
    msg.joints.push_back(j);
  msg.torques = swing.joint_torques;

  double t0 = ros::Time::now().toSec() + .01;
  for (uint i = 0; i < msg.times.size(); i++)
    msg.times[i] = t0 + swing.t[i] - swing.t[0];

  if (noise > 0.0)
    for (uint i = 0; i < msg.torques.size(); i++)
      msg.torques[i] += noise*(frand() - 0.5);

  for (int s = 0; s < smoothing; s++) {
    double torques[msg.torques.size()];
    for (uint i = d; i < msg.torques.size()-d; i++)
      torques[i] = (msg.torques[i-d] + msg.torques[i] + msg.torques[i+d]) / 3.0;
    for (uint i = d; i < msg.torques.size()-d; i++)
      msg.torques[i] = torques[i];
  }

  wam_torque_traj_pub.publish(msg);
  got_swing = false;
}


void torque_swing_with_paddle_orientation(const fastwam::SwingData &swing, double *paddle_orientation)
{
  double t0 = ros::Time::now().toSec() + .01;

  // send the paddle orientation message
  fastwam::PaddleOrientationTrajectory paddle_msg;
  paddle_msg.times.push_back(t0);
  paddle_msg.times.push_back(t0 + swing.t.back() - swing.t.front());
  paddle_msg.orientations.push_back(paddle_orientation[0]);
  paddle_msg.orientations.push_back(paddle_orientation[1]);
  paddle_msg.orientations.push_back(paddle_orientation[2]);
  paddle_msg.orientations.push_back(paddle_orientation[0]);
  paddle_msg.orientations.push_back(paddle_orientation[1]);
  paddle_msg.orientations.push_back(paddle_orientation[2]);
  paddle_orient_traj_pub.publish(paddle_msg);

  // send the torque swing message (with no wrist)
  int d = 4;
  fastwam::TorqueTrajectory msg;
  msg.times = swing.t;
  for (int j = 0; j < d; j++)
    msg.joints.push_back(j);
  for (uint i = 0; i < swing.t.size(); i++)
    for (int j = 0; j < d; j++)
      msg.torques.push_back(swing.joint_torques[i*7+j]);

  for (uint i = 0; i < msg.times.size(); i++)
    msg.times[i] = t0 + swing.t[i] - swing.t[0];

  printf("Commanding swing msg with n=%d\n", msg.times.size());  //dbug

  wam_torque_traj_pub.publish(msg);
  got_swing = false;
}


void save_swing(const char *base_name, const fastwam::SwingData &swing)
{
  int n = swing.t.size();
  double **times = new_matrix2(n,1);
  double **angles = new_matrix2(n,7);
  double **torques = new_matrix2(n,7);

  for (int i = 0; i < n; i++) {
    times[i][0] = swing.t[i];
    for (int j = 0; j < 7; j++) {
      angles[i][j] = swing.joint_angles[7*i+j];
      torques[i][j] = swing.joint_torques[7*i+j];
    }
  }

  char fname[1024];
  sprintf(fname, "%s_times.txt", base_name);
  save_matrix(fname, times, n, 1);
  sprintf(fname, "%s_angles.txt", base_name);
  save_matrix(fname, angles, n, 7);
  sprintf(fname, "%s_torques.txt", base_name);
  save_matrix(fname, torques, n, 7);

  //cleanup
  free_matrix2(times);
  free_matrix2(angles);
  free_matrix2(torques);
}


void swingdata_callback(const fastwam::SwingData &msg)
{
  printf("Received swing msg with n=%d (%.2f secs)\n", msg.t.size(), msg.t.back() - msg.t.front());  //dbug

  printf("Adding swing data..."); fflush(0);
  arm->addSwingData(msg);
  printf("done\n");

  last_swing = msg;
  got_swing = true;
}


void ros_sleep(int seconds)
{
  for (int j = 0; j < seconds; j++) {
    ros::spinOnce();
    sleep(1);
  }
}


void wait_for_swing()
{
  printf("Waiting for swing");
  for (int i = 0; i < 10; i++) {
    ros_sleep(1);
    printf("."); fflush(0);
    if (got_swing)
      return;
  }

  printf("Error: wait_for_swing() timed out after 10 seconds\n");
  exit(1);
}


/*
 * Average all the torques in time-blocks of size 'dt'.
 */
fastwam::SwingData compress_swing(const fastwam::SwingData &swing, double dt)
{
  fastwam::SwingData new_swing;
  double t = swing.t[0];
  new_swing.t.push_back(t);
  for (int j = 0; j < 7; j++)
    new_swing.joint_angles.push_back(swing.joint_angles[j]);

  double u[7] = {0,0,0,0,0,0,0};
  int n = 0;
  for (uint i = 0; i < swing.t.size(); i++) {
    if (swing.t[i] - t >= dt) {
      for (int j = 0; j < 7; j++)
	new_swing.joint_torques.push_back(u[j]/(double)n);
      memset(u, 0, 7*sizeof(double));
      n = 0;
      t = swing.t[i];
      new_swing.t.push_back(t);
      for (int j = 0; j < 7; j++)
	new_swing.joint_angles.push_back(swing.joint_angles[7*i+j]);      
    }
    for (int j = 0; j < 7; j++)
      u[j] += swing.joint_torques[7*i+j];
    n++;
  }
  for (int j = 0; j < 7; j++)
    new_swing.joint_torques.push_back(u[j]/(double)n);

  new_swing.t.push_back(swing.t.back());  // add an extra t to make sure the last command gets executed

  //printf("compressed swing: %d joint angles, %d torques, %d times\n", new_swing.joint_angles.size(), new_swing.joint_torques.size(), new_swing.t.size());

  return new_swing;
}



fastwam::SwingData optimize_linear_swing(double *JA, double *line_velocity, double *paddle_normal, fastwam::SwingData &swing)
{
  // create swing tester
  double *init_paddle_pose = joints_to_paddle(JA);
  double *line_start = init_paddle_pose;
  LinearGradientSwingTester T(line_start, line_velocity, paddle_normal);
  free(init_paddle_pose);

  // compress the swing data
  fastwam::SwingData new_swing = compress_swing(swing, .02);
  GradientSwingOptimizer S(&T);
  S.setMaxIterations(20);

  // swing the arm
  moveto(JA);
  ros_sleep(3);
  //torque_swing(new_swing, 0, 1);
  torque_swing_with_paddle_orientation(new_swing, paddle_normal);
  wait_for_swing();

  printf("Testing the real swing:\n");
  fastwam::SwingData best_swing = compress_swing(last_swing, .02);
  double best_score = T.test(best_swing);

  printf("***** NEW BEST SWING *****\n");
  printf("------------------------------------------------------------------------------------------------------\n\n");

  for (int i = 0; i < 30; i++) {
    new_swing = S.optimizeSwing(best_swing);  //best_swing

    printf("Testing the simulated swing:\n");
    T.test(new_swing);

    // swing the arm
    moveto(JA);
    ros_sleep(3);
    //torque_swing(new_swing, 0, 1);
    torque_swing_with_paddle_orientation(new_swing, paddle_normal);
    wait_for_swing();

    printf("Testing the real swing:\n");
    fastwam::SwingData real_swing = compress_swing(last_swing, .02);
    double new_score = T.test(real_swing);

    if (new_score < best_score) {
      printf("***** NEW BEST SWING *****\n");
      best_swing = real_swing; //new_swing;
      best_score = new_score;
    }

    printf("------------------------------------------------------------------------------------------------------\n\n");
  }

  // test the best swing several times
  double avg_score = 0;
  for (int i = 0; i < 3; i++) {
    // swing the arm
    moveto(JA);
    ros_sleep(3);
    //torque_swing(new_swing, 0, 1);
    torque_swing_with_paddle_orientation(best_swing, paddle_normal);
    wait_for_swing();

    printf("Testing the real swing:\n");
    new_swing = compress_swing(last_swing, .02);
    avg_score += T.test(new_swing);
  }
  printf("average score = %.2f\n", avg_score / 3.0);
  printf("------------------------------------------------------------------------------------------------------\n\n");

  return best_swing;
}




int main(int argc, char *argv[])
{
  if (argc < 15) {
    printf("usage: %s <init_joint_angles(7)> <velocity(3)> <paddle_normal(3)> <swing_path_name>\n", argv[0]);
    exit(1);
  }

  // read inputs
  double JA[7];
  for (int i = 0; i < 7; i++)
    JA[i] = atof(argv[i+1]);
  double line_velocity[3];
  for (int i = 0; i < 3; i++)
    line_velocity[i] = atof(argv[i+8]);
  double paddle_normal[3];
  for (int i = 0; i < 3; i++)
    paddle_normal[i] = atof(argv[i+11]);
  normalize(paddle_normal, paddle_normal, 3);
  char *swing_path_name = argv[14];
  
  paddle_normal_ = paddle_normal;

  // init ROS
  ros::init(argc, argv, "train_swing");
  ros::NodeHandle nh;
  wam_moveto_pub = nh.advertise<fastwam::MoveTo>("/fastwam/moveto", 1);
  wam_joint_traj_pub = nh.advertise<fastwam::JointTrajectory>("/fastwam/joint_trajectory", 1);
  wam_torque_traj_pub = nh.advertise<fastwam::TorqueTrajectory>("/fastwam/torque_trajectory", 1);
  paddle_orient_traj_pub = nh.advertise<fastwam::PaddleOrientationTrajectory>("/fastwam/paddle_orientation_trajectory", 1);
  ros::Subscriber sub_swingdata = nh.subscribe("/fastwam/swingdata", 1, swingdata_callback);
  arm = new ArmDynamics(4); //7
  //arm->load(argv[1]);

  sleep(3);

  // start with a kinematic swing
  moveto(JA);
  ros_sleep(3);
  double dp[6] = {line_velocity[0], line_velocity[1], line_velocity[2], 0, 0, 0};
  jacobian_swing(JA, dp);
  wait_for_swing();

  // repeat the swing with smoothing and noise
  double noise = 0.2;
  int smoothing = 10;
  moveto(JA);
  ros_sleep(3);
  torque_swing(last_swing, noise, smoothing);
  wait_for_swing();

  // optimize the dynamic swing
  fastwam::SwingData swing = optimize_linear_swing(JA, line_velocity, paddle_normal, last_swing);

  // save the swing data
  printf("Saving swing data..."); fflush(0);
  arm->save(swing_path_name);
  save_swing(swing_path_name, swing); //last_swing);
  printf("done\n");

  return 0;
}

