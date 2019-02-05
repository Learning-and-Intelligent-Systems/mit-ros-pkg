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
bool record_swing = true;
bool got_swing = false;

#define NJA 19
double xJA = -.3;
double yJA[NJA] = {.2, .25, .3, .35, .4, .45, .5, .55, .6, .65, .7, .75, .8, .85, .9, .95, 1, 1.05, 1.1};
double JA[NJA][7] = {
{0.7521, 0.8526, 0.6985, 2.8328, -2.2999, 0.6714, 0.5013, },
{0.7578, 0.6584, 0.6191, 2.7814, -2.2505, 0.4391, 0.3999, },
{0.7325, 0.4855, 0.5736, 2.7373, -2.2392, 0.2615, 0.3315, },
{0.6834, 0.3272, 0.5472, 2.6841, -2.2473, 0.0958, 0.2859, },
{0.5536, 0.1921, 0.6089, 2.6234, -2.1646, -0.0500, 0.1660, },
{0.5534, 0.0608, 0.5026, 2.5449, -2.1720, -0.2375, 0.1334, },
{0.7157, -0.0397, 0.2562, 2.4695, -2.1155, -0.3845, 0.0671, },
{0.3613, -0.1266, 0.5109, 2.3594, -2.1922, -0.5682, 0.1446, },
{0.3074, -0.1896, 0.4806, 2.2573, -2.1744, -0.7145, 0.1527, },
{0.3032, -0.2305, 0.4224, 2.1565, -2.1356, -0.8253, 0.1653, },
{0.3077, -0.2558, 0.3703, 2.0547, -2.0850, -0.9102, 0.1861, },
{0.3195, -0.2688, 0.3245, 1.9523, -2.0236, -0.9698, 0.2130, },
{0.3328, -0.2720, 0.2879, 1.8478, -1.9559, -1.0071, 0.2459, },
{0.3491, -0.2666, 0.2593, 1.7398, -1.8856, -1.0218, 0.2840, },
{0.3552, -0.2543, 0.2452, 1.6282, -1.8090, -1.0193, 0.3215, },
{0.3600, -0.2338, 0.2417, 1.5074, -1.7393, -0.9967, 0.3653, },
{0.3629, -0.2059, 0.2504, 1.3806, -1.6668, -0.9494, 0.4026, },
{0.3309, -0.1696, 0.2956, 1.2385, -1.5981, -0.8876, 0.4408, },
{0.3436, -0.1189, 0.3407, 1.0801, -1.5537, -0.7756, 0.4666, }};



//-----------------------------  SWING OPTIMIZATION  --------------------------//


/*
 * Fills in swing.joint_angles and gradients
 */
void simulate_trajectory(ArmDynamics &arm, fastwam::SwingData &swing, double ***gradients)
{
  int num_joints = arm.getNumJoints();
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
    for (int j = 0; j < d; j++)
      u[j] = swing.joint_torques[(i-1)*d+j];

    if (gradients)
      arm.dynamics(ddq, gradients[i-1], q, dq, u);
    else
      arm.dynamics(ddq, NULL, q, dq, u);

    double dt = swing.t[i] - swing.t[i-1];
    for (int j = 0; j < d; j++) {
      q[j] += dt*dq[j];
      dq[j] += dt*ddq[j];
    }

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
  virtual VectorXf gradient(fastwam::SwingData &swing) = 0;
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

    for (int i = 0; i < s1.joint_torques.size(); i++)
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
    speed_weight = .01;
    normal_weight = .01;
    test_delay = .1;
  }

  double test(fastwam::SwingData &swing)
  {
    //printf("test()\n"); //dbug

    if (!swing_equals(swing, last_swing)) {
      simulate_trajectory(*arm, swing, NULL);
      last_swing = swing;
    }
    
    int n = swing.t.size() - 2;
    int m = arm->getNumJoints();
    
    double g[n];
    
    //dbug
    //double swing_speed_err[n]
    double line_cost = 0, speed_cost = 0, normal_cost = 0;

    for (int i = 0; i < n; i++) {

      if (swing.t[i] - swing.t[0] < test_delay) {
	g[i] = 0;
	continue;
      }

      double q[m], q2[m];
      for (int j = 0; j < m; j++) {
	q[j] = swing.joint_angles[i*m+j];
	q2[j] = swing.joint_angles[(i+1)*m+j];
      }
      double dt = swing.t[i+1] - swing.t[i];

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

      double normal_err = dist2(N, n_des, 3);

      //double u_err = U(i,:)*U(i,:)';

      // cost at time i
      g[i] = line_weight*line_err + speed_weight*speed_err + normal_weight*normal_err; // + u_weight*u_err;

      //dbug
      line_cost += line_weight*line_err;
      speed_cost += speed_weight*speed_err;
      normal_cost += normal_weight*normal_err;

      free(paddle_pose);
      free(paddle_pose2);
    }

    double cost = sum(g,n);

    printf(" --> total cost = %.2f (%.2f, %2f, %2f)\n\n", cost, line_cost, speed_cost, normal_cost);  //dbug

    return cost;
  }

  VectorXf gradient(fastwam::SwingData &swing)
  {
    //printf("gradient()\n");

    int NT = swing.t.size() - 1;
    int num_joints = arm->getNumJoints();

    // state gradients (dfdx, dfdu)
    double ***gradients = new_matrix3(NT, num_joints, 2*num_joints + 2);

    simulate_trajectory(*arm, swing, gradients);
    last_swing = swing;

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
      for (int j = 0; j < UD; j++) {
	memcpy(dfdx[1+UD+j], gradients[n][j], XD*sizeof(double));
	dfdu[1+UD+j][j] = gradients[n][j][XD];
      }

      // compute dgdx & dgdu at time n
      double dgdx[XD], dgdu[UD];
      memset(dgdx, 0, XD*sizeof(double));
      memset(dgdu, 0, XD*sizeof(double));
      
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

  return swing_tester_->test(swing_);
}

VectorXf GradientSwingOptimizer::gradient(VectorXf x)
{
  int n = swing_.joint_torques.size();
  for (int i = 0; i < n; i++)
    swing_.joint_torques[i] = x(i);

  return swing_tester_->gradient(swing_);
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


void jacobian_swing(double *joint_angles_start)
{
  double t0 = ros::Time::now().toSec() + 0.01;
  double dt = .02;

  fastwam::JointTrajectory msg;

  //double dp[6] = {.01, 0, .005, 0, 0, 0};
  double dp[6] = {.02, 0, .007, 0, 0, 0};
  double w[7] = {1,1,.1,1,.3,1,1};
  double ja[7];
  memcpy(ja, joint_angles_start, 7*sizeof(double));
  for (int j = 0; j < 30; j++) {
    double *dj = paddle_to_joints_displacement_weighted(ja, dp, w);
    for (int k = 0; k < 7; k++)
      ja[k] += dj[k];
    free(dj);

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


fastwam::SwingData load_swing(const char *base_name)
{
  int n,m;
  char fname[1024];
  sprintf(fname, "%s_times.txt", base_name);
  double **times = load_matrix(fname, &n, &m);
  sprintf(fname, "%s_angles.txt", base_name);
  double **angles = load_matrix(fname, &n, &m);
  sprintf(fname, "%s_torques.txt", base_name);
  double **torques = load_matrix(fname, &n, &m);

  fastwam::SwingData swing;

  for (int i = 0; i < n; i++) {
    swing.t.push_back(times[i][0]);
    for (int j = 0; j < 7; j++) {
      swing.joint_angles.push_back(angles[i][j]);
      swing.joint_torques.push_back(torques[i][j]);
    }
  }

  //cleanup
  free_matrix2(times);
  free_matrix2(angles);
  free_matrix2(torques);

  return swing;
}


void swingdata_callback(const fastwam::SwingData &msg)
{
  printf("Received swing msg with n=%d (%.2f secs)\n", msg.t.size(), msg.t.back() - msg.t.front());  //dbug

  printf("Adding swing data..."); fflush(0);
  arm->addSwingData(msg);
  printf("done\n");

  if (record_swing) {
    printf("Saving last swing..."); fflush(0);
    save_swing(last_swing_name, msg);
    printf("done\n");
  }

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



void optimize_linear_swing(double *ja, fastwam::SwingData &swing)
{
  double line_velocity[6] = {.02, 0, .007};
  double *init_paddle_pose = joints_to_paddle(ja);
  double *line_start = init_paddle_pose;
  //double *paddle_normal = &init_paddle_pose[3];
  double paddle_normal[3] = {1,0,0};

  LinearGradientSwingTester T(line_start, line_velocity, paddle_normal);

  free(init_paddle_pose);

  // compress the swing data
  fastwam::SwingData new_swing = compress_swing(swing, .02);
  GradientSwingOptimizer S(&T);
  S.setMaxIterations(20);

  for (int i = 0; i < 20; i++) {
    new_swing = S.optimizeSwing(new_swing);

    printf("Testing the simulated swing:\n");
    T.test(new_swing);

    // swing the arm
    moveto(JA[10]);
    ros_sleep(3);
    //torque_swing(new_swing, 0, 1);
    torque_swing_with_paddle_orientation(new_swing, paddle_normal);
    wait_for_swing();

    printf("Testing the real swing:\n");
    new_swing = compress_swing(last_swing, .02);
    T.test(new_swing);
    printf("------------------------------------------------------------------------------------------------------\n\n");
  }
}




int main(int argc, char *argv[])
{
  if (argc < 2) {
    printf("usage: %s <data> [last_swing] [test_swing]\n", argv[0]);
    exit(1);
  }

  if (argc >= 3)
    last_swing_name = argv[2];

  // init ROS
  ros::init(argc, argv, "test_ik");
  ros::NodeHandle nh;
  wam_moveto_pub = nh.advertise<fastwam::MoveTo>("/fastwam/moveto", 1);
  wam_joint_traj_pub = nh.advertise<fastwam::JointTrajectory>("/fastwam/joint_trajectory", 1);
  wam_torque_traj_pub = nh.advertise<fastwam::TorqueTrajectory>("/fastwam/torque_trajectory", 1);
  paddle_orient_traj_pub = nh.advertise<fastwam::PaddleOrientationTrajectory>("/fastwam/paddle_orientation_trajectory", 1);
  ros::Subscriber sub_swingdata = nh.subscribe("/fastwam/swingdata", 1, swingdata_callback);
  arm = new ArmDynamics(7);
  arm->load(argv[1]);

  sleep(3);

  if (last_swing_name != NULL)
    last_swing = load_swing(last_swing_name);
    
  if (last_swing.t.empty() || arm->isEmpty()) {

    moveto(JA[10]);
    ros_sleep(3);
    jacobian_swing(JA[10]);
    wait_for_swing();

    double noise = 0.2;
    int smoothing = 10;

    for (int i = 0; i < 1; i++) {
      moveto(JA[10]);
      ros_sleep(3);
      torque_swing(last_swing, noise, smoothing);
      wait_for_swing();
    }
  }

  // test_last_swing
  if (argc >= 4) {
    //record_swing = false;
    last_swing_name = argv[3];
    double line_velocity[6] = {.02, 0, .007};
    double *init_paddle_pose = joints_to_paddle(JA[10]);
    double *line_start = init_paddle_pose;
    //double *paddle_normal = &init_paddle_pose[3];
    double paddle_normal[3] = {1,0,0};
    moveto(JA[10]);
    ros_sleep(3);
    fastwam::SwingData new_swing = compress_swing(last_swing, .02);
    torque_swing_with_paddle_orientation(new_swing, paddle_normal);
    //torque_swing(last_swing, 0, 1);
    wait_for_swing();
    LinearGradientSwingTester T(line_start, line_velocity, paddle_normal);
    free(init_paddle_pose);
    new_swing = compress_swing(last_swing, .02);
    T.test(new_swing);
    //record_swing = true;
    last_swing_name = argv[2];
    return 0; //dbug
  }

  optimize_linear_swing(JA[10], last_swing);

  //test_swing_optimization_real(JA[10], last_swing);
  //test_swing_optimization_sim(JA[10], last_swing);

  printf("Saving swing data..."); fflush(0);
  arm->save(argv[1]);
  printf("done\n");

  return 0;
}






//------------------------------------------------------------------------//
//---------------------------------  OLD  --------------------------------//
//------------------------------------------------------------------------//



/**************  RealPaddleTrajectorySwingTester  **************

class RealPaddleTrajectorySwingTester : public SwingTester {
protected:
  double **traj_;
  double *times_;
  int traj_len_;
public:
  RealPaddleTrajectorySwingTester(double **traj, double *times, int traj_len)
  {
    traj_ = traj;
    times_ = times;
    traj_len_ = traj_len;
  }

  double test(fastwam::SwingData &swing)
  {
    //dbug
    //for (int i = 0; i < swing.t.size(); i++) {
    //  printf("swing.joint_torques[t=%d] = ", i);
    //  for (int j = 0; j < 7; j++)
    //	printf("%.2f ", swing.joint_torques[7*i+j]);
    //  printf("\n");
    //}
    //double t0 = swing.t[0];

    double t0 = ros::Time::now().toSec();

    // swing the arm
    moveto(JA[10]);
    ros_sleep(3);
    torque_swing(swing, 0, 1);
    wait_for_swing();

    // get paddle poses for the last swing
    int n = last_swing.t.size();
    double **P = new_matrix2(n,6);
    for (int i = 0; i < n; i++) {
      double ja[7];
      for (int j = 0; j < 7; j++)
	ja[j] = last_swing.joint_angles[i*7+j];
      double *paddle_pose = joints_to_paddle(ja);
      memcpy(P[i], paddle_pose, 6*sizeof(double));
      free(paddle_pose);
    }
    
    // evaluate paddle poses
    printf("costs = ");  //dbug
    double cost = 0.0;
    double normal_weight = 1;
    int t = 0;
    for (int i = 0; i < traj_len_; i++) {
      for (; t < n; t++)  // find first swing time >= paddle time
	if (last_swing.t[t] - t0 >= times_[i])
	  break;
      if (t==n)
	break;
      double c = dist2(P[t], traj_[i], 3) + normal_weight * dist2(&P[t][3], &traj_[i][3], 3);
      printf("%f ", c);  //dbug
      cost += c;
    }
    printf("\n --> total cost = %f\n\n", cost);  //dbug

    //cleanup
    free_matrix2(P);

    return cost;
  }
};
*/


/**************  SimPaddleTrajectorySwingTester  **************

class SimPaddleTrajectorySwingTester : public GradientSwingTester {
protected:
  double *init_joint_angles_;
  double **traj_;
  double **traj_vel_;
  double *times_;
  int traj_len_;
public:
  SimPaddleTrajectorySwingTester(double *init_joint_angles, double **traj, double **traj_vel, double *times, int traj_len)
  {
    init_joint_angles_ = init_joint_angles;
    traj_ = traj;
    traj_vel_ = traj_vel;
    times_ = times;
    traj_len_ = traj_len;
  }

  double testSwing(fastwam::SwingData &swing, bool verbose, double *gradients)
  {
    // predict paddle poses and velocities
    int n = swing.t.size();
    double **P = new_matrix2(n,6);
    for (int i = 0; i < n; i++) {
      double q[7];
      for (int j = 0; j < 7; j++)
	q[j] = swing.joint_angles[i*7+j];
      double *paddle_pose = joints_to_paddle(q);
      memcpy(P[i], paddle_pose, 6*sizeof(double));
      free(paddle_pose);
    }
    double **V = new_matrix2(n,6);
    for (int i = 0; i < n-1; i++) {
      sub(V[i], P[i+1], P[i], 6);
      mult(V[i], V[i], 1.0 / (double)(swing.t[i+1] - swing.t[i]), 6);
    }

    // evaluate paddle poses
    int t = 0;
    double cost = 0.0;
    double normal_weight = 0.1;
    double vel_weight = 0.1;
    double pos_err[traj_len_], norm_err[traj_len_], vel_err[traj_len_];
    for (int i = 0; i < traj_len_; i++) {
      for (; t < n; t++)  // find first swing time >= paddle time
	if (swing.t[t] - swing.t[0] >= times_[i])
	  break;
      if (t==n)
	break;

      //pos_err[i] = dist(P[t], traj_[i], 3);
      
      // line error
      double v[3];
      sub(v, P[t], traj_[i], 3);
      double c2 = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
      double a = dot(v, traj_vel_[i], 3) / norm(traj_vel_[i], 3);
      double b2 = c2 - a*a;
      pos_err[i] = sqrt(b2);

      norm_err[i] = dist(&P[t][3], &traj_[i][3], 3);
      //vel_err[i] = dist(V[t], traj_vel_[i], 3);

      double desired_speed = norm(traj_vel_[i], 3);
      double actual_speed = norm(V[t], 3);
      vel_err[i] = actual_speed - desired_speed;

      cost += pos_err[i]*pos_err[i] + normal_weight*norm_err[i]*norm_err[i] + vel_weight*vel_err[i]*vel_err[i];
    }

    //dbug
    if (verbose) {
      printf("pos_err = [ ");  for (int i = 0; i < traj_len_; i++)  printf("%.2f ", pos_err[i]);  printf("]\n");
      printf("norm_err = [ ");  for (int i = 0; i < traj_len_; i++)  printf("%.2f ", norm_err[i]);  printf("]\n");
      printf("vel_err = [ ");  for (int i = 0; i < traj_len_; i++)  printf("%.2f ", vel_err[i]);  printf("]\n");
      printf(" --> total cost = %f\n\n", cost);
    }
    
    //cleanup
    free_matrix2(P);
    free_matrix2(V);

    return cost;
  }

  double test(fastwam::SwingData &swing)
  {
    simulate_trajectory(*arm, swing, NULL);
    
    return testSwing(swing, false, NULL);
  }
};
*/





/*
class SwingOptimizer : public GradientFreeRandomizedGradientOptimizer {
 private:
  SwingTester *swing_tester_;
  fastwam::SwingData swing_;
 public:
  SwingOptimizer(SwingTester *T);
  float evaluate(VectorXf x);
  fastwam::SwingData optimizeSwing(const fastwam::SwingData &initial_swing);
};

SwingOptimizer::SwingOptimizer(SwingTester *T) :
  GradientFreeRandomizedGradientOptimizer(.01, VectorXf::Zero(1), 1.0),   // TODO: clean this up
  swing_tester_(T)
{
}

float SwingOptimizer::evaluate(VectorXf x)
{
  int n = swing_.joint_torques.size();
  for (int i = 0; i < n; i++)
    swing_.joint_torques[i] = x(i);

  return swing_tester_->test(swing_);
}

fastwam::SwingData SwingOptimizer::optimizeSwing(const fastwam::SwingData &initial_swing)
{
  //double joint_noise[7] = {5,5,5,5,.2,.2,.2};
  double joint_noise[7] = {.2,.2,.1,.1,.02,.02,.005};

  swing_ = initial_swing;

  int n = initial_swing.joint_torques.size();
  noise_ = VectorXf::Zero(n);
  for (int i = 0; i < n/7; i++)
    for (int j = 0; j < 7; j++)
      noise_(7*i+j) = joint_noise[j];

  VectorXf x = optimize(swing_to_vector(initial_swing));

  for (int i = 0; i < n; i++)
    swing_.joint_torques[i] = x(i);

  return swing_;
}
*/

/*
void test_swing_optimization_real(double *ja, fastwam::SwingData &swing)
{
  double t0 = 0.5;
  double dt = .02;
  int n = 20;
  double dp[6] = {.01, 0, .005, 0, 0, 0};

  double **paddle_poses = new_matrix2(n,6);
  double paddle_times[n];
  
  // get first paddle state
  double *init_paddle_pose = joints_to_paddle(ja);
  for (int i = 0; i < 6; i++)
    paddle_poses[0][i] = init_paddle_pose[i] + 10*dp[i];
  paddle_times[0] = t0;

  // get the rest of the linear paddle trajectory
  for (int i = 1; i < n; i++) {
    add(paddle_poses[i], paddle_poses[i-1], dp, 6);
    paddle_times[i] = paddle_times[i-1] + dt;
  }

  // compress the swing data
  fastwam::SwingData new_swing = compress_swing(swing, .02);

  RealPaddleTrajectorySwingTester T(paddle_poses, paddle_times, n);
  SwingOptimizer S(&T);
  S.optimizeSwing(new_swing);

  //cleanup
  free_matrix2(paddle_poses);
  free(init_paddle_pose);
}
*/

/*
void test_swing_optimization_sim(double *ja, fastwam::SwingData &swing)
{
  double t0 = 0.5;
  double dt = .02;
  int n = 20;
  //double dp[6] = {.01, 0, .005, 0, 0, 0};
  double dp[6] = {.02, 0, .007, 0, 0, 0};

  double **paddle_poses = new_matrix2(n,6);
  double **paddle_vel = new_matrix2(n,6);
  double paddle_times[n];
  
  // get first paddle state
  double *init_paddle_pose = joints_to_paddle(ja);
  for (int i = 0; i < 6; i++)
    paddle_poses[0][i] = init_paddle_pose[i] + 10*dp[i];
  paddle_times[0] = t0;

  // get the rest of the linear paddle trajectory
  for (int i = 1; i < n; i++) {
    add(paddle_poses[i], paddle_poses[i-1], dp, 6);
    paddle_times[i] = paddle_times[i-1] + dt;
  }

  // get the paddle velocities
  for (int i = 0; i < n; i++)
    mult(paddle_vel[i], dp, 1.0/dt, 6);

  // compress the swing data
  fastwam::SwingData new_swing = compress_swing(swing, .02);

  SimPaddleTrajectorySwingTester T(ja, paddle_poses, paddle_vel, paddle_times, n);
  SwingOptimizer S(&T);
  S.setMaxIterations(50);

  for (int i = 0; i < 20; i++) {
    new_swing = S.optimizeSwing(new_swing);

    printf("Testing the simulated swing:\n");
    T.testSwing(new_swing, true, NULL);

    // swing the arm
    moveto(JA[10]);
    ros_sleep(3);
    torque_swing(new_swing, 0, 1);
    wait_for_swing();

    printf("Testing the real swing:\n");
    new_swing = compress_swing(last_swing, .02);
    T.testSwing(new_swing, true, NULL);
    printf("------------------------------------------------------------------------------------------------------\n\n");
  }

  //cleanup
  free_matrix2(paddle_poses);
  free_matrix2(paddle_vel);
  free(init_paddle_pose);
}
*/




