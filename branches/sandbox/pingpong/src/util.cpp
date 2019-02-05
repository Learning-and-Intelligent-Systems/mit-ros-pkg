#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <bingham.h>
#include <bingham/util.h>
#include "util.h"

// SSE instructions
#include <xmmintrin.h>
#include <emmintrin.h>
#include <mmintrin.h>


namespace enc = sensor_msgs::image_encodings;

using namespace std;
using namespace cv;



//--------------------- Ball Prediction Routines ----------------------//

//static const double air_density = 1.204;    // at 20 degrees celcius
//static const double drag_coefficient = .1;  // for a smooth sphere
//static const double spin_down_beta = .01;
//static const double table_height = .76;
//static const double net_height = .1525;
//static const double paddle_length = .165;
//static const double paddle_width = .02;
//static const double ball_mass = .0027;
//static const double ball_coeff_rest_rubber = .5;  // coefficient of restitution
//static const double ball_table_friction_coeff = .2;  //.5
//static const double ball_paddle_friction_coeff = .9;
//static const double ball_bounce_duration = .00071;


inline bool ball_in_table_region(const ball_t &b)
{
  static const double table_length = 2.74;
  static const double table_width = 1.525;

  return (b.x > 0 && b.x < table_width && b.y > 0 && b.y < table_length);
}


ball_t ball_dynamics(const ball_t &b, double dt)
{
  // all units are SI
  const double r = BALL_RADIUS;
  const double g = 9.8;
  const double ball_coeff_rest = .88;  // coefficient of restitution
  const double p = 1.204;              // air density at 20 degrees celcius
  const double C = .4; //.1;                 // drag coeff. for a smooth sphere
  const double m = .0027;              // ball mass

  // drag force
  double v2 = b.vx*b.vx + b.vy*b.vy + b.vz*b.vz;
  double v = sqrt(v2);
  double A = M_PI*r*r;
  double Fd = .5*C*p*A*v;
  double Fdx = -Fd*b.vx;
  double Fdy = -Fd*b.vy;
  double Fdz = -Fd*b.vz;

  ball_t b2;

  b2.t = b.t + dt;
  b2.x = b.x + dt*b.vx;
  b2.y = b.y + dt*b.vy;
  b2.z = b.z + dt*b.vz;

  // check for ball-table collision
  if (b2.z < r && ball_in_table_region(b2) && fabs(b.z - b2.z) > 1e-16) {
    // first calculate (z,vz) at collision time (dt_bounce)
    double dt_bounce = dt * (b.z - r) / (b.z - b2.z);
    double z = r;  //b.z + dt_bounce*b.vz;
    double vz = -ball_coeff_rest*(b.vz - dt_bounce*g);
    // then calculate (z,vz) at dt
    b2.z = z + (dt-dt_bounce)*vz;
    b2.vz = vz - (dt-dt_bounce)*g;

    // ignore drag during bounce; just model friction (TODO: use spin!)
    b2.vx = .8 * b.vx;
    b2.vy = .8 * b.vy;

  }
  else {
    b2.vx = b.vx + dt*Fdx/m;
    b2.vy = b.vy + dt*Fdy/m;
    b2.vz = b.vz + dt*(Fdz/m - g);
  }

  return b2;
}


ball_t ball_dynamics_with_spin(const ball_t &b, double dt)
{
  // all units are SI
  const double r = BALL_RADIUS;
  const double g = 9.8;
  const double p = 1.204;              // air density at 20 degrees celcius
  const double C = .4; //.1;                 // drag coeff. for a smooth sphere
  const double m = .0027;              // ball mass
  double A = M_PI*r*r;
  double Cd = .5*C*p*A/m;              // coefficient of drag force
  const double Cm = .0004;             // coefficient of magnus force
  const double Cf = .3;                // coefficient of friction
  const double Cr = .88;               // coefficient of restitution

  double v[3] = {b.vx, b.vy, b.vz};  // current velocity
  double w[3] = {b.wx, b.wy, b.wz};  // current spin

  ball_t b2;

  b2.t = b.t + dt;
  b2.x = b.x + dt*b.vx;
  b2.y = b.y + dt*b.vy;
  b2.z = b.z + dt*b.vz;

  // check for ball-table collision
  if (b2.z < r && ball_in_table_region(b2) && fabs(b.z - b2.z) > 1e-16) {
    // first calculate (z,vz) at collision time (dt_bounce)
    double dt_bounce = dt * (b.z - r) / (b.z - b2.z);
    double z = r;  //b.z + dt_bounce*b.vz;
    double vz0 = b.vz - dt_bounce*g;
    v[2] = vz0;

    // compute relative velocity direction of ball surface w.r.t table
    double vr[3] = {v[0] - w[1]*r, v[1] + w[0]*r, 0.0};

    bool rolling = (norm(vr,3) < -2.5*Cf*v[2]*(1+Cr));

    // compute post-bounce velocity and spin
    if (rolling) {
      b2.vx = .6*v[0] + .4*r*w[1];
      b2.vy = .6*v[1] - .4*r*w[0];
      b2.vz = -Cr*v[2];
      b2.wx = .4*w[0] - .6*v[1]/r;
      b2.wy = .4*w[1] + .6*v[0]/r;
      b2.wz = w[2];
    }
    else {
      normalize(vr,vr,3);
      b2.vx = v[0] + Cf*vr[0]*v[2]*(1+Cr);
      b2.vy = v[1] + Cf*vr[1]*v[2]*(1+Cr);
      b2.vz = v[2] + (Cf*vr[2]-1)*v[2]*(1+Cr);
      b2.wx = w[0] + 1.5*(Cf/r)*vr[1]*v[2]*(1+Cr);
      b2.wy = w[1] - 1.5*(Cf/r)*vr[0]*v[2]*(1+Cr);
      b2.wz = w[2];
    }
    b2.z = r + (dt-dt_bounce)*b2.vz;

    //double vz = -ball_coeff_rest*vz0;
    // then calculate (z,vz) at dt
    //b2.z = z + (dt-dt_bounce)*vz;
    //b2.vz = vz - (dt-dt_bounce)*g;

    // ignore drag during bounce; just model friction (TODO: use spin!)
    //b2.vx = .8 * b.vx;
    //b2.vy = .8 * b.vy;

  }
  else {
    // acceleration from drag force and gravity
    double s = norm(v,3);
    double dvdt[3] = {-Cd*s*v[0], -Cd*s*v[1], -Cd*s*v[2] - g};

    // acceleration from magnus force
    double wxv[3], dvdt_magnus[3];
    cross(wxv, w, v);
    mult(dvdt_magnus, wxv, Cm*s, 3);
    add(dvdt, dvdt, dvdt_magnus, 3);

    b2.vx = b.vx + dt*dvdt[0];
    b2.vy = b.vy + dt*dvdt[1];
    b2.vz = b.vz + dt*dvdt[2];
    b2.wx = b.wx;
    b2.wy = b.wy;
    b2.wz = b.wz;
  }

  return b2;
}


std::vector<ball_t> predict_ball_trajectory(const ball_t &b0, int n, double dt, bool use_qbf)
{
  //printf("predict_ball_trajectory(): b0 = (%f, %f, %f)\n", b0.x, b0.y, b0.z); //dbug

  ball_t b = b0;

  if (use_qbf) {
    double s[4], a[3];;
    bingham_mode(s, &b.dB);
    quaternion_to_axis_angle(a,s);

    double qbf_wx = a[0]/.005;
    double qbf_wy = a[1]/.005;
    //double wz = a[2]/.005;

    b.wx = .5*b.wx + .5*qbf_wx;
    b.wy = .1*b.wy + .9*qbf_wy;
  }

  vector<ball_t> traj(n);
  traj[0] = b;
  for (int i = 1; i < n; i++)
    traj[i] = ball_dynamics_with_spin(traj[i-1], dt);

  return traj;
}


void filter_ball_init(ball_t &ball)
{
  // position, velocity, and spin (Magnus)
  //const double ball_filter_init_variance[9] = {.006, .006, .02, 100, 100, 100, 1e6, 1e6, 1e6};
  const double ball_filter_init_variance[9] = {.006, .006, .02, 10, 10, 10, 1e6, 1e6, 1e6};

  ball.ox = ball.x;
  ball.oy = ball.y;
  ball.oz = ball.z;

  //velocity prior
  ball.vx = 0;
  ball.vy = (ball.y > 1.5 ? -4.5 : 4.5);
  ball.vz = 0;

  memset(ball.cov, 0, 36*sizeof(double));
  for (int i = 0; i < 9; i++)
    ball.cov[10*i] = ball_filter_init_variance[i];

  // QBF orientation
  const int d = 4;
  if (norm(ball.oq,d) > 0.01) {  // observed ball logo
    double Z[3] = {-10, -10, -10};
    double V_data[3][4] = {{0,0,1,0}, {0,0,0,1}, {0,1,0,0}};
    double *V[3] = {&V_data[0][0], &V_data[1][0], &V_data[2][0]};
    bingham_new(&ball.B, d, V, Z);
    bingham_post_rotate_3d(&ball.B, &ball.B, ball.oq);
  }
  else {  // dark side of the moon
    double Z[3] = {-3.6, -3.6, 0};
    double V_data[3][4] = {{.5,-.5,-.5,-.5}, {.5,.5,-.5,.5}, {.5,.5,.5,-.5}};
    double *V[3] = {&V_data[0][0], &V_data[1][0], &V_data[2][0]};
    bingham_new(&ball.B, d, V, Z);
  }

  // QBF spin
  double Z[3] = {-3, -10, -3}; //{-3, -3, -3};
  double V_data[3][4] = {{0,0,1,0}, {0,0,0,1}, {0,1,0,0}};
  double *V[3] = {&V_data[0][0], &V_data[1][0], &V_data[2][0]};
  bingham_new(&ball.dB, d, V, Z);

  // spin prior
  double theta = .005*ball.wx;
  if (fabs(theta) > .00001) {
    // QBF spin prior
    double q[4] = {cos(theta/2.0), sin(theta/2.0), 0, 0};
    bingham_pre_rotate_3d(&ball.dB, &ball.dB, q);
    ball.dB.Z[2] = -30;

    // EKF (Magnus) spin prior
    ball.cov[6] = 1000;
    ball.cov[7] = 1000;
  }
}


/*
 * Kalman filter to track position, velocity, and spin (via the Magnus effect).
 */
void filter_ball_position_magnus(ball_t &ball, ball_t &prev_ball)
{
  // all units are SI
  const double r = BALL_RADIUS;
  //const double g = 9.8;
  //const double ball_coeff_rest = .88;  // coefficient of restitution
  const double p = 1.204;              // air density at 20 degrees celcius
  const double C = .4; //.1;                 // drag coeff. for a smooth sphere
  const double m = .0027;              // ball mass
  double A = M_PI*r*r;
  double Cd = .5*C*p*A/m;
  double Cm = .0004;
  const double ball_filter_measurement_noise[3] = {.006, .006, .02};
  const double ball_filter_process_noise_ballistic[9] = {0, 0, 0, .1, .1, .1, 1, 1, 1};
  const double ball_filter_process_noise_bounce[9] = {0, 0, 0, 50, 50, 50, 50, 50, 50};

  double dt = ball.t - prev_ball.t;
  ball_t predicted_ball = ball_dynamics_with_spin(prev_ball, dt);
  bool bounce = prev_ball.vz < 0 && predicted_ball.vz > 0;

  if (bounce)
    printf("----------------------------------------bounce-----------------------------------------\n"); //dbug

  ball.ox = ball.x;
  ball.oy = ball.y;
  ball.oz = ball.z;

  // prediction update
  double vx = prev_ball.vx;
  double vy = prev_ball.vy;
  double vz = prev_ball.vz;
  double v[3] = {vx, vy, vz};
  double s = norm(v,3);

  double **dvsdv = new_matrix2(3,3);
  if (s > 1e-8) {
    double v_s[3] = {v[0]/s, v[1]/s, v[2]/s};
    outer_prod(dvsdv, v_s, v, 3, 3);
  }
  for (int i = 0; i < 3; i++)
    dvsdv[i][i] += s;

  double wx = prev_ball.wx;
  double wy = prev_ball.wy;
  double wz = prev_ball.wz;
  double W[9] = {0., -wz, wy,  wz, 0., -wx,  -wy, wx, 0.};
  double V[9] = {0., vz, -vy,  -vz, 0., vx,  vy, -vx, 0.};

  //dadv = (Cm*W - Cd)*dvsdv;
  double **dadv = new_matrix2(3,3);
  for (int i = 0; i < 9; i++)
    dadv[0][i] = Cm*W[i] - Cd;
  matrix_mult(dadv, dadv, dvsdv, 3, 3, 3);

  //dadw = Cm*s*V;
  double **dadw = new_matrix2(3,3);
  mult(dadw[0], V, Cm*s, 9);

  // F = [ eye(3,3) dt*eye(3) zeros(3,3) ; zeros(3,3) eye(3)+dt*dadv dt*dadw ; zeros(3,3) zeros(3,3) eye(3,3) ];
  double **F = new_identity_matrix2(9);
  for (int i = 0; i < 3; i++)
    F[i][3+i] = dt;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      F[3+i][3+j] += dt*dadv[i][j];
      F[3+i][6+j] = dt*dadw[i][j];
    }
  }

  double **Ft = new_matrix2(9,9);
  transpose(Ft,F,9,9);
  double **P = new_matrix2(9,9);    // P = F*P*Ft +Q
  memcpy(P[0], prev_ball.cov, 81*sizeof(double));
  matrix_mult(P,F,P,9,9,9);
  matrix_mult(P,P,Ft,9,9,9);
  if (bounce) {
    for (int i = 0; i < 9; i++)
      P[i][i] += dt*ball_filter_process_noise_bounce[i];
  }
  else
    for (int i = 0; i < 9; i++)
      P[i][i] += dt*ball_filter_process_noise_ballistic[i];

  // measurement update
  double y[3] = {ball.ox - predicted_ball.x, ball.oy - predicted_ball.y, ball.oz - predicted_ball.z};
  double S_data[9] = {P[0][0], P[0][1], P[0][2], P[1][0], P[1][1], P[1][2], P[2][0], P[2][1], P[2][2]};
  double **S = new_matrix2_data(3,3,S_data);  // S = H*P*H' + R and H = [I 0 0]
  for (int i = 0; i < 3; i++)
    S[i][i] += ball_filter_measurement_noise[i];
  double **S_inv = new_matrix2(3,3);
  inv(S_inv, S, 3);
  memset(F[0], 0, 81*sizeof(double));  // so that K = [K 0 0] = K*H for H = [I 0 0]
  double **K = F;  // save an alloc
  matrix_mult(K, P, S_inv, 9, 3, 3);  // K = P*H'*S_inv = P(:,1:3)*S_inv for H = [I 0 0]
  double x[9], x_pred[9] = {predicted_ball.x, predicted_ball.y, predicted_ball.z,
			    predicted_ball.vx, predicted_ball.vy, predicted_ball.vz,
			    predicted_ball.wx, predicted_ball.wy, predicted_ball.wz};
  matrix_vec_mult(x, K, y, 9, 3);
  add(x, x, x_pred, 9);
  double **KHP = Ft;  // save an alloc
  matrix_mult(KHP, K, P, 9, 9, 9);  // since K = [K 0 0] = K*H for H = [I 0 0]
  matrix_sub(P, P, KHP, 9, 9);

  ball.x = x[0];
  ball.y = x[1];
  ball.z = x[2];
  ball.vx = x[3];
  ball.vy = x[4];
  ball.vz = x[5];
  ball.wx = x[6];
  ball.wy = x[7];
  ball.wz = x[8];

  if (ball.vz < 0.0 || ball.y > 1.0)
    printf("magnus spin = (%.2f, %.2f, %.2f)\n", ball.wx, ball.wy, ball.wz); //dbug

  memcpy(ball.cov, P[0], 81*sizeof(double));

  //TODO: handle humans hitting the ball better
  if (ball.vy > 0 && ball.y > 1.4) {
    const double ball_filter_init_variance[9] = {.006, .006, .02, 100, 100, 100, 1e6, 1e6, 1e6};
    memset(ball.cov, 0, 36*sizeof(double));
    for (int i = 0; i < 9; i++)
      ball.cov[10*i] = ball_filter_init_variance[i];
  }

  //cleanup
  free_matrix2(dvsdv);
  free_matrix2(dadv);
  free_matrix2(dadw);
  free_matrix2(F);
  free_matrix2(Ft);
  free_matrix2(P);
  free_matrix2(S);
  free_matrix2(S_inv);
}


/*
 * Kalman filter to track position and velocity.
 */
void filter_ball_position(ball_t &ball, ball_t &prev_ball)
{
  // all units are SI
  const double r = BALL_RADIUS;
  //const double g = 9.8;
  //const double ball_coeff_rest = .88;  // coefficient of restitution
  const double p = 1.204;              // air density at 20 degrees celcius
  const double C = .4; //.1;                 // drag coeff. for a smooth sphere
  const double m = .0027;              // ball mass
  const double ball_filter_measurement_noise[3] = {.006, .006, .02};
  const double ball_filter_process_noise_ballistic[6] = {0, 0, 0, 1, 1, 1};
  const double ball_filter_process_noise_bounce[6] = {0, 0, 0, 50, 50, 50};

  double dt = ball.t - prev_ball.t;
  vector<ball_t> traj = predict_ball_trajectory(prev_ball, 1, dt);
  ball_t &predicted_ball = traj.back();

  //dbug
  //printf("dt = %f, predicted_ball = (%.2f, %.2f, %.2f,  %.2f, %.2f, %.2f)\n", dt,
  //	 predicted_ball.x, predicted_ball.y, predicted_ball.z, predicted_ball.vx, predicted_ball.vy, predicted_ball.vz);

  ball.ox = ball.x;
  ball.oy = ball.y;
  ball.oz = ball.z;

  // prediction update
  double vx = prev_ball.vx;
  double vy = prev_ball.vy;
  double vz = prev_ball.vz;
  double v2 = vx*vx + vy*vy + vz*vz;
  double v = sqrt(v2);
  double vx_v = (fabs(v) > 1e-8 ? vx/v : 0);
  double vy_v = (fabs(v) > 1e-8 ? vy/v : 0);
  double vz_v = (fabs(v) > 1e-8 ? vz/v : 0);
  double A = M_PI*r*r;
  double d = .5*C*p*A/m;
  double F_data[36] = {1, 0, 0,                    dt,                     0,                     0,
		       0, 1, 0,                     0,                    dt,                     0,
		       0, 0, 1,                     0,                     0,                    dt,
		       0, 0, 0,  1+dt*(d*v+d*vx*vx_v),        dt*(d*vx*vy_v),        dt*(d*vx*vz_v),
		       0, 0, 0,        dt*(d*vx*vy_v),  1+dt*(d*v+d*vy*vy_v),        dt*(d*vy*vz_v),
		       0, 0, 0,        dt*(d*vx*vz_v),        dt*(d*vy*vz_v),  1+dt*(d*v+d*vz*vz_v)};
  double **F = new_matrix2_data(6, 6, F_data);
  double **Ft = new_matrix2(6, 6);
  transpose(Ft,F,6,6);
  double **P = new_matrix2(6, 6);    // P = F*P*Ft +Q
  memcpy(P[0], prev_ball.cov, 36*sizeof(double));
  matrix_mult(P,F,P,6,6,6);
  matrix_mult(P,P,Ft,6,6,6);
  if (prev_ball.vz < 0 && predicted_ball.vz > 0)
    for (int i = 0; i < 6; i++)
      P[i][i] += dt*ball_filter_process_noise_bounce[i];
  else
    for (int i = 0; i < 6; i++)
      P[i][i] += dt*ball_filter_process_noise_ballistic[i];
  
  // measurement update
  double y[3] = {ball.ox - predicted_ball.x, ball.oy - predicted_ball.y, ball.oz - predicted_ball.z};
  double S_data[9] = {P[0][0], P[0][1], P[0][2], P[1][0], P[1][1], P[1][2], P[2][0], P[2][1], P[2][2]};
  double **S = new_matrix2_data(3,3,S_data);  // S = H*P*H' + R and H = [I 0]
  for (int i = 0; i < 3; i++)
    S[i][i] += ball_filter_measurement_noise[i];
  double **S_inv = new_matrix2(3,3);
  inv(S_inv, S, 3);
  memset(F[0], 0, 36*sizeof(double));  // so that K = [K 0] = K*H for H = [I 0]
  double **K = F;  // save an alloc
  matrix_mult(K, P, S_inv, 6, 3, 3);  // K = P*H'*S_inv = P(:,1:3)*S_inv for H = [I 0]
  double x[6], x_pred[6] = {predicted_ball.x, predicted_ball.y, predicted_ball.z, predicted_ball.vx, predicted_ball.vy, predicted_ball.vz};
  matrix_vec_mult(x, K, y, 6, 3);
  add(x, x, x_pred, 6);
  double **KHP = Ft;  // save an alloc
  matrix_mult(KHP, K, P, 6, 6, 6);  // since K = [K 0] = K*H for H = [I 0]
  matrix_sub(P, P, KHP, 6, 6);
  
  ball.x = x[0];
  ball.y = x[1];
  ball.z = x[2];
  ball.vx = x[3];
  ball.vy = x[4];
  ball.vz = x[5];

  memcpy(ball.cov, P[0], 36*sizeof(double));

  free_matrix2(P);
  free_matrix2(F);
  free_matrix2(Ft);
  free_matrix2(S);
  free_matrix2(S_inv);
}


void quaternion_to_axis_angle(double *a, double *q)
{
  double theta = 2*acos(q[0]);
  double v[3] = {1,0,0};
  if (fabs(q[1]) + fabs(q[2]) + fabs(q[3]) > 1e-16)
    mult(v, &q[1], 1.0/sin(theta/2.0), 3);

  if (theta > M_PI)
    theta = theta - 2*M_PI;

  mult(a, v, theta, 3);
}


/*
 * reverse principal curvature
 */
void quaternion_flip(double *q2, double *q)
{
  if (q == q2) {
    double q1[4];
    q1[0] = -q[1];
    q1[1] = q[0];
    q1[2] = q[3];
    q1[3] = -q[2];
    memcpy(q2, q1, 4*sizeof(double));
  }
  else {
    q2[0] = -q[1];
    q2[1] = q[0];
    q2[2] = q[3];
    q2[3] = -q[2];
  }
}

/*
 * Bingham filter to track ball orientation and spin.
 */
void filter_ball_spin(ball_t &ball, ball_t &prev_ball)
{
  int use_dark_side = 0;
  int compute_compose_errors = 0;

  //TODO: continuous-time Bingham filter
  double dt = ball.t - prev_ball.t;
  int nt = MAX(1, round(dt/.005));
  const int d = 4;

  printf("nt = %d\n", nt); //dbug

  // update B (orientation distribution)
  bingham_alloc(&ball.B, d);
  bingham_copy(&ball.B, &prev_ball.B);
  for (int i = 0; i < nt; i++)
    bingham_compose(&ball.B, &prev_ball.dB, &ball.B);  // prediction update

  bingham_t B_obs;
  if (norm(ball.oq,d) > 0.01) {  // if q exists, multiply in observation term
    double q2[4];
    quaternion_flip(q2, ball.oq);
    if (bingham_pdf(q2, &ball.B) > bingham_pdf(ball.oq, &ball.B))  // are observation axes flipped?
      memcpy(ball.oq, q2, 4*sizeof(double));

    double Z_obs[3] = {-10, -10, -10};  // should this be anisotropic?
    double V[3][4] = {{0,0,1,0}, {0,0,0,1}, {0,1,0,0}};
    double *Vp[3] = {&V[0][0], &V[1][0], &V[2][0]};
    bingham_new(&B_obs, d, Vp, Z_obs);
    bingham_post_rotate_3d(&B_obs, &B_obs, ball.oq);
    bingham_mult(&ball.B, &ball.B, &B_obs);
  }
  else if (use_dark_side) {  // dark side of the moon
    double Z[3] = {-3.6, -3.6, 0};
    double V[3][4] = {{.5,-.5,-.5,-.5}, {.5,.5,-.5,.5}, {.5,.5,.5,-.5}};
    double *Vp[3] = {&V[0][0], &V[1][0], &V[2][0]};
    bingham_new(&B_obs, d, Vp, Z);
    bingham_mult(&ball.B, &ball.B, &B_obs);
  }

  // update dB (spin distribution)
  bingham_alloc(&ball.dB, d);
  bingham_copy(&ball.dB, &prev_ball.dB);
  int have_curr_obs = (norm(ball.oq,d) > 0.01);
  int have_prev_obs = (norm(prev_ball.oq,d) > 0.01);
  if (have_curr_obs || (have_prev_obs && use_dark_side)) {  // if orientation is observed at time i or i-1
    bingham_t dB_obs;
    bingham_t B_inv;
    bingham_alloc(&B_inv, d);
    bingham_copy(&B_inv, &prev_ball.B);
    for (int j = 0; j < d-1; j++)
      quaternion_inverse(B_inv.V[j], B_inv.V[j]);
    if (compute_compose_errors) 
      printf("compose error 3: %f\n", bingham_compose_error(&B_obs, &B_inv));
    bingham_compose(&dB_obs, &B_obs, &B_inv);
    bingham_free(&B_inv);
    //bingham_compose(&dB_obs, &B_process_noise, &dB_obs);  // double-noise model
    bingham_mult(&ball.dB, &ball.dB, &dB_obs);
    bingham_free(&dB_obs);
  }

  /*
  bingham_t B_process_noise;
  double Z_process[3] = {-400, -400, -400};
  double V[3][4] = {{0,0,1,0}, {0,0,0,1}, {0,1,0,0}};
  double *Vp[3] = {&V[0][0], &V[1][0], &V[2][0]};
  bingham_new(&B_process_noise, d, Vp, Z_process);
  if (compute_compose_errors)
    printf("compose error 2: %f\n", bingham_compose_error(&B_process_noise, &prev_ball.dB));
  bingham_compose(&ball.dB, &B_process_noise, &ball.dB);
  */

  //dbug
  double s[4], a[3];;
  bingham_mode(s, &ball.dB);
  quaternion_to_axis_angle(a,s);
  printf("detected orientation: (%.2f, %.2f, %.2f, %.2f), spin=(%.2f, %.2f, %.2f)\n",
	 ball.oq[0], ball.oq[1], ball.oq[2], ball.oq[3], a[0], a[1], a[2]);

  //ball.wx = a[0] / .005;
  //ball.wy = a[1] / .005;
  //ball.wz = a[2] / .005;
}


/*
 * Kalman filter + Bingham filter
 */
void filter_ball(ball_t &ball, ball_t &prev_ball)
{
  //filter_ball_position(ball, prev_ball);
  filter_ball_position_magnus(ball, prev_ball);
  //filter_ball_spin(ball, prev_ball);
}


/*
 *  Poor man's Kalman filter:
 *    (1) predict where current ball should be based on prev_ball and a forward model
 *    (2) estimate position as a weighted average of predicted and observed positions
 *    (3) estimate velocity with finite differences
 *
void filter_ball(ball_t &ball, const ball_t &prev_ball)
{
  double dt = ball.t - prev_ball.t;
  vector<ball_t> traj = predict_ball_trajectory(prev_ball, 1, dt);
  ball_t &predicted_ball = traj.back();

  //printf("dt = %f\n", dt);

  ball.ox = ball.x;
  ball.oy = ball.y;
  ball.oz = ball.z;
  
  double a = .1;
  ball.x = a*ball.x + (1-a)*predicted_ball.x;
  ball.y = a*ball.y + (1-a)*predicted_ball.y;
  ball.z = a*ball.z + (1-a)*predicted_ball.z;

  ball.vx = a*(ball.x - prev_ball.x)/dt + (1-a)*predicted_ball.vx;
  ball.vy = a*(ball.y - prev_ball.y)/dt + (1-a)*predicted_ball.vy;

  // handle z-velocity of ball bounces differently
  if (prev_ball.z + dt*prev_ball.vz < BALL_RADIUS && ball_in_table_region(prev_ball) && fabs(prev_ball.vz) > 1e-16) {
    double dt_bounce = (prev_ball.z - BALL_RADIUS) / prev_ball.vz;
    ball.vz = a*(ball.z - BALL_RADIUS)/(dt-dt_bounce) + (1-a)*predicted_ball.vz;
  }
  else
    ball.vz = a*(ball.z - prev_ball.z)/dt + (1-a)*predicted_ball.vz;
}
*/



//--------------------- Image Processing Routines ----------------------//

rect_t roi_from_points(std::vector<cv::Point2d> points, uint roi_padding, rect_t max_roi)
{
  double x0 = points[0].x;
  double x1 = points[0].x;
  double y0 = points[0].y;
  double y1 = points[0].y;

  for (uint i = 1; i < points.size(); i++) {
    if (points[i].x < x0)
      x0 = points[i].x;
    else if (points[i].x > x1)
      x1 = points[i].x;
    if (points[i].y < y0)
      y0 = points[i].y;
    else if (points[i].y > y1)
      y1 = points[i].y;
  }

  // add padding
  rect_t roi;
  double p = (double)roi_padding;
  roi.x = MAX(x0 - p, max_roi.x);
  roi.x = MIN(roi.x, max_roi.x + max_roi.w);
  roi.y = MAX(y0 - p, max_roi.y);
  roi.y = MIN(roi.y, max_roi.y + max_roi.h);
  roi.w = MIN(x1-x0+1 + 2*p, max_roi.w - roi.x + max_roi.x);
  roi.h = MIN(y1-y0+1 + 2*p, max_roi.h - roi.y + max_roi.y);

  //printf("roi = (%d, %d, %d, %d)\n", roi.x, roi.y, roi.w, roi.h); //dbug

  return roi;
}


static const uchar BITS_NUM[256] = {0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4,
				    1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,
				    1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,
				    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
				    1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,
				    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
				    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
				    3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
				    1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,
				    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
				    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
				    3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
				    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
				    3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
				    3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
				    4,5,5,6,5,6,6,7,5,6,6,7,6,7,7,8};

static const uchar BITS_LIST[256][8] = {{0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {1,0,0,0,0,0,0,0}, {0,1,0,0,0,0,0,0},
					{2,0,0,0,0,0,0,0}, {0,2,0,0,0,0,0,0}, {1,2,0,0,0,0,0,0}, {0,1,2,0,0,0,0,0},
					{3,0,0,0,0,0,0,0}, {0,3,0,0,0,0,0,0}, {1,3,0,0,0,0,0,0}, {0,1,3,0,0,0,0,0},
					{2,3,0,0,0,0,0,0}, {0,2,3,0,0,0,0,0}, {1,2,3,0,0,0,0,0}, {0,1,2,3,0,0,0,0},
					{4,0,0,0,0,0,0,0}, {0,4,0,0,0,0,0,0}, {1,4,0,0,0,0,0,0}, {0,1,4,0,0,0,0,0},
					{2,4,0,0,0,0,0,0}, {0,2,4,0,0,0,0,0}, {1,2,4,0,0,0,0,0}, {0,1,2,4,0,0,0,0},
					{3,4,0,0,0,0,0,0}, {0,3,4,0,0,0,0,0}, {1,3,4,0,0,0,0,0}, {0,1,3,4,0,0,0,0},
					{2,3,4,0,0,0,0,0}, {0,2,3,4,0,0,0,0}, {1,2,3,4,0,0,0,0}, {0,1,2,3,4,0,0,0},
					{5,0,0,0,0,0,0,0}, {0,5,0,0,0,0,0,0}, {1,5,0,0,0,0,0,0}, {0,1,5,0,0,0,0,0},
					{2,5,0,0,0,0,0,0}, {0,2,5,0,0,0,0,0}, {1,2,5,0,0,0,0,0}, {0,1,2,5,0,0,0,0},
					{3,5,0,0,0,0,0,0}, {0,3,5,0,0,0,0,0}, {1,3,5,0,0,0,0,0}, {0,1,3,5,0,0,0,0},
					{2,3,5,0,0,0,0,0}, {0,2,3,5,0,0,0,0}, {1,2,3,5,0,0,0,0}, {0,1,2,3,5,0,0,0},
					{4,5,0,0,0,0,0,0}, {0,4,5,0,0,0,0,0}, {1,4,5,0,0,0,0,0}, {0,1,4,5,0,0,0,0},
					{2,4,5,0,0,0,0,0}, {0,2,4,5,0,0,0,0}, {1,2,4,5,0,0,0,0}, {0,1,2,4,5,0,0,0},
					{3,4,5,0,0,0,0,0}, {0,3,4,5,0,0,0,0}, {1,3,4,5,0,0,0,0}, {0,1,3,4,5,0,0,0},
					{2,3,4,5,0,0,0,0}, {0,2,3,4,5,0,0,0}, {1,2,3,4,5,0,0,0}, {0,1,2,3,4,5,0,0},
					{6,0,0,0,0,0,0,0}, {0,6,0,0,0,0,0,0}, {1,6,0,0,0,0,0,0}, {0,1,6,0,0,0,0,0},
					{2,6,0,0,0,0,0,0}, {0,2,6,0,0,0,0,0}, {1,2,6,0,0,0,0,0}, {0,1,2,6,0,0,0,0},
					{3,6,0,0,0,0,0,0}, {0,3,6,0,0,0,0,0}, {1,3,6,0,0,0,0,0}, {0,1,3,6,0,0,0,0},
					{2,3,6,0,0,0,0,0}, {0,2,3,6,0,0,0,0}, {1,2,3,6,0,0,0,0}, {0,1,2,3,6,0,0,0},
					{4,6,0,0,0,0,0,0}, {0,4,6,0,0,0,0,0}, {1,4,6,0,0,0,0,0}, {0,1,4,6,0,0,0,0},
					{2,4,6,0,0,0,0,0}, {0,2,4,6,0,0,0,0}, {1,2,4,6,0,0,0,0}, {0,1,2,4,6,0,0,0},
					{3,4,6,0,0,0,0,0}, {0,3,4,6,0,0,0,0}, {1,3,4,6,0,0,0,0}, {0,1,3,4,6,0,0,0},
					{2,3,4,6,0,0,0,0}, {0,2,3,4,6,0,0,0}, {1,2,3,4,6,0,0,0}, {0,1,2,3,4,6,0,0},
					{5,6,0,0,0,0,0,0}, {0,5,6,0,0,0,0,0}, {1,5,6,0,0,0,0,0}, {0,1,5,6,0,0,0,0},
					{2,5,6,0,0,0,0,0}, {0,2,5,6,0,0,0,0}, {1,2,5,6,0,0,0,0}, {0,1,2,5,6,0,0,0},
					{3,5,6,0,0,0,0,0}, {0,3,5,6,0,0,0,0}, {1,3,5,6,0,0,0,0}, {0,1,3,5,6,0,0,0},
					{2,3,5,6,0,0,0,0}, {0,2,3,5,6,0,0,0}, {1,2,3,5,6,0,0,0}, {0,1,2,3,5,6,0,0},
					{4,5,6,0,0,0,0,0}, {0,4,5,6,0,0,0,0}, {1,4,5,6,0,0,0,0}, {0,1,4,5,6,0,0,0},
					{2,4,5,6,0,0,0,0}, {0,2,4,5,6,0,0,0}, {1,2,4,5,6,0,0,0}, {0,1,2,4,5,6,0,0},
					{3,4,5,6,0,0,0,0}, {0,3,4,5,6,0,0,0}, {1,3,4,5,6,0,0,0}, {0,1,3,4,5,6,0,0},
					{2,3,4,5,6,0,0,0}, {0,2,3,4,5,6,0,0}, {1,2,3,4,5,6,0,0}, {0,1,2,3,4,5,6,0},
					{7,0,0,0,0,0,0,0}, {0,7,0,0,0,0,0,0}, {1,7,0,0,0,0,0,0}, {0,1,7,0,0,0,0,0},
					{2,7,0,0,0,0,0,0}, {0,2,7,0,0,0,0,0}, {1,2,7,0,0,0,0,0}, {0,1,2,7,0,0,0,0},
					{3,7,0,0,0,0,0,0}, {0,3,7,0,0,0,0,0}, {1,3,7,0,0,0,0,0}, {0,1,3,7,0,0,0,0},
					{2,3,7,0,0,0,0,0}, {0,2,3,7,0,0,0,0}, {1,2,3,7,0,0,0,0}, {0,1,2,3,7,0,0,0},
					{4,7,0,0,0,0,0,0}, {0,4,7,0,0,0,0,0}, {1,4,7,0,0,0,0,0}, {0,1,4,7,0,0,0,0},
					{2,4,7,0,0,0,0,0}, {0,2,4,7,0,0,0,0}, {1,2,4,7,0,0,0,0}, {0,1,2,4,7,0,0,0},
					{3,4,7,0,0,0,0,0}, {0,3,4,7,0,0,0,0}, {1,3,4,7,0,0,0,0}, {0,1,3,4,7,0,0,0},
					{2,3,4,7,0,0,0,0}, {0,2,3,4,7,0,0,0}, {1,2,3,4,7,0,0,0}, {0,1,2,3,4,7,0,0},
					{5,7,0,0,0,0,0,0}, {0,5,7,0,0,0,0,0}, {1,5,7,0,0,0,0,0}, {0,1,5,7,0,0,0,0},
					{2,5,7,0,0,0,0,0}, {0,2,5,7,0,0,0,0}, {1,2,5,7,0,0,0,0}, {0,1,2,5,7,0,0,0},
					{3,5,7,0,0,0,0,0}, {0,3,5,7,0,0,0,0}, {1,3,5,7,0,0,0,0}, {0,1,3,5,7,0,0,0},
					{2,3,5,7,0,0,0,0}, {0,2,3,5,7,0,0,0}, {1,2,3,5,7,0,0,0}, {0,1,2,3,5,7,0,0},
					{4,5,7,0,0,0,0,0}, {0,4,5,7,0,0,0,0}, {1,4,5,7,0,0,0,0}, {0,1,4,5,7,0,0,0},
					{2,4,5,7,0,0,0,0}, {0,2,4,5,7,0,0,0}, {1,2,4,5,7,0,0,0}, {0,1,2,4,5,7,0,0},
					{3,4,5,7,0,0,0,0}, {0,3,4,5,7,0,0,0}, {1,3,4,5,7,0,0,0}, {0,1,3,4,5,7,0,0},
					{2,3,4,5,7,0,0,0}, {0,2,3,4,5,7,0,0}, {1,2,3,4,5,7,0,0}, {0,1,2,3,4,5,7,0},
					{6,7,0,0,0,0,0,0}, {0,6,7,0,0,0,0,0}, {1,6,7,0,0,0,0,0}, {0,1,6,7,0,0,0,0},
					{2,6,7,0,0,0,0,0}, {0,2,6,7,0,0,0,0}, {1,2,6,7,0,0,0,0}, {0,1,2,6,7,0,0,0},
					{3,6,7,0,0,0,0,0}, {0,3,6,7,0,0,0,0}, {1,3,6,7,0,0,0,0}, {0,1,3,6,7,0,0,0},
					{2,3,6,7,0,0,0,0}, {0,2,3,6,7,0,0,0}, {1,2,3,6,7,0,0,0}, {0,1,2,3,6,7,0,0},
					{4,6,7,0,0,0,0,0}, {0,4,6,7,0,0,0,0}, {1,4,6,7,0,0,0,0}, {0,1,4,6,7,0,0,0},
					{2,4,6,7,0,0,0,0}, {0,2,4,6,7,0,0,0}, {1,2,4,6,7,0,0,0}, {0,1,2,4,6,7,0,0},
					{3,4,6,7,0,0,0,0}, {0,3,4,6,7,0,0,0}, {1,3,4,6,7,0,0,0}, {0,1,3,4,6,7,0,0},
					{2,3,4,6,7,0,0,0}, {0,2,3,4,6,7,0,0}, {1,2,3,4,6,7,0,0}, {0,1,2,3,4,6,7,0},
					{5,6,7,0,0,0,0,0}, {0,5,6,7,0,0,0,0}, {1,5,6,7,0,0,0,0}, {0,1,5,6,7,0,0,0},
					{2,5,6,7,0,0,0,0}, {0,2,5,6,7,0,0,0}, {1,2,5,6,7,0,0,0}, {0,1,2,5,6,7,0,0},
					{3,5,6,7,0,0,0,0}, {0,3,5,6,7,0,0,0}, {1,3,5,6,7,0,0,0}, {0,1,3,5,6,7,0,0},
					{2,3,5,6,7,0,0,0}, {0,2,3,5,6,7,0,0}, {1,2,3,5,6,7,0,0}, {0,1,2,3,5,6,7,0},
					{4,5,6,7,0,0,0,0}, {0,4,5,6,7,0,0,0}, {1,4,5,6,7,0,0,0}, {0,1,4,5,6,7,0,0},
					{2,4,5,6,7,0,0,0}, {0,2,4,5,6,7,0,0}, {1,2,4,5,6,7,0,0}, {0,1,2,4,5,6,7,0},
					{3,4,5,6,7,0,0,0}, {0,3,4,5,6,7,0,0}, {1,3,4,5,6,7,0,0}, {0,1,3,4,5,6,7,0},
					{2,3,4,5,6,7,0,0}, {0,2,3,4,5,6,7,0}, {1,2,3,4,5,6,7,0}, {0,1,2,3,4,5,6,7}};

int connected_components_sse(connected_components_t &C, uchar *I, int w, int h)
{
  int ncomp = 0;
  std::vector<uint> x_tot;
  std::vector<uint> y_tot;
  std::vector<uint> num_pixels;
  //std::vector<short> label_holes;  // labels that have been merged with other labels
  std::vector<short> label_map;    // mapping the holes to non-holes

  ushort RLE[16][h];   // current 16 columns' RLE
  ushort RLE_cnt[16];  // current 16 columns' RLE cnt

  ushort RLE_prev_cnt = 0;  // previous column's RLE cnt
  ushort RLE_prev[h];       // previous column's RLE
  short L_prev[h];         // previous column's cluster labels
  short L[h];              // current column's cluster labels

  for (int x = 0; x < w; x+=16) {

    //if (x > 0)  //dbug
    //  goto CLEANUP;

    // step 1: create column-wise RLE (run-length encoding)
    memset(RLE_cnt, 0, 16*sizeof(ushort));
    uchar *I_ptr = I+x;
    __m128i prev_row = _mm_set1_epi8(0);
    for (int y = 0; y < h; y++, I_ptr+=w) {
      __m128i curr_row = _mm_load_si128((__m128i *)(I_ptr));
      uint nochange_mask = _mm_movemask_epi8(_mm_cmpeq_epi8(prev_row, curr_row));
      uint change_mask = ~nochange_mask;
      // bits 0-7
      uint c = change_mask & 0xFF;
      uchar bnum = BITS_NUM[c];
      for (uchar i = 0; i < bnum; i++) {
	uchar bi = BITS_LIST[c][i];
	RLE[bi][ RLE_cnt[bi]++ ] = y;
      }
      // bits 8-15
      c = (change_mask>>8) & 0xFF;
      bnum = BITS_NUM[c];
      for (uchar i = 0; i < bnum; i++) {
	uchar bi = BITS_LIST[c][i] + 8;
	RLE[bi][ RLE_cnt[bi]++ ] = y;
      }
      prev_row = curr_row;
    }
    __m128i curr_row = _mm_set1_epi8(0);
    uint nochange_mask = _mm_movemask_epi8(_mm_cmpeq_epi8(prev_row, curr_row));
    uint change_mask = ~nochange_mask;
    // bits 0-7
    uint c = change_mask & 0xFF;
    uchar bnum = BITS_NUM[c];
    for (uchar i = 0; i < bnum; i++) {
      uchar bi = BITS_LIST[c][i];
      RLE[bi][ RLE_cnt[bi]++ ] = h;
    }
    // bits 8-15
    c = (change_mask>>8) & 0xFF;
    bnum = BITS_NUM[c];
    for (uchar i = 0; i < bnum; i++) {
      uchar bi = BITS_LIST[c][i] + 8;
      RLE[bi][ RLE_cnt[bi]++ ] = h;
    }
    
    //dbug
    //for (int i = 0; i < 16; i++) {
    //  for (int j = 0; j < RLE_cnt[i]; j++)
    //	printf("%d ", RLE[i][j]);
    //  printf("\n");
    //}

    // step 2: merge clusters and compute statistics
    for (int dx = 0; dx < 16; dx++) {
      int i_prev = 0;
      for (int i = 0; i < RLE_cnt[dx]; i+=2) {  // for each run in the current column
	short label = -1, prev_label;
	ushort r0 = RLE[dx][i];        // current run start
	ushort r1 = RLE[dx][i+1];      // current run end
	uint rn = r1-r0;               // number of pixels in current run
	uint xi_sum = rn*(x+dx);       // x-sum of current run
	uint yi_sum = (r0+r1-1)*rn/2;  // y-sum of current run
	ushort p0, p1;

	//printf("r0 = %d, r1 = %d\n", r0, r1);

	// find first run in previous column that ends after current run starts
	for (; i_prev < RLE_prev_cnt; i_prev+=2) {
	  p1 = RLE_prev[i_prev+1];  // previous column's run end
	  //printf("p1 = %d\n", p1);
	  if (p1 > r0)
	    break;
	}

	// if previous column's run overlaps with current run, add current run's pixels to previous run's pixels
	if (i_prev < RLE_prev_cnt) {
	  p0 = RLE_prev[i_prev];  // previous column's run start
	  //printf("p0 = %d\n", p0);
	  if (p0 < r1) {  // overlap
	    //printf("overlap\n");
	    prev_label = L_prev[i_prev/2];  // label of previous column's run
	    while (label_map[prev_label] != prev_label)  // previous column's run has been relabeled
	      prev_label = label_map[prev_label];
	    //printf("prev_label = %d\n", prev_label);
	    label = prev_label;
	    x_tot[label] += xi_sum;
	    y_tot[label] += yi_sum;
	    num_pixels[label] += rn;
	  }
	}

	// if no run in previous column overlaps with current run, create a new label
	if (label < 0) {  // new label
	  label = ncomp++;
	  //printf("new label: %d\n", label);
	  x_tot.push_back(xi_sum);
	  y_tot.push_back(yi_sum);
	  num_pixels.push_back(rn);
	  label_map.push_back(label);
	}
	else if (i_prev < RLE_prev_cnt && p1 < r1) {  // check for more overlaps with previous column
	  for (i_prev+=2; i_prev < RLE_prev_cnt; i_prev+=2) {
	    p0 = RLE_prev[i_prev];  // previous column's run start
	    //printf("p0 = %d\n", p0);
	    if (p0 < r1) {  // overlap
	      prev_label = L_prev[i_prev/2];
	      while (label_map[prev_label] != prev_label)  // previous column's run has been relabeled
		prev_label = label_map[prev_label];
	      //printf("prev_label = %d\n", prev_label);
	      if (prev_label != label) {  // merge prev_label with label
		//printf("prev_label != label, merging...\n");
		x_tot[prev_label] += x_tot[label];
		y_tot[prev_label] += y_tot[label];
		num_pixels[prev_label] += num_pixels[label];
		//label_holes.push_back(label);
		label_map[label] = prev_label;
		label = prev_label;
	      }
	      p1 = RLE_prev[i_prev+1];
	      if (p1 >= r1)  // previous run ends after current run
		break;
	    }
	    else  // no overlap
	      break;
	  }
	}

	// set label
	L[i/2] = label;
      }

      // done with the column, so shift buffers
      RLE_prev_cnt = RLE_cnt[dx];
      for (int i = 0; i < RLE_prev_cnt; i++)
	RLE_prev[i] = RLE[dx][i];
      memcpy(L_prev, L, sizeof(short)*RLE_prev_cnt/2);
    }
  }

  //std::vector<uint> x_tot;
  //std::vector<uint> y_tot;
  //std::vector<uint> num_pixels;

  //CLEANUP:

  // clean up the holes and copy component statistics into C
  for (int i = 0; i < ncomp; i++) {
    if (label_map[i] == i) {
      C.cluster_x.push_back(x_tot[i] / (float)num_pixels[i]);
      C.cluster_y.push_back(y_tot[i] / (float)num_pixels[i]);
      C.cluster_size.push_back(num_pixels[i]);
    }
  }

  //printf("Removed holes: %d -> %d\n", ncomp, C.cluster_x.size());

  return C.cluster_x.size();
}


int connected_components(ushort *C, uchar *I, int w, int h)
{
  //double t0 = ros::Time::now().toSec();

  int n = 0;

  // top-left pixel
  C[0] = (I[0] ? ++n : 0);

  // top row
  for (int x = 1; x < w; x++) {
    if (I[x] == 0)
      C[x] = 0;
    else
      C[x] = (C[x-1] ? C[x-1] : ++n);
  }
  
  int max_cluster = w*h/2;
  ushort equivalences[max_cluster][2];
  int eq_cnt = 0;

  //printf("connected components 1:  %f sec.\n", ros::Time::now().toSec() - t0);  //dbug

  for (int y = 1; y < h; y++) {
    // left pixel
    int i = y*w;
    if (I[i] == 0)
      C[i] = 0;
    else
      C[i] = (C[i-w] ? C[i-w] : ++n);

    // rest of the row
    for (int x = 1; x < w; x++) {
      i = y*w+x;
      bool empty = (I[i] == 0);
      ushort c; //, eq0=0, eq1=0;
      if (empty)
	c = 0;
      else {
	//ushort left = 1, top = 0;
	//ushort left = C[i-1], top = C[i-w];
	ushort left = I[i-1], top = I[i-w];
	if (left && top) {
	  if (left == top)
	    c = left;
	  else {
	    int c1 = MIN(left, top);
	    c = c1;
	    //eq0 = c1;
	    //eq1 = MAX(left, top);
	    equivalences[eq_cnt][0] = c1;
	    equivalences[eq_cnt++][1] = MAX(left, top);
	  }
	}
	else if (left)
	  c = left;
	else if (top)
	  c = top;
	else
	  c = ++n;
      }
      C[i] = c;
    }
  }

  //printf("n = %d, eq_cnt = %d\n", n, eq_cnt);
  //printf("connected components 2:  %f sec.\n", ros::Time::now().toSec() - t0);  //dbug

  short cmap[n+1];  // cluster mappings
  memset(cmap, 0, sizeof(short)*(n+1));
  for (int i = 0; i < eq_cnt; i++) {
    int c1 = equivalences[i][0];
    int c2 = equivalences[i][1];
    while (cmap[c2] && c2!=c1) {
      int c3 = cmap[c2];
      cmap[c2] = MIN(c1,c3);
      c2 = MAX(c1,c3);
      c1 = cmap[c2];
    }
    cmap[c2] = c1;
  }

  //printf("connected components 3:  %f sec.\n", ros::Time::now().toSec() - t0);  //dbug

  // compress cluster map, step 1  [0 0 1 2 0 0 5 6 4 8] --> [0 1 1 1 4 5 5 5 4 4]
  for (int i = 1; i <= n; i++) {  
    int c = cmap[i];
    cmap[i] = (c ? cmap[c] : i);
  }
  // compress cluster map, step 2  [0 1 1 1 4 5 5 5 4 4] --> [0 1 1 1 2 3 3 3 2 2]
  int cnt = 0;
  for (int i = 1; i <= n; i++) {  
    cmap[i] = cmap[cmap[i]];
    if (cmap[i] > cnt)
      cmap[i] = ++cnt;
  }

  //printf("connected components 4:  %f sec.\n", ros::Time::now().toSec() - t0);  //dbug

  // relabel connected components image
  for (int i = 0; i < w*h; i++)
    C[i] = cmap[C[i]];

  //printf("connected components 5:  %f sec.\n", ros::Time::now().toSec() - t0);  //dbug

  printf("connected components --> found %d clusters\n", cnt);

  return cnt;
}

/*
uint *connected_components(const image_t &I, rect_t &roi)
{
  //printf("connected_components()\n");

  //C = I;  // initialize headers, etc.
  uint w = I.width;
  uint h = I.height;
  uint *C;
  safe_calloc(C, w*h, uint);

  // compute non-zero pixel indices
  uint num_pixels = 0;
  for (uint y = roi.y; y < roi.y+roi.h; y++) {
    for (uint x = roi.x; x < roi.x+roi.w; x++) {
      uint i = y*w + x;
      if (I.data[i])
	num_pixels++;
    }
  }
  uint idx[num_pixels];
  num_pixels = 0;
  for (uint y = roi.y; y < roi.y+roi.h; y++) {
    for (uint x = roi.x; x < roi.x+roi.w; x++) {
      uint i = y*w + x;
      if (I.data[i])
	idx[num_pixels++] = i;
    }
  }

  // compute connected components image
  uint num_comp = 0;
  uchar holes[num_pixels];
  for (uint i = 0; i < num_pixels; i++)
    holes[i] = 0;
  for (uint i = 0; i < num_pixels; i++) {
    uint x = idx[i] % w;
    uint y = idx[i] / w;
    //printf("x = %d, y = %d, i = %d, idx[i] = %d, y*w+x = %d\n", x, y, i, idx[i], y*w+x);
    //if (I.data[y*w+x]) {
    uint top = (y > 0 ? C[(y-1)*w+x] : 0);
    uint left = (x > 0 ? C[y*w+x-1] : 0);
    //printf("i = %d, x = %d, y = %d, num_comp = %d, top = %d, left = %d\n", i, x, y, num_comp, top, left);
    if (top>0) {
      C[y*w+x] = top;
      if (left>0 && left!=top) {  // connect left and top
	//if (left > num_comp)
	//  printf("num_comp = %d,left = %d, top = %d\n", num_comp, left, top);
	holes[left] = 1;
	for (uint j = 0; j < i; j++)
	  if (C[idx[j]] == left)
	    C[idx[j]] = top;
      }
    }
    else if (left>0)
      C[y*w+x] = left;
    else
      C[y*w+x] = ++num_comp;
    //}
  }

  // compress component indices
  uint cmap[num_comp+1];
  uint cnt = 0;
  for (uint i = 1; i <= num_comp; i++)
    if (!holes[i])
      cmap[i] = ++cnt;
  num_comp = cnt;
  for (uint i = 0; i < num_pixels; i++) {
    if (holes[C[idx[i]]])
      printf("ERROR: %d is a hole!\n", C[idx[i]]);
    C[idx[i]] = cmap[C[idx[i]]];
  }

  return C;
}
*/

void crop_image(unsigned char *dst, unsigned char *src, int src_width, int src_height, int x, int y, int w, int h)
{
  int x0 = MAX(0, x);
  int y0 = MAX(0, y);
  int x1 = MIN(src_width, x+w);
  int y1 = MIN(src_height, y+h);

  for (int i = y0; i < y1; i++)
    for (int j = x0; j < x1; j++)
      dst[(i-y)*w + j-x] = src[i*src_width + j];
}


// resize an image with bilinear interpolation
void resize_image(uchar *dst, uchar *src, int src_width, int src_height, int w, int h)
{
  int w0 = src_width;
  int h0 = src_height;
  double x_step = w0 / (double) w;
  double y_step = h0 / (double) h;

  double y = 0;
  for (int i = 0; i < h; i++, y+=y_step) {
    double dy = y - floor(y);
    int i0 = (int)y;
    int i1 = (i0+1 < h0 ? i0+1 : i0);
    double x = 0;
    for (int j = 0; j < w; j++, x+=x_step) {
      double dx = x - floor(x);
      int j0 = (int)x;
      int j1 = (j0+1 < w0 ? j0+1 : j0);
      dst[i*w+j] = (1-dx)*(1-dy)*src[i0*w0+j0] + (1-dx)*dy*src[i1*w0+j0] + dx*(1-dy)*src[i0*w0+j1] + dx*dy*src[i1*w0+j1];
    }
  }
}


// compute alignment of A onto B with search window radius r
void compute_image_alignment(int &dx, int &dy, uchar *A, uchar *B, int w, int h, int r)
{
  double fmax = 0;
  int dxmax = -1;
  int dymax = -1;
  for (dy = -r; dy <= r; dy++) {
    int y0 = MAX(0, -dy);
    int y1 = h + MIN(0, -dy);
    for (dx = -r; dx <= r; dx++) {
      int x0 = MAX(0, -dx);
      int x1 = w + MIN(0, -dx);
      double f = 0;
      for (int y = y0; y < y1; y++) {
	for (int x = x0; x < x1; x++) {
	  double a = A[y*w+x];
	  double b = B[(y+dy)*w+(x+dx)];
	  f += a*b;
	}
      }
      if (f > fmax) {
	fmax = f;
	dxmax = dx;
	dymax = dy;
      }
    }
  }

  dx = dxmax;
  dy = dymax;
}


// shift an image, filling empty space with zeros
void shift_image(uchar *dst, uchar *src, int w, int h, int dx, int dy)
{
  int x0 = MAX(0, dx);
  int y0 = MAX(0, dy);
  int x1 = MIN(w, dx+w);
  int y1 = MIN(h, dy+h);

  memset(dst, 0, w*h);

  for (int i = y0; i < y1; i++)
    for (int j = x0; j < x1; j++)
      dst[i*w+j] = src[(i-y0)*w+(j-x0)];
}


// compute the integral image
void integral_image(uint *dst, uchar *src, int w, int h)
{
  dst[0] = src[0];
  for (int x = 1; x < w; x++)
    dst[x] = dst[x-1] + src[x];
  for (int y = 1; y < h; y++) {
    dst[y*w] = dst[(y-1)*w] + src[y*w];
    for (int x = 1; x < w; x++)
      dst[y*w+x] = dst[y*w+x-1] + dst[(y-1)*w+x] - dst[(y-1)*w+x-1] + src[y*w+x];
  }
}


// blur image with a moving average filter
void blur_image(uchar *dst, uchar *src, int w, int h, int r)
{
  uint I[w*h];
  integral_image(I, src, w, h);
  
  //printf("I = ["); for (int i = 0; i < w*h; i++) printf("%d ", I[i]); printf("]\n"); //dbug

  for (int y = 0; y < h; y++) {
    int y0 = MAX(y-r, 0);
    int y1 = MIN(y+r, h-1);
    for (int x = 0; x < w; x++) {
      int x0 = MAX(x-r, 0);
      int x1 = MIN(x+r, w-1);
      int v = I[y1*w+x1];
      if (x0 > 0)
	v -= I[y1*w+x0-1];
      if (y0 > 0)
	v -= I[(y0-1)*w+x1];
      if (x0 > 0 && y0 > 0)
	v += I[(y0-1)*w+x0-1];
      double d = (y1-y0+1)*(x1-x0+1);
      dst[y*w+x] = v/d;
    }
  }
}


// find the pixel-weighted centroid of image I
void image_centroid(int &mx, int &my, uchar *I, int w, int h)
{
  mx = 0;
  my = 0;
  int vtot = 0;
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      uchar v = I[y*w+x];
      vtot += v;
      mx += x*v;
      my += y*v;
    }
  }

  if (vtot > 0) {
    mx /= (double)vtot;
    my /= (double)vtot;
  }
}


// local grid search optimization: (mx,my) is both the starting point and the finish point
void image_find_local_mode(int &mx, int &my, uchar *I, int w, int h, int r, int rmax)
{
  int by0 = MAX(my-rmax, 0);
  int by1 = MIN(my+rmax, h-1);
  int bx0 = MAX(mx-rmax, 0);
  int bx1 = MIN(mx+rmax, w-1);

  uchar fmax = I[my*w+mx];
  while (1) {
    int y0 = MAX(my-r, by0);
    int y1 = MIN(my+r, by1);
    int x0 = MAX(mx-r, bx0);
    int x1 = MIN(mx+r, bx1);
    
    bool converged = true;
    for (int y = y0; y <= y1; y++) {
      for (int x = x0; x <= x1; x++) {
	uchar f = I[y*w+x];
	if (f > fmax) {
	  fmax = f;
	  mx = x;
	  my = y;
	  converged = false;
	}
      }
    }

    if (converged)
      break;
  }
}


void edge_image(sensor_msgs::Image &E, const sensor_msgs::Image &I)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(I, enc::MONO8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  Mat1b E_cv;
  //I.assignTo(I2, DataType<uchar>::type);
  E_cv.create(cv_ptr->image.size());
  //Canny(cv_ptr->image, E_cv, 20, 40);
  Canny(cv_ptr->image, E_cv, 30, 50);

  E = I;
  uint w = E.width, h = E.height;
  for (uint y = 0; y < h; y++)
    for (uint x = 0; x < w; x++)
      E.data[y*w+x] = E_cv(y,x);
}


void distance_transform(sensor_msgs::Image &D, const sensor_msgs::Image &I)
{
  std::queue<int> Q;  // queue for wavefront algorithm
  int w = I.width, h = I.height;
  D = I;
  for (int i = 0; i < w*h; i++) {
    if (I.data[i] == 0)
      D.data[i] = 255;
    else {
      D.data[i] = 0;
      Q.push(i);
    }
  }

  while (!Q.empty()) {
    int i = Q.front();
    Q.pop();
    int x = i%w, y = i/w;
    int d = D.data[i] + 1;
    int neighbors[4] = {(x>0 ? y*w+x-1 : -1), (x<w-1 ? y*w+x+1 : -1),
			(y>0 ? (y-1)*w+x : -1), (y<h-1 ? (y+1)*w+x : -1)};
    for (int j = 0; j < 4; j++) {
      int n = neighbors[j];
      if (n >= 0 && d < D.data[n]) {
	D.data[n] = d;
	Q.push(n);
      }
    }
  }
}


void table_corners_from_transform(cv::Point3d corners[], const tf::Transform &table_camera_transform)
{
  cv::Point3d SW, SE, NW, NE;

  SW.x = table_camera_transform.getOrigin()[0];
  SW.y = table_camera_transform.getOrigin()[1];
  SW.z = table_camera_transform.getOrigin()[2];

  double **R = new_matrix2(3,3);
  for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
      R[i][j] = table_camera_transform.getBasis()[i][j];

  SE.x = SW.x + TABLE_WIDTH*R[0][0];
  SE.y = SW.y + TABLE_WIDTH*R[1][0];
  SE.z = SW.z + TABLE_WIDTH*R[2][0];

  NW.x = SW.x + TABLE_LENGTH*R[0][1];
  NW.y = SW.y + TABLE_LENGTH*R[1][1];
  NW.z = SW.z + TABLE_LENGTH*R[2][1];

  NE.x = NW.x + TABLE_WIDTH*R[0][0];
  NE.y = NW.y + TABLE_WIDTH*R[1][0];
  NE.z = NW.z + TABLE_WIDTH*R[2][0];

  // corner order: NW, NE, SE, SW (where NORTH points towards the far side of the table)
  corners[0] = NW;
  corners[1] = NE;
  corners[2] = SE;
  corners[3] = SW;

  free_matrix2(R);
}


inline bool point_in_image_bounds(const cv::Point2d &p, const sensor_msgs::Image &I)
{
  return p.x >= 0 && p.x < I.width && p.y >= 0 && p.y < I.height;
}

// lower is better
double table_fitness(const tf::Transform &table_camera_transform,
		     const sensor_msgs::Image &D_left, const sensor_msgs::Image &D_right,
		     const image_geometry::StereoCameraModel &stereo_camera_model)
{
  // outside corners, inside corners, NS middle line, EW middle line (all in "NW-NE-SE-SW" order)
  tf::Point corners[16];  

  // get table corners (in camera coordinates) - NW, NE, SE, SW
  corners[0] = table_camera_transform(tf::Point(0, TABLE_LENGTH, 0));
  corners[1] = table_camera_transform(tf::Point(TABLE_WIDTH, TABLE_LENGTH, 0));
  corners[2] = table_camera_transform(tf::Point(TABLE_WIDTH, 0, 0));
  corners[3] = table_camera_transform(tf::Point(0, 0, 0));

  // get inside table corners (inside the white lines)
  corners[4] = table_camera_transform(tf::Point(TABLE_EDGE_WIDTH, TABLE_LENGTH - TABLE_EDGE_WIDTH, 0));
  corners[5] = table_camera_transform(tf::Point(TABLE_WIDTH - TABLE_EDGE_WIDTH, TABLE_LENGTH - TABLE_EDGE_WIDTH, 0));
  corners[6] = table_camera_transform(tf::Point(TABLE_WIDTH - TABLE_EDGE_WIDTH, TABLE_EDGE_WIDTH, 0));
  corners[7] = table_camera_transform(tf::Point(TABLE_EDGE_WIDTH, TABLE_EDGE_WIDTH, 0));

  // get NS middle line corners
  corners[8] = table_camera_transform(tf::Point(TABLE_WIDTH/2.0 - TABLE_EDGE_WIDTH/4.0, TABLE_LENGTH, 0));
  corners[9] = table_camera_transform(tf::Point(TABLE_WIDTH/2.0 + TABLE_EDGE_WIDTH/4.0, TABLE_LENGTH, 0));
  corners[10] = table_camera_transform(tf::Point(TABLE_WIDTH/2.0 + TABLE_EDGE_WIDTH/4.0, 0, 0));
  corners[11] = table_camera_transform(tf::Point(TABLE_WIDTH/2.0 - TABLE_EDGE_WIDTH/4.0, 0, 0));

  // get EW middle line corners
  corners[12] = table_camera_transform(tf::Point(0, TABLE_LENGTH/2.0 + TABLE_EDGE_WIDTH/4.0, 0));
  corners[13] = table_camera_transform(tf::Point(TABLE_WIDTH, TABLE_LENGTH/2.0 + TABLE_EDGE_WIDTH/4.0, 0));
  corners[14] = table_camera_transform(tf::Point(TABLE_WIDTH, TABLE_LENGTH/2.0 - TABLE_EDGE_WIDTH/4.0, 0));
  corners[15] = table_camera_transform(tf::Point(0, TABLE_LENGTH/2.0 - TABLE_EDGE_WIDTH/4.0, 0));

  const int num_edges = 12;
  int edges[num_edges][2] = {{0,1},{1,2},{2,3},{3,0}, {4,5},{5,6},{6,7},{7,4}, {8,11},{9,10}, {12,13},{14,15}};
  double npoints[num_edges] = {10,20,10,20, 10,20,10,20, 20,20, 20,20};

  // project table points into left and right images, and test for fitness
  vector<cv::Point2d> left_points, right_points;
  for (int i = 0; i < num_edges; i++) {
    int e0 = edges[i][0];
    int e1 = edges[i][1];
    for (int j = 0; j < npoints[i]; j++) {
      double alpha = j/npoints[i];
      cv::Point3d p;
      p.x = alpha*corners[e0].x() + (1-alpha)*corners[e1].x();
      p.y = alpha*corners[e0].y() + (1-alpha)*corners[e1].y();
      p.z = alpha*corners[e0].z() + (1-alpha)*corners[e1].z();
      cv::Point2d p1 = stereo_camera_model.left().project3dToPixel(p);
      cv::Point2d p2 = stereo_camera_model.right().project3dToPixel(p);

      //p1 = stereo_camera_model.left().unrectifyPoint(p1);
      //p2 = stereo_camera_model.right().unrectifyPoint(p2);
      //p2.x -= stereo_camera_model.getDisparity(p.z);

      left_points.push_back(p1);
      right_points.push_back(p2);
    }
  }

  double d_tot = 0;
  for (int i = 0; i < left_points.size(); i++) {
    cv::Point2d &p = left_points[i];
    if (point_in_image_bounds(p, D_left)) {
      double d = D_left.data[((int)round(p.y))*D_left.width + (int)round(p.x)];
      d_tot += d*d;
    }
  }
  for (int i = 0; i < right_points.size(); i++) {
    cv::Point2d &p = right_points[i];
    if (point_in_image_bounds(p, D_right)) {
      double d = D_right.data[((int)round(p.y))*D_right.width + (int)round(p.x)];
      d_tot += d*d;
    }
  }

  return d_tot / sum(npoints,4);
}


double fit_table(tf::Transform &table_camera_transform,
		 const sensor_msgs::Image &left, const sensor_msgs::Image &right,
		 const image_geometry::StereoCameraModel &stereo_camera_model)
{
  // compute edge image distance transforms
  sensor_msgs::Image E_left, E_right, D_left, D_right;
  edge_image(E_left, left);
  edge_image(E_right, right);
  distance_transform(D_left, E_left);
  distance_transform(D_right, E_right);

  TableCameraTransformTester T(D_left, D_right, stereo_camera_model);
  TransformOptimizer O(&T);
  //O.setMaxIterations(100);
  table_camera_transform = O.optimizeTransform(table_camera_transform);
  double fitness = table_fitness(table_camera_transform, D_left, D_right, stereo_camera_model);

  /*dbug
  double **E_left_mat = new_matrix2(E_left.height, E_left.width);
  for (int i = 0; i < E_left.height*E_left.width; i++)
    E_left_mat[0][i] = E_left.data[i];
  save_matrix("E_left.txt", E_left_mat, E_left.height, E_left.width);
  free_matrix2(E_left_mat);
  double **E_right_mat = new_matrix2(E_right.height, E_right.width);
  for (int i = 0; i < E_right.height*E_right.width; i++)
    E_right_mat[0][i] = E_right.data[i];
  save_matrix("E_right.txt", E_right_mat, E_right.height, E_right.width);
  free_matrix2(E_right_mat);
  */

  /* dbug
  printf("min,max(D_left.data) = %f\n", min(D_left.data, D_left.width*D_left.height),
	 max(D_left.data, D_left.width*D_left.height));

  printf("table_camera_transform: t=(%f, %f, %f)\n", table_camera_transform.getOrigin()[0],
	 table_camera_transform.getOrigin()[1], table_camera_transform.getOrigin()[2]);
  */
  //printf("table fitness = %f\n", table_fitness(table_camera_transform, D_left, D_right, stereo_camera_model));

  //R[i][j] = table_camera_transform.getBasis()[i][j];

  return fitness;
}


// x y z qw qx qy qz
tf::Transform vector_to_transform(VectorXf x)
{
  tf::Transform T;
  T.setOrigin(tf::Vector3(x(0), x(1), x(2)));
  double d = sqrt(x(3)*x(3) + x(4)*x(4) + x(5)*x(5) + x(6)*x(6));  // normalize the quaternion
  T.setRotation(tf::Quaternion(x(4)/d, x(5)/d, x(6)/d, x(3)/d));
  return T;
}


VectorXf transform_to_vector(tf::Transform T)
{
  VectorXf x = VectorXf::Zero(7);
  x(0) = T.getOrigin()[0];
  x(1) = T.getOrigin()[1];
  x(2) = T.getOrigin()[2];
  x(3) = T.getRotation().w();
  x(4) = T.getRotation().x();
  x(5) = T.getRotation().y();
  x(6) = T.getRotation().z();
  return x;
}


//---------------------  TransformOptimizer class --------------------//

TransformOptimizer::TransformOptimizer(TransformTester *T) :
  GradientFreeRandomizedGradientOptimizer(.02, VectorXf::Zero(7), 1.0),   // TODO: clean this up
  transform_tester_(T)
{
  noise_ << .001, .001, .001, .003, .003, .003, .003;
}

float TransformOptimizer::evaluate(VectorXf x)
{
  return transform_tester_->test(vector_to_transform(x));
}

tf::Transform TransformOptimizer::optimizeTransform(const tf::Transform &initial_transform)
{
  VectorXf x = optimize(transform_to_vector(initial_transform));
  return vector_to_transform(x);
}


//---------------------  TableCameraTransformTester class --------------------//

TableCameraTransformTester::TableCameraTransformTester(const sensor_msgs::Image &left, const sensor_msgs::Image &right,
						       const image_geometry::StereoCameraModel &stereo_camera_model) :
L(left), R(right), S(stereo_camera_model)
{
}

double TableCameraTransformTester::test(const tf::Transform &T)
{
  return table_fitness(T,L,R,S);
}



//---------------------- TablePositionOptimizer class -------------------------//
float dist_from_plane(const pcl::PointXYZ& point, const Eigen::Vector4f& plane)
{ float num = plane(0)*point.x + plane(1)*point.y + plane(2)*point.z + plane(3);
  float denom = sqrt(plane(0)*plane(0) + plane(1)*plane(1) + plane(2)*plane(2));
  return num/denom;
}

Eigen::Vector3f project_to_plane(const pcl::PointXYZ& point, const Eigen::Vector4f& plane)
{ float d = dist_from_plane(point, plane);
  Eigen::Vector3f p,n;
  n(0) = plane(0);
  n(1) = plane(1);
  n(2) = plane(2);
  n.normalize();
  p(0) = point.x;
  p(1) = point.y;
  p(2) = point.z;
  return p - n * d;
}



  bool in_rectangle(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& p3,const Eigen::Vector3f& p4, const Eigen::Vector3f& p)
  { Eigen::Vector3f b,t,l,r,bn,tn,ln,rn;
    b = (p1 + p4)/2;
    r = (p1 + p2)/2;
    t = (p2 + p3)/2;
    l = (p3 + p4)/2;
    tn = (p2 - p1);
    tn.normalize();
    bn = -1 * tn;
    rn = (p1 - p4);
    rn.normalize();
    ln = -1 * rn;

    if ((p-r).dot(rn) < 0.0 && (p-t).dot(tn) < 0.0 && (p-l).dot(ln) < 0.0 && (p-b).dot(bn) < 0.0)
      return true;
    return false;
  }

TablePositionOptimizer::TablePositionOptimizer(float step_size, VectorXf noise, float gradient_step_size,Eigen::Vector4f peq, pcl::PointCloud<pcl::PointXYZ> pc) : 
  GradientFreeRandomizedGradientOptimizer(step_size, noise, gradient_step_size)
  {
    plane_equation = peq;
    plane_cloud = pc;


    pointsInPlane = pc;
    pointsInPlane.clear();

    // plane constants 
    a = plane_equation(0);
    b = plane_equation(1);
    c = plane_equation(2);
    d = plane_equation(3);
    faux_z_axis << a,b,c;
    faux_z_axis.normalize();
    // define plane-space origin
    origin << 0,0,-d/c;
    // define plane-space x axis
    faux_x_axis << 1, 0,(-d - a)/c;
    faux_x_axis = (faux_x_axis - origin);
    faux_x_axis.normalize();

    faux_y_axis = cos(M_PI/2)*faux_x_axis + faux_z_axis*(faux_z_axis.dot(faux_x_axis))*(1 - cos(M_PI/2)) + faux_z_axis.cross(faux_x_axis) * sin(M_PI/2);


  }


#define PLANE_TOP .04
#define PLANE_BOTTOM -.04
#define IN_B_IN_P_REWARD 10.0
#define IN_B_ABOVE_P_REWARD 0.0
#define IN_B_BELOW_P_REWARD -10.0
#define OUT_B_IN_P_REWARD 0.0

  float TablePositionOptimizer::evaluate(Eigen::VectorXf x)
  { //std::cout << "called evaluate" << std::endl;
    float theta,phi;

    theta = x(2);
    //std::cout << "finished evaluate" << std::endl;
    // find coordinates of bottom-right corner of estimate
    Eigen::Vector3f br,tr,bl,tl,lenVect,widVect;


    br = x(0)*faux_x_axis + origin + x(1)*faux_y_axis;

    // br + lenvect = tr
    lenVect = cos(theta)*faux_x_axis + faux_z_axis*(faux_z_axis.dot(faux_x_axis))*(1 - cos(theta)) + faux_z_axis.cross(faux_x_axis) * sin(theta);
    phi = theta + M_PI/2;
    widVect = cos(phi)*faux_x_axis + faux_z_axis*(faux_z_axis.dot(faux_x_axis))*(1 - cos(phi)) + faux_z_axis.cross(faux_x_axis) * sin(phi);
    lenVect.normalize();
    widVect.normalize();

    tr = br + TABLE_LENGTH * lenVect;
    bl = br + TABLE_WIDTH * widVect;
    tl = tr + TABLE_WIDTH * widVect;

    // iterate through all points, find ones in current rectangle
    float score = 0;
    float d;
    pcl::PointXYZ point;
    Eigen::Vector3f point_projected;
    for (int i = 0; i < plane_cloud.points.size(); i++)
    { 
      point = plane_cloud.points[i];
      d = dist_from_plane(point, plane_equation);
      point_projected = project_to_plane(point, plane_equation);
      bool in = in_rectangle(br,tr,tl,bl,point_projected);
      if (in)
      { 
        if (d < PLANE_TOP && d > PLANE_BOTTOM) // point is in plane
          score = score + IN_B_IN_P_REWARD;
        else if (d >= PLANE_TOP) // point is above plane
          score = score + IN_B_ABOVE_P_REWARD;
        else // point is below plane 
          score = score + IN_B_BELOW_P_REWARD;
      }
      else
      { //std::cout <<"out"<<std::endl;
        if (d < PLANE_TOP && d > PLANE_BOTTOM) // in plane
          score = score + OUT_B_IN_P_REWARD;
      }
    }
    //std::cout<< score<< std::endl;
    return -1 * score;

  }

  void TablePositionOptimizer::FillPointsInPlane(Eigen::VectorXf x)
  {
    float theta,phi;

    theta = x(2);
    //std::cout << "finished evaluate" << std::endl;
    // find coordinates of bottom-right corner of estimate
    Eigen::Vector3f br,tr,bl,tl,lenVect,widVect;


    br = x(0)*faux_x_axis + origin + x(1)*faux_y_axis;

    // br + lenvect = tr
    lenVect = cos(theta)*faux_x_axis + faux_z_axis*(faux_z_axis.dot(faux_x_axis))*(1 - cos(theta)) + faux_z_axis.cross(faux_x_axis) * sin(theta);
    phi = theta + M_PI/2;
    widVect = cos(phi)*faux_x_axis + faux_z_axis*(faux_z_axis.dot(faux_x_axis))*(1 - cos(phi)) + faux_z_axis.cross(faux_x_axis) * sin(phi);
    lenVect.normalize();
    widVect.normalize();

    tr = br + TABLE_LENGTH * lenVect;
    bl = br + TABLE_WIDTH * widVect;
    tl = tr + TABLE_WIDTH * widVect;


    float d;
    pcl::PointXYZ point;
    Eigen::Vector3f point_projected;
    for (int i = 0; i < plane_cloud.points.size(); i++)
    { 
      point = plane_cloud.points[i];
      d = dist_from_plane(point, plane_equation);
      point_projected = project_to_plane(point, plane_equation);
      bool in = in_rectangle(br,tr,tl,bl,point_projected);
      if (in)
      { if (d < PLANE_TOP && d > PLANE_BOTTOM) 
          { pointsInPlane.points.push_back(point);
            //std::cout<<"filling"<<std::endl;
          }
      }
    }

    pointsInPlane.width = pointsInPlane.points.size();
    pointsInPlane.height = 1;
    pointsInPlane.is_dense = true;




    return;
  }

  std::vector<pcl::PointXYZ> TablePositionOptimizer::corners(Eigen::Vector3f x)
  {
    float theta,phi;

    theta = x(2);
    std::vector<pcl::PointXYZ> output;
    pcl::PointXYZ brp,trp,tlp,blp;
    Eigen::Vector3f br,tr,tl,bl,lenVect,widVect;

    br = x(0)*faux_x_axis + origin + x(1)*faux_y_axis;

    lenVect = cos(theta)*faux_x_axis + faux_z_axis*(faux_z_axis.dot(faux_x_axis))*(1 - cos(theta)) + faux_z_axis.cross(faux_x_axis) * sin(theta);
    phi = theta + M_PI/2;
    widVect = cos(phi)*faux_x_axis + faux_z_axis*(faux_z_axis.dot(faux_x_axis))*(1 - cos(phi)) + faux_z_axis.cross(faux_x_axis) * sin(phi);
    lenVect.normalize();
    widVect.normalize();

    tr = br + TABLE_LENGTH * lenVect;
    bl = br + TABLE_WIDTH * widVect;
    tl = tr + TABLE_WIDTH * widVect;

    brp.x = br(0);
    brp.y = br(1);
    brp.z = br(2);

    blp.x = bl(0);
    blp.y = bl(1);
    blp.z = bl(2);

    trp.x = tr(0);
    trp.y = tr(1);
    trp.z = tr(2);

    tlp.x = tl(0);
    tlp.y = tl(1);
    tlp.z = tl(2);

    output.push_back(brp);
    output.push_back(trp);
    output.push_back(tlp);
    output.push_back(blp);

    return output;

  }




