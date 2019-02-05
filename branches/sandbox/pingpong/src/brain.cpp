#include <signal.h>
#include <string>
#include <bingham/util.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <fastwam/MoveTo.h>
#include <fastwam/GainTrajectory.h>
#include <fastwam/SwingData.h>
#include <pingpong/BallState.h>
#include <pingpong/HitPolicies.h>
#include <pingpong/HitPlan.h>
#include "dynamics.h"
#include "kinematics.h"






#define TABLE_WIDTH 1.525

//dbug
//double ja0[7] = { 0.0133,    1.3863,   -0.6662,   -1.0558,   -0.7243,    0.7522,   -0.5249};  // p=[.25,.7,.3], n=[0,0,1]
//double ja0[7] = {0.1719, -0.1166, 0.4356, 1.9674, -2.0583, -0.7924, 0.1701};  // p = [-.25, .7, .2]

double ja0[7] = {0.1864, -0.1043, 0.4233, 1.9249, -2.0697, -0.8967, 0.1346};  // 01/31/14
double p0[6] = {-.25, .7, .2, 1, 0, 0};

//double ja0[7] = {0.1267, -0.1779, 0.3341, 2.0026, -2.1035, -0.9358, 0.1372};  // p = [-.25, .7, .25]
//double ja0[7] = {0.3242, -0.1516, 1.0775, 2.0121, -1.8514, -0.0675, 0.3988};  // p = [-.5, .7, .1]
//double ja0[7] = {0.2928, -0.4401, 0.7191, 2.1214, -1.8867, -0.5534, 0.4306};  // p = [-.5, .7, .2]
//double ja0[7] = {0.2244, -0.5675, 0.8277, 2.1011, -1.8128, -0.5716, 0.5138};  // p = [-.575, .7, .2]

ros::Publisher wam_moveto_pub;
ros::Publisher wam_gain_traj_pub;
ros::Publisher paddle_plan_pub;

std::string data_folder;
bool use_swing_table = true;
swing_table_t swing_table;
//double last_swing_time = 0;
double last_p_hit[6];
swing_torques_t last_torques;
int last_hit_policy_index = -1;
bool swinging = false;
double swinging_hit_time = 0.0;
double training_swing_start = 0.0;
double training_swing_duration = 10.0;  // seconds to ignore ball messages (so the arm doesn't swing at itself!)

int hit_policies_length = 0;
double **hit_policies_in = NULL;
double **hit_policies_out = NULL;

double **data_points = NULL;
int num_data_points = 0;

double random_normal[3];
double random_vel = 0;

pingpong::BallState last_ball;
/*
bool have_ball_spin = false;
double ball_spin[4];
bool preparing_swing = false;
int swing_spin = 0;
*/

bool have_swing_plan = 0;
double **swing_plan_P;
double **swing_plan_Q;
double **swing_plan_U;
int swing_plan_NT;
double swing_plan_t0;
double swing_plan_dt;


void ros_sleep(int seconds)
{
  for (int j = 0; j < seconds; j++) {
    ros::spinOnce();
    sleep(1);
  }
}


void moveto(double *ja)
{
  fastwam::MoveTo msg;
  for (int j = 0; j < 7; j++)
    msg.joint_angles.push_back(ja[j]);
  wam_moveto_pub.publish(msg);
}


void gain_swing(double *gains, double t0, double dt, int n)
{
  fastwam::GainTrajectory msg;
  int m = 7*15;

  for (int i = 0; i <= n; i++)
    msg.times.push_back(t0 + dt*i);

  for (int i = 0; i < n*m; i++)
    msg.gains.push_back(gains[i]);

  wam_gain_traj_pub.publish(msg);
}


void publish_paddle_plan(double **P, int n, double t0)
{
  // note the change in coordinate frame (for eyes.cpp)
  sensor_msgs::PointCloud msg;
  msg.header.stamp = ros::Time(t0);
  msg.header.frame_id = "pingpongtable";
  msg.points.resize(n);
  for (int i = 0; i < n; i++) {
    msg.points[i].x = TABLE_WIDTH - P[i][1];
    msg.points[i].y = P[i][0];
    msg.points[i].z = P[i][2];
  }

  paddle_plan_pub.publish(msg);
}


void store_swing_plan(double **P, double **Q, double **U, int n, double t0, double dt)
{
  if (have_swing_plan) {
    free_matrix2(swing_plan_P);
    free_matrix2(swing_plan_Q);
    free_matrix2(swing_plan_U);
  }

  swing_plan_P = matrix_clone(P,n,6);
  swing_plan_Q = matrix_clone(Q,n,7);
  swing_plan_U = matrix_clone(U,n,7);
  swing_plan_NT = n;
  swing_plan_t0 = t0;
  swing_plan_dt = dt;
  have_swing_plan = true;
}


void save_swing_plan(char *base_name)
{
  if (!have_swing_plan) {
    printf("Warning: tried to save a swing plan that doesn't exist!\n");
    return;
  }
  printf("Saving swing plan\n");

  int n = swing_plan_NT;

  char fname[1024];
  sprintf(fname, "%s_paddle.txt", base_name);
  save_matrix(fname, swing_plan_P, n, 6);
  sprintf(fname, "%s_joints.txt", base_name);
  save_matrix(fname, swing_plan_Q, n, 7);
  sprintf(fname, "%s_torques.txt", base_name);
  save_matrix(fname, swing_plan_U, n, 7);

  double **T = new_matrix2(n,1);
  T[0][0] = swing_plan_t0;
  for (int i = 1; i < n; i++)
    T[i][0] = T[i-1][0] + swing_plan_dt;
  sprintf(fname, "%s_times.txt", base_name);
  save_matrix(fname, T, n, 1);
  free_matrix2(T);

  free_matrix2(swing_plan_P);
  free_matrix2(swing_plan_Q);
  free_matrix2(swing_plan_U);
  have_swing_plan = false;
}


typedef struct {
  double v;  // final velocity
  double a;  // acceleration (-a up to n0, +a after n0)
  int n0;
} bang_bang_t;

// assumes v0 = 0 and sign(x1-x0) == sign(v1)
bang_bang_t compute_bang_bang_controller(double x0, double x1, double v1, int n, double dt)
{
  // x-trajectory: pick the i that leads to the closest v to hit.vel
  //n = 25
  //i = 0:12
  //a = x./((i.^2 - (2*n+1)*i + n*(n+1)/2)*dt^2)
  //v = a.*((n-2*i)*dt)

  bang_bang_t bbc;
  bbc.v = 0.0;
  bbc.a = 0.0;
  bbc.n0 = -1;

  printf("n = %d, v_des = %.2f, v = [", n, v1);
  double dx = x1 - x0;
  for (int i = 0; i < n; i++) {
    double a = dx/((i*i - (2*n+1)*i + n*(n+1)/2.0)*dt*dt);
    double vx = (n-2*i)*a*dt;
    if (bbc.n0 < 0 || (fabs(vx - v1) < fabs(bbc.v - v1))) {
      bbc.v = vx;
      bbc.a = a;
      bbc.n0 = i;
    }
    printf("%.2f ", vx);
  }
  printf("] --> v = %.2f, a = %.2f, n0 = %d\n", bbc.v, bbc.a, bbc.n0);

  return bbc;
}


double **plan_smooth_paddle_path(int *plan_length, double *dt_ptr, hit_t hit, double t0)
{
  double dt = .01; //.02;
  int n = round((hit.t - t0)/dt);  // hit time step

  // bang-bang paddle swing params
  int n1 = n-20;  // reach the paddle normal goal
  int n2 = n+5;  // follow-through
  int n3 = n+15;  // stop

  bang_bang_t bbc[3];
  double a2[3];
  for (int i = 0; i < 3; i++) {
    bbc[i] = compute_bang_bang_controller(p0[i], hit.pos[i], hit.vel[i], n, dt);
    a2[i] = -bbc[i].v / ((n3-n2)*dt);  // slow-down dacceleration after the follow-through
  }

  // plan a paddle path
  double **P = new_matrix2(n3,6);
  memcpy(&P[0][0], p0, 3*sizeof(double));
  memcpy(&P[0][3], hit.normal, 3*sizeof(double));
  double v[3] = {0., 0., 0.};
  for (int i = 1; i < n3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i <= bbc[j].n0)
	v[j] -= bbc[j].a*dt;
      else if (i <= n)
	v[j] += bbc[j].a*dt;
      else if (i > n2)
	v[j] += a2[j]*dt;
    }

    double dp[3] = {v[0]*dt, v[1]*dt, v[2]*dt};
    add(P[i], P[i-1], dp, 3);

    if (i < n1) {
      double w2 = i/(double)n1;
      double w1 = 1.0 - w2;
      for (int j = 3; j < 6; j++)
	P[i][j] = w1*p0[j] + w2*hit.normal[j];
      normalize(&P[i][3], &P[i][3], 3);
    }
    else
      memcpy(&P[i][3], hit.normal, 3*sizeof(double));
  }

  printf("P[0] = (%.2f, %.2f, %.2f)\n", P[0][0], P[0][1], P[0][2]);
  printf("P[n=%d] = (%.2f, %.2f, %.2f)\n", n, P[n][0], P[n][1], P[n][2]);
  printf("P[n2=%d] = (%.2f, %.2f, %.2f)\n", n2, P[n2][0], P[n2][1], P[n2][2]);
  printf("P[n3-1=%d] = (%.2f, %.2f, %.2f)\n", n3-1, P[n3-1][0], P[n3-1][1], P[n3-1][2]);

  *plan_length = n3;
  *dt_ptr = dt;
  return P;
}


double **paddle_path_to_joint_path(double **P, int NT)
{
  // lookup every 5th Q(i,:), and interpolate the rest
  double **Q = new_matrix2(NT,7);
  memcpy(Q[0], ja0, 7*sizeof(double));
  int i;
  for (i = 5; i < NT; i+=5) {
    double *q0 = (i>0 ? Q[i-5] : ja0);
    double *q = paddle_to_joints(P[i], q0);
    memcpy(Q[i], q, 7*sizeof(double));
    free(q);
  }
  if (NT%5 != 1) {  // compute Q[NT-1], if necessary
    double *q = paddle_to_joints(P[NT-1], Q[i-5]);
    memcpy(Q[NT-1], q, 7*sizeof(double));
    free(q);    
  }
  for (i = 1; i < NT-1; i++) {  // interpolate
    int a = i % 5;
    if (a) {
      int i0 = i-a;
      int i1 = MIN(i0+5, NT-1);
      int b = i1-i;
      double w1 = b/(double)(a+b);
      double w2 = 1 - w1;
      for (int j = 0; j < 7; j++)
	Q[i][j] = w1*Q[i0][j] + w2*Q[i1][j];
    }
  }

  //dbug
  //for (i = NT-15; i < NT; i++)
  //  printf("Q[%d] = (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f)\n",
  //	   i, Q[i][0], Q[i][1], Q[i][2], Q[i][3], Q[i][4], Q[i][5], Q[i][6]);

  return Q;
}


double **compute_trajectory_velocity(double **Q, int n, int j, double dt)
{
  double **dQ = new_matrix2(n,j);
  for (int i = 1; i < n; i++)
    sub(dQ[i], Q[i], Q[i-1], j);
  mult(dQ[0], dQ[0], 1.0/dt, n*j);

  return dQ;
}


double ***compute_swing_gains(double **Q, double **U, int NT, double dt)
{
  double **dQ = compute_trajectory_velocity(Q, NT, 7, dt);

  double kp[7] = {500, 200, 200, 400, 10, 2, 2};  // joint position gains
  mult(kp, kp, .5, 7);
  double kd[7] = {2, 2, 2, 1, .5, .1, .1};

  double ***gains = new_matrix3(NT,7,15);
  for (int i = 0; i < NT; i++) {
    for (int j = 0; j < 7; j++) {
      gains[i][j][0] = U[i][j] + kp[j]*Q[i][j] + kd[j]*dQ[i][j];
      gains[i][j][j+1] = -kp[j];
      gains[i][j][j+8] = -kd[j];
    }
  }

  free_matrix2(dQ);

  return gains;
}


/*
 * Execute a smooth swing from ja0 to (hit.pos, hit.normal, hit.vel) and from time t0 to hit.t
 */
void smooth_swing(hit_t hit, double t0, bool training=false)
{
  double t = ros::Time::now().toSec();  //dbug

  // plan forward trajectory
  int NT;
  double dt;
  double **P = plan_smooth_paddle_path(&NT, &dt, hit, t0);
  double **Q = paddle_path_to_joint_path(P, NT);

  // make forward and backward trajectory
  double **P2 = new_matrix2(2*NT, 6);
  double **Q2 = new_matrix2(2*NT, 7);
  //double **P2 = new_matrix2(NT+NT/2, 6);
  //double **Q2 = new_matrix2(NT+NT/2, 7);
  memcpy(P2[0], P[0], NT*6*sizeof(double));
  memcpy(Q2[0], Q[0], NT*7*sizeof(double));
  for (int i = 0; i < NT; i++)
    memcpy(P2[NT+i], P[NT-i-1], 6*sizeof(double));
  for (int i = 0; i < NT; i++)
    memcpy(Q2[NT+i], Q[NT-i-1], 7*sizeof(double));
  //for (int i = 0; i < NT/2; i++)
  //  memcpy(P2[NT+i], P[NT-2*i-1], 6*sizeof(double));
  //for (int i = 0; i < NT/2; i++)
  //  memcpy(Q2[NT+i], Q[NT-2*i-1], 7*sizeof(double));
  free_matrix2(P);
  free_matrix2(Q);
  P = P2;
  Q = Q2;
  NT = 2*NT;
  //NT = NT+NT/2;

  // get swing torques / gravity torques
  double **U;
  if (use_swing_table && !training) {
    swing_torques_t torques;
    if (swinging)
      torques = last_torques;
    else {
      torques = get_swing_torques(&swing_table, hit, t0);
      last_torques = torques;
    }
    if (torques.n > 0) {
      U = torques.U;
      if (torques.n < NT)
	printf("ERROR: torques.n < NT!\n");
    }
    else
      U = get_gravity_torques_for_trajectory(Q, NT);
  }
  else
    U = get_gravity_torques_for_trajectory(Q, NT);

  // get gain matrices
  double ***gains = compute_swing_gains(Q, U, NT, dt);

  double t2 = ros::Time::now().toSec();
  printf("planning time = %.2f ms\n", 1000*(t2 - t));  //dbug

  memcpy(last_p_hit, hit.pos, 3*sizeof(double));
  if (t2 > t0-.01)
    swinging = true;

  gain_swing(gains[0][0], t0, dt, NT);

  //dbug: record swing plan
  publish_paddle_plan(P, NT, t0);
  store_swing_plan(P, Q, U, NT, t0, dt);

  //cleanup
  free_matrix2(P);
  free_matrix2(Q);
  if (!use_swing_table)
    free_matrix2(U);
  free_matrix3(gains);
}


void ball_state_callback(const pingpong::BallState &msg)
{
  last_ball = msg;
  //have_ball_spin = true;
  //ball_spin[0] = msg.sw;
  //ball_spin[1] = msg.sx;
  //ball_spin[2] = msg.sy;
  //ball_spin[3] = msg.sz;
}


void load_hit_policies()
{
  int n, m;
  char fname[1024];
  sprintf(fname, "%s/hit_policies_in.txt", data_folder.c_str());
  hit_policies_in = load_matrix(fname, &n, &m);
  sprintf(fname, "%s/hit_policies_out.txt", data_folder.c_str());
  hit_policies_out = load_matrix(fname, &n, &m);
  hit_policies_length = n;
}


void hit_policies_callback(const pingpong::HitPolicies &msg)
{
  printf("Got HitPolicies message\n");

  if (hit_policies_length > 0) {
    free_matrix2(hit_policies_in);
    free_matrix2(hit_policies_out);
  }

  const int d_in = 8;
  const int d_out = 9;
  hit_policies_length = msg.ball_bounce_params.size() / d_in;
  hit_policies_in = new_matrix2(hit_policies_length, d_in);
  hit_policies_out = new_matrix2(hit_policies_length, d_out);

  for (int i = 0; i < d_in * hit_policies_length; i++)
    hit_policies_in[0][i] = msg.ball_bounce_params[i];

  for (int i = 0; i < d_out * hit_policies_length; i++)
    hit_policies_out[0][i] = msg.hit_params[i];
}


void get_ball_bounce_params(double *bb_params, const sensor_msgs::PointCloud &msg)
{
  // find lowest point in msg
  double lowest_z = msg.points[0].z;
  int index = 0;
  for (int i = 1; i < msg.points.size(); i++) {
    if (msg.points[i].z < lowest_z) {
      lowest_z = msg.points[i].z;
      index = i;
      
      // local minima
      if (msg.points[i-1].z > lowest_z && msg.points[i+1].z > lowest_z)
	break;
    }
  }

  bb_params[0] = msg.points[index].y;
  bb_params[1] = TABLE_WIDTH - msg.points[index].x;

  bb_params[2] = (msg.points[index-1].y - msg.points[index-2].y) / 0.01;
  bb_params[3] = (-msg.points[index-1].x + msg.points[index-2].x) / 0.01;
  bb_params[4] = (msg.points[index-1].z - msg.points[index-2].z) / 0.01;

  bb_params[5] = last_ball.wy;
  bb_params[6] = -last_ball.wx;
  bb_params[7] = last_ball.wz;
}


int find_highest_trajectory_point(const sensor_msgs::PointCloud &ball_trajectory, double x_thresh)
{
  // find the ball's highest point near the end of the table
  double zmax = -1.0;
  int ihit = -1;
  for (uint i = 0; i < ball_trajectory.points.size(); i++) {
    double x = fabs(ball_trajectory.points[i].y);
    if (x > x_thresh)
      continue;
    double z = ball_trajectory.points[i].z;
    if (z > zmax) {
      zmax = z;
      ihit = i;
    }
  }

  return ihit;
}


// returns true iff policy found
bool get_hit_policy_params(hit_t &hit, const sensor_msgs::PointCloud &ball_trajectory)
{
  int imin = -1;

  double bb_params[8];
  get_ball_bounce_params(bb_params, ball_trajectory);

  if (swinging)
    imin = last_hit_policy_index;

  else {
    //double bb_thresh = 10000.0;
    //double bkts = 5;
    //double bb_thresh[8] = {0.6/bkts, 0.6/bkts, 1.5/bkts, 2.0/bkts, 1.0/bkts, 1000000, 200, 1000000};
    double bb_thresh[8] = {0.2, 0.2, 1.0, 1.0, 1.0, 1000000, 200, 1000000};


    // find the closest policy to bb_params
    double d2min = 1000000000.0;
    for (int i=0; i < hit_policies_length; i++) {
      bool close = true;

      /*dbug
      printf("hit_policies_in[%d] = (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f)\n",
	     i, hit_policies_in[i][0], hit_policies_in[i][1], hit_policies_in[i][2], hit_policies_in[i][3], 
	     hit_policies_in[i][4], hit_policies_in[i][5], hit_policies_in[i][6], hit_policies_in[i][7]);
      printf("bb_params =          (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f)\n", bb_params[0],
	     bb_params[1], bb_params[2], bb_params[3], bb_params[4], bb_params[5], bb_params[6], bb_params[7]);
      */

      double db[8];
      for (int j=0; j < 8; j++) {
	db[j] = fabs(hit_policies_in[i][j] - bb_params[j]);
	if (db[j] > bb_thresh[j]) {
	  //printf(" --> j=%d is too far!\n", j); //dbug
	  close = false;
	  break;
	}
      }

      if (close) {
	double d2 = 0.0;
	for (int j = 0; j < 5; j++)
	  d2 += db[j]*db[j] / (bb_thresh[j] * bb_thresh[j]);
	if (d2 < d2min) {
	  imin = i;
	  d2min = d2;
	}
      }
    }
  }

  last_hit_policy_index = imin;

  if (imin < 0)
    return false;

 // int imin = 0;
 //  double d2min = dist2(bb_params, hit_policies_in[0], 8);
 //  for (int i = 1; i < hit_policies_length; i++) {
 //    double d2 = dist2(bb_params, hit_policies_in[i], 8);
 //    if (d2 < d2min) {
 //      d2min = d2;
 //      imin = i;
 //    }
 //  }

 //  if (d2min > bb_thresh)
 //    return false;

  double *hit_params = hit_policies_out[imin];

  //double hit_x = hit_params[0];

  // find hit x (highest point shifted by hit_params[0])
  int high_idx = find_highest_trajectory_point(ball_trajectory, .15);
  double high_x = ball_trajectory.points[high_idx].y;
  double hit_x = high_x + hit_params[0];

  // find the ball's closest point to hit_x
  int ihit = -1;
  double dmin = 100000000.;
  for (uint i = 0; i < ball_trajectory.points.size(); i++) {
    double d = fabs(hit_x - ball_trajectory.points[i].y);
    if (d < dmin) {
      dmin = d;
      ihit = i;
    }
  }
  double dt = .01;
  hit.t = ball_trajectory.header.stamp.toSec() + dt*ihit;

  hit.pos_offset[0] = hit_params[0];
  hit.pos_offset[1] = hit_params[1];
  hit.pos_offset[2] = hit_params[2];

  hit.pos[0] = ball_trajectory.points[ihit].y;
  hit.pos[1] = hit_params[1] + TABLE_WIDTH - ball_trajectory.points[ihit].x;
  hit.pos[2] = hit_params[2] + ball_trajectory.points[ihit].z;
  hit.vel[0] = hit_params[3];
  hit.vel[1] = hit_params[4];
  hit.vel[2] = hit_params[5];
  hit.normal[0] = hit_params[6];
  hit.normal[1] = hit_params[7];
  hit.normal[2] = hit_params[8];
  normalize(hit.normal, hit.normal, 3);

  printf("Hit params (%.2f, %.2f, %.2f)\n",
	 hit_params[0], hit_params[1], hit_params[2]);
  
  printf("Using hit policy (t_hit=%.3f, t=%.3f, t_hit-t=%.3f) (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f)\n",
	 hit.t, ros::Time::now().toSec(), hit.t - ros::Time::now().toSec(),
	 hit.pos[0], hit.pos[1], hit.pos[2], hit.vel[0], hit.vel[1], hit.vel[2], hit.normal[0],
	 hit.normal[1], hit.normal[2]); //dbug

  return true;
}


bool interpolate_trajectory(double *p, double *v, const sensor_msgs::PointCloud &traj, double t)
{
  double dt = .01;
  double t0 = traj.header.stamp.toSec();
  int n = traj.points.size();
  if (t < t0) {
    p[0] = traj.points[0].x;
    p[1] = traj.points[0].y;
    p[2] = traj.points[0].z;
    if (v) {
      v[0] = (traj.points[1].x - traj.points[0].x) / dt;
      v[1] = (traj.points[1].y - traj.points[0].y) / dt;
      v[2] = (traj.points[1].z - traj.points[0].z) / dt;
    }
    return false;
  }
  else if (t >= t0 + (n-1)*dt) {
    p[0] = traj.points[n-1].x;
    p[1] = traj.points[n-1].y;
    p[2] = traj.points[n-1].z;
    if (v) {
      v[0] = (traj.points[n-1].x - traj.points[n-2].x) / dt;
      v[1] = (traj.points[n-1].y - traj.points[n-2].y) / dt;
      v[2] = (traj.points[n-1].z - traj.points[n-2].z) / dt;
    }
    return false;
  }

  int i = floor((t - t0) / dt);
  double w1 = (t - t0)/dt - i;
  double w0 = 1.0 - w1;

  p[0] = w0*traj.points[i].x + w1*traj.points[i+1].x;
  p[1] = w0*traj.points[i].y + w1*traj.points[i+1].y;
  p[2] = w0*traj.points[i].z + w1*traj.points[i+1].z;

  if (v) {
    v[0] = (traj.points[i+1].x - traj.points[i].x) / dt;
    v[1] = (traj.points[i+1].y - traj.points[i].y) / dt;
    v[2] = (traj.points[i+1].z - traj.points[i].z) / dt;
  }

  return true;
}


void adjust_swing(const sensor_msgs::PointCloud &ball_trajectory)
{
  double t0 = ros::Time::now().toSec();

  // find out when current plan's x-trajectory will meet ball's x-trajectory
  double paddle_hit_pos[3];
  for (int i = 1; i < swing_plan_NT/2; i++) {
    double t = swing_plan_t0 + i*swing_plan_dt;

    // interpolate to find the ball's x-position at time t
    double ball_pos[3], ball_vel[3];
    if (!interpolate_trajectory(ball_pos, ball_vel, ball_trajectory, t))
      continue;
    double bx = ball_pos[1];
    double bvx = ball_vel[1];

    double px = swing_plan_P[i][0];
    if (px >= bx) {  // swing intercepts ball between i-1 and i
      double pvx = (px - swing_plan_P[i-1][0]) / swing_plan_dt;
      double t_hit = t - (px-bx)/(pvx-bvx);

      // interpolate to get ball's position at t_hit
      interpolate_trajectory(ball_pos, NULL, ball_trajectory, t_hit);
      bx = ball_pos[1];
      double by = TABLE_WIDTH - ball_pos[0];
      double bz = ball_pos[2];

      // interpolate to get paddle position at t_hit
      double w0 = (t - t_hit) / swing_plan_dt;
      double w1 = 1.0 - w0;
      px = w0*swing_plan_P[i-1][0] + w1*swing_plan_P[i][0];
      double py = w0*swing_plan_P[i-1][1] + w1*swing_plan_P[i][1];
      double pz = w0*swing_plan_P[i-1][2] + w1*swing_plan_P[i][2];

      // adjust paddle's (y,z)-trajectory in the direction of (by-py, bz-pz)
      double dy = by - py;
      double dz = bz - pz;
      if (fabs(dy) < .01)  // dead zone
	dy = 0.0;
      if (fabs(dz) < .01)  // dead zone
	dz = 0.0;
      if (dy==0.0 && dz==0.0)
	return;

      printf(" --> adjust paddle trajectory (dy,dz) = (%.4f, %.4f)\n", dy, dz);  //dbug

      double max_adjust = .01;
      dy = MAX(MIN(dy, max_adjust), -max_adjust);
      dz = MAX(MIN(dz, max_adjust), -max_adjust);

      // adjust smoothly from current time to hit time (j0 to j1)
      int j0 = floor((t0 - swing_plan_t0) / swing_plan_dt) + 5;
      j0 = MAX(j0, 0);
      int j1 = floor((t_hit - swing_plan_t0) / swing_plan_dt) - 5;
      int j2 = MAX(swing_plan_NT/2, j1+15);
      j2 -= (j2-j0)%5;

      printf("j0=%d, j1=%d, j2=%d\n", j0, j1, j2); //dbug

      // convert paddle adjustment to joint adjustment
      double dp[6] = {0, dy, dz, 0, 0, 0};
      double w[7] = {1,1,1,1,2,2,2};
      double *dq = paddle_to_joints_displacement_weighted(swing_plan_Q[i], dp, w);
      for (int j = j0; j <= j2; j++) {
	double c = (j < j1 ? (j-j0)/(double)(j1-j0) : 1.0);
	for (int k = 0; k < 7; k++)
	  swing_plan_Q[j][k] += c*dq[k];
	double *p = joints_to_paddle(swing_plan_Q[j]);
	memcpy(swing_plan_P[j], p, 6*sizeof(double));
	free(p);
      }      
      free(dq);

      /* adjust joint angles every 5 timesteps
      double dp[6] = {0, dy, dz, 0, 0, 0};
      for (int j = j0; j <= j2; j+=5) {
	double c = (j < j1 ? (j-j0)/(double)(j1-j0) : 1.0);
	swing_plan_P[j][1] += c*dy;
	swing_plan_P[j][2] += c*dz;
	double w[7] = {1,1,1,1,2,2,2};
	double *dq = paddle_to_joints_displacement_weighted(swing_plan_Q[j], dp, w);
	add(swing_plan_Q[j], swing_plan_Q[j], dq, 7);
	free(dq);
      }
      // interpolate the rest
      for (int j = j0+1; j < j2; j++) {
	int a = j % 5;
	if (a) {
	  int i0 = j-a;
	  int i1 = i0+5;
	  int b = i1-j;
	  double w1 = b/(double)(a+b);
	  double w2 = 1 - w1;
	  for (int k = 0; k < 7; k++)
	    swing_plan_Q[j][k] = w1*swing_plan_Q[i0][k] + w2*swing_plan_Q[i1][k];
	}
      }
      */

      // send the new swing to the arm
      double ***gains = compute_swing_gains(swing_plan_Q, swing_plan_U, swing_plan_NT, swing_plan_dt);
      gain_swing(gains[0][0], swing_plan_t0, swing_plan_dt, swing_plan_NT);

      publish_paddle_plan(swing_plan_P, swing_plan_NT, swing_plan_t0);

      free_matrix3(gains);

      break;
    } 
  }

  printf("Adjusted swing in %.1f ms\n", 1000*(ros::Time::now().toSec() - t0)); //dbug
}


hit_t plan_hit(const sensor_msgs::PointCloud &ball_trajectory)
{
  hit_t hit;
  hit.t = 0.0;

  // don't swing if no ball is found or if ball is moving away from the robot
  int n = ball_trajectory.points.size();
  if (n < 5 || ball_trajectory.points[n-1].y > ball_trajectory.points[n-5].y || ball_trajectory.points[0].y < .5)
    return hit;

  // don't swing if the ball is going off the side of the table (or out of the robot's y-range)
  for (uint i = 0; i < ball_trajectory.points.size(); i++) {
    double y = TABLE_WIDTH - ball_trajectory.points[i].x;
    if (y < .3 || y > 1.0)
      return hit;
    double x = ball_trajectory.points[i].y;
    if (x < 0.0)
      break;
  }

  printf("------------------------------------\n");

  // look for an existing hit policy
  if (hit_policies_length > 0 && get_hit_policy_params(hit, ball_trajectory))
    return hit;
  // otherwise, plan a default hit

  printf("Planning hit policy\n"); //dbug

  // find the ball's highest point near the end of the table
  double zmax = -1.0;
  int ihit = -1;
  for (uint i = 0; i < ball_trajectory.points.size(); i++) {
    double x = fabs(ball_trajectory.points[i].y);
    if (x > .15)
      continue;
    double dx = (swinging ? fabs(ball_trajectory.points[i].y - last_p_hit[0]) : 0);
    double z = ball_trajectory.points[i].z - dx;  // penalty for dx
    if (z > zmax) {
      zmax = z;
      ihit = i;
    }
  }
  if (ihit < 0)  
    return hit;

  double dt = .01;  // from eyes.cpp
  hit.t = ball_trajectory.header.stamp.toSec() + dt*ihit;

  double t = ros::Time::now().toSec();
  printf("t_hit - t = %.3f, t - t_ball = %.3f\n", hit.t - t, t - ball_trajectory.header.stamp.toSec()); //dbug

  // desired paddle pose to hit the ball at time t_hit
  hit.pos[0] = ball_trajectory.points[ihit].y;
  hit.pos[1] = TABLE_WIDTH - ball_trajectory.points[ihit].x;
  hit.pos[2] = ball_trajectory.points[ihit].z;

  hit.pos[1] = MIN(MAX(hit.pos[1], .3), 1.0);
  hit.pos[2] = MIN(MAX(hit.pos[2], .1), .6);

  hit.pos_offset[0] = hit.pos_offset[1] = hit.pos_offset[2] = 0.0;

  double dx = ball_trajectory.points[ihit].y - ball_trajectory.points[ihit-1].y;
  double dy = ball_trajectory.points[ihit-1].x - ball_trajectory.points[ihit].x;
  double db[2] = {dx, dy};
  db[0] *= 2.0; // bias towards straight ahead
  normalize(db, db, 2);

  // only pick new random numbers if swinging is false
  if (!swinging) {
    for (int i = 0; i < 3; i++)
      random_normal[i] = 0.0; //.2*frand() - .1;
    random_vel = 0.0; //frand() - .5;
  }

  //hit.normal[0] = -db[0];
  //hit.normal[1] = -db[1];
  //hit.normal[2] = 0;
  hit.normal[0] = -db[0] + random_normal[0];
  hit.normal[1] = -db[1] + random_normal[1];
  hit.normal[2] = 0 + random_normal[2];
  normalize(hit.normal, hit.normal, 3);

  //TODO: plan this
  //hit.vel[0] = 1.3;
  //hit.vel[1] = 0;
  //hit.vel[2] = 0;
  hit.vel[0] = 1.3 + random_vel;
  hit.vel[1] = 0; // + frand() - .5;
  hit.vel[2] = 0; // + frand() - .5;
  
  return hit;
}


void predict_ball_hit(double b_final[3], double bv_in[3], hit_t hit, double **hit_model)
{
  // compute the relative ball velocity
  double u[3], x_axis[3] = {1,0,0};
  cross(u, hit.normal, x_axis);
  double theta = acos(dot(hit.normal, x_axis, 3));
  double q[4] = {cos(theta/2.0), sin(theta/2.0)*u[0], sin(theta/2.0)*u[1], sin(theta/2.0)*u[2]};
  double **R = new_matrix2(3,3);
  quaternion_to_rotation_matrix(R,q);
  double bv_rel_in[3];
  sub(bv_rel_in, bv_in, hit.vel, 3);
  matrix_vec_mult(bv_rel_in, R, bv_rel_in, 3, 3);

  double bv_rel_out[3];
  vec_matrix_mult(bv_rel_out, bv_rel_in, &hit_model[1], 3, 3);
  add(bv_rel_out, bv_rel_out, hit_model[0], 3);
  double bv_out[3];
  vec_matrix_mult(bv_out, bv_rel_out, R, 3, 3);
  add(bv_out, bv_out, hit.vel, 3);

  // simulate ball aerodynamics (without spin) until it falls to z=0
  double b[6] = {hit.pos[0], hit.pos[1], hit.pos[2], bv_out[0], bv_out[1], bv_out[2]};
  double dt = .02;
  double C = .1;
  double p = 1.204;
  double m = .0027;
  double g = 9.8;
  double r = .02;
  double A = M_PI*r*r;
  double Cd = -.5*C*p*A/m;
  for (int i = 0; i < 50; i++) {
    double *v = &b[3];
    double s = norm(v, 3);
    b[0] += dt*v[0];
    b[1] += dt*v[1];
    b[2] += dt*v[2];
    b[3] += dt*Cd*s*v[0];
    b[4] += dt*Cd*s*v[1];
    b[5] += dt*(Cd*s*v[2] - g);
    if (b[2] <= r)
      break;
  }
  memcpy(b_final, b, 6*sizeof(double));

  //cleanup
  free_matrix2(R);
}



hit_t improve_hit_plan_with_model(hit_t hit, const sensor_msgs::PointCloud &ball_trajectory)
{
  //TODO: get ball spin from eyes.cpp
  double topspin_hit_model_data[12] = {-0.9834, 0.6828, 2.2705,
				       -0.7118, 0.1379, 0.1306,
				       0.1301, 0.4379, -0.2235,
				       -0.1501, -0.0281, 0.8838};
  double underspin_hit_model_data[12] = {0.6470, 0.2219, 0.9182,
					 -0.3860, 0.1205, 0.1528,
					 -0.0673, 0.5997, 0.0691,
					 -0.1972, 0.0445, 0.3898};
  double **hit_model = new_matrix2_data(4,3,topspin_hit_model_data);
  //double **hit_model = new_matrix2_data(4,3,underspin_hit_model_data);

  double dt = .01;

  // find the ball velocity at collision time
  int ihit = -1;
  double dmin = 100000000.;
  for (uint i = 0; i < ball_trajectory.points.size(); i++) {
    double d = fabs(hit.pos[0] - ball_trajectory.points[i].y);
    if (d < dmin) {
      dmin = d;
      ihit = i;
    }
  }
  double bvx = (ball_trajectory.points[ihit].y - ball_trajectory.points[ihit-1].y) / dt;
  double bvy = (ball_trajectory.points[ihit-1].x - ball_trajectory.points[ihit].x) / dt;
  double bvz = (ball_trajectory.points[ihit].z - ball_trajectory.points[ihit-1].z) / dt;
  double bv_in[3] = {bvx, bvy, bvz};

  // optimize ball hit (over a small grid of paddle normals and velocities)
  hit_t new_hit = hit;
  hit_t min_hit = hit;
  double min_cost = 10000000.0;
  double min_vx, min_nz;
  double vx_step = .1;
  double nz_step = .1;
  for (int i = -5; i <= 5; i++) {
    for (int j = -1; j <= 1; j++) {
      new_hit.vel[0] = hit.vel[0] + i*vx_step;
      new_hit.normal[2] = hit.normal[2] + j*nz_step;
      normalize(new_hit.normal, new_hit.normal, 3);
      double b[6];
      predict_ball_hit(b, bv_in, new_hit, hit_model);

      double cost = fabs(b[0]-2.5);
      if (cost < min_cost) {
	min_cost = cost;
	min_hit = new_hit;
      }
    }
  }

  //cleanup
  free_matrix2(hit_model);

  //dbug
  printf("old hit: vx=%.2f, nz=%.2f,  improved hit: vx=%.2f, nz=%.2f\n",
	 hit.vel[0], hit.normal[2], min_hit.vel[0], min_hit.normal[2]);

  return min_hit;
}


// time, ball x, ball y, ball v, paddle x, paddle v, paddle n
void save_params(double time, hit_t hit, const sensor_msgs::PointCloud &msg)
{
  data_points[num_data_points][0] = time;

  get_ball_bounce_params(&data_points[num_data_points][1], msg);

  data_points[num_data_points][9] = hit.pos_offset[0];
  data_points[num_data_points][10] = hit.pos_offset[1];
  data_points[num_data_points][11] = hit.pos_offset[2];

  data_points[num_data_points][12] = hit.vel[0];
  data_points[num_data_points][13] = hit.vel[1];
  data_points[num_data_points][14] = hit.vel[2];

  data_points[num_data_points][15] = hit.normal[0];
  data_points[num_data_points][16] = hit.normal[1];
  data_points[num_data_points][17] = hit.normal[2];

  save_matrix("last_swing.txt", &data_points[num_data_points], 1, 18);

  num_data_points += 1;
}


void reload_swing_table_callback(const std_msgs::Empty &msg)
{
  if (use_swing_table) {
    swing_table = load_swing_table((char *)data_folder.c_str());
    printf("Reloaded swing table\n");
  }
}


void training_swing_callback(const std_msgs::Empty &msg)
{
  training_swing_start = ros::Time::now().toSec();
}


// predicted ball trajectory
void ball_trajectory_callback(const sensor_msgs::PointCloud &msg)
{
  double t = ros::Time::now().toSec();

  if (t < training_swing_start + training_swing_duration)
    return;

  //if (t > swinging_hit_time + .3)
  //  swinging = false;

  if (swinging) {
    double t_hit = swing_plan_t0 + .4;
    if (t_hit - t >= .2)
      adjust_swing(msg);
    else if (have_swing_plan)
      save_swing_plan((char *)"swing_plan");
    else
      return;
  }
  else {
    hit_t hit = plan_hit(msg);
    if (hit.t == 0.0)
      return;
    hit = improve_hit_plan_with_model(hit, msg);
    //double t0 = MIN(t + .03, hit.t - .5);
    double t0 = hit.t - .4;  //dbug
    smooth_swing(hit, t0);
    swinging_hit_time = hit.t;
    save_params(t, hit, msg);
  }

  /*
  hit_t hit = plan_hit(msg);
  if (hit.t == 0.0)
    return;
  double t0 = hit.t - .4;  //dbug
  if (t < hit.t - .3) {  //.38
    smooth_swing(hit, t0);
    swinging_hit_time = hit.t;
    save_params(t, hit, msg);
  }
  else {
    save_swing_plan((char *)"swing_plan");
  }
  */

  /*
  if (hit.t - t > .5)
    prepare_for_short_swing(hit.pos);  //TODO: pass in the hit_t

  if (hit.t - t > .4) {
    simple_swing_short(hit.pos, hit.t);  //TODO: pass in the hit_t
    last_swing_time = t;
  }
  */

  //dbug
  printf("ball x position = %.2f\n", msg.points[0].y);
  //printf("ball time = %.3f\n", msg.header.stamp.toSec());
  //printf("current time = %.3f\n", ros::Time::now().toSec());
}


void hit_plan_callback(const pingpong::HitPlan &msg)
{
  printf("Got HitPlan msg: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
	 msg.t, msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz, msg.nx, msg.ny, msg.nz); //dbug

  //dbug: temporarily disable ball callback
  training_swing_start = ros::Time::now().toSec();

  hit_t hit;
  hit.t = msg.t;
  hit.pos[0] = msg.x;
  hit.pos[1] = msg.y;
  hit.pos[2] = msg.z;
  hit.normal[0] = msg.nx;
  hit.normal[1] = msg.ny;
  hit.normal[2] = msg.nz;
  hit.vel[0] = msg.vx;
  hit.vel[1] = msg.vy;
  hit.vel[2] = msg.vz;
  //hit.pos_offset[0] = 0.0;
  //hit.pos_offset[1] = 0.0;
  //hit.pos_offset[2] = 0.0;

  double duration = hit.t;
  hit.t = ros::Time::now().toSec() + 1.0;
  double t0 = hit.t - duration;

  smooth_swing(hit, t0); //, true);
  //save_params(t, hit, msg);
  save_swing_plan((char *)"swing_plan");
}


void swingdata_callback(const fastwam::SwingData &msg)
{
  printf("Received swing msg with n=%d (%.2f secs)\n", msg.t.size(), msg.t.back() - msg.t.front());  //dbug

  moveto(ja0);

  //if (use_swing_torques)
  //  update_swing_torques(msg, last_p_hit);

  swinging = false;
}

void sigint_handler(int signum)
{
  save_matrix("data_points.txt", data_points, num_data_points, 15);
  exit(0);
}

int main(int argc, char *argv[])
{
  // init ROS
  ros::init(argc, argv, "pingpongbrain");
  ros::NodeHandle nh;
  ros::Subscriber sub_swingdata = nh.subscribe("/fastwam/swingdata", 1, swingdata_callback);
  ros::Subscriber ball_pred_sub = nh.subscribe("ball_trajectory_prediction", 1, ball_trajectory_callback);
  ros::Subscriber ball_state_sub = nh.subscribe("ball_state", 1, ball_state_callback);
  ros::Subscriber hit_policies_sub = nh.subscribe("hit_policies", 1, hit_policies_callback);
  ros::Subscriber hit_plan_sub = nh.subscribe("hit_plan", 1, hit_plan_callback);
  ros::Subscriber reload_swing_table_sub = nh.subscribe("reload_swing_table", 1, reload_swing_table_callback);
  ros::Subscriber training_swing_sub = nh.subscribe("training_swing", 1, training_swing_callback);
  wam_moveto_pub = nh.advertise<fastwam::MoveTo>("/fastwam/moveto", 1);
  wam_gain_traj_pub = nh.advertise<fastwam::GainTrajectory>("/fastwam/gain_trajectory", 1);
  paddle_plan_pub = nh.advertise<sensor_msgs::PointCloud>("paddle_plan", 1);

  signal(SIGINT, sigint_handler);

  if (nh.getParam("/brain/data_folder", data_folder))
    ROS_INFO("Got param 'data_folder': %s", data_folder.c_str());
  else
    ROS_ERROR("Failed to get param 'data_folder'");

  if (nh.getParam("/brain/use_swing_table", use_swing_table))
    ROS_INFO("Got param 'use_swing_table': %d", use_swing_table);
  else
    ROS_INFO("Failed to get param 'use_swing_table', setting default (TRUE)");

  if (use_swing_table) {
    swing_table = load_swing_table((char *)data_folder.c_str());
    printf("Loaded swing table\n");
  }

  sleep(3);
  moveto(ja0);
  ros_sleep(3);

  // time, ball params (5), hit params (9)
  
  data_points = new_matrix2(1000000, 15);

  ros::spin();

  return 0;
}
