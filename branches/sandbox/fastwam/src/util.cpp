#include <ros/ros.h>
#include <Eigen/Core>
#include <bingham/util.h>
#include "util.h"
#include "kinematics.h"

// constant variables
double CHECK_INTERVAL = 0.1;

// import most common Eigen types 
USING_PART_OF_NAMESPACE_EIGEN



//---------------- PID controller ----------------//

PID::PID()
{
  t = ros::Time::now().toSec();
  for (int i = 0; i < 7; i++) {
    P[i] = I[i] = torques[i] = 0;
    subgoal[i] = 0;
    reset[i] = true;
  }
  //gravity_compensation = false;
  resetVelocityLimits();
}

void PID::resetVelocityLimits()
{
  subgoal_vel_limits[0] = 1;
  subgoal_vel_limits[1] = 1;
  subgoal_vel_limits[2] = 1;
  subgoal_vel_limits[3] = 1;
  subgoal_vel_limits[4] = 2;
  subgoal_vel_limits[5] = 2;
  subgoal_vel_limits[6] = 4;
}

double *PID::update(double *joints)
{
  double t2 = ros::Time::now().toSec();
  double dt = t2-t;
  for (int i = 0; i < 7; i++) {

    if (reset[i]) {
      subgoal[i] = joints[i];
      reset[i] = false;
    }
    else {
      double new_subgoal = goal[i];
      if (new_subgoal > subgoal[i] + dt*subgoal_vel_limits[i])
	new_subgoal = subgoal[i] + dt*subgoal_vel_limits[i];
      else if (new_subgoal < subgoal[i] - dt*subgoal_vel_limits[i])
	new_subgoal = subgoal[i] - dt*subgoal_vel_limits[i];
      subgoal[i] = new_subgoal;
    }

    double err = subgoal[i] - joints[i];
    double D = (err - P[i])/dt;
    P[i] = err;
    I[i] += dt*err;

    double new_torque = pid_kp[i]*P[i] + pid_ki[i]*I[i] + pid_kd[i]*D;

    //if (gravity_compensation)
    //  new_torque += gravity_torques[i];
    
    if (new_torque > pid_torque_limits[i])
      new_torque = pid_torque_limits[i];
    else if (new_torque < -pid_torque_limits[i])
      new_torque = -pid_torque_limits[i];

    /*    
    double max_diff_torque = dt*pid_diff_torque_limits[i];
    if (new_torque > torques[i] + max_diff_torque)
      new_torque = torques[i] + max_diff_torque;
    else if (new_torque < torques[i] - max_diff_torque)
      new_torque = torques[i] - max_diff_torque;
    */
      
    torques[i] = new_torque;
  }
  t = t2;

  return torques;
}

void PID::setGoal(double *new_goal)
{
  for (int i = 0; i < 7; i++) {
    goal[i] = new_goal[i];
    //P[i] = I[i] = 0;
  }
}

void PID::setGoalForJoint(int joint, double new_goal)
{
  goal[joint] = new_goal;
}

void PID::resetJoint(int joint)
{
  I[joint] = 0;
  reset[joint] = true;
}

void PID::resetAllJoints()
{
  for (int i = 0; i < 7; i++)
    resetJoint(i);
}

/*
void PID::setGravityCompensation(bool gc)
{
  gravity_compensation = gc;

  if (gravity_compensation)
    memcpy(gravity_torques, torques, 7*sizeof(double));
}
*/

void PID::setVelocityLimit(int joint, double v)
{
  subgoal_vel_limits[joint] = v;
}


//--------------- GainTrajectoryController --------------//

GainTrajectoryController::GainTrajectoryController()
{
  traj_gains = NULL;
}

double *GainTrajectoryController::update(double *q, double *dq)
{
  //printf("GainTrajectoryController::update()\n"); //dbug

  // current time
  t = ros::Time::now().toSec();

  // compute index in the trajectory
  uint n;
  for (n = 0; n < traj_times.size(); n++)
    if (t < traj_times[n])
      break;

  // if time isn't within control tape, return null
  if (n == 0 || n == traj_times.size())
    return NULL;

  double x[15];
  x[0] = 1;
  memcpy(&x[1], q, 7*sizeof(double));
  memcpy(&x[8], dq, 7*sizeof(double));

  matrix_vec_mult(torques, traj_gains[n-1], x, 7, 15);

  for (int i = 0; i < 7; i++) {
    if (torques[i] > pid_torque_limits[i])
      torques[i] = pid_torque_limits[i];
    else if (torques[i] < -pid_torque_limits[i])
      torques[i] = -pid_torque_limits[i];
  }

  return torques;
}

void GainTrajectoryController::setGainTrajectory(const vector<double> &times, const vector<float> &gains)
{
  int N = times.size() - 1;

  // set trajectory times
  traj_times = times;

  // set gain matrices
  if (traj_gains)
    free_matrix3(traj_gains);
  traj_gains = new_matrix3(N, 7, 15);
  for (int i = 0; i < N*7*15; i++)
    traj_gains[0][0][i] = gains[i];
}

bool GainTrajectoryController::isStarted()
{
  return ros::Time::now().toSec() >= traj_times[0];
}

bool GainTrajectoryController::isFinished()
{
  double t = ros::Time::now().toSec();
  int n = traj_times.size() - 1;

  return (t >= traj_times[n]);
}



//--------------- PaddleOrientTrajectoryController --------------//

PaddleOrientTrajectoryController::PaddleOrientTrajectoryController(PID *pid)
{
  this->pid = pid;
}

double *PaddleOrientTrajectoryController::paddleOrientationToWristAngles(double *joint_angles, double *paddle_orientation)
{
  // use gradient descent to find a set of wrist joint angles that results in the desired paddle orientation
  
  double q[7];  // tmp joint angles
  double *w = &q[4];
  //double *w0 = &joint_angles[4];
  //double *ja = joint_angles;
  double w0[3] = {-2.07, -.89, .19};
  memcpy(q, joint_angles, 7*sizeof(double));
  double dfdq[7];
  double *dfdw = &dfdq[4];
  double lambda = 0; //.1;  // wrist angle change regularization

  double pdes[6] = {0,0,0, paddle_orientation[0], paddle_orientation[1], paddle_orientation[2]};  // desired paddle state

  double *paddle = joints_to_paddle(q);  // paddle position + normal
  double fmax = dot(&paddle[3], paddle_orientation, 3) - lambda*dist2(w, w0, 3);
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

    // add in gradient due to wrist angle change regularizer
    double dw[3];
    sub(dw, w, w0, 3);
    mult(dw, dw, -2*lambda, 3);
    add(dfdw, dfdw, dw, 3);

    // take steps in the direction of the gradient and pick the one with max f value
    double f[3];
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++)
	w[j] += a[i]*s*dfdw[j];
      paddle = joints_to_paddle(q);
      f[i] = dot(&paddle[3], paddle_orientation, 3) - lambda*dist2(w, w0, 3);
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

double *PaddleOrientTrajectoryController::update(double *joints)
{
  //printf("PaddleOrientTrajectoryController::update()\n"); //dbug

  // current time
  t = ros::Time::now().toSec();

  // compute index in the trajectory
  uint n;
  for (n = 0; n < traj_times.size(); n++)
    if (t < traj_times[n])
      break;

  // if time is within control tape
  if (n > 0 && n < traj_times.size()) {

    // find a set of wrist joint angles that results in the desired paddle orientation
    double paddle_orientation[3];
    for (int i = 0; i < 3; i++)
      paddle_orientation[i] = traj_orientations[i][n];
    double *wrist_angles = paddleOrientationToWristAngles(joints, paddle_orientation);

    // update pid goal
    for (int i = 0; i < 3; i++)
      pid->setGoalForJoint(i+4, wrist_angles[i]);

    free(wrist_angles);
  }

  double *torques = pid->update(joints);

  return &torques[4];
}

void PaddleOrientTrajectoryController::setOrientationTrajectory(const vector<double> &times, const vector<float> &orientations)
{
  int N = times.size();

  // set trajectory times
  traj_times.clear();
  for (int i = 0; i < N; i++)
    traj_times.push_back(times[i]);

  // set orientation command sequences
  for (int i = 0; i < 3; i++) {
    traj_orientations[i].clear();
    for (int j = 0; j < N; j++)
      traj_orientations[i].push_back(orientations[j*3+i]);
  }
}

bool PaddleOrientTrajectoryController::isStarted()
{
  return ros::Time::now().toSec() >= traj_times[0];
}

bool PaddleOrientTrajectoryController::isFinished()
{
  double t = ros::Time::now().toSec();
  int n = traj_times.size() - 1;

  return (traj_times[n] < t);
}

void PaddleOrientTrajectoryController::reset()
{
  for (int i = 0; i < 3; i++)
    pid->resetJoint(i+4);
}


//--------------- TorqueTrajectoryController --------------//

TorqueTrajectoryController::TorqueTrajectoryController(PID *pid)
{
  this->pid = pid;
}

double *TorqueTrajectoryController::update(double *joints)
{
  // default is to follow pid controller
  double *pid_torques = pid->update(joints);
  for (int i = 0; i < 7; i++)
    torques[i] = pid_torques[i];

  // current time
  t = ros::Time::now().toSec();

  // compute index in the control tape
  uint n;
  for (n = 0; n < torque_times.size(); n++)
    if (t < torque_times[n])
      break;

  // if time is within control tape
  if (n > 0 && n < torque_times.size()) {
    // override torques for controlled joints
    for (int i = 0; i < 7; i++)
      if (is_torque_controlled[i])
	torques[i] = torque_commands[i][n-1];
  }

  return torques;
}

void TorqueTrajectoryController::setTorqueTrajectory(const vector<int> &joints, const vector<double> &times, const vector<float> &torques)
{
  int N = times.size();
  int D = joints.size();

  // set joints to control
  for (int i = 0; i < 7; i++)
    is_torque_controlled[i] = false;
  for (int i = 0; i < D; i++)
    is_torque_controlled[joints[i]] = true;

  // set trajectory times
  torque_times.clear();
  for (int i = 0; i < N; i++)
    torque_times.push_back(times[i]);

  // set torque command sequences
  for (int i = 0; i < 7; i++) {
    torque_commands[i].clear();
    if (is_torque_controlled[i])
      for (int j = 0; j < N; j++)
	torque_commands[i].push_back(torques[j*D+i]);
  }
}

bool TorqueTrajectoryController::isStarted()
{
  return ros::Time::now().toSec() >= torque_times[0];
}

bool TorqueTrajectoryController::isFinished()
{
  double t = ros::Time::now().toSec();
  int n = torque_times.size() - 1;

  //printf("torque_times[0] = %f\n", torque_times[0]);
  //printf("torque_times[%d] = %f, t = %f\n", n, torque_times[n], t);

  if (torque_times[n] < t)
    return true;
  else
    return false;
}

void TorqueTrajectoryController::reset()
{
  for (int i = 0; i < 7; i++)
    if (is_torque_controlled[i])
      pid->resetJoint(i);
}


//--------------- JointTrajectoryController --------------//

JointTrajectoryController::JointTrajectoryController(PID *pid)
{
  this->pid = pid;
}

double *JointTrajectoryController::update(double *joints)
{
  // current time
  t = ros::Time::now().toSec();

  // compute index in the trajectory
  uint n;
  for (n = 0; n < traj_times.size(); n++)
    if (t < traj_times[n])
      break;

  // if time is within control tape
  if (n > 0 && n < traj_times.size()) {
    // update pid goal
    double ja[7];
    for (int i = 0; i < 7; i++)
      ja[i] = traj_joint_angles[i][n-1];
    pid->setGoal(ja);
  }

  // default is to follow pid controller
  double *torques = pid->update(joints);

  return torques;
}

void JointTrajectoryController::setJointTrajectory(const vector<float> &joint_angles, const vector<double> &times)
{
  int N = times.size();

  // set trajectory times
  traj_times.clear();
  for (int i = 0; i < N; i++)
    traj_times.push_back(times[i]);

  // set joint angle trajectory
  for (int i = 0; i < 7; i++) {
    traj_joint_angles[i].clear();
    for (int j = 0; j < N; j++)
      traj_joint_angles[i].push_back(joint_angles[j*7+i]);
  }

  // disable velocity limits
  //for (int i = 0; i < 7; i++)
  //  pid->setVelocityLimit(i, 100000);

  // set gravity compensation
  //reset();
  //pid->setGravityCompensation(true);

  printf("current time = %.3f, t0 = %.3f, duration = %.3f\n", ros::Time::now().toSec(),
	 times[0], times[N-1] - times[0]);

  /*dbug
  printf("times = [");
  for (int i = 0; i < N; i++)
    printf("%.3f, ", times[i]);
  printf("]\n");

  printf("joint_angles:");
  for (int i = 0; i < N; i++) {
    printf("  ");
    for (int j = 0; j < 7; j++)
      printf("%.3f  ", traj_joint_angles[j][i]);
    printf("\n");
  }
  */
}

bool JointTrajectoryController::isStarted()
{
  return ros::Time::now().toSec() >= traj_times[0];
}

bool JointTrajectoryController::isFinished()
{
  double t = ros::Time::now().toSec();
  int n = traj_times.size() - 1;

  return (traj_times[n] < t);
}

void JointTrajectoryController::reset()
{
  for (int i = 0; i < 7; i++)
    pid->resetJoint(i);
}


//--------------- ArmTracker --------------//

ArmTracker::ArmTracker(int num_joints)
{
  njoints = num_joints;
}


//--------------- MovingAverageArmTracker --------------//

MovingAverageArmTracker::MovingAverageArmTracker(int num_joints, int window_size) :
  ArmTracker(num_joints)
{
  wsize = window_size;
  safe_calloc(q, njoints, double);
  safe_calloc(dq, njoints, double);
  Q = new_matrix2(wsize, njoints);
  safe_calloc(T, wsize, double);
  pos = 0;
  cnt = 0;
}

void MovingAverageArmTracker::update(double *joint_angles, double t)
{
  memcpy(Q[pos], joint_angles, njoints*sizeof(double));
  T[pos] = t;
  pos = (pos+1) % wsize;
  if (cnt < wsize)
    cnt++;

  // don't filter joint angles
  memcpy(q, joint_angles, njoints*sizeof(double));
  
  // average velocity over the moving window
  int i0 = (cnt < wsize ? 0 : pos);
  if (t > T[i0]) {
    sub(dq, q, Q[0], njoints);
    mult(dq, dq, 1.0/(t-T[i0]), njoints);
  }
}

double *MovingAverageArmTracker::getJointAngles()
{
  return q;
}

double *MovingAverageArmTracker::getJointVelocities()
{
  return dq;
}


//-------------- Kinematics ----------------//


//frame k to frame 0 (not base)
Vector4f forward_kinematics_kToZero(double *joint_angles, int k, double *point_coord)
{
  static bool first = 1;

  Vector4f point(point_coord[0], point_coord[1], point_coord[2], point_coord[3]);
  for(int i = k; i > 0; i--) {

    double a, alpha, d;
    switch(i) {
    case 1: a = 0; alpha = -M_PI/2; d = 0; break;
    case 2: a = 0; alpha = M_PI/2; d = 0; break;
    case 3: a = 0.045; alpha = -M_PI/2; d = 0.55; break;
    case 4: a = -0.045; alpha = M_PI/2; d = 0; break;
    case 5: a = 0; alpha = -M_PI/2; d = 0.3; break;
    case 6: a = 0; alpha = M_PI/2; d = 0; break;
    case 7: a = 0; alpha = 0; d = 0.060; break;
    default: a = 0; alpha = 0; d = 0; break;
    }
    double c = cos(joint_angles[i-1]);
    double s = sin(joint_angles[i-1]);
   
    Matrix4f temp;
    temp << c, -s*cos(alpha), s*sin(alpha), a*c, s, c*cos(alpha), -c*sin(alpha), a*s, 0, sin(alpha), cos(alpha), d, 0, 0, 0, 1;
    point = temp * point;
  }

  first = 0;

  return point;

}

Vector4f forward_kinematics_ZeroToTable(Vector4f point_zero)
{

  Matrix4f zero_to_table;
  zero_to_table <<
    1,  0,  0, -.44, //-.71, //-0.589,
    0,  0,  1,  .28, //0.1492,
    0, -1,  0,  .50, //0.5589,
    0,  0,  0,    1;
  return zero_to_table * point_zero;

}

bool projected_in_collision(double *current_joint_angles, double *previous_joint_angles, double dt)
{
  double projected_joint_angles[7];
  
  for (int n = 0; n < 5; n++) {
    for(int i = 0; i < 7; i++) { 
      projected_joint_angles[i] = current_joint_angles[i] +
	((current_joint_angles[i] - previous_joint_angles[i]) / dt)
	*n*CHECK_INTERVAL;
      if(in_collision(projected_joint_angles)) return true;
    }
  }
  /*
  printf("projected joint angles: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", 
	 projected_joint_angles[0], projected_joint_angles[1], projected_joint_angles[2],
	 projected_joint_angles[3], projected_joint_angles[4], projected_joint_angles[5], projected_joint_angles[6]);    
  */
  return false;

}




bool in_collision(double *joint_angles)
{
  //check if bang into itself
  if ( joint_angles[1] < -2 || joint_angles[1] > 2) return true;
  else if ( joint_angles[3] < -0.9 || joint_angles[3] > 3.1) return true;

  //coordinate frame 4
  for (double z = 0; z <= 0.4; z+=0.03) {  
    double point_coord[4] = {0, 0, z, 1};
    Vector4f point  = forward_kinematics_kToZero(joint_angles, 4, point_coord);

    if (point[2] <= -0.15)
      return true;  //goes behind board 

    point = forward_kinematics_ZeroToTable(point);

    if (point[0] > 0 && point[1] > 0) {  //over table
      if (point[2] < .05 && point[2] > -0.06 -.05)
	return true;  // in contact with table
    }
    else if (point[0]*point[0] + point[2]*point[2] < .05*.05)
      return true;  //edge case
  }

  // tool
  for(double z = 0; z < 0.18; z+=0.03) {
    double point_coord[4] = {0, 0, z, 1};
    Vector4f point  = forward_kinematics_kToZero(joint_angles, 7, point_coord);

    if (point[2] <= -0.15)
      return true; //goes behind board 

    point = forward_kinematics_ZeroToTable(point);
    if (point[0] > 0 && point[1] > 0) { //over table
      if (point[2] < 0.08 && point[2] > -0.06 -.10) // changed from .10

	return true;  //in contact with table
    }
    else if (point[0]*point[0] + point[2]*point[2] < .10*.10)
      return true; //edge case
  }

  return false;

}


// get a sine wave torque signal for one joint
double get_sin_signal(double t, double period, double amp)
{
  return amp * sin(2 * M_PI * t / period);
}
