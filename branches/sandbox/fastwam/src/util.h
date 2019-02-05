#ifndef UTIL_H
#define UTIL_H

#include <vector>

using namespace std;


//const double pid_kp[7] = {2500, 2500, 600, 500, 50, 50, 8};
//const double pid_ki[7] = {5,    5,    2,  0.5,  0.5,  0.5,  0.1};
//const double pid_kd[7] = {20,   20,    5,    2,  0.5,  0.5, 0.05};
//const double pid_kp[7] = {1500, 2500, 500, 500, 50, 50, 8};
//const double pid_ki[7] = {5,    5,    2,  0.5,  0.5,  0.5,  0.1};
//const double pid_torque_limits[7] = {40, 40, 15, 15, 5, 5, 5};

const double pid_torque_limits[7] = {40, 40, 15, 15, 3, 10, 2};
//const double pid_diff_torque_limits[7] = {200, 200, 100, 100, 50, 50, 50};


const double pid_kp[7] = {500, 200, 200, 200,   10,   10,   2};
const double pid_ki[7] = {  5,   2,   2,   0.5,  0.1, 0.5, 0.1};
const double pid_kd[7] = { 20,  10,   5,   2,    0.0, 0.0, 0.0};

//const double pid_kp[7] = {100, 50, 50, 50,   10,   2,   2};
//const double pid_ki[7] = {  .5,   .2,   .2,   0.1,  0.1, 0.1, 0.1};
//const double pid_kd[7] = { 0,  0,   0,   0,    0.0, 0.0, 0.0};



class PID {

 public:
  PID();
  double *update(double *joints);
  void setGoal(double *new_goal);
  void setGoalForJoint(int joint, double new_goal);
  void resetJoint(int joint);
  void resetAllJoints();
  //void setGravityCompensation(bool gc);
  void setVelocityLimit(int joint, double v);
  void resetVelocityLimits();

 protected:
  double t;  // current time
  double P[7];
  double I[7];
  double torques[7];
  //double gravity_torques[7];
  //bool gravity_compensation;
  double goal[7];
  double subgoal[7];
  double subgoal_vel_limits[7];
  bool reset[7];
};


class GainTrajectoryController {

 public:
  GainTrajectoryController();
  double *update(double *q, double *dq);
  void setGainTrajectory(const vector<double> &times, const vector<float> &gains);
  bool isStarted();
  bool isFinished();

 protected:
  double t;           // current time
  vector<double> traj_times;
  double ***traj_gains;
  double torques[7];
};


class PaddleOrientTrajectoryController {

 public:
  PaddleOrientTrajectoryController(PID *pid);
  double *update(double *joints);
  void setOrientationTrajectory(const vector<double> &times, const vector<float> &orientations);
  bool isStarted();
  bool isFinished();
  void reset();

 protected:
  double t;           // current time
  PID *pid;
  vector<double> traj_times;
  vector<double> traj_orientations[3];
  double *paddleOrientationToWristAngles(double *wrist_angles, double *paddle_orientation);
};


class TorqueTrajectoryController {

 public:
  TorqueTrajectoryController(PID *pid);
  double *update(double *joints);
  void setTorqueTrajectory(const vector<int> &joints, const vector<double> &times, const vector<float> &torques);
  bool isStarted();
  bool isFinished();
  void reset();

 protected:
  double t;           // current time
  PID *pid;
  double torques[7];  // current torques
  bool is_torque_controlled[7];
  vector<double> torque_times;
  vector<double> torque_commands[7];
};


class JointTrajectoryController {

 public:
  JointTrajectoryController(PID *pid);
  double *update(double *joints);
  void setJointTrajectory(const vector<float> &joint_angles, const vector<double> &times);
  bool isStarted();
  bool isFinished();
  void reset();

 protected:
  double t;           // current time
  PID *pid;
  vector<double> traj_times;
  vector<double> traj_joint_angles[7];
};


class ArmTracker {
 public:
  ArmTracker(int num_joints);
  virtual void update(double *joint_angles, double t) = 0;
  virtual double *getJointAngles() = 0;
  virtual double *getJointVelocities() = 0;

 protected:
  int njoints;
};

class MovingAverageArmTracker : public ArmTracker {
 public:
  MovingAverageArmTracker(int n, int window_size);
  void update(double *joint_angles, double t);
  double *getJointAngles();
  double *getJointVelocities();

 protected:
  double *q;     // current joint angles estimate
  double *dq;    // current joint velocities estimate
  double **Q;    // circular buffer of joint angles
  double *T;     // circular buffer of times
  int pos;       // buffer position
  int cnt;       // buffer cnt
  int wsize;     // buffer size
};


bool projected_in_collision(double *current_joint_angles, double *previous_joint_angles, double dt);
bool in_collision(double *joint_angles);
double get_sin_signal(double t, double period, double amp);




#endif
