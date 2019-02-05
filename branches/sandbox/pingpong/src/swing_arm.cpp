#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <fastwam/GainTrajectory.h>
#include <fastwam/JointTrajectory.h>
#include <fastwam/MoveTo.h>
#include <fastwam/PaddleOrientationTrajectory.h>
#include <fastwam/SwingData.h>
#include <bingham/util.h>
#include "dynamics.h"


ros::Publisher wam_moveto_pub;
ros::Publisher wam_gain_traj_pub;
ros::Publisher wam_joint_traj_pub;
ros::Publisher paddle_orient_traj_pub;
//ArmDynamics *arm;
bool got_swing = false;
fastwam::SwingData last_swing;


void ros_sleep(int seconds)
{
  for (int j = 0; j < seconds; j++) {
    ros::spinOnce();
    sleep(1);
  }
}

void wait_for_swing(int secs)
{
  printf("Waiting for swing");
  for (int i = 0; i < secs; i++) {
    ros_sleep(1);
    printf("."); fflush(0);
    if (got_swing)
      return;
  }

  printf("Error: wait_for_swing() timed out after %d seconds\n", secs);
  exit(1);
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

  for (int i = 0; i < n; i++)
    msg.times.push_back(t0 + dt*i);
  msg.times.push_back(t0 + dt*n);

  for (int i = 0; i < n*m; i++)
    msg.gains.push_back(gains[i]);

  wam_gain_traj_pub.publish(msg);
  got_swing = false;
}

void joint_swing(double *joint_angles, double t0, double dt, int n)
{
  fastwam::JointTrajectory msg;
  int m = 7;

  for (int i = 0; i < n; i++)
    msg.times.push_back(t0 + dt*i);
  msg.times.push_back(t0 + dt*n);

  for (int i = 0; i < n*m; i++)
    msg.joint_angles.push_back(joint_angles[i]);

  wam_joint_traj_pub.publish(msg);
  got_swing = false;
}

void publish_constant_paddle_orientation(double *paddle_orientation, double t0, double t1)
{
  fastwam::PaddleOrientationTrajectory paddle_msg;
  paddle_msg.times.push_back(t0);
  paddle_msg.times.push_back(t1);
  paddle_msg.orientations.push_back(paddle_orientation[0]);
  paddle_msg.orientations.push_back(paddle_orientation[1]);
  paddle_msg.orientations.push_back(paddle_orientation[2]);
  paddle_msg.orientations.push_back(paddle_orientation[0]);
  paddle_msg.orientations.push_back(paddle_orientation[1]);
  paddle_msg.orientations.push_back(paddle_orientation[2]);
  paddle_orient_traj_pub.publish(paddle_msg);
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

  //printf("Adding swing data..."); fflush(0);
  //arm->addSwingData(msg);
  //printf("done\n");

  last_swing = msg;
  got_swing = true;
}

void usage(int argc, char *argv[])
{
  printf("usage: %s -g <gain swing args>  --or--  %s -j <joint swing args>\n", argv[0], argv[0]);
  printf("\nGain swing:\n");
  printf("       %s -g <init_joint_angles(7)> [<paddle_normal(3)> <dt> <gains_file> <swingdata_path>]\n", argv[0]);
  printf("\nJoint swing:\n");
  printf("       %s -j <init_joint_angles(7)> [<paddle_normal(3)> <dt> <joint_angles_file> <swingdata_path>]\n", argv[0]);
  exit(1);
}

int main(int argc, char *argv[])
{
  if ((argc != 9 && argc < 15) || argv[1][0] != '-')
    usage(argc, argv);

  enum {GAIN_SWING, JOINT_SWING} swing_type;
  if (argv[1][1] == 'g')
    swing_type = GAIN_SWING;
  else if (argv[1][1] == 'j')
    swing_type = JOINT_SWING;
  else
    usage(argc, argv);

  // read inputs
  double JA[7];
  for (int i = 0; i < 7; i++)
    JA[i] = atof(argv[i+2]);

  if (argc == 9) {  // just move the arm to a set configuration
    ros::init(argc, argv, "swing_arm");
    ros::NodeHandle nh;
    wam_moveto_pub = nh.advertise<fastwam::MoveTo>("/fastwam/moveto", 1);
    sleep(2);
    moveto(JA);
    ros_sleep(3);
    return 0;
  }

  double paddle_normal[3];
  paddle_normal[0] = atof(argv[9]);
  paddle_normal[1] = atof(argv[10]);
  paddle_normal[2] = atof(argv[11]);
  normalize(paddle_normal, paddle_normal, 3);
  double dt = atof(argv[12]);
  char *input_file = argv[13];
  char *swingdata_path = argv[14];

  // init ROS
  ros::init(argc, argv, "swing_arm");
  ros::NodeHandle nh;
  wam_moveto_pub = nh.advertise<fastwam::MoveTo>("/fastwam/moveto", 1);
  wam_gain_traj_pub = nh.advertise<fastwam::GainTrajectory>("/fastwam/gain_trajectory", 1);
  wam_joint_traj_pub = nh.advertise<fastwam::JointTrajectory>("/fastwam/joint_trajectory", 1);
  paddle_orient_traj_pub = nh.advertise<fastwam::PaddleOrientationTrajectory>("/fastwam/paddle_orientation_trajectory", 1);
  ros::Publisher training_swing_pub = nh.advertise<std_msgs::Empty>("training_swing", 1);
  ros::Subscriber sub_swingdata = nh.subscribe("/fastwam/swingdata", 1, swingdata_callback);
  //arm = new ArmDynamics(7);
  //arm->load(swingdata_path);

  sleep(2);

  // tell the brain we're training swings
  std_msgs::Empty training_swing_msg;
  training_swing_pub.publish(training_swing_msg);

  //moveto(JA);
  //ros_sleep(3);

  double duration = 0;

  if (swing_type == GAIN_SWING) {
    int n, m;
    double **gains = load_matrix(input_file, &n, &m);
    if (m != 7*15) {
      printf("Error: m != 7*15\n");
      exit(1);
    }

    //for (int i = 0; i < n; i++) {
    //  for (int j = 0; j < m; j++)
    //    printf("%.2f ", gains[i][j]);
    //  printf("\n");
    //}
  
    double t0 = ros::Time::now().toSec() + 0.1;
    gain_swing(gains[0], t0, dt, n);
    //publish_constant_paddle_orientation(paddle_normal, t0, t0 + dt*n);

    duration = dt*n;
  }
  else {  //(swing_type == JOINT_SWING)
    int n, m;
    double **joints = load_matrix(input_file, &n, &m);
    if (m != 7) {
      printf("Error: m != 7\n");
      exit(1);
    }
  
    double t0 = ros::Time::now().toSec() + 0.01;
    joint_swing(joints[0], t0, dt, n);
    //publish_constant_paddle_orientation(paddle_normal, t0, t0 + dt*n);

    duration = dt*n;
  }

  wait_for_swing(ceil(duration + 10));

  // save the swing data
  printf("Saving swing data..."); fflush(0);
  //arm->save(swingdata_path);
  save_swing(swingdata_path, last_swing);
  printf("done\n");

  return 0;
}

