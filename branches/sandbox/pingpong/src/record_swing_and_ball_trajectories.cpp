#include <ros/ros.h>
#include <fastwam/SwingData.h>
#include <sensor_msgs/PointCloud.h>
#include <bingham/util.h>
#include <pingpong/BallState.h>

char *swing_file, *traj_file, *pred_traj_file, *ball_file;
bool got_swing = false;
fastwam::SwingData last_swing;
double **traj = NULL, **pred_traj = NULL, **ball = NULL;
int traj_len = 0, pred_traj_len = 0, ball_len = 0;


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

  last_swing = msg;
  got_swing = true;

  save_swing(swing_file, last_swing);
  save_matrix(ball_file, ball, ball_len, 46);
  //save_matrix(traj_file, traj, traj_len, 4);
  save_matrix(pred_traj_file, pred_traj, pred_traj_len, 4);

  exit(0);
}

void ball_callback(const pingpong::BallState &msg)
{
  double **new_ball = new_matrix2(ball_len + 1, 46);

  new_ball[ball_len][0] = msg.t;
  new_ball[ball_len][1] = msg.ox;
  new_ball[ball_len][2] = msg.oy;
  new_ball[ball_len][3] = msg.oz;
  new_ball[ball_len][4] = msg.x;
  new_ball[ball_len][5] = msg.y;
  new_ball[ball_len][6] = msg.z;
  new_ball[ball_len][7] = msg.vx;
  new_ball[ball_len][8] = msg.vy;
  new_ball[ball_len][9] = msg.vz;
  for (int i = 0; i < 36; i++)
    new_ball[ball_len][10+i] = msg.cov[i];

  if (ball) {
    matrix_copy(new_ball, ball, ball_len, 46);
    free_matrix2(ball);
  }

  ball = new_ball;
  ball_len++;

  printf("o"); fflush(0);
}

/*
void traj_callback(const sensor_msgs::PointCloud &msg)
{
  int n = msg.points.size();
  double **new_traj = new_matrix2(traj_len + n, 4);

  double t = msg.header.stamp.toSec();
  for (int i = 0; i < n; i++) {
    new_traj[traj_len + i][0] = t;
    new_traj[traj_len + i][1] = msg.points[i].x;
    new_traj[traj_len + i][2] = msg.points[i].y;
    new_traj[traj_len + i][3] = msg.points[i].z;
  }

  if (traj) {
    matrix_copy(new_traj, traj, traj_len, 4);
    free_matrix2(traj);
  }

  traj = new_traj;
  traj_len += n;

  printf("."); fflush(0);
}
*/

void pred_traj_callback(const sensor_msgs::PointCloud &msg)
{
  double t0 = ros::Time::now().toSec();

  int n = msg.points.size();
  double **new_pred_traj = new_matrix2(pred_traj_len + n, 4);

  double t = msg.header.stamp.toSec();
  for (int i = 0; i < n; i++) {
    new_pred_traj[pred_traj_len + i][0] = t;
    new_pred_traj[pred_traj_len + i][1] = msg.points[i].x;
    new_pred_traj[pred_traj_len + i][2] = msg.points[i].y;
    new_pred_traj[pred_traj_len + i][3] = msg.points[i].z;
  }

  if (pred_traj) {
    matrix_copy(new_pred_traj, pred_traj, pred_traj_len, 4);
    free_matrix2(pred_traj);
  }

  pred_traj = new_pred_traj;
  pred_traj_len += n;

  printf("+"); fflush(0);
  //printf("Processed predicted trajectory message in %.3f sec\n", ros::Time::now().toSec() - t0); //dbug
}

void sigint_handler(int signum)
{
  save_swing(swing_file, last_swing);
  save_matrix(ball_file, ball, ball_len, 46);
  //save_matrix(traj_file, traj, traj_len, 4);
  save_matrix(pred_traj_file, pred_traj, pred_traj_len, 4);

  exit(0);
}

void usage(char *argv[])
{
  //printf("usage: %s <swing_file> <ball_file> <traj_file> <predicted_traj_file> [timeout]\n", argv[0]);
  printf("usage: %s <swing_file> <ball_file> <predicted_traj_file> [timeout]\n", argv[0]);
  exit(1);
}


int main(int argc, char *argv[])
{
  // load args
  if (argc < 4)
    usage(argv);

  swing_file = argv[1];
  ball_file = argv[2];
  //traj_file = argv[3];
  pred_traj_file = argv[3];
  double timeout = (argc < 5 ? -1.0 : atof(argv[4]));

  // init ROS
  ros::init(argc, argv, "record_swing_and_ball_trajectories");
  ros::NodeHandle nh;
  ros::Subscriber sub_swingdata = nh.subscribe("/fastwam/swingdata", 1, swingdata_callback);
  ros::Subscriber sub_ball = nh.subscribe("ball_state", 100, ball_callback);
  //ros::Subscriber sub_traj = nh.subscribe("ball_trajectory", 100, traj_callback);
  ros::Subscriber sub_pred_traj = nh.subscribe("ball_trajectory_prediction", 100, pred_traj_callback);

  // spin
  double t0 = ros::Time::now().toSec();
  ros::Rate rate(10000);
  while (nh.ok()) {
    ros::spinOnce();
    rate.sleep();
    if (timeout >= 0.0 && ros::Time::now().toSec() > t0 + timeout)
      break;
  }

  save_swing(swing_file, last_swing);
  save_matrix(ball_file, ball, ball_len, 46);
  //save_matrix(traj_file, traj, traj_len, 4);
  save_matrix(pred_traj_file, pred_traj, pred_traj_len, 4);

  printf("\n");

  return 0;
}
