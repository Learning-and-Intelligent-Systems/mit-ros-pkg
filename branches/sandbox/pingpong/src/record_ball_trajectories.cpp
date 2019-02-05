#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <bingham/util.h>

char *traj_file, *pred_traj_file;
double **traj = NULL, **pred_traj = NULL;
int traj_len = 0, pred_traj_len = 0;



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

  printf("p"); fflush(0);
  //printf("Processed predicted trajectory message in %.3f sec\n", ros::Time::now().toSec() - t0); //dbug
}

void sigint_handler(int signum)
{
  save_matrix(traj_file, traj, traj_len, 4);
  save_matrix(pred_traj_file, pred_traj, traj_len, 4);

  exit(0);
}

void usage(char *argv[])
{
  printf("usage: %s <traj_file> <predicted_traj_file> [timeout]\n", argv[0]);
  exit(1);
}


int main(int argc, char *argv[])
{
  // load args
  if (argc < 3)
    usage(argv);

  traj_file = argv[1];
  pred_traj_file = argv[2];
  double timeout = (argc < 4 ? -1.0 : atof(argv[3]));

  // init ROS
  ros::init(argc, argv, "record_ball_trajectories");
  ros::NodeHandle nh;
  ros::Subscriber sub_traj = nh.subscribe("ball_trajectory", 100, traj_callback);
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

  save_matrix(traj_file, traj, traj_len, 4);
  save_matrix(pred_traj_file, pred_traj, pred_traj_len, 4);

  printf("\n");

  return 0;
}
