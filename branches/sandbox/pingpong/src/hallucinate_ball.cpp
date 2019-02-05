#include <ros/ros.h>
#include <fastwam/SwingData.h>
#include <sensor_msgs/PointCloud.h>
#include <bingham/util.h>


char *swingdata_path;


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

  printf("Saving swing data..."); fflush(0);
  save_swing(swingdata_path, msg);
  printf("done\n");

  exit(0);
}


void usage(char *argv[])
{
  printf("usage: %s <ball_traj_file> <swingdata_path>\n", argv[0]);
  exit(1);
}


int main(int argc, char *argv[])
{
  // load args
  if (argc < 3)
    usage(argv);
  int n, m;
  double **ball_traj = load_matrix(argv[1], &n, &m);
  if (m != 3) {
    printf("Error: m != 3 in ball traj file\n");
    usage(argv);
  }
  swingdata_path = argv[2];


  // init ROS
  ros::init(argc, argv, "hallucinate_ball");
  ros::NodeHandle nh;
  ros::Publisher ball_pub = nh.advertise<sensor_msgs::PointCloud>("ball_trajectory_prediction", 1);
  ros::Subscriber sub_swingdata = nh.subscribe("/fastwam/swingdata", 1, swingdata_callback);

  sleep(3);


  // publish fake ball trajectory
  sensor_msgs::PointCloud cloud;
  cloud.header.stamp = ros::Time::now() + ros::Duration(.5);
  cloud.header.frame_id = "pingpongtable";
  cloud.points.resize(n);
  for (uint i = 0; i < n; i++) {
    cloud.points[i].x = ball_traj[i][0];
    cloud.points[i].y = ball_traj[i][1];
    cloud.points[i].z = ball_traj[i][2];
  }
  ball_pub.publish(cloud);

  printf("Waiting for swing...\n");
  ros::spin();

  return 0;
}
