#include <ros/ros.h>
#include <fastwam/SwingData.h>
#include <pingpong/HitPlan.h>
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


void ros_sleep(int seconds)
{
  for (int j = 0; j < seconds; j++) {
    ros::spinOnce();
    sleep(1);
  }
}


int main(int argc, char *argv[])
{
  if (argc != 12) {
    printf("usage: %s t x y z vx vy vz nx ny nz swingdatapath\n", argv[0]);
    exit(1);
  }

  // init ROS
  ros::init(argc, argv, "send_hit_policies");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<pingpong::HitPlan>("hit_plan", 1);
  ros::Subscriber sub_swingdata = nh.subscribe("/fastwam/swingdata", 1, swingdata_callback);

  ros_sleep(2);

  pingpong::HitPlan msg;
  msg.t = atof(argv[1]);
  msg.x = atof(argv[2]);
  msg.y = atof(argv[3]);
  msg.z = atof(argv[4]);
  msg.vx = atof(argv[5]);
  msg.vy = atof(argv[6]);
  msg.vz = atof(argv[7]);
  msg.nx = atof(argv[8]);
  msg.ny = atof(argv[9]);
  msg.nz = atof(argv[10]);
  swingdata_path = argv[11];

  pub.publish(msg);

  printf("Waiting for swing...\n");
  ros::spin();

  return 0;
}
