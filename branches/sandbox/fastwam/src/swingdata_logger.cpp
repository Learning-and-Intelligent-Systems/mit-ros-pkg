#include <stdio.h>
#include <ros/ros.h>
#include <fastwam/SwingData.h>




char *logdir = NULL;


void swingdata_callback(const fastwam::SwingData &msg)
{
  static int cnt = 0;

  char fname[128];
  if (logdir == NULL)
    sprintf(fname, "swingdata%d.m", cnt++);
  else
    sprintf(fname, "%s/swingdata%d.m", logdir, cnt++);
  FILE *f = fopen(fname, "w");

  int n = msg.t.size();
  int d = msg.joint_angles.size() / n;

  fprintf(f, "times = [");
  for (int i = 0; i < n; i++)
    fprintf(f, "%.6f ", msg.t[i]);
  fprintf(f, "];\n\n");

  fprintf(f, "joint_angles = [");
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < d; j++)
      fprintf(f, "%.6f ", msg.joint_angles[i*d+j]);
    fprintf(f, "; ...\n");
  }
  fprintf(f, "];\n\n");

  fprintf(f, "joint_torques = [");
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < d; j++)
      fprintf(f, "%.6f ", msg.joint_torques[i*d+j]);
    fprintf(f, "; ...\n");
  }
  fprintf(f, "];\n\n");

  fclose(f);
}



int main(int argc, char *argv[])
{
  // init ROS
  ros::init(argc, argv, "swingdata_logger");
  ros::NodeHandle nh;

  // subscribe to messages
  ros::Subscriber sub_swingdata = nh.subscribe("/fastwam/swingdata", 1, swingdata_callback);

  if (argc > 1)
    logdir = argv[1];

  ros::spin();

  return 0;
}
