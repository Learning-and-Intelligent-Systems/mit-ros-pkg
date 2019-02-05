#include <ros/ros.h>
#include <pingpong/HitPolicies.h>
#include <bingham/util.h>


void ros_sleep(int seconds)
{
  for (int j = 0; j < seconds; j++) {
    ros::spinOnce();
    sleep(1);
  }
}


int main(int argc, char *argv[])
{
  if (argc != 3) {
    printf("usage: %s <HP_in> <HP_out>\n", argv[0]);
    exit(1);
  }

  int n, n2, m1, m2;
  double **HP_in = load_matrix(argv[1], &n, &m1);
  double **HP_out = load_matrix(argv[2], &n2, &m2);

  if (n != n2) {
    printf("Error: HP_in and HP_out have different lengths!\n");
    exit(1);
  }

  // init ROS
  ros::init(argc, argv, "send_hit_policies");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<pingpong::HitPolicies>("hit_policies", 1);

  ros_sleep(2);

  pingpong::HitPolicies msg;
  msg.ball_bounce_params.resize(n*m1);
  msg.hit_params.resize(n*m2);
  for (int i = 0; i < n*m1; i++)
    msg.ball_bounce_params[i] = HP_in[0][i];
  for (int i = 0; i < n*m2; i++)
    msg.hit_params[i] = HP_out[0][i];

  pub.publish(msg);

  ros_sleep(2);

  return 0;
}
