#include <signal.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <fastwam/MoveTo.h>
#include <fastwam/JointTrajectory.h>
#include "dynamics.h"
#include "kinematics.h"


#define TABLE_WIDTH 1.525


ros::Publisher wam_moveto_pub;
ros::Publisher wam_joint_traj_pub;
int state;
double *joint_angles;
double last_swing_time = 0;
ArmDynamics *arm;


#define NJA 19
double xJA = -.3;
double yJA[NJA] = {.2, .25, .3, .35, .4, .45, .5, .55, .6, .65, .7, .75, .8, .85, .9, .95, 1, 1.05, 1.1};
double JA[NJA][7] = {
{0.7521, 0.8526, 0.6985, 2.8328, -2.2999, 0.6714, 0.5013, },
{0.7578, 0.6584, 0.6191, 2.7814, -2.2505, 0.4391, 0.3999, },
{0.7325, 0.4855, 0.5736, 2.7373, -2.2392, 0.2615, 0.3315, },
{0.6834, 0.3272, 0.5472, 2.6841, -2.2473, 0.0958, 0.2859, },
{0.5536, 0.1921, 0.6089, 2.6234, -2.1646, -0.0500, 0.1660, },
{0.5534, 0.0608, 0.5026, 2.5449, -2.1720, -0.2375, 0.1334, },
{0.7157, -0.0397, 0.2562, 2.4695, -2.1155, -0.3845, 0.0671, },
{0.3613, -0.1266, 0.5109, 2.3594, -2.1922, -0.5682, 0.1446, },
{0.3074, -0.1896, 0.4806, 2.2573, -2.1744, -0.7145, 0.1527, },
{0.3032, -0.2305, 0.4224, 2.1565, -2.1356, -0.8253, 0.1653, },
{0.3077, -0.2558, 0.3703, 2.0547, -2.0850, -0.9102, 0.1861, },
{0.3195, -0.2688, 0.3245, 1.9523, -2.0236, -0.9698, 0.2130, },
{0.3328, -0.2720, 0.2879, 1.8478, -1.9559, -1.0071, 0.2459, },
{0.3491, -0.2666, 0.2593, 1.7398, -1.8856, -1.0218, 0.2840, },
{0.3552, -0.2543, 0.2452, 1.6282, -1.8090, -1.0193, 0.3215, },
{0.3600, -0.2338, 0.2417, 1.5074, -1.7393, -0.9967, 0.3653, },
{0.3629, -0.2059, 0.2504, 1.3806, -1.6668, -0.9494, 0.4026, },
{0.3309, -0.1696, 0.2956, 1.2385, -1.5981, -0.8876, 0.4408, },
{0.3436, -0.1189, 0.3407, 1.0801, -1.5537, -0.7756, 0.4666, }};


void move_paddle(double x, double y, double z)
{
  // find closest stored y
  int imin = 0;
  double dmin = 100;
  for (int i = 0; i < NJA; i++) {
    double d = fabs(yJA[i] - y);
    if (d < dmin) {
      dmin = d;
      imin = i;
    }
  }
  joint_angles = JA[imin];

  fastwam::MoveTo msg;
  for (int j = 0; j < 7; j++)
    msg.joint_angles.push_back(joint_angles[j]);
  wam_moveto_pub.publish(msg);
}


void move_paddle_traj(double x, double y, double z)
{
  // find closest stored y
  int imin = 0;
  double dmin = 100;
  for (int i = 0; i < NJA; i++) {
    double d = fabs(yJA[i] - y);
    if (d < dmin) {
      dmin = d;
      imin = i;
    }
  }
  joint_angles = JA[imin];

  double t0 = ros::Time::now().toSec();

  fastwam::JointTrajectory msg;
  msg.times.push_back(t0);
  msg.times.push_back(t0 + .2);
  for (int j = 0; j < 7; j++)
    msg.joint_angles.push_back(joint_angles[j]);
  for (int j = 0; j < 7; j++)
    msg.joint_angles.push_back(joint_angles[j]);

  wam_joint_traj_pub.publish(msg);
}


void swing()
{
    double t0 = ros::Time::now().toSec();
    double dt = .01;

    fastwam::JointTrajectory msg;

    double dp[6] = {.01, 0, .005, 0, 0, 0};
    double w[7] = {1,1,.1,1,.3,1,1};
    double ja[7];
    memcpy(ja, joint_angles, 7*sizeof(double));
    for (int j = 0; j < 50; j++) {
      double *dj = paddle_to_joints_displacement_weighted(ja, dp, w);
      for (int k = 0; k < 7; k++)
	ja[k] += dj[k];
      free(dj);

      if (j >= 10) {
	msg.times.push_back(t0 + dt*j);
	for (int k = 0; k < 7; k++)
	  msg.joint_angles.push_back(ja[k]);
      }

      //moveto(ja);
      //ros::spinOnce();
    }
    msg.times.push_back(t0 + dt*100);
    for (int k = 0; k < 7; k++)
      msg.joint_angles.push_back(ja[k]);


    wam_joint_traj_pub.publish(msg);

    last_swing_time = t0 + 1.0;
}


// predicted ball trajectory
void ball_trajectory_callback(const sensor_msgs::PointCloud &msg)
{
  int n = msg.points.size();

  // don't move if no ball is found or if ball is moving away from the robot
  if (n < 5 || msg.points[n-1].y > msg.points[n-5].y) {
    state = 0;
    return;
  }

  //if (state == 2 && ros::Time::now().toSec() > last_swing_time)
  //  state = 0;

  if (state == 0) {
    // find out where predicted trajectory intersects robot arm plane
    int ihit = -1;
    for (uint i = 0; i < msg.points.size(); i++) {
      if (msg.points[i].y < xJA) {
	ihit = i;
	break;
      }
    }
      
    if (ihit >= 0) {
      move_paddle_traj(xJA, TABLE_WIDTH - msg.points[ihit].x, .2);
      state = 1;

      //dbug
      printf("ball x position = %.2f\n", msg.points[0].y);
      printf("ball time = %.3f\n", msg.header.stamp.toSec());
      printf("current time = %.3f\n", ros::Time::now().toSec());
    }
  }
  else if (state == 2)
    swing();

  //printf("state = %d\n", state);
}


void swingdata_callback(const fastwam::SwingData &msg)
{
  printf("Adding swing data..."); fflush(0);
  arm->addSwingData(msg);
  printf("done\n");

  if (state==1)
    state = 2;
  else if (state==2)
    state = 0;
}


void sigint_handler(int signum)
{
  printf("Saving swing data..."); fflush(0);
  arm->save("data/arm7");
  printf("done\n");
  exit(0);
}


int main(int argc, char *argv[])
{
  // init ROS
  ros::init(argc, argv, "pingpongbrain");
  ros::NodeHandle nh;
  ros::Subscriber ball_sub = nh.subscribe("ball_trajectory_prediction", 1, ball_trajectory_callback);
  ros::Subscriber sub_swingdata = nh.subscribe("/fastwam/swingdata", 1, swingdata_callback);
  wam_moveto_pub = nh.advertise<fastwam::MoveTo>("/fastwam/moveto", 1);
  wam_joint_traj_pub = nh.advertise<fastwam::JointTrajectory>("/fastwam/joint_trajectory", 1);

  signal(SIGINT, sigint_handler);

  state = 0;
  arm = new ArmDynamics(7);

  sleep(3);

  move_paddle(xJA, .7, .2);

  ros::spin();

  return 0;
}
