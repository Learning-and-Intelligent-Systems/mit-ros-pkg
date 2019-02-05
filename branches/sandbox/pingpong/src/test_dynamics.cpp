#include <ros/ros.h>
#include <bingham/util.h>
#include <fastwam/SwingData.h>
//#include <pingpong/SwingData.h>
#include <fastwam/TorqueTrajectory.h>
#include "dynamics.h"


using namespace fastwam;
//using namespace pingpong;


// global variables
bool got_swing_data = false;
SwingData swing_data;


void swingdata_callback(const SwingData &msg)
{
  swing_data = msg;
  got_swing_data = true;
}


void publish_constant_torque_traj(ros::Publisher &pub, double torque)
{
  fastwam::TorqueTrajectory msg;

  double duration = 5;

  double t = ros::Time::now().toSec();

  msg.joints.push_back(1);

  msg.times.push_back(t);
  msg.times.push_back(t + duration);

  msg.torques.push_back(torque);
  msg.torques.push_back(torque);

  pub.publish(msg);
}


double simulate_trajectory(const SwingData &swing, ArmDynamics &arm)
{
  int num_joints = arm.getNumJoints();
  double q[num_joints], dq[num_joints], ddq[num_joints], u[num_joints];

  // get initial arm state (q,dq)
  int n = swing.t.size();
  int d = swing.joint_angles.size() / n;
  for (int i = 0; i < num_joints; i++) {
    double dt = swing.t[1] - swing.t[0];
    q[i] = swing.joint_angles[d+i];
    dq[i] = (q[i] - swing.joint_angles[i]) / dt;
  }

  int NT = MIN(n, 1002);  // simulate for a max of 2 seconds

  // simulate trajectory
  double err_tot = 0.;
  for (int i = 1; i < NT-1; i++) {
    for (int j = 0; j < num_joints; j++)
      u[j] = swing.joint_torques[i*d+j];
    arm.dynamics(ddq, NULL, q, dq, u);
    double dt = swing.t[i+1] - swing.t[i];
    for (int j = 0; j < num_joints; j++) {
      q[j] += dt*dq[j];
      dq[j] += dt*ddq[j];
    }

    // compute prediction error
    double err = 0.;
    for (int j = 0; j < num_joints; j++) {
      double err_j = q[j] - swing.joint_angles[i*d+j];
      err += err_j*err_j;
    }
    err_tot += err;

    if (i%100 == 0) {
      printf(" --> true state (%d): (", i);
      for (int j = 0; j < num_joints-1; j++)
	printf("%.2f, ", swing.joint_angles[i*d+j]);
      printf("%.2f)\n", swing.joint_angles[i*d+num_joints-1]);
      printf(" --> sim. state (%d): (", i);
      for (int j = 0; j < num_joints-1; j++)
	printf("%.2f, ", q[j]);
      printf("%.2f)\n", q[num_joints-1]);
    }
  }

  double RMSE = sqrt(err_tot / (double)(NT-2));

  printf(" *** RMSE = %.4f ***\n", RMSE);

  return RMSE;
}


int main(int argc, char *argv[])
{
  // init ROS
  ros::init(argc, argv, "test_dynamics");
  ros::NodeHandle nh;
  ros::Rate poll_rate(10);
  ros::Publisher torque_traj_pub = nh.advertise<fastwam::TorqueTrajectory>("/fastwam/torque_trajectory", 1);
  ros::Subscriber sub_swingdata = nh.subscribe("/fastwam/swingdata", 1, swingdata_callback);

  // create arm dynamics object
  int num_joints = 3;
  ArmDynamics arm(num_joints);
  printf("Loading arm data..."); fflush(0);
  if (arm.load("data/arm3"))
    printf("done\n");
  else
    printf("FAILED\n");

  double RMSE_train[10];
  double RMSE_test[10];

  for (int i = 0; i < 10; i++) {

    printf("Sleeping for 3 seconds"); fflush(0);
    for (int xxx=0; xxx<3; xxx++) {
      printf("."); fflush(0);
      sleep(1);
    }
    printf("\n");

    // send swing command
    printf("Sending swing command...\n");
    double torque = 4 + 4*frand();  // uniform random between 4 and 8
    publish_constant_torque_traj(torque_traj_pub, torque);

    // wait for swing data
    printf("Waiting for swing data...\n");
    got_swing_data = false;
    while (!got_swing_data) {
      ros::spinOnce();
      poll_rate.sleep();
    }
    printf("Got swing data\n");

    if (i > 0) {
      // simulate trajectory
      printf("Simulating trajectory...\n");
      RMSE_test[i-1] = simulate_trajectory(swing_data, arm);
    }

    // add swing data to dynamics model
    printf("Adding swing to dynamics model...\n");
    arm.addSwingData(swing_data);

    if (i > 0) {
      // simulate trajectory
      printf("Simulating trajectory again...\n");
      RMSE_train[i-1] = simulate_trajectory(swing_data, arm);

      printf(" *** AVG. RMSE_train = %.4f ***\n", sum(RMSE_train,i) / (double)(i));
      printf(" *** AVG. RMSE_test  = %.4f ***\n", sum(RMSE_test,i) / (double)(i));
    }

  }

  printf("Saving arm data..."); fflush(0);
  if (arm.save("data/arm3"))
    printf("done\n");
  else
    printf("FAILED\n");

  return 0;
}
