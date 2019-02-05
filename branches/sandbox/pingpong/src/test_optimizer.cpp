#include <ros/ros.h>
#include <bingham/util.h>
#include <fastwam/SwingData.h>
//#include <pingpong/SwingData.h>
#include <fastwam/TorqueTrajectory.h>
#include "dynamics.h"
#include "optimizer.h"
#include <eigen2/Eigen/Core>
#include <eigen2/Eigen/LU>
#include <eigen2/Eigen/Array>
#include <time.h>
#include <iostream>

using namespace std;
using namespace fastwam;
//using namespace pingpong;
USING_PART_OF_NAMESPACE_EIGEN;

int joint_num;
SwingData swing_data;
bool got_swing_data;
VectorXf current_state;

double max_joint_angles [3] = {2.6, 2.0, 2.8};
double max_joint_velocity = 1.0; //??

// give random vector within safe range
VectorXf random_state(int njoints) 
{
  VectorXf v = VectorXf(njoints);
  for(int i = 0; i < njoints; i++) {
    v(i) = rand()/RAND_MAX * max_joint_angles[i];
    v(njoints+i) = rand()/RAND_MAX * max_joint_velocity;
  }
  return v;
}

void swingdata_callback(const SwingData &msg)
{
  swing_data = msg;
  got_swing_data = true;

  /*  // update current_state vector
  for (int i = 0; i < joint_num; i++) {
    double dt = swing_data.t[1] - swing_data.t[0];
    current_state(i) = swing_data.joint_angles[joint_num + i];
    current_state(joint_num + i) = (current_state(i) - swing_data.joint_angles[i]) / dt;
  }
  */
}

/*
void nextstate_callback(const NextState &msg)
{
  // convert msg data into MatrixXf

  while(!got_data) {
    ros::spinOnce();
    poll_rate.sleep();
  }

  //convert current_state into MatrixXf
  //figure out velocity
  //use optimizer to find torques

  publish_torque_traj(torque_traj_pub, torques);

}
*/

void publish_constant_torque_traj(ros::Publisher &pub, int joint, double torque, double dt)
{
  fastwam::TorqueTrajectory msg;

  msg.joints.push_back(joint);

  double t = ros::Time::now().toSec();
  double duration = 5;
  msg.times.push_back(t + duration);

  msg.torques.push_back(torque);
  msg.torques.push_back(torque);

  pub.publish(msg);
}



void publish_torque_traj(ros::Publisher &pub, MatrixXf torques, double dt)
{
  fastwam::TorqueTrajectory msg;
  int N = torques.row(0).size();

  // joints controlled
  for (int i = 0; i < joint_num; i++) {
    msg.joints.push_back(i);
  }

  //time
  double t = ros::Time::now().toSec();
  for (int i = 0; i < N; i++) {
    msg.times.push_back(t+i*dt);
  }

  // torques to be applied
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < joint_num; j++) {
      msg.torques.push_back(torques(j, i));
    }
  }

  pub.publish(msg);

  /*
  double torque = 5.0;

  fastwam::TorqueTrajectory msg;
  double duration = 5;
  double t = ros::Time::now().toSec();

  msg.joints.push_back(1);

  msg.times.push_back(t);
  msg.times.push_back(t+duration);

  msg.torques.push_back(torque);
  msg.torques.push_back(torque);

  pub.publish(msg);
  */
}


int main(int argc, char *argv[])
{
  /*
  //init ROS
  ros::init(argc, argv, "test_optimizer");
  ros::NodeHandle nh;
  ros::Rate poll_rate(10);
  ros::Publisher torque_traj_pub = nh.advertise<fastwam::TorqueTrajectory>("/fastwam/torque_trajectory", 1);
  ros::Subscriber sub_swingdata = nh.subscribe("/fastwam/swingdata", 1, swingdata_callback);
  //ros::Subscriber sub_nextstate = nh.subscribe("/fastwam/nextstate", 1, nextstate_callback);

  srand( time(NULL) );
  */
  joint_num = 3;
  TrajectoryOptimizer opt(joint_num);
  ArmDynamics arm(3);
  //arm.load("data/arm3");

  double T = .2, dt = 0.01;
  int N = T / dt;
  MatrixXf torques = MatrixXf::Zero(joint_num, N);
  current_state = VectorXf::Zero(2*joint_num);
  VectorXf desired_state = VectorXf::Zero(2*joint_num);
  desired_state(1) = 0.07; // joint 1 angle
  desired_state(4) = 1.0; // joint 1 velocity

  opt.test();
  /*

  for(int iter = 0; iter < 3; iter++) {

    printf("Sleeping for 3 seconds"); fflush(0);
    for (int xxx=0; xxx<3; xxx++) {
      printf("."); fflush(0);
      sleep(1);
    }
    printf("\n\n");

    opt.find_opt(torques, arm, current_state, desired_state, T, dt);

    MatrixXf commands = MatrixXf::Zero(joint_num, 300);
    commands.block(0, 0, joint_num, N) = torques;
    double random_torque = 4 + 4*frand();
    commands.block(1, N+1, joint_num, 200) = random_torque * MatrixXf::Ones(joint_num, 200);

    // not sure if this will work
    publish_torque_traj(torque_traj_pub, commands, dt);
    while(!got_swing_data) {
      ros::spinOnce();
      poll_rate.sleep();
    }
    // print swing data from "optimized" portion
    printf("swing data:\n");
    for (int i = 0; i < joint_num; i++) {
      for(int j = 0; j < N; j++) {
	cout<< std::setprecision(4) << swing_data.joint_angles[7 * j + i]<<" ";
      }
      printf("\n\n");
    }
    printf("adding swing data\n");
    arm.addSwingData(swing_data);
    printf("swing data added\n\n");    

  }
    
  printf("Saving arm data...\n"); fflush(0);
  arm.save("data/arm3");

  */

  /*
  for (int i = 0; i < 10; i++) {

    got_swing_data = false;
    while(!got_swing_data) {
      ros::spinOnce();
      poll_rate.sleep();
    }

    arm.addSwingData(swing_data);

    // default current state

    opt.find_opt(torques, arm, current_state, desired_state, T, dt);
    publish_torque_traj(torque_traj_pub, torques, dt);

  }
  */
  //arm.save("data/arm3");

}

 /*
int main() {

  int joint_num = 1;
  TrajectoryOptimizer opt(joint_num);
  ArmDynamics arm(joint_num);
  arm.load("data/arm3");
  MatrixXf torques;

  opt.test();

}
 */
