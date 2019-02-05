#include <ros/ros.h>
#include <fastwam/SwingData.h>
#include <fastwam/TorqueTrajectory.h>

using namespace fastwam;


void publish_constant_torque_traj(ros::Publisher &pub, double torque)
{
  TorqueTrajectory msg;

  double duration = 10;

  double t = ros::Time::now().toSec();

  msg.joints.push_back(1);

  msg.times.push_back(t);
  msg.times.push_back(t + duration);

  msg.torques.push_back(torque);
  msg.torques.push_back(torque);

  pub.publish(msg);
}


int main(int argc, char* argv[])
{
  // init ROS
  ros::init(argc, argv, "torque_test");
  ros::NodeHandle nh;

  // advertise messages
  ros::Publisher torque_traj_pub = nh.advertise<fastwam::TorqueTrajectory>("/fastwam/torque_trajectory", 1);

  // subscribe to messages
  //ros::Subscriber sub_swingdata = nh.subscribe("/fastwam/swingdata", 1, swingdata_callback);

  double torque = atof(argv[1]);

  ros::Rate poll_rate(100);
  while(torque_traj_pub.getNumSubscribers() == 0)
    poll_rate.sleep();
  //sleep(.5);

  publish_constant_torque_traj(torque_traj_pub, torque);

  ros::spin();

  return 0;
}
