#include <ros/ros.h>
#include <std_msgs/Empty.h>


void ros_sleep(int seconds)
{
  for (int j = 0; j < seconds; j++) {
    ros::spinOnce();
    sleep(1);
  }
}


int main(int argc, char *argv[])
{
  // init ROS
  ros::init(argc, argv, "reload_swing_table");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Empty>("reload_swing_table", 1);

  ros_sleep(2);

  std_msgs::Empty msg;
  pub.publish(msg);

  ros_sleep(2);

  return 0;
}
