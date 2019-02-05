#include <ros/ros.h>
#include <fastwam/MoveTo.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "wamtest_client");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<fastwam::MoveTo>("/fastwam/moveto", 10);

  fastwam::MoveTo msg;
  for (int i = 0; i < 7; i++)
    msg.joint_angles[i] = 0;

  ros::Rate poll_rate(100);
  while(pub.getNumSubscribers() == 0)
    poll_rate.sleep();
  
  sleep(1);

  pub.publish(msg);

  //for (int i = 0; i < 100; i++)
  //  ros::spinOnce();

  //for (int i = 0; i < 100; i++)
  //  ros::spinOnce();
  ros::spin();

  return 0;
}
