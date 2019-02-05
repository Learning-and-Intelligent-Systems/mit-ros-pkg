#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <fastwam/MoveTo.h>


#define TABLE_WIDTH 1.525
#define METERS_PER_INCH .0254
const double ARM_OFFSET_X = (TABLE_WIDTH - 9.5*METERS_PER_INCH);
const double ARM_OFFSET_Y = 19*METERS_PER_INCH;
const double ARM_OFFSET_Z = 21.5*METERS_PER_INCH;

const double ARM_LINK1 = 23*METERS_PER_INCH;
const double ARM_LINK2 = 19*METERS_PER_INCH;


ros::Publisher wam_moveto_pub;




void move_paddle_2d(double x, double z)
{
  if (x < .5)
    return;

  // solve inverse kinematics in the 2d plane (joint 1 and joint 3)
  double r1 = ARM_LINK1;
  double r2 = ARM_LINK2;

  double theta2 = -2*atan(sqrt( ((r1+r2)*(r1+r2) - (x*x + z*z)) / (x*x+z*z - (r1-r2)*(r1-r2)) ));

  double a = atan2(z,x);
  double b = atan2(r2*sin(theta2), r1 + r2*cos(theta2));
  double theta1 = a - b;

  if (isnan(theta1) || isnan(theta2))
    return;

  double x2 = r1*cos(theta1) + r2*cos(theta1 + theta2);
  double z2 = r1*sin(theta1) + r2*sin(theta1 + theta2);

  printf("(x,z) = (%.3f, %.3f), (theta1,theta2) = (%.2f, %.2f), (x2,z2) = (%.3f, %.3f)\n", x, z, theta1, theta2, x2, z2);

  fastwam::MoveTo msg;
  msg.joint_angles[0] = 1.57;
  msg.joint_angles[1] = -theta1;
  msg.joint_angles[3] = -theta2;
  msg.joint_angles[4] = 1.57;
  msg.joint_angles[5] = .5;

  wam_moveto_pub.publish(msg);
}


// predicted ball trajectory
void ball_trajectory_callback(const sensor_msgs::PointCloud &msg)
{
  int n = msg.points.size();

  // don't move if no ball is found or if ball is moving away from the robot
  if (n < 5 || msg.points[n-1].y > msg.points[n-5].y)
    return;

  // find out where predicted trajectory intersects robot arm plane
  int ihit = -1;
  for (uint i = 0; i < msg.points.size(); i++) {
    if (msg.points[i].y < -ARM_OFFSET_Y) {
      ihit = i;
      break;
    }
  }
      
  if (ihit >= 0)
    move_paddle_2d(ARM_OFFSET_X - msg.points[ihit].x, msg.points[ihit].z - ARM_OFFSET_Z);
}

int main(int argc, char *argv[])
{
  // init ROS
  ros::init(argc, argv, "pingpongbrain");
  ros::NodeHandle nh;
  ros::Subscriber ball_sub = nh.subscribe("ball_trajectory_prediction", 1, ball_trajectory_callback);
  wam_moveto_pub = nh.advertise<fastwam::MoveTo>("/fastwam/moveto", 1);

  ros::spin();

  return 0;
}
