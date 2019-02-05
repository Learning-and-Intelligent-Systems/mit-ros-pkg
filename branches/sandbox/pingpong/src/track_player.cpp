#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <bingham/util.h>

double expected_player_position[3] = {0.4252, -0.3956, 2.8179};
double table_x_axis[3] = {0.9991,  0.0433,  0.0000};
double table_y_axis[3] = {0.0251, -0.5803,  0.8140};
double table_z_axis[3] = {0.0352, -0.8132, -0.5809};
double table_origin[3] = {-0.4170, 1.7194, 0.6663};


tf::TransformListener *listener;
ros::Publisher rh_pub;
ros::Publisher rh_vel_pub;

int buflen = 5;
double rh_buf[5][3];
double rh_buf_times[5] = {0,0,0,0,0};
int rh_buf_idx = 0;
double rh_vel[3] = {0,0,0};





void transform_camera_to_table(double *p2, double *p)
{
  double p1[3];
  sub(p1, p, table_origin, 3);

  p2[0] = dot(table_x_axis, p1, 3);
  p2[1] = dot(table_y_axis, p1, 3);
  p2[2] = dot(table_z_axis, p1, 3);
}


int get_user_id()
{
  static char frame[128];
  tf::StampedTransform transform;

  int id = -1;

  for (int i = 1; i < 10; i++) {
    sprintf(frame, "torso_%d", i);
    try {
      listener->lookupTransform("openni_depth_frame", frame, ros::Time(0), transform);
      double x = -transform.getOrigin().y();
      double y = -transform.getOrigin().z();
      double z = transform.getOrigin().x();
      double d[3] = {x - expected_player_position[0], y - expected_player_position[1], z - expected_player_position[2]};
      //printf("Found torso %d at (%.2f, %.2f, %.2f)\n", i, x, y, z);
      if (norm(d,3) < 1.0)
	id = i;
    }
    catch (tf::TransformException ex) {
      //ROS_ERROR("%s",ex.what());
    }
  }

  return id;
}


void publish_player(int id)
{
  static char frame[128];
  tf::StampedTransform transform;

  try {
    geometry_msgs::PointStamped rh;
    sprintf(frame, "right_hand_%d", id);
    listener->lookupTransform("openni_depth_frame", frame, ros::Time(0), transform);
    double p[3] = {-transform.getOrigin().y(), -transform.getOrigin().z(), transform.getOrigin().x()};
    transform_camera_to_table(p, p);
    rh.point.x = p[0];
    rh.point.y = p[1];
    rh.point.z = p[2];
    rh.header.stamp = transform.stamp_;
    rh_pub.publish(rh);
    printf("Publishing right hand %d at (%.2f, %.2f, %.2f)\n", id, rh.point.x, rh.point.y, rh.point.z);

    // update rh velocity
    int i0 = (rh_buf_idx-3) % buflen;
    double t0 = rh_buf_times[i0];
    int i = rh_buf_idx;
    double t = rh.header.stamp.toSec();
    if (t - t0 < .001)
      return;

    rh_buf[i][0] = p[0];
    rh_buf[i][1] = p[1];
    rh_buf[i][2] = p[2];
    rh_buf_times[rh_buf_idx] = t;
    if (t0 > t - 1.0) {
      for (int j = 0; j < 3; j++) {
	double v = (rh_buf[i][j] - rh_buf[i0][j]) / (t-t0);
	rh_vel[j] = .25*v + .75*rh_vel[j];
      }
      // publish rh velocity
      geometry_msgs::PointStamped rh_vel_msg;
      rh_vel_msg.point.x = rh_vel[0];
      rh_vel_msg.point.y = rh_vel[1];
      rh_vel_msg.point.z = rh_vel[2];
      rh_vel_msg.header.stamp = transform.stamp_;
      rh_vel_pub.publish(rh_vel_msg);
      printf("Publishing right hand velocity (%.2f, %.2f, %.2f)\n", rh_vel_msg.point.x, rh_vel_msg.point.y, rh_vel_msg.point.z);
    }
    rh_buf_idx = (rh_buf_idx+1) % buflen;
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
  }

}


//----------------------- MAIN --------------------//

int main(int argc, char *argv[])
{
  // init ROS
  ros::init(argc, argv, "pingpongtrackplayer");
  ros::NodeHandle nh;

  rh_pub = nh.advertise<geometry_msgs::PointStamped>("right_hand", 1);
  rh_vel_pub = nh.advertise<geometry_msgs::PointStamped>("right_hand_velocity", 1);
  listener = new tf::TransformListener();

  ros::Rate rate(30.0);
  while (nh.ok()){
    rate.sleep();

    int id = get_user_id();
    if (id > 0)
      publish_player(id);
  }
  ros::spin();

  return 0;
}
