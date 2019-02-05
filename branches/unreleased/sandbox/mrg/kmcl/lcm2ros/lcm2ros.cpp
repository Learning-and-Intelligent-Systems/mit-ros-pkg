#include <stdio.h>
#include <signal.h>
#include <sys/types.h>
#include <dirent.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <cstdio> 
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <fstream>

// lcm and visualization:
#include <lcm/lcm.h>
#include <bot_core/rotations.h>
#include <bot_core/trans.h>
#include <lcmtypes/bot_core.h>
#include <lcmtypes/ps_pf_cloud_t.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include <sensor_msgs/LaserScan.h>

#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>


using namespace std;

typedef struct  {
  lcm_t* publish_lcm;
  lcm_t* subscribe_lcm;

  ros::Duration transform_tolerance_;
  tf::Transform latest_tf_;
  tf::TransformBroadcaster broadcaster;
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformBroadcaster tfb_;
  ros::Publisher particlecloud_pub;
  ros::Publisher obstaclescan_pub; 
  
  //lcm reading thread stuff
  pthread_t processor_thread;
  pthread_mutex_t lcm_data_mutex;
  pthread_cond_t newLcmData_cv;
  
  BotTrans base2odom; 
  double base2odom_yaw;
} state_t;

// signal handler
sig_atomic_t shutdown_flag = 0;
static void sig_action(int signal, siginfo_t *s, void *user){
  fprintf(stderr,"Shutting Down!\n");
  shutdown_flag = 1;
}


void on_pf_mean(const lcm_recv_buf_t *rbuf, const char *channel,
    const bot_core_pose_t *msg, void *user_data)
{
  state_t* state = static_cast<state_t*>(user_data);
  double rpy_internal[] = {0,0,0}; 
  bot_quat_to_roll_pitch_yaw(msg->orientation,rpy_internal);

// given a and c find b = a (-) c
// a = map->base [computed by particle filter]
// b = map->odom [made here]
// c = odom->base[produced by move base]
//c=-c
//b_th_calc = a_th - c_th
//[c_vt,c_vr]= cart2pol(c(1),c(2));
//c_vt_adjust = c_vt + b_th_calc;%a_th;
//[c_dash(1),c_dash(2)]= pol2cart(c_vt_adjust,c_vr);
//b_calc= a + c_dash
//
cout << "a = [" << msg->pos[0] << ", " << msg->pos[1] << "]\n";
cout << "a_th = " << rpy_internal[2]<<"\n";
cout << "c = [" << state->base2odom.trans_vec[0] << ", " << state->base2odom.trans_vec[1] << "]\n";
cout << "c_th = " << state->base2odom_yaw<<"\n";


double c[2];
c[0] = -state->base2odom.trans_vec[0];
c[1] = -state->base2odom.trans_vec[1];

double b_th_calc = rpy_internal[2] - state->base2odom_yaw;
double c_vt = atan2(c[1],c[0]);
double c_vr =  sqrt( pow(c[0],2)+ pow(c[1],2));
double c_vt_adjust = c_vt + b_th_calc;
double c_dash[2];
c_dash[0] = c_vr*cos(c_vt_adjust);
c_dash[1] = c_vr*sin(c_vt_adjust);

double b_calc[2];
b_calc[0] = msg->pos[0] + c_dash[0];
b_calc[1] = msg->pos[1] + c_dash[1];

cout << "b = [" << b_calc[0] << ", " << b_calc[1] << "]\n";
cout << "b_th = " << b_th_calc<<"\n";


  geometry_msgs::Quaternion mean_quatb =  tf::createQuaternionMsgFromYaw(b_th_calc);

  tf::Quaternion mean_quat = tf::Quaternion(mean_quatb.x, 
    mean_quatb.y,mean_quatb.z,mean_quatb.w);

  //tf::Vector3 mean_pos = tf::Vector3(msg->pos[0], msg->pos[1], msg->pos[2]);
//  tf::Vector3 mean_pos = tf::Vector3(kmclTrans.trans_vec[0], kmclTrans.trans_vec[1], kmclTrans.trans_vec[2]);
  tf::Vector3 mean_pos = tf::Vector3(b_calc[0], b_calc[1], 0);

  state->latest_tf_ = tf::Transform(mean_quat,mean_pos);

  // We want to send a transform that is good up until a
  // tolerance time so that odom can be used
  ros::Time transform_expiration = (ros::Time::now() +
				  state->transform_tolerance_);
  tf::StampedTransform tmp_tf_stamped(state->latest_tf_,
				    transform_expiration,
				    "map", "odom");
/*    tf::StampedTransform tmp_tf_stamped(state->latest_tf_.inverse(),
				    transform_expiration,
				    "map", "odom");*/
  state->tfb_.sendTransform(tmp_tf_stamped);  
  cout << "sent map to odom tf " << ros::Time::now() <<"\n";
  return;  
}

void on_pf_state(const lcm_recv_buf_t *rbuf, const char *channel,
    const ps_pf_cloud_t *msg, void *user_data)
{
  state_t* state = static_cast<state_t*>(user_data);

  // Publish the particle cloud
  // TODO: set maximum rate for publishing
  geometry_msgs::PoseArray cloud_msg;
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = "map";
  cloud_msg.poses.resize(msg->nparticles);
  for(int i=0;i<msg->nparticles;i++){
    double rpy_internal[] = {0,0,0}; 
    bot_quat_to_roll_pitch_yaw(msg->particles[i].orientation,
    rpy_internal);        

    tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(rpy_internal[2]),
    btVector3(msg->particles[i].pos[0],
    msg->particles[i].pos[1], msg->particles[i].pos[2])),
    cloud_msg.poses[i]);

    /*      tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(msg->particles[i].pos[2]),
    btVector3(msg->particles[i].pos[0],
    msg->particles[i].pos[1], 0)),
    cloud_msg.poses[i]);*/
  }
  state->particlecloud_pub.publish(cloud_msg);

  //cout << "sent particlecloud " << ros::Time::now() <<"\n";
  return;
}

void on_obstacle_lidar(const lcm_recv_buf_t *rbuf, const char *channel,
    const bot_core_planar_lidar_t *msg, void *user_data)
{
  state_t* state = static_cast<state_t*>(user_data);

  //populate the LaserScan message
  ros::Time scan_time = ros::Time::now();
  sensor_msgs::LaserScan scan;
  scan.header.stamp = scan_time;
  scan.header.frame_id = "/base_laser";
  scan.angle_min = msg->rad0;
  scan.angle_max = msg->rad0 + msg->radstep*msg->nranges;
  scan.angle_increment = msg->radstep;
  //scan.time_increment = (1 / laser_frequency) / (num_readings);
  scan.time_increment = (1 / 10) / (msg->nranges); // a guess... we dont have this
  scan.range_min = 0.0;
  scan.range_max = 30.0;
  scan.set_ranges_size(msg->nranges);
  scan.set_intensities_size(0);
  for(unsigned int i = 0; i < msg->nranges; ++i){
    scan.ranges[i] = msg->ranges[i]; 
    //scan.intensities[i] = intensities[i];
  }
  state->obstaclescan_pub.publish(scan);
  cout << "obstacle scan published: " << scan_time<< "\n";
  return;
}


void transformPoint(const tf::TransformListener& listener, void * user){
  state_t * state = (state_t *) user;
  ros::Time t= ros::Time::now();  

	tf::StampedTransform transform;
       try{
         listener.lookupTransform("/odom", "/base",  
                                  ros::Time(0), transform);
       }
       catch (tf::TransformException ex){
         ROS_ERROR("%s",ex.what());
       }  

  	tf::Quaternion base2odom_quat = tf::Quaternion(transform.getRotation().x(), 
  	  transform.getRotation().y(),transform.getRotation().z(),transform.getRotation().w());

       
       state->base2odom.trans_vec[0] = transform.getOrigin().x();
       state->base2odom.trans_vec[1] = transform.getOrigin().y();
       state->base2odom.trans_vec[2] = transform.getOrigin().z();
       state->base2odom.rot_quat[2] = transform.getRotation().x();
       state->base2odom.rot_quat[1] = transform.getRotation().y();
       state->base2odom.rot_quat[0] = transform.getRotation().z();
       state->base2odom.rot_quat[3] = transform.getRotation().w();
	state->base2odom_yaw = tf::getYaw(base2odom_quat);


		

       cout << t << " | xyz " << transform.getOrigin().x() << ", "<< transform.getOrigin().y() << ", "<< transform.getOrigin().z() 
	    << " | xyzw " << transform.getRotation().x() << ", "<< transform.getRotation().y() << ", "<< transform.getRotation().z() << ", " << transform.getRotation().w() 
	    << " | yaw " << state->base2odom_yaw 
 	    << "\n";
/*  tf::Stamped<tf::Pose> ident (btTransform(tf::createIdentityQuaternion(),
                                           btVector3(0,0,0)), t, "/base");
  
  tf::Stamped<tf::Pose> odom_pose;
  listener.transformPose("/odom",ident,odom_pose);*/
}

//dispatcher for new freenect data
static void * processingFunc(void * user)
{
  state_t * state = (state_t *) user;

  pthread_mutex_lock(&state->lcm_data_mutex);
  int x=0;
  char dog[3][3];
  char **cat;
  ros::init(x, cat, "robot_tf_listener");
  ros::NodeHandle n;
  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once 0.1 second
  ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&transformPoint, boost::ref(listener),state));
  
  ros::spin();
  
  pthread_mutex_unlock(&state->lcm_data_mutex);  
}

int main (int argc, char** argv)
{
  cout << "Started lcm2ros\n";
  ros::init(argc, argv, "lcm2ros");
  ros::NodeHandle nh;

  state_t* state = new state_t();
  state->particlecloud_pub = nh.advertise<geometry_msgs::PoseArray>("particlecloud", 2);
  state->obstaclescan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 50);
  state->transform_tolerance_.fromSec(0.1);
  state->publish_lcm= lcm_create(NULL);
  state->subscribe_lcm = state->publish_lcm;
  

  //setup lcm reading thread
  pthread_mutex_init(&state->lcm_data_mutex, NULL);
  pthread_cond_init(&state->newLcmData_cv, NULL);
  //create processing thread
  pthread_create(&state->processor_thread, 0, (void *
  (*)(void *)) processingFunc, (void *) state);
  

  ////////////////////////////////////////////////////////////////////
  ps_pf_cloud_t_subscribe(state->subscribe_lcm, "PARTICLE_CLOUD", on_pf_state, state);
  bot_core_pose_t_subscribe(state->subscribe_lcm, "PARTICLE_MEAN",on_pf_mean, state);
  bot_core_planar_lidar_t_subscribe(state->subscribe_lcm,"KINECT_LIDAR",on_obstacle_lidar,state); 
  
  
  // Register signal handlers so that we can
  // exit cleanly when interrupted
  struct sigaction new_action;
  new_action.sa_sigaction = sig_action;
  sigemptyset(&new_action.sa_mask);
  new_action.sa_flags = 0;
  sigaction(SIGINT, &new_action, NULL);
  sigaction(SIGTERM, &new_action, NULL);
  sigaction(SIGHUP, &new_action, NULL);  
  
  // go!
//  while(0 == lcm_handle(state->subscribe_lcm) );
  while(0 == lcm_handle(state->subscribe_lcm) && !shutdown_flag);

  return 0;   
}
