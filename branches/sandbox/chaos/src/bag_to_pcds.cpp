
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//#include <tf/transform_broadcaster.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>
#include <boost/foreach.hpp>
#include <pcl_tf/transforms.h>
#include <pcl/io/pcd_io.h>

using namespace std;


int cnt = 0;
//FILE *f_views = NULL;



void process_point_cloud_msg(sensor_msgs::PointCloud2ConstPtr msg)
{
  cnt++;
  //if (cnt%10)
  //  return;

  //pcl::PointCloud<pcl::PointXYZRGB> cloud; //, aligned_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(*msg, cloud);

  //tf::Transform stereo_to_odom_orig;
  //tf::transformMsgToTF(msg->camera_scans[0].narrow_stereo_transform.transform, stereo_to_odom_orig);

  //cout << msg->camera_scans[0].narrow_stereo_transform << endl;

  //float vp_x = msg->camera_scans[0].narrow_stereo_transform.transform.translation.x;
  //float vp_y = msg->camera_scans[0].narrow_stereo_transform.transform.translation.y;
  //float vp_z = msg->camera_scans[0].narrow_stereo_transform.transform.translation.z;
  //float vp_qw = msg->camera_scans[0].narrow_stereo_transform.transform.rotation.w;
  //float vp_qx = msg->camera_scans[0].narrow_stereo_transform.transform.rotation.x;
  //float vp_qy = msg->camera_scans[0].narrow_stereo_transform.transform.rotation.y;
  //float vp_qz = msg->camera_scans[0].narrow_stereo_transform.transform.rotation.z;

  //fprintf(f_views, "camera_views(%d,:) = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f];\n",
  //	  cnt, vp_x, vp_y, vp_z, vp_qw, vp_qx, vp_qy, vp_qz);

  //pcl::transformPointCloud(cloud, aligned_cloud, stereo_to_odom_orig);

  // output pcd file
  std::stringstream pcd_fname;
  pcd_fname << "cloud" << cnt << ".pcd";
  pcl::io::savePCDFile(pcd_fname.str(), cloud); //aligned_cloud);
}





int main(int argc, char **argv)
{
  //f_views = fopen("views.m", "w");
  //fprintf(f_views, "camera_views = [];\n");

  bool load_from_bag = false;
  if (argc >= 2)
    load_from_bag = true;

  if (load_from_bag) {
    ROS_INFO("Loading from bag file %s", argv[1]);
    rosbag::Bag bag(argv[1]);
    rosbag::View view(bag, rosbag::TopicQuery("/narrow_stereo_textured/points2"), ros::TIME_MIN, ros::TIME_MAX);

    cout << "view.size() = " << view.size() << endl;  //dbug
    
    int i = 0;
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
      {
	if ((i++) % 10 == 0) {
	  sensor_msgs::PointCloud2ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
	  if (msg != NULL) {
	    process_point_cloud_msg(msg);
	  }
	}
      }
    bag.close();
  }
  else {
    // init ROS
    ros::init(argc, argv, "bag_to_pcds");
    ros::NodeHandle nh;
    ROS_INFO("Waiting for messages...");
    while (nh.ok()) {
      // Spin until we get a ScanAnalysis message
      sensor_msgs::PointCloud2ConstPtr msg =
	ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/narrow_stereo_textured/points2");
      process_point_cloud_msg(msg);
    }
  }
}
   
