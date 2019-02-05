
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <object_survey/ScanAnalysis.h>
#include <object_survey/SurveyScan.h>
#include <tf/transform_broadcaster.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>
#include <boost/foreach.hpp>
#include <pcl_tf/transforms.h>
#include <pcl/io/pcd_io.h>

using namespace std;


ros::Publisher pub;
tf::TransformBroadcaster *br;
const bool publish_aligned = false;
int cnt = 1;
FILE *f_views = NULL;



void process_survey_scan_msg(object_survey::SurveyScanConstPtr msg)
{
  // publish the narrow stereo point cloud
  pub.publish(msg->camera_scans[0].narrow_stereo_cloud);

  pcl::PointCloud<pcl::PointXYZRGB> cloud, aligned_cloud;
  pcl::fromROSMsg(msg->camera_scans[0].narrow_stereo_cloud, cloud);

  tf::Transform stereo_to_odom_orig;
  tf::transformMsgToTF(msg->camera_scans[0].narrow_stereo_transform.transform, stereo_to_odom_orig);

  br->sendTransform(msg->camera_scans[0].narrow_stereo_transform);

  cout << msg->camera_scans[0].narrow_stereo_transform << endl;

  float vp_x = msg->camera_scans[0].narrow_stereo_transform.transform.translation.x;
  float vp_y = msg->camera_scans[0].narrow_stereo_transform.transform.translation.y;
  float vp_z = msg->camera_scans[0].narrow_stereo_transform.transform.translation.z;
  float vp_qw = msg->camera_scans[0].narrow_stereo_transform.transform.rotation.w;
  float vp_qx = msg->camera_scans[0].narrow_stereo_transform.transform.rotation.x;
  float vp_qy = msg->camera_scans[0].narrow_stereo_transform.transform.rotation.y;
  float vp_qz = msg->camera_scans[0].narrow_stereo_transform.transform.rotation.z;

  fprintf(f_views, "camera_views(%d,:) = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f];\n",
	  cnt, vp_x, vp_y, vp_z, vp_qw, vp_qx, vp_qy, vp_qz);

  pcl::transformPointCloud(cloud, aligned_cloud, stereo_to_odom_orig);

  // output pcd file
  std::stringstream pcd_fname;
  pcd_fname << "cloud" << cnt++ << ".pcd";
  pcl::io::savePCDFile(pcd_fname.str(), aligned_cloud);
}



void process_scan_analysis_msg(object_survey::ScanAnalysisConstPtr msg)
{
  // publish the narrow stereo point cloud
  pub.publish(msg->camera_scans[0].narrow_stereo_cloud);

  pcl::PointCloud<pcl::PointXYZRGB> cloud, aligned_cloud;
  pcl::fromROSMsg(msg->camera_scans[0].narrow_stereo_cloud, cloud);

  tf::Transform stereo_to_odom_orig;
  tf::transformMsgToTF(msg->camera_scans[0].narrow_stereo_transform.transform, stereo_to_odom_orig);

  // publish the narrow_stereo -> odom_combined transform
  if (publish_aligned) {
    tf::Transform base_to_odom_orig, stereo_to_odom_aligned, base_to_odom_aligned;
    tf::transformMsgToTF(msg->base_transform.transform, base_to_odom_orig);
    tf::transformMsgToTF(msg->base_aligned.transform, base_to_odom_aligned);
    stereo_to_odom_aligned = base_to_odom_aligned * base_to_odom_orig.inverse() * stereo_to_odom_orig;
    geometry_msgs::TransformStamped stereo_to_odom_aligned_msg = msg->camera_scans[0].narrow_stereo_transform;
    tf::transformTFToMsg(stereo_to_odom_aligned, stereo_to_odom_aligned_msg.transform);

    br->sendTransform(stereo_to_odom_aligned_msg);

    cout << stereo_to_odom_aligned_msg << endl;

    float vp_x = stereo_to_odom_aligned_msg.transform.translation.x;
    float vp_y = stereo_to_odom_aligned_msg.transform.translation.y;
    float vp_z = stereo_to_odom_aligned_msg.transform.translation.z;
    fprintf(f_views, "camera_views(%d,:) = [%.4f, %.4f, %.4f];\n", cnt, vp_x, vp_y, vp_z);

    pcl::transformPointCloud(cloud, aligned_cloud, stereo_to_odom_aligned);
  }
  else {
    br->sendTransform(msg->camera_scans[0].narrow_stereo_transform);

    cout << msg->camera_scans[0].narrow_stereo_transform << endl;

    float vp_x = msg->camera_scans[0].narrow_stereo_transform.transform.translation.x;
    float vp_y = msg->camera_scans[0].narrow_stereo_transform.transform.translation.y;
    float vp_z = msg->camera_scans[0].narrow_stereo_transform.transform.translation.z;
    fprintf(f_views, "camera_views(%d,:) = [%.4f, %.4f, %.4f];\n", cnt, vp_x, vp_y, vp_z);

    pcl::transformPointCloud(cloud, aligned_cloud, stereo_to_odom_orig);
  }

  // output pcd file
  std::stringstream pcd_fname;
  pcd_fname << "cloud" << cnt++ << ".pcd";
  pcl::io::savePCDFile(pcd_fname.str(), aligned_cloud);
}


int main(int argc, char **argv)
{
  // init ROS
  ros::init(argc, argv, "publish_stereo_clouds");
  ros::NodeHandle nh;
  pub = nh.advertise<sensor_msgs::PointCloud2>("aligned_stereo_clouds", 1);
  br = new tf::TransformBroadcaster();
  f_views = fopen("views.m", "w");
  fprintf(f_views, "camera_views = [];\n");

  bool load_from_bag = false;
  if (argc >= 2)
    load_from_bag = true;

  if (load_from_bag) {
    ROS_INFO("Loading from bag file %s", argv[1]);
    rosbag::Bag bag(argv[1]);
    rosbag::View view(bag, rosbag::TopicQuery("object_survey/ScanAnalysis"), ros::TIME_MIN, ros::TIME_MAX);

    cout << "view.size() = " << view.size() << endl;  //dbug
    
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
      {
	object_survey::ScanAnalysisConstPtr msg = m.instantiate<object_survey::ScanAnalysis>();
	cout << msg << endl; //dbug
	if (msg != NULL)
	  process_scan_analysis_msg(msg);
      }
    bag.close();
  }
  else {
    ROS_INFO("Waiting for messages...");
    while (nh.ok()) {
      // Spin until we get a ScanAnalysis message
      if (publish_aligned) {
	object_survey::ScanAnalysisConstPtr msg =
	  ros::topic::waitForMessage<object_survey::ScanAnalysis>("analyzed_scans");
	process_scan_analysis_msg(msg);
      }
      else {
	object_survey::SurveyScanConstPtr msg =
	  ros::topic::waitForMessage<object_survey::SurveyScan>("/survey_results");
	process_survey_scan_msg(msg);
      }
    }
  }
}
   
