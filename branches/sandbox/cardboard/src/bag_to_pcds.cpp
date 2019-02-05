

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
//#include <tf/transform_broadcaster.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>
#include <boost/foreach.hpp>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl_ros/io/bag_io.h>



using namespace std;
using namespace Eigen;



struct PointXYZRedGreenBlue
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  int red;
  int green;
  int blue;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRedGreenBlue,
				   (float, x, x)
				   (float, y, y)
				   (float, z, z)
				   (int, red, red)
				   (int, green, green)
				   (int, blue, blue)
				   )




char* fname;
int cnt = 0;
//FILE *f_views = NULL;

tf::TransformListener *tf_listener;
string target_frame;



pcl::PointCloud<PointXYZRedGreenBlue> RGB_to_RedGreenBlue(const pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
  pcl::PointCloud<PointXYZRedGreenBlue> cloud2;
  cloud2.width = cloud.width;
  cloud2.height = cloud.height;
  cloud2.is_dense = false;
  cloud2.points.resize(cloud.width * cloud.height);

  for (int i = 0; i < cloud.points.size(); i++) {
    cloud2.points[i].x = cloud.points[i].x;
    cloud2.points[i].y = cloud.points[i].y;
    cloud2.points[i].z = cloud.points[i].z;

    float rgb = cloud.points[i].rgb;
    int rgbi = *(int*)(&rgb);
    int red = (rgbi >> 16) & 0xFF;
    int green = (rgbi >> 8) & 0xFF;
    int blue = rgbi & 0xFF;


    cloud2.points[i].red = red;
    cloud2.points[i].green = green;
    cloud2.points[i].blue = blue;
  }

  return cloud2;
}


/* convert a tf transform to an eigen transform */
void transformTFToEigen(const tf::Transform &t, Matrix4f &k)
{
  for(int i=0; i<3; i++)
    {
      k(i,3) = t.getOrigin()[i];
      for(int j=0; j<3; j++)
	{
	  k(i,j) = t.getBasis()[i][j];
	}
    }
  // Fill in identity in last row
  for (int col = 0 ; col < 3; col ++)
    k(3, col) = 0;
  k(3,3) = 1;
}


void process_point_cloud2_msg(const sensor_msgs::PointCloud2 &msg)
{
  //cout << endl << msg.header.stamp << endl;
  //cout << endl << msg.header.frame_id << endl;
  cnt++;

  //pcl::PointCloud<pcl::PointXYZRGB> cloud_orig, cloud; //, aligned_cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud_orig, cloud; //dbug
  pcl::fromROSMsg(msg, cloud_orig);
  
  //sensor_msgs::PointCloud2 msg2 = *msg;
  
  cloud_orig.header.stamp = ros::Time(0);
  if (target_frame.length() > 0) {

    if (!pcl_ros::transformPointCloud(target_frame, cloud_orig, cloud, *tf_listener)) {
      ROS_ERROR("Can't transform point cloud; aborting...");
      exit(1);
    }

    tf::StampedTransform tf_transform;
    Matrix4f sensor_pose;
    try {
      //tf_listener->lookupTransform(msg.header.frame_id, target_frame, msg.header.stamp, tf_transform);
      tf_listener->lookupTransform(target_frame, msg.header.frame_id, msg.header.stamp, tf_transform);
      transformTFToEigen(tf_transform, sensor_pose);
    }
    catch (tf::TransformException& ex) {
      ROS_ERROR("TF exception:\n%s", ex.what());
      exit(1);
    }
    Matrix3f R = sensor_pose.topLeftCorner(3,3);
    Quaternionf q(R);
    Vector3f t = sensor_pose.topRightCorner(3,1);
    cloud.sensor_origin_(0) = t(0);
    cloud.sensor_origin_(1) = t(1);
    cloud.sensor_origin_(2) = t(2);
    cloud.sensor_orientation_ = q;
  }
  else {
    cloud = cloud_orig;
  }

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
  char pcd_fname[100];
  sprintf(pcd_fname, "cloud%03d.pcd", cnt);

  cout << endl << "Saving cloud " << cnt << endl;

  //pcl::PointCloud<PointXYZRedGreenBlue> cloud_rgb = RGB_to_RedGreenBlue(cloud);
  //pcl::io::savePCDFile(pcd_fname, cloud_rgb); //aligned_cloud);
  pcl::io::savePCDFile(pcd_fname, cloud); //aligned_cloud);
}

void point_cloud_callback(sensor_msgs::PointCloudConstPtr msg)
{
  sensor_msgs::PointCloud2 msg2;
  sensor_msgs::convertPointCloudToPointCloud2(*msg, msg2);

  process_point_cloud2_msg(msg2);
}




int main(int argc, char **argv)
{
  //f_views = fopen("views.m", "w");
  //fprintf(f_views, "camera_views = [];\n");

  bool load_from_bag = false;
  bool capture_scenes = false;
  int num_clouds = 0;
  int start = 0;
  if (argc == 4 && !strcmp(argv[1], "m")) {
    capture_scenes = true;
    num_clouds = atoi(argv[2]);
    start = atoi(argv[3]);
  }
  else if (argc >= 2)
    load_from_bag = true;

  if (load_from_bag) {
    ROS_INFO("Loading from bag file %s", argv[1]);
    rosbag::Bag bag(argv[1]);
    //rosbag::View view(bag, rosbag::TopicQuery("/narrow_stereo_textured/points2"), ros::TIME_MIN, ros::TIME_MAX);
    rosbag::View view(bag, rosbag::TopicQuery(argv[2])); //, ros::TIME_MIN, ros::TIME_MAX);

    cout << "view.size() = " << view.size() << endl;  //dbug
    
    int i = 0;
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
      {
	//if ((i++) % 10 == 0) {
	sensor_msgs::PointCloud2ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
	//sensor_msgs::PointCloudConstPtr msg = m.instantiate<sensor_msgs::PointCloud>();
	if (msg != NULL) {
	  process_point_cloud2_msg(*msg);
	  //printf("break 1\n");
	  //process_point_cloud_msg(*msg);
	  //printf("break 2\n");
	}
	//}
      }
    bag.close();
  }
  else if (capture_scenes) {
    cnt = start;
    ros::init(argc, argv, "bag_to_pcds");
    ros::NodeHandle nh;
    int i;
    char cmd[50];
    while (nh.ok()) {
      ROS_INFO("Waiting for input...");
      std::cin.getline(cmd, 50);
      for (i = 0; i < num_clouds; ++i) {
	sensor_msgs::PointCloud2ConstPtr cloud_blob_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points");
	sensor_msgs::PointCloud2 msg = *cloud_blob_ptr;
	process_point_cloud2_msg(msg);
      }
    }
  } else {

    // init ROS
    ros::init(argc, argv, "bag_to_pcds");
    ros::NodeHandle nh;
    ROS_INFO("Waiting for messages...");

    tf_listener = new tf::TransformListener;

    //ros::Subscriber sub_cloud = nh.subscribe("points", 100, point_cloud_callback);
    ros::Subscriber sub_cloud2 = nh.subscribe("/camera/depth_registered/points", 100, process_point_cloud2_msg);

    nh.param<string>("/bag_to_pcds/target_frame", target_frame, "");
    cout << "target_frame=" << target_frame << endl;

    ros::spin();

    //while (nh.ok()) {

    // Spin until we get a ScanAnalysis message
    //sensor_msgs::PointCloud2ConstPtr msg =
    //ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/narrow_stereo_textured/points2");
    //process_point_cloud2_msg(msg);

    //sensor_msgs::PointCloudConstPtr msg =
    //  ros::topic::waitForMessage<sensor_msgs::PointCloud>("points");
    //point_cloud_callback(msg);
    //}

  }
}
   
