

#include <cardboard/common.h>
#include <cardboard/detection.h>
#include <cardboard/testing.h>
#include <cardboard/SceneHypothesis.h>
#include <cardboard/DetectModels.h>
#include <cardboard/AlignModels.h>
#include <cardboard/PointCloud.h>
#include <cardboard/AddModel.h>
#include <cardboard/util.h>

#include <pthread.h>

//dbug
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

using namespace cardboard;



//---------------------------- GLOBAL VARIABLES --------------------------//


ros::Publisher debug_pub;
ros::Publisher object_pub, alignment_pub;
ros::Publisher cloud_pub, alignment_cloud_pub;
tf::TransformListener *tf_listener;
sensor_msgs::PointCloud2 latest_cloud_msg;
ModelManager *model_manager;
double table_filter_thresh;

pthread_mutex_t cloud_mutex;
pthread_cond_t cloud_cond;
pthread_mutex_t model_manager_mutex;





//---------------------------- HELPER FUNCTIONS ---------------------------//


/* publish scene hypothesis message and accompanying point cloud */
void publish_scene_hypothesis(SceneHypothesis *H)
{
  // publish detected objects
  object_pub.publish(*H);

  // publish detected objects point cloud
  PointCloudXYZ cloud, objs_cloud;
  sensor_msgs::PointCloud2 objs_cloud_msg;
  for (size_t i = 0; i < H->objects.size(); i++) {
    model_manager->get_model_at_pose(H->objects[i].name, H->objects[i].pose, cloud);
    objs_cloud += cloud;
  }
  pcl::toROSMsg(objs_cloud, objs_cloud_msg);
  objs_cloud_msg.header = H->header;
  cloud_pub.publish(objs_cloud_msg);
}

void publish_alignment_hypothesis(SceneHypothesis *H)
{
  // publish detected objects
  alignment_pub.publish(*H);

  // publish detected objects point cloud
  PointCloudXYZ cloud, objs_cloud;
  sensor_msgs::PointCloud2 objs_cloud_msg;
  for (size_t i = 0; i < H->objects.size(); i++) {
    model_manager->get_model_at_pose(H->objects[i].name, H->objects[i].pose, cloud);
    objs_cloud += cloud;
  }
  pcl::toROSMsg(objs_cloud, objs_cloud_msg);
  objs_cloud_msg.header = H->header;
  alignment_cloud_pub.publish(objs_cloud_msg);
}


//---------------------- MESSAGE HANDLERS -----------------------//


/* handle a PointCloud2 message */
void pointcloud2_callback(sensor_msgs::PointCloud2 msg)
{
  //ROS_INFO("Got PointCloud2 msg with %u points\n", msg.width * msg.height);

  pthread_mutex_lock(&cloud_mutex);

  latest_cloud_msg = msg;

  pthread_cond_broadcast(&cloud_cond);
  pthread_mutex_unlock(&cloud_mutex);
}

void pointcloud_callback(sensor_msgs::PointCloud msg)
{
  sensor_msgs::PointCloud2 msg2;
  sensor_msgs::convertPointCloudToPointCloud2(msg, msg2);

  pointcloud2_callback(msg2);
}



//---------------------- SERVICE HANDLERS -----------------------//


/* handle an AddModel request */
bool add_model_handler(cardboard::AddModel::Request &req, cardboard::AddModel::Response &res)
{
  // detect models
  pthread_mutex_lock(&model_manager_mutex);
  PointCloudXYZ cloud;
  pcl::fromROSMsg(req.cloud, cloud);
  model_manager->add_model(req.name, cloud);
  pthread_mutex_unlock(&model_manager_mutex);

  res.success = true;

  return true;
}


/* handle a DetectModels request */
bool detect_models_handler(cardboard::DetectModels::Request &req, cardboard::DetectModels::Response &res)
{
  bool timed_out = false;

  // lock the cloud mutex
  pthread_mutex_lock(&cloud_mutex);

  if (req.header.stamp > latest_cloud_msg.header.stamp) {  // latest point cloud is too old

    if (req.timeout.sec <= 0 && req.timeout.nsec <= 0) {  // immediate timeout
      timed_out = true;
    }
    else {
      // wait for new cloud or timeout
      ROS_INFO("Waiting for a new point cloud");
      struct timeval now;
      struct timespec timeout;
      gettimeofday(&now, NULL);
      timeout.tv_sec = now.tv_sec + req.timeout.sec;
      timeout.tv_nsec = 1000*now.tv_usec + req.timeout.nsec;

      while (1) {
	pthread_cond_timedwait(&cloud_cond, &cloud_mutex, &timeout);

	cout << "req.header.stamp: " << req.header.stamp << ", " << "latest_cloud_msg.header.stamp: " << latest_cloud_msg.header.stamp << endl;

	if (req.header.stamp <= latest_cloud_msg.header.stamp)  // got a new point cloud
	  break;

	gettimeofday(&now, NULL);
	if (now.tv_sec > timeout.tv_sec || (now.tv_sec == timeout.tv_sec && 1000*now.tv_usec > timeout.tv_nsec)) {  // timeout
	  timed_out = true;
	  break;
	}
      }
    }

    if (timed_out) {
      ROS_WARN("DetectModels request timed out waiting for point cloud");
      pthread_mutex_unlock(&cloud_mutex);
      return false;
    }
  }

  // copy latest point cloud so we can release the lock
  sensor_msgs::PointCloud2 cloud_msg = latest_cloud_msg;
  pthread_mutex_unlock(&cloud_mutex);

  //dbug--copy model manager so we don't have to use a R/W lock for AddModels
  pthread_mutex_lock(&model_manager_mutex);
  ModelManager &M = *model_manager;
  pthread_mutex_unlock(&model_manager_mutex);

  // detect models
  SceneHypothesis *H = cardboard::detect_models(cloud_msg, req.header.frame_id, req.surface_polygons, tf_listener, table_filter_thresh, &M, req.models, debug_pub);

  if (H == NULL)
    return false;  // change to empty hypothesis?

  else {
    res.scene = *H;
    publish_scene_hypothesis(H);
  }

  return true;
}


bool align_models_handler(cardboard::AlignModels::Request &req, cardboard::AlignModels::Response &res){
  
  bool timed_out = false;
  
  // lock the cloud mutex
  pthread_mutex_lock(&cloud_mutex);
  
  if (req.header.stamp > latest_cloud_msg.header.stamp) {  // latest point cloud is too old
    
    if (req.timeout.sec <= 0 && req.timeout.nsec <= 0)  // immediate timeout
      timed_out = true;
    
    else {
      // wait for new cloud or timeout
      ROS_INFO("Waiting for a new point cloud");
      struct timeval now;
      struct timespec timeout;
      gettimeofday(&now, NULL);
      timeout.tv_sec = now.tv_sec + req.timeout.sec;
      timeout.tv_nsec = 1000*now.tv_usec + req.timeout.nsec;
      
      
      while (1) {
	pthread_cond_timedwait(&cloud_cond, &cloud_mutex, &timeout);
	
	cout << "req.header.stamp: " << req.header.stamp << ", " << "latest_cloud_msg.header.stamp: " << latest_cloud_msg.header.stamp << endl;
	
	if (req.header.stamp <= latest_cloud_msg.header.stamp)  // got a new point cloud
	  break;
	
	gettimeofday(&now, NULL);
	if (now.tv_sec > timeout.tv_sec || (now.tv_sec == timeout.tv_sec && 1000*now.tv_usec > timeout.tv_nsec)) {  // timeout
	  timed_out = true;
	  break;
	}
      }
    }
    
    
    if (timed_out) {
      ROS_WARN("Align request timed out waiting for point cloud");
      pthread_mutex_unlock(&cloud_mutex);
      return false;
    }
  }
  res.header = latest_cloud_msg.header;
  // copy latest point cloud so we can release the lock
  sensor_msgs::PointCloud2 cloud_msg = latest_cloud_msg;
  pthread_mutex_unlock(&cloud_mutex);
  
  //dbug--copy model manager so we don't have to use a R/W lock for AddModels
  pthread_mutex_lock(&model_manager_mutex);
  ModelManager &M = *model_manager;
  pthread_mutex_unlock(&model_manager_mutex);

  SceneHypothesis *H = cardboard::align_models(cloud_msg, req.header.frame_id, req.surface_polygons, req.initial_poses, req.models, tf_listener, table_filter_thresh, &M);

  for (uint i = 0; i < H->objects.size(); i++){
    res.aligned_poses.push_back(H->objects[i].pose);
    res.fitness_scores.push_back(H->objects[i].fitness_score);
  }
  
  if (H == NULL)
    return false;  // change to empty hypothesis?
  else{
    publish_alignment_hypothesis(H);
  }
  
  return true;
}  


bool point_cloud_handler(cardboard::PointCloud::Request &req, cardboard::PointCloud::Response &res)
{
  pthread_mutex_lock(&cloud_mutex);

  PointCloudXYZ cloud, cloud2;
  pcl::fromROSMsg(latest_cloud_msg, cloud);

  pthread_cond_broadcast(&cloud_cond);
  pthread_mutex_unlock(&cloud_mutex);

  //ROS_INFO("Downsampling point cloud with %lu points", .points.size());
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  grid.setLeafSize(req.resolution, req.resolution, req.resolution);
  grid.setInputCloud(cloud.makeShared());
  grid.filter(cloud2);

  pcl::toROSMsg(cloud2, res.cloud);

  return true;
}

  
  
//------------------- MAIN -------------------//

int main(int argc, char **argv)
{
  // init ROS
  ros::init(argc, argv, "object_detector_service");
  ros::NodeHandle nh;

  // services
  ros::ServiceServer align_models_service = nh.advertiseService("align_models", align_models_handler);
  ros::ServiceServer detect_models_service = nh.advertiseService("detect_models", detect_models_handler);
  ros::ServiceServer add_model_service = nh.advertiseService("add_model", add_model_handler);
  ros::ServiceServer point_cloud_service = nh.advertiseService("point_cloud", point_cloud_handler);

  // publishers (dbug)
  object_pub = nh.advertise<cardboard::SceneHypothesis>("scene_hypothesis", 1);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("scene_hypothesis_cloud", 1);
  alignment_pub = nh.advertise<cardboard::SceneHypothesis>("alignment_hypothesis", 1);
  alignment_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("alignment_hypothesis_cloud", 1);

  debug_pub = nh.advertise<sensor_msgs::PointCloud2>("debug_cloud", 1);

  // subscribers
  ros::Subscriber sub_cloud = nh.subscribe("pointcloud", 1, pointcloud_callback);
  ros::Subscriber sub_cloud2 = nh.subscribe("pointcloud2", 1, pointcloud2_callback);
  tf_listener = new tf::TransformListener;

  // parameters
  nh.param("/object_detector_service/table_filter_thresh", table_filter_thresh, .02);
  string objects_folder;
  if (nh.getParam("/object_detector_service/objects_folder", objects_folder))
    ROS_INFO("Got param: %s", objects_folder.c_str());
  else
    ROS_ERROR("Failed to get param 'objects_folder'");

  // model manager
  model_manager = new cardboard::ModelManager(objects_folder);

  // pthreads
  assert (pthread_mutex_init(&cloud_mutex, NULL) == 0);
  assert (pthread_cond_init(&cloud_cond, NULL) == 0);

  ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  spinner.spin(); // spin() will not return until the node has been shutdown

  //ros::spin();
  /*
  ros::Rate r(10);
  while (nh.ok()) {
    ros::spinOnce();
    r.sleep();
  }
  */
}
