

#include <chaos/common.h>
#include <chaos/detection.h>
#include <chaos/testing.h>
#include <chaos/SceneAnalysis.h>
#include "util.h"


//---------------------------- GLOBAL VARIABLES --------------------------//

string target_frame("/base_footprint");
ros::Publisher object_pub;
ros::Publisher cloud_pub;
tf::TransformListener *tf_listener;
geometry_msgs::Polygon table_polygon;
bool have_table = false;
chaos::ModelManager model_manager;



//---------------------------- HELPER FUNCTIONS ---------------------------//

/* convert a tf transform to an eigen3 transform */
void transformTFToEigen3(const tf::Transform &t, Affine3f &k)
{
  for(int i=0; i<3; i++)
    {
      k.matrix()(i,3) = t.getOrigin()[i];
      for(int j=0; j<3; j++)
	{
	  k.matrix()(i,j) = t.getBasis()[i][j];
	}
    }
  // Fill in identity in last row
  for (int col = 0 ; col < 3; col ++)
    k.matrix()(3, col) = 0;
  k.matrix()(3,3) = 1;
}


/* project a point cloud into the XY plane */
void flatten_point_cloud(const PointCloudXYZ &cloud_in, PointCloudXYZ &cloud_out)
{
  cloud_out = cloud_in;
  for (size_t i = 0; i < cloud_out.points.size(); i++)
    cloud_out.points[i].z = 0;
}


/** \brief Check if a 2d point (X and Y coordinates considered only!) is inside or outside a given polygon. This 
  * method assumes that both the point and the polygon are projected onto the XY plane.
  * \note (This is highly optimized code taken from http://www.visibone.com/inpoly/)
  *       Copyright (c) 1995-1996 Galacticomm, Inc.  Freeware source code.
  * \param point a 3D point projected onto the same plane as the polygon
  * \param polygon a polygon
  */
static bool isXYPointIn2DXYPolygon (const pcl::PointXYZ &point, const geometry_msgs::Polygon &polygon)
{
  bool in_poly = false;
  double x1, x2, y1, y2;

  int nr_poly_points = polygon.points.size ();
  double xold = polygon.points[nr_poly_points - 1].x;
  double yold = polygon.points[nr_poly_points - 1].y;
  for (int i = 0; i < nr_poly_points; i++)
  {
    double xnew = polygon.points[i].x;
    double ynew = polygon.points[i].y;
    if (xnew > xold)
    {
      x1 = xold;
      x2 = xnew;
      y1 = yold;
      y2 = ynew;
    }
    else
    {
      x1 = xnew;
      x2 = xold;
      y1 = ynew;
      y2 = yold;
    }

    if ( (xnew < point.x) == (point.x <= xold) && (point.y - y1) * (x2 - x1) < (y2 - y1) * (point.x - x1) )
    {
      in_poly = !in_poly;
    }
    xold = xnew;
    yold = ynew;
  }
  return (in_poly);
}


/* filter a point cloud based on a polygon in the XY plane */
static void polygon_filter(const geometry_msgs::Polygon &polygon, const PointCloudXYZ &cloud_in, PointCloudXYZ &cloud_out)
{
  cloud_out.height = 1;
  cloud_out.is_dense = false;
  cloud_out.points.resize(0);

  for (size_t i = 0; i < cloud_in.points.size(); i++) {
    if (isXYPointIn2DXYPolygon(cloud_in.points[i], polygon))
      cloud_out.push_back(cloud_in.points[i]);
  }

  cloud_out.width = cloud_out.points.size();
}


/* convert a Polygon to a PointCloud (unused)
static PointCloudXYZ polygon_to_point_cloud(const geometry_msgs::Polygon &polygon)
{
  PointCloudXYZ cloud;
  cloud.width = polygon.points.size();
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  for (size_t i = 0; i < polygon.points.size(); i++) {
    cloud.points[i].x = polygon.points[i].x;
    cloud.points[i].y = polygon.points[i].y;
    cloud.points[i].z = polygon.points[i].z;
  }

  return cloud;
}
*/



//---------------------- MESSAGE HANDLERS -----------------------//


/* handle a PointCloud2 message */
void point_cloud_callback(sensor_msgs::PointCloud2 msg)
{
  ROS_INFO("Got PointCloud2 msg with %u points\n", msg.width * msg.height);

  // get the cloud in PCL format in the right coordinate frame
  PointCloudXYZ cloud, cloud2, full_cloud;
  pcl::fromROSMsg(msg, cloud);
  transformPointCloud(target_frame, cloud, cloud2, *tf_listener);
  //if (!transformPointCloud(target_frame, cloud, cloud2, *tf_listener)) {
  //  ROS_WARN("Can't transform point cloud; aborting object detection.");
  //  return;
  //}
  full_cloud = cloud2;

  // lookup the transform from target frame to sensor
  tf::StampedTransform tf_transform;
  Eigen3::Affine3f sensor_pose;
  try {
    //tf_listener->lookupTransform(msg.header.frame_id, target_frame, msg.header.stamp, tf_transform);
    tf_listener->lookupTransform(target_frame, msg.header.frame_id, msg.header.stamp, tf_transform);
    transformTFToEigen3(tf_transform, sensor_pose);
  }
  catch (tf::TransformException& ex) {
    ROS_WARN("[point_cloud_callback] TF exception:\n%s", ex.what());
    return;
  }

  //pcl::RangeImage full_range_image;
  //pcl::RangeImage::CoordinateFrame frame = pcl::RangeImage::CAMERA_FRAME;
  //full_range_image.createFromPointCloud(full_cloud, .2*M_PI/180, 2*M_PI, M_PI, sensor_pose, frame, 0, 0, 0);

  ROS_INFO("Got PointCloud2 msg with %lu points\n", cloud2.points.size());

  if (!have_table)
    return;

  // filter out points outside of the attention area, and only keep points above the table
  ROS_INFO("Have table, filtering points on table...");
  polygon_filter(table_polygon, cloud2, cloud);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud.makeShared());
  pass.setFilterFieldName("z");
  float table_height = table_polygon.points[0].z;
  pass.setFilterLimits(table_height+.01, 1000.);
  pass.filter(cloud2);
  ROS_INFO("Done filtering table.");

  if (cloud2.points.size() < 100) {
    ROS_WARN("Not enough points; aborting object detection.");
    return;
  }

  ROS_INFO("Downsampling point cloud with %lu points", cloud2.points.size());
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  //grid.setFilterFieldName ("z");
  grid.setLeafSize (0.006, 0.006, 0.006);
  //grid.setFilterLimits (0.4, 1.1); //assuming there might be very low and very high tables
  grid.setInputCloud(cloud2.makeShared());
  grid.filter(cloud);


  ROS_INFO("Clustering remaining %lu points", cloud.points.size());

  // cluster groups of points (in the XY plane?)
  //flatten_point_cloud(cloud, cloud2);  //dbug
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster;
  KdTreeXYZ::Ptr clusters_tree = boost::make_shared< pcl::KdTreeFLANN<pcl::PointXYZ> > ();
  clusters_tree->setEpsilon (.0001);
  cluster.setClusterTolerance (0.02);
  cluster.setMinClusterSize (100);
  cluster.setSearchMethod (clusters_tree);
  vector<pcl::PointIndices> cluster_indices;
  cluster.setInputCloud (cloud.makeShared());  //cloud2
  cluster.extract(cluster_indices);
  ROS_INFO("Found %lu clusters.", cluster_indices.size());

  // for each cluster, try to fit an object model
  chaos::SceneAnalysis detected_objects;
  for (size_t i = 0; i < cluster_indices.size(); i++) {
  //for (size_t i = 1; i < 3; i++) {  //dbug
    PointCloudXYZ cluster;
    pcl::copyPointCloud (cloud, cluster_indices[i], cluster);

    // compute cluster centroid
    Vector4f centroid;
    pcl::compute3DCentroid(cluster, centroid);
    geometry_msgs::Pose init_pose;
    init_pose.position.x = centroid(0);
    init_pose.position.y = centroid(1);
    init_pose.position.z = centroid(2);

    ROS_INFO("Cluster has %lu points with centroid (%.2f, %.2f, %.2f)\n",
	     cluster.points.size(), centroid(0), centroid(1), centroid(2));
    //cout << "sensor_pose = \n" << sensor_pose.matrix() << endl;


    // create range image for point cluster
    pcl::RangeImage range_image0;
    pcl::RangeImage::CoordinateFrame frame = pcl::RangeImage::CAMERA_FRAME;
    range_image0.createFromPointCloud(cluster, .2*M_PI/180, 2*M_PI, M_PI, sensor_pose, frame, 0, 0, 0);
    //range_image0.createFromPointCloud(cluster, .3*M_PI/180, 2*M_PI, M_PI, sensor_pose, frame, 0, 0, 0);

    // dilate range image
    pcl::RangeImage range_image;
    chaos::dilate_range_image(range_image0, range_image);

    // add background points from full point cloud to cluster range image
    for (size_t j = 0; j < cloud.points.size(); j++) {
      Vector3f p(cloud.points[j].x, cloud.points[j].y, cloud.points[j].z);
      int xi, yi;
      float r;
      range_image.getImagePoint(p, xi, yi, r);
      if (range_image.isInImage(xi, yi) && isinf(r))
	range_image.getPoint(xi,yi).range = INFINITY;
    }


    // compute cluster normals
    PointCloudXYZN cluster_normals;
    chaos::compute_normals(cluster, cluster_normals, .03);

    //dbug
    //    for (int i=0; i < cluster_normals.points.size(); i++)
    //  printf("%.4f %.4f %.4f %.4f %.4f %.4f\n", cluster_normals.points[i].x, cluster_normals.points[i].y, cluster_normals.points[i].z,
    //	     cluster_normals.points[i].normal[0], cluster_normals.points[i].normal[1], cluster_normals.points[i].normal[2]);


    // compute range image with normals
    pcl::RangeImage range_image_normals;
    chaos::compute_range_image_normals(range_image, cluster_normals, range_image_normals);

    //chaos::visualize_range_image_with_normals(range_image, range_image_normals);  //dbug


    //dbug
    //for (int i=0; i < range_image_normals.points.size(); i++)
    //  if (isfinite(range_image_normals.points[i].range)
    //	  printf("%.4f %.4f %.4f %.4f\n", range_image_normals.points[i].x, range_image_normals.points[i].y,
    //		 range_image_normals.points[i].z, range_image_normals.points[i].range);


    // fit an object model to the cluster
    chaos::RangeImageTester T1(range_image);
    chaos::PointCloudDistanceTester T2(cluster);
    //chaos::RangeImageNormalTester T3(range_image_normals);
    chaos::CompositeTester model_tester(&T1,&T2); //,&T3);
    chaos::ObjectAnalysis obj = chaos::fit_object(model_manager, &model_tester, init_pose, table_height);


    //chaos::PointCloudNormalTester

    //TODO: add a PointCloudNormalTester using a kd-tree

    T1.testHypothesis(model_manager.get_model(obj.name), chaos::pose_to_affine_matrix(obj.pose), true);

    /*dbug
    PointCloudXYZ obs_cloud;
    PointCloudN obs_normals;
    chaos::range_image_with_normals_to_points_and_normals(range_image, range_image_normals, obs_cloud, obs_normals);
    PointCloudXYZN model_cloud_normals;
    pcl::transformPointCloudWithNormals(model_manager.get_model(obj.name).cloud_normals, model_cloud_normals,
					chaos::pose_to_affine_matrix(obj.pose));
    PointCloudXYZ model_cloud;
    PointCloudN model_normals;
    chaos::point_cloud_normals_to_points_and_normals(model_cloud_normals, model_cloud, model_normals);
    pcl_visualization::PCLVisualizer viewer("3D Viewer");
    viewer.addCoordinateSystem(1.0f);
    viewer.addPointCloud(obs_cloud, "obs points");
    viewer.addPointCloudNormals(obs_cloud, obs_normals, 3, .01, "obs normals");
    viewer.addPointCloud(model_cloud, "model points");
    viewer.addPointCloudNormals(model_cloud, model_normals, 3, .01, "model normals");
    while(!viewer.wasStopped()) {
      viewer.spinOnce(100);
      usleep(100000);
    }
    */


    if (obj.fitness_score < 1.6)  // 1.6 sigma (less that 5% false negative)
      detected_objects.objects.push_back(obj);
  }

  // publish detected objects
  detected_objects.header.frame_id = target_frame;
  detected_objects.header.stamp = msg.header.stamp;
  object_pub.publish(detected_objects);

  // publish detected objects point cloud
  PointCloudXYZ objs_cloud;
  sensor_msgs::PointCloud2 objs_cloud_msg;
  for (size_t i = 0; i < detected_objects.objects.size(); i++) {
    model_manager.get_model_at_pose(detected_objects.objects[i].name, detected_objects.objects[i].pose, cloud);
    objs_cloud += cloud;
  }
  pcl::toROSMsg(objs_cloud, objs_cloud_msg);
  objs_cloud_msg.header = detected_objects.header;
  cloud_pub.publish(objs_cloud_msg);
}


/* handle a Polygon message */
void table_callback(geometry_msgs::Polygon msg)
{
  ROS_INFO("Got Polygon msg\n");

  table_polygon = msg;
  have_table = true;
}


/* handle an All_Hulls message */
void all_hulls_callback(furniture::All_Hulls msg)
{
  ROS_INFO("Got All_Hulls msg\n");

  if (msg.hulls.size() > 0) {
    // hack: assume first table is the one we want
    table_polygon.points.resize(msg.hulls[0].polygon.points.size());
    for (size_t i = 0; i < msg.hulls[0].polygon.points.size(); i++) {
      table_polygon.points[i].x = msg.hulls[0].polygon.points[i].x;
      table_polygon.points[i].y = msg.hulls[0].polygon.points[i].y;
      table_polygon.points[i].z = msg.hulls[0].polygon.points[i].z;
    }
    have_table = true;
  }
}



//------------------- MAIN -------------------//

int main(int argc, char **argv)
{
  // init ROS
  ros::init(argc, argv, "chaos_object_detector");
  ros::NodeHandle nh;
  object_pub = nh.advertise<chaos::SceneAnalysis>("scene_analysis", 1);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("scene_analysis_cloud", 1);
  ros::Subscriber sub_cloud = nh.subscribe("pointcloud2", 1, point_cloud_callback);
  ros::Subscriber sub_table = nh.subscribe("table", 1, table_callback);
  ros::Subscriber sub_all_hulls = nh.subscribe("/convex_hulls", 1, all_hulls_callback);
  tf_listener = new tf::TransformListener;

  ros::Rate r(10);
  while (nh.ok()) {
    ros::spinOnce();
    r.sleep();
  }
}
