

#include <cardboard/detection.h>
#include <cardboard/models.h>
#include <cardboard/optimization.h>
#include <cardboard/ply.h>
#include <cardboard/util.h>
#include "fitness.h"


#include <cardboard/common.h>
#include <cardboard/testing.h>
#include <cardboard/SceneHypothesis.h>
#include <cardboard/DetectModels.h>
#include <cardboard/AlignModels.h>
#include <pthread.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>


#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>


namespace cardboard {



  //------------------ INTERNAL CLASSES -------------------//


  class ModelPoseOptimizer : public GradientFreeRandomizedGradientOptimizer {
  private:
    ModelTester *model_tester_;
    const Model &model_;
    Matrix4f initial_pose_;
  public:
    ModelPoseOptimizer(ModelTester *T, const Model &model);
    float evaluate(VectorXf x);
    Matrix4f x2affine(VectorXf x);
    Matrix4f optimizeAffine(const Matrix4f &initial_pose);
  };



  //---------------------  ModelPoseOptimizer class --------------------//

  ModelPoseOptimizer::ModelPoseOptimizer(ModelTester *T, const Model &model) :
    GradientFreeRandomizedGradientOptimizer(.07, Vector4f(.004, .004, 0, .03), 3.0),   // TODO: clean this up
    model_tester_(T),
    model_(model)
  {
  }

  Matrix4f ModelPoseOptimizer::x2affine(VectorXf x)
  {
    Matrix4f A_shift = Matrix4f::Identity();  // shift to origin
    Vector3f t = initial_pose_.topRightCorner(3,1);
    A_shift.col(3) << -t, 1;
    A_shift(3,3) = 1;
    t += x.topRows(3);
    float theta = x(3);
    Quaternionf q(cos(theta/2.0), 0, 0, sin(theta/2.0));
    Matrix4f A = pose_to_affine_matrix(t, q);  // rotate, then unshift and translate by x(0:2)
    return A * A_shift * initial_pose_;
  }

  float ModelPoseOptimizer::evaluate(VectorXf x)
  {
    Matrix4f A = x2affine(x);
    return model_tester_->testHypothesis(model_, A);
  }

  Matrix4f ModelPoseOptimizer::optimizeAffine(const Matrix4f &initial_pose)
  {
    initial_pose_ = initial_pose;
    VectorXf x0 = VectorXf::Zero(4);
    VectorXf x = optimize(x0);
    return x2affine(x);    
  }


  

//------------Helper Functions --------------------------//

double square(double num){
	return num*num;
}

int findClosestPoseIndex(geometry_msgs::Pose initPose, vector<geometry_msgs::Pose> cluster_poses){
	double smallestDist = -1;
	int smallestIndex = 0;
	for (uint i = 0; i < cluster_poses.size(); i++){
		geometry_msgs::Pose clusterPose = cluster_poses[i];
		double xDist = clusterPose.position.x - initPose.position.x;
		double yDist = clusterPose.position.y - initPose.position.y;
		double zDist = clusterPose.position.z - initPose.position.z;
		double eucDist = sqrt(square(xDist)+square(yDist)+square(zDist));
		if (smallestDist == -1)
			smallestDist = eucDist;
		if (eucDist < smallestDist){
			smallestDist = eucDist;
			smallestIndex = i;
		}
	}
	return smallestIndex;
}

// dbug -- move this to table_tracker
static double adjust_table_height(const PointCloudXYZ &cloud, double table_height_init)
{
  cardboard::init_rand();

  int i, j, iter = 100, n = cloud.points.size();
  double thresh = .01;

  double zmin = table_height_init, dmin = 1000*n;
  for (i = 0; i < iter; i++) {

    int j = rand() % n;
    double z = cloud.points[j].z;
    //printf("j = %d, z = %.2f\n", j, z);
    if (fabs(z - table_height_init) > .1)
      continue;

    double d = 0.0;
    for (j = 0; j < n; j += 10) {
      double dz = fabs(z - cloud.points[j].z);
      if (dz > thresh)
	dz = thresh;
      d += dz;
    }

    if (d < dmin) {
      dmin = d;
      zmin = z;
    }
  }

  return zmin;
}



/* convert a tf transform to an eigen transform */
void transformTFToEigen(const tf::Transform &t, Affine3f &k)
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


static void crossproduct(double vector1[],double vector2[], double Xproduct[])
{
	//initialize the cross product vector 

	//calculate the i,j,k coefficients of the cross product 
	Xproduct[0]=vector1[1]*vector2[2]-vector2[1]*vector1[2];
	Xproduct[1]=vector1[0]*vector2[2]-vector2[0]*vector1[2];
	Xproduct[2]=vector1[0]*vector2[1]-vector2[0]*vector1[1];
}

static double dotProduct(double v1[], double v2[])
{
  double dp = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
  return dp;

}

static vector<geometry_msgs::Polygon> find_horizontal_surfaces(const vector<geometry_msgs::Polygon> &surface_polygons)
{
  printf("break 1\n");

  vector<geometry_msgs::Polygon> horizontal_surfaces;
  vector<geometry_msgs::Polygon> horizontal_surfaces_sorted;
  //TODO: Sort by height
  for (int i = 0; i < surface_polygons.size(); i++) {
    geometry_msgs::Polygon current = surface_polygons[i];
    geometry_msgs::Point32 p1, p2, p3;
    
    p1 = current.points[0];
    p2 = current.points[1];
    p3 = current.points[2];

    double v1[3] = {p1.x - p2.x, p1.y - p2.y, p1.z - p2.z};
    double v2[3] = {p3.x - p2.x, p3.y - p2.y, p3.z - p2.z};
    double normal[3];
    crossproduct(v1, v2, normal);
    double mag = sqrt((normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2]));    
    if (fabs(normal[2]/mag) > .9) {
        horizontal_surfaces.push_back(current);
        printf("horizontal normal is (%f, %f, %f)\n", normal[0]/mag, normal[1]/mag, normal[2]/mag);
    
    }  
  }

  printf("break 2\n");

  double lowestValBigger = 0;
  for (int j = 0; j < horizontal_surfaces.size(); j++) {
    double currentMin = 1000; //FIND OUT HOW SCALED
    geometry_msgs::Polygon currentMinPlane;
    for (int k = 0; k < horizontal_surfaces.size(); k++) {
      if (horizontal_surfaces[k].points[0].z > lowestValBigger) {
        if (horizontal_surfaces[k].points[0].z < currentMin) {
          currentMin = horizontal_surfaces[k].points[0].z;
          currentMinPlane = horizontal_surfaces[k];

        }

      }

    }
    lowestValBigger = currentMin;
    horizontal_surfaces_sorted.push_back(currentMinPlane);
  }

  printf("break 3\n");
 
  return horizontal_surfaces_sorted;
}

static vector<geometry_msgs::Polygon> find_nonhorizontal_surfaces(const vector<geometry_msgs::Polygon> &surface_polygons)
{ 
  printf("break 1\n");

  vector<geometry_msgs::Polygon> nonhorizontal_surfaces;

  for (int i = 0; i < surface_polygons.size(); i++) {
    geometry_msgs::Polygon current = surface_polygons[i];
    geometry_msgs::Point32 p1, p2, p3;

    p1 = current.points[0];
    p2 = current.points[1];
    p3 = current.points[2];

    double v1[3] = {p1.x - p2.x, p1.y - p2.y, p1.z - p2.z};
    double v2[3] = {p3.x - p2.x, p3.y - p2.y, p3.z - p2.z};

    double normal[3];
    crossproduct(v1, v2, normal);
    double mag = sqrt((normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2]));
    if (fabs(normal[2]/mag) <= .9) {
        nonhorizontal_surfaces.push_back(current);
        printf("non-horizontal normal is (%f, %f, %f)\n", normal[0]/mag, normal[1]/mag, normal[2]/mag);
   
    }
  }

  printf("break 2\n");

  return nonhorizontal_surfaces;
}

static void remove_plane_points(const vector<geometry_msgs::Polygon> &plane_polygons, const PointCloudXYZ &cloud_in, PointCloudXYZ &cloud_out)
{
  printf("break 1\n");

  // TODO: remove points near planes (optional: only remove points within polygon)
  bool inNewCloud[cloud_in.points.size()];
  for (int k = 0; k < cloud_in.points.size(); k++) {
    inNewCloud[k] = true;
  }

  printf("break 2\n");

  for (int i = 0; i < plane_polygons.size(); i++) {
    geometry_msgs::Point32 p1, p2, p3;

    p1 = plane_polygons[i].points[0];
    p2 = plane_polygons[i].points[1];
    p3 = plane_polygons[i].points[2];

    double v1[] = {p1.x - p2.x, p1.y - p2.y, p1.z - p2.z};
    double v2[] = {p3.x - p2.x, p3.y - p2.y, p3.z - p2.z};

    double normal[3];

    crossproduct(v1, v2, normal);

    double mag = sqrt((normal[0]*normal[0]+ normal[1]*normal[1] + normal[2]*normal[2]));
    normal[0] = normal[0]/mag;
    normal[1] = normal[1]/mag;
    normal[2] = normal[2]/mag;

    for (int j = 0; j < cloud_in.points.size(); j++) {
      pcl::PointXYZ cloudPoint = cloud_in.points[j];

      double planeToPoint[] = {cloudPoint.x - p1.x, cloudPoint.y - p1.y, cloudPoint.z - p1.z};
      double distFromPlane = fabs(dotProduct(planeToPoint, normal));
      if (distFromPlane < .06) { //FIND OUT HOW THIS IS SCALED!!!!
        inNewCloud[j] = false;
      }
    }
  }

  printf("break 3\n");

  cloud_out.points.resize(0);
  for (int n = 0; n < cloud_in.points.size(); n++) {
    if (inNewCloud[n]) {
      cloud_out.points.push_back(cloud_in.points[n]);
    }
  }

  printf("break 4\n");

}

  

  //--------------------- EXTERNAL API --------------------//


  SceneHypothesis *detect_models(sensor_msgs::PointCloud2 &msg, string target_frame, vector<geometry_msgs::Polygon> surface_polygons,
				 tf::TransformListener *tf_listener, double table_filter_thresh, ModelManager *model_manager,
				 std::vector<string> models, ros::Publisher &debug_pub)  //dbug
{
  // get the cloud in PCL format in the right coordinate frame
  PointCloudXYZ cloud, cloud2, full_cloud;
  pcl::fromROSMsg(msg, cloud);
  if (!pcl_ros::transformPointCloud(target_frame, cloud, cloud2, *tf_listener)) {
    ROS_WARN("Can't transform point cloud; aborting object detection.");
    return NULL;
  }
  full_cloud = cloud2;

  // lookup the transform from target frame to sensor
  tf::StampedTransform tf_transform;
  Eigen::Affine3f sensor_pose;
  try {
    //tf_listener->lookupTransform(msg.header.frame_id, target_frame, msg.header.stamp, tf_transform);
    tf_listener->lookupTransform(target_frame, msg.header.frame_id, msg.header.stamp, tf_transform);
    transformTFToEigen(tf_transform, sensor_pose);
  }
  catch (tf::TransformException& ex) {
    ROS_WARN("[point_cloud_callback] TF exception:\n%s", ex.what());
    return NULL;
  }

  //pcl::RangeImage full_range_image;
  //pcl::RangeImage::CoordinateFrame frame = pcl::RangeImage::CAMERA_FRAME;
  //full_range_image.createFromPointCloud(full_cloud, .2*M_PI/180, 2*M_PI, M_PI, sensor_pose, frame, 0, 0, 0);

  //ROS_INFO("Got PointCloud2 msg with %lu points\n", cloud2.points.size());

  //if (!have_table)
  //  return NULL;



  // find horizontal planes using normals, and sort by height
  vector<geometry_msgs::Polygon> shelves = find_horizontal_surfaces(surface_polygons);

  // remove points near vertical planes
  vector<geometry_msgs::Polygon> walls = find_nonhorizontal_surfaces(surface_polygons);
  remove_plane_points(walls, full_cloud, cloud2);

  full_cloud = cloud2;

  SceneHypothesis *detected_objects = new SceneHypothesis();  


  // loop over flat surfaces, removing everything above the next shelf
  for (int i = 0; i < shelves.size(); i++) {

    // filter out points outside of the attention area, and only keep points above the table
    //ROS_INFO("Have table, filtering points on table...");
    //polygon_filter(table_polygon, cloud2, cloud);
    ROS_INFO("Found flat surface, filtering points on shelf...");
    polygon_filter(shelves[i], full_cloud, cloud);

    if (cloud.points.size() < 100) {
      ROS_WARN("Not enough points in table region; aborting object detection.");
      //return NULL;
      continue;
    }

    float table_height = adjust_table_height(cloud, shelves[i].points[0].z);

    printf("shelf_height = %.3f, table_height = %.3f\n", shelves[i].points[0].z, table_height);

    // remove everything above and below current shelf
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud.makeShared());
    pass.setFilterFieldName("z");
    double min_z = table_height + table_filter_thresh;
    double max_z = (i+1 < shelves.size() ? shelves[i+1].points[0].z - 2*table_filter_thresh : 1000.0);
    pass.setFilterLimits(min_z, max_z);
    pass.filter(cloud2);
    ROS_INFO("Removed shelves.");
    
    //dbug
    if (i==0) {
      sensor_msgs::PointCloud2 dbug_msg;
      pcl::toROSMsg(cloud2, dbug_msg);
      dbug_msg.header = msg.header;
      dbug_msg.header.frame_id = target_frame;
      debug_pub.publish(dbug_msg);
    }

    if (cloud2.points.size() < 100) {
      ROS_WARN("Not enough points above table; aborting object detection.");
      //return NULL;
      continue;
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
    //KdTreeXYZ::Ptr clusters_tree = boost::make_shared< pcl::KdTreeFLANN<pcl::PointXYZ> > ();
    pcl::search::KdTree<pcl::PointXYZ>::Ptr clusters_tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    clusters_tree->setEpsilon (.0001);
    cluster.setClusterTolerance (0.02);
    cluster.setMinClusterSize (100);
    cluster.setSearchMethod (clusters_tree);
    vector<pcl::PointIndices> cluster_indices;
    cluster.setInputCloud (cloud.makeShared());  //cloud2
    cluster.extract(cluster_indices);
    ROS_INFO("Found %lu clusters.", cluster_indices.size());
    
    for (size_t i = 0; i < cluster_indices.size(); i++) {
      
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
      
      
      // create range image for point cluster
      pcl::RangeImage range_image0;
      pcl::RangeImage::CoordinateFrame frame = pcl::RangeImage::CAMERA_FRAME;
      range_image0.createFromPointCloud(cluster, .2*M_PI/180, 2*M_PI, M_PI, sensor_pose, frame, 0, 0, 0);
      //range_image0.createFromPointCloud(cluster, .3*M_PI/180, 2*M_PI, M_PI, sensor_pose, frame, 0, 0, 0);
      
      // dilate range image
      pcl::RangeImage range_image;
      cardboard::dilate_range_image(range_image0, range_image);
      
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
      //   PointCloudXYZN cluster_normals;
      //    cardboard::compute_normals(cluster, cluster_normals, .03);


      // compute range image with normals
      //    pcl::RangeImage range_image_normals;
      //    cardboard::compute_range_image_normals(range_image, cluster_normals, range_image_normals);


      // fit an object model to the cluster
      cardboard::RangeImageTester T1(range_image);
      cardboard::PointCloudDistanceTester T2(cluster);
      //    cardboard::RangeImageNormalTester T3(range_image_normals);
      cardboard::CompositeTester model_tester(&T1,&T2);//,&T3);
      cardboard::ObjectHypothesis obj = fit_object(*model_manager, &model_tester, init_pose, table_height, tf_listener, models);
      
      
      T1.testHypothesis(model_manager->get_model(obj.name), cardboard::pose_to_affine_matrix(obj.pose), true);

      //if (obj.fitness_score < 1.6)  // 1.6 sigma (less that 5% false negative)
        detected_objects->objects.push_back(obj);
      //else {
        //ROS_INFO("Current object is unidentifiable");
	// compute PolygonMesh from cluster, add it to junk
        // Normal estimation*
        // Normal estimation*
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (cluster);
        // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
        // pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        // pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
        // tree->setInputCloud (cloud);
        // n.setInputCloud (cloud);
        // n.setSearchMethod (tree);
        // n.setKSearch (20);
        // n.compute (*normals);
         //* normals should not contain the point normals + surface curvatures
         // Concatenate the XYZ and normal fields*
        // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
        // pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
        // //* cloud_with_normals = cloud + normals
        // // Create search tree*
        // pcl::KdTreeFLANN<pcl::PointNormal>::Ptr tree2 (new pcl::KdTreeFLANN<pcl::PointNormal>);
        // tree2->setInputCloud (cloud_with_normals);
         // Initialize objects
        // pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        // pcl::PolygonMesh triangles;
        // // Set the maximum distance between connected points (maximum edge length)
        // gp3.setSearchRadius (0.025);
        // // Set typical values for the parameters
        // gp3.setMu (2.5);
        // gp3.setMaximumNearestNeighbors (100);
        // gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
        // gp3.setMinimumAngle(M_PI/18); // 10 degrees
        // gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
        // gp3.setNormalConsistency(false);
         // Get result
       //  gp3.setInputCloud (cloud_with_normals);
      //   gp3.setSearchMethod (tree2);
      //   gp3.reconstruct (triangles);
      //   cardboard::Unidentifiable junk;
      //   junk.mesh = triangles;
      //   detected_objects->junk.push_back(junk);
    //  }
    }
  }

  detected_objects->header.stamp = msg.header.stamp;
  detected_objects->header.frame_id = target_frame;
  //publish_scene_hypothesis(detected_objects, target_frame, msg.header.stamp);

  return detected_objects;
}


//-----------------------------------------------------------------------------------------------//  
  
  
  ObjectHypothesis fit_object(const ModelManager &model_manager, ModelTester *model_tester,
			      geometry_msgs::Pose init_pose, float table_height, tf::TransformListener *tf_listener, std::vector<string> models)
  {
    Matrix4f A0 = pose_to_affine_matrix(init_pose);

    ObjectHypothesis fit_model;
    
    // try fitting each model to the point cloud at several orientations
    fit_model.fitness_score = std::numeric_limits<float>::max();
    

    //--- Create vector housing orientation rotations ----//
    std::vector<Quaternionf> orientations;
    orientations.push_back(Quaternionf(1,0,0,0));
    orientations.push_back(Quaternionf(sqrt(2)/2,sqrt(2)/2,0,0));
    orientations.push_back(Quaternionf(sqrt(2)/2, -1*sqrt(2)/2,0,0));
    orientations.push_back(Quaternionf(0,1,0,0));
    orientations.push_back(Quaternionf(sqrt(2)/2,0,sqrt(2)/2,0));
    orientations.push_back(Quaternionf(sqrt(2)/2,0,-1*sqrt(2)/2,0));

    for (uint i = 0; i < model_manager.models.size(); i++) {

      bool include_model = false;
      if (models.size() == 0)
	include_model = true;
      else {
	for (uint j = 0; j < models.size(); j++) {
	  if (models[j].compare(model_manager.models[i].name) == 0) {
	    include_model = true;
	    break;
	  }
	}
      }
      if (!include_model)
	continue;

      float min_score = std::numeric_limits<float>::max();

      double t1, t2;
      t1 = get_time_ms();


// change to loop through 6 orientation Quaternions


#     pragma omp parallel for 
      for (uint j = 0; j < orientations.size(); j++) {  //dbug
	
	// rotate model onto a face of its convex hull
	Quaternionf q1 = orientations[j];
	Vector3f t = Vector3f::Zero();
	Matrix4f A_face = pose_to_affine_matrix(t, q1);
	
#       pragma omp parallel for 
	for (uint k = 0 /*cnt*/; k < 5; k++) {  //dbug

	  // rotate model incrementally about the z-axis
	  double a = 2*M_PI*(k/5.0);  //dbug
	  Quaternionf q2 = Quaternionf(cos(a/2), 0, 0, sin(a/2));
	  Matrix4f A_zrot = pose_to_affine_matrix(t, q2);
	  Matrix4f A = A0 * A_zrot * A_face;
	  pcl::PointCloud<pcl::PointXYZ> obj;
	  pcl::transformPointCloud(model_manager.models[i].cloud, obj, A);
	  
	  // make sure bottom of model is touching table
	  Vector4f pmin, pmax;
	  pcl::getMinMax3D(obj, pmin, pmax);
	  Matrix4f A_gravity = Matrix4f::Identity();
	  A_gravity(2,3) = table_height - pmin(2);
	  pcl::transformPointCloud(obj, obj, A_gravity);
	  A = A_gravity * A;
	  
	  // fit rotated model to range image
	  ModelPoseOptimizer opt(model_tester, model_manager.models[i]);
	  opt.setMaxIterations(200);
	  Matrix4f A2 = opt.optimizeAffine(A);
	  float score = opt.getFinalCost();
	  
	  printf(".");
	  fflush(0);
	  
#         pragma omp critical
	  {
	    if (score < fit_model.fitness_score) {
	      fit_model.fitness_score = score;
	      fit_model.name = model_manager.models[i].name;
	      fit_model.pose = affine_matrix_to_pose(A2);  // * A
	    }
	    if (score < min_score)
	      min_score = score;
	  }
	}

      }

      t2 = get_time_ms();
      printf("finished j loop in %.2f ms (min_score = %.4f)\n", t2-t1, min_score);
    }
    
    cout << "Best fit: obj = " << fit_model.name << ", score = " << fit_model.fitness_score << endl;
    //cout << fit_model.pose << endl;
    
    // print component scores
    const Model &best_model = model_manager.get_model(fit_model.name);
    if (best_model.name.size() != 0) {
      printf("component scores:  [  ");
      CompositeTester *CT = (CompositeTester *)model_tester;
      for (uint i = 0; i < CT->testers.size(); i++) {
	float score = CT->testers[i]->testHypothesis(best_model, pose_to_affine_matrix(fit_model.pose));
	printf("%.4f  ", CT->weights[i] * score);
      }
      printf("]\n");
    }

    return fit_model;
  }


 

  //----------------------- Alignment ----------------------//

  SceneHypothesis *align_models(sensor_msgs::PointCloud2 &msg, string target_frame, vector<geometry_msgs::Polygon> surface_polygons,
				std::vector<geometry_msgs::Pose> initial_poses, std::vector<string> models,
				tf::TransformListener *tf_listener, double table_filter_thresh, ModelManager *model_manager)
{
  // get the cloud in PCL format in the right coordinate frame
  PointCloudXYZ cloud, cloud2, full_cloud;
  pcl::fromROSMsg(msg, cloud);
  if (!pcl_ros::transformPointCloud(target_frame, cloud, cloud2, *tf_listener)) {
    ROS_WARN("Can't transform point cloud; aborting object detection.");
    return NULL;
  }
  full_cloud = cloud2;

  // lookup the transform from target frame to sensor
  tf::StampedTransform tf_transform;
  Eigen::Affine3f sensor_pose;
  try {
    //tf_listener->lookupTransform(msg.header.frame_id, target_frame, msg.header.stamp, tf_transform);
    tf_listener->lookupTransform(target_frame, msg.header.frame_id, msg.header.stamp, tf_transform);
    transformTFToEigen(tf_transform, sensor_pose);
  }
  catch (tf::TransformException& ex) {
    ROS_WARN("[point_cloud_callback] TF exception:\n%s", ex.what());
    return NULL;
  }

  //pcl::RangeImage full_range_image;
  //pcl::RangeImage::CoordinateFrame frame = pcl::RangeImage::CAMERA_FRAME;
  //full_range_image.createFromPointCloud(full_cloud, .2*M_PI/180, 2*M_PI, M_PI, sensor_pose, frame, 0, 0, 0);

  //ROS_INFO("Got PointCloud2 msg with %lu points\n", cloud2.points.size());

  //if (!have_table)
  //  return NULL;

  // filter out points outside of the attention area, and only keep points above the tabl e
  ROS_INFO("Have table, filtering points on table...");
  polygon_filter(surface_polygons[0], cloud2, cloud);

  if (cloud.points.size() < 100) {
    ROS_WARN("Not enough points in table region; aborting object detection.");
    return NULL;
  }

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud.makeShared());
  pass.setFilterFieldName("z");
  //float table_height = table_polygon.points[0].z;
  float table_height = adjust_table_height(cloud, surface_polygons[0].points[0].z);
  pass.setFilterLimits(table_height + table_filter_thresh, 1000.);
  pass.filter(cloud2);
  ROS_INFO("Done filtering table.");

  if (cloud2.points.size() < 100) {
    ROS_WARN("Not enough points above table; aborting object detection.");
    return NULL;
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
  //KdTreeXYZ::Ptr clusters_tree = boost::make_shared< pcl::KdTreeFLANN<pcl::PointXYZ> > ();
  pcl::search::KdTree<pcl::PointXYZ>::Ptr clusters_tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  clusters_tree->setEpsilon (.0001);
  cluster.setClusterTolerance (0.02);
  cluster.setMinClusterSize (100);
  cluster.setSearchMethod (clusters_tree);
  vector<pcl::PointIndices> cluster_indices;
  cluster.setInputCloud (cloud.makeShared());  //cloud2
  cluster.extract(cluster_indices);
  ROS_INFO("Found %lu clusters.", cluster_indices.size());

  // for each cluster, try to fit an object model
  SceneHypothesis *detected_objects = new SceneHypothesis();


  // store point-clusters and accompanying clouds
  vector<geometry_msgs::Pose> clusterPoses;
  vector<PointCloudXYZ> clusterClouds;
  for (size_t i = 0; i < cluster_indices.size(); i++) {

    PointCloudXYZ cluster;
    pcl::copyPointCloud (cloud, cluster_indices[i], cluster);

    // compute cluster centroid
    Vector4f centroid;
    pcl::compute3DCentroid(cluster, centroid);
    geometry_msgs::Pose init_pose;
    init_pose.position.x = centroid(0);
    init_pose.position.y = centroid(1);
    init_pose.position.z = centroid(2);
    clusterPoses.push_back(init_pose);
    clusterClouds.push_back(cluster);
  }

  // for each initial pose, find closest point-cluster
  for (uint i = 0; i < initial_poses.size(); i++){
	  int poseIndex = findClosestPoseIndex(initial_poses[i], clusterPoses);
	  geometry_msgs::Pose clusterPose = clusterPoses[poseIndex];
	  PointCloudXYZ cluster = clusterClouds[poseIndex];




    // create range image for point cluster
    pcl::RangeImage range_image0;
    pcl::RangeImage::CoordinateFrame frame = pcl::RangeImage::CAMERA_FRAME;
    range_image0.createFromPointCloud(cluster, .2*M_PI/180, 2*M_PI, M_PI, sensor_pose, frame, 0, 0, 0);

    // dilate range image
    pcl::RangeImage range_image;
    cardboard::dilate_range_image(range_image0, range_image);

    // add background points from full point cloud to cluster range image
    for (size_t j = 0; j < cloud.points.size(); j++) {
      Vector3f p(cloud.points[j].x, cloud.points[j].y, cloud.points[j].z);
      int xi, yi;
      float r;
      range_image.getImagePoint(p, xi, yi, r);
      if (range_image.isInImage(xi, yi) && isinf(r))
	range_image.getPoint(xi,yi).range = INFINITY;
    }


    // do one alignment for each cluster
    cardboard::RangeImageTester T1(range_image);
    cardboard::PointCloudDistanceTester T2(cluster);
    //cardboard::RangeImageNormalTester T3(range_image_normals);
    cardboard::CompositeTester model_tester(&T1,&T2); //,&T3);
   cardboard::ObjectHypothesis obj = align_object(*model_manager, &model_tester, initial_poses[i], table_height, models[i]);


    T1.testHypothesis(model_manager->get_model(models[i]), cardboard::pose_to_affine_matrix(initial_poses[i]), true);

    //if (obj.fitness_score < 1.6)  // 1.6 sigma (less that 5% false negative)
      detected_objects->objects.push_back(obj);
  }

  detected_objects->header.stamp = msg.header.stamp;
  detected_objects->header.frame_id = target_frame;
  //publish_scene_hypothesis(detected_objects, target_frame, msg.header.stamp);

  return detected_objects;
}

  
//-----------------------------------------------------------------------------------------------//  
  
  ObjectHypothesis align_object(const ModelManager &model_manager, ModelTester *model_tester,
			      geometry_msgs::Pose init_pose, float table_height, string modelName)
  {
    Matrix4f A0 = pose_to_affine_matrix(init_pose);

    ObjectHypothesis fit_model;
    
    fit_model.fitness_score = std::numeric_limits<float>::max();

      float min_score = std::numeric_limits<float>::max();
      double t1, t2;
      Vector3f t = Vector3f::Zero();
      t1 = get_time_ms();

      for (uint k = 0 /*cnt*/; k < 5; k++) {  //dbug

      	  // rotate model incrementally about the z-axis
      	  double a = 2*M_PI*(k/5.0);  //dbug
      	  Quaternionf q2 = Quaternionf(cos(a/2), 0, 0, sin(a/2));
      	  Matrix4f A_zrot = pose_to_affine_matrix(t, q2);
      	  Matrix4f A = A0 * A_zrot;
      	  pcl::PointCloud<pcl::PointXYZ> obj;
      	  pcl::transformPointCloud(model_manager.get_model(modelName).cloud, obj, A);

      	  // make sure bottom of model is touching table
      	  Vector4f pmin, pmax;
      	  pcl::getMinMax3D(obj, pmin, pmax);
      	  Matrix4f A_gravity = Matrix4f::Identity();
      	  A_gravity(2,3) = table_height - pmin(2);
      	  pcl::transformPointCloud(obj, obj, A_gravity);
      	  A = A_gravity * A;

	  
      // Do one aligment at the detected pose with the detected model
	  ModelPoseOptimizer opt(model_tester, model_manager.get_model(modelName));
	  opt.setMaxIterations(200);
	  Matrix4f A2 = opt.optimizeAffine(A);
	  float score = opt.getFinalCost();
	  
	  printf(".");
	  fflush(0);

	  {
	    if (score < fit_model.fitness_score) {
	      fit_model.fitness_score = score;
	      fit_model.name = modelName;
	      fit_model.pose = affine_matrix_to_pose(A2);  // * A
	    }
	    if (score < min_score)
	      min_score = score;
	  }
      }

      t2 = get_time_ms();
      printf("finished Alignment loop in %.2f ms (min_score = %.4f)\n", t2-t1, min_score);

    // print component scores
    const Model &best_model = model_manager.get_model(modelName);
    if (best_model.name.size() != 0) {
      printf("component scores:  [  ");
      CompositeTester *CT = (CompositeTester *)model_tester;
      for (uint i = 0; i < CT->testers.size(); i++) {
	float score = CT->testers[i]->testHypothesis(best_model, pose_to_affine_matrix(fit_model.pose));
	printf("%.4f  ", CT->weights[i] * score);
      }
      printf("]\n");
    }

    return fit_model;
  }










  //---------------------------  DEPRECATED ---------------------------//




  /* RangeImageOptimizer class (deprecated)
   *    - Optimizes the placement of a point cloud with respect to a range image via gradient descent
   *
  class RangeImageOptimizer : public GradientFreeRandomizedGradientOptimizer {
  private:
    //MatrixXf eigen_cloud_;
    //MatrixXf eigen_range_image_;
    const pcl::RangeImage &range_image_;
    const pcl::PointCloud<pcl::PointXYZ> &cloud_;
    const DistanceTransform3D &distance_transform_;  // distance transform of the point cloud
    Matrix4f initial_pose_;
    //pcl::KdTreeANN<pcl::PointXYZ> kdtree_;
    Vector3f cloud_centroid_;
  public:
    RangeImageOptimizer(const pcl::RangeImage &range_image,
			const pcl::PointCloud<pcl::PointXYZ> &cloud,
			const DistanceTransform3D &distance_transform);
    float evaluate(VectorXf x);
    VectorXf gradient(VectorXf x);
    Matrix4f optimizeAffine(const Matrix4f &initial_pose);
    void setInitialPose(const Matrix4f &initial_pose);
    Matrix4f x2affine(VectorXf x);
  };
  *****************/
  
  /* RangeImageGridOptimizer class (deprecated)
   *    - Optimizes the placement of a point cloud with respect to a range image via grid search
   *
  class RangeImageGridOptimizer : public GridOptimizer {
  private:
    RangeImageOptimizer opt_;
  public:
    RangeImageGridOptimizer(RangeImageOptimizer opt);
    ~RangeImageGridOptimizer();
    float evaluate(VectorXf x);
    Matrix4f optimizeAffine(const Matrix4f &initial_pose);
  };

  //dbug
  //static int grid_cnt = 0;
  //static int eval_cnt = 0;
  //static FILE *grid_f = NULL;
  
  *****************/



  
  //---------------------  RangeImageOptimizer class deprecated) --------------------//
  /*************************

  RangeImageOptimizer::RangeImageOptimizer(const pcl::RangeImage &range_image,
					   const pcl::PointCloud<pcl::PointXYZ> &cloud,
					   const DistanceTransform3D &distance_transform) :
    GradientFreeRandomizedGradientOptimizer(.07, Vector4f(.004, .004, 0, .03), 3.0),
    range_image_(range_image),
    cloud_(cloud),
    distance_transform_(distance_transform)
  {
  }
  
  Matrix4f RangeImageOptimizer::x2affine(VectorXf x)
  {
    Matrix4f A_shift = Matrix4f::Identity();  // shift to origin
    A_shift.col(3) << -cloud_centroid_, 1;
    Vector3f t = cloud_centroid_ + x.topRows(3);
    float theta = x(3);
    Quaternionf q(cos(theta/2.0), 0, 0, sin(theta/2.0));
    Matrix4f A = pose_to_affine_matrix(t, q);  // rotate, then unshift and translate by x(0:2)
    return A * A_shift * initial_pose_;
  }

  float RangeImageOptimizer::evaluate(VectorXf x)
  {
    Matrix4f A = x2affine(x);
    Matrix4f A_inv = A.inverse();

    pcl::PointCloud<pcl::PointXYZ> cloud2;
    pcl::transformPointCloud(cloud_, cloud2, A);
    pcl::RangeImage range_image2;
    pcl::transformPointCloud(range_image_, range_image2, A_inv);
    
    float f_neg = 1500*range_image_fitness(range_image_, cloud2);   // negative information
    float f_pos = 5000*range_cloud_fitness(distance_transform_, range_image2);  // positive information

    return f_pos + f_neg;
  }
  
  VectorXf RangeImageOptimizer::gradient(VectorXf x)
  {
    VectorXf dfdx = VectorXf::Zero(4);
    float dt = .005;
    float da = .05;
    
    float f = evaluate(x);
    
    for (int i = 0; i < 2; i++) {
      VectorXf x2 = x;
      x2(i) += dt;
      dfdx(i) = (evaluate(x2) - f) / dt;
    }
    VectorXf x2 = x;
    x2(3) += da;
    dfdx(3) = (evaluate(x2) - f) / dt;
    
    return dfdx;
  }
  
  Matrix4f RangeImageOptimizer::optimizeAffine(const Matrix4f &initial_pose)
  {
    setInitialPose(initial_pose);
    VectorXf x0 = VectorXf::Zero(4);
    VectorXf x = optimize(x0);
    
    return x2affine(x);
  }

  void RangeImageOptimizer::setInitialPose(const Matrix4f &initial_pose)
  {
    initial_pose_ = initial_pose;

    pcl::PointCloud<pcl::PointXYZ> cloud2;
    pcl::transformPointCloud(cloud_, cloud2, initial_pose);
    Vector4f centroid;
    pcl::compute3DCentroid(cloud2, centroid);
    cloud_centroid_ = centroid.topRows(3);
  }
  ***********************/


  //---------------------  RangeImageGridOptimizer class (deprecated) --------------------//
  /**********************

  RangeImageGridOptimizer::RangeImageGridOptimizer(RangeImageOptimizer opt) :
    opt_(opt)
  {
  }
  
  RangeImageGridOptimizer::~RangeImageGridOptimizer()
  {
  }
  
  float RangeImageGridOptimizer::evaluate(VectorXf x)
  {
    float score = opt_.evaluate(x);
    fprintf(grid_f, "%.6f ", score);  //dbug
    return score;
  }
  
  Matrix4f RangeImageGridOptimizer::optimizeAffine(const Matrix4f &initial_pose)
  {
    opt_.setInitialPose(initial_pose);

    //dbug
    char fname[100];
    sprintf(fname, "out%d.m", grid_cnt);
    grid_f = fopen(fname, "w");
    grid_cnt++;
    //eval_cnt = 0;
    
    fprintf(grid_f, "F = [");
    VectorXf x = GridOptimizer::optimize();
    
    //dbug
    fprintf(grid_f, "];\n");
    fprintf(grid_f, "F = F(1:end-1);\n");
    fprintf(grid_f, "F_dims = [");
    for (int i = 0; i < x.size(); i++)
      fprintf(grid_f, "%d ", 1 + (int)round((xmax_(i) - xmin_(i)) / resolution_(i)));
    fprintf(grid_f, "];\n");
    fclose(grid_f);

    return opt_.x2affine(x);
  }
  ************************/
  


  
  /***** deprecated *****
  TrackedObject fit_object_to_range_image(const pcl::RangeImage &range_image_in, geometry_msgs::Pose init_pose, float table_height)
  {

    // dilate range image
    pcl::RangeImage range_image = range_image_in;
    //dilate_range_image(range_image_in, range_image);


    //static uint cnt = 9;
    
    Matrix4f A0 = pose_to_affine_matrix(init_pose);
    
    ROS_INFO("fit_object_to_range_image()");
    //ROS_INFO("Initial Pose:");
    //cout << A0 << endl;
    
    if (!modelManager.loaded_models)
      modelManager.load_models();
    
    TrackedObject fit_model;
    
    // try fitting each model to the point cloud at several orientations
    double min_score = std::numeric_limits<double>::max();
    
    for (uint i = 0; i < modelManager.model_clouds.size(); i++) {  //dbug

      double t1, t2;
      t1 = get_time_ms();


#     pragma omp parallel for 
      for (uint j = 0; j < modelManager.model_orientations[i].size(); j++) {  //dbug
	
	//printf("[i = %u, j = %u]\n", i, j);

	// rotate model onto a face of its convex hull
	Quaternionf q1 = modelManager.model_orientations[i][j];
	Vector3f t = Vector3f::Zero();
	Matrix4f A_face = pose_to_affine_matrix(t, q1);
	
#       pragma omp parallel for 
	for (uint k = 0 ; k < 5; k++) {  //dbug

          // rotate model incrementally about the z-axis
          double a = 2*M_PI*(k/5.0);  //dbug
	  Quaternionf q2 = Quaternionf(cos(a/2), 0, 0, sin(a/2));
	  Matrix4f A_zrot = pose_to_affine_matrix(t, q2);
	  Matrix4f A = A0 * A_zrot * A_face;
	  pcl::PointCloud<pcl::PointXYZ> obj;
	  pcl::transformPointCloud(modelManager.model_clouds[i], obj, A);
	  
	  // make sure bottom of model is touching table
	  Vector4f pmin, pmax;
	  pcl::getMinMax3D(obj, pmin, pmax);
	  Matrix4f A_gravity = Matrix4f::Identity();
	  A_gravity(2,3) = table_height - pmin(2);
	  pcl::transformPointCloud(obj, obj, A_gravity);
	  A = A_gravity * A;
	  
	  // fit rotated model to range image
	  double score = range_image_fitness(range_image, obj);
	  RangeImageOptimizer opt(range_image, modelManager.model_clouds[i],
				  modelManager.model_distance_transforms[i]);
	  opt.setMaxIterations(200); //50
	  //RangeImageGridOptimizer optimizer(opt);
	  //VectorXf res(4), xmin(4), xmax(4);
	  //res << .004, .004, 1, (M_PI/100.0);
	  //xmin << -.04, -.04, 0, -M_PI/10.0;
	  //xmax << .0401, .0401, 0, (M_PI/10.0 + .0001);
	  //optimizer.setResolution(res);
	  //optimizer.setBounds(xmin, xmax);
	  Matrix4f A2 = opt.optimizeAffine(A);  //optimizer.optimizeAffine(A);
	  score = opt.getFinalCost();  //optimizer.getFinalCost();
	  
	  printf(".");
	  fflush(0);
	  
#         pragma omp critical
	  {
	    if (score < min_score) {
	      //printf("min_score = %.6f\n", score);
	      min_score = score;
	      fit_model.name = modelManager.object_names[i];
	      fit_model.pose = affine_matrix_to_pose(A2);  // * A
	    }
	    //else
	    //  printf("score = %.6f\n", score);
	  }
	}

      }


      t2 = get_time_ms();
      printf("finished j loop in %.2f ms\n", t2-t1);

    }
    
    cout << "Best fit: obj = " << fit_model.name << ", score = " << min_score << endl;
    cout << fit_model.pose << endl;
    
    return fit_model;
  }
  ****************/

  





  
}  // end of namespace cardboard

