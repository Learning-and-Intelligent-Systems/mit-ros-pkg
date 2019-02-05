
#include <chaos/models.h>
#include "util.h"
#include "ply.h"


namespace chaos {


  static string object_folder_const = "data";
  static string object_names_file_const = "objects.txt";
  //static string object_names_const[] = {"LBlock", "RectCup"};  //TODO: load names automatically
  //static int num_objects_const = 2;
  

  //------------------ GLOBAL VARIABLES -------------------//

  ModelManager::ModelManager()
  {
    loaded_models = false;
    load_models();
  }


  // Load model point clouds and convex hulls
  void ModelManager::load_models()
  {
    // load object names from file
    char fname[128], buf[1024];
    sprintf(fname, "%s/%s", object_folder_const.c_str(), object_names_file_const.c_str());
    FILE *f = fopen(fname, "r");
    if (f == NULL) {
      ROS_ERROR("Couldn't load object names from file %s", fname);
      return;
    }
    num_objects = 0;
    while (!feof(f)) {
      if (fgets(buf, 1024, f) == NULL)
	break;
      if (strlen(buf) == 0)
	break;
      if (buf[strlen(buf)-1] == '\n')
	buf[strlen(buf)-1] = '\0';
      models.resize(num_objects+1);
      models[num_objects].name = buf;
      num_objects++;
    }

    object_folder = object_folder_const;
    //num_objects = num_objects_const;
    //models.resize(num_objects);

    //for (int i = 0; i < num_objects; i++)
    //  models[i].name = object_names_const[i];

    
    // for each object
    for (int i = 0; i < num_objects; i++) {
      
      // load PCD file
      sprintf(fname, "%s/%s.pcd", object_folder.c_str(), models[i].name.c_str());
      pcl::PointCloud<pcl::PointXYZ> &cloud = models[i].cloud;
      if (pcl::io::loadPCDFile(fname, cloud) < 0) {
	ROS_ERROR("Couldn't load point cloud from file %s", fname);
	return;
      }
      ROS_INFO("Loaded model point cloud from %s.pcd with %lu points.",  models[i].name.c_str(), cloud.points.size());

      // load PLY file (convex hull)
      sprintf(fname, "%s/%sHull.ply", object_folder.c_str(), models[i].name.c_str());
      int num_vertices, num_faces;
      PlyVertex *hull_vertices = ply_read_vertices(fname, &num_vertices);
      PlyFace *hull_faces = ply_read_faces(fname, &num_faces);
      ROS_INFO("Loaded model convex hull from %sHull.ply with %d vertices and %d faces.",  models[i].name.c_str(), num_vertices, num_faces);
      
      // convert models from cm to m, and shift to the origin
      Vector4f centroid;
      pcl::compute3DCentroid(cloud, centroid);
      for (size_t j = 0; j < cloud.points.size (); j++) {
	cloud.points[j].x = (cloud.points[j].x - centroid(0)) / 100.;
	cloud.points[j].y = (cloud.points[j].y - centroid(1)) / 100.;
	cloud.points[j].z = (cloud.points[j].z - centroid(2)) / 100.;
      }
      for (int j = 0; j < num_vertices; j++) {
	hull_vertices[j].x = (hull_vertices[j].x - centroid(0)) / 100.;
	hull_vertices[j].y = (hull_vertices[j].y - centroid(1)) / 100.;
	hull_vertices[j].z = (hull_vertices[j].z - centroid(2)) / 100.;
      }
      

      // compute point cloud normals
      //printf("radius = .01\n");
      //compute_normals(cloud, models[i].cloud_normals, .01);
      //visualize_point_cloud_normals(models[i].cloud_normals);  //dbug

      //printf("radius = .02\n");
      //compute_normals(cloud, models[i].cloud_normals, .02);
      //visualize_point_cloud_normals(models[i].cloud_normals);  //dbug

      //printf("radius = .03\n");
      compute_normals(cloud, models[i].cloud_normals, .03);
      //visualize_point_cloud_normals(models[i].cloud_normals);  //dbug

      //printf("radius = .04\n");
      //compute_normals(cloud, models[i].cloud_normals, .04);
      //visualize_point_cloud_normals(models[i].cloud_normals);  //dbug

      //dbug
      //for (int j=0; j < models[i].cloud_normals.points.size(); j++)
      //  printf("%.4f %.4f %.4f %.4f %.4f %.4f\n", models[i].cloud_normals.points[j].x, models[i].cloud_normals.points[j].y, models[i].cloud_normals.points[j].z,
      //	       models[i].cloud_normals.points[j].normal[0], models[i].cloud_normals.points[j].normal[1], models[i].cloud_normals.points[j].normal[2]);



      

      // compute model convex hull face orientations
      vector< Vector3f > normals;
      for (int j = 0; j < num_faces; j++) {
	
	// get triangle area and normal vector (pointing out)
	Vector3f p1 = vertex_to_eigen(&hull_vertices[hull_faces[j].verts[0]]);
	Vector3f p2 = vertex_to_eigen(&hull_vertices[hull_faces[j].verts[1]]);
	Vector3f p3 = vertex_to_eigen(&hull_vertices[hull_faces[j].verts[2]]);
	double area = triangle_area(p1, p2, p3);
	Vector3f n = triangle_orientation(p1, p2, p3);
	if (n.dot(p1) < 0)
	  n = -n;
	
	// if normal is not unique, don't add a new orientation
	double norm_thresh = .01;
	bool unique_normal = true;
	for (uint k = 0; k < normals.size(); k++) {
	  if ((n - normals[k]).norm() < norm_thresh) {
	    models[i].orientation_weights[k] += area;
	    unique_normal = false;
	    break;
	  }
	}
	
	if (unique_normal) {
	  
	  // get a quaternion rotation mapping the normal to (0,0,-1) and p2-p1 to (0,1,0)
	  Vector3f r3 = -n;
	  Vector3f r2 = (p2-p1).normalized();
	  Vector3f r1 = r2.cross(r3);
	  Matrix3f R;
	  R << r1, r2, r3;
	  R = R.inverse().eval();
	  Quaternionf q(R);
	  
	  normals.push_back(n);
	  models[i].orientations.push_back(q);
	  models[i].orientation_weights.push_back(area);
	}
      }
      
      // normalize orientation weights
      double wsum = sum(models[i].orientation_weights);
      for (uint j = 0; j < models[i].orientation_weights.size(); j++)
	models[i].orientation_weights[j] /= wsum;

      // create distance transforms
      float resolution = .005;
      float padding = .05;
      models[i].distance_transform.init(models[i].cloud, resolution, padding);
    }
    
    loaded_models = true;
  }
  
  
  /* get model at pose (affine) */  
  bool ModelManager::get_model_at_pose(string model_name, Matrix4f pose_affine, pcl::PointCloud<pcl::PointXYZ> &cloud) const
  {
    for (int i = 0; i < num_objects; i++) {
      if (model_name.compare(models[i].name) == 0) {
	pcl::transformPointCloud(models[i].cloud, cloud, pose_affine);
	return true;
      }
    }
    
    return false;
  }


  /* get model at pose */  
  bool ModelManager::get_model_at_pose(string model_name, geometry_msgs::Pose pose, pcl::PointCloud<pcl::PointXYZ> &cloud) const
  {
    return get_model_at_pose(model_name, pose_to_affine_matrix(pose), cloud);
  }


  /* get model index */
  int ModelManager::get_model_index(string model_name) const
  {
    for (int i = 0; i < num_objects; i++) {
      if (model_name.compare(models[i].name) == 0) {
	return i;
      }
    }

    return -1;
  }


  /* get model pointer */
  const Model &ModelManager::get_model(string model_name) const
  {
    for (int i = 0; i < num_objects; i++) {
      if (model_name.compare(models[i].name) == 0) {
	return models[i];
      }
    }

    return NullModel;
  }


}  // namespace chaos
