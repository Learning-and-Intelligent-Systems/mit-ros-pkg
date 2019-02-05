#ifndef CHAOS_MODELS_H_
#define CHAOS_MODELS_H_


#include <chaos/common.h>
#include <chaos/transforms.h>



//TODO -- adopt ROS naming conventions


namespace chaos {


  class Model {
  public:
    string name;
    PointCloudXYZ cloud;                      // model point cloud
    PointCloudXYZN cloud_normals;             // model point cloud with normals
    vector<Quaternionf> orientations;         // possible resting orientations (from convex hull faces)
    vector<double> orientation_weights;       // orientation probabilities
    DistanceTransform3D distance_transform;   // model distance transform
  };

  const Model NullModel;


  class ModelManager {

  public:
    ModelManager();

    string object_folder;
    int num_objects;
    bool loaded_models;
    vector<Model> models;

    // Load model point clouds and convex hulls (called by the constructor)
    void load_models();

    // Get a model at a given pose
    bool get_model_at_pose(string model_name, geometry_msgs::Pose pose, PointCloudXYZ &cloud) const;
    bool get_model_at_pose(string model_name, Matrix4f pose, PointCloudXYZ &cloud) const;

    int get_model_index(string model_name) const;
    const Model &get_model(string model_name) const;
  };


  //------------------ GLOBAL VARIABLES -------------------//
  
  

  


}  // namespace chaos



#endif

