#include <ros/ros.h>

#include <cardboard/scope_util.h>

#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <fstream>

vector<scope_model_data_t> models_data;
vector<string> models_name;

scope_params_t params;
cu_model_data_t *cu_models;
scope_params_t *cu_params;

void save_pcd(char *filename, pcd_t pcd) {
  FILE *f;
  f = fopen(filename, "w");
  fprintf(f, "# .PCD v.6 - Point Cloud Data file format\n");

  fprintf(f, "FIELDS ");
  for (int i = 0; i < pcd.num_channels; ++i) {
    fprintf(f, "%s ", pcd.channels[i]);
  }
  fprintf(f, "\n");

  fprintf(f, "SIZE ");
  for (int i = 0; i < pcd.num_channels; ++i) {
    fprintf(f, "4 ");
  }
  fprintf(f, "\n");

  fprintf(f, "TYPE ");
  for (int i = 0; i < pcd.num_channels; ++i) {
    fprintf(f, "F ");
  }
  fprintf(f, "\n");

  fprintf(f, "COUNT ");
  for (int i = 0; i < pcd.num_channels; ++i) {
    fprintf(f, "1 ");
  }
  fprintf(f, "\n");

  // TODO: Find a way to properly pass width and height
  int width = pcd.num_points;
  int height = 1;
  fprintf(f, "WIDTH %d\n", width);
  fprintf(f, "HEIGHT %d\n", height);
  
  // TODO: Add viewpoint to pcd_t or find some other way to record it.
  
  fprintf(f, "POINTS %d\n", pcd.num_points);
  fprintf(f, "DATA ascii\n");

  for (int i = 0; i < pcd.num_points; ++i) {
    for (int j = 0; j < pcd.num_channels; ++j) {
      fprintf(f, "%e ", pcd.data[j][i]);      
    }
    fprintf(f, "\n");
  }
  
  fclose(f);
}

bool scope_serv(cardboard::Scope::Request &req, cardboard::Scope::Response &res) {
  ROS_INFO("Request received...");
  pcd_t pcd_bg_full;
  pcd_t pcd_objects_full;
  find_all_the_features(pcd_bg_full, pcd_objects_full, req);

  simple_pose_t true_pose;
  bool have_true_pose = req.true_pose.compare("") != 0;
  if (have_true_pose) {
    load_true_pose(req.true_pose, &true_pose); // Copy this into bingham util
  }

  olf_obs_t obs;
  obs.bg_pcd = &pcd_bg_full;
  obs.fpfh_pcd = &pcd_objects_full;

  scope_obs_data_t obs_data;
  get_scope_obs_data(&obs_data, &obs, &params);

  scope_samples_t *poses;

  int model_idx = -1;
  for (int i = 0; i < models_data.size(); ++i) {
    if (req.model_name.compare(models_name[i]) == 0) {
      model_idx = i;
      break;
    }
  }
  if (model_idx == -1) {
    ROS_ERROR("Invalid model name in request");
    return false;
  }

  double t0 = get_time_ms();
  if (params.use_cuda) {
    cu_obs_data_t cu_obs;
    cu_init_obs(&obs_data, &cu_obs, &params);
    poses = scope(&models_data[model_idx], &obs_data, &params, (have_true_pose ? &true_pose : NULL), &cu_models[model_idx], &cu_obs, cu_params, NULL);
    cu_free_all_the_obs_things(&cu_obs, &params);
  }
  else
    poses = scope(&models_data[model_idx], &obs_data, &params, (have_true_pose ? &true_pose : NULL), NULL, NULL, NULL, NULL);
  printf("Ran SCOPE in %.3f seconds\n", (get_time_ms() - t0) / 1000.0);

  for (int i = 0; i < poses->num_samples; ++i) {
    cardboard::ObjectHypothesis h;
    h.name = req.model_name;
    h.type = 1; // NOTE(sanja): I arbitrarily decided 1 is a real object, 0 is a plane (e.g. table).
    h.fitness_score = poses->W[i];
    geometry_msgs::Pose p;
    p.position.x = poses->samples[i].x[0];
    p.position.y = poses->samples[i].x[1];
    p.position.z = poses->samples[i].x[2];
    p.orientation.x = poses->samples[i].q[0];
    p.orientation.y = poses->samples[i].q[1];
    p.orientation.z = poses->samples[i].q[2];
    p.orientation.w = poses->samples[i].q[3];
    h.pose = p;
    res.object_samples.objects.push_back(h);
  }

  // dbug: Writing to MATLAB file
  // TODO(sanja): Add launch file stuff to specify is matlab plotting is needed

  /*FILE *f = fopen("/home/sanja/scope_results.m", "w");
  
  int n = poses->num_samples;
  fprintf(f, "model_name = %s;\n", req.model_name.c_str());

  fprintf(f, "X = [");
  for (int i = 0; i < n; i++)
    fprintf(f, "%f, %f, %f;  ", poses->samples[i].x[0], poses->samples[i].x[1], poses->samples[i].x[2]);
    fprintf(f, "];\n");

  fprintf(f, "Q = [");
  for (int i = 0; i < n; i++)
    fprintf(f, "%f, %f, %f, %f;  ", poses->samples[i].q[0], poses->samples[i].q[1], poses->samples[i].q[2], poses->samples[i].q[3]);
  fprintf(f, "];\n");

  fprintf(f, "W = [");
  for (int i = 0; i < n; i++)
    fprintf(f, "%f ", poses->W[i]);
  fprintf(f, "];\n");

  fclose(f);*/

  ROS_INFO("Request processed.");

  return true;
}

int main(int argc, char **argv) {
  // init ROS
  ros::init(argc, argv, "scope_server");
  ros::NodeHandle nh;

  string models_path = "/home/sanja/papers/icra13/data/kinect_scans/"; // NOTE(sanja): Hardcoded for now
  char* param_file = "/home/sanja/papers/icra13/exp/default_scope_params.txt";
  load_scope_params(&params, param_file);  

  // NEXT: Load all the models. Maybe make a struct with model and its name so the callback can easily identify the model. Global variable?
  string model_list = "models.txt";
  string full_model_list_path = models_path + model_list;
  
  double t0;
  t0 = get_time_ms();
  ifstream model_path_file;
  model_path_file.open(full_model_list_path.c_str());
  for(int i = 0; model_path_file.good(); ++i) {
    string subpath;
    getline(model_path_file, subpath);
    int end = subpath.find("/model.txt");
    olf_model_t olf_model;
    scope_model_data_t model;
    models_name.push_back(subpath.substr(0, end));
    string model_path = models_path + subpath;
    char *chr_path;
    safe_calloc(chr_path, model_path.length(), char);
    strcpy(chr_path, model_path.c_str());
    load_olf_model(&olf_model, chr_path, &params);
    get_scope_model_data(&model, &olf_model, &params);
    models_data.push_back(model);
  }  
  
  /*// load model data
  olf_model_t model;
  string full_path = model_path + req.model_name + "/model.txt";
  char *fp_file;
  safe_malloc(fp_file, full_path.length(), char);
  strcpy(fp_file, full_path.c_str());
  load_olf_model(&model, fp_file, &params);
  free(fp_file);
  scope_model_data_t model_data;
  get_scope_model_data(&model_data, &model, &params);*/
  printf("Got models in %.3f seconds\n", (get_time_ms() - t0) / 1000.0);
  
  if (params.use_cuda) {
    safe_calloc(cu_models, models_data.size(), cu_model_data_t);
    printf("Initializing CUDA\n");
    cu_init();
    printf("CUDA initialized\n");
    // NEXT: Break down initialization to model and obs since for the server we don't have obs readily available
    cu_init_scoring(&cu_params, &params);
    cu_init_all_models(&models_data[0], models_data.size(), cu_models);
    printf("Data copied\n");
  }

  ROS_INFO("Server started, waiting for requests...");

  ros::ServiceServer service = nh.advertiseService("scope_service", scope_serv);
  ros::spin();

  if (params.use_cuda) {
    cu_free_all_the_things_init(cu_params);
    cu_free_all_the_things_all_models(cu_models, models_data.size());
    free(cu_models);
    free(cu_params);
  }

  model_path_file.close();
  return 0;
}

    
