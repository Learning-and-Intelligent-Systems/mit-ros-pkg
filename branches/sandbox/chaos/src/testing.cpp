
#include <chaos/testing.h>
#include "fitness.h"
#include "util.h"


namespace chaos {



  //------------------ ModelTester class ------------------//

  float ModelTester::testHypothesis(const Model &model, Matrix4f affine_pose) const
  {
    return 0.0;
  }



  //------------------ CompositeTester class ------------------//

  CompositeTester::CompositeTester()
  {
  }

  CompositeTester::CompositeTester(vector<ModelTester*> T)
  {
    testers = T;
    for (uint i = 0; i < T.size(); i++)
      weights[i] = 1.0;
  }

  CompositeTester::CompositeTester(vector<ModelTester*> T, vector<float> W)
  {
    testers = T;
    weights = W;
  }

  CompositeTester::CompositeTester(ModelTester *T1, ModelTester *T2)
  {
    testers.resize(2);
    weights.resize(2);
    testers[0] = T1;
    testers[1] = T2;
    weights[0] = 1.0;
    weights[1] = 1.0;
  }

  CompositeTester::CompositeTester(ModelTester *T1, ModelTester *T2, float w1, float w2)
  {
    testers.resize(2);
    weights.resize(2);
    testers[0] = T1;
    testers[1] = T2;
    weights[0] = w1;
    weights[1] = w2;
  }

  CompositeTester::CompositeTester(ModelTester *T1, ModelTester *T2, ModelTester *T3)
  {
    testers.resize(3);
    weights.resize(3);
    testers[0] = T1;
    testers[1] = T2;
    testers[2] = T3;
    weights[0] = 1.0;
    weights[1] = 1.0;
    weights[2] = 1.0;
  }

  CompositeTester::CompositeTester(ModelTester *T1, ModelTester *T2, ModelTester *T3, float w1, float w2, float w3)
  {
    testers.resize(3);
    weights.resize(3);
    testers[0] = T1;
    testers[1] = T2;
    testers[2] = T3;
    weights[0] = w1;
    weights[1] = w2;
    weights[2] = w3;
  }

  float CompositeTester::testHypothesis(const Model &model, Matrix4f affine_pose) const
  {
    float score = 0.0;
    for (uint i = 0; i < testers.size(); i++)
      score += weights[i] * testers[i]->testHypothesis(model, affine_pose);

    return score;
  }



  //------------------ RangeImageTester class ------------------//

  RangeImageTester::RangeImageTester(const pcl::RangeImage &range_image)
  {
    range_image_ = range_image;
    background_range_cost_ = .01;  // .1 m effective background distance
    boundary_noise_sigma_ = 1.0;   // 1.0 pixel fuzzy boundary noise

    computeCostMap();
  }

  void RangeImageTester::computeCostMap()
  {
    cost_map_ = background_range_cost_ * get_far_range_matrix(range_image_);

    // printf("------ before blur ------\n");
    //cout << cost_map_ << endl;

    cost_map_ = blur_matrix(cost_map_, boundary_noise_sigma_);

    //printf("------ after blur ------\n");
    //cout << cost_map_;
  }

  void RangeImageTester::setBackgroundRangeCost(float background_range_cost)
  {
    background_range_cost_ = background_range_cost;
    computeCostMap();
  }

  void RangeImageTester::setBoundaryNoiseSigma(float boundary_noise_sigma)
  {
    boundary_noise_sigma_ = boundary_noise_sigma;
    computeCostMap();
  }

  float RangeImageTester::testHypothesis(const Model &model, Matrix4f affine_pose) const
  {
    PointCloudXYZ cloud;
    pcl::transformPointCloud(model.cloud, cloud, affine_pose);

    return 500*range_image_fitness(range_image_, cost_map_, cloud, background_range_cost_);
  }

  float RangeImageTester::testHypothesis(const Model &model, Matrix4f affine_pose, bool debug) const
  {
    PointCloudXYZ cloud;
    pcl::transformPointCloud(model.cloud, cloud, affine_pose);

    return 1500*range_image_fitness(range_image_, cost_map_, cloud, background_range_cost_, debug);
  }


  //------------------ PointCloudDistanceTester class ------------------//

  PointCloudDistanceTester::PointCloudDistanceTester(const pcl::PointCloud<pcl::PointXYZ> &point_cloud)
  {
    point_cloud_ = point_cloud;

    // create distance transform
    //float resolution = .005;
    //float padding = .05;
    //distance_transform_.init(point_cloud, resolution, padding);
  }

  float PointCloudDistanceTester::testHypothesis(const Model &model, Matrix4f affine_pose) const
  {
    Matrix4f A_inv = affine_pose.inverse();
    PointCloudXYZ cloud_transformed;
    pcl::transformPointCloud(point_cloud_, cloud_transformed, A_inv);
    float score = 15000*point_cloud_fitness(model.distance_transform, cloud_transformed);  // cloud to model

    //Matrix4f A = affine_pose;
    //PointCloudXYZ model_transformed;
    //pcl::transformPointCloud(model.cloud, model_transformed, A);
    //score += 2500*point_cloud_fitness(distance_transform_, model_transformed);  // model to cloud

    return score;
  }



  //----------------- RangeImageNormalTester ----------------//

  RangeImageNormalTester::RangeImageNormalTester(const pcl::RangeImage &range_image_normals) :
    range_image_normals_(range_image_normals)
  {
  }

  float RangeImageNormalTester::testHypothesis(const Model &model, Matrix4f affine_pose) const
  {
    PointCloudXYZN model_normals;
    pcl::transformPointCloudWithNormals(model.cloud_normals, model_normals, affine_pose);

    return range_image_normal_fitness(range_image_normals_, model_normals);
  }

  float RangeImageNormalTester::testHypothesis(const Model &model, Matrix4f affine_pose, bool debug) const
  {
    PointCloudXYZN model_normals;
    pcl::transformPointCloudWithNormals(model.cloud_normals, model_normals, affine_pose);

    return range_image_normal_fitness(range_image_normals_, model_normals, debug);
  }


}  // namespace chaos
