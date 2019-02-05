#ifndef CARDBOARD_TESTING_H_
#define CARDBOARD_TESTING_H_
#ifndef EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#endif

#include <cardboard/common.h>
#include <cardboard/models.h>



namespace cardboard {

  class ModelTester {
  public:
    virtual float testHypothesis(const Model &model, Matrix4f affine_pose) const = 0;   // TODO: make this more general!
  };

  class CompositeTester : public ModelTester {
  public:
    vector<ModelTester*> testers;
    vector<float> weights;
    CompositeTester();
    CompositeTester(vector<ModelTester*> T);
    CompositeTester(vector<ModelTester*> T, vector<float> W);
    CompositeTester(ModelTester *T1, ModelTester *T2);
    CompositeTester(ModelTester *T1, ModelTester *T2, float w1, float w2);
    CompositeTester(ModelTester *T1, ModelTester *T2, ModelTester *T3);
    CompositeTester(ModelTester *T1, ModelTester *T2, ModelTester *T3, float w1, float w2, float w3);
    float testHypothesis(const Model &model, Matrix4f affine_pose) const;
  };


  /*
   * RangeImageTester computes the fitness of a model range image to
   * an observed range image.
   */
  class RangeImageTester : public ModelTester {
  protected:
    pcl::RangeImage range_image_;
    MatrixXf cost_map_;
    float background_range_cost_;
    float boundary_noise_sigma_;
    void computeCostMap();
  public:
    RangeImageTester(const pcl::RangeImage &range_image);
    void setBoundaryNoiseSigma(float boundary_noise_sigma);
    void setBackgroundRangeCost(float background_range_cost);
    float testHypothesis(const Model &model, Matrix4f affine_pose) const;
    float testHypothesis(const Model &model, Matrix4f affine_pose, bool debug) const;  //dbug
  };


  /*
   * PointCloudDistanceTester computes the fitness of an observed point cloud
   * with respect to a model distance transform.
   */
  class PointCloudDistanceTester : public ModelTester {
  protected:
    PointCloudXYZ point_cloud_;  // TODO: use use more general point type
    //DistanceTransform3D distance_transform_;
  public:
    PointCloudDistanceTester(const PointCloudXYZ &point_cloud);
    float testHypothesis(const Model &model, Matrix4f affine_pose) const;
  };


  /*
   * RangeImageNormalTester computes the fitness of the normals of a model range image to
   * the normals of an observed range image.
   */
  class RangeImageNormalTester : public ModelTester {
  protected:
    pcl::RangeImage range_image_normals_;
  public:
    RangeImageNormalTester(const pcl::RangeImage &range_image_normals);
    float testHypothesis(const Model &model, Matrix4f affine_pose) const;
    float testHypothesis(const Model &model, Matrix4f affine_pose, bool debug) const;  //dbug
  };




}  // namespace cardboard





#endif
