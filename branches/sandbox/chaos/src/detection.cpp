

#include <chaos/detection.h>
#include <chaos/models.h>
#include <chaos/optimization.h>
#include <chaos/scene.h>
#include "ply.h"
#include "util.h"
#include "fitness.h"




namespace chaos {



  //------------------ INTERNAL CLASSES -------------------//


  // Given Hypothesizers H1:1->many, H2:1->many, H3:many->1,
  //   Breadth-first:  (H1->H2)->H3 takes O(N*M) space
  //   Depth-first:    H1->(H2->H3) takes O(N+M) space

  //BranchingHypothesizer *H4 = new BranchingHypothesizer(H2);  // applies H2 to each hypothesis in a VariableHypothesis
  //HypothesizerChain *H5 = new HypothesizerChain(H1, H4);      // applies H1, then H4
  //HypothesizerChain *H = new HypothesizerChain(H5, H3);       // applies H5, then H3




  //---------------------  ObjectHypothesizer class  --------------------//

  class ObjectHypothesizer : public Hypothesizer {
  public:
    Hypothesizer *identity_hypothesizer;
    Hypothesizer *pose_hypothesizer;

    ObjectHypothesizer(Hypothesizer *H_id, Hypothesizer *H_pose)
    {
      identity_hypothesizer = H_id;
      pose_hypothesizer = H_pose;
    }

    Hypothesis *getHypothesis(Hypothesis *assumptions)
    {
      Hypothesis *h_obj;

      // get a list of possible object identities
      Hypothesis *h_id = identity_hypothesizer->getHypothesis(assumptions);

      if (h_id->isVariable()) {
	const vector<Hypothesis*> &h_ids = ((VariableHypothesis *)h_id)->getHypothesisList();

	for (uint i = 0; i < h_ids.size(); i++) {
	  h_obj = pose_hypothesizer->getHypothesis(h_ids[i]);  // get a list of possible object poses

	  // align the object


	}
      }
      else { // !h_id->isVariable()
	h_obj = pose_hypothesizer->getHypothesis(h_id);  // get a list of possible object poses

	  // align the object


      }
      
      return h_obj;
    }
  };


  //---------------------  NaiveObjectIdentityHypothesizer class  --------------------//

  class NaiveObjectIdentityHypothesizer : public Hypothesizer {
  public:
    const ModelManager &model_manager;

    NaiveObjectIdentityHypothesizer(const ModelManager &M) :
      model_manager(M)
    {
    }

    Hypothesis *getHypothesis(Hypothesis *assumptions)
    {
      return NULL;  //stub
    }
  };


  //---------------------  RestingSurfacePoseHypothesizer class  --------------------//

  class RestingSurfacePoseHypothesizer : public Hypothesizer {
  public:
    const ModelManager &model_manager;

    RestingSurfacePoseHypothesizer(const ModelManager &M) :
      model_manager(M)
    {
    }

    Hypothesis *getHypothesis(Hypothesis *assumptions)
    {
      return NULL;  //stub
    }
  };


  //---------------------  OLFPoseHypothesizer class --------------------//

  class OLFPoseHypothesizer : public Hypothesizer {
  public:
    const ModelManager &model_manager;

    OLFPoseHypothesizer(const ModelManager &M) :
      model_manager(M)
    {
    }

    Hypothesis *getHypothesis(Hypothesis *assumptions)
    {
      return NULL;  //stub
    }
  };


  //---------------------  ModelPoseOptimizer class  --------------------//

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


  

  

  //--------------------- EXTERNAL API --------------------//
  


  /*
   * TODO:
   *
   *   - Use a PoseHypothesizer to abstract away initial pose guessing.
   *   - Make two PoseHypothesizer classes:
   *      --> RestingSurfacePoseHypothesizer
   *      --> OLFPoseHypothesizer
   *
   */

  
  ObjectAnalysis fit_object(const ModelManager &model_manager, ModelTester *model_tester,
			      geometry_msgs::Pose init_pose, float table_height)
  {
    Matrix4f A0 = pose_to_affine_matrix(init_pose);

    ObjectAnalysis fit_model;
    
    // try fitting each model to the point cloud at several orientations
    fit_model.fitness_score = std::numeric_limits<float>::max();
    
    for (uint i = 0; i < model_manager.models.size(); i++) {

      float min_score = std::numeric_limits<float>::max();

      double t1, t2;
      t1 = get_time_ms();

#     pragma omp parallel for 
      for (uint j = 0; j < model_manager.models[i].orientations.size(); j++) {  //dbug
	
	// rotate model onto a face of its convex hull
	Quaternionf q1 = model_manager.models[i].orientations[j];
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

  





  
}  // end of namespace chaos

