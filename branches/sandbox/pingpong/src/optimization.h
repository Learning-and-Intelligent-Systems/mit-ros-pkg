#ifndef PINGPONG_OPTIMIZATION_H_
#define PINGPONG_OPTIMIZATION_H_
#ifndef EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#endif

#include <Eigen/Core>

using namespace Eigen;



  /* Optimizer class */
  class Optimizer {

  private:
    VectorXf final_value_;
    float final_cost_;

  protected:
    int max_iter_;
    float lineSearch(VectorXf x, VectorXf dfdx);
    void setFinalValue(VectorXf);

  public:
    Optimizer();
    void setMaxIterations(int max_iterations);
    VectorXf getFinalValue();
    float getFinalCost();
    virtual float evaluate(VectorXf x) = 0;
    virtual VectorXf optimize() = 0;
  };


  /* GradientOptimizer class */
  class GradientOptimizer : public Optimizer {
  protected:
    float step_size_;
    float learning_rate_;
    bool use_line_search_;
    VectorXf initial_value_;
  public:
    GradientOptimizer(float step_size);
    VectorXf optimize();
    VectorXf optimize(VectorXf x);
    virtual VectorXf gradient(VectorXf x) = 0;
  };


  /* RandomizedGradientOptimizer class */
  class RandomizedGradientOptimizer : public GradientOptimizer {
  protected:
    VectorXf noise_;
  public:
    RandomizedGradientOptimizer(float step_size, VectorXf noise);
    VectorXf optimize();
    VectorXf optimize(VectorXf x);
  };


  /* GradientFreeRandomizedGradientDescentOptimizer class */
  class GradientFreeRandomizedGradientOptimizer : public RandomizedGradientOptimizer {
  protected:
    float gradient_step_size_;
  public:
    GradientFreeRandomizedGradientOptimizer(float step_size, VectorXf noise, float gradient_step_size);
    VectorXf optimize();
    VectorXf optimize(VectorXf x);
    VectorXf gradient(VectorXf x);  // dummy to make GradientOptimizer happy
  };


  /* GridOptimizer class */
  class GridOptimizer : public Optimizer {
  protected:
    VectorXf resolution_;
    VectorXf xmin_;
    VectorXf xmax_;
  public:
    void setBounds(VectorXf xmin, VectorXf xmax);
    void setResolution(VectorXf resolution);
    VectorXf optimize();
  };



#endif
