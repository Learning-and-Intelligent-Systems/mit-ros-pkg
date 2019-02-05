#include <stdio.h>
#include <chaos/optimization.h>
#include "util.h"

using namespace Eigen3;


namespace chaos {

  //---------------------  Optimizer class  ---------------------//

  Optimizer::Optimizer()
  {
    max_iter_ = 500;
  }

  void Optimizer::setMaxIterations(int max_iter)
  {
    max_iter_ = max_iter;
  }

  float Optimizer::lineSearch(VectorXf x, VectorXf dfdx)
  {
    return 0.0;  //stub
  }

  void Optimizer::setFinalValue(VectorXf x)
  {
    final_value_ = x;
    final_cost_ = evaluate(x);
  }

  VectorXf Optimizer::getFinalValue()
  {
    return final_value_;
  }

  float Optimizer::getFinalCost()
  {
    return final_cost_;
  }



  //----------------  GradientOptimizer class  ----------------//

  GradientOptimizer::GradientOptimizer(float step_size) :
    Optimizer()
  {
    step_size_ = step_size;
    learning_rate_ = .95;
    use_line_search_ = false;
  }

  VectorXf GradientOptimizer::optimize()
  {
    VectorXf x = initial_value_;
    float eta = step_size_;

    for (int i = 0; i < max_iter_; i++) {
      VectorXf gx = gradient(x).normalized();
      //TODO: lineSearch
      x -= eta*gx;
      eta *= learning_rate_;
    }

    setFinalValue(x);

    return x;
  }

  VectorXf GradientOptimizer::optimize(VectorXf x0)
  {
    initial_value_ = x0;
    return optimize();
  }



  //----------------  RandomizedGradientOptimizer class  ----------------//

  RandomizedGradientOptimizer::RandomizedGradientOptimizer(float step_size, VectorXf noise) :
    GradientOptimizer(step_size)
  {
    noise_ = noise;
  }

  VectorXf RandomizedGradientOptimizer::optimize()
  {
    VectorXf x = initial_value_;
    float f = evaluate(x);
    float eta = step_size_;  // decreasing step size and acceptance threshold

    VectorXf xmin = x;
    float fmin = f;

    for (int i = 0; i < max_iter_; i++) {
      VectorXf x2 = x;
      for (int j = 0; j < x2.size(); j++)
	x2(j) += normrand(0.0, noise_(j));  // initialize x2 at x plus random noise

      VectorXf df = gradient(x);
      if (df.norm() > 0.0)
	x2 -= eta*df.normalized();  // take a step along the gradient

      //TODO: lineSearch

      float f2 = evaluate(x2);
      float accept_prob = eta / step_size_;

      if (f2 < f || frand() < accept_prob) {  // annealing (randomized acceptance)
	x = x2;
	f = f2;
	if (f < fmin) {
	  xmin = x;
	  fmin = f;
	}
      }

      eta *= learning_rate_;
    }

    setFinalValue(x);

    return x;
  }

  VectorXf RandomizedGradientOptimizer::optimize(VectorXf x0)
  {
    initial_value_ = x0;
    return optimize();
  }


  //------------ GradientFreeRandomizedGradientDescentOptimizer class -------------//

  GradientFreeRandomizedGradientOptimizer::GradientFreeRandomizedGradientOptimizer(float step_size, VectorXf noise, float gradient_step_size) :
    RandomizedGradientOptimizer(step_size, noise)
  {
    gradient_step_size_ = gradient_step_size;
  }

  VectorXf GradientFreeRandomizedGradientOptimizer::optimize()
  {
    VectorXf x = initial_value_;
    float f = evaluate(x);
    float eta = step_size_;  // decreasing step size and acceptance threshold

    VectorXf xmin = x;
    float fmin = f;

    for (int i = 0; i < max_iter_; i++) {

      // pick a random gradient direction
      VectorXf gdir(x.size());
      for (int j = 0; j < x.size(); j++)
	gdir(j) = normrand(0.0, noise_(j));
      gdir.normalize();
      VectorXf x2 = x;
      for (int j = 0; j < x.size(); j++)
	x2(j) += gradient_step_size_ * noise_(j) * gdir(j);
      float f2 = evaluate(x2);
      VectorXf dx = ((f2 - f) / (x2-x).norm()) * (x2-x);

      //cout << "gdir = " << gdir << endl;
      //cout << "gradient_step_size_ = " << gradient_step_size_ << endl;
      //cout << "f2 = " << f2 << endl;
      //cout << "dx = " << dx << endl;

      x2 = x;
      for (int j = 0; j < x2.size(); j++)
	x2(j) += normrand(0.0, noise_(j));  // initialize x2 at x plus random noise

      x2 -= eta*dx;  // take a step along the chosen gradient direction

      //TODO: lineSearch

      f2 = evaluate(x2);
      float accept_prob = eta / step_size_;

      if (f2 < f || frand() < accept_prob) {  // annealing (randomized acceptance)
	x = x2;
	f = f2;
	if (f < fmin) {
	  xmin = x;
	  fmin = f;
	}
      }

      eta *= learning_rate_;
    }

    setFinalValue(x);

    return x;
  }

  VectorXf GradientFreeRandomizedGradientOptimizer::optimize(VectorXf x0)
  {
    initial_value_ = x0;
    return optimize();
  }

  VectorXf GradientFreeRandomizedGradientOptimizer::gradient(VectorXf x)   // dummy to make GradientOptimizer happy
  {
    return VectorXf::Zero(x.size());
  }



  //----------------  GridOptimizer class  ----------------//

  void GridOptimizer::setBounds(VectorXf xmin, VectorXf xmax)
  {
    xmin_ = xmin;
    xmax_ = xmax;
  }

  void GridOptimizer::setResolution(VectorXf resolution)
  {
    resolution_ = resolution;
  }

  VectorXf GridOptimizer::optimize()
  {
    int i, n = xmin_.size();
    VectorXf x = xmin_;
    float f_opt = std::numeric_limits<float>::max();
    VectorXf x_opt;

    while (1) {
      float f = evaluate(x);
      if (f < f_opt) {
	f_opt = f;
	x_opt = x;
      }

      // go to next grid point
      for (i = n-1; i >= 0; i--) {
	x(i) += resolution_(i);
	if (x(i) > xmax_(i))
	  x(i) = xmin_(i);
	else
	  break;
      }
      if (i < 0)
	break;
    }

    setFinalValue(x_opt);

    return x_opt;
  }

}
