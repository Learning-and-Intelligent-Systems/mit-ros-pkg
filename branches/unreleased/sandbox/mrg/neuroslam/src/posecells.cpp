/*
 * posecells.cpp
 *
 *  Created on: Feb 23, 2011
 *      Author: hordurj
 */

#include "neuroslam/posecells.hpp"

using std::min;
using std::max;
using namespace Eigen;

namespace neuroslam
{
bool operator< (const ViewPose & a, const ViewPose & b)
{
  return  a.i < b.i || (a.i==b.i && (a.x < b.x || (a.x==b.x && (a.y < b.y || (a.y==b.y && a.t < b.t)))));
}

void wrappedCopy(int x, int y, int nx, int ny, const Eigen::ArrayXXd & in, Eigen::ArrayXXd & out)
{
  int w = nx - x;
  int h = ny - y;
  out.block(0,0,w,h) = in.block(x,y,w,h);

  if (x>0) {
    out.block(w,0,x,h) = in.block(0,y,x,h);
  }
  if (y>0) {
    out.block(0,h,w,y) = in.block(x,0,w,y);
  }
  if (x>0 && y>0) {
    out.block(w,h,x,y) = in.block(0,0,x,y);
  }
}

inline double g(double u, int v)
{
  if (v==1) return u;
  else return 1.0-u;
}

//// Sparse implementation of PoseCells
PoseCellsSparse::PoseCellsSparse(ViewCells* vc) : n_x_(60), n_y_(60), n_t_(36),
    s_x_(0.25), s_y_(0.25), s_t_(10),
//  Number from IJRR 2009 paper
//    k_p_e_(4.0), k_d_e_(4.0),  // four cells
//    k_p_i_(8.0), k_d_i_(8.0),  // eight cells
    k_p_e_(2.0), k_d_e_(2.0),
    k_p_i_(4.0), k_d_i_(4.0),
    theta_(0.00002), vc_(vc), delta_(0.2), lambda_(0.36)
{
  P_array = Array<ArrayXXd, Dynamic, 1>(n_t_);
  for (int i=0; i<n_t_; ++i) P_array(i) = Eigen::ArrayXXd::Zero(n_x_, n_y_);

  // Initialize the CAN function
  for (int a=-e_width_; a<=e_width_;++a) {
    for (int b=-e_width_; b<=e_width_;++b) {
      for (int c=-e_width_; c<=e_width_;++c) {
        // The function from the IJRR 2009 paper
        // e_{a,b,c} =   exp(-(a^2+b^2)/k_p_e) * exp(-c^2/k_d_e))
        //             - exp(-(a^2+b^2)/k_p_i) * exp(-c^2/k_d_i))

        e_abc(a,b,c) =  exp(-(a*a+b*b)/k_p_e_)/sqrt(k_p_e_)*exp(-(c*c)/k_d_e_)/sqrt(k_d_e_)
                       -exp(-(a*a+b*b)/k_p_i_)/sqrt(k_p_i_)*exp(-(c*c)/k_d_i_)/sqrt(k_d_i_);
      }
    }
  }

  set(0,n_x_/2,n_y_/2,1.0);
  for (int i=0; i<5; ++i) update();

}

void PoseCellsSparse::addOdometry(int64_t utime, const isam::Pose3d & delta)
{
  last_utime_ = utime;
  delta_pose_ = delta_pose_.oplus(delta);
}

// Calculates and prints out network stats
void PoseCellsSparse::printStats()
{
  double maxP = 0.0;
  double sumP = 0.0;
  for (PoseCellMap::iterator it=P.begin(); it!=P.end(); ++it)
  {
    maxP = max(maxP, it->second);
    sumP += it->second;
  }

  std::cout << "MaxP: " << maxP << " SumP: " << sumP << std::endl;
}

double PoseCellsSparse::get(int t, int x, int y)
{
  PoseCellMap::iterator it = P.find(
      CellKey((t+n_t_) % n_t_,
              (x+n_x_) % n_x_,
              (y+n_y_) % n_y_));

  if (it==P.end()) return 0.0;
  else return it->second;
}

void PoseCellsSparse::set(int t, int x, int y, double val)
{
  P[CellKey(
      (t+n_t_) % n_t_,
      (x+n_x_) % n_x_,
      (y+n_y_) % n_y_)] = val;
}

void PoseCellsSparse::updateViewCells()
{
  const std::vector<std::pair<int,double> > & view_cells = vc_->getActiveCells();
  for (PoseCellMap::iterator it=P.begin(); it!=P.end(); ++it)
  {
    const CellKey & cell= it->first;
    double p = it->second;

    for (std::vector<std::pair<int,double> >::const_iterator
        it = view_cells.begin(); it != view_cells.end(); ++it)
    {
      int i = it->first;
      double v = it->second;

      // Update weight using Bxyt = max(Bxyt, lambda*v*p)
      double lvp = lambda_*v*p;
      std::map<int, std::map<CellKey,double> >::iterator it_view = B.find(i);
      if (it_view == B.end())
      {
        // No weight collection for this view cell
        B[i] = std::map<CellKey, double>();
        B[i][cell] = lvp;
      }
      else
      {
        std::map<CellKey,double>::iterator it_cell = it_view->second.find(cell);
        if (it_cell == it_view->second.end())
          it_view->second[cell] = lvp; // No weight for this pose cell
        else
          it_cell->second = max(it_cell->second, lvp);
      }
    }
  }

  for (std::vector<std::pair<int,double> >::const_iterator
      it = view_cells.begin(); it != view_cells.end(); ++it)
  {
    int i = it->first;
    double v = it->second;
    std::map<int,std::map<CellKey, double> >::iterator it_by_view = B.find(i);
    if (it_by_view != B.end())
    {
      for (std::map<CellKey,double>::iterator it2 = it_by_view->second.begin(); it2!=it_by_view->second.end(); ++it2)
      {
        const CellKey & p = it2->first;
        set(p.get<0>(),p.get<1>(),p.get<2>(), get(p.get<0>(),p.get<1>(),p.get<2>()) + delta_*v*it2->second);
      }
    }
  }
}

void PoseCellsSparse::updatePathIntegration()
{
  // @todo  improve path integration
  //        the current model is really restriced to forward motion.

  // Notes on the path integration as described in Milford 2004
  // 1) The delta update is dP_ijk = P_(i+dx,j+dy,k+dt)
  //    which results in reflection of the coordinate system.
  //    Because if the motion is dx then x at time t+1 should be
  //    x^(t+1)_x+dx = x^t_x
  // 2) The formulas assume only forward motion.
  // 3) Need to look into if the fractional behavior
  //    is correct for negative velocity/change.
  //    e.g. dx,dy or dt are negative
  //
  // In the current implementation have changed the dP to
  //  dP_ijk = P_(i-dx,j-dy,k-dt)
  //

  // Shift activity based on path integration
  //Array<ArrayXXd, Dynamic, 1> nP = P;
  // Updates to cell activities
  std::vector<std::pair<CellKey, double> > nP;

  double kx = 1.0/s_x_;
  double ky = 1.0/s_y_;
  double kt = 1.0/(s_t_ / 180.0 * M_PI);

  double dtime = (last_utime_ - last_update_) / 1E6;
  if (dtime > 0)
  {
    double v = sqrt(delta_pose_.x()*delta_pose_.x() + delta_pose_.y()*delta_pose_.y()); // Linear change
    double w = delta_pose_.yaw(); // Rotational change

    int dt = floor(kt * w);
    double dtf = kt * w - dt;

    for (PoseCellMap::iterator it=P.begin(); it!=P.end(); ++it)
    {
      const CellKey & key = it->first;
      int t = key.get<0>();
      int x = key.get<1>();
      int y = key.get<2>();

      double th = (double)t/n_t_ * 2.0 * M_PI;

      int dx = floor(kx * v * cos(th));
      int dy = floor(ky * v * sin(th));
      double dxf = kx * v * cos(th) - dx;
      double dyf = ky * v * sin(th) - dy;

      double p = 0.0;
      for (int a=dx; a<=(dx+1); ++a) {
        for (int b=dy; b<=(dy+1); ++b) {
          for (int c=dt; c<=(dt+1); ++c) {
            double alpha = g(dxf, a-dx)*g(dyf, b-dy)*g(dtf, c-dt);
            p += alpha*get(t-c,x-a,y-b);
          }
        }
      }
      nP.push_back(std::pair<CellKey, double>(key, p));
    }
  }

  // Apply changes
  //P = nP;
  for (std::vector<std::pair<CellKey, double> >::iterator it=nP.begin(); it!=nP.end(); ++it)
  {
    const CellKey & p = it->first;
    set(p.get<0>(), p.get<1>(), p.get<2>(), it->second);
  }
}

void PoseCellsSparse::updateCAN()
{
  // Update with network dynamics

  // Updates to cell activities
  std::vector<std::pair<CellKey, double> > nP;

  // Compute the attractor dynamics
  //  Pxyt  = sum Pijk * e_{a,b,c}
  for (PoseCellMap::iterator it=P.begin(); it!=P.end(); ++it)
  {
    const CellKey & key = it->first;
    int t = key.get<0>();
    int x = key.get<1>();
    int y = key.get<2>();

    double p = it->second;
    for (int a=-e_width_; a<=e_width_;++a) {
      for (int b=-e_width_; b<=e_width_;++b) {
        for (int c=-e_width_; c<=e_width_;++c) {
          // P'(k)(i,j) += p * e;
          double e = e_abc(a,b,c);
          nP.push_back(std::pair<CellKey, double>(CellKey(t+c,x+a,y+b),p*e));
        }
      }
    }
  }
  // P = nP;
  for (std::vector<std::pair<CellKey, double> >::iterator it=nP.begin(); it!=nP.end(); ++it)
  {
    CellKey & p = it->first;
    set(p.get<0>(), p.get<1>(), p.get<2>(), get(p.get<0>(),p.get<1>(),p.get<2>())+it->second);
  }
}

void PoseCellsSparse::update()
{
  updatePathIntegration();
  updateViewCells();
  updateCAN();

  // Find the most active node and report as
  //  current pose estimate
  double maxP = 0.0;
  isam::Pose3d maxPose;
  double sumP = 0.0;

  for (PoseCellMap::iterator it=P.begin(); it!=P.end();) // ++it)
  {
    const CellKey & key = it->first;
    int t = key.get<0>();
    int x = key.get<1>();
    int y = key.get<2>();

    if (it->second > maxP) {
      maxP = it->second;
      maxPose = isam::Pose3d(x*s_x_,y*s_y_,0,(t*s_t_)/180.0*M_PI,0,0);
    }

    it->second = max(it->second - theta_, 0.0);
    sumP += it->second;
    // if zero remove
    if (it->second<=0.0) it = P.erase(it);
    else ++it;
  }
  last_cell_pose_ = maxPose;

  // @todo last_pose_ should unwrap when going over network boundaries
  last_pose_ = maxPose;

  std::cout << "PoseCell::update(): max p: " << maxP << " max pose: " << last_pose_ << std::endl;
  // Normalize
  for (PoseCellMap::iterator it=P.begin(); it!=P.end(); ++it) it->second /= sumP;

  // Update P_array for visualization
  for (int i=0; i<n_t_; ++i) P_array(i) = Eigen::ArrayXXd::Zero(n_x_, n_y_);
  for (PoseCellMap::iterator it=P.begin(); it!=P.end(); ++it)
  {
    const CellKey & key = it->first;
    int t = key.get<0>();
    int x = key.get<1>();
    int y = key.get<2>();
    P_array(t)(x,y) = it->second;
  }

  delta_pose_ = isam::Pose3d(0.0,0.0,0.0,0.0,0.0,0.0);
  last_update_ = last_utime_;
}

PoseCells::PoseCells(ViewCells* vc) : n_x_(60), n_y_(60), n_t_(36),
    s_x_(0.25), s_y_(0.25), s_t_(10),
//  Number from IJRR 2009 paper
//    k_p_e_(4.0), k_d_e_(4.0),  // four cells
//    k_p_i_(8.0), k_d_i_(8.0),  // eight cells
    k_p_e_(2.0), k_d_e_(2.0),
    k_p_i_(4.0), k_d_i_(4.0),
    theta_(0.00002), vc_(vc), delta_(0.1), lambda_(0.36)
{
  P = Array<ArrayXXd, Dynamic, 1>(n_t_);
  for (int i=0; i<n_t_; ++i) P(i) = Eigen::ArrayXXd::Zero(n_x_, n_y_);

  // Initialize in the center
  //P(n_t_/2)(n_x_/2,n_y_/2) = 1.0;
  P(0)(n_x_/2,n_y_/2) = 1.0;
  for (int i=0; i<5; ++i) update();

  std::cout << "PoseCells initialized" << std::endl;
}

void PoseCells::addOdometry(int64_t utime, const isam::Pose3d & delta)
{
  last_utime_ = utime;
  delta_pose_ = delta_pose_.oplus(delta);
}

// Calculates and prints out network stats
void PoseCells::printStats()
{
  double maxP = 0.0;
  for (int t=0; t<n_t_; ++t)
    for (int x=0; x<n_x_; ++x)
      for (int y=0; y<n_y_; ++y) maxP = max(maxP, P(t)(x,y));

  double sumP = 0.0;
  for (int t=0; t<n_t_; ++t)
    sumP += P(t).sum();

  std::cout << "MaxP: " << maxP << " SumP: " << sumP << std::endl;
}

void PoseCells::updateViewCells()
{
  for (int t=0; t<n_t_; ++t) {
    for (int x=0; x<n_x_; ++x) {
      for (int y=0; y<n_y_; ++y) {
        double p = P(t)(x,y);
        if (p>theta_) { //(p>0.0) {
          size_t n = vc_->count();
          double vB_sum = 0.0;
          double vSum = 0.0;
          int activeCount = 0;
          for (size_t i=0; i<n; ++i) {
            double v = vc_->score(i);
            if (v > 0.0)
            {
              // update weight i, x,y,t
              ViewPose idx(i,x,y,t); // = boost::make_tuple(i,x,y,t);
              std::map<const ViewPose, double>::const_iterator it = B.find(idx);
              if (it == B.end()) {
                B[idx] = lambda_*v*p;
                std::map<int,std::map<ViewPose,double> >::iterator it_by_view = B_by_view_id.find(i);
                if (it_by_view == B_by_view_id.end()) {
                  B_by_view_id[i] = std::map<ViewPose, double>();
                }

                B_by_view_id[i][idx] = 0.0;
              }
              else B[idx] = max(it->second, lambda_*v*p);

              //std::cout << "weights: " << (lambda_*v*p) << " B= " << B[idx] << std::endl;

              vB_sum += v*B[idx];
              ++activeCount;
              vSum += v;
            }
          }
          //if (activeCount>0)
          //  std::cout << "update weight: vSum=" << vSum << " active=" << activeCount << " n=" << n << " p=" << p << " vB: " << vB_sum << std::endl;
          //P(t)(x,y) += vB_sum;
        }
      }
    }
  }

  size_t n = vc_->count();
  for (size_t i=0; i<n; ++i) {
    double v = vc_->score(i);
    if (v > 0.0)
    {
      std::map<int,std::map<ViewPose, double> >::iterator it_by_view = B_by_view_id.find(i);
      if (it_by_view != B_by_view_id.end()) {
        std::map<ViewPose,double> & weights = it_by_view->second;

        for (std::map<ViewPose,double>::iterator it = weights.begin(); it!=weights.end(); ++it)
        {
          ViewPose idx = it->first;
          P(idx.t)(idx.x,idx.y) += v*B[idx];
        }
      }
    }
  }
}

void PoseCells::updatePathIntegration()
{
  // @todo  improve path integration
  //        the current model is really restriced to forward motion.

  // Notes on the path integration as described in Milford 2004
  // 1) The delta update is dP_ijk = P_(i+dx,j+dy,k+dt)
  //    which results in reflection of the coordinate system.
  //    Because if the motion is dx then x at time t+1 should be
  //    x^(t+1)_x+dx = x^t_x
  // 2) The formulas assume only forward motion.
  // 3) Need to look into if the fractional behavior
  //    is correct for negative velocity/change.
  //    e.g. dx,dy or dt are negative
  //
  // In the current implementation have changed the dP to
  //  dP_ijk = P_(i-dx,j-dy,k-dt)
  //

  // Shift activity based on path integration
  Array<ArrayXXd, Dynamic, 1> nP = P;

  double kx = 1.0/s_x_;
  double ky = 1.0/s_y_;
  double kt = 1.0/(s_t_ / 180.0 * M_PI);

  double dtime = (last_utime_ - last_update_) / 1E6;
  if (dtime > 0)
  {
    double v = sqrt(delta_pose_.x()*delta_pose_.x() + delta_pose_.y()*delta_pose_.y()); // Linear change
    double w = delta_pose_.yaw(); // Rotational change

    std::cout << "dx= " << floor(kx * v * cos(0.0)) << " dxf: " << (kx * v * cos(0.0)-floor(kx * v * cos(0.0)))
              << " dy= " << floor(ky * v * sin(0.0)) << " dyf: " << (ky * v * sin(0.0)-floor(ky * v * sin(0.0)))
              << " v: " << v
              << " w: " << w
              << std::endl;

    int dt = floor(kt * w);
    double dtf = kt * w - dt;

    for (int t=0; t<n_t_; ++t) {
      for (int x=0; x<n_x_; ++x) {
        for (int y=0; y<n_y_; ++y) {
          double th = (double)t/n_t_ * 2.0 * M_PI;

          int dx = floor(kx * v * cos(th));
          int dy = floor(ky * v * sin(th));
          double dxf = kx * v * cos(th) - dx;
          double dyf = ky * v * sin(th) - dy;

          nP(t)(x,y) = 0.0;
          for (int a=dx; a<=(dx+1); ++a) {
            for (int b=dy; b<=(dy+1); ++b) {
              for (int c=dt; c<=(dt+1); ++c) {
                double alpha = g(dxf, a-dx)*g(dyf, b-dy)*g(dtf, c-dt);
//                nP(t)(x,y) += alpha*P((t+c+n_t_) % n_t_)((x+a+n_x_) % n_x_ , (y+b+n_y_) % n_y_);
                nP(t)(x,y) += alpha*P((t-c+n_t_) % n_t_)((x-a+n_x_) % n_x_ , (y-b+n_y_) % n_y_);
              }
            }
          }
        }
      }
    }
  }
  P = nP;
}

void PoseCells::updateCAN()
{
  // Update with network dynamics
          //
          //  a (x' - i)(mod n_x)
          //  b (y' - j)(mod n_y)
          //  c (t' - k)(mod n_t)

  Array<ArrayXXd, Dynamic, 1> nP = P;
  ArrayXXd dP = Eigen::ArrayXXd(n_x_, n_y_);

  // Compute the attractor dynamics
  /*
  for (int a=-4; a<=4; ++a) {
    for (int b=-4; b<=4; ++b) {
      for (int c=-4; c<=4; ++c) {
        for (int t=0; t<n_t_; ++t) {
          // e_{a,b,c} =   exp(-(a^2+b^2)/k_p_e) * exp(-c^2/k_d_e))
          //             - exp(-(a^2+b^2)/k_p_i) * exp(-c^2/k_d_i))
          double e =  exp(-(a*a+b*b)/k_p_e_)*exp(-(c*c)/k_d_e_);
                     -exp(-(a*a+b*b)/k_p_i_)*exp(-(c*c)/k_d_i_);

          // Construct dP for t
          wrappedCopy((a+n_x_)%n_x_, (b+n_y_)%n_y_, n_x_, n_y_, P((t+c+n_t_) % n_t_), dP);
          nP(t) += e*dP;
        }
      }
    }
  }*/

  //  Pxyt  = sum Pijk * e_{a,b,c}
  for (int t=0; t<n_t_; ++t) {
    for (int x=0; x<n_x_; ++x) {
      for (int y=0; y<n_y_; ++y) {
        double p = P(t)(x,y);
        if (p>0.0) {
          for (int a=-5; a<=5;++a) {
            for (int b=-5; b<=5;++b) {
              for (int c=-5; c<=5;++c) {
                int i = (a + x + n_x_) % n_x_;
                int j = (b + y + n_y_) % n_y_;
                int k = (c + t + n_t_) % n_t_;
                // The function from the IJRR 2009 paper
                // e_{a,b,c} =   exp(-(a^2+b^2)/k_p_e) * exp(-c^2/k_d_e))
                //             - exp(-(a^2+b^2)/k_p_i) * exp(-c^2/k_d_i))

                // Correct Mexican hat function
                double e = exp(-(a*a+b*b)/k_p_e_)/sqrt(k_p_e_)*exp(-(c*c)/k_d_e_)/sqrt(k_d_e_)
                          -exp(-(a*a+b*b)/k_p_i_)/sqrt(k_p_i_)*exp(-(c*c)/k_d_i_)/sqrt(k_d_i_);
                nP(k)(i,j) += p * e;
              }
            }
          }
        }
      }
    }
  }
  P = nP;
}

void PoseCells::update()
{
  updatePathIntegration();
  updateViewCells();
  updateCAN();

  // Find the most active node and report as
  //  current pose estimate
  double maxP = 0.0;
  isam::Pose3d maxPose;

  for (int t=0; t<n_t_; ++t) {
    for (int x=0; x<n_x_; ++x) {
      for (int y=0; y<n_y_; ++y) {
        if (P(t)(x,y) > maxP) {
          maxP = P(t)(x,y);
          maxPose = isam::Pose3d(x*s_x_,y*s_y_,0,(t*s_t_)/180.0*M_PI,0,0);
        }
      }
    }
  }

  last_cell_pose_ = maxPose;

  // @todo last_pose_ should unwrap when going over network boundaries
  last_pose_ = maxPose;

  std::cout << "max p: " << maxP << " max pose: " << last_pose_ << std::endl;

  for (int t=0; t<n_t_; ++t) {
    for (int x=0; x<n_x_; ++x) {
      for (int y=0; y<n_y_; ++y) {
        double p = P(t)(x,y);
        // P(t)(x,y) = max(p + theta_*(p-maxP),0.0);  // Global inhibition from the thesis
        P(t)(x,y) = max(p - theta_, 0.0); // Global inhibition from IJRR 2009
      }
    }
  }

  // Normalize total activation to one
  double sumP = 0.0;
  for (int t=0; t<n_t_; ++t)
    sumP += P(t).sum();

  for (int t=0; t<n_t_; ++t)
    P(t) = P(t)/sumP;

  delta_pose_ = isam::Pose3d(0.0,0.0,0.0,0.0,0.0,0.0);
  last_update_ = last_utime_;
}

}
