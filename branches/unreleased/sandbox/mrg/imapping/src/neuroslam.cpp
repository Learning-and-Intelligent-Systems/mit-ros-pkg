/*
 * neuroslam.cpp
 *
 *  Created on: Feb 9, 2011
 *      Author: Hordur Johannsson
 *
 * Neuro SLAM is a re-implementation of the RatSLAM algorithm.
 *
 * Reference: Persistent Navigation and Mapping using a Biologically
 *            Inspired SLAM system, Michael Milford and Gordon Wyeth,
 *            IJRR 2009
 *
 */

#include <iostream>
#include <map>
#include <cmath>
#include <algorithm>
#include <Eigen/Core>
#include <bot_core/rotations.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/stereo_camera_model.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/CvBridge.h>

#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_comparison.hpp>
#include <boost/unordered_map.hpp>
#include <boost/thread.hpp>
#include <boost/functional/hash.hpp>

#include "nav_msgs/Odometry.h"
#include "imapping/util.hpp"
#include "isam/isam.h"
#include "viewer.h"

using std::min;
using std::max;
using namespace Eigen;

lcm_t* g_lcm;

void odometry_to_pose3d(const nav_msgs::Odometry::ConstPtr& odom, isam::Pose3d & pose)
{
  double rpy[3];
  double q[4];
  q[0] = odom->pose.pose.orientation.w;
  q[1] = odom->pose.pose.orientation.x;
  q[2] = odom->pose.pose.orientation.y;
  q[3] = odom->pose.pose.orientation.z;
  bot_quat_to_roll_pitch_yaw(q,rpy);

  pose = isam::Pose3d(odom->pose.pose.position.x,
                      odom->pose.pose.position.y,
                      odom->pose.pose.position.z,
                      rpy[2], rpy[1], rpy[0]);
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

// #include <isam/robust.h>
inline double cost_pseudo_huber(double d, double b) {
  double b2 = b*b;
  return 2*b2*(sqrt(1+d*d/b2) - 1);
}
double huber(double a) { return cost_pseudo_huber(a,0.5); }


//typedef boost::tuple<int,int,int,int> ViewPose;
struct ViewPose
{
  ViewPose(int i, int x, int y, int t) : i(i), x(x), y(y), t(t) {}
  int i;
  int x;
  int y;
  int t;
};
bool operator< (const ViewPose & a, const ViewPose & b)
{
  return  a.i < b.i || (a.i==b.i && (a.x < b.x || (a.x==b.x && (a.y < b.y || (a.y==b.y && a.t < b.t)))));
}

class ViewCells;

class PoseCells
{
public:
  PoseCells(ViewCells* vc);

  void addOdometry(int64_t utime, const isam::Pose3d & delta);
  void update();

  const Array<ArrayXXd, Dynamic, 1> & poseNetwork() const {return P;}

  isam::Pose3d getPose() { return last_pose_; }
  isam::Pose3d getCellPose() { return last_cell_pose_; }

private:
  void setDefaults();

  // Dimensions off pose cell network
  int n_x_;
  int n_y_;
  int n_t_;

  // Pose cell size
  double s_x_;
  double s_y_;
  double s_t_;

  // Excitation
  double k_p_e_;
  double k_d_e_;

  // Inhibition
  double k_p_i_;
  double k_d_i_;

  // Theta
  double theta_;

  Array<ArrayXXd, Dynamic, 1> P;

  int64_t last_update_;
  int64_t last_utime_;
  isam::Pose3d delta_pose_;

  /// The last pose in world coordinates
  isam::Pose3d last_pose_;
  /// The last pose in cell coordinates
  isam::Pose3d last_cell_pose_;

  ViewCells* vc_;
  double delta_;
  double lambda_;

  /// Weights between view cells and pose cells
  //boost::unordered_map<ViewPose, double> B;
  std::map<const ViewPose, double> B;
  std::map<int, std::map<ViewPose,double> > B_by_view_id;

  void updateViewCells();
  void updatePathIntegration();
  void updateCAN();

  void printStats();
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// t,x,y
typedef boost::tuple<int, int, int> CellKey;
struct CellHash : std::unary_function<CellKey, std::size_t>
{
  size_t operator()(CellKey const& key) const
  {
    std::size_t seed = 0;
    boost::hash_combine( seed, key.get<0>() );
    boost::hash_combine( seed, key.get<1>() );
    boost::hash_combine( seed, key.get<2>() );
    return seed;
  }
};
typedef boost::unordered_map<CellKey, double, CellHash> PoseCellMap;

class PoseCellsSparse
{
public:
  PoseCellsSparse(ViewCells* vc);

  void addOdometry(int64_t utime, const isam::Pose3d & delta);
  void update();

  const Array<ArrayXXd, Dynamic, 1> & poseNetwork() const {return P_array;}

  isam::Pose3d getPose() { return last_pose_; }
  isam::Pose3d getCellPose() { return last_cell_pose_; }

  double get(int t, int x, int y);
  void set(int t, int x, int y, double val);

private:
  void setDefaults();

  // Dimensions off pose cell network
  int n_x_;
  int n_y_;
  int n_t_;

  // Pose cell size
  double s_x_;
  double s_y_;
  double s_t_;

  // Excitation
  double k_p_e_;
  double k_d_e_;

  // Inhibition
  double k_p_i_;
  double k_d_i_;

  // Theta
  double theta_;

  Array<ArrayXXd, Dynamic, 1> P_array;
  PoseCellMap P;

  int64_t last_update_;
  int64_t last_utime_;
  isam::Pose3d delta_pose_;

  /// The last pose in world coordinates
  isam::Pose3d last_pose_;
  /// The last pose in cell coordinates
  isam::Pose3d last_cell_pose_;

  ViewCells* vc_;
  double delta_;
  double lambda_;

  /// Weights between view cells and pose cells
  //boost::unordered_map<ViewPose, double> B;
  std::map<const ViewPose, double> B;
  std::map<int, std::map<ViewPose,double> > B_by_view_id;

  void updateViewCells();
  void updatePathIntegration();
  void updateCAN();

  void printStats();
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef VectorXd View;
class ViewCells
{
public:
//  ViewCells() : lambda_(0.36), m_min_(0.1), delta_(0.1), d_max_(1500.0), max_cell_id_(-1) {}
  ViewCells() : lambda_(0.36), m_min_(0.1), delta_(0.1), d_max_(1000.0), max_cell_id_(-1) {}

  /**
   * Returns the number of view cells
   */
  size_t count() const {return views_.size();}

  /**
   * Returns the current score of a view cell
   */
  double score(size_t idx) const {return score_[idx];}

  /**
   * Sets the current view.
   *
   * This method sets the active view. The score is updated relative to this
   * view. If the maximum score is below a certain threshold the view will
   * be added as a new view cell.
   *
   * @param view is the current active view.
   *
   * @return true if a new view is added
   */
  bool setView(const View & view);

  /**
   * Get the maximally activated view cell.
   *
   * @return id of the maximally activated cell
   *         -1 if none is active
   *
   */
  int getMaxView() {return max_cell_id_;}

  /**
   * Update the view cell activity using the last received view.
   */
  void update();

private:
  double calculateScore(const View & view1, const View & view2);

  double lambda_;
  double m_min_;
  double delta_;
  double d_max_;

  std::vector<View > views_;
  std::vector<double> score_;

  int max_cell_id_;
};

double ViewCells::calculateScore(const View & view1, const View & view2)
{
  double d = (view1-view2).lpNorm<1>();
  double e = 0.5;
  if (d > d_max_) return 0.0;
  else return (1.0/(d+e));
}

void ViewCells::update()
{
}

bool ViewCells::setView(const View & view)
{
  bool added = false;
  double total_score = 0.0;
  for (size_t i=0; i<views_.size(); ++i) {
    score_[i] = calculateScore(view, views_[i]);
    total_score += score_[i];
  }

  int max_id = -1;
  int max_score = 0.0;

  if (total_score == 0.0) {
    views_.push_back(view);
    score_.push_back(1.0);
    max_id = views_.size()-1;
    added = true;
  } else {
    for (size_t i=0; i<views_.size(); ++i) {
      score_[i] = score_[i]/total_score;
      if (score_[i]>0.0) {
        if (score_[i]>max_score) {
          max_id = i;
          max_score = score_[i];
        }
        std::cout << "ViewCells::setView: view = " << i << " score = " << score_[i] << " n: " << views_.size() << std::endl;
      }
    }
  }
  max_cell_id_ = max_id;

  std::cout << "ViewCells::setView: diff: " << (view-views_[max_cell_id_]).lpNorm<1>() << std::endl;


  return added;
}

struct Experience
{
  int id;
  isam::Pose3d pose;
  isam::Pose3d cell_pose;
  int view_id;
  int64_t utime;
//  ViewPose view_pose;
};

class ExperienceMap
{
public:
  ExperienceMap(PoseCellsSparse* pc, ViewCells* vc) : A_(0.5),
                                                u_p_(1.0),
                                                u_v_(1.0),
                                                S_max_(1.0),
                                                s_g_(0.1),
                                                pos_r_(1.0),
                                                angle_r_(0.2),
                                                pc_(pc),
                                                vc_(vc),
                                                last_utime_(0)
  {
    current_id_ = 0;
    Experience new_experience;
    new_experience.cell_pose = isam::Pose3d(0,0,0,0,0,0);
    new_experience.id = 0;
    new_experience.pose = isam::Pose3d(0,0,0,0,0,0);
    new_experience.view_id = -1;
    new_experience.utime = 0;
    experiences_.push_back(new_experience);

    initSlam(new_experience.pose);
  }

  void update(int64_t utime, isam::Pose3d & odometry);
  void relaxConstraints();
  Experience getExperience();
  double getExperienceEnergy(int i, int view_id, const isam::Pose3d & pose);
  const std::vector<Experience> & getExperiences() const {return experiences_;}

private:
  void initSlam(isam::Pose3d pose0);

  double A_;
  double u_p_;
  double u_v_;
  double S_max_;
  double s_g_;

  double pos_r_;        // Radius of influence for position
  double angle_r_;      // Range of influence for angle

  int current_id_;
  std::vector<Experience> experiences_;
  std::vector<std::pair<int,int> > links_;
  std::vector<isam::Pose3d> constraints_;
  isam::Pose3d last_odometry_;

  std::vector<isam::Pose3d_Node*> nodes_;
  std::vector<isam::Pose3d_Pose3d_Factor*> factors_;
  isam::Pose3d_Factor* prior_;

  PoseCellsSparse* pc_;
  ViewCells* vc_;

  isam::Slam slam_;
  int64_t last_utime_;
};

void ExperienceMap::initSlam(isam::Pose3d pose0)
{
  isam::Properties prop;
  prop.mod_solve = 1;
  prop.mod_update = 1;
  prop.mod_batch = 1; //00;
  prop.verbose = true;;
  slam_.set_properties(prop);
  slam_.set_cost_function(huber);

  // initialize with Pose0 (from cnv and rnv)
  MatrixXd sqrtinf = 100. * Matrix<double, 6, 6>::Identity();
  isam::Pose3d_Node* new_pose_node = new isam::Pose3d_Node();
  slam_.add_node(new_pose_node);
  nodes_.push_back(new_pose_node);
  prior_ = new isam::Pose3d_Factor(new_pose_node, pose0, sqrtinf);
  slam_.add_factor(prior_); // add it to the Slam graph
  slam_.update();

  // origin_ = new_pose_node;
  // slam_ = true;
  // last_node_ = new_pose_node;
}

Experience ExperienceMap::getExperience()
{
  return experiences_[current_id_];
}

double ExperienceMap::getExperienceEnergy(int i, int view_id, const isam::Pose3d & pose)
{
  const Experience & current = experiences_[i];

  //double dx = abs(current.cell_pose.x() - pose.x());
  //double dy = abs(current.cell_pose.y() - pose.y());
  //double dt = abs(current.cell_pose.yaw() - pose.yaw());

  if (view_id != current.view_id) return 0.0;

  double dx = current.cell_pose.x() - pose.x();
  double dy = current.cell_pose.y() - pose.y();
  double dt = current.cell_pose.yaw() - pose.yaw();

  double rr = sqrt(dx*dx + dy*dy) / pos_r_;
  double tr = abs(dt) / angle_r_;

  if (rr>1.0 || tr>1.0) return 0.0;
  else return 2.0-rr-tr;
}

void ExperienceMap::relaxConstraints()
{
  slam_.update();

  for (size_t i=0;i<experiences_.size();++i)
  {
    experiences_[i].pose = nodes_[i]->value();
  }

  return;
  for (size_t k=0;k<links_.size();++k)
  {
    int i = links_[k].first;
    int j = links_[k].second;

    isam::Pose3d & delta = constraints_[k];
    isam::Pose3d & pi = experiences_[i].pose;
    isam::Pose3d & pj = experiences_[j].pose;

    double a = 0.5;
    isam::Pose3d pij = pi.oplus(delta);
    isam::Pose3d pji = pj.oplus((isam::Pose3d(0,0,0,0,0,0)).ominus(delta));

    if (j>1) {
      pj.set_x(pj.x() + a*(pij.x()-pj.x()));
      pj.set_y(pj.y() + a*(pij.y()-pj.y()));
      pj.set_yaw(pj.yaw() + a*(pij.yaw()-pj.yaw()));
    }

    if (i>1) {
      pi.set_x(pi.x() + a*(pji.x()-pi.x()));
      pi.set_y(pi.y() + a*(pji.y()-pi.y()));
      pi.set_yaw(pi.yaw() + a*(pji.yaw()-pi.yaw()));
    }
  }
}

void ExperienceMap::update(int64_t utime, isam::Pose3d & odometry)
{
  // Get current view and pose cell
  isam::Pose3d cell_pose = pc_->getCellPose();
  int view_id = vc_->getMaxView();
  // ViewPose vp = vc->getLastViewPose();
  Experience current = experiences_[current_id_];

  double e_value = getExperienceEnergy(current_id_, view_id, cell_pose);
//  if (e_value > 0.0) {
//    std::cout << "Experience Map: keep current experience active: " << current_id_
//        << " view id: " << view_id << " cell_pose: " << cell_pose << " experience: " << e_value << std::endl;
//    return ;
//  }

  // note: even though current experience is within bounds
  //       there might be another experience that is
  //       maximally activated.

  // If unlike current experience
  //   check other experiences
  //   if found - activate
  //   otherwise - create a new experience
  int experience_count = 0;
  double max_energy = 0.0;
  int max_id = -1;
  for (size_t i = 0; i<experiences_.size(); ++i) {
    double e = getExperienceEnergy(i, view_id, cell_pose);
    if (e > 0.0) ++experience_count;
    if (e > max_energy) {
      max_id = i;
      max_energy = e;
    }
  }

  if (experience_count>1) {
    std::cout << "ExperienceMap: multiple experiences: keep current experience active: " << current_id_
        << " view id: " << view_id << " cell_pose: " << cell_pose << " experience: " << e_value
        << " count: " << experience_count << std::endl;

    return ;
  }

  if (max_id == current_id_) {
    std::cout << "ExperienceMap: keep current experience active: " << current_id_
        << " view id: " << view_id << " cell_pose: " << cell_pose << " experience: " << e_value << std::endl;
    std::cout << "ExperienceMap: count: " << experiences_.size() << std::endl;

    return ;
  }

  int new_id;

  if (current_id_ == 0) last_odometry_ = odometry;
  isam::Pose3d constraint = odometry.ominus(last_odometry_);

  isam::Pose3d_Node* node = 0;
  if (max_id>-1) {
    // Activate existing experience
    new_id = max_id;
    node = nodes_[new_id];

    std::cout << "ExperienceMap: switch to an existing experience: " << current_id_
        << " new experience: " << new_id
        << " view id: " << view_id << " cell_pose: " << cell_pose << " experience: " << max_energy << std::endl;
  } else {
    Experience new_experience;
    new_experience.cell_pose = cell_pose;
    new_experience.id = experiences_.size();
    new_experience.pose = experiences_[current_id_].pose.oplus(constraint);
    new_experience.view_id = view_id;
    new_experience.utime = utime;
    experiences_.push_back(new_experience);
    node = new isam::Pose3d_Node();
    nodes_.push_back(node);
    slam_.add_node(node);

    new_id = new_experience.id;

    std::cout << "ExperienceMap: create new experience: " << new_id << " view_id: " << view_id << " cell_pose: " << cell_pose << std::endl;
  }

  std::cout << "ExperienceMap: count: " << experiences_.size() << std::endl;

  // Add link
  // @todo check if link already exists
  links_.push_back(std::pair<int,int>(current_id_, new_id));
  constraints_.push_back(constraint);

  MatrixXd sqrtinf = 100. * Matrix<double, 6, 6>::Identity();
  isam::Pose3d_Pose3d_Factor* factor = new isam::Pose3d_Pose3d_Factor(nodes_[current_id_],node,constraint,sqrtinf);
  factors_.push_back(factor);
  slam_.add_factor(factor);

  std::cout << "ExperienceMap: Link Added: "
      << " Last Odometry: " << last_odometry_
      << " Odometry: " << odometry
      << " Constraint: " << constraint
      << " old id: " << current_id_
      << " new id: " << new_id
      << " last utime: " << last_utime_
      << " utime: " << utime
      << std::endl;

  std::cout << "ExperienceMap: Link Added: "
      << " from: " << nodes_[current_id_]->value()
      << " to: " << node->value()
      << std::endl;

  current_id_ = new_id;
  last_odometry_ = odometry;
  last_utime_ = utime;
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
  // Initialize in the center
  //P[CellKey(0,n_x/2,n_y/2)] = 1.0;

  P_array = Array<ArrayXXd, Dynamic, 1>(n_t_);
  for (int i=0; i<n_t_; ++i) P_array(i) = Eigen::ArrayXXd::Zero(n_x_, n_y_);

  set(0,n_x_/2,n_y_/2,1.0);
  for (int i=0; i<5; ++i) update();

  std::cout << "PoseCellsSparse initialized" << std::endl;
}

void PoseCellsSparse::addOdometry(int64_t utime, const isam::Pose3d & delta)
{
  last_utime_ = utime;
  delta_pose_ = delta_pose_.oplus(delta);
}

// Calculates and prints out network stats
void PoseCellsSparse::printStats()
{
  /*
  double maxP = 0.0;
  for (int t=0; t<n_t_; ++t)
    for (int x=0; x<n_x_; ++x)
      for (int y=0; y<n_y_; ++y) maxP = max(maxP, P(t)(x,y));

  double sumP = 0.0;
  for (int t=0; t<n_t_; ++t)
    sumP += P(t).sum();

  std::cout << "MaxP: " << maxP << " SumP: " << sumP << std::endl;
  */
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
  for (PoseCellMap::iterator it=P.begin(); it!=P.end(); ++it)
  {
    double p = it->second;
    const CellKey & key = it->first;
    int t = key.get<0>();
    int x = key.get<1>();
    int y = key.get<2>();

    size_t n = vc_->count();
    double vB_sum = 0.0;
    double vSum = 0.0;
    int activeCount = 0;
    // @todo do not loop over all view cells for each pose cell
    for (size_t i=0; i<n; ++i) {
      double v = vc_->score(i);
      if (v > 0.0)
      {
        // update weight i, x,y,t
        ViewPose idx(i,x,y,t);
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
  }
  //if (activeCount>0)
  //  std::cout << "update weight: vSum=" << vSum << " active=" << activeCount << " n=" << n << " p=" << p << " vB: " << vB_sum << std::endl;
  //P(t)(x,y) += vB_sum;

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
          //P(idx.t)(idx.x,idx.y) += v*B[idx];
          set(idx.t,idx.x,idx.y,  get(idx.t,idx.x,idx.y) + delta_*v*B[idx]);
        }
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
          //
          //  a (x' - i)(mod n_x)
          //  b (y' - j)(mod n_y)
          //  c (t' - k)(mod n_t)

  //Array<ArrayXXd, Dynamic, 1> nP = P;
  //ArrayXXd dP = Eigen::ArrayXXd(n_x_, n_y_);

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
    for (int a=-3; a<=3;++a) {
      for (int b=-3; b<=3;++b) {
        for (int c=-3; c<=3;++c) {
          int i = (a + x + n_x_) % n_x_;
          int j = (b + y + n_y_) % n_y_;
          int k = (c + t + n_t_) % n_t_;
          // The function from the IJRR 2009 paper
          // e_{a,b,c} =   exp(-(a^2+b^2)/k_p_e) * exp(-c^2/k_d_e))
          //             - exp(-(a^2+b^2)/k_p_i) * exp(-c^2/k_d_i))

          // Correct Mexican hat function
          double e = exp(-(a*a+b*b)/k_p_e_)/sqrt(k_p_e_)*exp(-(c*c)/k_d_e_)/sqrt(k_d_e_)
                    -exp(-(a*a+b*b)/k_p_i_)/sqrt(k_p_i_)*exp(-(c*c)/k_d_i_)/sqrt(k_d_i_);
          //nP(k)(i,j) += p * e;
          //np += p * e;
          nP.push_back(std::pair<CellKey, double>(CellKey(k,i,j),p*e));
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

class NeuroSlam
{
public:
  NeuroSlam();
  void addOdometry(int64_t utime, const isam::Pose3d & delta);
  bool setView(const View & view) {
    boost::mutex::scoped_lock lock(mutex_);
    return vc_.setView(view);
  }

  void update();
private:
  ViewCells vc_;
  //PoseCells pc_;
  PoseCellsSparse pc_;
  ExperienceMap em_;

  int64_t last_update_;
  int64_t last_utime_;
  isam::Pose3d delta_;
  isam::Pose3d pose_;
  isam::Pose3d first_cell_pose_;

  // We need to synchronize access to the view and pose cells/
  boost::mutex mutex_;

  Viewer viewer_;

  // Object collections
  ObjectCollection col_vo_;
  ObjectCollection col_ns_;
  ObjectCollection col_emap_;
  ObjectCollection col_emap_active_;

  // Link collections
  LinkCollection link_vo_;
  LinkCollection link_ns_;
  LinkCollection link_emap_;
};

NeuroSlam::NeuroSlam() : pc_(&vc_),
                         em_(&pc_, &vc_),
                         last_update_(0.0),
                         last_utime_(0.0),
                         viewer_(g_lcm),
                         col_vo_(1, "Odometry", MRLCM_OBJ_COLLECTION_T_POSE),
                         col_ns_(2, "Pose cell", MRLCM_OBJ_COLLECTION_T_POSE),
                         col_emap_(5, "Experience Map", MRLCM_OBJ_COLLECTION_T_POSE),
                         col_emap_active_(6, "Experience Map - Active", MRLCM_OBJ_COLLECTION_T_SQUARE),
                         link_vo_(3, "Odometry - path"),
                         link_ns_(4, "Pose cells - path"),
                         link_emap_(7, "Experience Map - Links")
{
}

void NeuroSlam::addOdometry(int64_t utime, const isam::Pose3d & delta)
{
  boost::mutex::scoped_lock lock(mutex_);

  pc_.addOdometry(utime, delta);
  last_utime_ = utime;
  delta_ = delta_.oplus(delta);
  pose_ = pose_.oplus(delta);

  std::cout << "Delta: " << delta << std::endl;
}

void NeuroSlam::update()
{
  boost::mutex::scoped_lock lock(mutex_);

  pc_.update();
  if (last_update_ > 0.0)
  {
    Experience current_experience = em_.getExperience();
    em_.update(last_utime_, pose_);
    //for (int i=0; i<50; i++)  em_.relaxConstraints();
    em_.relaxConstraints();
    Experience new_experience = em_.getExperience();

    if (current_experience.id != new_experience.id)
    {
      const std::vector<Experience> & experiences = em_.getExperiences();
      for (size_t i=1; i<experiences.size();++i) {
        col_emap_.add(experiences[i].utime, experiences[i].pose);
  //      col_emap_.add(new_experience.utime, new_experience.pose); // @todo only add if new
      }

      col_emap_active_.add(new_experience.utime, new_experience.pose);

      viewer_.sendCollection(col_emap_,false);
      viewer_.sendCollection(col_emap_active_,true);
      col_emap_.clear();
      col_emap_active_.clear();

      if (current_experience.id != -1) {
        link_emap_.add(last_utime_, 5, current_experience.utime, 5, new_experience.utime);
        viewer_.sendCollection(link_emap_,false);
        link_emap_.clear();
      }
    }
  }

  // Visualize pose cells
  const Array<ArrayXXd, Dynamic, 1> & P = pc_.poseNetwork();
  int width = P(0).rows();
  int height = P(0).cols();

  cv::Mat img(cv::Size(600,600), CV_32FC1);
  cv::Mat cells(cv::Size(width,height), CV_32FC1);

  for (int x=0; x<width; ++x) {
    for (int y=0; y<height; ++y) {
      cells.at<float>(x,y) = 0.0;
      for (int t=0; t<P.rows(); ++t) {
        cells.at<float>(x,y) += P(t)(x,y);
      }
      cells.at<float>(x,y) = pow(cells.at<float>(x,y),0.2);
    }
  }

  cv::resize(cells, img, cv::Size(600,600), 0, 0, cv::INTER_NEAREST);
  cv::imshow("PoseCells", img);
  //cv::imshow("PoseCells", cells);

  // Update trajectory
  if (last_update_ == 0.0) {
    first_cell_pose_ = pc_.getCellPose();
  }

  // Add nodes
  if (last_utime_ > 0.0) {
    std::cout << "PoseCell: " << (pc_.getCellPose()).ominus(first_cell_pose_) << std::endl;
    std::cout << "Odometry: " << pose_ << std::endl;

    col_vo_.add(last_utime_, pose_);
    col_ns_.add(last_utime_, (pc_.getCellPose()).ominus(first_cell_pose_));

    viewer_.sendCollection(col_vo_,false);
    viewer_.sendCollection(col_ns_,false);
  }

  // Add links
  if (last_update_ > 0.0) {
    link_vo_.add(last_utime_, 1, last_update_, 1, last_utime_);
    link_ns_.add(last_utime_, 2, last_update_, 2, last_utime_);
    viewer_.sendCollection(link_vo_,false);
    viewer_.sendCollection(link_ns_,false);
  }

  delta_ = isam::Pose3d(0.0,0.0,0.0,0.0,0.0,0.0);
  last_update_ = last_utime_;
}

class ROSAdapter
{
public:
  ROSAdapter(NeuroSlam* ns);
  void run();
private:
  void on_image(const sensor_msgs::ImageConstPtr& l_image,
                           const sensor_msgs::CameraInfoConstPtr& l_cam_info,
                           const sensor_msgs::ImageConstPtr& r_image,
                           const sensor_msgs::CameraInfoConstPtr& r_cam_info);
  void on_odometry(const nav_msgs::Odometry::ConstPtr& odom);

  ros::NodeHandle nh_;
  NeuroSlam* ns_;
  ros::Subscriber odom_sub_;
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter l_image_sub_, r_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub_, r_info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
  sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;

  int odo_count_;
  int view_count_;
  nav_msgs::Odometry last_odom_;
  isam::Pose3d last_pose_;
  int64_t last_utime_;

  int64_t last_view_utime_;
  View last_view_;

  boost::mutex mutex_;
};

ROSAdapter::ROSAdapter(NeuroSlam* ns) : ns_(ns),
                                        it_(nh_),
                                        sync_(3),
                                        odo_count_(0),
                                        view_count_(0),
                                        last_utime_(0)
{
  odom_sub_ = nh_.subscribe("odometry", 100, &ROSAdapter::on_odometry, this);

  l_image_sub_.subscribe(it_, "/wide_stereo/left/image_rect", 1);
  l_info_sub_.subscribe(nh_, "/wide_stereo/left/camera_info", 1);
  r_image_sub_.subscribe(it_, "/wide_stereo/right/image_rect", 1);
  r_info_sub_.subscribe(nh_, "/wide_stereo/right/camera_info", 1);

  sync_.connectInput(l_image_sub_, l_info_sub_, r_image_sub_, r_info_sub_);
  sync_.registerCallback( boost::bind(&ROSAdapter::on_image, this, _1, _2, _3, _4) );

  // @todo  notify the neuro slam
}

void ROSAdapter::on_image(const sensor_msgs::ImageConstPtr& l_image,
                         const sensor_msgs::CameraInfoConstPtr& l_cam_info,
                         const sensor_msgs::ImageConstPtr& r_image,
                         const sensor_msgs::CameraInfoConstPtr& r_cam_info)
{

  sensor_msgs::CvBridge bridge;
  try
  {
    cv::Mat left = bridge.imgMsgToCv(l_image, "mono8");
    cv::Mat right = bridge.imgMsgToCv(r_image, "mono8");

    cv::Mat tag(cv::Size(12,8), CV_8UC1);
    cv::Mat img(cv::Size(300,300), CV_8UC1);

    cv::resize(left, tag, cv::Size(12,8), 0, 0, cv::INTER_AREA);
    cv::resize(tag, img, cv::Size(300,300), 0, 0, cv::INTER_NEAREST);
    {
      boost::mutex::scoped_lock lock(mutex_);

      View v(12*8);
      for (int i=0; i<(12*8); ++i) v(i) = tag.at<unsigned char>(i);
      last_view_utime_ = time_to_utime(l_image->header.stamp);
      last_view_ = v;
      view_count_ = 1;
    }

    cv::imshow("tag", img);
    cv::imshow("view left", left);
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", l_image->encoding.c_str());
  }
}

void ROSAdapter::on_odometry(const nav_msgs::Odometry::ConstPtr& odom)
{
  boost::mutex::scoped_lock lock(mutex_);
  last_odom_ = *odom;
  odo_count_ = 1;
}

void ROSAdapter::run()
{
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    View view(12*8);
    int64 view_utime = 0;
    int64 utime = 0;
    isam::Pose3d pose;

    int odo_count;
    int view_count;

    {
      // Read shared variables
      boost::mutex::scoped_lock lock(mutex_);
      view_count = view_count_;
      odo_count = odo_count_;
      if (view_count>0 && odo_count>0)
      {
        view = last_view_;
        view_utime = last_view_utime_;
        odometry_to_pose3d(boost::make_shared<nav_msgs::Odometry>(last_odom_), pose);
        utime = time_to_utime(last_odom_.header.stamp);
        view_count_ = 0;
        odo_count_ = 0;
      }
    }

    if (view_count>0 && odo_count>0)
    {
      if (last_utime_ == 0) {
        last_utime_ = utime;
        last_pose_ = pose;
      }

      std::cout << "RUN: time diff " << (view_utime - utime)*1E-6 << std::endl;
      isam::Pose3d delta = pose.ominus(last_pose_);
      ns_->setView(view);
      ns_->addOdometry(utime, delta);
      ns_->update();

      last_pose_ = pose;
      last_utime_ = utime;
    }

    loop_rate.sleep();
  }
}

/*
 * Visualization
 *
 * Show the posecell network in 2d
 *          -- " --          in 3d
 *
 * Show active view cells
 * Show view cell connections for active view cells
 *
 * Show the experience map
 * Show trajectory of max activated pose cell
 *
 */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "neuroslam");
  g_lcm = lcm_create(NULL);

  NeuroSlam neuro_slam;
  ROSAdapter rosadapter(&neuro_slam);
  cvNamedWindow("PoseCells");
  cvNamedWindow("tag");
  cvNamedWindow("view left");
  cvStartWindowThread();

  boost::thread neuroThead(&ROSAdapter::run, &rosadapter);

  ros::spin();

}

