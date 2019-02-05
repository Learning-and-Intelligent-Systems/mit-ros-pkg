/*
 * posecells.hpp
 *
 *  Created on: Feb 23, 2011
 *      Author: hordurj
 */

#ifndef POSECELLS_HPP_
#define POSECELLS_HPP_

#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_comparison.hpp>
#include <boost/unordered_map.hpp>
#include <boost/thread.hpp>
#include <boost/functional/hash.hpp>
#include <Eigen/Dense>
#include <isam/isam.h>

#include "neuroslam/viewcells.hpp"

namespace neuroslam
{
  //typedef boost::tuple<int,int,int,int> ViewPose;
  struct ViewPose
  {
    ViewPose(int i, int x, int y, int t) : i(i), x(x), y(y), t(t) {}
    int i;
    int x;
    int y;
    int t;
  };
  bool operator< (const ViewPose & a, const ViewPose & b);

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

    const Eigen::Array<Eigen::ArrayXXd, Eigen::Dynamic, 1> & poseNetwork() const {return P_array;}

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

    // Excitation variance for position and direction.
    double k_p_e_;
    double k_d_e_;

    // Inhibition variance for position and direction.
    double k_p_i_;
    double k_d_i_;

    // The global inhibition constant.
    double theta_;

    Eigen::Array<Eigen::ArrayXXd, Eigen::Dynamic, 1> P_array;
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

    static const int e_width_ = 3;
    inline double & e_abc(int a, int b, int c) {return e_abc_[a+e_width_][b+e_width_][c+e_width_];}
    double e_abc_[2*e_width_ + 1][2*e_width_ + 1][2*e_width_ + 1];
    /// Weights between view cells and pose cells
    std::map<int, std::map<CellKey,double> > B;

    void updateViewCells();
    void updatePathIntegration();
    void updateCAN();

    void printStats();
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };


  class PoseCells
  {
  public:
    PoseCells(ViewCells* vc);

    void addOdometry(int64_t utime, const isam::Pose3d & delta);
    void update();

    const Eigen::Array<Eigen::ArrayXXd, Eigen::Dynamic, 1> & poseNetwork() const {return P;}

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

    Eigen::Array<Eigen::ArrayXXd, Eigen::Dynamic, 1> P;

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
}

#endif /* POSECELLS_HPP_ */
