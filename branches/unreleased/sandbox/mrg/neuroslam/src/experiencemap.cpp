/*
 * experiencemap.cpp
 *
 *  Created on: Feb 23, 2011
 *      Author: hordurj
 */

#include <cmath>
#include <Eigen/Dense>
#include "neuroslam/experiencemap.hpp"

using namespace Eigen;

namespace neuroslam
{
  // #include <isam/robust.h>
  inline double cost_pseudo_huber(double d, double b) {
    double b2 = b*b;
    return 2*b2*(sqrt(1+d*d/b2) - 1);
  }
  double huber(double a) { return cost_pseudo_huber(a,0.5); }


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
}
