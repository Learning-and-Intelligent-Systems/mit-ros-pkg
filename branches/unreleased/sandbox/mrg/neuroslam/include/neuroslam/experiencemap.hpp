/*
 * experiencemap.hpp
 *
 *  Created on: Feb 23, 2011
 *      Author: hordurj
 */

#ifndef EXPERIENCEMAP_HPP_
#define EXPERIENCEMAP_HPP_

#include <isam/isam.h>

#include "neuroslam/posecells.hpp"
#include "neuroslam/viewcells.hpp"

namespace neuroslam
{
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

}

#endif /* EXPERIENCEMAP_HPP_ */
