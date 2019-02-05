#ifndef __DARRT_ENVIRONMENT_INTERFACE_H__
#define __DARRT_ENVIRONMENT_INTERFACE_H__

#include <planning_environment/models/collision_models.h>
#include <planning_environment/models/model_utils.h>
#include <pr2_collision_checker/pr2_collision_space.h>

namespace pm = planning_models;
namespace pe=planning_environment;
namespace an=arm_navigation_msgs;

namespace darrt {
  class EnvironmentInterface : public pe::CollisionModels {
  public:    
    EnvironmentInterface(const std::string &description);
    EnvironmentInterface(const std::string &description, const pm::KinematicState &kinematic_state);
    void setKinematicState(const pm::KinematicState &kinematic_state)
    {if (kinematic_state_) {delete kinematic_state_;} 
      kinematic_state_ = new pm::KinematicState(kinematic_state);}
    void setKinematicState(pm::KinematicState *kinematic_state)
    {kinematic_state_ = kinematic_state;}
    const pm::KinematicState *kinematicState() const {return kinematic_state_;}
    pm::KinematicState *kinematicState() {return kinematic_state_;}
    ~EnvironmentInterface();// {if (kinematic_state_) {delete kinematic_state_;}}

    void updateApproximateEnvironment();
    void robotPose(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &pose) const;
    void removeObjectFromApproximateEnvironment(const an::CollisionObject &obj);

    pr2_collision_checker::PR2CollisionSpace *approximateEnvironment() const {return approximate_env_;}
    const std::vector<std::string> &right_arm_joints() const {return rarm_names_;}
    const std::vector<std::string> &left_arm_joints() const {return larm_names_;}
    std::string torso_joint() const {return "torso_lift_joint";}
    visualization_msgs::Marker getGridMarker();


  protected:
    pm::KinematicState *kinematic_state_;
    pr2_collision_checker::PR2CollisionSpace *approximate_env_;
    sbpl_arm_planner::OccupancyGrid *grid_;
    sbpl_arm_planner::SBPLArmModel *rarm_, *larm_;
    std::vector<std::string> rarm_names_, larm_names_;

    void initApproxEnvironment();
    
  };
}

#endif //environment_interface.hh
