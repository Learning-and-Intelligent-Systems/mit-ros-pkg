#ifndef __DARRT_ROBOT_BASE_SPACE_HH__
#define __DARRT_ROBOT_BASE_SPACE_HH__

#include "darrt/collision_aware_types.hh"
#include "darrt/types.hh"
#include "darrt/ompl_ros_conversions.h"
#include "darrt/robot_space.hh"

#include <geometry_msgs/Pose2D.h>

namespace orc=ompl_ros_conversions;

namespace darrt {

  class RobotBaseStateSpace : public RobotStateSpace {
  public:
    typedef ob::SE2StateSpace::StateType BaseState;

    RobotBaseStateSpace(const ros::NodeHandle &node_handle,
			const std::string &group_name,
			EnvironmentInterface *env,
			const ob::RealVectorBounds &bounds);
    void setup();
    bool init();
    std::string state_string(const ob::State *state) const
    {std::stringstream ss; printState(state, ss); return ss.str();}
    void printState(const ob::State *state,
		    std::ostream &out = std::cout) const;
    
    bool near_states(const ob::State *state1, const ob::State *state2, 
		     double deps=DIST_EPS, double aeps=ANGLE_EPS) const;
    double between(const ob::State *state, const ob::State *source,
		   const ob::State *destination,
		   double eps=DIST_EPS, double aeps=ANGLE_EPS) const;

    double distance(const ob::State *state1, const ob::State *state2) const;

    bool initialized() {return initialized_;}

    void convert_ompl_to_robot_state(const ob::State *ompl_state, 
				     an::RobotState &robot_state) const;

    void convert_robot_to_ompl_state(const an::RobotState &robot_state,
				     ob::State *ompl_state) const;
    
    void convert_ompl_to_kinematic_state(const ob::State *ompl_state, 
					 pm::KinematicState &kstate) const;

    void convert_kinematic_to_ompl_state(const pm::KinematicState &kstate,
					 ob::State *ompl_state) const;

    std::string group_name() const {return group_name_;}
    std::string multi_dof_joint_name() const {return multi_dof_joint_name_;}

    gm::Pose2D get_pose(const ob::State *state) const;
    void set_pose(ob::State *state, const gm::Pose2D &pose) const;

    BaseState *base_state(ob::State *state) const;
    const BaseState *base_state(const ob::State *state) const;


    const EnvironmentInterface *environment_interface() const 
    {return environment_interface_;}

    bool update_model(const ob::State *new_state, EnvironmentInterface *env) const;
    
  protected:
    ros::NodeHandle node_;
    ros::Publisher viz_pub_;
    const planning_models::KinematicModel::JointModelGroup *physical_joint_group_;
    const pm::KinematicModel::FloatingJointModel *multi_dof_joint_;
    EnvironmentInterface *environment_interface_;
    bool initialized_;
    std::string group_name_, multi_dof_joint_name_;
    ob::RealVectorBounds bounds_;

    bool initialize_physical_group();
  };
}

#endif //robot_base_space.hh
