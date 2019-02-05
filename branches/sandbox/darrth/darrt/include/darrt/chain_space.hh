#ifndef __DARRT_CHAIN_SPACE_HH__
#define __DARRT_CHAIN_SPACE_HH__

#include "darrt/collision_aware_types.hh"
#include "darrt/types.hh"
#include "darrt/ompl_ros_conversions.h"
#include "darrt/robot_space.hh"

#include <kinematics_base/kinematics_base.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <planning_environment/util/kinematic_state_constraint_evaluator.h>
#include <pluginlib/class_loader.h>


namespace orc=ompl_ros_conversions;

namespace darrt {

  class RobotChainStateSpace : public RobotStateSpace, public Displayable {
  public:
    RobotChainStateSpace(const ros::NodeHandle &node_handle,
			 const std::string &group_name,
			 EnvironmentInterface *env);
    void setup();
    bool init();
    double distance(const ob::State *state1, const ob::State *state2) const;
    double cartesian_distance(const ob::State *state1, const ob::State *state2) const;
    double joint_space_distance(const ob::State *state1, const ob::State *state2) const;

    std::string state_string(const ob::State *state) const
    {std::stringstream ss; printState(state, ss); return ss.str();}
    void printState(const ob::State *state,
		    std::ostream &out = std::cout) const;
    visualization_msgs::MarkerArray displayable_state
    (const ob::State *state, std::string ns, int id, double scale=0.05,
     ColorPalate p=PRIMARYPALATE, double alpha=0.5, bool minimal=false) const;

    void set_joint_positions(const std::vector<double> &positions, ob::State *state) const;
    
    bool forward_kinematics(const ob::State *state, 
			    geometry_msgs::PoseStamped &pose) const;
    bool inverse_kinematics(const geometry_msgs::PoseStamped &pose,
			    std::vector<double> &positions,
			    const std::vector<double> &seed_positions_in) const;
    bool inverse_kinematics(const geometry_msgs::PoseStamped &pose,
			    std::map<std::string, double> &pos_map) const;
    bool inverse_kinematics(const geometry_msgs::PoseStamped &pose, ob::State *state) 
      const;
    bool inverse_kinematics(const geometry_msgs::PoseStamped &pose,
			    ob::State *state, const ob::State *seed_state) const;
    bool near_states(const ob::State *s1, const ob::State *s2,
		     double deps=DIST_EPS, double aeps=ANGLE_EPS) const;

    double between(const ob::State *state, const ob::State *source,
		   const ob::State *destination,
		   double eps=DIST_EPS, double aeps=ANGLE_EPS) const;

    bool initialized() {return initialized_;}

    void convert_ompl_to_robot_state(const ob::State *ompl_state, 
				     an::RobotState &robot_state) const;

    void convert_robot_to_ompl_state(const an::RobotState &robot_state,
				     ob::State *ompl_state) const;
    
    void convert_ompl_to_kinematic_state(const ob::State *ompl_state, 
					 pm::KinematicState &kstate) const;

    void convert_kinematic_to_ompl_state(const pm::KinematicState &kstate,
					 ob::State *ompl_state) const;

    const ompl_ros_conversions::OmplStateToKinematicStateMapping 
    &ompl_to_kinematic_mapping() const {return ompl_to_kinematic_mapping_;}
    
    const ompl_ros_conversions::KinematicStateToOmplStateMapping
    &kinematic_to_ompl_mapping() const {return kinematic_to_ompl_mapping_;}

    const ompl_ros_conversions::OmplStateToRobotStateMapping
    &ompl_to_robot_mapping() const {return ompl_to_robot_mapping_;}

    const ompl_ros_conversions::RobotStateToOmplStateMapping
    &robot_to_ompl_mapping() const {return robot_to_ompl_mapping_;}


    const ompl_ros_conversions::OmplStateToKinematicStateMapping 
    *ompl_to_kinematic_mapping_ptr() const {return &ompl_to_kinematic_mapping_;}

    const planning_models::KinematicModel::JointModelGroup
    *physical_joint_group() {return physical_joint_group_;}
    std::string physical_group_name() const 
    {return physical_joint_group_->getName();}
    std::vector<std::string> joint_names() const 
    {return kinematics_solver_->getJointNames();}
    const EnvironmentInterface *environment_interface() const 
    {return environment_interface_;}

    bool restrict_to_group(const an::RobotState &full_state,
			   an::RobotState &group_state) const;


    bool update_model(const ob::State *new_state, EnvironmentInterface *env) const;

    std::string root_name() const {return root_name_;}
    std::string end_effector_name() const {return end_effector_name_;}
    
  protected:
    ros::NodeHandle node_;
    ros::Publisher viz_pub_;
    pluginlib::ClassLoader<kinematics::KinematicsBase> kinematics_loader_;
    std::string kinematics_solver_name_, root_name_, group_name_,
      end_effector_name_;
    kinematics::KinematicsBase *kinematics_solver_;
    EnvironmentInterface *environment_interface_;
    const planning_models::KinematicModel::JointModelGroup 
    *physical_joint_group_;
    bool initialized_;
    ompl_ros_conversions::OmplStateToRobotStateMapping ompl_to_robot_mapping_;
    ompl_ros_conversions::RobotStateToOmplStateMapping robot_to_ompl_mapping_;
    ompl_ros_conversions::OmplStateToKinematicStateMapping
    ompl_to_kinematic_mapping_;
    ompl_ros_conversions::KinematicStateToOmplStateMapping
    kinematic_to_ompl_mapping_;

    //find out if there is a way of getting this
    //without calling the ROS service.
    kinematics_msgs::KinematicSolverInfo kinematic_info_;

    bool initialize_kinematics();
    bool initialize_physical_group();
  };
}

#endif //chain_space.hh
