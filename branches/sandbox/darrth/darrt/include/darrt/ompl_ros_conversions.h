/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \author Sachin Chitta, Ioan Sucan */

#ifndef OMPL_ROS_CONVERSIONS_
#define OMPL_ROS_CONVERSIONS_

#include <ros/ros.h>
#include <ros/console.h>
#include <angles/angles.h>

#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/State.h>

#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <arm_navigation_msgs/convert_messages.h>
#include <arm_navigation_msgs/RobotState.h>
#include <arm_navigation_msgs/RobotTrajectory.h>
#include <arm_navigation_msgs/JointConstraint.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose2D.h>

namespace ompl_ros_conversions {

/**
   @brief enumeration of different mapping types for ompl
*/
typedef enum{REAL_VECTOR, SO2, SO3, SE2, SE3, COMPOUND, UNKNOWN}MAPPING_TYPE;

/**
 * @class This class contains a mapping from an ompl::base::CompoundState object 
 * to a arm_navigation_msgs::RobotState object
 */
class OmplStateToRobotStateMapping
{
public:
  OmplStateToRobotStateMapping():real_vector_index(-1){}
  std::vector<int> ompl_state_mapping;
  int real_vector_index;
  std::vector<int> real_vector_mapping;
  std::vector<ompl_ros_conversions::MAPPING_TYPE> mapping_type;
  unsigned int num_joints;
};

/**
 * @class This class contains a mapping from an arm_navigation_msgs::RobotState object 
 * to a ompl::base::CompoundState object
 */
class RobotStateToOmplStateMapping
{
public:
  RobotStateToOmplStateMapping():real_vector_index(-1){}
  std::vector<int> multi_dof_mapping;
  int real_vector_index;
  std::vector<int> joint_state_mapping;
  std::vector<ompl_ros_conversions::MAPPING_TYPE> joint_mapping_type;
  std::vector<ompl_ros_conversions::MAPPING_TYPE> multi_dof_joint_mapping_type;
};

/**
 * @class This class contains a mapping from a kinematic_state object
 * to an ompl::base::CompoundState object
 */
class KinematicStateToOmplStateMapping
{
public:
  KinematicStateToOmplStateMapping():real_vector_index(-1){}
  int real_vector_index;
  std::vector<unsigned int> joint_state_mapping;
  std::vector<ompl_ros_conversions::MAPPING_TYPE> joint_mapping_type;
};

/**
 * @class This class contains a mapping from a kinematic_state object
 * to an ompl::base::CompoundState object
 */
class OmplStateToKinematicStateMapping
{
public:
  OmplStateToKinematicStateMapping():real_vector_index(-1){}
  int real_vector_index;
  std::vector<unsigned int> ompl_state_mapping;
  std::vector<unsigned int> real_vector_mapping;
  std::vector<ompl_ros_conversions::MAPPING_TYPE> mapping_type;
};

/**
 * @class This class contains a mapping from a kinematic_state object
 * to an ompl::base::CompoundState object
 */
class RobotStateToKinematicStateMapping
{
public:
  std::vector<int> multi_dof_mapping;
  std::vector<int> joint_state_mapping;
};


ompl_ros_conversions::MAPPING_TYPE get_mapping_type
(const ompl::base::StateSpace *state_space);

/**
 * @brief fill in a object of type ompl::base::CompoundStateSpace from an object 
 * of type planning_models::KinematicModel::JointGroup
 * @param joint_group The joint group to construct this from
 * @param state_space The output state space
 * @param ompl_kinematic_mapping Mapping from the ompl state to the 
 *corresponding kinematic state
 * @param kinematic_ompl_mapping Mapping from the kinematic state to the 
 *corresponding ompl state
 */
bool joint_group_to_ompl_state_space
(const planning_models::KinematicModel::JointModelGroup *joint_group, 
 ompl::base::CompoundStateSpace *state_space,
 ompl_ros_conversions::OmplStateToKinematicStateMapping &ompl_kinematic_mapping,
 ompl_ros_conversions::KinematicStateToOmplStateMapping &kinematic_ompl_mapping);

/**
 * @brief Create a mapping from an ompl state to the ROS message robot_state.  
 * This function is intended to help in efficient conversion between ROS messages 
 * and OMPL state objects by providing a mapping that can be cached and used 
 * frequently.
 * @param state_space The ompl state space to create the mapping from
 * @param robot_state The joint state message to create the mapping to
 * @param mapping The resultant mapping
 */
bool get_ompl_state_to_robot_state_mapping
(const ompl::base::CompoundStateSpace *state_space,
 const arm_navigation_msgs::RobotState &robot_state, 
 ompl_ros_conversions::OmplStateToRobotStateMapping &mapping,
 bool fail_if_match_not_found = true);


/**
 * @brief Create a mapping from the ROS message robot_state to an ompl state. This function is 
 * intended to help in efficient conversion between ROS messages and an ompl state object by 
 * providing a mapping that can be cached and used frequently.
 * @param robot_state The robot state to create the mapping from
 * @param ompl_state The ompl state to create the mapping to
 * @param mapping The resultant mapping
 */
 bool get_robot_state_to_ompl_state_mapping
   (const arm_navigation_msgs::RobotState &robot_state, 
    const ompl::base::CompoundStateSpace *state_space,
    ompl_ros_conversions::RobotStateToOmplStateMapping &mapping,
    bool fail_if_match_not_found = true);

/**
 * @brief Create a mapping from the ROS message joint_state to an ompl state. This function is 
 * intended to help in efficient conversion between ROS messages and OMPL state objects by 
 * providing a mapping that can be cached and used frequently.
 * @param robot_state The joint state message to create the mapping from
 * @param ompl_state The ompl state to create the mapping to
 * @param mapping The resultant mapping
 */
bool get_joint_state_to_ompl_state_mapping
  (const sensor_msgs::JointState &joint_state, 
   const ompl::base::CompoundStateSpace *state_space,
   ompl_ros_conversions::RobotStateToOmplStateMapping &mapping,
   bool fail_if_match_not_found = true);

bool ompl_state_to_joint_positions(const ompl::base::State *state,
				   const OmplStateToRobotStateMapping &mapping,
				   const std::vector<std::string> &link_names,
				   std::vector<double> &positions);
bool joint_positions_to_ompl_state(const std::vector<double> &positions,
				   const RobotStateToOmplStateMapping &mapping,
				   const std::vector<std::string> link_names,
				   ompl::base::State *state);

bool kinematic_state_group_to_ompl_state
  (const planning_models::KinematicState::JointStateGroup* joint_state_group, 
   const ompl_ros_conversions::KinematicStateToOmplStateMapping &mapping,
   ompl::base::State *ompl_state);

bool ompl_state_to_kinematic_state_group
(const ompl::base::State *ompl_state,
 const ompl_ros_conversions::OmplStateToKinematicStateMapping &mapping,
 planning_models::KinematicState::JointStateGroup* joint_state_group);

bool ompl_state_to_robot_state
(const ompl::base::State *ompl_state,
 const ompl_ros_conversions::OmplStateToRobotStateMapping &mapping,
 arm_navigation_msgs::RobotState &robot_state);

bool robot_state_to_ompl_state
(const arm_navigation_msgs::RobotState &robot_state,
 const ompl_ros_conversions::RobotStateToOmplStateMapping &mapping,
 ompl::base::State *ompl_state,
 const bool &fail_if_match_not_found = true);

bool joint_state_to_real_vector_state
(const sensor_msgs::JointState &joint_state,
 const ompl_ros_conversions::RobotStateToOmplStateMapping &mapping,
 ompl::base::RealVectorStateSpace::StateType &real_vector_state,
 const bool &fail_if_match_not_found);

bool ompl_real_vector_state_to_joint_state
(const ompl::base::RealVectorStateSpace::StateType &ompl_state,
 const ompl_ros_conversions::OmplStateToRobotStateMapping &mapping,
 sensor_msgs::JointState &joint_state);

bool joint_model_group_to_robot_trajectory
(const planning_models::KinematicModel::JointModelGroup *joint_model_group, 
 arm_navigation_msgs::RobotTrajectory &robot_trajectory);

bool ompl_path_geometric_to_robot_trajectory
(const ompl::geometric::PathGeometric &path,
 const ompl::base::StateSpacePtr &state_space,
 const ompl_ros_conversions::OmplStateToRobotStateMapping &mapping,
 arm_navigation_msgs::RobotTrajectory &robot_trajectory,
 const planning_models::KinematicModel::JointModelGroup *joint_model_group=NULL);

void SE3StateSpace_to_pose_msg(const ompl::base::SE3StateSpace::StateType &pose,
			       geometry_msgs::Pose &pose_msg);
void SO3StateSpace_to_quaternion_msg
(const ompl::base::SO3StateSpace::StateType &quaternion,
 geometry_msgs::Quaternion &quaternion_msg);

void pose_msg_to_SE3StateSpace(const geometry_msgs::Pose &pose_msg,
			       ompl::base::SE3StateSpace::StateType &pose);

void quaternion_msg_to_SO3StateSpace
(const geometry_msgs::Quaternion &quaternion_msg,
 ompl::base::SO3StateSpace::StateType &quaternion);

void SE2StateSpace_to_pose_msg(const ompl::base::SE2StateSpace::StateType &pose,
			       geometry_msgs::Pose &pose_msg);

void pose_msg_to_SE2StateSpace(const geometry_msgs::Pose &pose_msg,
			       ompl::base::SE2StateSpace::StateType &pose);
void pose2D_msg_to_SE2StateSpace(const geometry_msgs::Pose2D &pose_msg,
				 ompl::base::SE2StateSpace::StateType &pose);

void SE2StateSpace_to_pose2D_msg(const ompl::base::SE2StateSpace::StateType &pose,
				 geometry_msgs::Pose2D &pose_msg);


//all of these fill in the extra values with random numbers
//used for sampling from goal shapes
bool convert_point_to_state(ompl::base::State *state,
			    const ompl::base::StateSpace *space,
			    const geometry_msgs::Point &pt);

bool convert_point_to_state(ompl::base::RealVectorStateSpace::StateType *state,
			    const ompl::base::RealVectorStateSpace *space,
			    const geometry_msgs::Point &pt);

bool convert_point_to_state(ompl::base::SE2StateSpace::StateType *state,
			    const ompl::base::SE2StateSpace *space,
			    const geometry_msgs::Point &pt);

bool convert_point_to_state(const ompl::base::SE3StateSpace *state,
			    const ompl::base::SE3StateSpace *space,
			    geometry_msgs::Point &pt);

bool convert_state_to_point(const ompl::base::State *state,
			    const ompl::base::StateSpace *space,
			    geometry_msgs::Point &pt);

bool convert_state_to_point(const ompl::base::RealVectorStateSpace::StateType 
			    *state,
			    const ompl::base::RealVectorStateSpace *space,
			    geometry_msgs::Point &pt);

bool convert_state_to_point(const ompl::base::SE2StateSpace::StateType *state,
			    const ompl::base::SE2StateSpace *space,
			    geometry_msgs::Point &pt);

bool convert_state_to_point(const ompl::base::SE3StateSpace *state,
			    const ompl::base::SE3StateSpace *space,
			    geometry_msgs::Point &pt);

}
#endif
