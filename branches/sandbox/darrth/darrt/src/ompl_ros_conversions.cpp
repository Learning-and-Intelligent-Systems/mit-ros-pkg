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

#include "darrt/ompl_ros_conversions.h"
#include "darrt/utils.hh"

namespace ompl_ros_conversions {

bool joint_group_to_ompl_state_space
(const planning_models::KinematicModel::JointModelGroup *joint_group, 
 ompl::base::CompoundStateSpace *state_space,
 ompl_ros_conversions::OmplStateToKinematicStateMapping 
 &ompl_kinematic_mapping,
 ompl_ros_conversions::KinematicStateToOmplStateMapping 
 &kinematic_ompl_mapping) {
  // initially assume dimension of R(K) subspace is 0.
  ompl::base::RealVectorBounds real_vector_bounds(0); 
  ompl::base::RealVectorStateSpace real_vector_joints(0);
  
  std::vector<std::string> real_vector_names;
  std::vector<const planning_models::KinematicModel::JointModel*> 
    joint_models = joint_group->getJointModels();
  
  for (unsigned int i = 0 ; i < joint_models.size() ; ++i) {
    const planning_models::KinematicModel::RevoluteJointModel
      *revolute_joint = dynamic_cast
      <const planning_models::KinematicModel::RevoluteJointModel*>
      (joint_models[i]);
    if (revolute_joint && revolute_joint->continuous_) {
      ompl::base::SO2StateSpace *subspace = new ompl::base::SO2StateSpace();
      subspace->setName(revolute_joint->getName());
      state_space->addSubspace(ompl::base::StateSpacePtr(subspace), 1.0);
      
      kinematic_ompl_mapping.joint_state_mapping.push_back
	(state_space->getSubspaceCount()-1);
      kinematic_ompl_mapping.joint_mapping_type.push_back
	(ompl_ros_conversions::SO2);
      
      ompl_kinematic_mapping.ompl_state_mapping.push_back(i);
      ompl_kinematic_mapping.mapping_type.push_back
	(ompl_ros_conversions::SO2);
      ROS_DEBUG("Adding SO2 state space with name %s",
		revolute_joint->getName().c_str());
    } else {
      const planning_models::KinematicModel::PlanarJointModel* planar_joint = 
	dynamic_cast<const planning_models::KinematicModel::PlanarJointModel*>
	(joint_models[i]);
      if (planar_joint) {
	ompl::base::SE2StateSpace *subspace = new ompl::base::SE2StateSpace();
	subspace->setName(planar_joint->getName());
	state_space->addSubspace(ompl::base::StateSpacePtr(subspace), 1.0);
	
	kinematic_ompl_mapping.joint_state_mapping.push_back
	  (state_space->getSubspaceCount()-1);
	kinematic_ompl_mapping.joint_mapping_type.push_back
	  (ompl_ros_conversions::SE2);
	
	ompl_kinematic_mapping.ompl_state_mapping.push_back(i);
	ompl_kinematic_mapping.mapping_type.push_back
	  (ompl_ros_conversions::SE2);
	ROS_DEBUG("Adding SE2 state space with name %s",
		  planar_joint->getName().c_str());
      } else {
	//actually for any real robot, I think we want this to be
	//an SE2 or SE2PROJECTION (not in this version)
	//unless the robot can go up stairs or something
	const planning_models::KinematicModel::FloatingJointModel
	  *floating_joint = 
	  dynamic_cast
	  <const planning_models::KinematicModel::FloatingJointModel*>
	  (joint_models[i]);
	if (floating_joint) {
	  ompl::base::SE3StateSpace *subspace = 
	    new ompl::base::SE3StateSpace();
	  subspace->setName(floating_joint->getName());
	  state_space->addSubspace(ompl::base::StateSpacePtr(subspace), 1.0);
	  
	  kinematic_ompl_mapping.joint_state_mapping.push_back
	    (state_space->getSubspaceCount()-1);
	  kinematic_ompl_mapping.joint_mapping_type.push_back
	    (ompl_ros_conversions::SE3);
	  
	  ompl_kinematic_mapping.ompl_state_mapping.push_back(i);
	  ompl_kinematic_mapping.mapping_type.push_back
	    (ompl_ros_conversions::SE3);
	  ROS_DEBUG("Adding SE3 state space with name %s",
		    revolute_joint->getName().c_str());
	} else {
	  // the only other case we consider is R^n; 
	  //since we know that for now at least the only other type of joint 
	  //available is a single-dof non-continuous revolute 
	  //or prismatic joint
	  std::pair<double,double> bounds;
	  joint_models[i]->getVariableBounds(joint_models[i]->getName(), 
					     bounds);
	  real_vector_bounds.low.push_back(bounds.first);
	  real_vector_bounds.high.push_back(bounds.second);
	  real_vector_names.push_back(joint_models[i]->getName());
	  kinematic_ompl_mapping.joint_state_mapping.push_back
	    (real_vector_bounds.low.size()-1);
	  kinematic_ompl_mapping.joint_mapping_type.push_back
	    (ompl_ros_conversions::REAL_VECTOR);
	  
	  ompl_kinematic_mapping.real_vector_mapping.push_back(i);
	  ROS_DEBUG("Adding real vector joint %s with bounds %f %f",
		    joint_models[i]->getName().c_str(),
		    real_vector_bounds.low.back(),
		    real_vector_bounds.high.back());
	}
      }
    }
  }
  
  // we added everything now, except the R(N) state space. 
  //we now add R(N) as well, if needed
  if (real_vector_bounds.low.size() > 0) {
    ompl::base::RealVectorStateSpace *subspace = 
      new ompl::base::RealVectorStateSpace(real_vector_bounds.low.size());
    subspace->setName("real_vector");
    subspace->setBounds(real_vector_bounds);
    for(unsigned int i=0; i < real_vector_names.size(); i++)
      subspace->setDimensionName(i,real_vector_names[i]);
    state_space->addSubspace(ompl::base::StateSpacePtr(subspace),1.0);
    kinematic_ompl_mapping.real_vector_index = 
      state_space->getSubspaceCount()-1;
    ompl_kinematic_mapping.real_vector_index = 
	kinematic_ompl_mapping.real_vector_index;
    ompl_kinematic_mapping.ompl_state_mapping.push_back(-1);
    ompl_kinematic_mapping.mapping_type.push_back
      (ompl_ros_conversions::REAL_VECTOR);
  }
  for(unsigned int i=0; i < state_space->getSubspaceCount(); i++) {
    ROS_DEBUG("State state space: subspace %d has name %s",i,
	      state_space->getSubspace(i)->getName().c_str());
  }
  return true;
}


ompl_ros_conversions::MAPPING_TYPE get_mapping_type
(const ompl::base::StateSpace *state_space) {
  const ompl::base::SO2StateSpace* SO2_state_space = 
    dynamic_cast<const ompl::base::SO2StateSpace*>(state_space);
  if(SO2_state_space)
    return ompl_ros_conversions::SO2;
  
  const ompl::base::SE2StateSpace* SE2_state_space = 
    dynamic_cast<const ompl::base::SE2StateSpace*>(state_space);
  if(SE2_state_space)
    return ompl_ros_conversions::SE2;
  
  const ompl::base::SO3StateSpace* SO3_state_space = 
    dynamic_cast<const ompl::base::SO3StateSpace*>(state_space);
  if(SO3_state_space)
    return ompl_ros_conversions::SO3;
  
  const ompl::base::SE3StateSpace* SE3_state_space = 
    dynamic_cast<const ompl::base::SE3StateSpace*>(state_space);
  if(SE3_state_space)
    return ompl_ros_conversions::SE3;
  
  const ompl::base::RealVectorStateSpace* real_vector_state_space = 
    dynamic_cast<const ompl::base::RealVectorStateSpace*>(state_space);
  if(real_vector_state_space)
    return ompl_ros_conversions::REAL_VECTOR;
  
  return ompl_ros_conversions::UNKNOWN;
}
  
bool get_ompl_state_to_robot_state_mapping
(const ompl::base::CompoundStateSpace *state_space,
 const arm_navigation_msgs::RobotState &robot_state,
 OmplStateToRobotStateMapping &mapping,
 bool fail_if_match_not_found) {
  
  unsigned int num_state_spaces = state_space->getSubspaceCount();
  mapping.ompl_state_mapping.resize(num_state_spaces,-1);
  mapping.mapping_type.resize(num_state_spaces,ompl_ros_conversions::UNKNOWN);
  for(unsigned int i=0; i < num_state_spaces; i++) {
    
    if(dynamic_cast<ompl::base::RealVectorStateSpace*>
       (state_space->getSubspace(i).get())) {
      //each of the dimensions of this needs to match its own joint
      mapping.real_vector_index = i;
      mapping.mapping_type[i] = ompl_ros_conversions::REAL_VECTOR;
      const ompl::base::RealVectorStateSpace *real_vector_state_space = 
	state_space->getSubspace(mapping.real_vector_index).get()
	->as<ompl::base::RealVectorStateSpace>();
      mapping.real_vector_mapping.resize
	(real_vector_state_space->getDimension(),-1);
      
      for(unsigned int k=0; k<real_vector_state_space->getDimension(); k++) {
	bool match_found = false;
	for (unsigned int j=0; j<robot_state.joint_state.name.size();j++) {
	  if(real_vector_state_space->getDimensionName(k) == 
	     robot_state.joint_state.name[j]) {
	    ROS_INFO
	      ("Mapping for joint state %s index %u is real vector index %u",
	       robot_state.joint_state.name[j].c_str(), j, k);
	    mapping.real_vector_mapping[k] = j;
	    match_found = true;
	    break;
	  } 
	}
	if (!match_found) {
	  ROS_ERROR("Unable to find mapping for RV ompl state %s dim %u",
		    real_vector_state_space->getDimensionName(k).c_str(), k);
	  return false;
	}
      }
      continue;
    }

    bool ompl_state_mapping_found = false;
    //does it match joints?
    for(unsigned int j=0; j < robot_state.joint_state.name.size(); j++) {
      if(state_space->getSubspace(i)->getName() == 
	 robot_state.joint_state.name[j]) {
	mapping.ompl_state_mapping[i] = j;
	ompl_state_mapping_found = true;
	mapping.mapping_type[i] = get_mapping_type
	  (state_space->getSubspace(i).get());
	ROS_INFO("Mapping for state space %u is joint %s, index %u",
		  i, robot_state.joint_state.name[j].c_str(), j);
	break;
      }
    }
    
    if (ompl_state_mapping_found) {
      continue;
    }
    
    //does it match multi DOF joints?
    for(unsigned int j=0; j < 
	  robot_state.multi_dof_joint_state.joint_names.size(); j++) {
      if(state_space->getSubspace(i)->getName() == 
	 robot_state.multi_dof_joint_state.joint_names[j]) {
	mapping.ompl_state_mapping[i] = j;
	mapping.mapping_type[i] = 
	  get_mapping_type(state_space->getSubspace(i).get());
	ompl_state_mapping_found = true;
	break;
      } 
    }
   
    //if we could not match the ompl state, fail if requested
    if (!ompl_state_mapping_found && fail_if_match_not_found) {
      ROS_ERROR("Could not find matching joint for subspace %s", 
		state_space->getSubspace(i)->getName().c_str());
      return false;
    }
  }
  ROS_INFO("Mapping displayed above");
  //darrt::pause("Look at mapping!", 0);
  return true;
}
  

bool get_robot_state_to_ompl_state_mapping
(const arm_navigation_msgs::RobotState &robot_state, 
 const ompl::base::CompoundStateSpace *state_space,
 ompl_ros_conversions::RobotStateToOmplStateMapping &mapping,
 bool fail_if_match_not_found) {
  unsigned int num_state_spaces = state_space->getSubspaceCount();
  mapping.multi_dof_mapping.resize
    (robot_state.multi_dof_joint_state.joint_names.size(),-1);
  mapping.multi_dof_joint_mapping_type.resize
    (robot_state.multi_dof_joint_state.joint_names.size(),
     ompl_ros_conversions::UNKNOWN);
  for(unsigned int i=0; i < 
	robot_state.multi_dof_joint_state.joint_names.size(); i++) {
    std::string name = robot_state.multi_dof_joint_state.joint_names[i];
    bool mapping_found = false;
    for(unsigned int j=0; j < num_state_spaces; j++) {
      if(state_space->getSubspace(j)->getName() == name) {
        mapping.multi_dof_mapping[i] = j;
        mapping.multi_dof_joint_mapping_type[i] = 
	  get_mapping_type(state_space->getSubspace(j).get());
	mapping_found = true;
	break;
      }    
    }
    if(!mapping_found && fail_if_match_not_found) {
      ROS_ERROR("Could not find mapping for multi_dof_joint_state %s",
		name.c_str());
      return false;
    }
  }

  mapping.real_vector_index = -1;
  //If robot state has no joint state, don't care about this part
  if(robot_state.joint_state.name.empty())
    return true;
  return get_joint_state_to_ompl_state_mapping
    (robot_state.joint_state,state_space,mapping,fail_if_match_not_found);
}


bool get_joint_state_to_ompl_state_mapping
(const sensor_msgs::JointState &joint_state, 
 const ompl::base::CompoundStateSpace *state_space,
 ompl_ros_conversions::RobotStateToOmplStateMapping &mapping,
 bool fail_if_match_not_found) {
  unsigned int num_state_spaces = state_space->getSubspaceCount();

  for(unsigned int j=0; j < num_state_spaces; j++) {
    if(dynamic_cast<ompl::base::RealVectorStateSpace*>
       (state_space->getSubspace(j).get())) {
      mapping.real_vector_index = j;
      break;
    }
  }
  const ompl::base::RealVectorStateSpace *real_vector_state_space = 
    state_space->getSubspace(mapping.real_vector_index).get()->
    as<ompl::base::RealVectorStateSpace>();
  mapping.joint_state_mapping.resize(joint_state.name.size(),-1);  
  mapping.joint_mapping_type.resize
    (joint_state.name.size(),ompl_ros_conversions::UNKNOWN);

  for(unsigned int i=0; i < joint_state.name.size(); i++) {
    bool joint_state_mapping_found = false;
    for (unsigned int j=0; j < num_state_spaces; j++) {
      if (state_space->getSubspace(j)->getName() == joint_state.name[i]) {
	mapping.joint_state_mapping[i] = j;
	joint_state_mapping_found = true;
        mapping.joint_mapping_type[i] = 
	  get_mapping_type(state_space->getSubspace(j).get());
	
	break;
      }
    }
    if (joint_state_mapping_found) {
      continue;
    }
    //check the real vectro state space

    for(unsigned int k=0; k<real_vector_state_space->getDimension(); k++) {
      if(real_vector_state_space->getDimensionName(k) == joint_state.name[i]) {
	mapping.joint_state_mapping[i] = k;
	joint_state_mapping_found = true;
	mapping.joint_mapping_type[i] = ompl_ros_conversions::REAL_VECTOR;
	break;
      } 
    }
    if (!joint_state_mapping_found && fail_if_match_not_found) {
      ROS_ERROR("Unable to find matching subspace for joint %s",
		joint_state.name[i].c_str());
      return false;
    }
  }
  return true;
}

bool ompl_state_to_joint_positions(const ompl::base::State *state,
				   const OmplStateToRobotStateMapping &mapping,
				   const std::vector<std::string> &link_names,
				   std::vector<double> &positions) {
  arm_navigation_msgs::RobotState robot_state;
  robot_state.joint_state.name = link_names;
  robot_state.joint_state.position.resize(robot_state.joint_state.name.size());
  if (!ompl_ros_conversions::ompl_state_to_robot_state
      (state, mapping, robot_state)) {
    ROS_ERROR("unable to convert from ompl state to joint positions");
    return false;
  }
  
  positions.resize(link_names.size());
  for (size_t j = 0; j < link_names.size(); j++) {
    ROS_DEBUG("Finding match for link %s", 
	     link_names[j].c_str());
    bool foundmatch = false;
    for (size_t i = 0; i < robot_state.joint_state.name.size(); i++) {
      if (!(link_names[j].compare(robot_state.joint_state.name[i]))) {
	ROS_DEBUG("Position is %f", robot_state.joint_state.position[i]);
	positions[j] = robot_state.joint_state.position[i];
	foundmatch = true;
	break;
      }
    }
    if (!foundmatch) {
      //darrt::pause("Match not found for link " + link_names[j]);
      return false;
    }
  }
  return true;
}

bool joint_positions_to_ompl_state(const std::vector<double> &positions,
				   const RobotStateToOmplStateMapping &mapping,
				   const std::vector<std::string> link_names,
				   ompl::base::State *state) {
  arm_navigation_msgs::RobotState robot_state;
  robot_state.joint_state.name = link_names;
  robot_state.joint_state.position = positions;

  if (!ompl_ros_conversions::robot_state_to_ompl_state
      (robot_state, mapping, state)) {
    ROS_ERROR("unable to convert robot state to ompl state");
    return false;
  }
  return true;

}

/**
   @brief Convert an ompl state to a kinematic state given the appropriate mapping
   @param ompl_state The ompl state to convert to
   @param mapping The given mapping
   @param joint_state_group The kinematic state to convert from
*/
bool ompl_state_to_kinematic_state_group
(const ompl::base::State *ompl_state,
 const ompl_ros_conversions::OmplStateToKinematicStateMapping &mapping,
 planning_models::KinematicState::JointStateGroup* joint_state_group) {
  unsigned int num_state_spaces = mapping.ompl_state_mapping.size();
  const ompl::base::CompoundState *ompl_compound_state = 
    dynamic_cast<const ompl::base::CompoundState*> (ompl_state);

  std::vector<planning_models::KinematicState::JointState*> joint_states = 
    joint_state_group->getJointStateVector();

  ROS_DEBUG_STREAM("Size of mapping_type is " << mapping.mapping_type.size()
		   << " number of state spaces is "
		   << num_state_spaces);

  for(unsigned int i=0; i < num_state_spaces; i++) {
    if(mapping.mapping_type[i] == ompl_ros_conversions::SO2) {
      std::vector<double> value;
      value.push_back(ompl_compound_state->
		      as<ompl::base::SO2StateSpace::StateType>(i)->value);
      joint_states[mapping.ompl_state_mapping[i]]->setJointStateValues(value);
    } else if(mapping.mapping_type[i] == ompl_ros_conversions::SE2) {
      std::vector<double> values;
      values.push_back(ompl_compound_state->
		       as<ompl::base::SE2StateSpace::StateType>(i)->getX());
      values.push_back(ompl_compound_state->
		       as<ompl::base::SE2StateSpace::StateType>(i)->getY());
      values.push_back(ompl_compound_state->
		       as<ompl::base::SE2StateSpace::StateType>(i)->getYaw());
      joint_states[mapping.ompl_state_mapping[i]]->setJointStateValues(values);
    } else if(mapping.mapping_type[i] == ompl_ros_conversions::SE3) {
      std::vector<double> values;
      values.push_back(ompl_compound_state->
		       as<ompl::base::SE3StateSpace::StateType>(i)->getX());
      values.push_back(ompl_compound_state->
		       as<ompl::base::SE3StateSpace::StateType>(i)->getY());
      values.push_back(ompl_compound_state->
		       as<ompl::base::SE3StateSpace::StateType>(i)->getZ());
      values.push_back(ompl_compound_state->as
		       <ompl::base::SE3StateSpace::StateType>(i)->rotation().x);
      values.push_back(ompl_compound_state->as
		       <ompl::base::SE3StateSpace::StateType>(i)->rotation().y);
      values.push_back(ompl_compound_state->as
		       <ompl::base::SE3StateSpace::StateType>(i)->rotation().z);
      values.push_back(ompl_compound_state->as
		       <ompl::base::SE3StateSpace::StateType>(i)->rotation().w);
      joint_states[mapping.ompl_state_mapping[i]]->setJointStateValues(values);
    } else if(mapping.mapping_type[i] == ompl_ros_conversions::REAL_VECTOR) {
      unsigned int real_vector_size = mapping.real_vector_mapping.size();
      for(unsigned int j=0; j < real_vector_size; j++) {
        std::vector<double> values;
        values.push_back(ompl_compound_state->as
			 <ompl::base::RealVectorStateSpace::StateType>
			 (mapping.real_vector_index)->values[j]);
        joint_states[mapping.real_vector_mapping[j]]->
	  setJointStateValues(values);
      }
    }
  }
  joint_state_group->updateKinematicLinks();
  return true;
}


bool kinematic_state_group_to_ompl_state
(const planning_models::KinematicState::JointStateGroup* joint_state_group, 
 const ompl_ros_conversions::KinematicStateToOmplStateMapping &mapping,
 ompl::base::State *ompl_state) {

  ompl::base::CompoundState *ompl_compound_state =
    ompl_state->as<ompl::base::CompoundState>();

  unsigned int num_states = joint_state_group->getDimension();

  std::vector<planning_models::KinematicState::JointState*> joint_states = 
    joint_state_group->getJointStateVector();
  std::vector<std::string> joint_names = joint_state_group->getJointNames();
  for(unsigned int i=0; i < num_states; i++) {
    if (mapping.joint_mapping_type[i] == ompl_ros_conversions::SO2) {
      std::vector<double> value = joint_states[i]->getJointStateValues();
      ompl_compound_state->as<ompl::base::SO2StateSpace::StateType>
	(mapping.joint_state_mapping[i])->value = angles::normalize_angle(value[0]);
    } else if(mapping.joint_mapping_type[i] == ompl_ros_conversions::SE2) {
      std::vector<double> value = joint_states[i]->getJointStateValues();
      ompl_compound_state->as<ompl::base::SE2StateSpace::StateType>
	(mapping.joint_state_mapping[i])->setX(value[0]);
      ompl_compound_state->as<ompl::base::SE2StateSpace::StateType>
	(mapping.joint_state_mapping[i])->setY(value[1]);
      ompl_compound_state->as<ompl::base::SE2StateSpace::StateType>
	(mapping.joint_state_mapping[i])->setYaw(value[2]);
    } else if(mapping.joint_mapping_type[i] == ompl_ros_conversions::SE3) {
      std::vector<double> value = joint_states[i]->getJointStateValues();
      ompl_compound_state->as<ompl::base::SE3StateSpace::StateType>
	(mapping.joint_state_mapping[i])->setX(value[0]);
      ompl_compound_state->as<ompl::base::SE3StateSpace::StateType>
	(mapping.joint_state_mapping[i])->setY(value[1]);
      ompl_compound_state->as<ompl::base::SE3StateSpace::StateType>
	(mapping.joint_state_mapping[i])->setZ(value[2]);
      ompl_compound_state->as<ompl::base::SE3StateSpace::StateType>
	(mapping.joint_state_mapping[i])->rotation().x = value[3];
      ompl_compound_state->as<ompl::base::SE3StateSpace::StateType>
	(mapping.joint_state_mapping[i])->rotation().y = value[4];
      ompl_compound_state->as<ompl::base::SE3StateSpace::StateType>
	(mapping.joint_state_mapping[i])->rotation().z = value[5];
      ompl_compound_state->as<ompl::base::SE3StateSpace::StateType>
	(mapping.joint_state_mapping[i])->rotation().w = value[6];
    } else if(mapping.joint_mapping_type[i] == ompl_ros_conversions::REAL_VECTOR) {
      std::vector<double> value = joint_states[i]->getJointStateValues();
      ompl_compound_state->as<ompl::base::RealVectorStateSpace::StateType>
	(mapping.real_vector_index)->values[mapping.joint_state_mapping[i]] = value[0];
    }
  }
  return true;
};

//this doesn't work if there are planar joints!
bool ompl_state_to_robot_state
(const ompl::base::State *ompl_state,
 const ompl_ros_conversions::OmplStateToRobotStateMapping &mapping,
 arm_navigation_msgs::RobotState &robot_state) {
  unsigned int num_state_spaces = mapping.ompl_state_mapping.size();
  const ompl::base::CompoundState *state = 
    ompl_state->as<ompl::base::CompoundState>();
  for(unsigned int i=0; i < num_state_spaces; i++) {
    if (mapping.mapping_type[i] == ompl_ros_conversions::SO2 && 
	mapping.ompl_state_mapping[i] > -1) {
      robot_state.joint_state.position[mapping.ompl_state_mapping[i]] = 
	state->as<ompl::base::SO2StateSpace::StateType>(i)->value;
    } else if (mapping.mapping_type[i] == ompl_ros_conversions::SE3 && 
	       mapping.ompl_state_mapping[i] > -1) {
      ompl_ros_conversions::SE3StateSpace_to_pose_msg
	(*(state->as<ompl::base::SE3StateSpace::StateType>(i)),
	 robot_state.multi_dof_joint_state.poses
	 [mapping.ompl_state_mapping[i]]);
    } else if (mapping.mapping_type[i] == ompl_ros_conversions::REAL_VECTOR) {
      ompl_ros_conversions::ompl_real_vector_state_to_joint_state
	(*(state->as<ompl::base::RealVectorStateSpace::StateType>(i)),
	 mapping,robot_state.joint_state);
    } else {
      ROS_ERROR("Unrecognized mapping type %d for state space %u",
		mapping.mapping_type[i], i);
      return false;
    }
  }
  return true;
}


bool robot_state_to_ompl_state
(const arm_navigation_msgs::RobotState &robot_state,
 const ompl_ros_conversions::RobotStateToOmplStateMapping &mapping,
 ompl::base::State *ompl_state,
 const bool &fail_if_match_not_found) {
  if (robot_state.multi_dof_joint_state.joint_names.size() >
      mapping.multi_dof_mapping.size() && fail_if_match_not_found) {
    return false;
  }
  for(unsigned int i=0; i < mapping.multi_dof_mapping.size(); i++) {
    if(mapping.multi_dof_mapping[i] > -1) {
      ompl_ros_conversions::pose_msg_to_SE3StateSpace
	(robot_state.multi_dof_joint_state.poses[i],
	 *(ompl_state->as<ompl::base::CompoundState>()->
	   as<ompl::base::SE3StateSpace::StateType>
	   (mapping.multi_dof_mapping[i])));
    }
  }
  if(mapping.real_vector_index > -1) {
    if (robot_state.joint_state.name.size() >
	mapping.joint_mapping_type.size() && fail_if_match_not_found) {
      return false;
    }
    for(unsigned int i=0; i < mapping.joint_mapping_type.size(); i++) {
      if(mapping.joint_mapping_type[i] == ompl_ros_conversions::SO2 && 
	 mapping.joint_state_mapping[i] > -1)
        ompl_state->as<ompl::base::CompoundState>()->
	  as<ompl::base::SO2StateSpace::StateType>
	  (mapping.joint_state_mapping[i])->value = 
	  angles::normalize_angle(robot_state.joint_state.position[i]);
    }
  }
  if(mapping.real_vector_index > -1) {
    return joint_state_to_real_vector_state
      (robot_state.joint_state, mapping,
       *(ompl_state->as<ompl::base::CompoundState>()->
	 as<ompl::base::RealVectorStateSpace::StateType>
	 (mapping.real_vector_index)), fail_if_match_not_found);
  }
  return true;                                       
}

bool joint_state_to_real_vector_state
(const sensor_msgs::JointState &joint_state,
 const ompl_ros_conversions::RobotStateToOmplStateMapping &mapping,
 ompl::base::RealVectorStateSpace::StateType &real_vector_state,
 const bool &fail_if_match_not_found) {
  if (joint_state.name.size() > mapping.joint_mapping_type.size() &&
      fail_if_match_not_found) {
    return false;
  }
  for (unsigned int i=0; i < mapping.joint_mapping_type.size(); i++) {
    if (mapping.joint_mapping_type[i] == ompl_ros_conversions::REAL_VECTOR && 
	mapping.joint_state_mapping[i] > -1) {
      real_vector_state.as<ompl::base::RealVectorStateSpace::StateType>()->
	values[mapping.joint_state_mapping[i]] = joint_state.position[i];
    }
  }
  return true;
}

bool ompl_real_vector_state_to_joint_state
(const ompl::base::RealVectorStateSpace::StateType &ompl_state,
 const ompl_ros_conversions::OmplStateToRobotStateMapping &mapping,
 sensor_msgs::JointState &joint_state) {
  for (unsigned int i=0; i < mapping.real_vector_mapping.size(); i++) {
    if (mapping.real_vector_mapping[i] > -1) {
      ROS_DEBUG("Real vector mapping at %u is %d",
		i, mapping.real_vector_mapping[i]);
      joint_state.position[mapping.real_vector_mapping[i]] = 
	ompl_state.values[i];
    } else {
      return false;
    }
  }
  return true;
}

/**
   @brief Create a robot trajectory message for a joint state group
   @param joint_model_group The input group
   @param robot_trajectory The output robot trajectory
*/
bool joint_model_group_to_robot_trajectory
(const planning_models::KinematicModel::JointModelGroup *joint_model_group, 
 arm_navigation_msgs::RobotTrajectory &robot_trajectory) {
  const std::vector<const planning_models::KinematicModel::JointModel*> 
    &joint_models = joint_model_group->getJointModels();
  for(unsigned int i=0; i < joint_models.size(); i++) {
    const planning_models::KinematicModel::RevoluteJointModel* revolute_joint = 
      dynamic_cast<const planning_models::KinematicModel::RevoluteJointModel*>
      (joint_models[i]);
    const planning_models::KinematicModel::PrismaticJointModel* prismatic_joint = 
      dynamic_cast<const planning_models::KinematicModel::PrismaticJointModel*>
      (joint_models[i]);
    const planning_models::KinematicModel::PlanarJointModel* planar_joint = 
      dynamic_cast<const planning_models::KinematicModel::PlanarJointModel*>
      (joint_models[i]);
    const planning_models::KinematicModel::FloatingJointModel* floating_joint = 
      dynamic_cast<const planning_models::KinematicModel::FloatingJointModel*>
      (joint_models[i]);
    if (revolute_joint || prismatic_joint) {
      robot_trajectory.joint_trajectory.joint_names.push_back
	(joint_models[i]->getName());
    } else if (planar_joint || floating_joint) {
      robot_trajectory.multi_dof_joint_trajectory.joint_names.push_back
	(joint_models[i]->getName());
      robot_trajectory.multi_dof_joint_trajectory.frame_ids.push_back
	(joint_models[i]->getParentFrameId());
      robot_trajectory.multi_dof_joint_trajectory.child_frame_ids.push_back
	(joint_models[i]->getChildFrameId());
    } else {
      return false;  
    }
  }
  return true;
}

bool ompl_path_geometric_to_robot_trajectory
(const ompl::geometric::PathGeometric &path,
 const ompl::base::StateSpacePtr &state_space,
 const ompl_ros_conversions::OmplStateToRobotStateMapping &mapping,
 arm_navigation_msgs::RobotTrajectory &robot_trajectory,
 const planning_models::KinematicModel::JointModelGroup *joint_model_group) {
  
  if(robot_trajectory.joint_trajectory.joint_names.empty() && 
     robot_trajectory.multi_dof_joint_trajectory.joint_names.empty()) {
    if (!joint_model_group) {
      ROS_ERROR("Robot trajectory needs to initialized before calling this function or you need to pass an argument for joint_model_group");
      return false;
    }
    if (!joint_model_group_to_robot_trajectory
	(joint_model_group, robot_trajectory)) {
      ROS_ERROR("Unable to convert joint state group to robot trajectory");
      return false;
    }
  }

  unsigned int num_points = path.getStateCount();
  unsigned int num_state_spaces = mapping.ompl_state_mapping.size();
  bool multi_dof = false;
  bool single_dof = false;

  if(!robot_trajectory.multi_dof_joint_trajectory.joint_names.empty()) {
    multi_dof = true;
    robot_trajectory.multi_dof_joint_trajectory.points.resize(num_points);
    for(unsigned int i=0; i < num_points; i++) {
	robot_trajectory.multi_dof_joint_trajectory.points[i].poses.resize
	  (robot_trajectory.multi_dof_joint_trajectory.joint_names.size());
	robot_trajectory.multi_dof_joint_trajectory.points[i].time_from_start = 
	  ros::Duration(0.0);
    }
  }
  if(!robot_trajectory.joint_trajectory.joint_names.empty()) {
    single_dof = true;
    robot_trajectory.joint_trajectory.points.resize(num_points);
    for(unsigned int i=0; i < num_points; i++) {      
	robot_trajectory.joint_trajectory.points[i].positions.resize
	  (robot_trajectory.joint_trajectory.joint_names.size());
	robot_trajectory.joint_trajectory.points[i].time_from_start = 
	  ros::Duration(0.0);
    }
  }
  
  for(unsigned int i=0; i < num_points; i++) {
    //std::ostringstream ostr;
    //ostr << "Converting state ";
    //state_space.get()->printState(path.getState(i), ostr);
    //darrt::pause(ostr.str(), 0);
    for(unsigned int j=0; j < num_state_spaces; j++) {
      if(mapping.mapping_type[j] == ompl_ros_conversions::SO2) {
        robot_trajectory.joint_trajectory.points[i].
	  positions[mapping.ompl_state_mapping[j]] = 
	  path.getState(i)->as<ompl::base::CompoundState>()->
	  as<ompl::base::SO2StateSpace::StateType>(j)->value;
      } else if(mapping.mapping_type[j] == ompl_ros_conversions::SE2) {
        ompl_ros_conversions::SE2StateSpace_to_pose_msg
	  (*(path.getState(i)->as<ompl::base::CompoundState>()->
	     as<ompl::base::SE2StateSpace::StateType>(j)),
	   robot_trajectory.multi_dof_joint_trajectory.points[i].
	   poses[mapping.ompl_state_mapping[j]]);
      } else if(mapping.mapping_type[j] == ompl_ros_conversions::SE3) {
        ompl_ros_conversions::SE3StateSpace_to_pose_msg
	  (*(path.getState(i)->as<ompl::base::CompoundState>()->
	     as<ompl::base::SE3StateSpace::StateType>(j)),
	   robot_trajectory.multi_dof_joint_trajectory.points[i].
	   poses[mapping.ompl_state_mapping[j]]);
      } else if(mapping.mapping_type[j] == ompl_ros_conversions::SO3) {
        ompl_ros_conversions::SO3StateSpace_to_quaternion_msg
	  (*(path.getState(i)->as<ompl::base::CompoundState>()->
	     as<ompl::base::SO3StateSpace::StateType>(j)),
	   robot_trajectory.multi_dof_joint_trajectory.points[i].
	   poses[mapping.ompl_state_mapping[j]].orientation);
      } else {
	ROS_DEBUG("Space %u is real vector space, index %u, size %u",
		  j, mapping.real_vector_index, 
		  (unsigned int)mapping.real_vector_mapping.size());
	//real vector value
        const ompl::base::RealVectorStateSpace::StateType *real_vector_state = 
	  path.getState(i)->as<ompl::base::CompoundState>()->
	  as<ompl::base::RealVectorStateSpace::StateType>(mapping.real_vector_index);
        for(unsigned int k=0; k < mapping.real_vector_mapping.size(); k++) {
          robot_trajectory.joint_trajectory.points[i].positions
	    [mapping.real_vector_mapping[k]] = real_vector_state->values[k];
	}
      }
    }
  }
  return true;  
}

void SE3StateSpace_to_pose_msg(const ompl::base::SE3StateSpace::StateType &pose,
			       geometry_msgs::Pose &pose_msg) {
  pose_msg.position.x = pose.getX();
  pose_msg.position.y = pose.getY();
  pose_msg.position.z = pose.getZ();
  ompl::base::SO3StateSpace::StateType quaternion;
  quaternion.x = pose.rotation().x;
  quaternion.y = pose.rotation().y;
  quaternion.z = pose.rotation().z;
  quaternion.w = pose.rotation().w;

  SO3StateSpace_to_quaternion_msg(quaternion,pose_msg.orientation);
}

void SO3StateSpace_to_quaternion_msg
(const ompl::base::SO3StateSpace::StateType &quaternion,
 geometry_msgs::Quaternion &quaternion_msg) {
  quaternion_msg.x = quaternion.x;
  quaternion_msg.y = quaternion.y;
  quaternion_msg.z = quaternion.z;
  quaternion_msg.w = quaternion.w;
}


void pose_msg_to_SE3StateSpace(const geometry_msgs::Pose &pose_msg,
			       ompl::base::SE3StateSpace::StateType &pose) {
  pose.setX(pose_msg.position.x);
  pose.setY(pose_msg.position.y);
  pose.setZ(pose_msg.position.z);
  quaternion_msg_to_SO3StateSpace(pose_msg.orientation,pose.rotation());  
}

void quaternion_msg_to_SO3StateSpace
(const geometry_msgs::Quaternion &quaternion_msg,
 ompl::base::SO3StateSpace::StateType &quaternion) {
  quaternion.x = quaternion_msg.x;
  quaternion.y = quaternion_msg.y;
  quaternion.z = quaternion_msg.z;
  quaternion.w = quaternion_msg.w;
}


void SE2StateSpace_to_pose_msg(const ompl::base::SE2StateSpace::StateType &pose,
			       geometry_msgs::Pose &pose_msg) {
  pose_msg.position.x = pose.getX();
  pose_msg.position.y = pose.getY();
  //theta is always a rotation around the z axis
  pose_msg.orientation.z = sin(pose.getYaw()/2.0);
  pose_msg.orientation.w = cos(pose.getYaw()/2.0);
}

void pose_msg_to_SE2StateSpace(const geometry_msgs::Pose &pose_msg,
			       ompl::base::SE2StateSpace::StateType &pose) {
  pose.setX(pose_msg.position.x);
  pose.setY(pose_msg.position.y);
  pose.setYaw(darrt::quaternion_to_yaw(pose_msg.orientation));
}

void pose2D_msg_to_SE2StateSpace(const geometry_msgs::Pose2D &pose_msg,
				 ompl::base::SE2StateSpace::StateType &pose) {
  pose.setX(pose_msg.x);
  pose.setY(pose_msg.y);
  pose.setYaw(pose_msg.theta);
}

void SE2StateSpace_to_pose2D_msg(const ompl::base::SE2StateSpace::StateType &pose,
				 geometry_msgs::Pose2D &pose_msg) {
  pose_msg.x = pose.getX();
  pose_msg.y = pose.getY();
  pose_msg.theta = pose.getYaw();
}


bool convert_point_to_state(ompl::base::State *state,
			    const ompl::base::StateSpace *space,
			    const geometry_msgs::Point &pt) {

  //figure out what state space we have...
  const ompl::base::RealVectorStateSpace *rv = 
    dynamic_cast<const ompl::base::RealVectorStateSpace *>(space);
  if (rv) {
    return convert_point_to_state
      (state->as<ompl::base::RealVectorStateSpace::StateType>(), rv, pt);
  }
  
  const ompl::base::SE2StateSpace *se2 = 
    dynamic_cast<const ompl::base::SE2StateSpace *>(space);
  if (se2) {
    return convert_point_to_state
      (state->as<ompl::base::SE2StateSpace::StateType>(), se2, pt);
  }

  const ompl::base::SE3StateSpace *se3 = 
    dynamic_cast<const ompl::base::SE3StateSpace *>(space);
  if (se3) {
    return convert_point_to_state
      (state->as<ompl::base::SE3StateSpace::StateType>(), se3, pt);
  }

  ROS_ERROR("Attempt to convert point to unkown space type: %s",
	    space->getName().c_str());
  return false;
}

bool convert_point_to_state(ompl::base::RealVectorStateSpace::StateType *state,
			    const ompl::base::RealVectorStateSpace *space,
			    const geometry_msgs::Point &pt) {
  if (space->getDimension() == 0) {
    return true;
  }
  bool ret = true;

  if (pt.x < space->getBounds().low[0] || pt.x > space->getBounds().high[0]) {
    ROS_WARN("Converting point to state: x = %f is not within bounds!",
	     pt.x);
    ret = false;
  }
  state->values[0] = pt.x;
  if (space->getDimension() == 1) {
    return ret;
  }
  if (pt.y < space->getBounds().low[1] || pt.y > space->getBounds().high[1]) {
    ROS_WARN("Converting point to state: y = %f is not within bounds!",
	     pt.y);
    ret = false;
  }
  state->values[1] = pt.y;
  if (space->getDimension() == 2) {
    return ret;
  }
  if (pt.z < space->getBounds().low[2] || pt.z > space->getBounds().high[2]) {
    ROS_WARN("Converting point to state: z is not within bounds!");
    ret = false;
  }
  state->values[2] = pt.z;
  for (unsigned int i = 3; i < space->getDimension(); i++) {
    double range = space->getBounds().high[i] - space->getBounds().low[i];
    state->values[i] = rand()/(double)RAND_MAX*range + space->getBounds().low[i];
  }
  return ret;
}

bool convert_point_to_state(ompl::base::SE2StateSpace::StateType *state,
			    const ompl::base::SE2StateSpace *space,
			    const geometry_msgs::Point &pt) {
  state->setXY(pt.x, pt.y);
  state->setYaw(rand()/(double)RAND_MAX*
		(2.0*boost::math::constants::pi<double>()));
  if (pt.x < space->getBounds().low[0] || pt.x > space->getBounds().high[0]) {
    ROS_WARN("Converting point to state: x is not within bounds");
    return false;
  }
  if (pt.y < space->getBounds().low[1] || pt.y > space->getBounds().high[1]) {
    ROS_WARN("Converting point to state: y is not within bounds");
    return false;
  }
  return true;
}

bool convert_point_to_state(ompl::base::SE3StateSpace::StateType *state,
			    const ompl::base::SE3StateSpace *space,
			    const geometry_msgs::Point &pt) {
  state->setXYZ(pt.x, pt.y, pt.z);
  state->rotation().x = rand()/(double)RAND_MAX;
  state->rotation().y = rand()/(double)RAND_MAX;
  state->rotation().z = rand()/(double)RAND_MAX;
  state->rotation().w = rand()/(double)RAND_MAX;
  double norm = sqrt(state->rotation().x*state->rotation().x + 
		     state->rotation().y*state->rotation().y + 
		     state->rotation().z*state->rotation().z +
		     state->rotation().w*state->rotation().w);
  state->rotation().x /= norm;
  state->rotation().y /= norm;
  state->rotation().z /= norm;
  state->rotation().w /= norm;
  
  if (pt.x < space->getBounds().low[0] || pt.x > space->getBounds().high[0]) {
    ROS_WARN("Converting point to state: x is not within bounds");
    return false;
  }
  if (pt.y < space->getBounds().low[1] || pt.y > space->getBounds().high[1]) {
    ROS_WARN("Converting point to state: y is not within bounds");
    return false;
  }
  if (pt.z < space->getBounds().low[2] || pt.z > space->getBounds().high[2]) {
    ROS_WARN("Converting point to state: z is not within bounds");
    return false;
  }
  return true;
}

bool convert_state_to_point(const ompl::base::State *state,
			    const ompl::base::StateSpace *space,
			    geometry_msgs::Point &pt) {
  //figure out what state space we have...
  const ompl::base::RealVectorStateSpace *rv = 
    dynamic_cast<const ompl::base::RealVectorStateSpace *>(space);
  if (rv) {
    return convert_state_to_point
      (state->as<ompl::base::RealVectorStateSpace::StateType>(), rv, pt);
  }
  
  const ompl::base::SE2StateSpace *se2 = 
    dynamic_cast<const ompl::base::SE2StateSpace *>(space);
  if (se2) {
    return convert_state_to_point
      (state->as<ompl::base::SE2StateSpace::StateType>(), se2, pt);
  }

  const ompl::base::SE3StateSpace *se3 = 
    dynamic_cast<const ompl::base::SE3StateSpace *>(space);
  if (se3) {
    return convert_state_to_point
      (state->as<ompl::base::SE3StateSpace::StateType>(), se3, pt);
  }

  ROS_ERROR("Attempt to convert from point to unkown space type: %s",
	    space->getName().c_str());
  return false;
}

bool convert_state_to_point(const ompl::base::RealVectorStateSpace::StateType 
			    *state,
			    const ompl::base::RealVectorStateSpace *space,
			    geometry_msgs::Point &pt) {
  if (space->getDimension() == 0) {
    return true;
  }
  pt.x = state->values[0];
  if (space->getDimension() == 1) {
    return true;
  }
  pt.y = state->values[1];
  if (space->getDimension() == 2) {
    return true;
  }
  pt.z = state->values[2];
  return true;
}

bool convert_state_to_point(const ompl::base::SE2StateSpace::StateType *state,
			    const ompl::base::SE2StateSpace *space,
			    geometry_msgs::Point &pt) {
  pt.x = state->getX();
  pt.y = state->getY();
  return true;
}

bool convert_state_to_point(const ompl::base::SE3StateSpace::StateType *state,
			    const ompl::base::SE2StateSpace *space,
			    geometry_msgs::Point &pt) {
  pt.x = state->getX();
  pt.y = state->getY();
  pt.z = state->getZ();
  return true;
}
}
