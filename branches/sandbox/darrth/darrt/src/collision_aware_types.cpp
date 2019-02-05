#include "darrt/collision_aware_types.hh"
#include "darrt/utils.hh"

#include <planning_environment/models/model_utils.h>

bool darrt::CollisionAwareState::add_allowed_collisions
(const an::OrderedCollisionOperations &ops) {
  for (size_t i = 0; i < ops.collision_operations.size(); i++) {
    if (!add_allowed_collision(ops.collision_operations[i])) {
      return false;
    }
  }
  return true;
}

darrt::CollisionAwareStateValidityChecker::CollisionAwareStateValidityChecker
(ob::SpaceInformation *si,
 collision_space::EnvironmentModel::AllowedCollisionMatrix
 &default_allowed_collisions) :
  ob::StateValidityChecker(si),
  default_allowed_collisions_(default_allowed_collisions) 
{
  environment_interface_ = NULL;
  caspace_ = 
    dynamic_cast<const CollisionAwareStateSpace *>(si->getStateSpace().get());
  if (!caspace_) {
    ROS_ERROR("Collision aware state validity checker only works with collision aware state spaces!");
  }
}


bool darrt::CollisionAwareStateValidityChecker::revert_collision_model() const {
  if (!caspace_) {
    ROS_ERROR("Cannot use collision aware state validity checker with state space that cannot be cast to collision aware state space");
    return false;
  }
  if (!environment_interface_) {
    ROS_ERROR("Must call setup before reverting collision model!");
    return false;
  }
  environment_interface_->setAlteredAllowedCollisionMatrix
    (default_allowed_collisions_);
  return true;
}

bool darrt::CollisionAwareStateValidityChecker::
reconcile_collision_model(const ob::State *ompl_state) const {
  if (!caspace_) {
    ROS_ERROR("Cannot use collision aware state validity checker with state space that cannot be cast to collision aware state space");
    return false;
  }

  if (!environment_interface_) {
    ROS_ERROR("Must call setup before reconciling collision model!");
    return false;
  }


  //update the locations of manipulated objects
  //usually also checks bounds
  if (!caspace_->update_model(ompl_state, environment_interface_)) {
    return false;
  }
  ROS_DEBUG("Updated locations");

  ROS_DEBUG("Reverting collision model");
  if (!revert_collision_model()) {
    ROS_ERROR("Unable to revert collision model while reconciling");
    return false;
  }

  //enable the collisions that the state specifies
  const CollisionAwareState *state = 
    dynamic_cast<const CollisionAwareState *>(ompl_state);

  collision_space::EnvironmentModel::AllowedCollisionMatrix acm =
    default_allowed_collisions_;
  if (debug_level >= DCOLLISIONS) {
    ROS_INFO_STREAM("Applying collision ops: " << state->allowed_collisions);
  }
  //note: use this instead of applyOrderedCollisionOperationsToMatrix
  //because this can recognized group names like "wheels" or
  //"r_end_effector"
  std::vector<std::string> objnames;
  environment_interface_->getCollisionObjectNames(objnames);
  std::vector<std::string> anames;
  environment_interface_->getAttachedCollisionObjectNames(anames);
  planning_environment::applyOrderedCollisionOperationsListToACM
    (state->allowed_collisions, objnames, anames, 
     environment_interface_->getKinematicModel(), acm);
  environment_interface_->setAlteredAllowedCollisionMatrix(acm);
  //if (debug_level >= DCOLLISIONS) {
  //an::AllowedCollisionMatrix matrix;
  //environment_interface_->getCollisionSpaceAllowedCollisions(matrix);
  //     //in this matrix 1 means the collision IS ALLOWED
  //     //i dunno why it says "enabled" because it means disabled.
  //     //very confusing
  //ROS_INFO_STREAM("Collision ops applied.  Matrix is:\n" << matrix);
  //   }
  return true;
}
  
darrt::CollisionAwarePrimitiveInstance::CollisionAwarePrimitiveInstance(const Primitive *prim,
									std::string type,
									const ob::State *source,
									const ob::State *destination) :
  PrimitiveInstance(prim, type, source, destination) { 
  dynamic_cast<CollisionAwareState *>(source_)->set_allowed_collisions(allowed_collisions);
  dynamic_cast<CollisionAwareState *>(destination_)->set_allowed_collisions(allowed_collisions);
}


bool darrt::CollisionAwarePrimitiveInstance::add_allowed_collisions
(const an::OrderedCollisionOperations &ops) {
  for (size_t i = 0; i < ops.collision_operations.size(); i++) {
    if (!add_allowed_collision(ops.collision_operations[i])) {
      return false;
    }
  }
  return true;
}

bool darrt::CollisionAwarePrimitiveInstance::add_allowed_collision
(an::CollisionOperation op) {
  allowed_collisions.collision_operations.push_back(op);
  dynamic_cast<CollisionAwareState *>(source_)->add_allowed_collision(op);
  dynamic_cast<CollisionAwareState *>(destination_)->add_allowed_collision(op);
  return true;
}

bool darrt::CollisionAwarePrimitiveInstance::set_allowed_collisions
(const an::OrderedCollisionOperations &ops) {
  allowed_collisions.collision_operations.clear();
  dynamic_cast<CollisionAwareState *>(source_)->allowed_collisions.collision_operations.clear();
  dynamic_cast<CollisionAwareState *>(destination_)->allowed_collisions.collision_operations.clear();
  add_allowed_collisions(ops);
  return true;
}

std::string darrt::CollisionAwarePrimitiveInstance::str() const {
  std::ostringstream ostr;
  ostr << prim_->str() << "[" << turns_ << "](";
  prim_->space_information()->printState(source_, ostr);
  ostr << ", ";
  prim_->space_information()->printState(destination_, ostr);
  ostr << ", Ignoring collisions:";
  for (size_t i = 0; i < allowed_collisions.collision_operations.size(); i++) {
    ostr << " (" << allowed_collisions.collision_operations[i].object1 
	 << ", " << allowed_collisions.collision_operations[i].object2 << ")";
  }
  ostr << ")";
  return ostr.str();
}
