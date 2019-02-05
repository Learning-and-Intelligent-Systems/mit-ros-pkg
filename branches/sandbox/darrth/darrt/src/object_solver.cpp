#include "darrt/object_solver.hh"
#include "darrt/object_space.hh"
#include "darrt/robot_space.hh"
#include "darrt/space.hh"

bool darrt::ObjectSolver::display_solution(bool step, unsigned int resolution) const {
  oc::PathControl &solution = solver_->getSolutionPath();
  ROS_INFO("Displaying object path");

  const DARRTStateSpace *dspace = getSpaceInformation()->getStateSpace()
    ->as<DARRTStateSpace>();

  visualization_msgs::MarkerArray marray;
  for (size_t i = 0; i < solution.getStateCount(); i+= resolution) {
    if (step) {
      marray.markers.clear();
    }
    for (unsigned int j = 0; j < dspace->object_indexes().size(); j++) {
      unsigned int ind = dspace->object_indexes().at(j);
      unsigned int id = i;
      if (step) {
	id = 0;
      }
      const ObjectStateSpace *ospace = dspace->getSubspace(ind)->as<ObjectStateSpace>();
      visualization_msgs::MarkerArray ar = ospace->displayable_state
	(dspace->get_state(ind, solution.getState(i)), ospace->getName()+"_path", id);
      for (unsigned int k = 0; k < ar.markers.size(); k++) {
	marray.markers.push_back(ar.markers[k]);
      }
    }
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("%s", dspace->state_string(solution.getState(i)).c_str());
    }
    if (step) {
      solver_->getStateSpace()->as<DARRTStateSpace>()->display(marray);
      pause("", 0, true);
    }   
  }
  if (!step) {
    solver_->getStateSpace()->as<DARRTStateSpace>()->display(marray);
  }
  return true;
}

bool darrt::ObjectSolver::setup_state_validity_checker(const Goal &goal) {

  ROS_INFO("This is the object state validity checker setup.");

  const DARRTStateSpace *space = getSpaceInformation()->getStateSpace()
    ->as<DARRTStateSpace>();

  //disable collisions for all the non-updated links
  //note that we want to KEEP the collisions with the objects
  //we can't use the handy function, but
  //since the collisions are ordered we can do this by first
  //disabling all collisions and then enabling the ones we want

  an::OrderedCollisionOperations default_collisions;

  //disable all collisions
  an::CollisionOperation op;
  op.object1 = op.COLLISION_SET_ALL;
  op.object2 = op.COLLISION_SET_ALL;
  op.operation = op.DISABLE;
  default_collisions.collision_operations.push_back(op);

  for (unsigned int i = 0; i < space->object_indexes().size(); i++) {
    unsigned int ind = space->object_indexes().at(i);
    //enable collisions for the object
    op.object1 = space->getSubspace(ind)->getName();
    op.object2 = op.COLLISION_SET_ALL;
    op.operation = op.ENABLE;
 
    default_collisions.collision_operations.push_back(op);
    
    //except disable the self collision
    op.object2 = space->getSubspace(ind)->getName();
    op.operation = op.DISABLE;
    default_collisions.collision_operations.push_back(op);
  
    //and disable collisions with all moving parts of the robot
    for (unsigned int i = 0; i < goal.robot_groups.size(); i++) {
      std::vector<std::string> updated_link_names = 
	environment_interface_->getKinematicModel()->
	getModelGroup(goal.robot_groups[i])->getUpdatedLinkModelNames();
      for (size_t i = 0; i < updated_link_names.size(); i++) {
	op.object2 = updated_link_names[i];
	default_collisions.collision_operations.push_back(op);
      }
    }
  }
  
  //get the collision matrix corresponding to these plus the usual
  //default collisions (except these are wiped out by the first collision
  //disabling everything)
  collision_space::EnvironmentModel::AllowedCollisionMatrix acm
    = environment_interface_->getDefaultAllowedCollisionMatrix();  
  std::vector<std::string> objnames;
  environment_interface_->getCollisionObjectNames(objnames);
  std::vector<std::string> anames;
  environment_interface_->getAttachedCollisionObjectNames(anames);
  
  planning_environment::applyOrderedCollisionOperationsListToACM
    (default_collisions, objnames, anames,
     environment_interface_->getKinematicModel(), acm);
  
  //subspace validity checkers
  std::vector<ob::StateValidityCheckerPtr> checkers;
  for (unsigned int i = 0; i < space->getSubspaceCount(); i++) {
    ob::SpaceInformationPtr si_ptr
      (new ob::SpaceInformation(space->getSubspace(i)));
    ob::StateValidityCheckerPtr chkr;
    if (space->robot_index() >= 0 && i == static_cast<unsigned int>(space->robot_index())) {
      //don't do robot state checks
      chkr.reset();
    } else {
      chkr.reset(new ObjectStateValidityChecker(si_ptr.get(), acm, false, true));
    }
    checkers.push_back(chkr);
  }


  ob::StateValidityCheckerPtr sv_ptr
    (new DARRTStateValidityChecker
      (solver_->getSpaceInformation().get(), environment_interface_,
       acm, checkers));

  solver_->setStateValidityChecker(sv_ptr);
  return true;
}
