//C++ includes
#include <numeric>
#include <iostream>
#include <sstream>

//local includes
#include "darrt/primitive.hh"
#include "darrt/utils.hh"
#include "darrt/space.hh"

//for the conversion functions
#include "darrt/robot_space.hh"
#include "darrt/object_space.hh"
#include "darrt/transform_ros_types.hh"

//ROS includes
#include <arm_navigation_msgs/ArmNavigationErrorCodes.h>
#include <arm_navigation_msgs/convert_messages.h>
#include <arm_navigation_msgs/GetRobotState.h>
#include <arm_navigation_msgs/GetStateValidity.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>

//OMPL includes
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>

//for testing only
#include "darrt/chain_space.hh"
#include "darrt/robot_base_space.hh"


void darrt::DARRTStateSpace::StateType::set_control(const Primitive *prim, std::string type) {
  control_ = prim;
  type_ = type;
  if (!prim) {
    return;
  }
  for (unsigned int i = 0; i < prim->space_information()->getStateSpace()->as<DARRTStateSpace>()
	 ->getSubspaceCount(); i++) {
    CollisionAwareState *cas = dynamic_cast<CollisionAwareState *>(components[i]);
    if (cas) {
      cas->set_control(prim, type);
    }
  }
}

darrt::DARRTStateSpace::DARRTStateSpace
(const PrimitiveList &prims, 
 const SupportSurfaceList &support_surfaces, 
 const std::vector< std::pair<DARRTProjectionFunction, double> > 
 &projection_functions) :
  CompoundStateSpace(), Displayable("robot_state_markers"), 
  support_surfaces_(support_surfaces) 
{
  plist_ = prims;
  for (size_t i = 0; i < plist_.size(); i++) {
    if (plist_[i]->transfer()) {
      tplist_.push_back(dynamic_cast<const TransferPrimitive *>(plist_[i]));
    }
  }
  goal_ = NULL;
  projection_functions_ = projection_functions;
  subspace_distance_ = -1;
}

darrt::DARRTStateSpace::
DARRTStateSpace(const std::vector<ob::StateSpacePtr> &components,
		int robot_index, const PrimitiveList &plist,
		const SupportSurfaceList &support_surfaces, 
		const std::vector< std::pair<DARRTProjectionFunction, double> > &projection_functions) :
  CompoundStateSpace(components, std::vector<double>(components.size(), 1.0)),
  Displayable("robot_state_markers"),
  support_surfaces_(support_surfaces) 
{
  
  robot_index_ = robot_index;

  goal_ = NULL;
  projection_functions_ = projection_functions;
  subspace_distance_ = -1;

  for (unsigned int i = 0; i < components.size(); i++) {
    if (robot_index_ < 0 || i != static_cast<unsigned int>(robot_index_)) {
      object_indexes_.push_back(i);
    }
  }
  //create the transfer primitive map
  plist_ = plist;
  for (size_t i = 0; i < plist_.size(); i++) {
    if (plist_[i]->transfer()) {
      tplist_.push_back(dynamic_cast<const TransferPrimitive *>(plist_[i]));
    }
	    
  }
  
}

void darrt::DARRTStateSpace::set_goal(ob::GoalSampleableRegion *goal) {
  goal_ = goal;
}

void darrt::DARRTStateSpace::reset_goal() {
  goal_ = NULL;
}
  
void darrt::DARRTStateSpace::addSubspace(const ob::StateSpacePtr &component,
					 double weight) {
  throw "Cannot call addSubspace with a weight.  Must call with a thing.";
}

void darrt::DARRTStateSpace::addSubspace(const ob::StateSpacePtr &component,
					 bool robot_space) {
  if (robot_space) {
    robot_index_ = componentCount_;
  } else {
    object_indexes_.push_back(componentCount_);
  }  

  CompoundStateSpace::addSubspace(component, 1.0);
  ROS_INFO("When adding subspace count for component %s is %ld",
	   component->getName().c_str(), component.use_count());
}
  
ob::State *darrt::DARRTStateSpace::allocState() const {
  StateType *state = new StateType();
  allocStateComponents(state);
  //ROS_INFO("Created state %p", state);
  return state;
}

void darrt::DARRTStateSpace::freeState(ob::State *s) const {
  //cout << "Freeing state " << s << endl;
  CompoundStateSpace::freeState(s);
}

//should probably use a ValidStateSampler
ob::StateSamplerPtr darrt::DARRTStateSpace::allocDefaultStateSampler() const {
  ob::StateSamplerPtr ss_ptr(new ProjectibleStateSampler(this, projection_functions_, goal_));
  ProjectibleStateSampler *ss = dynamic_cast<ProjectibleStateSampler *>(ss_ptr.get());
  for (unsigned int i = 0; i < componentCount_; i++) {
    ss->addSampler(components_[i]->allocStateSampler(), 1.0);
  }
  return ss_ptr;
}

void darrt::DARRTStateSpace::copyState
(ob::State *destination, const ob::State *source) const {
  CompoundStateSpace::copyState(destination, source);
  StateType *d = destination->as<StateType>();
  d->set_allowed_collisions(allowed_collisions(source));
  d->setProjectionFunction(source->as<StateType>()->getProjectionFunction());
  d->set_control(source->as<StateType>()->control(),
		 source->as<StateType>()->type());  
  if (source->as<StateType>()->valid()) {
    d->set_valid();
  } else {
    d->set_invalid();
  } 
}
  
		 
//manipulation states have to be able 
//to return possible grasps
//if the object is already in place, this should return an empty vector

//HELP!!!  I THINK DISTANCE IS BACKWARDS IN A LOT OF THE NEAREST NEIGHBOR STRUCTURES!!!
double darrt::DARRTStateSpace::distance(const ob::State *source, 
					const ob::State *destination) const {

  //max distance function in each subspace
  double maxdist = -1;
  const StateType *s = source->as<StateType>();
  const StateType *d = destination->as<StateType>();
  if (subspace_distance_ >= 0) {
    return getSubspace(subspace_distance_)->distance(s->components[subspace_distance_],
						     d->components[subspace_distance_]);
  }
  if (debug_level >= DDISTANCE) {
    ROS_INFO("Calculating distance from\n%s\nto\n%s",
	     state_string(s).c_str(), state_string(d).c_str());
  }
  for (unsigned int i = 0; i < getSubspaceCount(); i++) {
    double dist = getSubspace(i)->distance(s->components[i], 
					   d->components[i]);
    if (debug_level == DDISTANCE) {
      ROS_INFO("Distance in subspace %s is %f", getSubspace(i)->getName().c_str(), dist);
    }
    if (dist > maxdist) {
      maxdist = dist;
    }
  }
  //we add a small random component
  //because some of these may have equal distances
  //if the object distances count
  double ret = maxdist + rand()/(double)RAND_MAX*DIST_EPS;
  if (debug_level == DDISTANCE) {
    ROS_INFO("Returning %f", ret);
  }
  return ret;
}

bool darrt::DARRTStateSpace::equalStates(const ob::State *state1, 
					 const ob::State *state2) const {
  return robot_states_match(state1, state2) && 
    object_states_match(state1, state2);
}


bool darrt::DARRTStateSpace::near_states(const ob::State *state1, 
					 const ob::State *state2,
					 double deps, double aeps) const {
  if (subspace_distance_ >= 0) {
    if (subspace_distance_ == robot_index_) {
      return near_robot_states(state1, state2, deps, aeps);
    } else {
      return near_object_states(subspace_distance_, state1, state2,
				deps, aeps);
    }
  }
  
  for (unsigned int i = 0; i < componentCount_; i++) {
    if (static_cast<int>(i) == robot_index_) {
      if (!near_robot_states(state1, state2, deps, aeps)) {
	return false;
      }
    } else {
      if (!near_object_states(i, state1, state2, deps, aeps)) {
	return false;
      }
    }
  }
  return true;
}

bool darrt::DARRTStateSpace::satisfiesBounds(const ob::State *state) const {
  for (unsigned int i = 0; i < getSubspaceCount(); i++) {
    if (!getSubspace(i)->satisfiesBounds(get_state(i, state))) {
      return false;
    }
  }
  return true;
}


std::string darrt::DARRTStateSpace::state_string
(const ob::State *state) const {
  std::ostringstream str;
  str << "(";
  for (unsigned int i = 0; i < getSubspaceCount(); i++) {
    getSubspace(i)->printState(get_state(i, state), str);
  }

  str << ", Projection function: ";
  if (!state->as<StateType>()->getProjectionFunction()) {
    str << "None";
  } else {
    str << state->as<StateType>()->getProjectionFunction();
  }

  str << ", Allowed collisions:";
  const arm_navigation_msgs::OrderedCollisionOperations &ops =
    allowed_collisions(state);
  for (size_t i = 0; i < ops.collision_operations.size(); i++) {
    str << " (" << ops.collision_operations[i].object1 << ", " << 
      ops.collision_operations[i].object2 << ")";
  }
  str << ", Control: ";
  if (!state->as<StateType>()->control()) {
    str << "None";
  } else {
    str << state->as<StateType>()->control()->name();
  }
  str << ")";
  return str.str();
}

visualization_msgs::MarkerArray darrt::DARRTStateSpace::displayable_state
(const ob::State *state, std::string ns, int id,
 double scale, ColorPalate p, double alpha, bool minimal) const {

  visualization_msgs::MarkerArray marray;
  for (unsigned int i = 0; i < getSubspaceCount(); i++) {
    if (minimal && debug_level < DRRT && static_cast<int>(i) == robot_index()) {
      continue;
    }
    visualization_msgs::MarkerArray sd =
      dynamic_cast<const Displayable *>(getSubspace(i).get())->
      displayable_state(get_state(i, state), getSubspace(i)->getName()+"_"+ns, 
			id, scale, p, alpha, minimal);
    for (size_t j = 0; j < sd.markers.size(); j++) {
      marray.markers.push_back(sd.markers[j]);
    }
  }
  return marray;
}


ob::StateSpace *darrt::DARRTStateSpace::robot_state_space() {
  if (robot_index_ < 0 || 
      static_cast<unsigned int>(robot_index_) >= componentCount_) {
    ROS_ERROR("No robot state space!");
    return NULL;
  }
  return components_[robot_index_].get();
}

const ob::StateSpace *darrt::DARRTStateSpace::robot_state_space() const {
  if (robot_index_ < 0 || 
      static_cast<unsigned int>(robot_index_) >= componentCount_) {
    ROS_ERROR("No robot state space!");
    return NULL;
  }
  return components_[robot_index_].get();
}

ob::State *darrt::DARRTStateSpace::get_state(unsigned int ind, ob::State *state)
  const {
  return state->as<ob::CompoundState>()->components[ind];
}

const ob::State *darrt::DARRTStateSpace::get_state
(unsigned int ind, const ob::State *state) const {
  return state->as<ob::CompoundState>()->components[ind];
}

ob::State *darrt::DARRTStateSpace::robot_state(ob::State *state) const {
  return get_state(robot_index_, state);
}

const ob::State *darrt::DARRTStateSpace::robot_state
(const ob::State *state) const {
  return get_state(robot_index_, state);
}

double darrt::DARRTStateSpace::robot_distance
(const ob::State *destination, const ob::State *source) const {
  //by adding a small amount of randomness we ensure that identical states
  //(which happen when moving the base with the arm in the same position for example)
  //get sampled in random orders.  really, this should be randomized by the planner
  //but it's not
  return robot_state_space()->distance(robot_state(destination),
				       robot_state(source)) + rand()/(double)RAND_MAX*0.001;
}

bool darrt::DARRTStateSpace::robot_states_match
(const ob::State *s1, const ob::State *s2) const {
  return robot_state_space()->equalStates(robot_state(s1), robot_state(s2));
}

bool darrt::DARRTStateSpace::near_robot_states
(const ob::State *s1, const ob::State *s2, double deps, double aeps) const {
  return dynamic_cast<const Approximable *>(robot_state_space())->
    near_states(robot_state(s1), robot_state(s2), deps, aeps);
}

ob::StateSpace *darrt::DARRTStateSpace::object_state_space(unsigned int ind) {
  if (ind >= componentCount_) {
    ROS_ERROR("Unable to find state space %u!", ind);
    return NULL;
  }
  return components_[ind].get();
}

const ob::StateSpace *darrt::DARRTStateSpace::object_state_space
(unsigned int ind) const {
  if (ind >= componentCount_) {
    ROS_ERROR("Unable to find state space %u!", ind);
    return NULL;
  }
  return components_[ind].get();
}

ob::State *darrt::DARRTStateSpace::object_state
(unsigned int ind, ob::State *state) const {
  return get_state(ind, state);
}

const ob::State *darrt::DARRTStateSpace::object_state
(unsigned int ind, const ob::State *state) const {
  return get_state(ind, state);
}

bool darrt::DARRTStateSpace::object_states_match
(unsigned int ind, const ob::State *s1, const ob::State *s2) const {
  return object_state_space(ind)->equalStates(object_state(ind, s1), 
					      object_state(ind, s2));
}

bool darrt::DARRTStateSpace::object_states_match
(const ob::State *s1, const ob::State *s2) const {
  for (size_t i = 0; i < object_indexes_.size(); i++) {
    if (!object_states_match(object_indexes_[i], s1, s2)) {
      return false;
    }
  }
  return true;
}

bool darrt::DARRTStateSpace::near_object_states
(unsigned int ind, const ob::State *s1, const ob::State *s2, 
 double deps, double aeps) const {
  return dynamic_cast<const Approximable *>(object_state_space(ind))->
    near_states(object_state(ind,s1), object_state(ind,s2), deps, aeps);
}

bool darrt::DARRTStateSpace::near_object_states
(const ob::State *s1, const ob::State *s2, double deps, double aeps) const {
  if (subspace_distance_ >= 0 && subspace_distance_ != robot_index_) {
    return near_object_states(subspace_distance_, s1, s2, deps, aeps);
  }
  for (size_t i = 0; i < object_indexes_.size(); i++) {
    if (!near_object_states(object_indexes_[i], s1, s2, deps, aeps)) {
	return false;
    }
  }
  return true;
}


const an::OrderedCollisionOperations &darrt::
DARRTStateSpace::allowed_collisions(const ob::State *state) const {
  return state->as<DARRTStateSpace::StateType>()->allowed_collisions;
}

darrt::ThingType darrt::DARRTStateSpace::space_type(unsigned int ind) const {
  if (robot_index_ >= 0 && ind == static_cast<unsigned int>(robot_index_)) {
    return ROBOT;
  }
  for (size_t i = 0; i < object_indexes_.size(); i++) {
    if (ind == object_indexes_[i]) {
      return OBJECT;
    }
  }
  ROS_ERROR("Request for type for space %u which doesn't exist", ind);
  return UNKNOWN;
}


bool darrt::DARRTStateSpace::disable_object_collisions
(an::OrderedCollisionOperations &ops) const {
  for (size_t i = 0; i < object_indexes_.size(); i++) {
    an::CollisionOperation op;
    op.object1 = getSubspace(object_indexes_[i])->getName();
    op.object2 = op.COLLISION_SET_ALL;
    op.operation = op.DISABLE;
    ops.collision_operations.push_back(op);
    //re-enable collisions with robot
    op.object2 = robot_state_space()->getName();
    op.operation = op.ENABLE;
    ops.collision_operations.push_back(op);
  }
  return true;
}


bool darrt::DARRTStateSpace::disable_object_collisions(ob::State *state) const {
  //note that you can't for some unknown reason
  //use op.COLLISION_SET_OBJECT and op.COLLISION_SET_ALL
  //(it just won't work)
  return disable_object_collisions(state->as<StateType>()->allowed_collisions);
}

bool darrt::DARRTStateSpace::update_model
(const ob::State *new_state, EnvironmentInterface *env) const {
  //check the gravity constraint here because it requires that the robot's
  //space be active... with removal of that constraint can we move it
  //... oh no... by disabling active did we kill object solver?

  //Gravity constraint: the object must either be attached or 
  //sitting on something
  //moved to object's update model
  // for (unsigned int i = 0; i < object_indexes_.size(); i++) {
  //   unsigned int ind = object_indexes_[i];
  //   const ObjectStateSpace::StateType *ns = 
  //     new_state->as<StateType>()->as<ObjectStateSpace::StateType>(ind);
  //   if (!ns->attach_link.size() && !ns->support_surface) {
  //     if (debug_level >= DCOLLISIONS) {
  // 	ROS_INFO("Object state invalid because object is floating");
  //     }
  //     return false;
  //   }
  // }

  for (unsigned int i = 0; i < getSubspaceCount(); i++) {
    if (!dynamic_cast<const CollisionAwareStateSpace *>(getSubspace(i).get())->
	update_model(get_state(i, new_state), env)) {
      return false;
    }
  }
  return true;
}

void darrt::DARRTStateSpace::convert_ompl_state_to_darrt_state
(const ob::State *ompl_state, darrt_msgs::DARRTState &darrt_state) const {
  robot_state_space()->as<RobotStateSpace>()->
    convert_ompl_to_robot_state(robot_state(ompl_state), darrt_state.robot_state);
  
  for (size_t i = 0; i < object_indexes().size(); i++) {
    unsigned int ind = object_indexes().at(i);
    const ObjectStateSpace *ospace = 
      object_state_space(ind)->as<ObjectStateSpace>();
    const ObjectStateSpace::StateType *os = 
      get_state(ind, ompl_state)->as<ObjectStateSpace::StateType>();
    an::CollisionObject obj = ospace->object();
    gm::Pose trans;
    ospace->object_position(os, trans);
    for (unsigned int i = 0; i < obj.poses.size(); i++) {
      obj.poses[i] = transform_pose(obj.poses[i], trans);
    }
    if (os->attach_link.size()) {
      an::AttachedCollisionObject ao;
      ao.link_name = os->attach_link;
      for (unsigned int i = 0; i < obj.poses.size(); i++) {
	obj.poses[i] = transformer_.transform
	  (ao.link_name, obj.header.frame_id, obj.poses[i], 
	   darrt_state.robot_state);
      }
      obj.header.frame_id = ao.link_name;
      //note: assume ordered collisions covers touch-links
      ao.object = obj;
      darrt_state.attached_collision_objects.push_back(ao);
    } else {			
      darrt_state.collision_objects.push_back(obj);
    }
  }
  darrt_state.ordered_collisions = allowed_collisions(ompl_state);
  for (size_t i = 0; i < support_surfaces_.size(); i++) {
    darrt_state.support_surface_ids.push_back(support_surfaces_[i]->name());
  }
}

void darrt::DARRTStateSpace::convert_darrt_state_to_ompl_state
(const darrt_msgs::DARRTState &darrt_state, ob::State *ompl_state) const {
  robot_state_space()->as<RobotStateSpace>()->convert_robot_to_ompl_state
    (darrt_state.robot_state, robot_state(ompl_state));

  StateType *s = ompl_state->as<StateType>();

  for (size_t i = 0; i < darrt_state.collision_objects.size(); i++) {
    unsigned int ind;
    try {
      ind = getSubspaceIndex(darrt_state.collision_objects[i].id);
    } catch (ompl::Exception &e) {
      ROS_WARN("No subspace matching collision object %s",
	       darrt_state.collision_objects[i].id.c_str());
      continue;
    }
    const ObjectStateSpace *ospace = getSubspace(ind)->as<ObjectStateSpace>();
    ObjectStateSpace::StateType *os = 
      get_state(ind, ompl_state)->as<ObjectStateSpace::StateType>();
    an::CollisionObject obj = ospace->object();
    //we have to figure out the overall pose for the object
    //based on the poses in the message
    gm::Pose pose0 = transformer_.transform
      (obj.header.frame_id, 
       darrt_state.collision_objects[i].header.frame_id,
       darrt_state.collision_objects[i].poses[0], darrt_state.robot_state);
    gm::Pose origin;
    origin.orientation.w = 1.0;
    gm::Pose objpose = transform_pose(inverse_transform_pose(origin, obj.poses[0]), pose0);
    ospace->set_object_position(os, objpose);
    if (debug_level >= DPROPAGATE) {
      pause("New way of doing poses!  Make sure it WORKS!");
    }
    //find the support surface
    for (size_t j = 0; j < support_surfaces_.size(); j++) {
      if (support_surfaces_[j]->supporting(obj, objpose, 0.05)) {
	os->support_surface = support_surfaces_[j];
	an::CollisionOperation op;
	op.object1 = obj.id;
	op.object2 = support_surfaces_[j]->name();
	op.operation = op.DISABLE;
	s->add_allowed_collision(op);
	os->add_allowed_collision(op);
	break;
      }
    }
  }
  for (size_t i = 0; i < darrt_state.attached_collision_objects.size(); i++) {
    unsigned int ind;
    try {
      ind = getSubspaceIndex
	(darrt_state.attached_collision_objects[i].object.id);
    } catch (ompl::Exception &e) {
      ROS_WARN("No subspace matching attached collision object %s",
	       darrt_state.attached_collision_objects[i].object.id.c_str());
      continue;
    }
    const an::AttachedCollisionObject &ao = 
      darrt_state.attached_collision_objects[i];
    const ObjectStateSpace *ospace = getSubspace(ind)->as<ObjectStateSpace>();
    ObjectStateSpace::StateType *os = 
      get_state(ind, ompl_state)->as<ObjectStateSpace::StateType>();
    gm::Pose pose = transformer_.transform
      (ospace->object().header.frame_id, ao.object.header.frame_id, 
       ao.object.poses[0], darrt_state.robot_state);
    gm::Pose origin;
    origin.orientation.w = 1;
    ospace->set_object_position(get_state(ind, ompl_state), 
				transform_pose(inverse_transform_pose(origin, 
								      ospace->object().poses[0]),
					       pose));
    os->attach_link = ao.link_name;
    os->touch_links = ao.touch_links;
    an::CollisionOperation op;
    op.object1 = ao.object.id;
    op.operation = op.DISABLE;
    for (size_t j = 0; j < ao.touch_links.size(); j++) {
      op.object2 = ao.touch_links[j];
      s->add_allowed_collision(op);
      os->add_allowed_collision(op);
    }
  }
  ompl_state->as<DARRTStateSpace::StateType>()->add_allowed_collisions
    (darrt_state.ordered_collisions);
}


darrt::DARRTStateValidityChecker::
DARRTStateValidityChecker
(ompl::base::SpaceInformation *si, EnvironmentInterface *environment_interface,
 collision_space::EnvironmentModel::AllowedCollisionMatrix
 &default_allowed_collisions,
 const std::vector<ob::StateValidityCheckerPtr> &state_validity_checkers) :
  CollisionAwareStateValidityChecker(si, default_allowed_collisions),
  state_validity_checkers_(state_validity_checkers) {
  ros::NodeHandle n("~");
  collision_pub_ =
    n.advertise<visualization_msgs::Marker>("collisions", 1);
  space_ = si->getStateSpace()->as<DARRTStateSpace>();
  setup(environment_interface);
  for (size_t i = 0; i < state_validity_checkers_.size(); i++) {
    if (!state_validity_checkers_[i]) {
      continue;
    }
    dynamic_cast<CollisionAwareStateValidityChecker *>
      (state_validity_checkers_[i].get())->setup(environment_interface_);
  }
}

bool darrt::DARRTStateValidityChecker::isValid
(const ob::State *ompl_state) const {

  if (!ompl_state->as<ProjectibleState>()->valid()) {
    return false;
  }

  //could add in allowed collisions between objects and their
  //support surfaces but... eh

  if (!reconcile_collision_model(ompl_state)) {
    if (debug_level >= DRRT) {
      ROS_INFO("Invalid state because of a constraint violation (bounds or gravity)");
    }
    //weren't in bounds
    return false;
  }
  ROS_DEBUG("Returned from reconciling the collision model");
  bool return_value = true;

  ROS_DEBUG("State is %s", space_->state_string(ompl_state).c_str());


  //check if the objects in the environment are in collision
  //this is usually faster so do it first
  for (size_t i = 0; i < space_->object_indexes().size(); i++) {
    unsigned int ind = space_->object_indexes().at(i);
    ObjectStateValidityChecker *ovc = dynamic_cast<ObjectStateValidityChecker *>
      (state_validity_checkers_[ind].get());
    if (ovc) {
      ovc->setCheckRobotCollisions(true);
      if (!state_validity_checkers_[ind]->isValid
	  (space_->get_state(ind, ompl_state))) {
	return_value = false;
	ovc->setCheckRobotCollisions(true);
	break;
      }
      ovc->setCheckRobotCollisions(true);
    }
  }

  if (return_value && state_validity_checkers_[space_->robot_index()] &&
      space_->robot_index() >= 0) {
    return_value = state_validity_checkers_[space_->robot_index()]->
      isValid(space_->get_state(space_->robot_index(), ompl_state));
  }

  if (!return_value) {
    ROS_DEBUG("State %s is not valid", 
	      space_->state_string(ompl_state).c_str());
  }

  ROS_DEBUG("Reverting collision model");
  if (!revert_collision_model()) {
    ROS_ERROR("Unable to revert collision model after checking state validity");
  }
  ROS_DEBUG("Reverted collision model");
  if (debug_level == DCOLLISIONS && 
      ompl_state->as<DARRTStateSpace::StateType>()->
      allowed_collisions.collision_operations.size()) {
    ob::State *copy = space_->allocState();
    space_->copyState(copy, ompl_state);
    copy->as<DARRTStateSpace::StateType>()->
      allowed_collisions.collision_operations.clear();
    ROS_INFO("Calling state validity checker AGAIN for state without collisions: %s", space_->state_string(copy).c_str());
    bool cv = isValid(copy);
    ROS_INFO("Without allowed collisions, is valid: %d", cv);
  }
  return return_value;
}

darrt::ProjectibleStateSampler::ProjectibleStateSampler
(const ob::StateSpace *space, 
 const std::vector< std::pair<DARRTProjectionFunction, double> > &functions, 
 ob::GoalSampleableRegion *goal) : ob::CompoundStateSampler(space) {
  
  sample_ = 0;
  goal_ = goal;
  //turn to true if yuo want the goal to be sampled as the first state!
  first_sample_ = false;
  functions_ = functions;
  //normalize the weights
  double total = 0.0;
  for (unsigned int i = 0; i < functions_.size(); i++) {
    total += functions_[i].second;
  }
  for (unsigned int i = 0; i < functions_.size(); i++) {
    functions_[i].second /= total;
  }
  
}

void darrt::ProjectibleStateSampler::sampleUniform
(ob::State *state) {

  if (goal_ && first_sample_) {
    if (debug_level >= DRRT) {
      ROS_INFO("First sample, sampling goal state");
    }
    goal_->sampleGoal(state);
    first_sample_ = false;
    return;
  }

  ProjectibleState *s = state->as<ProjectibleState>();
  s->set_valid();

  //really should use state samplers at the level below - this is bad

  for (unsigned int i = 0; i < space_->as<ob::CompoundStateSpace>()->getSubspaceCount(); i++) {
    ObjectStateSpace::StateType *os = 
      dynamic_cast<ObjectStateSpace::StateType *>(s->components[i]);
    if (os) {
      os->attach_link = "";
      os->support_surface = NULL;
      os->set_control(NULL, "");
      os->set_allowed_collisions(an::OrderedCollisionOperations());
    }
  }

  CollisionAwareState *cs = dynamic_cast<CollisionAwareState *>(state);
  if (cs) {
    cs->set_control(NULL, "");
    cs->set_allowed_collisions(an::OrderedCollisionOperations());
  }

  //sample a random state
  for (unsigned int i = 0; i < space_->as<ob::CompoundStateSpace>()->getSubspaceCount(); i++) {
    samplers_[i]->sampleUniform(s->components[i]);
  }

  //choose projection function
  double r = rand()/(double)RAND_MAX;
  if (debug_level >= DPROPAGATE) {
    //r = 0.4; 0.0-0.25 = transit, 0.25-0.50 = rigid transfer, 
    //0.50-0.75 = push, 0.75-1.0 = spatula transfer
    //pause("Not randomizing projection function!");
    ROS_INFO("projection r = %f", r);
  }

  double total = 0.0;
  unsigned int function;
  for (function = 0; function < functions_.size(); function++) {
    total += functions_[function].second;
    if (r <= total) {
      break;
    }
  }
  
  s->setProjectionFunction(functions_[function].first);
}
