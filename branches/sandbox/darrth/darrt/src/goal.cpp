#include "darrt/robot_space.hh"
#include "darrt/goal.hh"
#include "darrt/space.hh"
#include "darrt/control.hh"
#include "darrt/ompl_ros_conversions.h"
#include "darrt/object_space.hh"
#include "darrt/transform_ros_types.hh"

#include <planning_environment/models/model_utils.h>

#include <ros/ros.h>

void darrt::goalProjectionFunction(const ob::State *from, ob::State *sample) {}

void darrt::Goal::copyInfo(const darrt::Goal &goal) {
  objects = goal.objects;
  primitives = goal.primitives;
  params = goal.params;
  robot_groups = goal.robot_groups;
}

ob::Goal *darrt::Goal::satisfiable_goal(const oc::SpaceInformationPtr &si,
					const ob::State *starting_state) const {
  const SingleGoal *prim_goal = NULL;
  for (const_iterator it = begin(); it != end(); it++) {
    if (it->second.type == SingleGoal::PRIMITIVE) {
      prim_goal = &(it->second);
      break;
    }
  }
  if (prim_goal) {
    ROS_INFO("PRIMTIVE GOAL!");
    if (size() > 1) {
      ROS_ERROR("Primitive AND compound goal is not possible!");
    }
    return new PrimitiveGoal(si, prim_goal->primitive, starting_state,
			     prim_goal->primitive_actual_goal);
  }
  return new CompoundGoal(si, *this, starting_state);
} 

ob::Goal *darrt::SingleGoal::satisfiable_goal
(const ob::SpaceInformationPtr &si) const {
  const ObjectStateSpace *ospace = si->getStateSpace()->as<ObjectStateSpace>();
  switch(type) {
  case POSE:
    {
      ob::State *goal_state = si->allocState();
      ospace->set_object_position(goal_state, pose);
      ob::GoalState *sg = new ob::GoalState(si);
      sg->setState(goal_state);
      ROS_INFO("Setting single state threshold to %f", threshold);
      sg->setThreshold(threshold);
      return sg;
    }
  case POSES:
    {
      RandomGoalStates *sg = new RandomGoalStates(si);
      ROS_INFO("Setting multiple state threshold to %f", threshold);
      sg->setThreshold(threshold);
      for (size_t i = 0; i < poses.size(); i++) {
	//not sure if these ever get deleted...
	ob::State *goal_state = si->allocState();
	ospace->set_object_position(goal_state, poses[i]);
	sg->addState(goal_state);
      }
      return sg;
    }
  case ROTATIONALLY_SYMMETRIC_POSE:
    {
      RotationallySymmetricGoalState *rsgs = new RotationallySymmetricGoalState(si, pose);
      ROS_INFO("Setting rotationally symmetric state threshold to %f", threshold);
      rsgs->setThreshold(threshold);
      return rsgs;
    }
  case ROTATIONALLY_SYMMETRIC_POSES:
    {
      RotationallySymmetricGoalStates *rsgs = new RotationallySymmetricGoalStates(si, poses);
      ROS_INFO("Setting rotationally symmetric states threshold to %f", threshold);
      rsgs->setThreshold(threshold);
      return rsgs;
    }
  case PRIMITIVE:
    {
      ROS_ERROR("Primitive is overarching goal - cannot be called from single goal satisfiable!");
      return NULL;
    }
  case SHAPE:
    return new GoalShape(si, shape);
  case BORDER:
    return new GoalBorder(si, border);
  default:
    ROS_ERROR("Unknown type %d for single sampleable goal", type);
    return NULL;
  }
}

visualization_msgs::MarkerArray darrt::SingleGoal::displayable
(std::string frame_id) const {
  visualization_msgs::MarkerArray marray;
  switch(type) {
  case ROTATIONALLY_SYMMETRIC_POSE:
  case POSE:
    {
      gm::PoseStamped ps;
      ps.header.frame_id = frame_id;
      ps.pose = pose;
      marray.markers.push_back
	(make_marker(ps, visualization_msgs::Marker::SPHERE,
		     0.02, 0, 1.0, 0, 0.5, "goal_states", 0));
      return marray;
    }
  case ROTATIONALLY_SYMMETRIC_POSES:
  case POSES:
    {
      for (unsigned int i = 0; i < poses.size(); i++) {
	gm::PoseStamped ps;
	ps.header.frame_id = frame_id;
	ps.pose = poses[i];
	marray.markers.push_back
	  (make_marker(ps, visualization_msgs::Marker::SPHERE,
		       0.02, 0, 1.0, 0, 0.5, "goal_states", i));
      }
      return marray;
    }
  case SHAPE:
    {
      gm::PoseStamped ps;
      ps.header.frame_id = frame_id;
      ps.pose = shape.pose;
      visualization_msgs::Marker marker;
      marker.header.frame_id = frame_id;
      marker.ns = "goal_states";
      marker.id = 0;
      marker.action = marker.ADD;
      marker.pose = shape.pose;
      marker.color.g = 1.0;
      planning_environment::setMarkerShapeFromShape(shape.shape, marker);
      marray.markers.push_back(marker);
      return marray;
    }
  case BORDER:
    {
      for (size_t i = 0; i < border.size(); i++) {
	gm::PoseStamped ps;
	ps.header.frame_id = frame_id;
	ps.pose = border[i].pose;
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame_id;
	marker.ns = "goal_states";
	marker.id = i;
	marker.action = marker.ADD;
	marker.pose = shape.pose;
	marker.color.g = 1.0;
	planning_environment::setMarkerShapeFromShape(border[i].shape, marker);
	marray.markers.push_back(marker);
      }
      return marray;
    }
  default:
    ROS_ERROR("Unable to display unknown goal type %d", type);
    return marray;
  }
}
		
	


darrt::RandomGoalStates::RandomGoalStates(const ob::SpaceInformationPtr &si) :
  ob::GoalStates(si) {
  ros::NodeHandle n("~");
  viz_ = n.advertise<visualization_msgs::MarkerArray>("robot_state_markers", 1);
}	  

void darrt::RandomGoalStates::sampleGoal(ob::State *state) const {
  if (states_.empty()) {
    throw ompl::Exception("No goals to sample in RandomGoalStates");
  }
  unsigned int r = rand() % states_.size();
  si_->copyState(state, states_[r]);
}

void darrt::RandomGoalStates::display(std::string ns, int id, double r, 
				      double g, double b, double a) const {
  const ObjectStateSpace *ospace = si_->getStateSpace()->as<ObjectStateSpace>();
  visualization_msgs::MarkerArray marray;
  for (unsigned int g = 0; g < states_.size(); g++) {
    geometry_msgs::PoseStamped ps;
    ospace->object_position(states_[g], ps.pose);
    ps.header.frame_id = ospace->object().header.frame_id;
    marray.markers.push_back
      (make_marker(ps, visualization_msgs::Marker::SPHERE,
		   0.02, 0, 1.0, 0, 0.5, "goal_states", g+id));
  }
  viz_.publish(marray);
}

double darrt::RotationallySymmetricGoalState::distanceGoal(const ob::State *st) const {
  const ObjectStateSpace *ospace = si_->getStateSpace()->as<ObjectStateSpace>();
  gm::Pose objpose;
  ospace->object_position(st, objpose);
  double dist = darrt::distance(objpose.position, pose_.position);
  return dist;
}

void darrt::RotationallySymmetricGoalState::sampleGoal(ob::State *st) const {
  const ObjectStateSpace *ospace = si_->getStateSpace()->as<ObjectStateSpace>();
  gm::Pose objpose = pose_;
  double yaw = rand()/((double)RAND_MAX)*2.0*MATH_PI;
  apply_yaw_to_quaternion(yaw, pose_.orientation, objpose.orientation);
  //objpose.orientation.z = sin(angle/2.0);
  //obj.pose.orientation.w = cos(angle.2.0);
  //ROS_INFO_STREAM("Setting goal pose to\n" << objpose);
  ospace->set_object_position(st, objpose);
}

darrt::RotationallySymmetricGoalStates::RotationallySymmetricGoalStates
(const ob::SpaceInformationPtr &si, const std::vector<gm::Pose> &poses) : 
  ob::GoalSampleableRegion(si) {
  for (unsigned int i = 0; i < poses.size(); i++) {
    states_.push_back(new RotationallySymmetricGoalState(si, poses[i]));
  }
}

darrt::RotationallySymmetricGoalStates::~RotationallySymmetricGoalStates() { 
 for (unsigned int i = 0; i < states_.size(); i++) {
    delete states_[i];
  }
}

double darrt::RotationallySymmetricGoalStates::distanceGoal(const ob::State *st) const {
  double mindist = MATH_INF;
  for (unsigned int i = 0; i < states_.size(); i++) {
    double dist = states_[i]->distanceGoal(st);
    if (dist < mindist) {
      mindist = dist;
    }
  }
  return mindist;
}
 
 void darrt::RotationallySymmetricGoalStates::sampleGoal(ob::State *st) const {
  unsigned int r = rand() % states_.size();
  states_[r]->sampleGoal(st);
}

//really you're looking for a path that has the same transition
//in it... but we only get a single state
//so for now we just look for a single primitive
darrt::PrimitiveGoal::PrimitiveGoal(const oc::SpaceInformationPtr &si,
				    const Primitive *prim,
				    const ob::State *starting_state,
				    const darrt::Goal *actual_goal) :
  ob::GoalSampleableRegion(si), siC_(si) {
  prim_ = prim;
  starting_state_ = NULL;
  actual_goal_ = NULL;
  setStartingState(starting_state);
  setActualGoal(actual_goal);
  if (!prim_) {
    ROS_ERROR("Primitive goal with NULL primitive!  This is not satisfiable!");
  }
  state_sampler_ = siC_->allocStateSampler();
  control_sampler_ = siC_->allocDirectedControlSampler();
  sampled_state_ = siC_->allocState();
  sampled_control_ = siC_->allocControl();
  propagator_ = siC_->getStatePropagator();

  robot_index_ = -1;
  base_index_ = -1;
  base_space_ = NULL;
  const ob::CompoundStateSpace *cmpd_space = 
    dynamic_cast<const ob::CompoundStateSpace *>(siC_->getStateSpace().get());
  if (cmpd_space) {
    for (unsigned int i = 0; i < cmpd_space->getSubspaceCount(); i++) {
	const CompoundRobotStateSpace *crs = 
	  dynamic_cast<const CompoundRobotStateSpace *>(cmpd_space->getSubspace(i).get());
	if (crs) {
	  robot_index_ = i;
	  for (unsigned int j = 0; j < crs->getSubspaceCount(); j++) {
	    base_space_ = dynamic_cast<const RobotBaseStateSpace *>(crs->getSubspace(j).get());
	    if (base_space_) {
	      base_index_ = j;
	      break;
	    }
	  }
	  break;
	}
    }
  }
  if (base_index_ < 0) {
    pause("Unable to find base index when creating primitive goal!");
  }	
}
 
void darrt::PrimitiveGoal::setActualGoal(const darrt::Goal *actual_goal) {
  if (!actual_goal) {
    actual_goal_ = NULL;
    return;
  }
  ob::Goal *sat_goal = actual_goal->satisfiable_goal(siC_, starting_state_);
  actual_goal_ = dynamic_cast<ob::GoalSampleableRegion *>(sat_goal);
  if (!actual_goal_) {
    delete sat_goal;
  }
}

bool darrt::PrimitiveGoal::isSatisfied(const ob::State *st, double *distance) const {
  if (!prim_) {
    return false;
  }
  const CollisionAwareState *cs = dynamic_cast<const CollisionAwareState *>
    (st);
  if (!cs) {
    ROS_ERROR("Called is satisfied for primitive goal on a state not guaranteed to have a control");
    return false;
  }
  if (!cs->control()) {
    return false;
  }
  if (debug_level >= DDISTANCE) {
    ROS_INFO("Checking if %s and desired %s match", cs->control()->name().c_str(), prim_->name().c_str());
  }
  bool matches = prim_->matches(cs->control(), st);
  if (!distance) {
    return matches;
  }
  if (matches) {
    *distance = 0;
  } else {
    *distance = siC_->getStateSpace()->getMaximumExtent();
  }
  return matches;
}

void darrt::PrimitiveGoal::sampleGoal(ob::State *state) const {
  ROS_INFO("Sampling a primitive goal for %s!", prim_->str().c_str());
  unsigned int iter = 0;
  while (true) {
    int actual_debug_level = debug_level;
    if (actual_debug_level >= DRRT) {
      debug_level = DMINIMAL;
    } else {
      debug_level = DSILENT;
    }
    state_sampler_->sampleUniform(sampled_state_);
    if (actual_goal_ && (iter == 0 || rand() % 5)) {
      actual_goal_->sampleGoal(sampled_state_);
    }
    control_sampler_->sampleTo(sampled_control_, starting_state_, sampled_state_);
    DARRTControlSpace::ControlType *sc = dynamic_cast<DARRTControlSpace::ControlType *>(sampled_control_);
    PIList &clist = sc->clist_;
    bool found_prim = false;
    unsigned int starting_turn = 0;
    unsigned int total_turns = 0;
    for (unsigned int i = 0; i < clist.size(); i++) {
      total_turns += clist[i]->turns();
    }
    for (unsigned int i = 0; i < clist.size(); i++) {
      if (actual_debug_level >= DRRT) {
	ROS_INFO("Primitive %d = %s", i, clist[i]->str().c_str());
      }
      if (prim_->matches(clist[i]->primitive(), clist[i]->source())) {
	//if (!siC_->isValid(clist[i]->source())) {
	//try propagating...
	*(sc->curr_turn) = starting_turn;
	if (starting_turn + 2 >= total_turns) {
	  break;
	}
	propagator_->propagate(clist[i]->source(), sampled_control_, siC_->getPropagationStepSize(), sampled_state_);
	propagator_->propagate(sampled_state_, sampled_control_, siC_->getPropagationStepSize(), state);
	//make sure we haven't found a very short primitive!
	const CollisionAwareState *cas = dynamic_cast<const CollisionAwareState *>(state);
	if (!prim_->matches(cas->control(), state)) {
	  pause("Primitive no longer matches when propagating for goal "
		+prim_->name()+".  Turns are "+makestring(i)+", current primitive is "
		+cas->control()->name()+", control is "+ 
		siC_->getControlSpace()->as<DARRTControlSpace>()->control_string(sampled_control_));
	} else if (siC_->isValid(state)) {
	  found_prim = true;
	  if (actual_debug_level >= DMINIMAL) {
	    dynamic_cast<const Displayable *>(siC_->getStateSpace().get())
	      ->display_state(state, "goal_sample", 0, 1, Displayable::GOALPALATE);
	  }
	  if (actual_debug_level == DRRT) {
	    ROS_INFO("For goal %s, returning goal state %s", 
		     prim_->name().c_str(),
		     siC_->getStateSpace()->as<DARRTStateSpace>()->state_string(state).c_str());
	  } 
	  if (actual_debug_level >= DPROPAGATE) {
	    //show this state
	    dynamic_cast<const Displayable *>(siC_->getStateSpace().get())
	      ->display_state(state, "goal_sample", 0, 1, Displayable::GOALPALATE);
	    pause("Returning goal state "+
		  siC_->getStateSpace()->as<DARRTStateSpace>()->state_string(state), 0);
	    
	  }
	} else {
	  ROS_INFO("Goal sampling: No collision free state in primitive");
	}
	break;
      }
      starting_turn += clist[i]->turns();
    }
    siC_->getControlSpace()->as<DARRTControlSpace>()->freeControlData(sampled_control_);
    debug_level = actual_debug_level;
    if (found_prim) {
      return;
    }
    ROS_INFO("Goal sampling: Sampled path didn't include goal");
  }
}

void darrt::PrimitiveGoal::print(std::ostream &out) const {
  out << "Goal: " << prim_->str();
}

darrt::PrimitiveGoal::~PrimitiveGoal() {
  siC_->freeControl(sampled_control_);
  siC_->freeState(sampled_state_);
  if (actual_goal_) {
    delete actual_goal_;
  }
}

darrt::GoalShape::GoalShape(const ob::SpaceInformationPtr &si,
			    const Body &body) :
  GoalSampleableRegion(si), body_(body) {
  ros::NodeHandle n("~");
  viz_ = n.advertise<visualization_msgs::MarkerArray>("robot_state_markers", 1);
}

void darrt::GoalShape::sampleGoal(ob::State *state) const {
  gm::Point spt = sample();
  //convert it into the pose
  gm::Point pt = transform_point(spt, body_.pose);
  
  //now convert it into the space
  //and sample randomly for everything else
  ompl_ros_conversions::convert_point_to_state
    (state, si_->getStateSpace().get(), pt);
}

gm::Point darrt::GoalShape::sample() const {
  
  gm::Point spt;

  switch(body_.shape.type) {
  case an::Shape::SPHERE:
    {
      double radius = body_.shape.dimensions[0];
      spt.x = rand()/(double)RAND_MAX*2.0*radius - radius;
      double rem = sqrt(radius*radius - spt.x*spt.x);
      spt.y = rand()/(double)RAND_MAX*2.0*rem - rem;
      rem = sqrt(radius*radius - spt.x*spt.x - spt.y*spt.y);
      spt.z = rand()/(double)RAND_MAX*2.0*rem - rem;
      break;
    }
  case an::Shape::BOX:
    {
      spt.x = rand()/(double)RAND_MAX*body_.shape.dimensions[0]
	- body_.shape.dimensions[0]/2.0;
      spt.y = rand()/(double)RAND_MAX*body_.shape.dimensions[1]
	- body_.shape.dimensions[1]/2.0;
      spt.z = rand()/(double)RAND_MAX*body_.shape.dimensions[2]
	- body_.shape.dimensions[2]/2.0;
      break;
    }
  case an::Shape::CYLINDER:
    {
      double radius = body_.shape.dimensions[0];
      spt.x = rand()/(double)RAND_MAX*2.0*radius - radius;
      double rem = sqrt(radius*radius - spt.x*spt.x);
      spt.y = rand()/(double)RAND_MAX*2.0*rem - rem;
      spt.z = rand()/(double)RAND_MAX*body_.shape.dimensions[1]
	- body_.shape.dimensions[1]/2.0;
      break;
    }
  case an::Shape::MESH:
    pause("Sampling from mesh shape not supported.");
    break;
  default:
    pause("Goal shape is not a recognized type.");
    break;
  }
  return spt;

}

bool darrt::GoalShape::isSatisfied(const ob::State *state,
				   double *distance) const {
  
  gm::Point opt = body_.pose.position;
  if (!ompl_ros_conversions::convert_state_to_point
      (state, si_->getStateSpace().get(), opt)) {
    ROS_ERROR("Unable to convert state to point!");
    return false;
  }
  
  gm::Point pt = inverse_transform_point(opt, body_.pose);
  bool retval;

  switch(body_.shape.type) {
  case an::Shape::BOX:
    retval = ((fabs(body_.shape.dimensions[0]) < EPSILON || 
	       (fabs(pt.x)<=body_.shape.dimensions[0]/2.0)) &&
	      (fabs(body_.shape.dimensions[1]) < EPSILON || 
	       (fabs(pt.y)<=body_.shape.dimensions[1]/2.0)) && 
	      (fabs(body_.shape.dimensions[2]) < EPSILON || 
	       (fabs(pt.z)<=body_.shape.dimensions[2]/2.0)));
    break;
  case an::Shape::SPHERE:
    retval = sqrt(pt.x*pt.x + pt.y*pt.y + pt.z *pt.z) <=
      body_.shape.dimensions[0];
    break;
  case an::Shape::CYLINDER:
    retval = ((fabs(body_.shape.dimensions[0]) < EPSILON ||
	       sqrt(pt.x*pt.x + pt.y*pt.y) > body_.shape.dimensions[0]) &&
	      (fabs(body_.shape.dimensions[1]) < EPSILON ||
	       fabs(pt.z) < body_.shape.dimensions[1]/2.0));
    break;
  case an::Shape::MESH:
    pause("Mesh goal not supported");
    retval = false;
    break;
  default:
    pause("Unknown goal shape!");
    retval = false;
    break;
  }
    
  if (distance) {
    if (retval) {
      *distance = 0.0;
    } else {
      *distance = si_.get()->getStateSpace().get()->getMaximumExtent();
    }
  }
  return retval;
}

void darrt::GoalShape::display(std::string ns, int id, double r,
			       double g, double b, double a) const {

  //display a marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = 
    si_->getStateSpace()->as<ObjectStateSpace>()->object().header.frame_id;
  marker.header.stamp = ros::Time(0);
  marker.ns = ns;
  marker.id = id;
  marker.action = marker.ADD;
  marker.pose = body_.pose;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  

  ROS_INFO("Shape is %d, id %d, pose is (%f, %f, %f, %f, %f, %f, %f)",
	   body_.shape.type, id, body_.pose.position.x,
	   body_.pose.position.y, body_.pose.position.z,
	   body_.pose.orientation.x, body_.pose.orientation.y,
	   body_.pose.orientation.z, body_.pose.orientation.w);

  planning_environment::setMarkerShapeFromShape(body_.shape, marker);

  visualization_msgs::MarkerArray marray;
  marray.markers.push_back(marker);
  viz_.publish(marray);
}

darrt::GoalBorder::GoalBorder(const ob::SpaceInformationPtr &si,
			      const Border &border) :
  GoalSampleableRegion(si) {

  ros::NodeHandle n("~");
  viz_ = n.advertise<visualization_msgs::MarkerArray>("robot_state_markers", 1);

  samplers_.resize(border.size(), NULL);
  for (size_t i = 0; i < border.size(); i++) {
    samplers_[i] = new GoalShape(si, border[i]);
  }
}

darrt::GoalBorder::~GoalBorder() {
  for (size_t i = 0; i < samplers_.size(); i++) {
    delete samplers_[i];
  }
}

void darrt::GoalBorder::sampleGoal(ob::State *state) const {
  if (samplers_.size() == 0) {
    pause("Goal area of no size!");
  }

  unsigned int r = rand() % samplers_.size();
  samplers_[r]->sampleGoal(state);
}

bool darrt::GoalBorder::isSatisfied(const ob::State *state,
				    double *distance) const {
  double mindist = si_.get()->getStateSpace().get()->getMaximumExtent();
  for (size_t i = 0; i < samplers_.size(); i++) {
    double dist;
    if (samplers_[i]->isSatisfied(state, &dist)) {
      if (distance) {
	*distance = dist;
      }
      return true;
    }
    if (dist < mindist) {
      mindist = dist;
    }
  }
  if (*distance) {
    *distance = mindist;
  }
  return false;
}

void darrt::GoalBorder::display(std::string ns, int id, 
				double r, double g, double b, double a) const {
  int curr_id = id;
  for (size_t i = 0; i < samplers_.size(); i++) {
   samplers_[i]->display(ns, curr_id, r, g, b, a);
    curr_id++;
  }
}

darrt::CompoundGoal::CompoundGoal(const oc::SpaceInformationPtr &si, 
				  const darrt::Goal &goal,
				  const ob::State *starting_state) :
  ob::GoalSampleableRegion(si), siC_(si) {
  ros::NodeHandle n("~");
  viz_ = n.advertise<visualization_msgs::MarkerArray>("robot_state_markers", 1);

  max_samples_ = 0;
  bool can_sample = true;
  
  state_sampler_ = si->allocStateSampler();
  starting_state_ = starting_state;
  control_sampler_ = siC_->allocDirectedControlSampler();
  sampled_control_ = siC_->allocControl();

  for (darrt::Goal::const_iterator it = goal.begin(); it != goal.end(); it++) {
    std::string subspace_name = it->first;
    if (!subspace_name.compare("robot")) {
      ROS_ERROR("Robot goals currently unimplemented!");
      continue;
    }

    const DARRTStateSpace *space = 
      si->getStateSpace()->as<DARRTStateSpace>();

    unsigned int index;
    try {
      index = space->getSubspaceIndex(subspace_name);
    } catch (ompl::Exception &e) {
      ROS_ERROR("Trying to set goal for object %s, but this object is not in the state space", 
		it->first.c_str());
      continue;
    }

    const ob::StateSpacePtr &obj_space = 
      si->getStateSpace()->as<ob::CompoundStateSpace>()->getSubspace(index);
    obj_space->as<ObjectStateSpace>()->setBounds(it->second.bounds);
    //this may cause a seg fault!  check your shared pointers
    ob::SpaceInformationPtr 
      osi_ptr(new ob::SpaceInformation(obj_space));
    
    ob::Goal *subgoal = it->second.satisfiable_goal(osi_ptr);    
    if (subgoal) {
      goals_[index] = subgoal;
    } else {
      ROS_ERROR("Error getting goal for space %s", it->first.c_str());
      continue;
    }
    active_spaces_.push_back(index);
    ob::GoalSampleableRegion *gsr = dynamic_cast<ob::GoalSampleableRegion *>(subgoal);
    if (!gsr) {
      can_sample = false;
      continue;
    }
    sampleable_spaces_.push_back(index);
    if (gsr->maxSampleCount() > max_samples_) {
      max_samples_ = gsr->maxSampleCount();
    }
  }
  if (!can_sample) {
    max_samples_ = 0;
  }
}

darrt::CompoundGoal::~CompoundGoal() {
  siC_->freeControl(sampled_control_);
  for (std::map<unsigned int, ob::Goal *>::iterator it =
	 goals_.begin(); it != goals_.end(); it++) {
    if (it->second) {
      delete it->second;
    }
  }
}

void darrt::CompoundGoal::sampleGoal(ob::State *state) const {
  if (debug_level >= DMINIMAL) {
    ROS_INFO("Sampling a goal state!");
  }
  state_sampler_->sampleUniform(state);
  DARRTStateSpace *space = 
    si_->getStateSpace()->as<DARRTStateSpace>();
  DARRTStateSpace::StateType *s = state->as<DARRTStateSpace::StateType>();
  s->setProjectionFunction(&goalProjectionFunction);

  
  while (true) {
    for (unsigned int i = 0; i < sampleable_spaces_.size(); i++) {
      
      ob::GoalSampleableRegion *gsr = dynamic_cast<ob::GoalSampleableRegion *>
	(goals_.at(sampleable_spaces_[i]));
      gsr->sampleGoal(s->components[sampleable_spaces_[i]]);
    }
    if (active_spaces_.size() != space->getSubspaceCount()) {
      //sample from the starting state to the goal state
      int actual_debug_level = debug_level;
      debug_level = DMINIMAL;
      (dynamic_cast<DARRTDirectedControlSampler *>(control_sampler_.get()))->
	sampleDirectlyTo(sampled_control_, starting_state_, state);
      debug_level = actual_debug_level;
      PIList clist = dynamic_cast<DARRTControlSpace::ControlType *>(sampled_control_)->clist_;
      const DARRTControlSpace *cspace = siC_->getControlSpace()->as<DARRTControlSpace>();
      const ob::CompoundState *last_state = NULL;
      for (int j = clist.size()-1; j >= 0; j--) {
	if (!dynamic_cast<const InvalidPrimitiveInstance *>(clist[j])) {
	  last_state = clist[j]->destination()->as<ob::CompoundState>();
	  break;
	}
      }
      if (!last_state) {
	cspace->freeControlData(sampled_control_);
	continue;
      }
      bool satisfies = true;
      for (unsigned int j = 0; j < active_spaces_.size(); j++) {
	if (debug_level >= DPROPAGATE) {
	  ROS_INFO("Checking satisfiability for goal %d", active_spaces_[j]);
	}
	double dist;
	if (!goals_.at(active_spaces_[j])->isSatisfied
	    (last_state->components[active_spaces_[j]], &dist)) {
	  ROS_INFO("distance = %f", dist);
	  satisfies = false;
	  break;
	}
      }
      if (satisfies) {
	if (siC_->isValid(last_state)) {
	  space->copyState(state, last_state);
	} else {
	  satisfies = false;
	}
      }
      cspace->freeControlData(sampled_control_);
      if (satisfies) {
	if (debug_level >= DRRT) {
	  ROS_INFO("Returning %s", space->state_string(state).c_str());
	}
	return;
      }
    } else {
      if (debug_level >= DRRT) {
	ROS_INFO("Returning %s", space->state_string(state).c_str());
      }
      return;
    }
  }
}

bool darrt::CompoundGoal::isSatisfied(const ob::State *state,
				      double *distance) const {

  if (debug_level >= DDISTANCE) {
    ROS_INFO("Checking if goal is satisfied");
  }
  DARRTStateSpace *space = 
    si_.get()->getStateSpace().get()->as<DARRTStateSpace>();
  //ROS_INFO("Checking %s", space->state_string(state).c_str());
  const DARRTStateSpace::StateType *s = state->as<DARRTStateSpace::StateType>();

  double fulldist = 0;
  bool retval = true;
  for (unsigned int i = 0; i < space->getSubspaceCount(); i++) {
    std::map<unsigned int, ob::Goal *>::const_iterator git =
      goals_.find(i);
    if (git == goals_.end()) {
      continue;
    }
    double dist;
    if (!git->second->isSatisfied(s->components[i], &dist)) {
      if (debug_level >= DDISTANCE) {
	ROS_INFO("Checking if goal in subspace %u is satisfied.  Distance is %f",
		 i, dist);
      }
      if (!distance) {
	return false;
      }
      retval = false;
    }
    fulldist += dist;
  }
  if (distance) {
    *distance = fulldist;
  }
  return retval;
}

void darrt::CompoundGoal::display(std::string ns, Displayable::ColorPalate palate,
				  double a) const {

  float r, g, b;
  Displayable::interpret_palate(palate, r, g, b);
  int id = 0;
  ROS_INFO("Displaying goal state");
  for (std::map<unsigned int, ob::Goal *>::const_iterator
	 it = goals_.begin(); it != goals_.end(); it++) {
    const GoalShape *gs = dynamic_cast<const GoalShape *>(it->second);
    if (gs) {
      gs->display(ns, id, r, g, b, a);
      id++;
      continue;
    }
    const GoalBorder *gb = dynamic_cast<const GoalBorder *>(it->second);
    if (gb) {
      gb->display(ns, id, r, g, b, a);
      id += gb->num_shapes();
      continue;
    }
    const RandomGoalStates *gl = 
      dynamic_cast<const RandomGoalStates *>(it->second);
    if (gl) {
      gl->display(ns, id, r, g, b, a);
      id += gl->getStateCount();
      continue;
    }
    const ob::GoalState *gt = dynamic_cast<const ob::GoalState *>(it->second);
    if (gt) {
      viz_.publish(dynamic_cast<const Displayable *>
		   (si_->getStateSpace()->as<DARRTStateSpace>()->
		    getSubspace(it->first).get())->
		   displayable_state(gt->getState(), ns, id, 0.1, palate, a));
      //       ROS_INFO("Displaying object state");
      
      //       si_->getStateSpace().get()->as<DARRTStateSpace>()->
      // 	display_object_state(gt->getState(), it->first, ns, id, 0.1, palate, a);
      id++;
      continue;
    }

    ROS_ERROR("Attempt to display unknown goal type for space %u", it->first);
  }
  ROS_INFO("Done displaying goal state");
}
