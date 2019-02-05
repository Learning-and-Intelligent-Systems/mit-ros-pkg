//C++ includes
#include <numeric>
#include <iostream>
#include <sstream>

//local includes
#include "darrt/control.hh"
#include "darrt/object_space.hh"
#include "darrt/space.hh"
#include "darrt/utils.hh"

//ROS includes
#include <arm_navigation_msgs/ArmNavigationErrorCodes.h>
#include <arm_navigation_msgs/convert_messages.h>
#include <arm_navigation_msgs/GetRobotState.h>
#include <arm_navigation_msgs/GetStateValidity.h>
#include <planning_environment/models/model_utils.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>

//OMPL includes
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>

void darrt::convert_to_primitive_path
(const oc::PathControl &path, PrimitivePath &prim_path) {

  const Primitive *curr_prim = NULL;
  std::string curr_type = "";
  std::vector<const ob::State *> curr_states;

  for (unsigned int i = 0; i < path.getStateCount(); i++) {
    const Primitive *prim = 
      path.getState(i)->as<DARRTStateSpace::StateType>()->control(); 
    std::string type = 
      path.getState(i)->as<DARRTStateSpace::StateType>()->type(); 
    if (prim != curr_prim || type != curr_type) {
      if (curr_prim) {
	PrimitivePathEntry ppe(curr_prim, curr_type, curr_states);
	prim_path.push_back(ppe);
	curr_states.clear();
      }
      curr_prim = prim;
      curr_type = type;
    }
    curr_states.push_back(path.getState(i));
  }						
  if (curr_prim) {
    PrimitivePathEntry ppe(curr_prim, curr_type, curr_states);
    prim_path.push_back(ppe);
  }
}


oc::Control *darrt::DARRTControlSpace::allocControl() const {
  ControlType *c = new ControlType();
  return c;
}

void darrt::DARRTControlSpace::freeControl
(oc::Control *c) const {
  freeControlData(c);
  delete c->as<ControlType>();
}

void darrt::DARRTControlSpace::copyControl
(oc::Control *destination, const oc::Control *source) const {
  freeControlData(destination);
  const ControlType *s = source->as<ControlType>();
  ControlType *d = destination->as<ControlType>();
  for (PIList::const_iterator it = s->clist_.begin(); 
       it != s->clist_.end(); it++) {
    d->clist_.push_back((*it)->copy());
  } 
}
   
bool darrt::DARRTControlSpace::equalControls
(const oc::Control *control1, const oc::Control *control2) const {
  const ControlType *c1 = control1->as<ControlType>();
  const ControlType *c2 = control2->as<ControlType>();
  if (c1->clist_.size() != c2->clist_.size()) {
    return false;
  }
  PIList::const_iterator it2 = c2->clist_.begin();
  for (PIList::const_iterator it1 = c1->clist_.begin();
       it1 != c1->clist_.end(); it1++) {
    if (!(*it1)->equals(*it2)) {
      return false;
    }
    it2++;
  }
  return true;
}

void darrt::DARRTControlSpace::nullControl
(oc::Control *control) const {
  freeControlData(control);
}


void darrt::DARRTControlSpace::printSettings
(std::ostream &out) const {
  out << "Control space for" << stateSpace_.get()->getName(); 
}

void darrt::DARRTControlSpace::freeControlData
(oc::Control *control) const {
  ControlType *c = control->as<ControlType>();
  for (PIList::const_iterator it = c->clist_.begin(); it != c->clist_.end();
       it++) {
    delete *it;
  }
  c->clist_.clear();
  *(control->as<ControlType>()->curr_turn) = 0;
}

std::string darrt::DARRTControlSpace::control_string
(const oc::Control *control) const {
  std::string s = "[";
  const ControlType *c = control->as<ControlType>();
  for (PIList::const_iterator it = c->clist_.begin();
       it != c->clist_.end(); it++) {
    s += (*it)->str() + " ";
  }
  s += "]";
  return s;
}

void darrt::DARRTControlSpace::waitForUnpause() const {
  ROS_INFO("Planning paused");
  ros::Rate r(10);
  while (paused_ && ros::ok()) {
    r.sleep();
  }
  ROS_INFO("Planning unpaused");
}

darrt::DARRTDirectedControlSampler::DARRTDirectedControlSampler
(const oc::SpaceInformation *si) : oc::BiDirectedControlSampler(si) {
  sampler_ptr_ = si_->allocStateSampler();
  sampler_ = dynamic_cast<ProjectibleStateSampler *>(sampler_ptr_.get());
  projected_state_ = si_->allocState()->as<ProjectibleState>();
}

unsigned int darrt::DARRTDirectedControlSampler::sampleTo
(oc::Control *control, const ob::State *source, const ob::State *target) {
  Displayable *dspace = 
    dynamic_cast<Displayable *>(si_->getStateSpace().get());
  if (debug_level >= DPROPAGATE) {
    ROS_INFO("Unprojected state is %s (purple)", 
	     dspace->state_string(target).c_str());
    dspace->display_state(target, "unprojected", 0, 0.05,
    			  Displayable::PURPLEPALATE, 0.9);
  }

  //apply the function
  si_->copyState(projected_state_, target);
  if (!target->as<ProjectibleState>()->getProjectionFunction()) {
    ROS_WARN("NULL projection function in sampling!  Assuming identity!");
  } else {
    const DARRTProjectionFunction &f = target->as<ProjectibleState>()->getProjectionFunction();
    f(source, projected_state_);
  }
  return sampleDirectlyTo(control, source, projected_state_);
}


unsigned int darrt::DARRTDirectedControlSampler::sampleTo
(oc::Control *control, const oc::Control *previous, const ob::State *source, 
 const ob::State *target) {
  return sampleTo(control, source, target);
}

int darrt::DARRTDirectedControlSampler::sampleFrom
(oc::Control *control, const ob::State *target, const ob::State *source) {

  const Displayable *dspace = dynamic_cast<const Displayable *>
    (si_->getStateSpace().get());
  if (debug_level >= DPROPAGATE) {
    ROS_INFO("Unprojected state is %s (purple)", 
	     dspace->state_string(source).c_str());
    dspace->display_state(source, "unprojected", 0, 0.05,
    			  Displayable::PURPLEPALATE, 0.9);
  }

  if (debug_level >= DPROPAGATE) {
    ROS_INFO("SAMPLING IN REVERSE.  Original source was %s", 
	     dspace->state_string(source).c_str());
  }

  //fill in the state
  si_->copyState(projected_state_, source);
  if (!source->as<ProjectibleState>()->getProjectionFunction()) {
    ROS_WARN("NULL projection function in sampling!  Assuming identity!");
  } else {
    const DARRTProjectionFunction &f = source->as<ProjectibleState>()->getProjectionFunction();
    f(target, projected_state_);
  }

  //if we have a partial state for the object or for the destination, fill it in
  //because the result is the same (this is a little stupid but it works)
  //commented out during change over to projection functions

  // const ob::CompoundStateSpace *caspace = 
  //   si_->getStateSpace()->as<ob::CompoundStateSpace>();
  // const DARRTStateSpace *drspace = dynamic_cast<const DARRTStateSpace *>
  //   (si_->getStateSpace().get());
  // if ((drspace && !source->as<SelectivelySampledState>()->active(drspace->robot_index())) ||
  //     target->as<SelectivelySampledState>()->inactive_spaces.size()) {
  //   for (unsigned int i = 0; i < caspace->getSubspaceCount(); i++) {
  //     if (!full_state_->active(i)) {
  // 	sampler_->sampler(i)->sampleUniform(full_state_->components[i]);
  // 	//a bit of a hack here - assume we don't randomly sample a state
  // 	//that is on a surface or in the robot's hand
  // 	ObjectStateSpace::StateType *os = 
  // 	  dynamic_cast<ObjectStateSpace::StateType *>(full_state_->components[i]);
  // 	if (os) {
  // 	  os->attach_link = "";
  // 	  os->support_surface = NULL;
  // 	}
  //     }
  //   }
  //   full_state_->clear_inactive();
  // }

  unsigned int to_turns = sampleDirectlyTo(control, projected_state_, target);
  PIList clist = control->as<DARRTControlSpace::ControlType>()->clist_;
  if (!clist.size()) {
    return 0;
  }

  if (dynamic_cast<const InvalidPrimitiveInstance *>(clist[clist.size()-1])) {
    if (debug_level >= DPROPAGATE) {
      //pause("Unable to propagate from source all the way to target while sampling backwards.\nSource = "+dspace->state_string(source)+"\nTarget ="+
      //    dspace->state_string(target));
    }
    return 0;
  }
  *(control->as<DARRTControlSpace::ControlType>()->curr_turn) = to_turns+1;
  return -1.0*to_turns;
}
      
unsigned int darrt::DARRTDirectedControlSampler::sampleDirectlyTo
(oc::Control *control, const ob::State *source, const ob::State *target) {
  //assume the projection function has already been applied
  //this makes sure that all states in the tree have null projection functions
  //so that when we try to connect trees, we are actually connecting them!

  Displayable *sspace = 
    dynamic_cast<Displayable *>(si_->getStateSpace().get());
  const DARRTControlSpace *cspace =
    si_->getControlSpace().get()->as<DARRTControlSpace>();
  cspace->freeControlData(control);
    
  if (debug_level >= DRRT) {
    sspace->display_state(source, "source", 0, 0.05,
    			  Displayable::PRIMARYPALATE, 0.9);
    sspace->display_state(target, "target", 1, 0.05,
			  Displayable::BWPALATE, 0.9);
    if (debug_level == DRRT) {
      ROS_INFO("Sampling from %s (primary) to %s (bw)", 
	       sspace->state_string(source).c_str(),
	       sspace->state_string(target).c_str());
    } else {
      pause("Sampling from " + sspace->state_string(source) + "(primary) to "
	    + sspace->state_string(target) + "(bw)", 0);
    }
  }

  unsigned int turns = sampleToRecur(control, source, target, 0);
  if (debug_level >= DRRT) {
    const DARRTControlSpace::ControlType *c = 
      control->as<DARRTControlSpace::ControlType>();
    if (c->clist_.size()) {
      int index = c->clist_.size()-1;
      while (index >= 0 && dynamic_cast<const InvalidPrimitiveInstance *>(c->clist_[index])) {
	index--;
      }
      if (index >= 0) {
	sspace->display_state(c->clist_[index]->destination(), "final", 
			      2, 0.05, Displayable::GREENPALATE, 0.9);
      }
    }
    pause("Control (" + makestring(turns) + ") turns is " +
	  cspace->control_string(control), 0);//, sampling_to_goal);
  }
  return turns;
}


unsigned int darrt::DARRTDirectedControlSampler::sampleToRecur
(oc::Control *control, const ob::State *source, const ob::State *target,
 unsigned int depth) {

  const DARRTControlSpace *cspace =
    si_->getControlSpace().get()->as<DARRTControlSpace>();

  const Approximable *aspace =
    dynamic_cast<const Approximable *>(si_->getStateSpace().get());

    DARRTControlSpace::ControlType *c = 
      control->as<DARRTControlSpace::ControlType>();

  if (cspace->paused()) {
    cspace->waitForUnpause();
  }


  if (cspace->max_depth() >= 0 && 
      depth > (unsigned int)(cspace->max_depth())) {
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("Sampling depth exceeds max depth of %u", cspace->max_depth());
    }
    if (!c->clist_.size() || aspace->near_states(c->clist_[c->clist_.size()-1]->destination(), target)) {
      return 0;
    } else {
      InvalidPrimitiveInstance *ip = new InvalidPrimitiveInstance(1);
      c->clist_.push_back(ip);
      return 1;
    }
  }

  const Displayable *sspace = 
    dynamic_cast<const Displayable *>(si_->getStateSpace().get());

  if (debug_level >= DPROPAGATE) {
    ROS_INFO("Recursive sampling from %s to %s", 
	     sspace->state_string(source).c_str(),
	     sspace->state_string(target).c_str());
  }
  PrimitiveList useful;

  for (PrimitiveList::const_iterator it = cspace->primitives().begin();
       it != cspace->primitives().end(); it++) {
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("Testing usefulness of primitive %s", (*it)->str().c_str());
    }
    if ((*it)->useful(source, target)) {
      if (debug_level >= DPROPAGATE) {
	ROS_INFO("Primitive %s is useful", (*it)->str().c_str());
      }
      useful.push_back(*it);
    }
  }


  unsigned int turns = 0;
  bool progress = false;
  const ob::State *newsource = NULL;
  while (ros::ok() && !progress) {
    if (useful.empty()) {
      //this is as far as we can go
      if (!c->clist_.size() || aspace->near_states(c->clist_[c->clist_.size()-1]->destination(), target)) {
	return 0;
      } else {
	InvalidPrimitiveInstance *ip = new InvalidPrimitiveInstance(1);
	c->clist_.push_back(ip);
	return 1;
      }
    }
    //randomly choose a useful primitive
    int r = rand() % useful.size();
    const Primitive *prim = useful[r];
    

    //propagate it as far as it will go
    unsigned int prior_size = c->clist_.size();
    turns = prim->sampleTo(source, target, c->clist_);
    if (c->clist_.size()) {
      progress = true;
      newsource = c->clist_.back()->destination();
      if (turns == 0) {
	progress = false;
      } else {
	//by checking prior size we include the first control that was added
	//which will have a source equal to source
	for (int i = static_cast<int>(prior_size); i >= 0; i--) {
	  if (aspace->near_states(c->clist_[i]->source(), newsource)) {
	    if (debug_level >= DPROPAGATE) {
	      ROS_INFO("Primitive made no progress because %s is near %s",
		       sspace->state_string(c->clist_[i]->source()).c_str(),
		       sspace->state_string(newsource).c_str());
	    }
	    progress = false;
	    break;
	  }
	}
      }
      if (!progress) {
	if (debug_level >= DPROPAGATE) {
	  ROS_INFO("Primitive %s made no progress.  Removing %zu controls.",
		   prim->str().c_str(), c->clist_.size() - prior_size);
	}
	unsigned int final_size = c->clist_.size();
	for (unsigned int i = prior_size; i < final_size; i++) {
	  c->clist_.pop_back();
	}
      }
    }
    useful[r] = useful.back();
    useful.pop_back();
  }
  if (!ros::ok()) {
    return turns;
  }

  return turns + sampleToRecur(control, newsource, target, depth+1);
}


oc::DirectedControlSamplerPtr darrt::alloc_darrt_sampler
(const oc::SpaceInformation *si) {
  return oc::DirectedControlSamplerPtr
    (new DARRTDirectedControlSampler(si));
}

void darrt::DARRTStatePropagator::propagate
(const ob::State *state, const oc::Control *control,
 const double duration, ob::State *result) const {

  if (si_->getControlSpace()->as<DARRTControlSpace>()->paused()) {
    si_->getControlSpace()->as<DARRTControlSpace>()->waitForUnpause();
  }

  if (debug_level >= DPROPAGATE) {
    ROS_INFO("Propogate: Duration %f", duration);
  }
  const ob::CompoundStateSpace *sspace = 
    si_->getStateSpace()->as<ob::CompoundStateSpace>();
  const Displayable *dspace = dynamic_cast<const Displayable *>(sspace);

  const DARRTControlSpace::ControlType *c = 
    control->as<DARRTControlSpace::ControlType>();

  const ob::State *start = state;

  //assuming we don't need this any longer...
  //const SelectivelySampledState *s = state->as<SelectivelySampledState>();
  // if (c->clist_.size() > 0) {
  //   //does the state we're progatating from have in active spaces?
  //   for (unsigned int i = 0; i < sspace->getSubspaceCount(); i++) {
  //     if (!s->active(i)) {
  // 	if (duration < 0) {
  // 	  start = c->clist_[c->clist_.size()-1]->destination();
  // 	} else {
  // 	  start = c->clist_[0]->source();
  // 	}
  // 	break;
  //     }
  //   }
  // }

  if (duration > 0) {
    (*(c->curr_turn))++;
  } else {
    (*(c->curr_turn))--;
  }

  unsigned int t = 0;
  const PrimitiveInstance *pi = NULL;
  if ((*(c->curr_turn)) < 0) {
    pi = dynamic_cast<const InvalidPrimitiveInstance *>(c->clist_[c->clist_.size()-1]);
  } else {
    for (unsigned int i = 0; i < c->clist_.size(); i++) {
      pi = c->clist_[i];
      if (pi->turns() + t >= static_cast<unsigned int>(*(c->curr_turn))) {
	break;
      }
      if (debug_level == DBETWEEN) {
	ROS_INFO("Finished with primitive %s", pi->str().c_str());
      }
      t += pi->turns();
    }
 
    const InvalidPrimitiveInstance *ip = dynamic_cast<const InvalidPrimitiveInstance *>(pi);
    unsigned int times = 1;
    while (ip) {
      if (duration > 0) {
	break;
      } else {
	(*(c->curr_turn))--;
	int index = c->clist_.size()-1-times;
	if (index < 0) {
	  pi = NULL;
	  break;
	}
	pi = c->clist_[index];
	times++;
	ip = dynamic_cast<const InvalidPrimitiveInstance *>(pi);
      }
    }
  }

  if (dynamic_cast<const InvalidPrimitiveInstance *>(pi)) {
    result->as<ProjectibleState>()->set_invalid();
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("Adding INVALID state");
    }
    return;
  }
  
  if (!pi) {
    pause("Control has a total of " + makestring(t) +  
	  " turns, but current turn is "
	  + makestring(*(c->curr_turn)) + 
	  " and we should still be propagating.  Control is\n%s"
	  + si_->getControlSpace()->as<DARRTControlSpace>()->control_string(c));
    si_->copyState(result, start);
    result->as<ProjectibleState>()->setProjectionFunction(NULL);
    return;
  }
  
  pi->propagate(start, duration, result);
  //ensure that no states added to the tree have
  //projection functions
  result->as<ProjectibleState>()->setProjectionFunction(NULL);

  if (debug_level > DSILENT) {
    Displayable::ColorPalate color = Displayable::PASTELPALATE;
    if (duration < 0) {
      color = Displayable::YELLOWPALATE;
    }
    dspace->display_state(result, "states", 1,
			  0.05, color, 0.8, 
			  debug_level <= DMINIMAL);
  }
  if (debug_level >= DPROPAGATE) {
    ROS_INFO("TURN %u: Using primitive %s", *(c->curr_turn), pi->str().c_str());
    std::ostringstream str;
    str << "Added state ";
    sspace->printState(result, str);
    //pause(str.str(), 0);
    ROS_INFO("%s", str.str().c_str());
  }
}
