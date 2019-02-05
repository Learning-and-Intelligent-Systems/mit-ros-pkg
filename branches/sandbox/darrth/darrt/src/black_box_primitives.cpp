#include "darrt/black_box_primitives.hh"
#include "darrt/pr2_arm_primitives.hh"

darrt::BlackBoxPrimitive::BlackBoxPrimitive(std::string planning_action_name) :
  TransferPrimitive(execution_action_name),
  planning_action_name_(planning_action_name),
  execution_action_name_(execution_action_name),
  transit_(NULL)
{
  planning_action_ = 
    new actionlib::SimpleActionClient<darrt_msgs::DARRTPrimitivePlanningAction>
    (planning_action_name_);
  ROS_INFO("Waiting for planning action %s", planning_action_name_.c_str());
  planning_action_->waitForServer();
  execute_action_ = NULL;
  dspace_ = NULL;
}
  
darrt::BlackBoxPrimitive::BlackBoxPrimitive
(std::string planning_action_name, std::string execution_action_name) :
  TransferPrimitive(execution_action_name),
  planning_action_name_(planning_action_name),
  execution_action_name_(execution_action_name),
  transit_(NULL)
{
  planning_action_ = 
    new actionlib::SimpleActionClient<darrt_msgs::DARRTPrimitivePlanningAction>
    (planning_action_name_);
  ROS_INFO("Waiting for planning action %s", planning_action_name_.c_str());
  planning_action_->waitForServer();
  execute_action_ =
    new actionlib::SimpleActionClient<darrt_msgs::DARRTPrimitiveAction>
    (execution_action_name_);
  ROS_INFO("Waiting for execution action %s", execution_action_name_.c_str());
  execute_action_->waitForServer();
  dspace_ = NULL;
}

bool darrt::BlackBoxPrimitive::setup(const oc::SpaceInformation *si) {
  if (!TransferPrimitive::setup(si)) {
    return false;
  }

  dspace_ = si_->getStateSpace()->as<DARRTStateSpace>();
  
  //also find a transit primitive
  for (size_t i = 0; i < dspace_->primitives().size(); i++) {
    transit_ = dynamic_cast<const PR2ArmTransit *>
      (dspace_->primitives().at(i));
    if (transit_) {
      break;
    }
  }
  if (!transit_) {
    pause("Unable to find transit primitive for black box primitive!");
    is_setup_ = false;
    return false;
  }
  return true;
}

void darrt::BlackBoxPrimitive::get_goal
(const ob::State *source, const ob::State *target, 
 darrt_msgs::DARRTPrimitivePlanningGoal &goal) const {
  //fill in inactive spaces with what they were in the source state
  ob::State *active_target = dspace_->allocState();
  dspace_->copyState(active_target, source);
  const DARRTStateSpace::StateType *t = 
    target->as<DARRTStateSpace::StateType>();
  for (unsigned int i = 0; i < dspace_->getSubspaceCount(); i++) {
    if (t->active(i)) {
      dspace_->getSubspace(i)->copyState(dspace_->get_state(i, active_target),
					 dspace_->get_state(i, target));
    }
  }
  dspace_->convert_ompl_state_to_darrt_state(active_target, goal.to_state);
  dspace_->convert_ompl_state_to_darrt_state(source, goal.from_state);

  const CompoundRobotStateSpace *cspace = dspace_->robot_state_space()->
    as<CompoundRobotStateSpace>();
  for (unsigned int i = 0; i < cspace->getSubspaceCount(); i++) {
    goal.groups.push_back(cspace->getSubspace(i)->getName());
  }	   
}

bool darrt::BlackBoxPrimitive::useful(const ob::State *source, 
				      const ob::State *target) const {
  return true;
  // if (!source->as<DARRTStateSpace::StateType>()->valid()) {
  //   return false;
  // }

  // darrt_msgs::DARRTPrimitivePlanningGoal goal;

  // get_goal(source, target, goal);
  // goal.check_usefulness_only = true;

  // ROS_INFO("Useful: Sending goal to planning action");
  // planning_action_->sendGoal(goal);
  // planning_action_->waitForResult();
  
  // return planning_action_->getResult()->useful;
}

unsigned int darrt::BlackBoxPrimitive::sampleTo
(const ob::State *source, const ob::State *target, PIList &clist) const {
  darrt_msgs::DARRTPrimitivePlanningGoal goal;
  get_goal(source, target, goal);

  ROS_INFO("sampleTo: Sending goal to planning action");
  planning_action_->sendGoal(goal);
  planning_action_->waitForResult();
  darrt_msgs::DARRTPrimitivePlanningResultConstPtr result = 
    planning_action_->getResult();

  if (!result->useful || !result->path.size()) {
    //ROS_ERROR("Black box primitive called but not useful!");
    return 0;
  }

  std::vector<ob::State *> path;
  for (size_t i = 0; i < result->path.size(); i++) {
    ob::State *state = dspace_->allocState();
    dspace_->convert_darrt_state_to_ompl_state(result->path[i], state);
    path.push_back(state);
  }

  unsigned int nturns = 0;
  //transit to the "pre-grasp"
  if (!dspace_->near_states(source, path[0])) {
    nturns += transit_->sampleTo(source, path[0], clist);
  }
  
  StatePathInstance *pi = new BlackBoxInstance(this, path);
  clist.push_back(pi);
  return nturns + path.size();
}

ob::State *darrt::BlackBoxPrimitive::get_first_state
(unsigned int objind, const ob::State *source, const ob::State *destination)
  const {
  //this is probably too slow to call an action here?
  //how are we going to do distance... garrr
  //uggggh really assumes one object nrrrrr
  DARRTStateSpace::StateType *d = 
    dspace_->allocState()->as<DARRTStateSpace::StateType>();
  dspace_->copyState(d, destination);
  for (unsigned int i = 0; i < dspace_->getSubspaceCount(); i++) {
    if (i != objind) {
      d->set_inactive(i);
    }
  }
  darrt_msgs::DARRTPrimitivePlanningGoal goal;
  get_goal(source, d, goal);

  ROS_INFO("distance: Sending goal to planning action");
  planning_action_->sendGoal(goal);
  planning_action_->waitForResult();
  darrt_msgs::DARRTPrimitivePlanningResultConstPtr result = 
    planning_action_->getResult();

  if (!result->useful || !result->path.size()) {
    //ROS_ERROR("Black box distance primitive called but not useful!");
    return NULL;
  }

  ob::State *first_state = dspace_->allocState();
  dspace_->convert_darrt_state_to_ompl_state(result->path[0], first_state);
  return first_state;
}

//SHOULD BE CARTESIAN DISTANCE!!
//it is now
double darrt::BlackBoxPrimitive::distance
(const ob::State *source, const ob::State *destination) const {
  return dspace_->robot_state_space()->distance
    (dspace_->robot_state(destination), dspace_->robot_state(source));
}

double darrt::BlackBoxPrimitive::distance_to_nearest_grasp
(unsigned int objind, const ob::State *source, const ob::State *destination)
  const {
  ob::State *first_state = get_first_state(objind, source, destination);
  if (!first_state) {
    return MATH_INF;
  }
  double dist = distance(source, first_state);
  dspace_->freeState(first_state);
  return dist;
}


bool darrt::BlackBoxPrimitive::execute
(const std::vector<const ob::State *> &path) const {
  darrt_msgs::DARRTPrimitiveGoal goal;

  for (size_t i = 0; i < path.size(); i++) {
    darrt_msgs::DARRTState state;
    dspace_->convert_ompl_state_to_darrt_state(path[i], state);
    goal.path.push_back(state);
  }
  const CompoundRobotStateSpace *cspace = dspace_->robot_state_space()->
    as<CompoundRobotStateSpace>();
  for (unsigned int i = 0; i < cspace->getSubspaceCount(); i++) {
    goal.groups.push_back(cspace->getSubspace(i)->getName());
  }	   
  
  ROS_INFO_STREAM("Sending goal\n" << goal << "\nto black box action");
  execute_action_->sendGoal(goal);
  execute_action_->waitForResult();
  return execute_action_->getResult()->success;
}

double darrt::BlackBoxWithPregrasp::distance_to_nearest_grasp
(unsigned int objind, const ob::State *source, const ob::State *destination)
  const {
  
  PreGraspMap::const_iterator it = 
    pregrasp_map_->find(dspace_->getSubspace(objind)->getName());
  if (it != pregrasp_map_->end()) {
    return distance(source, it->second);
  }
  ob::State *first_state = get_first_state(objind, source, destination);
  if (!first_state) {
    return MATH_INF;
  }
  (*pregrasp_map_).insert
    (std::make_pair(dspace_->getSubspace(objind)->getName(), first_state));
  return distance(source, first_state);
}

darrt::BlackBoxWithPregrasp::~BlackBoxWithPregrasp() {
  for (PreGraspMap::iterator it = pregrasp_map_->begin();
       it != pregrasp_map_->end(); it++) {
    dspace_->freeState(it->second);
  }
  delete pregrasp_map_;
}
