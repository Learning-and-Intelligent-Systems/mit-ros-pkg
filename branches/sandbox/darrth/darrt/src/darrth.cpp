#include "darrt/darrth.hh"
#include "darrt/object_space.hh"
#include "darrt/object_primitives.hh"
#include "darrt/types.hh"

#include <arm_navigation_msgs/GetRobotState.h>

darrt::DARRTHSolver::DARRTHSolver(ros::NodeHandle n) : 
  InterruptableSolver(), darrt_solver_(n), object_solver_(n) {
  environment_interface_ = darrt_solver_.environment_interface();
  configured_ = false;
  full_path_ = NULL;
}

darrt::DARRTHSolver::~DARRTHSolver() {
  if (full_path_) {
    full_path_->getStates().clear();
    full_path_->getControls().clear();
    delete full_path_;
  }
}

const oc::PathControl *darrt::DARRTHSolver::getSolutionPath() {
  if (full_path_) {
    return full_path_;
  }
  full_path_ = new oc::PathControl(darrt_solver_.getSpaceInformation());
  std::vector<ob::State *> &states = full_path_->getStates();
  std::vector<oc::Control *> &controls = full_path_->getControls();
  std::vector<double> &durations = full_path_->getControlDurations();
  ROS_INFO("path is size %u", path_.size());
  for (unsigned int i = 0; i < path_.size(); i++) {
    //because starting states are doubled and we need to match up controls
    unsigned int start = 1;
    if (i == 0 || path_[i].getStateCount() == path_[i].getControlCount()) {
      start = 0;
    }
    for (unsigned int j = start; j < path_[i].getStateCount(); j++) {
      states.push_back(path_[i].getState(j));
    }
    for (unsigned int j = 0; j < path_[i].getControlCount(); j++) {
      controls.push_back(path_[i].getControl(j));
      durations.push_back(path_[i].getControlDuration(j));
    }
  }
  return full_path_;
}

bool darrt::DARRTHSolver::reset(ros::WallDuration timeout) {
  if (full_path_) {
    full_path_->getStates().clear();
    full_path_->getControls().clear();
    delete full_path_;
    full_path_ = NULL;
  }
  path_.clear();
  ROS_INFO("Deleting object solver");
  bool osr = object_solver_.reset(timeout);

  ROS_INFO("Deleting darrt solver");
  bool dr = darrt_solver_.reset(timeout);
  return (osr && dr && InterruptableSolver::reset(timeout));
}

bool darrt::DARRTHSolver::display_solution(bool step) const {
  int nstates = 0;
  for (size_t i = 0; i < path_.size(); i++) {
    if (!darrt_solver_.display_solution(path_[i], step, nstates)) {
      return false;
    }
    nstates += path_[i].getStateCount();
  }
  return true;
}

bool darrt::DARRTHSolver::execute_solution() const {
  for (size_t i = 0; i < path_.size(); i++) {
    if (!darrt_solver_.execute_solution(path_[i])) {
      return false;
    }
  }
  return true;
}

bool darrt::DARRTHSolver::configure(const Goal &goal) {
  reset();
  configured_ = false;
  info_ = goal.params;
  path_.clear();

  if (goal.size() > 1) {
    ROS_WARN("Multiple goals!  Will only solve for first goal");
  }

  goal_object_ = goal.begin()->first;

  if (!darrt_solver_.configure(goal)) {
    ROS_ERROR("Unable to configure DARRT solver");
    return false;
  }

  if (!object_solver_.configure(goal)) {
    ROS_ERROR("Unable to configure object solver");
    return false;
  }
  configured_ = true;
  return true;
}

bool darrt::DARRTHSolver::cancel(ros::WallDuration timeout) {
  ros::WallTime start = ros::WallTime::now();
  if (!darrt_solver_.cancel(timeout)) {
    return false;
  }
  if (!object_solver_.cancel(timeout)) {
    return false;
  }
  return InterruptableSolver::cancel(timeout);
}

//this needs fixing
bool darrt::DARRTHSolver::object_path_to_subgoals
(const Goal &goal, std::vector<Goal> &subgoals) const {

  const oc::PathControl *path = object_solver_.getSolutionPath();

  if (cancelled()) {
    return false;
  }
  if (!path || path->getStateCount() == 0) {
    ROS_ERROR("Error converting path to subogoals: empty path");
    return false;
  }

  // if (path->getStateCount() <= 2) {
  //   ROS_WARN("Object path too short!  Trying again!  This is a debugging tool you probably want to turn off");
  //   return false;
  // }

  ROS_INFO("There are %zd points on the object path", path->getStateCount());

  //we know the initial position
  const oc::SpaceInformation *si = darrt_solver_.space_information();
  const DARRTStateSpace *space = 
    si->getStateSpace().get()->as<DARRTStateSpace>();

  std::string curr_prim("");
  bool first = false;

  for (unsigned int i = 0; i < path->getStateCount()-1; i++) {
    const ob::State *s = path->getState(i);
    if (!s->as<ObjectStateSpace::StateType>()->control()) {
      continue;
    }
    const Primitive *prim = dynamic_cast<const CollisionAwareState *>(s)->
      control();
    if (!prim || !prim->name().compare(curr_prim) || prim->transit()) {
      continue;
    }
    if (!first && prim->becomesGoal()) {
      ROS_INFO("Adding switch from %s to %s to path", curr_prim.c_str(), 
  	       prim->name().c_str());
    }
    curr_prim = prim->name();
    if (first || !prim->becomesGoal()) {
      first = false;
      continue;
    }

    space->display_state(s, "subgoals", subgoals.size(), 1, 
			 Displayable::PURPLEPALATE);

    Goal sg;
    sg.copyInfo(goal);
    sg[goal_object_].type = SingleGoal::PRIMITIVE;
    sg[goal_object_].primitive = prim;
    sg[goal_object_].primitive_actual_goal = &goal;
    Goal::const_iterator ait = goal.find(goal_object_);
    if (ait == goal.end()) {
      ROS_ERROR("Trying to set subgoal for %s but unable to find in original goal!", 
		goal_object_.c_str());
    }
    sg[goal_object_].bounds = ait->second.bounds;
    sg[goal_object_].threshold = ait->second.threshold;
    subgoals.push_back(sg);
  }
  
  //append the actual goal as the last thing to reach
  subgoals.push_back(goal);
  return true;
      
}

bool darrt::DARRTHSolver::plan(const Goal &goal, float &time, float &object_time, std::vector<float> &subgoal_times) {
  if (!configured_) {
    ROS_ERROR("Must configure DARRTH solver before planning.");
    return false;
  }
  //if we're just using DARRT... do that
  if (info_.use_darrt) {
    ROS_INFO("Using DARRT to solve goal");
    bool solved = darrt_solver_.plan(goal, time);
    if (!solved) {
      return false;
    }
    path_.push_back(*(darrt_solver_.getSolutionPath()));
    return true;
  }

  //first plan the object paths
  const ob::State *initial_state = NULL;

  phase_ = SOLVING;
  ros::WallTime start = ros::WallTime::now();

  //plan a path
  const ob::State *curr_state;
  std::vector<oc::PathControl> sgpaths;
  //set the state back to the original starting pose
  curr_state = initial_state;
  ROS_INFO("Planning object path");
  bool actual_do_pause = do_pause;
  //do_pause = false;
  bool objsuccess = object_solver_.plan(goal, object_time);
  do_pause = actual_do_pause;
  ROS_INFO("Object solving took %f seconds", object_time);
  if (!objsuccess) {
    ROS_ERROR("Unable to find path for object");
    ros::WallTime end = ros::WallTime::now();
    time = (end - start).toSec();
    phase_ = READY;
    return false;
  }

  if (debug_level >= DMINIMAL) {
    object_solver_.display_solution();
    ROS_INFO("Found solution shown for objects");
    if (debug_level >= DRRT && do_pause) {
      std::string buff;
      getline(std::cin, buff);
    }
  }
  std::vector<Goal> subgoals;
  if (!object_path_to_subgoals(goal, subgoals)) {
    ROS_INFO("Unable to convert that path to subgoals");
    ros::WallTime end = ros::WallTime::now();
    time = (end - start).toSec();
    phase_ = READY;
    return false;
  }
  subgoal_times.resize(subgoals.size());
  ROS_INFO("There are %zd subgoals.  Starting to plan", subgoals.size());
  sgpaths.clear();
  for (unsigned int j = 0; j < subgoals.size(); j++) {
    ROS_INFO("Planning from %p to subgoal %u", curr_state, j);
    bool solved_subgoal = false;
    subgoal_times[j] = 0;
    for (unsigned int k = 0; k < goal.params.darrt_tries; k++) { 
      float subgoal_time;
      solved_subgoal = darrt_solver_.plan(subgoals[j], curr_state, subgoal_time);
      subgoal_times[j] += subgoal_time;
      if (solved_subgoal) {
	break;
      }
    }
    if (!solved_subgoal) {
      ROS_INFO("Failed to solve subgoal %u", j);
      ros::WallTime end = ros::WallTime::now();
      time = (end - start).toSec();
      phase_ = READY;
      return false;
    }
    const oc::PathControl *path = darrt_solver_.getSolutionPath();
    if (!path || path->getStateCount() == 0) {
      ROS_WARN("We think we solved for a subgoal but cannot get a path");
      ros::WallTime end = ros::WallTime::now();
      time = (end - start).toSec();
      phase_ = READY;
      return false;
    }
    //make sure we copy the memory before it gets deleted from the solver
    sgpaths.push_back(*path);
    const oc::PathControl &local_path = sgpaths[sgpaths.size()-1];
    curr_state = local_path.getState(local_path.getStateCount()-1);
    if (debug_level >= DPROPAGATE) {
      darrt_solver_.display_solution(true);
      pause("Successfully solved subgoal " + makestring(j), 0);
    } else {
      //darrt_solver_.display_solution(true);
      //pause("Successfully solved subgoal " + makestring(j), 0);
      ROS_INFO("Successfully solved subgoal %u", j);
    }
  }
  for (size_t j = 0; j < sgpaths.size(); j++) {
    path_.push_back(sgpaths[j]);
  }

  ros::WallTime end = ros::WallTime::now();
  time = (end - start).toSec();
  phase_ = READY;
  return true;
}
  
