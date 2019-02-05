//local includes
#include "darrt/chain_space.hh"
#include "darrt/control.hh"
#include "darrt/object_space.hh"
#include "darrt/pr2_base_primitives.hh" //:(
#include "darrt/solver.hh"
#include "darrt/space.hh"

//ompl includes
#include <ompl/control/planners/rrt/RRT.h>
#include <rrt_connect/RRTConnect.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/datastructures/NearestNeighborsLinear.h>
#include <ompl/util/Exception.h>

//ros includes
#include <arm_navigation_msgs/AttachedCollisionObject.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/GetRobotState.h>
#include <arm_navigation_msgs/GetStateValidity.h>
#include <arm_navigation_msgs/RobotTrajectory.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <planning_environment/models/model_utils.h>
//#include <valgrind/callgrind.h>

//i think this will usually be ok even with multiple threads
//maybe... i hope
bool darrt::InterruptableSolver::cancel(ros::WallDuration timeout) {
  if (phase_ == SOLVING) {
    phase_ = WAITING;
    ros::Rate wrate(100);
    ros::WallTime start = ros::WallTime::now(), current = ros::WallTime::now();
    while (phase_ == WAITING && 
	   (timeout == ros::WallDuration(0) || current - start < timeout)) {
      wrate.sleep();
      current = ros::WallTime::now();
    }
  } else {
    phase_ = READY;
  }
  return phase_ != WAITING;
}

bool darrt::InterruptableSolver::reset(ros::WallDuration timeout) {
  ros::Rate wrate(100);
  ros::WallTime start = ros::WallTime::now(), current = ros::WallTime::now();
  while (phase_ == WAITING && 
	 (timeout == ros::WallDuration(0) || current - start < timeout)) {
    wrate.sleep();
    current = ros::WallTime::now();
  }
  return phase_ != READY;
}

ob::PlannerTerminationCondition darrt::InterruptableSolver::terminate_after
(double time) const {
  ob::PlannerTerminationCondition 
    rok(boost::bind(&InterruptableSolver::cancelled, this));
  if (time < 1.0) {
    return ob::PlannerOrTerminationCondition
      (rok, ob::timedPlannerTerminationCondition(time));
  } 
  return ob::PlannerOrTerminationCondition
    (rok, ob::timedPlannerTerminationCondition(time, 
					       std::min(time/100.0, 0.1)));
}


darrt::DARRTSolver::DARRTSolver(ros::NodeHandle n) : InterruptableSolver() {
  node_ = n;
  environment_interface_ = new EnvironmentInterface("/robot_description");
  set_planning_scene_ = node_.serviceClient<an::SetPlanningSceneDiff>
    ("/environment_server/set_planning_scene_diff");
  ROS_INFO("Waiting for set planning scene service");
  set_planning_scene_.waitForExistence();
  ROS_INFO("Resetting planning scene");
  an::SetPlanningSceneDiff ssd;
  set_planning_scene_.call(ssd);

  get_planning_scene_ = node_.serviceClient<an::GetPlanningScene>
    ("/environment_server/get_planning_scene");
  ROS_INFO("Waiting for get planning scene service");
  get_planning_scene_.waitForExistence();

  get_robot_state_ = node_.serviceClient<an::GetRobotState>
    ("/environment_server/get_robot_state");
  ROS_INFO("Waiting for get robot state service");
  get_robot_state_.waitForExistence();

  solver_ = NULL;
  starting_state_ = NULL;
  configured_ = false;
}

bool darrt::DARRTSolver::reset(ros::WallDuration timeout) {
  if (solver_) {
    ROS_INFO("Deleting solver");
    delete solver_;
  }
  
  //NOTE: delete the primitives AFTER the solver!
  //don't try to delete the path afterwards
  for (unsigned int i = 0; i < primitives_.size(); i++) {
    delete primitives_[i];
  }
  primitives_.clear();

  goal_.reset();
  solver_ = NULL;
  return InterruptableSolver::reset(timeout);
}

darrt::DARRTSolver::~DARRTSolver() {
  delete environment_interface_;
  if (solver_) {
    delete solver_;
  }
}



//We have split this into configure and plan because
//when using DARRT we call plan for the same world pretty
//often and do not want to wait for configure to run
//every time
//Planning scene should already be set when we get here
bool darrt::DARRTSolver::configure(const Goal &goal) {

  ROS_INFO("Configuring solver");
  reset();
  info_ = goal.params;
  configured_ = false;

  //initialize the environment
  an::RobotState tmp;
  if (!initialize_environment(tmp, goal.objects)) {
    ROS_ERROR("Unable to initialize the environment");
    return false;
  }

  //set up the planner
  if (!setup_space(goal)) {
    ROS_ERROR("Unable to set up space and initialize simple setup");
    return false;
  }
  ROS_INFO("Successfully initialized simple setup.");


  if (!setup_state_validity_checker(goal)) {
    ROS_ERROR("Unable to create state validity checker");
    return false;
  }
  ROS_INFO("Successfully created the state validity checker");

  ROS_INFO("Successfully configured DARRT planner");

  // ROS_INFO("Resetting the planning scene");
  // an::SetPlanningSceneDiff ssd;
  // set_planning_scene_.call(ssd);

  configured_ = true;



  //hack to allow this to work over SSH without real two-way communication
  //(i.e. when i'm at home)
  //might be fixable just by setting ROS_IP to the laptop's IP!
  //pause("Initialized robot_state_markers topic.  If you are running over SSH, you need to reset your topic name now.  Press enter when ready.");

  return true;

}

bool darrt::DARRTSolver::display_solution(bool step, unsigned int resolution) const {
  return display_solution(solver_->getSolutionPath(), step, 0, resolution);
}


bool darrt::DARRTSolver::display_solution(const oc::PathControl &path, bool step,
					  int offset, unsigned int resolution) const {
  ROS_INFO("In display solution!");
  if (path.getControlCount() == 0) {
    ROS_INFO("Display solution: No path to display");
    return true;
  }
  std::ostringstream ss;
  if (debug_level >= DPROPAGATE) {
    ROS_INFO("control count = %u, state count = %u", 
	     path.getControlCount(), path.getStateCount());
    ss << "Full path is:";
    for (unsigned int i = 0; i < path.getControlCount(); i++) {
      ss << "\n\nState " << i << ": ";
      solver_->getStateSpace()->printState(path.getState(i), ss);
      ss << " using control: ";
      solver_->getControlSpace()->printControl(path.getControl(i), ss);
      ss << "\nIs valid: " << solver_->getSpaceInformation()->isValid
	(path.getState(i));
    }
    ROS_INFO("%s", ss.str().c_str());
    ss.str("");
    ss << "\nControl path is:\n";
    const oc::Control *curr = path.getControl(0);
    solver_->getControlSpace()->printControl(curr, ss);
    for (unsigned int i = 1; i < path.getControlCount(); i++) {
      if (!solver_->getControlSpace()->equalControls(curr, path.getControl(i))){
	curr = path.getControl(i);
	ss << "\n";
	solver_->getControlSpace()->printControl(curr, ss);
      } 
    }
    ROS_INFO("%s", ss.str().c_str());
  }
  ss.str("");
  ss << "Primitive path is:";
  visualization_msgs::MarkerArray marray;
  PrimitivePath prim_path;
  convert_to_primitive_path(path, prim_path);
  
  int ind = offset;
  
  for (size_t i = 0; i < prim_path.size(); i++) {
    const Primitive *prim = prim_path[i].primitive;
    std::string type = prim_path[i].type;
    ss << "\n" << prim->str() << "(Type: " << type << ")";
    Displayable::ColorPalate p = Displayable::BWPALATE;
    if (!prim->name().substr(0,10).compare("ArmTransit")) {
      p = Displayable::REDPALATE;
    }
    if (!prim->name().substr(0,13).compare("RigidTransfer")) {
      p = Displayable::ORANGEPALATE;
    }
    if (!prim->name().substr(0,10).compare("UseSpatula")) {
      p = Displayable::BLUEPALATE;
    }
    if (!prim->name().substr(0,15).compare("SpatulaTransfer")) {
      p = Displayable::YELLOWPALATE;
    }
    if (!prim->name().substr(0,8).compare("Approach")) {
      p = Displayable::BWPALATE;
    }
    if (!prim->name().substr(0,7).compare("Retreat")) {
      p = Displayable::BWPALATE;
    }
    if (!prim->name().substr(0,4).compare("Push")) {
      p = Displayable::CYANPALATE;
    }	    
    if (!prim->name().substr(0,6).compare("Pickup") || !prim->name().compare("Place")) {
      p = Displayable::GREENPALATE;
    }
    if (!prim->name().substr(0,11).compare("BaseTransit")) {
      p = Displayable::PURPLEPALATE;
    }
    ros::Rate rt(50);
    for (size_t j = 0; j < prim_path[i].states.size()+resolution; j+=resolution) {
      if (!prim_path[i].states.size()) {
	break;
      }
      unsigned int sind = j;
      if (j >= prim_path[i].states.size()) {
	sind = prim_path[i].states.size()-1;
      }
      visualization_msgs::MarkerArray sd = 
	solver_->getStateSpace()->as<DARRTStateSpace>()->displayable_state
	(prim_path[i].states[sind], "path", ind, 0.05, p);
      if (step) {
	visualization_msgs::MarkerArray td = sd;
	for (unsigned int k = 0; k < td.markers.size(); k++) {
	  td.markers[k].id = offset+k;
	}
	solver_->getStateSpace()->as<DARRTStateSpace>()->display(td);
	rt.sleep();
	//pause("Showing state", 0, true);
      }
      ind++;
      for (size_t k = 0; k < sd.markers.size(); k++) {
	marray.markers.push_back(sd.markers[k]);
      }
    }
  }
  ROS_INFO("%s", ss.str().c_str());
  if (!step) {
    solver_->getStateSpace()->as<DARRTStateSpace>()->display(marray);
  }
  return true;
}

bool darrt::DARRTSolver::initialize_environment(an::RobotState &starting_state, 
						const std::vector<const DARRTObject *> &moveable_objects) {
  an::GetPlanningScene srv;
  ros::NodeHandle n;
  if (!get_planning_scene_.call(srv)) {
    ROS_ERROR("initialize robot state: unable to get robot state");
    return false;
  }
  if (environment_interface_->kinematicState()) {
    environment_interface_->revertPlanningScene(environment_interface_->kinematicState());
    environment_interface_->setKinematicState(NULL);
  }
  environment_interface_->setKinematicState
    (environment_interface_->setPlanningScene(srv.response.planning_scene));
  if (!environment_interface_->kinematicState()) {
    return false;
  }
  environment_interface_->updateApproximateEnvironment();  
  for (unsigned int i = 0; i < moveable_objects.size(); i++) {
    environment_interface_->removeObjectFromApproximateEnvironment(*(moveable_objects[i]));
  }
  
  std::vector<std::string> objnames;
  environment_interface_->getCollisionObjectNames(objnames);
  ROS_INFO("Objects in environment are:");
  for (unsigned int i = 0; i < objnames.size(); i++) {
    ROS_INFO("%s", objnames[i].c_str());
  }
  if (starting_state.joint_state.name.size()) {
    planning_environment::setRobotStateAndComputeTransforms(starting_state, 
							    *(environment_interface_->kinematicState()));
  } else {
    starting_state = srv.response.planning_scene.robot_state;
  }
  return true;
}

const oc::PathControl *darrt::DARRTSolver::getSolutionPath() const {
  if (!solver_) {
    ROS_ERROR("Attempt to get path before configuring.");
    return NULL;
  }
  
  try {
    return &(solver_->getSolutionPath());
  } catch (ompl::Exception &e) {
    ROS_ERROR("No solution path.  Did you plan?");
    return NULL;
  }
}

const oc::SpaceInformation *darrt::DARRTSolver::space_information() const {
  if (!solver_) {
    ROS_ERROR("Must configure before requesting space information");
    return NULL;
  }
  
  return solver_->getSpaceInformation().get();
}

const oc::SpaceInformationPtr darrt::DARRTSolver::getSpaceInformation() const {
  if (!solver_) {
    ROS_ERROR("Must configure before requesting space information");
    return oc::SpaceInformationPtr();
  }
  
  return solver_->getSpaceInformation();
}



bool darrt::DARRTSolver::check_object_headers(const Goal &goal) {

  //check the poses on the goal
  //should just convert
  std::string world_frame = environment_interface_->getWorldFrameId(); 
  for (size_t i = 0; i < goal.objects.size(); i++) {
    std::string oid = goal.objects[i]->header.frame_id;
    if (oid.compare(world_frame) && oid.compare(1, oid.size()-1, world_frame) &&
	world_frame.compare(1, world_frame.size()-1, oid) &&
	oid.compare(1, oid.size()-1, world_frame, 1, world_frame.size()-1)) {
      ROS_ERROR("All objects passed with goal must be in frame %s in order to do proper collision checking", world_frame.c_str());
      return false;
    }
  }
  return true;
}

bool darrt::DARRTSolver::setup_problem
(const arm_navigation_msgs::RobotState &starting_state, const Goal &goal) {

  ROS_INFO("Setting up the problem");

  solver_->clear();

  DARRTStateSpace *space =
    solver_->getStateSpace()->as<DARRTStateSpace>();

  
  //set the bounds
  for (unsigned int i = 0; i < goal.objects.size(); i++) {
    ROS_INFO("Setting bounds for %s", goal.objects[i]->id.c_str());
    space->getSubspace(goal.objects[i]->id)->as<ObjectStateSpace>()->
      setBounds(goal.params.bounds);
  }

  //now create the starting state
  if (!starting_state_) {
    ROS_INFO("Getting starting state");
    starting_state_ = space->allocState();
    //create the darrt message state corresponding to the starting state
    darrt_msgs::DARRTState dstate;
    //this doesn't allow this to be different than the actual 
    //environment interface at this point
    dstate.robot_state = starting_state;
    std::vector<an::CollisionObject> cos;
    std::vector<an::AttachedCollisionObject> aos;
    environment_interface_->getCollisionSpaceAttachedCollisionObjects(aos);
    environment_interface_->getCollisionSpaceCollisionObjects(cos);
    for (unsigned int i = 0; i < aos.size(); i++) {
      for (unsigned int j = 0; j < goal.objects.size(); j++) {
	if (!aos[i].object.id.compare(goal.objects[j]->id)) {
	  ROS_INFO("Adding attached collision object %s", 
		   goal.objects[j]->id.c_str());
	  dstate.attached_collision_objects.push_back(aos[i]);
	}
      }
    }
    for (unsigned int i = 0; i < cos.size(); i++) {
      for (unsigned int j = 0; j < goal.objects.size(); j++) {
	if (!cos[i].id.compare(goal.objects[j]->id)) {
	  ROS_INFO("Adding collision object %s", cos[i].id.c_str());
	  dstate.collision_objects.push_back(cos[i]);
	}
      }
    }
    
    space->convert_darrt_state_to_ompl_state(dstate, starting_state_);
  }
  ROS_INFO("Setting starting state");
  solver_->setStartState(ob::ScopedState<DARRTStateSpace>
			 (solver_->getStateSpace(), starting_state_));

  
  //create the goal
  ROS_INFO("Creating goal state");
  goal_.reset(goal.satisfiable_goal(solver_->getSpaceInformation(), starting_state_));
  solver_->setGoal(goal_);
  space->setSubspaceDistance(-1);
  //CompoundGoal *cgoal = dynamic_cast<CompoundGoal *>(goal_.get());
  //space->set_goal(cgoal);

  // if (debug_level >= DSILENT) {
  //   space->display_state(starting_state_, "starting_state", 0, 0.1, 
  // 			 Displayable::STARTPALATE);
  //   cgoal->display("goal_state", Displayable::GOALPALATE);
  //   ROS_INFO("Starting state is %s", space->state_string(starting_state_).c_str());
  // }
  
  return true;
}

bool darrt::DARRTSolver::setup_space(const Goal &goal) {

  for (unsigned int i = 0; i < primitives_.size(); i++) {
    delete primitives_[i];
  }
  primitives_.clear();

  //create copies of the primitives so we can initialize them
  PrimitiveList cplist; //the constant counterparts
  std::vector< std::pair<DARRTProjectionFunction, double> > pfs;
  for (unsigned int i = 0; i < goal.primitives.size(); i++) {
    primitives_.push_back(goal.primitives[i].first->copy());
    cplist.push_back(primitives_[i]);
    if (goal.primitives[i].second > EPSILON) {
      pfs.push_back(make_pair(primitives_[i]->getProjectionFunction(), 
			      goal.primitives[i].second));
    }
  }

  //we need a compound space
  //one space for each object
  //and one the robot

  ob::StateSpacePtr space_ptr
    (new DARRTStateSpace(cplist, goal.support_surfaces, pfs));
  DARRTStateSpace *space = space_ptr.get()->as<DARRTStateSpace>();
  
  ob::StateSpacePtr robot_ptr(new CompoundRobotStateSpace
			      (node_, environment_interface_));
  CompoundRobotStateSpace *robot_space = 
    robot_ptr->as<CompoundRobotStateSpace>();
  ob::RealVectorBounds bnds(2);
  for (unsigned int i = 0; i < 2; i++) {
    bnds.low[i] = goal.params.bounds.low[i];
    bnds.high[i] = goal.params.bounds.high[i];
  }
  for (unsigned int i = 0; i < goal.robot_groups.size(); i++) {
    robot_space->addSubspace(goal.robot_groups[i], bnds);
  }

  an::CollisionObject robot_obj;
  robot_obj.id = "robot";
  space->addSubspace(robot_ptr, true);

  //now add a subspace for each object
  for (size_t i = 0; i < goal.objects.size(); i++) {
    ob::StateSpacePtr object_space(new ObjectStateSpace(*(goal.objects[i])));
    space->addSubspace(object_space, false);
  }

  oc::ControlSpacePtr control_ptr(new DARRTControlSpace(space_ptr, space->primitives()));
  control_ptr->as<DARRTControlSpace>()->set_max_depth(100);

  if (solver_) {
    delete solver_;
  }

  solver_ = new oc::SimpleSetup(control_ptr);

  solver_->getSpaceInformation().get()->setMinMaxControlDuration(1, 10000);
  solver_->getSpaceInformation().get()->setPropagationStepSize(0.03);

  solver_->setStatePropagator
    (oc::StatePropagatorPtr
     (new DARRTStatePropagator(solver_->getSpaceInformation().get())));

  solver_->getSpaceInformation().get()->setDirectedControlSamplerAllocator
    (&alloc_darrt_sampler);

  for (size_t i = 0; i < primitives_.size(); i++) {
    if (!primitives_[i]->setup(solver_->getSpaceInformation().get())) {
      ROS_ERROR("Unable to set up primitive %s", primitives_[i]->name().c_str());
      return false;
    }
    ROS_INFO("Successfully set up primitive %s", primitives_[i]->name().c_str());
  }

  ob::PlannerPtr rrt_ptr;
  if (goal.params.forward) {
    rrt_ptr.reset(new ompl::control::RRT(solver_->getSpaceInformation()));
    ompl::control::RRT *rrt = dynamic_cast<ompl::control::RRT *>(rrt_ptr.get());
    ROS_INFO("Setting goal bias to be %f", goal.params.goal_bias);
    rrt->setGoalBias(goal.params.goal_bias);
    rrt->setIntermediateStates(true);
    //rrt->setNearestNeighbors<ompl::NearestNeighborsGNAT>();
    rrt->setNearestNeighbors<ompl::NearestNeighborsLinear>();
  } else {
    rrt_ptr.reset(new ompl::control::RRTConnect(solver_->getSpaceInformation()));
    ompl::control::RRTConnect *rrt = dynamic_cast<ompl::control::RRTConnect *>(rrt_ptr.get());
    rrt->setIntermediateStates(true);
    rrt->setMaxConnectionAttempts(1);
    //rrt->setNearestNeighbors<ompl::NearestNeighborsGNAT>();
    rrt->setNearestNeighbors<ompl::NearestNeighborsLinear>();
  }
  solver_->setPlanner(rrt_ptr);

  return true;
}

bool darrt::DARRTSolver::setup_state_validity_checker(const Goal &goal) {

  DARRTStateSpace *space =
    solver_->getStateSpace().get()->as<DARRTStateSpace>();
  
  //disable collisions for all the non-updated links
  //note that we want to KEEP the collisions with the objects
  //we can't use the handy function, but
  //since the collisions are ordered we can do this by first
  //disabling all collisions and then enabling the ones we want
  an::CollisionOperation op;
  op.object1 = op.COLLISION_SET_ALL;
  op.object2 = op.COLLISION_SET_ALL;
  op.operation = op.DISABLE;
  default_collisions_.collision_operations.push_back(op);
  //enable collisions for any robot links that move
  CompoundRobotStateSpace *rspace = 
    space->robot_state_space()->as<CompoundRobotStateSpace>();
  const planning_models::KinematicModel *kmodel =
    environment_interface_->getKinematicModel();
  op.object2 = op.COLLISION_SET_ALL;
  op.operation = op.ENABLE;
  for (unsigned int i = 0; i < rspace->getSubspaceCount(); i++) {
    std::string group_name = rspace->getSubspace(i)->getName();
    if (!kmodel->hasModelGroup(group_name)) {
      continue;
    }
    std::vector<std::string> updated_link_names = 
      kmodel->getModelGroup(group_name)->getUpdatedLinkModelNames();
    for (unsigned int j = 0; j < updated_link_names.size(); j++) {
      op.object1 = updated_link_names[j];
      default_collisions_.collision_operations.push_back(op);
    }
  }
  //enable collisions for all objects that move
  for (size_t i = 0; i < goal.objects.size(); i++) {
    op.object1 = goal.objects[i]->id;
    default_collisions_.collision_operations.push_back(op);
  }
  //now again disable the self collisions and other default collisions
  std::vector<an::CollisionOperation> self_collisions;
  environment_interface_->getDefaultOrderedCollisionOperations
    (self_collisions);
  for (size_t i = 0; i < self_collisions.size(); i++) {
    default_collisions_.collision_operations.push_back(self_collisions[i]);
  }
  //also disable collisions between wheels and collision map
  op.object1 = "wheels";
  op.object2 = "collision_map";
  op.operation = op.DISABLE;
  default_collisions_.collision_operations.push_back(op);

  collision_space::EnvironmentModel::AllowedCollisionMatrix acm
    = environment_interface_->getDefaultAllowedCollisionMatrix();
  //setup the default collisions
  std::vector<std::string> objnames;
  environment_interface_->getCollisionObjectNames(objnames);
  std::vector<std::string> anames;
  environment_interface_->getAttachedCollisionObjectNames(anames);
  planning_environment::applyOrderedCollisionOperationsListToACM
    (default_collisions_, objnames, anames, environment_interface_->getKinematicModel(), 
     acm);
  environment_interface_->setAlteredAllowedCollisionMatrix(acm);
  //an::AllowedCollisionMatrix matrix;
  //environment_interface_->getCollisionSpaceAllowedCollisions(matrix);
  //ROS_INFO_STREAM("Allowed collision matrix is\n" << matrix);

  //subspace validity checkers
  std::vector<ob::StateValidityCheckerPtr> checkers;
  for (unsigned int i = 0; i < space->getSubspaceCount(); i++) {
    ob::SpaceInformationPtr si_ptr
      (new ob::SpaceInformation(space->getSubspace(i)));
    ob::StateValidityCheckerPtr chkr;
    if (space->robot_index() >= 0 && 
	i == static_cast<unsigned int>(space->robot_index())) {
      chkr.reset(new RobotStateValidityChecker(si_ptr.get(), acm, false));
    } else {
      chkr.reset(new ObjectStateValidityChecker(si_ptr.get(), acm, 
						false, true));
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



bool darrt::DARRTSolver::plan(const Goal &goal, 
			      const ob::State *starting_state, 
			      float &planning_time) {
  if (!configured_) {
    ROS_ERROR("You must call configured before attempting to plan");
    return false;
  }
  if (!check_object_headers(goal)) {
    return false;
  }

  //get the starting position of the robot
  an::RobotState robot_starting_state;
  if (!starting_state) {
    starting_state_ = NULL;
    if (true || !environment_interface_->kinematicState()) {
      if (!initialize_environment(robot_starting_state, goal.objects)) {
	ROS_ERROR("Unable to initialize robot state");
	return false;
      }
    } else {
      an::GetRobotState srv;
      if (!get_robot_state_.call(srv)) {
	ROS_ERROR("Call to get robot state failed");
	return false;
      }
      robot_starting_state = srv.response.robot_state;
    }
    ROS_INFO("Succesfully initialized robot state.");
  } else {
    starting_state_ = solver_->getStateSpace().get()->allocState();
    solver_->getStateSpace().get()->copyState(starting_state_, starting_state);
  }

  if (!setup_problem(robot_starting_state, goal)) {
    ROS_ERROR("Unable to set up the problem");
    return false;
  }
  ROS_INFO("Successfully set up the problem");
  solver_->setup();
  ROS_INFO("Beginning solving (time limit: %f).  Starting time: ", info_.darrt_allowed_time);
  phase_ = SOLVING;
  //ROS_INFO("In solving goal bias is %f", solver_->getPlanner()->as<oc::RRT>()->getGoalBias());
  
  bool solved;
  try {
    boost::thread *interactive_thread = NULL;
    if (goal.params.interactive) {
      interactive_thread = 
	new boost::thread(boost::bind(&DARRTSolver::listen, this));
    } 
    ros::WallTime start = ros::WallTime::now();
    ROS_INFO_STREAM("Starting time:\n" << start);
    //CALLGRIND_START_INSTRUMENTATION;
    solved = solver_->getPlanner().get()->
      solve(terminate_after(info_.darrt_allowed_time));
    //CALLGRIND_STOP_INSTRUMENTATION;
    ros::WallTime end = ros::WallTime::now();
    ROS_INFO_STREAM("Ending time:\n" << end);
    planning_time = (end - start).toSec();
    if (interactive_thread) {
      delete interactive_thread;
    }
  } catch(ompl::Exception &e) {
    ROS_ERROR("Exception during solving: %s", e.what());
    solved = false;
  }
  ROS_INFO("Returned from solving.  Success: %d, time: %f", solved &&
	   solver_->haveExactSolutionPath(), planning_time);
  phase_ = READY;

  solver_->getStateSpace().get()->freeState(starting_state_);
  return (solved && solver_->haveExactSolutionPath());
}

bool darrt::DARRTSolver::execute_solution() const {
  return execute_solution(solver_->getSolutionPath());
}
 
bool darrt::DARRTSolver::execute_solution(const oc::PathControl &path) const{
  PrimitivePath prim_path;
  convert_to_primitive_path(path, prim_path);
  for (size_t i = 0; i < prim_path.size(); i++) {
    ROS_INFO("Executing primitive %s", prim_path[i].primitive->str().c_str());
    if (!prim_path[i].primitive->execute(prim_path[i].states)) {
      return false;
    }
  }
  return true;
}

void darrt::DARRTSolver::listen() {
  DARRTControlSpace *cspace = 
    solver_->getControlSpace()->as<DARRTControlSpace>();
  ROS_INFO("Starting interactive control");
  while (ros::ok()) {
    std::string buff;
    getline(std::cin, buff);
    switch(buff[0]) {
    case 'p':
    case 'P':
      {
	cspace->pause();
	ROS_INFO("Welcome to interactive control.  d to enter a debugging level, u to unpause, r to disable pauses");
	break;
      }
    case 'u':
    case 'U':
      {
	ROS_INFO("Exiting interactive control");
	cspace->unpause();
	break;
      }
    case 'd':
    case 'D':
      {
	ROS_INFO("Enter a debug level you would like to change to");
	std::string level;
	getline(std::cin, level);
	try {
	  int d = boost::lexical_cast<int>(level.c_str());
	  debug_level = d;
	  do_pause = true;
	  ROS_INFO("Set debug level to %d and do_pause to true", d);
	} catch (boost::bad_lexical_cast &) {
	  ROS_ERROR("Unable to interpret argument for debugging level");
	}
	break;
      }
    case 'r':
    case 'R':
      {
	ROS_INFO("Disabling pauses in solving");
	do_pause = false;
      }
    default:
      break;
    }
  }
}


