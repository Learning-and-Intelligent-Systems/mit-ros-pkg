#include "darrt_actions/darrt_action.hh"

#include "darrt/control.hh"
#include "darrt/object_space.hh"
#include "darrt/pr2_arm_primitives.hh"
#include "darrt/pr2_base_primitives.hh"
#include "darrt/solver.hh"
#include "darrt/transform_ros_types.hh"
#include "darrt/utils.hh"

#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/GetRobotState.h>

#include <stdexcept>

darrt_actions::DARRTAction::DARRTAction(ros::NodeHandle &n) : 
  node_(n), 
  action_server_(node_, std::string("darrt_action"),
		 boost::bind(&darrt_actions::DARRTAction::goalCB, this, _1), false),
  solver_(node_),
  cluster_grasp_planner_(node_, getClusterGraspPlannerName()),
  database_grasp_planner_(node_, getDatabaseGraspPlannerName())
{

  srand(time(NULL));
  action_server_.registerPreemptCallback(boost::bind(&darrt_actions::DARRTAction::preemptCB, this));

  scene_client_ = node_.serviceClient<an::GetPlanningScene>("/environment_server/get_planning_scene");
  ROS_INFO("Waiting for get planning scene service");
  scene_client_.waitForExistence();

  robot_client_ = node_.serviceClient<an::GetRobotState>("/environment_server/get_robot_state");
  ROS_INFO("Waiting for robot state service");
  robot_client_.waitForExistence();

  ROS_INFO("Waiting for cluster grasp planner");
  cluster_grasp_planner_.waitForServer();
  
  ROS_INFO("Waiting for database grasp planner");
  database_grasp_planner_.waitForServer();


  std::vector<std::string> arm_names;
  arm_names.push_back("right_arm");
  arm_names.push_back("left_arm");
  for (unsigned int i = 0; i < arm_names.size(); i++) {
    touch_links_[arm_names[i]] = std::vector<std::string>(1, arm_names[i].substr(0,1)+"_end_effector");
    std::string param_name = "/hand_description/"+arm_names[i]+"/hand_touch_links";
    XmlRpc::XmlRpcValue list;
    std::vector<std::string> tl;
    if (node_.getParam(param_name, list) && list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int32_t j = 0; j < list.size(); j++) {
	if (list[j].getType() == XmlRpc::XmlRpcValue::TypeString) {
	  tl.push_back(static_cast<std::string>(list[j]));
	}
      }
    }
    if (tl.size()) {
      touch_links_[arm_names[i]] = tl;
    }
    std::vector<double> sj;
    if (node_.getParam("/arm_configurations/side_tuck/trajectory/"+arm_names[i], list) && 
	list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int32_t j = 0; j < list.size(); j++) {
	if (list[j].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
	  sj.push_back(static_cast<double>(list[j]));
	}
      }
    } else {
      sj.resize(7);
      double sgn = 1.0;
      if (arm_names[i][0] == 'r') {
	sgn = -1.0;
      }
      sj[0] = sgn*2.1;
      sj[1] = 1.26;
      sj[2] = sgn*1.8;
      sj[3] = -1.9;
      sj[4] = sgn*-3.5;
      sj[5] = -1.8;
      sj[6] = darrt::MATH_PI/2.0;
    }
    side_states_[arm_names[i]] = sj;

    node_.param<std::string>("/hand_description/"+arm_names[i]+"/attach_link", attach_links_[arm_names[i]],
			     arm_names[i].substr(0,1)+"_gripper_r_finger_tip_link");
  }

  ik_frame_ = "torso_lift_link";
  an::GetRobotState srv;
  if (!robot_client_.call(srv)) {
    ROS_ERROR("Unable to get robot state!");
    return;
  }
  world_frame_ = srv.response.robot_state.multi_dof_joint_state.frame_ids[0];
  if (world_frame_[0] == '/') {
    world_frame_ = world_frame_.substr(1, world_frame_.size());
  }
  ROS_INFO("World frame is %s", world_frame_.c_str());

  action_server_.start();
  ROS_INFO("DARRT action started");
}

darrt_actions::DARRTAction::~DARRTAction() {}

std::string darrt_actions::DARRTAction::getClusterGraspPlannerName() {
  std::string name;
  node_.param<std::string>("/object_manipulator/default_cluster_planner", name, "/plan_point_cluster_grasp");
  return name;
}

std::string darrt_actions::DARRTAction::getDatabaseGraspPlannerName() {
  std::string name;
  node_.param<std::string>("/object_manipulator/default_database_planner", name, "/database_grasp_planning");
  return name;
}

void darrt_actions::DARRTAction::goalCB(const darrt_msgs::DARRTGoalConstPtr &goal_ptr) {

  darrt::Goal goal;

  darrt_msgs::DARRTResult result;


  //set up parameters
  goal.params.use_darrt = !goal_ptr->hierarchical;
  goal.params.interactive = goal_ptr->interactive;
  goal.params.darrt_tries = goal_ptr->darrt_tries;
  goal.params.forward = goal_ptr->forward_planning;
  if (!goal.params.darrt_tries) {
    goal.params.darrt_tries = 1;
  }
  int ntries = goal_ptr->tries;
  if (goal_ptr->planning_time <= 0.001) {
    if (ntries <= 0) {
      ntries = 5;
      goal.params.darrt_allowed_time = 10;
    } else {
      //the planning time stays with the default
      ntries = 1;
    }
  } else {
    goal.params.darrt_allowed_time = goal_ptr->planning_time;
  }
  if (goal_ptr->object_planning_time > 0.001) {
    goal.params.object_allowed_time = goal_ptr->object_planning_time;
  }
  if (ntries <= 0) {
    ntries = 1;
  }
  
  if (goal_ptr->goal_bias > 0.001) {
    goal.params.goal_bias = goal_ptr->goal_bias;
  }
  if (goal_ptr->threshold > darrt::EPSILON) {
    goal.params.darrt_threshold = goal_ptr->threshold;
    goal.params.object_threshold = goal_ptr->threshold;
  }
  darrt::debug_level = goal_ptr->debug_level;
  darrt::do_pause = goal_ptr->do_pause;// || goal.params.interactive;
  if (darrt::do_pause) {
    if (goal.params.darrt_allowed_time < 1000) {
      goal.params.darrt_allowed_time = 1000;
    }
    if (goal.params.object_allowed_time < 1000) {
      goal.params.object_allowed_time = 1000;
    }
  }

  an::RobotState starting_state = goal_ptr->robot_state;
  if (!goal_ptr->robot_state.joint_state.name.size()) {
    an::GetRobotState srv;
    if (!robot_client_.call(srv)) {
      ROS_ERROR("Unable to get robot state");
      result.error_code = result.OTHER_ERROR;
      action_server_.setAborted(result);
      return;
    }
    starting_state = srv.response.robot_state;
  }



  //set up the goal for the objects
  bool has_goal = false;
  std::vector<gm::Pose> goal_poses; //for bounds
  for (unsigned int i = 0; i < goal_ptr->goals.size(); i++) {
    std::string oid = goal_ptr->goals[i].objid;
    if (!goal_ptr->goals[i].poses.size()) {
      ROS_WARN("Goal given for object %s, but no poses specified... skipping",
	       oid.c_str());
      continue;
    }
    has_goal = true;
    goal[oid] = darrt::SingleGoal();
    if (goal_ptr->goals[i].use_orientation) {
      goal[oid].type = darrt::SingleGoal::POSES;
    } else {
      goal[oid].type = darrt::SingleGoal::ROTATIONALLY_SYMMETRIC_POSES;
    }
    goal[oid].threshold = goal.params.darrt_threshold;
    for (unsigned int j = 0; j < goal_ptr->goals[i].poses.size(); j++) {
      gm::PoseStamped plw = transformer_.transform(transformer_.world_frame_id(),
						   goal_ptr->goals[i].poses[j], starting_state);
      goal[oid].poses.push_back(plw.pose);
      goal_poses.push_back(plw.pose);
    }
  }

  if (!has_goal) {
    ROS_ERROR("No goals specified!");
    result.error_code = result.NO_PLACE_LOCATIONS;
    action_server_.setAborted(result);
    return;
  }

  //set up the world (objects and support surfaces)
  for (unsigned int  i = 0; i < goal_ptr->objects.size(); i++) {
    goal.objects.push_back(getObject(goal_ptr->objects[i], goal_ptr->support_surfaces));
  }
  for (unsigned int i = 0; i < goal_ptr->support_surfaces.size(); i++) {
    darrt::MeshSurface *table = new darrt::MeshSurface(goal_ptr->support_surfaces[i]);
    if (getTable(*table)) {
      goal.support_surfaces.push_back(table);
    } else {
      delete table;
      ROS_WARN("Unable to find table  %s in collision map.  Will not be able to place on it.",
	       goal_ptr->support_surfaces[i].c_str());
    }
  }


  ob::RealVectorBounds bnds(3);
  if (goal_ptr->lower_bounds.size() != 3 || goal_ptr->upper_bounds.size() != 3) {
    ROS_INFO("Bounds not set");
    goal.params.bounds = getBounds(goal.support_surfaces, goal_poses, starting_state);
  } else {
    goal.params.bounds.low.resize(3);
    goal.params.bounds.high.resize(3);
    for (unsigned int i = 0; i < 3; i++) {
      goal.params.bounds.low[i] = goal_ptr->lower_bounds[i];
      goal.params.bounds.high[i] = goal_ptr->upper_bounds[i];
    }
  }

  for (darrt::Goal::iterator git = goal.begin(); git != goal.end(); git++) {
    git->second.bounds = goal.params.bounds;
  }

  ROS_INFO("Initializing primitives");
  for (unsigned int i = 0; i < goal_ptr->primitives.size(); i++) {
    if (!initializePrimitive(goal_ptr->primitives[i], goal)) {
      ROS_ERROR("Unable to initialize primitive %s!", goal_ptr->primitives[i].name.c_str());
    }
  }

  bool solved = false;
  bool success = false;
  for (int i = 0; i < ntries; i++) {
    ROS_INFO("Planning attempt %d", i);
    float time, object_time;
    std::vector<float> subgoal_times;
    //i'm pretty sure we only need to do this once
    //but it makes me feel better to do it every time :)
    try {
      ROS_INFO("Configuring for goal");
      success = solver_.configure(goal);
      ROS_INFO("Planning for goal");
      solved = solver_.plan(goal, time, object_time, subgoal_times);
    } catch(std::runtime_error &e) {
      success = false;
    }
    result.planning_time += time;
    result.object_time += object_time;
    for (unsigned int j = 0; j < subgoal_times.size(); j++) {
      if (result.subgoal_time.size() <= j) {
	result.subgoal_time.push_back(subgoal_times[j]);
      } else {
	result.subgoal_time[j] += subgoal_times[j];
      }
    }
    if (!success || !action_server_.isActive()) {
      //arm_space->freeState(side_state);
      solver_.reset();
      freeMemory(goal);
      if (!success) {
	result.error_code = result.PLANNING_FAILED;
	action_server_.setAborted(result);
      }
      return;
    }
    if (solved) {
      break;
    }
  }
  if (!solved) {
    ROS_ERROR("Unable to plan");
    //arm_space->freeState(side_state);
    solver_.reset();
    freeMemory(goal);
    result.error_code = result.PLANNING_FAILED;
    action_server_.setAborted(result);
    return;
  }
  const oc::PathControl *path = solver_.getSolutionPath();
  if (goal_ptr->visualize) {
    darrt::pause("Ready to see path?", 0, true);
    solver_.darrt_solver().display_solution(*path, true);
  }

  darrt::PrimitivePath prim_path;
  darrt::convert_to_primitive_path(*(solver_.getSolutionPath()), prim_path);
  const darrt::DARRTStateSpace *space = solver_.darrt_solver().space_information()->
    getStateSpace()->as<darrt::DARRTStateSpace>();
  const darrt::RobotStateSpace *robot_space = space->robot_state_space()->as<darrt::RobotStateSpace>();
  unsigned int objind = space->object_indexes().at(0);
  const darrt::ObjectStateSpace *object_space = space->object_state_space(objind)->as<darrt::ObjectStateSpace>();

  an::RobotState last_state = starting_state;
  //gm::Pose last_object_state = to_pickup.poses[0];
  gm::Pose last_object_state;
  bool set_object_state = false;
  for (unsigned int i = 0; i < prim_path.size(); i++) {
    //not the best way to do this but oh well
    if (!set_object_state && prim_path[i].states.size()) {
      object_space->object_position(space->get_state(objind, prim_path[0].states[0]), last_object_state);
      set_object_state = true;
    }
    result.primitive_names.push_back(prim_path[i].primitive->name());
    result.primitive_types.push_back(prim_path[i].type);
    an::RobotTrajectory traj;
    darrt_msgs::ObjectTrajectory obj_traj;
    obj_traj.header.frame_id = transformer_.world_frame_id();
    //double up so that the first state on the trajectory is always where the robot will actually be
    traj.joint_trajectory.header = last_state.joint_state.header;
    traj.joint_trajectory.header.stamp = ros::Time(0);
    traj.joint_trajectory.joint_names = last_state.joint_state.name;
    traj.multi_dof_joint_trajectory.joint_names = last_state.multi_dof_joint_state.joint_names;
    traj.multi_dof_joint_trajectory.frame_ids = last_state.multi_dof_joint_state.frame_ids;
    traj.multi_dof_joint_trajectory.child_frame_ids = last_state.multi_dof_joint_state.child_frame_ids;
    for (int j = -1; j < static_cast<int>(prim_path[i].states.size()); j++) {
      an::MultiDOFJointState prev_pt = last_state.multi_dof_joint_state;
      if (j >= 0) {
	robot_space->convert_ompl_to_robot_state(space->robot_state(prim_path[i].states[j]), last_state);
	object_space->object_position(space->get_state(objind, prim_path[i].states[j]), last_object_state);
      }   
      trajectory_msgs::JointTrajectoryPoint jp;
      jp.positions = last_state.joint_state.position;
      an::MultiDOFJointTrajectoryPoint mdofp;
      mdofp.poses = last_state.multi_dof_joint_state.poses;
      traj.joint_trajectory.points.push_back(jp);
      traj.multi_dof_joint_trajectory.points.push_back(mdofp);
      obj_traj.trajectory.push_back(last_object_state);
    }
    result.primitive_trajectories.push_back(traj);
    result.object_trajectories.push_back(obj_traj);
  }
  //REALLY important to reset the solver BEFORE we delete the primitives!
  //arm_space->freeState(side_state);
  solver_.reset();
  freeMemory(goal);
  result.error_code = result.SUCCESS;
  action_server_.setSucceeded(result);
}
  

void darrt_actions::DARRTAction::preemptCB() {
  ROS_INFO("DARRT Action: Caught cancel signal");
  if (!solver_.cancel(ros::WallDuration(CANCEL_TIMEOUT))) {
    ROS_WARN("Unable to cancel DARRTH in %f seconds", CANCEL_TIMEOUT);
  }
}

void darrt_actions::DARRTAction::freeMemory(darrt::Goal &goal) {
  for (unsigned int i = 0; i < goal.support_surfaces.size(); i++) {
    delete goal.support_surfaces[i];
  }
  for (unsigned int i = 0; i < goal.primitives.size(); i++) {
    delete goal.primitives[i].first;
  }
  for (unsigned i = 0; i < goal.objects.size(); i++) {
    delete goal.objects[i];
  }
}

darrt::DARRTObject *darrt_actions::DARRTAction::getObject(const darrt_msgs::ObjectType &object_msg,
							  const std::vector<std::string> &support_surfaces) 
{
  std::vector<std::string> ss = object_msg.parameters;
  if (!ss.size()) {
    ss = support_surfaces;
  }

  if (object_msg.type == "FixedObject") {
    an::CollisionObject fo;
    getObject(object_msg.collision_object.id, fo);
    gm::Pose origin;
    origin.orientation.w = 1.0;
    gm::Pose spose = darrt::transform_pose(darrt::inverse_transform_pose
					   (origin, object_msg.collision_object.poses[0]),
					   fo.poses[0]);
    ROS_INFO_STREAM("Starting pose of fixed object is header: " << fo.header.frame_id << ", Pose\n" << spose
		    << "first pose =\n" << fo.poses[0] << "\nco pose =\n" <<
		    object_msg.collision_object.poses[0]);
    darrt::FixedObject *fobject = new darrt::FixedObject(ss, spose);
    fobject->copyCollisionObject(object_msg.collision_object);
    return fobject;
  }

  if (object_msg.type == "RoundObject") {
    darrt::RoundObject *robject = new darrt::RoundObject(ss);
    robject->copyCollisionObject(object_msg.collision_object);
    return robject;
  }
  if (object_msg.type == "SpatulaObject") {
    std::vector<double> pdims(3), hdims(2);
    for (unsigned int i = 0; i < 3; i++) {
      pdims[i] = object_msg.numeric_parameters[i];
      if (i < 2) {
	hdims[i] = object_msg.numeric_parameters[3+i];
      }
    }
    //find its starting pose
    an::CollisionObject spat;
    getObject(object_msg.collision_object.id, spat);
    gm::Pose origin;
    origin.orientation.w = 1.0;
    gm::Pose spose = darrt::transform_pose(darrt::inverse_transform_pose
					   (origin, object_msg.collision_object.poses[0]),
					   spat.poses[0]);
    ROS_INFO_STREAM("Starting pose of spatula is header: " << spat.header.frame_id << ", Pose\n" << spose
		    << "first pose =\n" << spat.poses[0] << "\nco pose =\n" <<
		    object_msg.collision_object.poses[0]);
    return new darrt::SpatulaObject(spat.id, spat.header.frame_id, ss, pdims, hdims, 
				    object_msg.numeric_parameters[5], spose);
  }
  darrt::DARRTObject *dobject = new darrt::DARRTObject(ss);
  dobject->copyCollisionObject(object_msg.collision_object);
  if (object_msg.type != "DARRTObject") {
    darrt::pause("Unable to initialize object of type "+object_msg.type);
  }
  return dobject;
}
    
    
 
bool darrt_actions::DARRTAction::initializePrimitive(const darrt_msgs::Primitive &primitive,
						     darrt::Goal &goal) const {
  if (primitive.name == "ArmTransit") {
    return initializeArmTransit(primitive, goal);
  }
  if (primitive.name == "BaseTransit") {
    return initializeBaseTransit(primitive, goal);
  }
  if (primitive.name == "Approach") {
    return initializeApproach(primitive, goal);
  }
  if (primitive.name == "Retreat") {
    return initializeRetreat(primitive, goal);
  }
  if (primitive.name == "RigidTransfer") {
    return initializeRigidTransfer(primitive, goal);
  }
  if (primitive.name == "Push") {
    return initializePush(primitive, goal);
  }
  if (primitive.name == "Pickup") {
    return initializePickup(primitive, goal);
  }
  if (primitive.name == "UseSpatula") {
    return initializeUseSpatula(primitive, goal);
  }
  if (primitive.name == "SpatulaTransfer") {
    return initializeSpatulaTransfer(primitive, goal);
  }
  ROS_ERROR("Unable to find primitive %s", primitive.name.c_str());
  return false;
}

bool darrt_actions::DARRTAction::initializeArmTransit(const darrt_msgs::Primitive &primitive, darrt::Goal &goal) const {
  darrt::PR2ArmTransit *transit = new darrt::PR2ArmTransit(primitive.parameters[0]);

  //add the robot group if necessary
  bool has_group = false;
  for (unsigned int i = 0; i < goal.robot_groups.size(); i++) {
    if (goal.robot_groups[i] == primitive.parameters[0]) {
      has_group = true;
      break;
    }
  }
  if (!has_group) {
    goal.robot_groups.push_back(primitive.parameters[0]);
  }

  //also add the projection function
  //really should only do this once
  //but not sure how since the base one doesn't work
  goal.primitives.push_back(make_pair(transit, 1));
  ROS_INFO("Successfully arm transit primitive.");
  return true;
}

bool darrt_actions::DARRTAction::initializeBaseTransit(const darrt_msgs::Primitive &primitive, darrt::Goal &goal) const {
  darrt::PR2BaseTransit *transit = new darrt::PR2BaseTransit(primitive.parameters[0]);

  //add the robot group if necessary
  bool has_group = false;
  for (unsigned int i = 0; i < goal.robot_groups.size(); i++) {
    if (goal.robot_groups[i] == primitive.parameters[0]) {
      has_group = true;
      break;
    }
  }
  if (!has_group) {
    goal.robot_groups.push_back(primitive.parameters[0]);
  }

  goal.primitives.push_back(make_pair(transit, 0));  
  ROS_INFO("Successfully initialized BaseTransit primitive.");
  return true;
}

bool darrt_actions::DARRTAction::initializeApproach(const darrt_msgs::Primitive &primitive, darrt::Goal &goal) const {
  darrt::Approach *approach;
  if (primitive.numeric_parameters.size() == 0) {
    approach = new darrt::Approach(primitive.parameters[0]);
  } else if (primitive.numeric_parameters.size() == 1) {
    approach = new darrt::Approach(primitive.parameters[0], primitive.numeric_parameters[0]);
    
  } else {
    approach = new darrt::Approach(primitive.parameters[0],
				   primitive.numeric_parameters[0],
				   primitive.numeric_parameters[1]);
  }

  //add the robot group if necessary
  bool has_group = false;
  for (unsigned int i = 0; i < goal.robot_groups.size(); i++) {
    if (goal.robot_groups[i] == primitive.parameters[0]) {
      has_group = true;
      break;
    }
  }
  if (!has_group) {
    goal.robot_groups.push_back(primitive.parameters[0]);
  }

  goal.primitives.push_back(make_pair(approach, 0));  
  ROS_INFO("Successfully initialized Approach primitive.");
  return true;
}

bool darrt_actions::DARRTAction::initializeRetreat(const darrt_msgs::Primitive &primitive, darrt::Goal &goal) const {
  darrt::Retreat *retreat;
  
  if (primitive.numeric_parameters.size() == 0) {
    retreat = new darrt::Retreat(primitive.parameters[0]);
  } else if (primitive.numeric_parameters.size() == 1) {
    retreat = new darrt::Retreat(primitive.parameters[0], primitive.numeric_parameters[0]);
    
  } else {
    retreat = new darrt::Retreat(primitive.parameters[0],
				 primitive.numeric_parameters[0],
				 primitive.numeric_parameters[1]);
  }


  //add the robot group if necessary
  bool has_group = false;
  for (unsigned int i = 0; i < goal.robot_groups.size(); i++) {
    if (goal.robot_groups[i] == primitive.parameters[0]) {
      has_group = true;
      break;
    }
  }
  if (!has_group) {
    goal.robot_groups.push_back(primitive.parameters[0]);
  }

  goal.primitives.push_back(make_pair(retreat, 0));  
  ROS_INFO("Successfully initialized Retreat primitive.");
  return true;
}

bool darrt_actions::DARRTAction::initializeRigidTransfer(const darrt_msgs::Primitive &primitive, darrt::Goal &goal) const {
  darrt::RigidTransfer *transfer = new darrt::RigidTransfer(primitive.parameters[0], primitive.parameters[1]);
  
  //add the robot group if necessary
  bool has_group = false;
  for (unsigned int i = 0; i < goal.robot_groups.size(); i++) {
    if (goal.robot_groups[i] == primitive.parameters[0]) {
      has_group = true;
      break;
    }
  }
  if (!has_group) {
    goal.robot_groups.push_back(primitive.parameters[0]);
  }

  goal.primitives.push_back(make_pair(transfer, 1));  
  ROS_INFO("Successfully initialized RigidTransfer %s primitive", primitive.parameters[1].c_str());
  return true;
}

bool darrt_actions::DARRTAction::initializePush(const darrt_msgs::Primitive &primitive, darrt::Goal &goal) const {
  //the parameters here should be: arm name, object name, touching links
  //i'm sure this is a more efficient way to do this by it doesn't really matter
  std::vector<std::string> pushing_links(primitive.parameters.size()-2);
  for (unsigned int i = 2; i < primitive.parameters.size(); i++) {
    pushing_links[i-2] = primitive.parameters[i];
  }

  darrt::Push *push;
  if (primitive.numeric_parameters.size() == 0) {
    push = new darrt::Push(primitive.parameters[0], primitive.parameters[1], pushing_links);
  } else {
    push = new darrt::Push(primitive.parameters[0], primitive.parameters[1], pushing_links,
			   primitive.numeric_parameters[0]);
  }

  //add the robot group if necessary
  bool has_group = false;
  for (unsigned int i = 0; i < goal.robot_groups.size(); i++) {
    if (goal.robot_groups[i] == primitive.parameters[0]) {
      has_group = true;
      break;
    }
  }
  if (!has_group) {
    goal.robot_groups.push_back(primitive.parameters[0]);
  }

  goal.primitives.push_back(make_pair(push, 1));  
  ROS_INFO("Successfully initialized Push %s primitive", primitive.parameters[1].c_str());
  return true;
}

bool darrt_actions::DARRTAction::initializePickup(const darrt_msgs::Primitive &primitive, darrt::Goal &goal) const {
  darrt::PickupPrimitive::RigidGraspList grasps;
  for (unsigned int i = 0; i < primitive.grasps.size(); i++) {
    grasps.push_back(darrt::PickupPrimitive::RigidGrasp
		     (primitive.grasps[i].transform, primitive.grasps[i].touch_links,
		      primitive.grasps[i].attach_link, primitive.grasps[i].min_approach_distance,
		      primitive.grasps[i].desired_approach_distance,
		      primitive.grasps[i].min_distance_from_surface));
  }

  darrt::PickupPrimitive *pickup;
  if (primitive.numeric_parameters.size() == 0) {
    pickup = new darrt::PickupPrimitive(primitive.parameters[0], primitive.parameters[1], grasps,
					primitive.directional_parameters[0]);
  } else if (primitive.numeric_parameters.size() == 1) {
    pickup = new darrt::PickupPrimitive(primitive.parameters[0], primitive.parameters[1], grasps,
					primitive.directional_parameters[0], 
					primitive.numeric_parameters[0]);
  } else {
    pickup = new darrt::PickupPrimitive(primitive.parameters[0], primitive.parameters[1], grasps,
					primitive.directional_parameters[0], 
					primitive.numeric_parameters[0],
					primitive.numeric_parameters[1]);
  }
  //add the robot group if necessary
  bool has_group = false;
  for (unsigned int i = 0; i < goal.robot_groups.size(); i++) {
    if (goal.robot_groups[i] == primitive.parameters[0]) {
      has_group = true;
      break;
    }
  }
  if (!has_group) {
    goal.robot_groups.push_back(primitive.parameters[0]);
  }

  goal.primitives.push_back(make_pair(pickup, 0));  
  ROS_INFO("Successfully initialized Pickup %s primitive", primitive.parameters[1].c_str());
  return true;
}


bool darrt_actions::DARRTAction::initializeUseSpatula(const darrt_msgs::Primitive &primitive, darrt::Goal &goal) const {
  darrt::PickupPrimitive::RigidGraspList grasps;
  for (unsigned int i = 0; i < primitive.grasps.size(); i++) {
    grasps.push_back(darrt::PickupPrimitive::RigidGrasp
		     (primitive.grasps[i].transform, primitive.grasps[i].touch_links,
		      primitive.grasps[i].attach_link, primitive.grasps[i].min_approach_distance,
		      primitive.grasps[i].desired_approach_distance,
		      primitive.grasps[i].min_distance_from_surface));
  }

  darrt::UseSpatula *uspat;
  if (primitive.numeric_parameters.size() == 0) {
    uspat = new darrt::UseSpatula(primitive.parameters[0], primitive.parameters[1], 
				  primitive.parameters[2], primitive.parameters[3],
				  grasps, primitive.directional_parameters[0]);
  } else if (primitive.numeric_parameters.size() == 1) {
    uspat = new darrt::UseSpatula(primitive.parameters[0], primitive.parameters[1], 
				  primitive.parameters[2], primitive.parameters[3],
				  grasps, primitive.directional_parameters[0], 
				  primitive.numeric_parameters[0]);
  } else if (primitive.numeric_parameters.size() == 2) {
    uspat = new darrt::UseSpatula(primitive.parameters[0], primitive.parameters[1], 
				  primitive.parameters[2], primitive.parameters[3],
				  grasps, primitive.directional_parameters[0],
				  primitive.numeric_parameters[0],
				  primitive.numeric_parameters[1]);
  } else if (primitive.numeric_parameters.size() == 3) {
    uspat = new darrt::UseSpatula(primitive.parameters[0], primitive.parameters[1], 
				  primitive.parameters[2], primitive.parameters[3],
				  grasps, primitive.directional_parameters[0],
				  primitive.numeric_parameters[0],
				  primitive.numeric_parameters[1],
				  primitive.numeric_parameters[2]);
  }  else if (primitive.numeric_parameters.size() == 2) {
    uspat = new darrt::UseSpatula(primitive.parameters[0], primitive.parameters[1], 
				  primitive.parameters[2], primitive.parameters[3],
				  grasps, primitive.directional_parameters[0],
				  primitive.numeric_parameters[0],
				  primitive.numeric_parameters[1],
				  primitive.numeric_parameters[2],
				  primitive.numeric_parameters[3]);
  } else {
    uspat = new darrt::UseSpatula(primitive.parameters[0], primitive.parameters[1], 
				  primitive.parameters[2], primitive.parameters[3],
				  grasps, primitive.directional_parameters[0],
				  primitive.numeric_parameters[0],
				  primitive.numeric_parameters[1],
				  primitive.numeric_parameters[2],
				  primitive.numeric_parameters[3],
				  primitive.numeric_parameters[4]);
  }
  
  //add the robot group if necessary
  bool has_group = false;
  for (unsigned int i = 0; i < goal.robot_groups.size(); i++) {
    if (goal.robot_groups[i] == primitive.parameters[0]) {
      has_group = true;
      break;
    }
  }
  if (!has_group) {
    goal.robot_groups.push_back(primitive.parameters[0]);
  }

  goal.primitives.push_back(make_pair(uspat, 0));  
  ROS_INFO("Successfully initialized UseSpatula primitive");
  return true;
}

bool darrt_actions::DARRTAction::initializeSpatulaTransfer(const darrt_msgs::Primitive &primitive, darrt::Goal &goal) const {
  darrt::SpatulaTransfer *strans = new darrt::SpatulaTransfer(primitive.parameters[0], primitive.parameters[1],
							      primitive.parameters[2]);
  
  //add the robot group if necessary
  bool has_group = false;
  for (unsigned int i = 0; i < goal.robot_groups.size(); i++) {
    if (goal.robot_groups[i] == primitive.parameters[0]) {
      has_group = true;
      break;
    }
  }
  if (!has_group) {
    goal.robot_groups.push_back(primitive.parameters[0]);
  }

  goal.primitives.push_back(make_pair(strans, 1));  
  ROS_INFO("Successfully initialized SpatulaTransfer primitive");
  return true;
}

bool darrt_actions::DARRTAction::getGrasps(const object_manipulation_msgs::PickupGoal &pg, 
					   const gm::PoseStamped &object_pose, 
					   double min_distance, const an::RobotState &starting_state,
					   darrt::PickupPrimitive::RigidGraspList &grasps) {
  ROS_INFO_STREAM("object pose is\n" << object_pose);
  if (min_distance < 0.0001) {
    min_distance = -1;
  }
  std::vector<object_manipulation_msgs::Grasp> om_grasps = pg.desired_grasps;
  if (!om_grasps.size()) {
    object_manipulation_msgs::GraspPlanningGoal gg;
    gg.arm_name = pg.arm_name;
    gg.target = pg.target;
    gg.collision_object_name = pg.collision_object_name;
    gg.collision_support_surface_name = pg.collision_support_surface_name;
    object_manipulation_msgs::GraspPlanningResultConstPtr result;
    bool database_grasps = false;
    if (pg.target.potential_models.size()) {
      database_grasp_planner_.sendGoalAndWait(gg);
      result = database_grasp_planner_.getResult();
      if (result->error_code.value == result->error_code.SUCCESS && result->grasps.size()) {
	database_grasps = true;
      }
    }
    if (!database_grasps) {
      cluster_grasp_planner_.sendGoalAndWait(gg);
      result = cluster_grasp_planner_.getResult();
    }
    om_grasps = result->grasps;
  }

  //convert the object manipulation messages grasps to frame free rigid grasps
  for (unsigned int i = 0; i < om_grasps.size(); i++) {
    gm::PoseStamped gps;
    gps.header.frame_id = pg.target.reference_frame_id;
    gps.header.stamp = ros::Time(0);
    gps.pose = om_grasps[i].grasp_pose;
    ros::Rate r(10);
    for (int j = 0; j < 5; j++) {
      try {	
	gm::PoseStamped wrist_pose;
	wrist_pose = transformer_.transform(object_pose.header.frame_id, gps, starting_state);
	gm::Transform grasp = darrt::pose_to_transform(darrt::inverse_transform_pose(object_pose.pose, 
										     wrist_pose.pose));
	std::vector<std::string> tl(1, pg.arm_name.substr(0,1)+"_end_effector");
	std::map<std::string, std::vector<std::string> >::const_iterator tlit = touch_links_.find(pg.arm_name);
	if (tlit != touch_links_.end()) {
	  tl = tlit->second;
	}
	std::string al = pg.arm_name.substr(0,1)+"_gripper_r_finger_tip_link";
	std::map<std::string, std::string>::const_iterator alit = attach_links_.find(pg.arm_name);
	if (alit != attach_links_.end()) {
	  al = alit->second;
	}
	grasps.push_back(darrt::PickupPrimitive::RigidGrasp(grasp, tl, al, om_grasps[i].min_approach_distance, 
							    om_grasps[i].desired_approach_distance, min_distance));
	break;
      } catch (tf::TransformException &e) {
	ROS_WARN("Unable to transform grasp pose: exception was %s", e.what());
      }
      r.sleep();
    }
  }
  if (!grasps.size()) {
    ROS_ERROR("Unable to get any grasps");
    return false;
  }
  return true;
}

bool darrt_actions::DARRTAction::getObject(std::string name, an::CollisionObject &obj) {
  an::GetPlanningScene srv;
  if (!scene_client_.call(srv)) {
    ROS_ERROR("Unable to get planning scene");
    return false;
  }
  std::vector<an::CollisionObject> &objects = srv.response.planning_scene.collision_objects;
  for (size_t i = 0; i < objects.size(); i++) {
    if (!objects[i].id.compare(name)) {
      obj = objects[i];
      return true;
    }
  }
  ROS_ERROR("Cannot find object %s in collision map", name.c_str());
  return false;
}

bool darrt_actions::DARRTAction::getTable(darrt::MeshSurface &table) {
  an::CollisionObject ctable;
  if (!getObject(table.name(), ctable)) {
    return false;
  }
  if (!ctable.shapes.size() || ctable.shapes[0].type != an::Shape::MESH) {
    ROS_ERROR("Passed in table is not a mesh in collision map!  Only meshes can be used as support surfaces.");
    return false;
  }
  table.set_pose_and_mesh(ctable.poses[0], ctable.shapes[0]);
  return true;
}

ob::RealVectorBounds darrt_actions::DARRTAction::getBounds(const std::vector<const darrt::SupportSurface *> tables, 
							   const std::vector<gm::Pose> &poses,
							   const an::RobotState &starting_state) {
  double minx = darrt::MATH_INF, maxx = -1.0*darrt::MATH_INF, miny = darrt::MATH_INF, maxy = -1.0*darrt::MATH_INF;
  //make sure the current state is included
  gm::Pose startpose = starting_state.multi_dof_joint_state.poses[0];
  minx = startpose.position.x - 0.1;
  maxx = startpose.position.x + 0.1;
  miny = startpose.position.y - 0.1;
  maxy = startpose.position.y + 0.1;

  //1 m around tables and 0.1 m around place locations
  for (unsigned int i = 0; i < tables.size(); i++) {
    const darrt::MeshSurface *table = dynamic_cast<const darrt::MeshSurface *>(tables[i]);
    std::vector<gm::Point> corners = darrt::corners(table->get_mesh());
    for (unsigned int j = 0; j < corners.size(); j++) {
      gm::Point corner = darrt::transform_point(corners[j], table->get_pose());
      if (corner.x - 1.0 < minx) {
	minx = corner.x - 1.0;
      }
      if (corner.x + 1.0 > maxx) {
	maxx = corner.x + 1.0;
      }
      if (corner.y - 1.0 < miny) {
	miny = corner.y - 1.0;
      }
      if (corner.y + 1.0 > maxy) {
	maxy = corner.y + 1.0;
      }
    }
  }
   
  for (unsigned int i = 0; i < poses.size(); i++) {
    if (poses[i].position.x - 1.0 < minx) {
      minx = poses[i].position.x - 1.0;
    }
    if (poses[i].position.x + 1.0 > maxx) {
      maxx = poses[i].position.x + 1.0;
    }
    if (poses[i].position.y - 1.0 < miny) {
      miny = poses[i].position.y - 1.0;
    }
    if (poses[i].position.y + 1.0 > maxy) {
      maxy = poses[i].position.y + 1.0;
    }
  }
  ob::RealVectorBounds bounds(3);
  bounds.setLow(0, minx);
  bounds.setHigh(0, maxx);
  bounds.setLow(1, miny);
  bounds.setHigh(1, maxy);
  bounds.setLow(2, 0);
  bounds.setHigh(2, 1.7);
  ROS_INFO("Bounds are: ([%f, %f], [%f, %f])", minx, maxx, miny, maxy); 
  return bounds;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "darrt_planning");
  ros::NodeHandle node("~");
  darrt_actions::DARRTAction da(node); 
  ros::spin();
  return 0;
}
