#include "darrt_actions/pickplace.hh"

#include "darrt/utils.hh"
#include "darrt/pr2_arm_primitives.hh"
#include "darrt/pr2_base_primitives.hh"
#include "darrt/solver.hh"
#include "darrt/control.hh"
#include "darrt/transform_ros_types.hh"

#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/GetRobotState.h>

darrt_actions::PickPlace::PickPlace(ros::NodeHandle &n) : 
  node_(n), 
  action_server_(node_, std::string("darrt_pickplace_action"),
		 boost::bind(&PickPlace::goalCB, this, _1), false),
  solver_(node_),
  cluster_grasp_planner_(node_, getClusterGraspPlannerName()),
  database_grasp_planner_(node_, getDatabaseGraspPlannerName())
{

  srand(time(NULL));
  action_server_.registerPreemptCallback(boost::bind(&PickPlace::preemptCB, this));

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

  ROS_ERROR("Don't hardcode the ik frame - FIX ME!");
  ik_frame_ = "torso_lift_link";

  action_server_.start();
  ROS_INFO("Pickplace action started");
}

darrt_actions::PickPlace::~PickPlace() {}

std::string darrt_actions::PickPlace::getClusterGraspPlannerName() {
  std::string name;
  node_.param<std::string>("/object_manipulator/default_cluster_planner", name, "/plan_point_cluster_grasp");
  return name;
}

std::string darrt_actions::PickPlace::getDatabaseGraspPlannerName() {
  std::string name;
  node_.param<std::string>("/object_manipulator/default_database_planner", name, "/database_grasp_planning");
  return name;
}

void darrt_actions::PickPlace::goalCB(const darrt_msgs::PickPlaceGoalConstPtr &goal_ptr) {

  ROS_INFO("PickPlace Action: received goal");

  darrt::Goal goal;
  //for pick/place the object usually doesn't go very far so we can spend all the time on the robot
  goal.params.robot_sampling_fraction = 1.0;
  goal.params.sample_robot_groups_individually = false;
  if (goal_ptr->planning_time > 0.001) {
    goal.params.darrt_allowed_time = goal_ptr->planning_time;
  }
  if (goal_ptr->goal_bias > 0.001) {
    goal.params.goal_bias = goal_ptr->goal_bias;
  }
  if (goal_ptr->threshold > darrt::EPSILON) {
    goal.params.darrt_threshold = goal_ptr->threshold;
  }
  darrt::debug_level = goal_ptr->debug_level;
  darrt::do_pause = goal_ptr->do_pause;
  if (darrt::do_pause) {
    goal.params.darrt_allowed_time = 1000;
  }

  darrt_msgs::PickPlaceResult result;
  if (!goal_ptr->place_goal.place_locations.size()) {
    ROS_ERROR("Must specify at least one place location");
    result.error_code = result.NO_PLACE_LOCATIONS;
    action_server_.setAborted(result);
    return;
  }

  //set up the goal for the object
  an::CollisionObject to_pickup;
  if (!getObject(goal_ptr->pickup_goal.collision_object_name, to_pickup)) {
    ROS_ERROR("Unable to find object to pick up");
    result.error_code = result.OBJECT_DOES_NOT_EXIST;
    action_server_.setAborted(result);
    return;
  }
  goal.objects.push_back(to_pickup);
  goal[to_pickup.id] = darrt::SingleGoal();
  goal[to_pickup.id].type = darrt::SingleGoal::ROTATIONALLY_SYMMETRIC_POSES;
  for (unsigned int i = 0; i < goal_ptr->place_goal.place_locations.size(); i++) {
    gm::PoseStamped pl = goal_ptr->place_goal.place_locations[i];
    ROS_INFO_STREAM("pl =\n" << pl);
    pl.header.stamp = ros::Time(0);
    ros::Rate r(10);
    for (int j = 0; j < 5; j++) {
      try {
	gm::PoseStamped plw;
	tf_listener_.transformPose(to_pickup.header.frame_id, pl, plw);
	goal[to_pickup.id].poses.push_back(plw.pose);
	break;
      } catch (tf::TransformException &e) {
	ROS_WARN("Unable to transform place pose: exception was %s", e.what());
      }
      r.sleep();
    }
  }
  if (!goal[to_pickup.id].poses.size()) {
    result.error_code = result.ERROR;
    action_server_.setAborted(result);
    return;
  }


  //get the tables and bounds
  darrt::MeshSurface pick_table(goal_ptr->pickup_goal.collision_support_surface_name);
  darrt::MeshSurface place_table(goal_ptr->place_goal.collision_support_surface_name);
  if (!getTable(pick_table)) {
    ROS_ERROR("Pick table must exist!");
    result.error_code = result.NO_PICK_TABLE;
    action_server_.setAborted(result);
    return;
  }
  goal.support_surfaces.push_back(&pick_table);
  if (getTable(place_table)) {
    goal.support_surfaces.push_back(&place_table);
  } else {
    ROS_WARN("Unable to find place table in collision map.  May not be able to plan a pick and place");
  }

  goal.params.bounds = getBounds(goal.support_surfaces, goal[to_pickup.id].poses);
  ob::RealVectorBounds objbnds(3);
  for (int i = 0; i < 2; i++) {
    objbnds.low[i] = goal.params.bounds.low[i];
    objbnds.high[i] = goal.params.bounds.high[i];
  }
  objbnds.setLow(2, 0);
  objbnds.setHigh(2, 2);
  goal[to_pickup.id].bounds = objbnds;
  
  ROS_INFO("Initializing primitives");
  //set up the primitives
  //Pickup primitive
  darrt::PickupPrimitive::RigidGraspList grasps;
  gm::PoseStamped objpose;
  objpose.header.frame_id = to_pickup.header.frame_id;
  objpose.pose = to_pickup.poses[0];
  if (!getGrasps(goal_ptr->pickup_goal, objpose, grasps)) {
    ROS_ERROR("Unable to get any grasps");
    result.error_code = result.NO_GRASPS;
    action_server_.setAborted(result);
    return;
  }

  //really shouldn't hardcode ik frame!
  std::string lift_frame = goal_ptr->pickup_goal.lift.direction.header.frame_id,
    approach_frame = goal_ptr->place_goal.approach.direction.header.frame_id;
  if (lift_frame[0] == '/') {
    lift_frame = lift_frame.substr(1, lift_frame.size());
  }
  if (approach_frame[0] == '/') {
    approach_frame = approach_frame.substr(1, approach_frame.size());
  }
  if (lift_frame.compare(ik_frame_) || approach_frame.compare(ik_frame_)) {
    ROS_ERROR("All directions (lift and approach) must be specified in frame %s.  Lift was specified in %s and approach in %s", ik_frame_.c_str(), lift_frame.c_str(), approach_frame.c_str());
    result.error_code = result.ERROR;
    action_server_.setAborted(result);
    return;
  }

  darrt::PickupPrimitive pickup_prim(goal_ptr->pickup_goal.arm_name, goal_ptr->pickup_goal.collision_object_name,
				     grasps, goal_ptr->pickup_goal.lift.direction.vector, 
				     goal_ptr->pickup_goal.lift.min_distance, 
				     goal_ptr->pickup_goal.lift.desired_distance);
  goal.primitives.push_back(&pickup_prim);
  //Rigid transfer primitive
  darrt::RigidTransfer rigid_transfer(goal_ptr->pickup_goal.arm_name);
  goal.primitives.push_back(&rigid_transfer);
  //Place primitive
  darrt::PlacePrimitive place_prim(goal_ptr->pickup_goal.arm_name, goal_ptr->place_goal.approach.direction.vector,
				   goal_ptr->place_goal.approach.min_distance, 
				   goal_ptr->place_goal.approach.desired_distance);
  goal.primitives.push_back(&place_prim);
  //Transit
  darrt::PR2ArmTransit transit(goal_ptr->pickup_goal.arm_name);
  goal.primitives.push_back(&transit);
  goal.robot_groups.push_back(goal_ptr->pickup_goal.arm_name);

  //Warp
  darrt::PR2Warp warp("pr2_base");
  if (goal_ptr->allow_warp) {
    goal.primitives.push_back(&warp);
    goal.robot_groups.push_back("pr2_base");
  }

  ROS_INFO("Configuring for goal");
  if (!solver_.configure(goal)) {
    ROS_ERROR("Unable to configure solver.");
    result.error_code = result.CONFIGURE_ERROR;
    action_server_.setAborted(result);
    return;
  }
  if (!action_server_.isActive()) {
    return;
  }
  ob::State *side_state = NULL;
  const darrt::RobotChainStateSpace *arm_space =
    solver_.space_information()->getStateSpace()->as<darrt::DARRTStateSpace>()->robot_state_space()->
    as<ob::CompoundStateSpace>()->getSubspace(goal_ptr->pickup_goal.arm_name)->as<darrt::RobotChainStateSpace>();
  if (goal_ptr->allow_warp) {
    side_state = arm_space->allocState();
    orc::joint_positions_to_ompl_state
      (side_states_[goal_ptr->pickup_goal.arm_name], arm_space->robot_to_ompl_mapping(), arm_space->joint_names(),
       side_state);
    warp.add_set_position(goal_ptr->pickup_goal.arm_name, side_state);
  }
  int ntries = goal_ptr->tries;
  if (ntries <= 0) {
    ntries = 1;
  }
  bool solved = false;
  for (int i = 0; i < ntries; i++) {
    ROS_INFO("Planning attempt %d", i);
    solved = solver_.plan(goal);
    if (!action_server_.isActive()) {
      return;
    }
    if (solved) {
      break;
    }
  }
  if (!solved) {
    ROS_ERROR("Unable to plan");
    result.error_code = result.PLANNING_FAILED;
    action_server_.setAborted(result);
    return;
  }
  darrt::PrimitivePath prim_path;
  darrt::convert_to_primitive_path(*(solver_.getSolutionPath()), prim_path);
  const darrt::DARRTStateSpace *space = solver_.space_information()->getStateSpace()->as<darrt::DARRTStateSpace>();
  const darrt::RobotStateSpace *robot_space = space->robot_state_space()->as<darrt::RobotStateSpace>();
  for (unsigned int i = 0; i < prim_path.size(); i++) {
    result.primitive_names.push_back(prim_path[i].primitive->name());
    an::RobotTrajectory traj;
    for (unsigned int j = 0; j < prim_path[i].states.size(); j++) {
      an::RobotState rstate;
      robot_space->convert_ompl_to_robot_state(space->robot_state(prim_path[i].states[j]), rstate);
      if (!traj.joint_trajectory.joint_names.size()) {
	traj.joint_trajectory.header = rstate.joint_state.header;
	traj.joint_trajectory.joint_names = rstate.joint_state.name;
	traj.multi_dof_joint_trajectory.joint_names = rstate.multi_dof_joint_state.joint_names;
	traj.multi_dof_joint_trajectory.frame_ids = rstate.multi_dof_joint_state.frame_ids;
	traj.multi_dof_joint_trajectory.child_frame_ids = rstate.multi_dof_joint_state.child_frame_ids;
      }
      trajectory_msgs::JointTrajectoryPoint jp;
      jp.positions = rstate.joint_state.position;
      an::MultiDOFJointTrajectoryPoint mdofp;
      mdofp.poses = rstate.multi_dof_joint_state.poses;
      traj.joint_trajectory.points.push_back(jp);
      traj.multi_dof_joint_trajectory.points.push_back(mdofp);
    }
    result.primitive_trajectories.push_back(traj);
  }
  //REALLY important to do this BEFORE we delete the primitives!
  if (side_state) {
    arm_space->freeState(side_state);
  }
  solver_.reset();
  result.error_code = result.SUCCESS;
  action_server_.setSucceeded(result);
}
  

void darrt_actions::PickPlace::preemptCB() {
  ROS_INFO("PickPlace Action: Caught cancel signal");
  if (!solver_.cancel(ros::Duration(CANCEL_TIMEOUT))) {
    ROS_WARN("Unable to cancel DARRTH in %f seconds", CANCEL_TIMEOUT);
  }
}

bool darrt_actions::PickPlace::getGrasps(const object_manipulation_msgs::PickupGoal &pg, 
					 const gm::PoseStamped &object_pose, 
					 darrt::PickupPrimitive::RigidGraspList &grasps) {
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
	tf_listener_.transformPose(object_pose.header.frame_id, gps, wrist_pose);
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
							    om_grasps[i].desired_approach_distance));
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

bool darrt_actions::PickPlace::getObject(std::string name, an::CollisionObject &obj) {
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

bool darrt_actions::PickPlace::getTable(darrt::MeshSurface &table) {
  an::CollisionObject ctable;
  if (!getObject(table.name(), ctable)) {
    return false;
  }
  if (!ctable.shapes.size() || ctable.shapes[0].type != an::Shape::MESH) {
    ROS_ERROR("Passed in table is not a mesh in collision map!  Only meshes can be used as support surfaces.");
    return false;
  }
  table.set_pose(ctable.poses[0]);
  table.set_mesh(ctable.shapes[0]);
  return true;
}

//if you use this for the robot and not the object should take into account the robot's current location!
ob::RealVectorBounds darrt_actions::PickPlace::getBounds(const std::vector<const darrt::SupportSurface *> tables, 
							 const std::vector<gm::Pose> &poses) {
  double minx = darrt::MATH_INF, maxx = -1.0*darrt::MATH_INF, miny = darrt::MATH_INF, maxy = -1.0*darrt::MATH_INF;
  //make sure the current state is included
  an::GetRobotState srv;
  if (robot_client_.call(srv)) {
    gm::Pose startpose = srv.response.robot_state.multi_dof_joint_state.poses[0];
    minx = startpose.position.x - 0.1;
    maxx = startpose.position.x + 0.1;
    miny = startpose.position.y - 0.1;
    maxy = startpose.position.y + 0.1;
  }


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
  ob::RealVectorBounds bounds(2);
  bounds.setLow(0, minx);
  bounds.setHigh(0, maxx);
  bounds.setLow(1, miny);
  bounds.setHigh(1, maxy);
  return bounds;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "darrt_planning");
  ros::NodeHandle node("~");
  darrt_actions::PickPlace pp(node); 
  ros::spin();
  return 0;
}
