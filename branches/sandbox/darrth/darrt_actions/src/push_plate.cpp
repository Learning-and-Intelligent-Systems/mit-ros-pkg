#include "darrt_actions/push_plate.hh"

#include <darrt/chain_space.hh>
#include <darrt/pr2_primitives.hh>

#include <arm_navigation_msgs/GetPlanningScene.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <planning_environment/models/model_utils.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_mechanism_msgs/ListControllers.h>

darrt_actions::PushPlate::PushPlate(ros::NodeHandle &n, std::string arm_name) : 
  node_(n), 
  action_server_(node_, arm_name.substr(0,1)+std::string("_push_plate_action"),
		 boost::bind(&PushPlate::goalCB, this, _1),
		 boost::bind(&PushPlate::cancelCB, this, _1), false),
  solver_(node_),
  joint_executor_(std::string("/")+arm_name.substr(0,1)+
		  std::string("_arm_controller/joint_trajectory_action")),
  has_active_goal_(false) {
  
  initialized_ = false;
  
  viz_ = node_.advertise<visualization_msgs::MarkerArray>("robot_state_markers", 1);

  arm_name_ = arm_name;
  //deal with other controllers?
  //how do we know what else is there?
  ROS_INFO("Waiting for joint trajectory action for %s", arm_name_.c_str());
  joint_executor_.waitForServer();

  scene_client = node_.serviceClient<an::GetPlanningScene>
    ("/environment_server/get_planning_scene");
  ROS_INFO("Waiting for get planning scene service");
  scene_client.waitForExistence();
  
  controller_client = node_.serviceClient
    <pr2_mechanism_msgs::SwitchController>
    ("/pr2_controller_manager/switch_controller");
  ROS_INFO("Waiting for controller switcher service");
  controller_client.waitForExistence();

  controller_list_client = node_.serviceClient
    <pr2_mechanism_msgs::ListControllers>
    ("/pr2_controller_manager/list_controllers");
  ROS_INFO("Waiting for controller list service");
  controller_list_client.waitForExistence();

  phase_ = READY;
  action_server_.start();
  ROS_INFO("Push plate action started");
  initialized_ = true;
}

darrt_actions::PushPlate::~PushPlate() {}

void darrt_actions::PushPlate::goalCB(PPAS::GoalHandle gh) {

  ROS_INFO("PushPlate Action: received goal");
  darrt::debug_level = darrt::DRRT;
  //darrt::do_pause = false;
  if (has_active_goal_) {
    //set the current goal to canceled
    cancelCB(active_goal_handle_);
  }

  phase_ = SOLVING; 
  active_goal_handle_ = gh;
  has_active_goal_ = true;
  active_goal_handle_.setAccepted();

  ROS_INFO("Initializing primitives");
  darrt::Goal goal;
  goal.params = info_;
  goal.params.debug_level = darrt::DRRT;
  darrt::PR2Transit transit;
  goal.primitives.push_back(&transit);
  an::OrderedCollisionOperations pops;
  an::CollisionOperation pop;
  pop.object1 = gh.getGoal()->surface_id;
  pop.object2 = "r_end_effector";
  pop.operation = pop.DISABLE;
  pops.collision_operations.push_back(pop);
  pop.object1 = gh.getGoal()->plate_id;
  pops.collision_operations.push_back(pop);
  darrt::Retreat retreat;
  retreat.set_allowed_collisions(pops);
  goal.primitives.push_back(&retreat);
  std::vector<std::string> pushing_links;
  pushing_links.push_back(arm_name_.substr(0,1)+std::string("_end_effector"));
  darrt::Push push(pushing_links, gh.getGoal()->surface_id);
  goal.primitives.push_back(&push);

  ROS_INFO("Finding objects");
  //get the surface and plate out of the goal
  an::GetPlanningScene scene;
  if (!scene_client.call(scene)) {
    ROS_ERROR("Unable to get the planning scene");
    phase_ = READY;
    return;
  }
  std::vector<an::CollisionObject> &objects = 
    scene.response.planning_scene.collision_objects;
  an::CollisionObject table, plate;
  bool foundtable = false, foundplate = false;
  for (size_t i = 0; i < objects.size(); i++) {
    if (!objects[i].id.compare(gh.getGoal()->plate_id)) {
      plate = objects[i];
      foundplate = true;
    }
    if (!objects[i].id.compare(gh.getGoal()->surface_id)) {
      foundtable = true;
      table = objects[i];
    }
    if (foundplate && foundtable) {
      break;
    }
  }
  if (!foundplate) {
    ROS_ERROR("Unable to find plate with object ID %s", 
	      gh.getGoal()->plate_id.c_str());
    phase_ = READY;
    result_.manipulation_result.value = result_.manipulation_result.UNFEASIBLE;
    active_goal_handle_.setAborted(result_);
    has_active_goal_ = false;
    return;
  }
  if (!foundtable) {
    ROS_ERROR("Unable to find surface with object ID %s", 
	      gh.getGoal()->surface_id.c_str());
    phase_ = READY;
    result_.manipulation_result.value = result_.manipulation_result.UNFEASIBLE;
    active_goal_handle_.setAborted(result_);
    has_active_goal_ = false;
    return;
  }

  if (plate.shapes.size() != 1 || plate.shapes[0].type != an::Shape::CYLINDER) {
    ROS_ERROR_STREAM("Plate must be a cylinder in collision map.  Instead plate is:\n" 
		     << plate);
    phase_ = READY;
    result_.manipulation_result.value = result_.manipulation_result.UNFEASIBLE;
    active_goal_handle_.setAborted(result_);
    has_active_goal_ = false;
    return;
  }

  if (!table.shapes.size() || table.shapes.size() > 2 
      || table.shapes[0].type != an::Shape::MESH) {
    ROS_ERROR_STREAM("Surface must be a mesh in collision map.  Instead surface is:\n"
		     << table);
    phase_ = READY;
    result_.manipulation_result.value = result_.manipulation_result.UNFEASIBLE;
    active_goal_handle_.setAborted(result_);
    has_active_goal_ = false;
    return;
  }
  goal.objects.push_back(plate);

  ROS_INFO("Configuring solver");
  goal.chain_name = arm_name_;
  darrt::debug_level = goal.params.debug_level;
  darrt::do_pause = false;
  if (!solver_.configure(goal)) {
    ROS_ERROR("Unable to configure solver");
    phase_ = READY;
    result_.manipulation_result.value = result_.manipulation_result.UNFEASIBLE;
    active_goal_handle_.setAborted(result_);
    has_active_goal_ = false;
    return;
  }

  //interpret the goal
  ROS_INFO("Creating goal");
  if (!create_goal(table, plate, goal)) {
    phase_ = READY;
    if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE) {
      return;
    }
    ROS_ERROR("Unable to intepret goal");
    result_.manipulation_result.value = result_.manipulation_result.UNFEASIBLE;
    active_goal_handle_.setAborted(result_);
    has_active_goal_ = false;
    return;
  }



  ROS_INFO("Planning, goal status is %d", gh.getGoalStatus().status);
  bool solved = solver_.plan(goal);
  if (!solved) {
    ROS_INFO("Goal status is %d", gh.getGoalStatus().status);
    solver_.reset();
    phase_ = READY;
    if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE) {
      return;
    }
    ROS_ERROR("Unable to find plan.  Goal status is %d", gh.getGoalStatus().status);
    result_.manipulation_result.value = result_.manipulation_result.UNFEASIBLE;
    active_goal_handle_.setAborted(result_);
    has_active_goal_ = false;
    return;
  }
  
  //execute
  phase_ = EXECUTING;
  solver_.display_solution();
  an::RobotTrajectory rtraj;
  if (!solver_.solution_to_trajectory(rtraj) || 
      !rtraj.joint_trajectory.points.size()) {
    ROS_ERROR("Unable to convert solution to goal trajectory");
    result_.manipulation_result.value = result_.manipulation_result.FAILED;
    solver_.reset();
    phase_ = READY;
    active_goal_handle_.setAborted(result_);
    has_active_goal_ = false;
    return;
  }
  ROS_INFO("Switching to joint trajectory controller");
  pr2_mechanism_msgs::SwitchController swi;
  swi.request.start_controllers.push_back
    (arm_name_.substr(0,1)+"_arm_controller");
  swi.request.stop_controllers.push_back(arm_name_.substr(0,1)+"_cart");
  swi.request.strictness = swi.request.BEST_EFFORT;
  if (!controller_client.call(swi) || !swi.response.ok) {
    //check if the controller is already running
    pr2_mechanism_msgs::ListControllers lc;
    if (!controller_list_client.call(lc)) {
      ROS_ERROR("Unable to list controllers.  Cannot execute trajectory");
      solver_.reset();
      result_.manipulation_result.value = result_.manipulation_result.ERROR;
      phase_ = READY;
      active_goal_handle_.setAborted(result_);
      has_active_goal_ = false;
      return;
    }
    int rind = -1;
    for (size_t i = 0; i < lc.response.controllers.size(); i++) {
      if (!lc.response.controllers[i].compare("r_arm_controller")) {
	rind = i;
	break;
      }
    }
    if (rind < 0 || lc.response.state[rind][0] != 'r') {
      ROS_ERROR("Joint trajectory action is not running and we cannot switch to it.  Aborting execution");
      solver_.reset();
      result_.manipulation_result.value = result_.manipulation_result.ERROR;
      phase_ = READY;
      active_goal_handle_.setAborted(result_);
      has_active_goal_ = false;
      return;
    }
  }
	  

  ROS_INFO("Joint names are:");
  for (size_t i = 0; i < rtraj.joint_trajectory.joint_names.size(); i++) {
    ROS_INFO("%s", rtraj.joint_trajectory.joint_names[i].c_str());
  }

  ROS_INFO("Sending trajectory of %zd points with length %f seconds",
	   rtraj.joint_trajectory.points.size(), 
	   rtraj.joint_trajectory.points.back().time_from_start.toSec());

  pr2_controllers_msgs::JointTrajectoryGoal jgoal;
  jgoal.trajectory = rtraj.joint_trajectory;
  jgoal.trajectory.header.stamp = ros::Time::now();
  actionlib::SimpleClientGoalState status = 
    joint_executor_.sendGoalAndWait(jgoal);
  if (status != actionlib::SimpleClientGoalState::SUCCEEDED) {
    solver_.reset();
    phase_ = READY;
    if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE) {
      return;
    }
    ROS_ERROR("Joint trajectory action failed");
    result_.manipulation_result.value = result_.manipulation_result.ERROR;
    active_goal_handle_.setAborted(result_);
    has_active_goal_ = false;
    return;
  }
  ROS_INFO("Successfully pushed plate");
  result_.manipulation_result.value = result_.manipulation_result.SUCCESS;
  active_goal_handle_.setSucceeded(result_);
  has_active_goal_ = false;
  phase_ = READY;
  solver_.reset();
}
  

void darrt_actions::PushPlate::cancelCB(PPAS::GoalHandle gh) {
  ROS_INFO("PushPlate Action: Caught cancel signal");
  if (!has_active_goal_ || active_goal_handle_ != gh) {
    return;
  }
  ROS_INFO("PushPlate Action: Canceling active goal.");
  switch(phase_) {
  case EXECUTING:
    phase_ = WAITING;
    joint_executor_.cancelGoal();
    break;
  case SOLVING:
    phase_ = WAITING;
    if (!solver_.cancel(ros::Duration(CANCEL_TIMEOUT))) {
      ROS_WARN("Unable to cancel DARRTH in %f seconds", CANCEL_TIMEOUT);
    }
    break;
  default:
    phase_ = READY;
    break;
  }


  ros::Rate r(100);
  ros::Time start = ros::Time::now(), current = ros::Time::now();
  while (phase_ == WAITING && current - start < ros::Duration(CANCEL_TIMEOUT)) {
    r.sleep();
    current = ros::Time::now();
  }
  if (phase_ == WAITING) {
    ROS_WARN("PushPlate Action: Unable to cancel goal");
  } else {
    ROS_INFO("Successfully canceled goal");
  }
  result_.manipulation_result.value = result_.manipulation_result.CANCELLED;
  active_goal_handle_.setCanceled(result_);
  has_active_goal_ = false;

}

//we take a reference to the PushPlate goal instead of pulling
//it out of active_goal_handle_ to make thread safety easier
bool darrt_actions::PushPlate::create_goal
(const an::CollisionObject &table, const an::CollisionObject &plate, 
 darrt::Goal &goal) {

  //use point clouds to make the transforms easy
  //could also allow us to do more complicated things
  const std::vector<gm::Point> *tvp = &(table.shapes[0].vertices);
  const gm::Pose *point_transp = &(table.poses[0]);
  if (table.shapes.size() > 1 && table.poses[1].position.z >
      table.poses[0].position.z) {
    tvp = &(table.shapes[1].vertices);
    point_transp = &(table.poses[1]);
  }
  const std::vector<gm::Point> &tv = *tvp;
  const gm::Pose &point_trans = *point_transp;

  PointCloud perimeter_in, perimeter;
  perimeter_in.resize(tv.size());
  perimeter_in.header.frame_id = table.header.frame_id;
  for (size_t i = 0; i < tv.size(); i++) {
    gm::Point table_point = darrt::transform_point(tv[i], point_trans);
    perimeter_in[i].x = table_point.x;
    perimeter_in[i].y = table_point.y;
    perimeter_in[i].z = table_point.z;
  }

  std::string grasp_frame = "torso_lift_link";

  if (!pcl_ros::transformPointCloud(grasp_frame, ros::Time(0),
				    perimeter_in, table.header.frame_id,
				    perimeter, tf_listener_)) {
    ROS_ERROR("Unable to transform table perimeter from %s to %s", 
	      table.header.frame_id.c_str(), grasp_frame.c_str());
    return false;
  }
  

  //set up basic info for goal
  goal[plate.id].type = darrt::SingleGoal::POSES;
  goal[plate.id].bounds = arm_bounds();
  if (!goal[plate.id].bounds.low.size() || !goal[plate.id].bounds.high.size()) {
    ROS_ERROR("Unable to get bounds");
    return false;
  }
 
  an::CollisionOperation op;
  op.object1 = plate.id;
  op.object2 = table.id;
  op.operation = op.DISABLE;
  goal[plate.id].allowed_collisions.collision_operations.push_back(op);
  op.object2 = arm_name_;
  goal[plate.id].allowed_collisions.collision_operations.push_back(op);  
  op.object2 = arm_name_[0]+"_end_effector";
  goal[plate.id].allowed_collisions.collision_operations.push_back(op);
  goal[plate.id].allowed_time = info_.object_allowed_time;
  goal[plate.id].threshold = info_.object_threshold;

  //set up allowed collisions for collision checking
  planning_environment::CollisionModelsInterface *cmi =
    solver_.darrt_solver().collision_models_interface();
  if (!cmi->getPlanningSceneState()) {
    ROS_ERROR("Create goal: Planning scene state hasn't been set!");
    return false;
  }
  planning_models::KinematicState kstate(*(cmi->getPlanningSceneState()));
  const collision_space::EnvironmentModel::AllowedCollisionMatrix
    &original_acm = cmi->getCurrentAllowedCollisionMatrix();
  collision_space::EnvironmentModel::AllowedCollisionMatrix pacm(original_acm);
  planning_environment::applyOrderedCollisionOperationsListToACM
    (goal[plate.id].allowed_collisions, std::vector<std::string>(),
     std::vector<std::string>(), kstate.getKinematicModel(), pacm);

  an::OrderedCollisionOperations gops;
  an::CollisionOperation gop;
  gop.object1 = table.id;
  gop.object2 = arm_name_.substr(0,1)+std::string("_end_effector");
  gop.operation = gop.DISABLE;
  gops.collision_operations.push_back(gop);
  
  collision_space::EnvironmentModel::AllowedCollisionMatrix gacm(pacm);
  planning_environment::applyOrderedCollisionOperationsListToACM
    (gops, std::vector<std::string>(), std::vector<std::string>(), 
     kstate.getKinematicModel(), gacm);

  gm::PoseStamped vpt;
  vpt.header.frame_id = perimeter.header.frame_id;
  vpt.pose.orientation.w = 1.0;
  visualization_msgs::MarkerArray parray, garray;
  PointCloud goal_points;
  goal_points.header.frame_id = perimeter.header.frame_id;
  for (size_t i = 0; i < perimeter.size(); i++) {
    if (phase_ == WAITING) {
      return false;
    }
    
    vpt.pose.position.x = perimeter[i].x + 0.08;
    vpt.pose.position.y = perimeter[i].y;
    vpt.pose.position.z = perimeter[i].z;
    parray.markers.push_back
      (darrt::make_marker(vpt, visualization_msgs::Marker::SPHERE, 0.05,
			  0, 1, 0, 0.3, "possible_goals", i));
    geometry_msgs::PoseStamped grasp;
    grasp.header.frame_id = grasp_frame;
    grasp.header.stamp = ros::Time(0);
    //the gripper is 0.18 cm from the wrist and we want it 5cm
    //onto the plate, but the table is not quite in the right place...
    grasp.pose.position = vpt.pose.position;
    grasp.pose.position.x -= (plate.shapes[0].dimensions[0] + 0.16);
    grasp.pose.orientation.x = -1.0*sqrt(2.0)/2.0;
    grasp.pose.orientation.w = sqrt(2.0)/2.0;
    //is this reachable by the robot?
    const darrt::RobotChainStateSpace *rspace = 
      solver_.darrt_solver().space_information()->getStateSpace()
      ->as<darrt::DARRTStateSpace>()->robot_state_space()->
      as<darrt::RobotChainStateSpace>();
    std::map<std::string, double> grasppos, pregrasppos;
    bool feasible = rspace->inverse_kinematics(grasp, grasppos);
    for (int j = 0; j < 2; j++) {
      feasible = rspace->inverse_kinematics(grasp, grasppos);
      if (feasible) {
	break;
      }
    }
    if (!feasible) {
      ROS_INFO("Skipping goal state (%f, %f, %f) because no grasp exists at position (%f, %f, %f)", perimeter[i].x, perimeter[i].y, perimeter[i].z, grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z);
      continue;
    }
    geometry_msgs::PoseStamped pregrasp = grasp;
    pregrasp.pose.position.x -= 0.1;
    feasible = rspace->inverse_kinematics(pregrasp, pregrasppos);
    for (int j = 0; j < 2; j++) {
      feasible = rspace->inverse_kinematics(pregrasp, pregrasppos);
      if (feasible) {
	break;
      }
    }
    if (!feasible) {
      ROS_INFO("Skipping goal state (%f, %f, %f) because no pregrasp exists at position (%f, %f, %f)", perimeter[i].x, perimeter[i].y, perimeter[i].z, pregrasp.pose.position.x, pregrasp.pose.position.y, pregrasp.pose.position.z);
      continue;
    }
    
    //check that the pre-grasp position is collision-free
    kstate.setKinematicState(pregrasppos);
    cmi->setAlteredAllowedCollisionMatrix(pacm);
    if (cmi->isKinematicStateInCollision(kstate)) {
      ROS_INFO("Skipping goal state (%f, %f, %f) because pre-grasp (%f %f %f) is in collision", perimeter[i].x, perimeter[i].y,
	       perimeter[i].z, pregrasp.pose.position.x, 
	       pregrasp.pose.position.y, pregrasp.pose.position.z);
      continue;
    }
    //check that the grasp is collision-free
    kstate.setKinematicState(grasppos);
    cmi->setAlteredAllowedCollisionMatrix(gacm);
    if (cmi->isKinematicStateInCollision(kstate)) {
      ROS_INFO("Skipping goal state (%f, %f, %f) because grasp (%f %f %f) is in collision", perimeter[i].x, perimeter[i].y,
	       perimeter[i].z, grasp.pose.position.x, 
	       grasp.pose.position.y, grasp.pose.position.z);
      continue;
    }
    pcl::PointXYZ pgpt;
    pgpt.x = vpt.pose.position.x;
    pgpt.y = vpt.pose.position.y;
    pgpt.z = vpt.pose.position.z;
    goal_points.push_back(pgpt);
    garray.markers.push_back
      (darrt::make_marker(vpt, visualization_msgs::Marker::SPHERE, 0.05, 0, 1, 0, 0.9, 
			  "goal_states", i));
  }
  viz_.publish(parray);
  viz_.publish(garray);
  if (!goal_points.size()) {
    ROS_ERROR("No valid goal states!");
    return false;
  }
  
  PointCloud perimeter_robot_frame;
  if (!pcl_ros::transformPointCloud(darrt::ROBOT_FRAME_ID, ros::Time(0),
				    goal_points, grasp_frame,
				    perimeter_robot_frame, tf_listener_)) {
    ROS_ERROR("Unable to transform table perimeter from %s to %s", 
	      goal_points.header.frame_id.c_str(), darrt::ROBOT_FRAME_ID.c_str());
    return false;
  }
  goal[plate.id].poses.resize(perimeter_robot_frame.size());
  for (size_t i = 0; i < perimeter_robot_frame.size(); i++) {
    goal[plate.id].poses[i].position.x = perimeter_robot_frame[i].x;
    goal[plate.id].poses[i].position.y = perimeter_robot_frame[i].y;
    goal[plate.id].poses[i].position.z = perimeter_robot_frame[i].z;
    goal[plate.id].poses[i].orientation.w = 1.0;
  }

  return true;
} 

ob::RealVectorBounds darrt_actions::PushPlate::arm_bounds() {
  
  geometry_msgs::PointStamped minpt, maxpt, ominpt, omaxpt;
  minpt.header.frame_id = "/torso_lift_link";
  minpt.header.stamp = ros::Time(0);
  maxpt.header.frame_id = "/torso_lift_link";
  maxpt.header.stamp = ros::Time(0);
  ominpt.header.frame_id = darrt::ROBOT_FRAME_ID;
  ominpt.header.stamp = ros::Time(0);
  omaxpt.header.frame_id = darrt::ROBOT_FRAME_ID;
  omaxpt.header.stamp = ros::Time(0);

  //approximate the workspace of the arms
  //especially if there is a table in front of the robot
  minpt.point.x = -0.2;
  maxpt.point.x = 1.0;
  if (arm_name_[0] == 'l') {
    minpt.point.y = -0.2;
    maxpt.point.y = 1.0;
  } else {
    minpt.point.y = -1.0;
    maxpt.point.y = 0.2;
  }

  ros::Rate looprate(10);
  bool trans = false;
  for (int i =0; i < 10; i++) {
    try {
      tf_listener_.transformPoint(darrt::ROBOT_FRAME_ID, minpt, ominpt);
      tf_listener_.transformPoint(darrt::ROBOT_FRAME_ID, maxpt, omaxpt);
      trans = true;
      break;
    } catch(tf::TransformException &e) {
      ROS_WARN("Unable to transform bounds from torso_lift_link to %s: %s", 
	       darrt::ROBOT_FRAME_ID.c_str(), e.what());
    }
    looprate.sleep();
  }
  if (!trans) {
    ROS_ERROR("Cannot transform bounds");
    return ob::RealVectorBounds(0);
  }

  double minx = ominpt.point.x, maxx = omaxpt.point.x, 
    miny = ominpt.point.y, maxy = omaxpt.point.y;
  
  ROS_INFO("Bounds in %s are ([%f, %f], [%f, %f])", 
	   darrt::ROBOT_FRAME_ID.c_str(), minx, maxx, miny, maxy);

  if (minx > maxx) {
    minx = maxx;
    maxx = ominpt.point.x;
  }
  if (miny > maxy) {
    miny = maxy;
    maxy = ominpt.point.y;
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
  //one thread for listening for goals/cancels
  //one thread for solving
  //one thread for collision models interface?
  ros::AsyncSpinner spinner(2);
  spinner.start();
  darrt_actions::PushPlate rpp(node, "right_arm"); 
  //lpp(node, "left_arm");
  if (!rpp.initialized()) {
    ROS_ERROR("Unable to initialize push plate action");
    spinner.stop();
    return 0;
  }

  ros::waitForShutdown();
}
