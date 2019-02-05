#include "darrt/solver.hh"
#include "darrt/object_solver.hh"
#include "darrt/utils.hh"
#include "darrt/pr2_primitives.hh"
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/GetRobotState.h>
#include <boost/lexical_cast.hpp>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <planning_environment/models/model_utils.h>

//for grasp
#include <actionlib/client/simple_action_client.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <arm_navigation_msgs/GetMotionPlan.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <tf/transform_listener.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>

ros::Publisher objpub;
tf::TransformListener *tf_listener;

void printUsage(bool &print_again) {
  if (print_again) {
    ROS_INFO("Usage:\n-p: Run pre solver checks.\n-s: Run post solver checks.\n-t: Run only pre solver checks and return without solving.\n-r#: Set robot radius to #.\n-o#: Set number of obstacles to #.");
  }
  print_again = false;
}

void create_plate(an::CollisionObject &object, double radius,
		  double height, geometry_msgs::PoseStamped pose) {
  object.id = "plate";
  object.operation.operation =
    an::CollisionObjectOperation::ADD;
  object.header.frame_id = pose.header.frame_id;
  object.header.stamp = ros::Time(0);
  an::Shape shape;
  shape.type = shape.CYLINDER;
  shape.dimensions.resize(2);
  shape.dimensions[0] = radius;
  shape.dimensions[1] = height;
  object.shapes.push_back(shape);
  object.poses.push_back(pose.pose);
  objpub.publish(object);
  //publish a table that this is sitting on
  an::CollisionObject table;
  table.id = "table";
  table.header.frame_id = pose.header.frame_id;
  geometry_msgs::Pose tpose;
  tpose.position.x = pose.pose.position.x + 0.18;
  tpose.position.y = pose.pose.position.y - 0.04;
  tpose.position.z = pose.pose.position.z - 0.05;
  tpose.orientation.w = 1.0;
  an::Shape tshape;
  tshape.type = shape.BOX;
  tshape.dimensions.resize(3);
  tshape.dimensions[0] = 0.9;
  tshape.dimensions[1] = 2.0;
  tshape.dimensions[2] = 0.1;
  table.shapes.push_back(tshape);
  table.poses.push_back(tpose);
  ROS_INFO("Publishing table!");
  objpub.publish(table);
  //give some time to notice these things
  ros::Rate looprate(10);
  for (int i = 0; i < 10; i++) {
    looprate.sleep();
  }
}


void create_plate(an::CollisionObject &object) {
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0.5;
  pose.pose.position.y = -0.1;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  pose.header.frame_id = "/torso_lift_link";
  pose.header.stamp = ros::Time(0);
  create_plate(object, 0.07, 0.05, pose);
}

bool get_plate(an::CollisionObject &plate) {
  ros::NodeHandle n;
  ros::ServiceClient client =
    n.serviceClient<an::GetPlanningScene>
    ("/environment_server/get_planning_scene");
  an::GetPlanningScene srv;
  if (!client.call(srv)) {
    ROS_ERROR("Unable to get planning scene");
    return false;
  }
  std::vector<an::CollisionObject> &objects =
    srv.response.planning_scene.collision_objects;
  int objind = -1;
  for (size_t i = 0; i < objects.size(); i++) {
    if (!objects[i].id.compare("plate")) {
      objind = i;
      break;
    }
    if (objects[i].id.compare("table") &&
	objects[i].shapes.size() == 1 &&
	objects[i].shapes[0].dimensions.size() == 3) {
      objind = i;
    }
  }
  std::string objectstr;
  for (size_t i = 0; i < objects.size(); i++) {
    objectstr += " (" + objects[i].id + ")";
  }

  if (objind < 0) {
    ROS_ERROR("Unable to find plate in planning scene.  Objects are: %s.", 
	      objectstr.c_str());
    return false;
  }

  ROS_INFO("Objects are: %s.  Plate is %s.", objectstr.c_str(), 
	   objects[objind].id.c_str());
  if (!objects[objind].id.compare("plate")) {
    //assume we've also corrected for a thicker table
    plate = objects[objind];
    return true;
  }


  //create a plate from the pose and dimension of the bounding box
  geometry_msgs::PoseStamped pose;
  pose.header = objects[objind].header;
  pose.pose = objects[objind].poses[0];
  create_plate(plate, objects[objind].shapes[0].dimensions[0]/2.0,
	       objects[objind].shapes[0].dimensions[2], pose);
  //remove the box
  objects[objind].operation.operation = objects[objind].operation.REMOVE;
  objpub.publish(objects[objind]);
  return true;
}

void parseArguments(int argc, char **argv,
		    darrt::DARRTSolver &pusher) {
  int curr_arg = 1;
  bool usage=false;

  while (curr_arg < argc) {
    if (argv[curr_arg][0] == '-') {
      switch(argv[curr_arg][1]) {
      case 'p':
	pusher.set_pre_checks(true);
	break;
      case 's':
	pusher.set_post_checks(true);
	break;
      case 't':
	pusher.set_only_pre_checks(true);
	break;
      case 'o':
	try {
	  int n = boost::lexical_cast<int>(&(argv[curr_arg][2]));
	  pusher.set_obstacle_number(n);
	} catch (boost::bad_lexical_cast &) {
	  ROS_ERROR("Unable to interpret argument for obstacle number.  Number of obstacles is %d", pusher.obstacle_number());
	  printUsage(usage);
	}
	break;
      case 'd':
	try {
	  int d = boost::lexical_cast<int>(&(argv[curr_arg][2]));
	  darrt::debug_level = d;
	} catch (boost::bad_lexical_cast &) {
	  ROS_ERROR("Unable to interpret argument for debugging level.  Debugging level is %d", darrt::debug_level);
	  printUsage(usage);
	}
	break;
      case 'r':
	darrt::do_pause = false;
	break;
      default:
	ROS_WARN("Unrecognized argument");
	printUsage(usage);
	break;
      } 
    } else {
      ROS_WARN("All arguments must start with '-'.  Skipping argument %d",
	       curr_arg);
    }
    curr_arg++;
  }

}

// void create_pushing_object(arm_navigation_msgs::CollisionObject &object) {

//   object.id = "pushable_object";
//   object.operation.operation = 
//     arm_navigation_msgs::CollisionObjectOperation::ADD;
//   object.header.frame_id = "/base_link";
//   object.header.stamp = ros::Time::now();
//   arm_navigation_msgs::Shape shape;
//   shape.type = arm_navigation_msgs::Shape::SPHERE;
//   shape.dimensions.resize(1);
//   shape.dimensions[0] = 0.07;
//   geometry_msgs::Pose pose;
//   pose.position.x = 0.52;
//   pose.position.y = -0.07;
//   pose.position.z = 0.58;
//   pose.orientation.w = 1.0;
//   object.shapes.push_back(shape);
//   object.poses.push_back(pose);
// }

bool test_object_level(an::CollisionObject &obj, 
		       geometry_msgs::PoseStamped &goal,
		       ob::RealVectorBounds &bounds) {
  planning_environment::CollisionModelsInterface cmi("/robot_description");
  ros::NodeHandle n;
  ros::ServiceClient scene_client =
    n.serviceClient<an::SetPlanningSceneDiff>
    ("/environment_server/set_planning_scene_diff");
  ROS_INFO("Waiting for planning scene service");
  scene_client.waitForExistence();
  ROS_INFO("Planning scene service is now available");
  an::SetPlanningSceneDiff ssd;
  ROS_INFO("Resetting the planning scene");
  if (!scene_client.call(ssd)) {
    ROS_ERROR("Unable to reset planning scene");
    return false;
  }
  int ntries = 0;
  ros::Rate looprate(1);
  while (ntries < 10 && ros::ok()) {
    if (cmi.getPlanningSceneState()) {
      break;
    }
    ntries += 1;
    looprate.sleep();
  }

  if (!cmi.getPlanningSceneState()) {
    ROS_ERROR("No planning scene state in collision models interface");
    return false;
  }

  darrt::Object2DSolver solver(obj, &cmi);
  ROS_INFO("Collisions for object %s:", obj.id.c_str());
  std::vector<arm_navigation_msgs::ContactInformation> contacts;
  cmi.getAllEnvironmentCollisionsForObject(obj.id, contacts);
  ROS_INFO("There are %u contacts", (unsigned int)contacts.size());
  for (size_t i = 0; i < contacts.size(); i++) {
    ROS_INFO("Contact between %s and %s", 
	     contacts[i].contact_body_1.c_str(),
	     contacts[i].contact_body_2.c_str());
  }
  contacts.clear();
  an::OrderedCollisionOperations colls;
  an::CollisionOperation op;
  op.object1 = obj.id;
  op.object2 = "table";
  op.operation = op.DISABLE;
  colls.collision_operations.push_back(op);
  collision_space::EnvironmentModel::AllowedCollisionMatrix acm
    = cmi.getDefaultAllowedCollisionMatrix();
  planning_environment::applyOrderedCollisionOperationsListToACM
    (colls, std::vector<std::string>(), 
     std::vector<std::string>(), 
     cmi.getPlanningSceneState()->getKinematicModel(), 
     acm);
  cmi.setAlteredAllowedCollisionMatrix(acm);
  cmi.getAllEnvironmentCollisionsForObject(obj.id, contacts);
  ROS_INFO("After altering collision matrix there are %u contacts", 
	   (unsigned int)contacts.size());
  for (size_t i = 0; i < contacts.size(); i++) {
    ROS_INFO("Contact between %s and %s", 
	     contacts[i].contact_body_1.c_str(),
	     contacts[i].contact_body_2.c_str());
  }

  bool solved = solver.plan(goal, bounds);
  if (!solved) {
    ROS_ERROR("Unable to solve");
    return false;
  }
  solver.display_solution();
  return true;
}

bool convert_path_to_trajectory(trajectory_msgs::JointTrajectory &traj) {
  //put a reasonable amount of time between each point
  for (size_t i = 0; i < traj.points.size(); i++) {
    traj.points[i].time_from_start = ros::Duration((i+1.0)*0.25);
  }
  //make sure the rolling joints take the shortest
  //path between angles
  for (size_t j = 1; j < traj.points.size(); j++) {
    std::vector<double> newpositions;
    for (size_t i = 0; i < traj.joint_names.size(); i++) {
      double curr = traj.points[j].positions[i];
      //haha should check revolute but don't
      if (!traj.joint_names[i].compare("r_forearm_roll_joint") ||
	  !traj.joint_names[i].compare("l_forearm_roll_joint") ||
	  !traj.joint_names[i].compare("r_wrist_roll_joint") ||
	  !traj.joint_names[i].compare("l_wrist_roll_joint")) {
	double prev = traj.points[j-1].positions[i];
	while (prev - curr > darrt::MATH_PI) {
	  curr += 2.0*darrt::MATH_PI;
	}
	while (curr - prev > darrt::MATH_PI) {
	  curr -= 2.0*darrt::MATH_PI;
	}
      }
      newpositions.push_back(curr);
    }
    traj.points[j].positions = newpositions;
  }
  return true;
}

bool raise_arm() {
  //get the current state
  ros::NodeHandle n;
  ros::ServiceClient state_client = 
    n.serviceClient<arm_navigation_msgs::GetRobotState>
    ("/environment_server/get_robot_state");
  arm_navigation_msgs::GetRobotState grsrv;
  if (!state_client.call(grsrv)) {
    ROS_ERROR("Unable to get robot state");
    return false;
  }
  an::RobotState starting_state = grsrv.response.robot_state;
  //get the wrist position
  tf::StampedTransform trans;
  ros::Rate looprate(10.0);
  bool st = false;
  for (int i = 0; i < 10; i++) {
    try {
      tf_listener->lookupTransform("/torso_lift_link", "/r_wrist_roll_link",
				   ros::Time(0), trans);
      st = true;
      break;
    } catch(tf::TransformException &e) {
      ROS_WARN("Unable to transform between torso lift link and r wrist roll link.  Exception was %s", e.what());
    }
  }
  if (!st) {
    ROS_ERROR("Unable to transform from torso lift link to r_wrist_roll link");
    return false;
  }
  geometry_msgs::PoseStamped init_pose;
  init_pose.header.frame_id = "/torso_lift_link";
  init_pose.header.stamp = ros::Time(0);
  init_pose.pose.position.x = trans.getOrigin().x();
  init_pose.pose.position.y = trans.getOrigin().y();
  init_pose.pose.position.z = trans.getOrigin().z();
  init_pose.pose.orientation.x = trans.getRotation().x();
  init_pose.pose.orientation.y = trans.getRotation().y();
  init_pose.pose.orientation.z = trans.getRotation().z();
  init_pose.pose.orientation.w = trans.getRotation().w();
  
  geometry_msgs::PoseStamped final_pose = init_pose;
  final_pose.pose.position.z += 0.1;

  std::string link_name = "r_wrist_roll_link";

  an::PositionConstraint pc;
  pc.header = final_pose.header;
  pc.position = final_pose.pose.position;
  pc.link_name = link_name;
  pc.constraint_region_shape.type = pc.constraint_region_shape.BOX;
  pc.constraint_region_shape.dimensions.resize(3, 0.01);
  pc.constraint_region_orientation.w = 1.0;
  pc.weight = 1.0;

  an::OrientationConstraint oc;
  oc.header = final_pose.header;
  oc.link_name = link_name;
  oc.type = oc.HEADER_FRAME;
  oc.orientation = final_pose.pose.orientation;
  oc.absolute_roll_tolerance = 0.1;
  oc.absolute_pitch_tolerance = 0.1;
  oc.absolute_yaw_tolerance = 0.1;
  oc.weight = 1.0;
  
  an::MotionPlanRequest mpr;
  mpr.goal_constraints.position_constraints.push_back(pc);
  mpr.goal_constraints.orientation_constraints.push_back(oc);
  mpr.group_name = "right_arm";

  starting_state.multi_dof_joint_state = an::MultiDOFJointState();
  starting_state.multi_dof_joint_state.frame_ids.push_back
    (init_pose.header.frame_id);
  starting_state.multi_dof_joint_state.child_frame_ids.push_back(link_name);
  starting_state.multi_dof_joint_state.poses.push_back(init_pose.pose);
  starting_state.multi_dof_joint_state.stamp = ros::Time(0);
  mpr.start_state = starting_state;
  
  an::GetMotionPlan srv;
  srv.request.motion_plan_request = mpr;

  ros::ServiceClient int_ik_client = 
    n.serviceClient<an::GetMotionPlan>("/r_interpolated_ik_motion_plan");
  ROS_INFO("Waiting for interpolated IK solver");
  int_ik_client.waitForExistence();
  ROS_INFO("Found interpolated IK solver");
  if (!int_ik_client.call(srv)) {
    ROS_ERROR("Unable to call interpolated IK planning service");
    return false;
  }

  for (size_t i = 0; i < srv.response.trajectory_error_codes.size(); i++) {
    if (srv.response.trajectory_error_codes[i].val !=
	srv.response.trajectory_error_codes[i].SUCCESS) {
      ROS_ERROR("Unable to get interpolated IK for step %u.  Error was %d.",
		(unsigned int)i, srv.response.trajectory_error_codes[i].val);
      return false;
    }
  }

  if (!convert_path_to_trajectory(srv.response.trajectory.joint_trajectory)) {
    ROS_ERROR("Unable to convert from path to trajectory");
    return false;
  }
  
  actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>
    executor("/r_arm_controller/joint_trajectory_action");
  ROS_INFO("Waiting for joint trajectory action");
  executor.waitForServer();
  ROS_INFO("Sending trajectory");
  pr2_controllers_msgs::JointTrajectoryGoal jg;
  jg.trajectory = srv.response.trajectory.joint_trajectory;
  executor.sendGoalAndWait(jg);
  ROS_INFO("Successfully lifted arm");
  return true;
}

bool grasp(an::CollisionObject plate, geometry_msgs::PoseStamped ogoal) {
  //first move the plate to where it now is
  object_manipulation_msgs::PickupGoal pickup;
  pickup.arm_name = "right_arm";
  pickup.target.reference_frame_id = ogoal.header.frame_id;
  pickup.target.collision_name = plate.id;

  pickup.lift.direction.header.frame_id = "base_link";
  pickup.lift.direction.header.stamp = ros::Time(0);
  pickup.lift.direction.vector.x = 0;
  pickup.lift.direction.vector.y = 0;
  pickup.lift.direction.vector.z = 1;
  pickup.lift.desired_distance = 0.15;
  pickup.lift.min_distance = 0.05;

  pickup.collision_object_name = plate.id;
  pickup.collision_support_surface_name = "table";

  //give it a grasp to use on the far side of the plate
  object_manipulation_msgs::Grasp ograsp;
  ograsp.grasp_pose.position = ogoal.pose.position;
  ograsp.grasp_pose.position.x -= plate.shapes[0].dimensions[0] + 0.13;
  ograsp.grasp_pose.orientation.x = 0.707;
  ograsp.grasp_pose.orientation.w = 0.707;
  ograsp.pre_grasp_posture.name.push_back("r_gripper_l_finger_joint");
  ograsp.pre_grasp_posture.name.push_back("r_gripper_r_finger_joint");
  ograsp.pre_grasp_posture.name.push_back("r_gripper_l_finger_tip_joint");
  ograsp.pre_grasp_posture.name.push_back("r_gripper_r_finger_tip_joint");
  ograsp.grasp_posture.name.push_back("r_gripper_l_finger_joint");
  ograsp.grasp_posture.name.push_back("r_gripper_r_finger_joint");
  ograsp.grasp_posture.name.push_back("r_gripper_l_finger_tip_joint");
  ograsp.grasp_posture.name.push_back("r_gripper_r_finger_tip_joint");
  ograsp.pre_grasp_posture.position.resize(4, 0.5);
  ograsp.pre_grasp_posture.effort.resize(4, 100.0);
  ograsp.grasp_posture.position.resize(4, 0.0);
  ograsp.grasp_posture.effort.resize(4, 100.0);
  ograsp.min_approach_distance = 0.05;
  ograsp.desired_approach_distance = 0.1;
  pickup.desired_grasps.push_back(ograsp);
  pickup.only_perform_feasibility_test = false;
  ROS_INFO("blah");

  actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction>
    client("/object_manipulator/object_manipulator_pickup", true);

  ROS_INFO("Waiting for pickup server");
  client.waitForServer();
  ROS_INFO("Found pickup server!");
  client.sendGoalAndWait(pickup);
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "darrt_planning");
  ros::NodeHandle n("darrt_planning");
  
  tf_listener = new tf::TransformListener();

  //handles the callbacks for the collision model interface
  ros::AsyncSpinner spinner(1);
  spinner.start();

  objpub = n.advertise<an::CollisionObject>("/collision_object", 1);

  darrt::DARRTSolver pusher(n);
  parseArguments(argc, argv, pusher);

  arm_navigation_msgs::CollisionObject plate;
  if (!get_plate(plate)) {
    darrt::pause("Unable to find plate in collision objects.");
    create_plate(plate);
  }
  geometry_msgs::PoseStamped ogoal;
  ogoal.header = plate.header;
  ogoal.pose = plate.poses[0];
  
  //a place it can pick it up
  ogoal.header.frame_id = "/odom_combined";
  ogoal.pose.position.x = 0.35;
  ogoal.pose.position.y = -0.55;
  //ogoal.pose.position.x = 0.0;

  ros::ServiceClient state_client = 
    n.serviceClient<arm_navigation_msgs::GetRobotState>
    ("/environment_server/get_robot_state");
  arm_navigation_msgs::GetRobotState srv;
  state_client.call(srv);
  ROS_INFO_STREAM("Current state is " << srv.response.robot_state);
  geometry_msgs::PoseStamped rgoal;
  rgoal.header.frame_id = "/odom_combined";
  rgoal.pose.position.x = 0.275;
  rgoal.pose.position.y = 0.35;
  rgoal.pose.position.z = 1.05;
  rgoal.pose.orientation.x = 0.5;
  rgoal.pose.orientation.y = 0.5;
  rgoal.pose.orientation.z = 0.5;
  rgoal.pose.orientation.w = 0.5;
  //rgoal.pose.orientation.w = 1.0;
  
  darrt::DARRTGoal goal;
  //goal.robot_goal = &rgoal;
  goal.primitives.push_back(new darrt::PR2Transit("table"));
  std::vector<std::string> pushing_links;
  pushing_links.push_back("r_end_effector");
  goal.primitives.push_back(new darrt::Push(pushing_links, "table"));
  goal.object_goals[plate.id] = ogoal;
  goal.robot = "right_arm";
  //actually should be table for pushing
  //these give funny looking paths!
  ob::RealVectorBounds bounds(2);
  bounds.setLow(0, 0.2);
  bounds.setHigh(0, 1.0);
  bounds.setLow(1, -0.8);
  bounds.setHigh(1, 0.2); 
  goal.bounds = bounds;

  test_object_level(plate, ogoal, bounds);
  return 1;

  ROS_INFO("Calling plan");
  pusher.plan(goal);
  ROS_INFO("Returned from plan");
  pusher.display_solution();
  darrt::pause("Solution shown. Press enter to execute.", 0, true);
  if (!pusher.execute_solution()) {
    ROS_ERROR("Unable to execute");
    return 0;
  }
  ROS_INFO("Successful execution");
  // darrt::pause("Move the arm somewhere it can grasp from", true);
  
  //raise the arm

  plate.operation.operation = plate.operation.REMOVE;
  objpub.publish(plate);
  plate.operation.operation = plate.operation.ADD;
  plate.header.frame_id = ogoal.header.frame_id;
  plate.header.stamp = ros::Time(0);
  plate.poses[0] = ogoal.pose;
  objpub.publish(plate);
  ros::Rate looprate(10);
  //give it some time for these to go through
  for (int i =0; i < 10; i++) {
    looprate.sleep();
  }


  an::OrderedCollisionOperations rops;
  an::CollisionOperation op;
  op.object1 = "r_end_effector";
  op.object2 = "table";
  op.operation = op.DISABLE;
  rops.collision_operations.push_back(op);
  op.object2 = plate.id;
  rops.collision_operations.push_back(op);
  ros::ServiceClient scene_client =
    n.serviceClient<an::SetPlanningSceneDiff>
    ("/environment_server/set_planning_scene_diff");
  ROS_INFO("Waiting for planning scene service");
  scene_client.waitForExistence();
  ROS_INFO("Planning scene service is now available");
  an::SetPlanningSceneDiff ssd;
  ssd.request.operations = rops;
  if (!scene_client.call(ssd)) {
    ROS_WARN("Unable to disable collisions for lift");
  }

  if (!raise_arm()) {
    ROS_ERROR("Unable to raise arm");
    return 0;
  }

  ssd.request.operations.collision_operations.clear();
  if (!scene_client.call(ssd)) {
    ROS_WARN("Unable to reset planning scene");
  }

  //now grasp
  grasp(plate, ogoal);
  
  spinner.stop();
  return 0;
}
