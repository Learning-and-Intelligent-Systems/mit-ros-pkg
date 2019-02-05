#include "darrt/darrth.hh"
#include "darrt/solver.hh"
#include "darrt/object_solver.hh"
#include "darrt/utils.hh"
#include "darrt/pr2_arm_primitives.hh"
#include "darrt/pr2_base_primitives.hh"
#include <arm_navigation_msgs/AttachedCollisionObject.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/GetRobotState.h>
#include <boost/lexical_cast.hpp>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <planning_environment/models/model_utils.h>
//you can't do this for whatever reason
//#include <tabletop_object_detector/Table.h>

//an action or planning service would need:
//collision_id of plate (as a cylinder or rectangle) OR
//radius, height, pose stamped of plate
//arm to use
//Table message OR pose stamped goal

#include <tf/transform_listener.h>

ros::Publisher objpub, attached_objpub;
ros::Publisher vpub;
tf::TransformListener *tf_listener;

double TABLE_X = 0.65;
double TABLE_Y = 0.0;
double TABLE_WIDTH = 0.6;
double TABLE_HEIGHT = 0.6;

class Table {
public:
  gm::PoseStamped pose;
  double x_min, x_max, y_min, y_max;
};

void printUsage(bool &print_again) {
  if (print_again) {
    ROS_INFO("Usage:");
    ROS_INFO("-h: use DARRTH");
    ROS_INFO("-d#: debug level");
    ROS_INFO("-b#: goal bias");
    ROS_INFO("-r: run through non-error pauses");
  }
  print_again = false;
}

void create_plate(an::CollisionObject &object, double radius,
		  double height, geometry_msgs::PoseStamped pose,
		  bool publish_table=true) {
  object.id = "round_plate";
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
  ROS_INFO_STREAM("published " << object);
  if (!publish_table) {
    return;
  }
  //publish a table that this is sitting on
  an::CollisionObject table;
  table.id = "table";
  table.header.frame_id = pose.header.frame_id;
  geometry_msgs::Pose tpose;
  tpose.position.x = pose.pose.position.x;
  tpose.position.y = pose.pose.position.y;
  tpose.position.z = pose.pose.position.z - 0.05 - shape.dimensions[1]/2.0;
  tpose.orientation.w = 1.0;
  an::Shape tshape;
  tshape.type = shape.MESH;
  tshape.vertices.resize(4);
  tshape.vertices[0].x = -TABLE_WIDTH/2.0;
  tshape.vertices[0].y = -TABLE_HEIGHT/2.0;
  tshape.vertices[1].x = TABLE_WIDTH/2.0;
  tshape.vertices[1].y = -TABLE_HEIGHT/2.0;
  tshape.vertices[2].x = TABLE_WIDTH/2.0;
  tshape.vertices[2].y = TABLE_HEIGHT/2.0;
  tshape.vertices[3].x = -TABLE_WIDTH/2.0;
  tshape.vertices[3].y = TABLE_HEIGHT/2.0;
  tshape.triangles.resize(6);
  tshape.triangles[0] = 0;
  tshape.triangles[1] = 1;
  tshape.triangles[2] = 2;
  tshape.triangles[3] = 2;
  tshape.triangles[4] = 3;
  tshape.triangles[5] = 0;

  table.shapes.push_back(tshape);
  table.poses.push_back(tpose);
  ROS_INFO("Publishing table!");
  objpub.publish(table);
  //publish another table
  table.id = "place_table";
  table.poses[0].position.x = -2;
  table.poses[0].position.y = 1;
  objpub.publish(table);

  //give some time to notice these things
  ros::Rate looprate(10);
  for (int i = 0; i < 10; i++) {
    looprate.sleep();
  }
}


void create_plate(an::CollisionObject &object, std::string frame_id) {
  geometry_msgs::PoseStamped pose;
  //works all the time
  //  pose.pose.position.x = 0.55;
  pose.pose.position.x = 0.55;
  pose.pose.position.y = 0;
  pose.pose.position.z = -0.3;
  pose.pose.orientation.w = 1.0;
  pose.header.frame_id = "/torso_lift_link";
  pose.header.stamp = ros::Time(0);
  ros::Rate looprate(10);
  gm::PoseStamped opose;
  bool trans = false;
  opose.header.stamp = ros::Time(0);
  for (int i = 0; i < 10; i++) {
    try {
      tf_listener->transformPose(frame_id, pose, opose);
      trans = true;
      break;
    } catch (tf::TransformException &e) {}
    looprate.sleep();
  }
  if (!trans) {
    opose = pose;
  }
  create_plate(object, 0.2, 0.05, opose);
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
  for (unsigned int i = 0; 
       i < srv.response.planning_scene.attached_collision_objects.size(); i++){
    an::CollisionObject ao = 
      srv.response.planning_scene.attached_collision_objects[i].object;
    gm::PoseStamped ap, mp;
    ap.header = ao.header;
    ap.header.stamp = ros::Time(0);
    ap.pose = ao.poses[0];
    if (!tf_listener->waitForTransform
	("map", ao.header.frame_id, ros::Time(0), ros::Duration(2))) {
      ROS_ERROR("Unable to get transform for object %s", ao.id.c_str());
      continue;
    }
    tf_listener->transformPose("map", ap, mp);
    ao.header = mp.header;
    ao.poses[0] = mp.pose;
    objects.push_back(ao);
  }
  int objind = -1;
  for (size_t i = 0; i < objects.size(); i++) {
    if (!objects[i].id.compare("round_plate")) {
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
  if (!objects[objind].id.compare("round_plate")) {
    //assume we've also corrected for a thicker table
    plate = objects[objind];
    return true;
  }


  //create a plate from the pose and dimension of the bounding box
  geometry_msgs::PoseStamped pose;
  pose.header = objects[objind].header;
  pose.pose = objects[objind].poses[0];
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  create_plate(plate, objects[objind].shapes[0].dimensions[0]/2.0,
  	       objects[objind].shapes[0].dimensions[2], pose);
  //create_plate(plate, 0.15,
  //	       objects[objind].shapes[0].dimensions[2], pose);
  //remove the box
  objects[objind].operation.operation = objects[objind].operation.REMOVE;
  objpub.publish(objects[objind]);
  return true;
}

bool get_table(darrt::MeshSurface &table) {
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
  int tableind = -1;
  for (size_t i = 0; i < objects.size(); i++) {
    if (!objects[i].id.compare(table.name())) {
      tableind = i;
      break;
    }
  }

  std::string objectstr;
  for (size_t i = 0; i < objects.size(); i++) {
    objectstr += " (" + objects[i].id + ")";
  }
  if (tableind < 0) {
    ROS_ERROR("Unable to find table in planning scene.  Objects are: %s",
	      objectstr.c_str());
    return false;
  }
  table.set_pose_and_mesh(objects[tableind].poses[0], objects[tableind].shapes[0]);
  return true;

}

bool get_table(darrt::RectangularSurface &table) {
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
  int tableind = -1;
  for (size_t i = 0; i < objects.size(); i++) {
    if (!objects[i].id.compare(table.name())) {
      tableind = i;
      break;
    }
  }

  std::string objectstr;
  for (size_t i = 0; i < objects.size(); i++) {
    objectstr += " (" + objects[i].id + ")";
  }
  if (tableind < 0) {
    ROS_ERROR("Unable to find table in planning scene.  Objects are: %s",
	      objectstr.c_str());
    return false;
  }
  
  const an::CollisionObject &ctable = objects[tableind];

  //assume this is already axis oriented hahaha
  
  table.x_min = -1.0*ctable.shapes[0].dimensions[0]/2.0 + 
    ctable.poses[0].position.x;
  table.x_max = ctable.shapes[0].dimensions[0]/2.0 + 
    ctable.poses[0].position.x;
  table.y_min = -1.0*ctable.shapes[0].dimensions[1]/2.0 + 
    ctable.poses[0].position.y;
  table.y_max = ctable.shapes[0].dimensions[1]/2.0 + 
    ctable.poses[0].position.y;
  table.height = ctable.poses[0].position.z + 
    ctable.shapes[0].dimensions[2]/2.0;
  return true;
}

void attachPlate() {
  //figure out where the gripper is
  gm::PoseStamped ppose;

  ppose.header.frame_id = "r_wrist_roll_link";
  ppose.pose.position.x = (0.1+darrt::GRIPPER_LENGTH);
  ppose.pose.orientation.x = 0.707;
  ppose.pose.orientation.w = 0.707;

  an::AttachedCollisionObject plate;
  create_plate(plate.object, 0.2, 0.05, ppose, false);
  plate.object.operation.operation = 
    plate.object.operation.ATTACH_AND_REMOVE_AS_OBJECT;
  plate.link_name = "r_wrist_roll_link";
  plate.touch_links.push_back("r_end_effector");
  attached_objpub.publish(plate);
  ROS_INFO_STREAM("Attached object\n" << plate);
}


void parseArguments(int argc, char **argv,
		    darrt::Params &info) {
  int curr_arg = 1;
  bool usage=true;

  while (curr_arg < argc) {
    if (argv[curr_arg][0] == '-') {
      switch(argv[curr_arg][1]) {
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
      case 'h':
	info.use_darrt = false;
	info.darrt_allowed_time = 5.0;
	break;
      case 'a':
	attachPlate();
	break;
      case 'b':
	{
	  bool badcast = false;
	  try {
	    double b = boost::lexical_cast<double>(&(argv[curr_arg][2]));
	    if (b <= 0 || b > 1.01) {
	      badcast = true;
	    } else {
	      info.goal_bias = b;
	      if (info.goal_bias > 1) {
		info.goal_bias = 1;
	      }
	    }
	  } catch (boost::bad_lexical_cast &) {
	    badcast = true;
	  }
	  if (badcast) {
	    ROS_ERROR("Unable to interpret argument for goal bias.  Bias is %f",
		      info.goal_bias);
	    printUsage(usage);
	  }
	  break;
	}
      case 'i':
	info.interactive = true;
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

ob::RealVectorBounds arm_bounds() {
  //bounds for where the object can go
  //these shuold really be the arm's workspace
  //these are in base_footprint
  //transform them to odom_combined
  geometry_msgs::PointStamped minpt, maxpt, ominpt, omaxpt;
  minpt.header.frame_id = "/base_footprint";
  minpt.header.stamp = ros::Time(0);
  minpt.point.x = -0.2;
  minpt.point.y = -0.99;
  ominpt.header.frame_id = "/odom_combined";
  ominpt.header.stamp = ros::Time(0);

  maxpt.header.frame_id = "/base_footprint";
  maxpt.header.stamp = ros::Time(0);
  maxpt.point.x = 1.0;
  maxpt.point.y = 0.2;
  omaxpt.header.frame_id = "/odom_combined";
  omaxpt.header.stamp = ros::Time(0);
  ros::Rate looprate(10);
  bool trans = false;
  for (int i =0; i < 10; i++) {
    try {
      tf_listener->transformPoint("/odom_combined", minpt, ominpt);
      tf_listener->transformPoint("/odom_combined", maxpt, omaxpt);
      trans = true;
      break;
    } catch(tf::TransformException &e) {
      ROS_WARN("Unable to transform bounds to odom_combined: %s", e.what());
    }
    looprate.sleep();
  }
  if (!trans) {
    ROS_ERROR("Cannot transform bounds");
    return 0;
  }

  double minx = ominpt.point.x, maxx = omaxpt.point.x, 
    miny = ominpt.point.y, maxy = omaxpt.point.y;
  
  ROS_INFO("Bounds are ([%f, %f], [%f, %f])", minx, maxx, miny, maxy);

  if (minx > maxx) {
    minx = maxx;
    maxx = ominpt.point.x;
  }
  if (miny > maxy) {
    miny = maxy;
    maxy = ominpt.point.y;
  }
  ob::RealVectorBounds bounds(3);
  bounds.setLow(0, minx);
  bounds.setHigh(0, maxx);
  bounds.setLow(1, miny);
  bounds.setHigh(1, maxy);
  bounds.setLow(2, 0);
  bounds.setHigh(2, 2);
  return bounds;
}


int main(int argc, char **argv) {
  std::cout << "Hi!";
  ros::init(argc, argv, "darrt_planning");
  ros::NodeHandle n("~");

  std::cout << "Created node handle!";
  darrt::EnvironmentInterface ei("robot_description");
  std::vector<std::string> mg;
  ei.getKinematicModel()->getModelGroupNames(mg);
  ROS_INFO("Model groups are");
  for (unsigned int i = 0; i < mg.size(); i++) {
    ROS_INFO("%s", mg[i].c_str());
  }


  ROS_INFO("Grrr?");

  tf_listener = new tf::TransformListener();

  //handles the callbacks for the collision model interface
  //shouldn't need this any more :)
  //ros::AsyncSpinner spinner(1);
  //spinner.start();

  objpub = n.advertise<an::CollisionObject>("/collision_object", 1);
  attached_objpub = n.advertise<an::AttachedCollisionObject>
    ("/attached_collision_object", 1);

  //need to create this before setting the plannign scene
  darrt::DARRTHSolver pusher(n);
  //darrt::DARRTSolver pusher(n);

  ros::ServiceClient scene_client =
    n.serviceClient<an::SetPlanningSceneDiff>
    ("/environment_server/set_planning_scene_diff");
  ROS_INFO("Waiting for planning scene service");
  scene_client.waitForExistence();
  ROS_INFO("Planning scene service is now available");
  an::SetPlanningSceneDiff ssd;

  arm_navigation_msgs::CollisionObject plate;
  if (!get_plate(plate)) {
    darrt::pause("Unable to find plate in collision objects.");
    create_plate(plate, pusher.environment_interface()->getWorldFrameId());
  }


  darrt::MeshSurface table("table");
  darrt::MeshSurface place_table("place_table");
  if (!get_table(table)) {
    ROS_ERROR("No table in collision map");
    return 0;
  }
  bool use_place_table = true;
  if (!get_table(place_table)) {
    ROS_WARN("No place table in collision map");
    use_place_table = false;
  }


  darrt::Params info;
  parseArguments(argc, argv, info);
  if (darrt::debug_level > darrt::DMINIMAL) {
    if (darrt::do_pause) {
      info.darrt_allowed_time = 5000;
    } else if (info.use_darrt) {
      info.darrt_allowed_time = 15;
    }
  }
  info.darrt_allowed_time = 300;
  info.object_allowed_time = 300;


  ROS_INFO("Resetting the planning scene");
  if (!scene_client.call(ssd)) {
    ROS_ERROR("Unable to reset planning scene");
    return 0;
  }


  darrt::Goal goal;
  geometry_msgs::Pose spose;
  spose.position.x = -1;
  spose.position.y = 0;
  spose.position.z = 0.5;
  spose.orientation.w = 1.0;

  // spose.position.x = -2;
  // spose.position.y = 1;
  // spose.position.z = 0.5;
  // spose.orientation.w = 1.0;
  //spose.orientation.z = sin(darrt::MATH_PI/2.0);
  //spose.orientation.w = cos(darrt::MATH_PI/2.0);
  goal[plate.id] = darrt::SingleGoal();
  goal[plate.id].pose = spose;
  goal[plate.id].type = darrt::SingleGoal::ROTATIONALLY_SYMMETRIC_POSE;
  //goal[plate.id].poses.push_back(spose);

  // goal[plate.id].type = darrt::SingleGoal::POSE;
//   goal[plate.id].pose = plate.poses[0];
//   goal[plate.id].pose.position.y -= 0.3;
  //goal[plate.id].type = darrt::SingleGoal::POSES;
  //goal[plate.id].poses.push_back(plate.poses[0]);
  //goal[plate.id].poses.back().position.y -= 0.3;
//   if (!table_bounding_points(table, goal[plate.id].poses)) {
//     ROS_ERROR("Cannot get table border");
//     return 0;
//   }

  info.bounds.low[0] = -2.5;
  info.bounds.high[0] = 4;
  info.bounds.low[1] = -3;
  info.bounds.high[1] = 2.5;

  ob::RealVectorBounds bnds(3);
  for (int i = 0; i < 2; i++) {
    bnds.low[i] = info.bounds.low[i];
    bnds.high[i] = info.bounds.high[i];
  }
  bnds.low[2] = 0.0;
  bnds.high[2] = 1.5;

  goal[plate.id].bounds = bnds;
  goal[plate.id].allowed_time = 5000;
  goal[plate.id].threshold = 0.02;

  goal.primitives.push_back(new darrt::PR2ArmTransit("right_arm"));
  
  std::vector<std::string> pushing_links;
  pushing_links.push_back("r_end_effector");
  gm::Vector3 rdir;
  rdir.x = 0.0;
  rdir.y = 0.0;
  rdir.z = 1.0;

  goal.primitives.push_back(new darrt::Retreat("right_arm", 0.3));
  goal.primitives.push_back(new darrt::Push("right_arm", pushing_links));
  
  darrt::PickupPrimitive::RigidGraspList grasps;

  darrt::PickupPrimitive::RigidGrasp rg;
  rg.grasp.rotation.x = 0.707;
  rg.grasp.rotation.w = 0.707;
  rg.grasp.translation.x = 0.1+darrt::GRIPPER_LENGTH;
  rg.touch_links = pushing_links;
  rg.min_distance = darrt::MIN_APPROACH_DISTANCE;
  rg.desired_distance = darrt::MAX_APPROACH_DISTANCE;
  rg.attach_link = "r_gripper_r_finger_tip_link";
  rg.min_distance_from_surface = 0.01+darrt::GRIPPER_LENGTH;
  gm::Quaternion hand_ori = rg.grasp.rotation;
  for (int i = 0; i < 50; i++) {
    double alpha = 2.0*darrt::MATH_PI/10.0*i;
    gm::Quaternion yaw;
    yaw.z = sin(alpha/2.0);
    yaw.w = cos(alpha/2.0);
    darrt::multiply(hand_ori, yaw, rg.grasp.rotation);
    grasps.push_back(rg);
    //ROS_INFO_STREAM("Grasp is\n" << rg.grasp);
  }
  geometry_msgs::Vector3 lift;
  lift.z = 1;
  goal.primitives.push_back(new darrt::PickupPrimitive
  			    ("right_arm", plate.id, grasps, lift));
  gm::Vector3 app;
  app.z = -1.0;
  goal.primitives.push_back(new darrt::RigidTransfer("right_arm"));
  goal.primitives.push_back(new darrt::PlacePrimitive("right_arm", app, 0.1, 0.5));
  goal.primitives.push_back(new darrt::PR2BaseTransit("pr2_base"));
  //goal.primitives.push_back(new darrt::PR2BaseManipulation("pr2_base"));
  goal.robot_groups.push_back("right_arm");
  goal.robot_groups.push_back("pr2_base");
  goal.params = info;
  goal.objects.push_back(plate);

  goal.support_surfaces.push_back(&table);
  if (use_place_table) {
    goal.support_surfaces.push_back(&place_table);
  }

  ROS_INFO("Calling configure (time = %f)", goal.params.darrt_allowed_time);
  if (!pusher.configure(goal)) {
    ROS_INFO("Unable to configure pusher!");
    return 0;
  }

  //dynamic_cast<const darrt::PR2BaseManipulation *>(goal.primitives.back())->
  // base_test(goal[plate.id].poses[0]);
  //return 0;

  //test_object_level(plate, goal.object_goals[plate.id]);
  //return 1;

  ROS_INFO("Calling plan");
  //const ompl::base::State *start = NULL;
//   boost::thread planning_thread(&darrt::DARRTSolver::plan, &pusher, goal, start);
//   ros::Rate sleeper(10);
//   for (int i = 0; i < 2; i++) {
//     ROS_INFO("Main thread is sleeping...");
//     sleeper.sleep();
//   }
//   ROS_INFO("Sending cancel signal");
//   pusher.cancel();
//   ROS_INFO("Returned to main thread and joining");
//   planning_thread.join();
//   return 0;
  bool solved = false;
  int tries = 0;
  while (!solved && tries < 1 && ros::ok()) {
    ROS_INFO("Try %d", tries);
    solved = pusher.plan(goal);

    tries++;
    if (darrt::debug_level > darrt::DMINIMAL) {
      darrt::pause("Returned from plan", 0, true);
    }
  }
    // if (!pusher.plan(goal)) {
    // ROS_INFO("Unable to find a goal");
    // return 0;

  if (!solved) {
    ROS_INFO("Unable to solve");
    return 0;
  }
  pusher.display_solution();

  darrt::pause("Solution shown. Press enter to execute.", 0, true);
  if (!pusher.execute_solution()) {
    ROS_ERROR("Unable to execute");
    return 0;
  }
  ROS_INFO("Successful execution");
  
  delete tf_listener;
  //spinner.stop();
  return 0;
}
