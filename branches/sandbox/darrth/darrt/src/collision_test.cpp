#include "darrt/darrth.hh"
#include "darrt/solver.hh"
#include "darrt/object_solver.hh"
#include "darrt/utils.hh"
#include "darrt/pr2_arm_primitives.hh"
#include "darrt/pr2_base_primitives.hh"
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

ros::Publisher objpub;
tf::TransformListener *tf_listener;

void create_plate(an::CollisionObject &object, double radius,
		  double height, geometry_msgs::PoseStamped pose) {
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
  //give some time to notice these things
  ros::Rate looprate(10);
  for (int i = 0; i < 10; i++) {
    looprate.sleep();
  }
}


void create_random_thing(int id) {
  geometry_msgs::Pose pose;
  pose.position.x = rand()/(double)RAND_MAX*5.0 - 2.0;
  pose.position.y = rand()/(double)RAND_MAX*6 - 4.0;
  pose.position.z = rand()/(double)RAND_MAX*2.0;
  pose.orientation.w = 1.0;
  an::Shape shape;
  shape.type = an::Shape::BOX;
  int ndims;
  switch(shape.type) {
  case an::Shape::SPHERE:
    ndims = 1;
    break;
  case an::Shape::CYLINDER:
    ndims = 2;
    break;
  case an::Shape::BOX:
    ndims = 3;
    break;
  }
  shape.dimensions.resize(ndims);
  for (int i = 0; i < ndims; i++) {
    shape.dimensions[i] = rand()/(double)RAND_MAX*0.5;
  }
  an::CollisionObject thing;
  thing.id = "random_thing_"+makestring(id);
  thing.operation.operation = thing.operation.ADD;
  thing.header.frame_id = "/map";
  thing.poses.push_back(pose);
  thing.shapes.push_back(shape);
  objpub.publish(thing);
}

void create_plate(an::CollisionObject &object, std::string frame_id) {
  geometry_msgs::PoseStamped pose;
  //works all the time
  //  pose.pose.position.x = 0.55;
  pose.pose.position.x = -100;//0.55;//1.55;//0.55;//0.52;
  pose.pose.position.y = -0.6;
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
  create_plate(object, 0.12, 0.05, opose);
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


  tf_listener = new tf::TransformListener();

  //handles the callbacks for the collision model interface
  //shouldn't need this any more :)
  //ros::AsyncSpinner spinner(1);
  //spinner.start();

  objpub = n.advertise<an::CollisionObject>("/collision_object", 1);
  ros::Publisher vpub;
  vpub = n.advertise<visualization_msgs::Marker>("occupancy_grid", 1);

  //darrt::DARRTHSolver pusher(n);
  darrt::DARRTSolver pusher(n);

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
    return 0;
  }


  darrt::debug_level = darrt::DCOLLISIONS;

  // if (!get_plate(plate)) {
  //   darrt::pause("Unable to find plate in collision objects.");
  //   create_plate(plate, pusher.environment_interface()->getWorldFrameId());
  // }

  std::vector<std::string> ss(1, "far_corner");
  darrt::DARRTObject plate(ss);
  plate.poses.push_back(gm::Pose());
  plate.poses[0].orientation.w = 1.0;
  plate.shapes.push_back(an::Shape());
  plate.shapes[0].type = an::Shape::SPHERE;
  plate.shapes[0].dimensions.push_back(0.1);

  ros::Rate r(100);
  for (int i = 0; i < 10; i++) {
    create_random_thing(i);
    r.sleep();
  }

  ROS_INFO("Resetting the planning scene");
  if (!scene_client.call(ssd)) {
    ROS_ERROR("Unable to reset planning scene");
    return 0;
  }


  darrt::Goal goal;
  geometry_msgs::Pose spose;
  spose.position.x = 1.5;
  spose.position.y = 0.2;
  spose.position.z = 0.0;plate.poses[0].position.z;
  spose.orientation.w = 1.0;
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

  ob::RealVectorBounds bnds = arm_bounds();
  goal[plate.id].bounds = bnds;
  goal[plate.id].allowed_time = 5000;
  goal[plate.id].threshold = 0.02;

  goal.robot_groups.push_back("right_arm");
  goal.robot_groups.push_back("pr2_base");
  goal.objects.push_back(&plate);

  ROS_INFO("Calling configure (time = %f)", goal.params.darrt_allowed_time);
  if (!pusher.configure(goal)) {
    ROS_INFO("Unable to configure pusher!");
    return 0;
  }
  

  const darrt::DARRTStateSpace *space = 
    pusher.space_information()->getStateSpace()->as<darrt::DARRTStateSpace>();
  const darrt::CompoundRobotStateSpace *rspace =
    space->robot_state_space()->as<darrt::CompoundRobotStateSpace>();
  ob::StateSamplerPtr sampler = rspace->allocStateSampler();
  ob::State *rstate = rspace->allocState();
  const darrt::DARRTStateValidityChecker *sv = 
    dynamic_cast<const darrt::DARRTStateValidityChecker *>
    (pusher.space_information()->getStateValidityChecker().get());


  ros::Rate rr(500);
  for (int i = 0; i < 10; i++) {
    vpub.publish(pusher.environment_interface()->getGridMarker());
    rr.sleep();
  }
  sampler->sampleUniform(rstate);
  ROS_INFO("Checking validity");
  bool valid = sv->state_validity_checker(space->robot_index())->
    isValid(rstate);
  ROS_INFO("Valid: %d", valid);
  ROS_INFO("About to begin collision checking");
  std::string buff;
  getline(std::cin, buff);
  //starting state
  rspace->convert_robot_to_ompl_state(ssd.response.planning_scene.robot_state, rstate);
  valid = sv->state_validity_checker(space->robot_index())->isValid(rstate);
  ROS_INFO("Starting state valid: %d", valid);

  for (int i = 0; i < 1000; i++) {
    ROS_INFO("i = %d", i);
    sampler->sampleUniform(rstate);
    ros::Rate r(500);
    for (int i = 0; i < 10; i++) {
      rspace->display_state(rstate, "check_state", 0);
      r.sleep();
    }
    ROS_INFO("Checking validity");
    valid = sv->state_validity_checker(space->robot_index())->
      isValid(rstate);
    ROS_INFO("Valid: %d", valid);
    //std::string buff;
    //getline(std::cin, buff);
  }
  
  delete tf_listener;
  //spinner.stop();
  return 0;
}
