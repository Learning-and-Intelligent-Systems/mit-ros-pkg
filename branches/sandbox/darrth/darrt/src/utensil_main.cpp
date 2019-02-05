#include "darrt/darrth.hh"
#include "darrt/solver.hh"
#include "darrt/object_solver.hh"
#include "darrt/utils.hh"
#include "darrt/pr2_primitives.hh"
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/GetRobotState.h>
#include <boost/lexical_cast.hpp>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <planning_environment/models/model_utils.h>
//you can't do this for whatever reason
//#include <tabletop_object_detector/Table.h>

#include <tf/transform_listener.h>

ros::Publisher objpub;
tf::TransformListener *tf_listener;

double TABLE_X = 0.65;
double TABLE_Y = 0.0;
double TABLE_WIDTH = 0.6;
double TABLE_HEIGHT = 3.0;

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

void create_fork(an::CollisionObject &object, double length,
		 double width, geometry_msgs::PoseStamped pose) {
  object.id = "fork";
  object.operation.operation =
    an::CollisionObjectOperation::ADD;
  object.header.frame_id = pose.header.frame_id;
  object.header.stamp = ros::Time(0);
  an::Shape shape;
  shape.type = shape.BOX;
  shape.dimensions.resize(3);
  shape.dimensions[0] = length;
  shape.dimensions[1] = width;
  shape.dimensions[2] = 0.001;
  object.shapes.push_back(shape);
  object.poses.push_back(pose.pose);
  objpub.publish(object);
  ROS_INFO_STREAM("published " << object);
  //publish a table that this is sitting on
  an::CollisionObject table;
  table.id = "table";
  table.header.frame_id = pose.header.frame_id;
  geometry_msgs::Pose tpose;
  tpose.position.x = TABLE_X;
  tpose.position.y = TABLE_Y;
  tpose.position.z = pose.pose.position.z - shape.dimensions[2]/2.0 - 0.05;
  tpose.orientation.w = 1.0;
  an::Shape tshape;
  tshape.type = shape.BOX;
  tshape.dimensions.resize(3);
  tshape.dimensions[0] = TABLE_WIDTH;
  tshape.dimensions[1] = TABLE_HEIGHT;
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


void create_fork(an::CollisionObject &object, std::string frame_id) {
  geometry_msgs::PoseStamped pose;
  //works all the time
  //  pose.pose.position.x = 0.55;
  pose.pose.position.x = 0.55;//0.52;
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
  create_fork(object, 0.12, 0.02, opose);
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


int main(int argc, char **argv) {
  ros::init(argc, argv, "darrt_planning");
  ros::NodeHandle n("~");

  ROS_INFO("Grrr?");

  tf_listener = new tf::TransformListener();

  //handles the callbacks for the collision model interface
  ros::AsyncSpinner spinner(1);
  spinner.start();

  objpub = n.advertise<an::CollisionObject>("/collision_object", 1);

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
  ROS_INFO("Resetting the planning scene");
  if (!scene_client.call(ssd)) {
    ROS_ERROR("Unable to reset planning scene");
    return 0;
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

  arm_navigation_msgs::CollisionObject fork;
  create_fork(fork, pusher.collision_models_interface()->getWorldFrameId());

  ROS_INFO("Resetting the planning scene");
  if (!scene_client.call(ssd)) {
    ROS_ERROR("Unable to reset planning scene");
    return 0;
  }

  darrt::RectangularSurface table("table");
  if (!get_table(table)) {
    ROS_ERROR("No table in collision map");
    return 0;
  }


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


  darrt::Goal goal;
  geometry_msgs::Pose spose = fork.poses[0];
  spose.position.y -= 0.02;
  goal[fork.id] = darrt::SingleGoal();
  goal[fork.id].poses.push_back(spose);
  goal[fork.id].type = darrt::SingleGoal::POSES;
  goal[fork.id].bounds = bounds;

  goal[fork.id].allowed_time = 5000;
  goal[fork.id].threshold = 0.02;

  goal.primitives.push_back(new darrt::PR2Transit());
  goal.primitives.push_back
    (new darrt::BlackBoxWithPregrasp
     ("/manipulation_primitives_planning/pinch_grasp_planning",
      "/manipulation_primitives/pinch_grasp"));
  std::vector<std::string> touch_links;
  touch_links.push_back("r_gripper_palm_link");
  touch_links.push_back("r_gripper_r_finger_link");
  touch_links.push_back("r_gripper_l_finger_link");
  touch_links.push_back("r_gripper_r_finger_tip_link");
  touch_links.push_back("r_gripper_l_finger_tip_link");
  goal.primitives.push_back(new darrt::RigidTransfer(touch_links));
  goal.primitives.push_back(new darrt::PlacePrimitive(touch_links));
  goal.chain_name = "right_arm";
  goal.params = info;
  goal.objects.push_back(fork);
  goal.support_surfaces.push_back(&table);

  ROS_INFO("Calling configure");
  if (!pusher.configure(goal)) {
    ROS_INFO("Unable to configure pusher!");
    return 0;
  }

  
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

  solved = pusher.plan(goal);
  
  tries++;
  if (darrt::debug_level > darrt::DMINIMAL) {
    darrt::pause("Returned from plan", 0, true);
  }

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
  spinner.stop();
  return 0;
}
