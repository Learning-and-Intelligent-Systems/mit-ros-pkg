#include "darrt/control.hh"
#include "darrt/darrth.hh"
#include "darrt/solver.hh"
#include "darrt/object_solver.hh"
#include "darrt/utils.hh"
#include "darrt/pr2_arm_primitives.hh"
#include "darrt/pr2_base_primitives.hh"
#include "darrt/transform_ros_types.hh"
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
double TABLE_HEIGHT = 2.0;
double TABLE_Z = 0.8;

class Table {
public:
  gm::PoseStamped pose;
  double x_min, x_max, y_min, y_max;
};

darrt::RoundObject create_object_and_table(double radius,
					   double height, geometry_msgs::PoseStamped pose,
					   bool publish_table=true) {
  std::vector<std::string> ss(1, "table");
  darrt::RoundObject object(ss);
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
  object.poses.resize(1);
  object.poses[0].orientation.w = 1;
  an::CollisionObject obj_p = object;
  obj_p.poses[0] = pose.pose;
  objpub.publish(obj_p);
  if (!publish_table) {
    return object;
  }
  //publish a table that this is sitting on
  an::CollisionObject table;
  table.id = "table";
  table.header.frame_id = pose.header.frame_id;
  geometry_msgs::Pose tpose;
  tpose.position.x = pose.pose.position.x;
  tpose.position.y = pose.pose.position.y;
  tpose.position.z = pose.pose.position.z - shape.dimensions[1]/2.0;
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

  //give some time to notice these things
  ros::Rate looprate(10);
  for (int i = 0; i < 10; i++) {
    looprate.sleep();
  }
  return object;
}

darrt::RoundObject create_object_and_table(std::string frame_id) {
  geometry_msgs::PoseStamped pose;

  pose.pose.position.x = 0.55;
  pose.pose.position.y = 0;
  pose.pose.position.z = TABLE_Z+0.01;
  pose.pose.orientation.w = 1.0;
  pose.header.frame_id = frame_id;
  pose.header.stamp = ros::Time(0);
  return create_object_and_table(0.05, 0.02, pose);
}

darrt::SpatulaObject create_spatula(std::string frame_id) {
  std::vector<double> pdim(3), hdim(2);
  pdim[0] = 0.11;
  pdim[1] = 0.12;
  pdim[2] = 0.005;
  hdim[0] = 0.02;
  hdim[1] = 0.24;
  //this is where we want the origin to be in the torso frame
  gm::PoseStamped tpose;
  tpose.header.frame_id = frame_id;
  tpose.header.stamp = ros::Time(0);
  tpose.pose.position.x = 0.5;
  tpose.pose.position.y = -0.3;
  tpose.pose.position.z = TABLE_Z+0.02;
  double angle = darrt::MATH_PI/6.0;
  tpose.pose.orientation.x = sin(angle/2.0);
  tpose.pose.orientation.w = cos(angle/2.0);

  std::vector<std::string> ss(1, "table");
  darrt::SpatulaObject spatula = darrt::SpatulaObject("spatula", frame_id, ss, 
						      pdim, hdim, angle, tpose.pose);
  // spatula.id = "spatula";
  // spatula.operation.operation =
  //   an::CollisionObjectOperation::ADD;

  // //we have two shapes joined together
  // //a box and a handle
  // an::Shape paddle, handle;
  // paddle.type = an::Shape::BOX;
  // paddle.dimensions.resize(3);
  // paddle.dimensions[0] = 0.11;
  // paddle.dimensions[1] = 0.12;
  // paddle.dimensions[2] = 0.005;

  // handle.type = an::Shape::CYLINDER;
  // handle.dimensions.resize(2);
  // handle.dimensions[0] = 0.02;
  // handle.dimensions[1] = 0.24;

  // spatula.shapes.resize(2);
  // spatula.shapes[0] = paddle;
  // spatula.shapes[1] = handle;

  // //we put the "origin" in the center
  // spatula.poses.resize(2);
  // spatula.poses[0].position.y = paddle.dimensions[1]/2.0;
  // spatula.poses[0].orientation.w = 1;

  // double angle = darrt::MATH_PI/2.0-darrt::MATH_PI/6.0;
  // spatula.poses[1].position.y = -1.0*handle.dimensions[1]/2.0*sin(angle);
  // spatula.poses[1].position.z = handle.dimensions[1]/2.0*cos(angle);
  // spatula.poses[1].orientation.x = sin(angle/2.0);
  // spatula.poses[1].orientation.w = cos(angle/2.0);
  // spatula.header.frame_id = frame_id;
  // spatula.header.stamp = ros::Time(0);

  // //this is the inverse grasp
  // double pos_on_handle = handle.dimensions[1] - 0.1;
  // gm::Transform inv_grasp;
  // inv_grasp.translation.y = darrt::GRIPPER_LENGTH;
  // inv_grasp.translation.z = pos_on_handle/2.0;
  // inv_grasp.rotation.z = sin(-1.0*darrt::MATH_PI/4.0);
  // inv_grasp.rotation.w = cos(-1.0*darrt::MATH_PI/4.0);
  // grasp.grasp = darrt::pose_to_transform(darrt::transform_pose(darrt::transform_to_pose(inv_grasp),
  // 							       spatula.poses[1]));
  // gm::Pose origin;
  // origin.orientation.w = 1.0;
  // grasp.grasp = darrt::pose_to_transform(darrt::inverse_transform_pose(origin, grasp.grasp));

  //create the grasp
  //this is slightly wrong but will do for now
  // grasp.grasp.rotation.w = 1.0;

  // grasp.grasp.translation.x = darrt::GRIPPER_LENGTH - 0.05 + pos_on_handle*sin(darrt::MATH_PI/2.0 - angle);
  // grasp.grasp.translation.y = pos_on_handle*cos(darrt::MATH_PI/2.0 - angle);
  // gm::Transform handle_trans;
  // handle_trans.rotation = spatula.poses[1].orientation;
  // grasp.grasp = darrt::pose_to_transform(darrt::transform_pose
  // 					 (darrt::transform_to_pose(grasp.grasp), handle_trans));

  //ROS_INFO_STREAM("Grasp is\n" << grasp.grasp);

  an::CollisionObject spat_pub = spatula;

  spat_pub.poses[0] = darrt::transform_pose(spatula.poses[0], tpose.pose);
  spat_pub.poses[1] = darrt::transform_pose(spatula.poses[1], tpose.pose);

  objpub.publish(spat_pub);
  return spatula;
}

darrt::DARRTObject create_block(std::string frame_id) {
  std::vector<std::string> ss(1, "table");
  darrt::DARRTObject block(ss);

  geometry_msgs::PoseStamped pose;

  pose.pose.position.x = 0.55;
  pose.pose.position.y = 0.3;
  pose.pose.position.z = TABLE_Z+0.05;

  pose.pose.orientation.w = 1.0;  
  pose.pose.orientation.z = sin(-1.0*darrt::MATH_PI/10.0);  
  pose.pose.orientation.w = cos(-1.0*darrt::MATH_PI/10.0);
  pose.header.frame_id = frame_id;
  pose.header.stamp = ros::Time(0);
  ros::Rate looprate(10);

  block.header.frame_id = pose.header.frame_id;
  block.header.stamp = ros::Time(0);
  block.id = "block";
  block.operation.operation = an::CollisionObjectOperation::ADD;
  block.shapes.resize(1);
  block.shapes[0].type = an::Shape::BOX;
  block.shapes[0].dimensions.resize(3, 0.1);
  block.poses.resize(1);
  block.poses[0].orientation.w = 1;
  an::CollisionObject block_p = block;
  block_p.poses[0] = pose.pose;
  objpub.publish(block_p);

  for (int i = 0; i < 10; i++) {
    looprate.sleep();
  }
  return block;
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

void testLocalPlanner(darrt::DARRTSolver &solver, const darrt::Goal &goal) {
 //create a starting state
  an::RobotState robot_starting_state;
  solver.initialize_environment(robot_starting_state, goal.objects);
  solver.setup_problem(robot_starting_state, goal);
  darrt::DARRTControlSpace::ControlType c;
  //ob::State *goalstate = solver.getSpaceInformation()->allocState();
  //solver.sample_goal_state(goalstate);
  ob::StateSamplerPtr ss = solver.getSpaceInformation()->allocStateSampler();
  ob::State *sample = solver.getSpaceInformation()->allocState();

  oc::DirectedControlSamplerPtr cs = dynamic_cast<const oc::SpaceInformation *>(solver.getSpaceInformation().get())
    ->allocDirectedControlSampler();

  for (unsigned int j = 0; j < 2; j++) {
    ss->sampleUniform(sample);
    unsigned int turns = cs->sampleTo(&c, solver.starting_state(), sample);
    ROS_INFO("Returned %u turns.", turns);
  
    ob::State *prev = solver.getSpaceInformation()->allocState();
    ob::State *curr = solver.getSpaceInformation()->allocState();
    solver.getSpaceInformation()->copyState(prev, solver.starting_state());
    
    const oc::StatePropagatorPtr prop = solver.space_information()->getStatePropagator();
    
    for (unsigned int i = 0; i < turns; i++) {
      prop->propagate(prev, &c, solver.space_information()->getPropagationStepSize(), curr);
      solver.getSpaceInformation()->copyState(prev, curr);
      ROS_INFO("CHECKING VALIDITY");
      bool valid = solver.getSpaceInformation()->isValid(curr);
      ROS_INFO("VALID: %d", valid);
    }
  }
}

void testSampleFrom(darrt::DARRTSolver &solver, const darrt::Goal &goal) {
  //create a starting state
  an::RobotState robot_starting_state;
  solver.initialize_environment(robot_starting_state, goal.objects);
  solver.setup_problem(robot_starting_state, goal);
  darrt::DARRTControlSpace::ControlType c;
  // darrt::DARRTDirectedControlSampler *cs = dynamic_cast<darrt::DARRTDirectedControlSampler *>
  //   (dynamic_cast<const oc::SpaceInformation *>(solver.getSpaceInformation().get())
  //    ->allocDirectedControlSampler().get());

  oc::DirectedControlSamplerPtr dcs = dynamic_cast<const oc::SpaceInformation *>(solver.getSpaceInformation().get())
    ->allocDirectedControlSampler();
  darrt::DARRTDirectedControlSampler *cs = dynamic_cast<darrt::DARRTDirectedControlSampler *>(dcs.get());
  if (!cs) {
    ROS_ERROR("control sampler is null!");
    return;
  }
  ob::State *goalstate = solver.getSpaceInformation()->allocState();

  //create a random state
  ob::StateSamplerPtr ss = solver.getSpaceInformation()->allocStateSampler();

  ob::State *sample = solver.getSpaceInformation()->allocState();

  for (unsigned int i = 0; i < 2; i++) {
    solver.sample_goal_state(goalstate);
    ss->sampleUniform(sample);
    
    int turns = cs->sampleFrom(&c, goalstate, sample);
  
    ROS_INFO("Returned %d turns.", turns);
    
    ob::State *prev = solver.getSpaceInformation()->allocState();
    ob::State *curr = solver.getSpaceInformation()->allocState();
    solver.getSpaceInformation()->copyState(prev, goalstate);
    
    const oc::StatePropagatorPtr prop = solver.space_information()->getStatePropagator();
    
    for (unsigned int i = 0; i < -1.0*turns; i++) {
      prop->propagate(prev, &c, -1.0*solver.space_information()->getPropagationStepSize(), curr);
      solver.getSpaceInformation()->copyState(prev, curr);
      ROS_INFO("CHECKING VALIDITY");
      bool valid = solver.getSpaceInformation()->isValid(curr);
      ROS_INFO("VALID: %d", valid);
    }
  }

}


void primitiveSampleTo(darrt::DARRTSolver &solver, const darrt::Goal &goal,
		       const darrt::Primitive *prim) {
  //now everything is configured
  //yay!
  
  //we'll just test the local planner here...
  
  //create a starting state
  an::RobotState robot_starting_state;
  solver.initialize_environment(robot_starting_state, goal.objects);
  solver.setup_problem(robot_starting_state, goal);
  
  //show the starting state
  const darrt::DARRTStateSpace *dspace = 
    solver.space_information()->getStateSpace()->as<darrt::DARRTStateSpace>();
  dspace->display_state(solver.starting_state(), "starting_state", 0);
  ROS_INFO("Starting state shown");
  ROS_INFO("Calling sampleto!");
  darrt::DARRTControlSpace::ControlType c;
  unsigned int turns = prim->sampleTo(solver.starting_state(), solver.starting_state(), c.clist_);
  
  ROS_INFO("Returned %u turns.", turns);
  
  ob::State *prev = dspace->allocState();
  ob::State *curr = dspace->allocState();
  dspace->copyState(prev, solver.starting_state());
  
  const oc::StatePropagatorPtr prop = solver.space_information()->getStatePropagator();
  
  for (unsigned int i = 0; i < turns; i++) {
    prop->propagate(prev, &c, solver.space_information()->getPropagationStepSize(), curr);
    dspace->copyState(prev, curr);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "darrt_planning");
  ros::NodeHandle n("~");

  darrt::EnvironmentInterface ei("robot_description");


  tf_listener = new tf::TransformListener();


  objpub = n.advertise<an::CollisionObject>("/collision_object", 1);
  vpub = n.advertise<gm::PoseStamped>("/grasp_pose", 1);
  attached_objpub = n.advertise<an::AttachedCollisionObject>
    ("/attached_collision_object", 1);

  //need to create this before setting the plannign scene
  //darrt::DARRTHSolver pusher(n);
  darrt::debug_level = darrt::DPROPAGATE;
  darrt::do_pause = true;
  darrt::DARRTHSolver hsolver(n);
  //darrt::DARRTSolver solver(n);
  const darrt::DARRTSolver &solver = hsolver.darrt_solver();

  ros::ServiceClient scene_client =
    n.serviceClient<an::SetPlanningSceneDiff>
    ("/environment_server/set_planning_scene_diff");
  ROS_INFO("Waiting for planning scene service");
  scene_client.waitForExistence();
  ROS_INFO("Planning scene service is now available");
  an::SetPlanningSceneDiff ssd;

  darrt::RoundObject obj = create_object_and_table(solver.environment_interface()->getWorldFrameId());
  darrt::DARRTObject block = create_block(solver.environment_interface()->getWorldFrameId());
  darrt::SpatulaObject spatula = create_spatula(solver.environment_interface()->getWorldFrameId());

  darrt::PickupPrimitive::RigidGrasp spat_grasp;
  std::vector<gm::Transform> spat_grasps;
  spatula.getGrasps(spat_grasps);
  spat_grasp.grasp = spat_grasps[0];

  ROS_INFO("Resetting the planning scene");
  if (!scene_client.call(ssd)) {
    ROS_ERROR("Unable to reset planning scene");
    return 0;
  }

  darrt::MeshSurface table("table");
  darrt::MeshSurface place_table("place_table");
  if (!get_table(table)) {
    ROS_ERROR("No table in collision map");
    return 0;
  }

  darrt::Params info;
  info.darrt_allowed_time = 5000;
  info.object_allowed_time = 5000;
  info.forward = false;
  info.use_darrt = true;
  //info.goal_bias = 1;

  darrt::Goal goal;
  geometry_msgs::Pose spose;
  spose.position.x = -2.0;
  spose.position.y = 0.3;
  spose.position.z = 0.95;
  spose.orientation.w = 1.0;
  // spose.position.x = -2.0;
  // spose.position.y = 0.3;
  // spose.position.z = 0.95;
  // spose.orientation.w = 1.0;


  goal[obj.id] = darrt::SingleGoal();
  goal[obj.id].pose = spose;
  goal[obj.id].type = darrt::SingleGoal::ROTATIONALLY_SYMMETRIC_POSE;

  ob::RealVectorBounds bnds(3);

  bnds.low[0] = -5;
  bnds.high[0] = 5;
  bnds.low[1] = -5;
  bnds.high[1] = 5;
  bnds.low[2] = 0.0;
  bnds.high[2] = 1.5;

  info.bounds = bnds;

  goal[obj.id].bounds = bnds;
  goal[obj.id].threshold = 0.02;


  
  std::vector<std::string> pushing_links;
  pushing_links.push_back("r_end_effector");
  gm::Vector3 rdir;
  rdir.x = 0.0;
  rdir.y = 0.0;
  rdir.z = 1.0;

  darrt::PickupPrimitive::RigidGraspList grasps;

  spat_grasp.touch_links = pushing_links;
  spat_grasp.min_distance = darrt::MIN_APPROACH_DISTANCE;
  spat_grasp.desired_distance = darrt::MAX_APPROACH_DISTANCE;
  spat_grasp.attach_link = "r_gripper_r_finger_tip_link";
  grasps.push_back(spat_grasp);

  geometry_msgs::Vector3 lift;
  lift.z = 1;

  //no projection functions since these are helper primitives
  goal.primitives.push_back(make_pair(new darrt::Retreat("right_arm"), 0));
  goal.primitives.push_back(make_pair(new darrt::Approach("right_arm"), 0));
  goal.primitives.push_back(make_pair(new darrt::PR2BaseTransit("pr2_base"), 0));
  goal.primitives.push_back(make_pair(new darrt::UseSpatula("right_arm",
							    spatula.id,
							    obj.id,
							    block.id,
							    grasps, lift), 0));
  //primitives with projection functions associated
  darrt::PR2ArmTransit *transit = new darrt::PR2ArmTransit("right_arm");
  darrt::Push *push = new darrt::Push("right_arm", obj.id, pushing_links, 0.02);
  darrt::SpatulaTransfer *utrans = new darrt::SpatulaTransfer("right_arm",
							      spatula.id,
							      obj.id);
  darrt::RigidTransfer *transfer = new darrt::RigidTransfer("right_arm", spatula.id);
  goal.primitives.push_back(make_pair(transit, 1));
  goal.primitives.push_back(make_pair(push, 1));
  goal.primitives.push_back(make_pair(utrans, 1));
  goal.primitives.push_back(make_pair(transfer, 1));

  

  //projection functions
  // goal.projection_functions.push_back(std::make_pair(boost::bind(&darrt::PR2ArmTransit::projectionFunction,
  //  								 transit, _1, _2), 1.0));
  // goal.projection_functions.push_back(std::make_pair(boost::bind(&darrt::Push::projectionFunction,
  // 								 push, _1, _2), 1.0));
  // goal.projection_functions.push_back(std::make_pair(boost::bind(&darrt::SpatulaTransfer::projectionFunction,
  // 								 utrans, _1, _2), 1.0));
  // goal.projection_functions.push_back(std::make_pair(boost::bind(&darrt::RigidTransfer::projectionFunction,
  //  								 transfer, _1, _2), 1.0));

  goal.robot_groups.push_back("right_arm");
  goal.robot_groups.push_back("pr2_base");
  goal.params = info;
  goal.objects.push_back(&obj);
  goal.objects.push_back(&block);
  goal.objects.push_back(&spatula);

  goal.support_surfaces.push_back(&table);

  ROS_INFO("Calling configure (time = %f)", goal.params.darrt_allowed_time);
  if (!hsolver.configure(goal)) {
    ROS_INFO("Unable to configure pusher!");
    return 0;
  }

  //srand(time(NULL));
  // darrt::debug_level = darrt::DPROPAGATE;
  // darrt::do_pause = true;
  // testSampleFrom(solver, goal);
  // testLocalPlanner(solver, goal);
  //return 0;

  darrt::debug_level = darrt::DRRT;
  darrt::do_pause = false;
  //goal.params.interactive = true;
  
  ROS_INFO("Calling solve!");
  hsolver.plan(goal);

  // ROS_INFO("Getting solution path!");
  //const oc::PathControl *path = hsolver.getSolutionPath();
  //solver.display_solution(*path, true);

  // darrt::pause("Ready to execute?");
  // solver.execute_solution();

  ROS_INFO("Calling reset");
  hsolver.reset();
  ROS_INFO("Done with reset");
  delete tf_listener;
  //spinner.stop();
  return 0;
}
