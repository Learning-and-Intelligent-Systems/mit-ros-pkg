#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

#include "sushi_manipulation_msgs/PushPlateAction.h"
#include <darrt/utils.hh>

namespace an = arm_navigation_msgs;

ros::Publisher objpub;

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
}

bool get_plate() {

  ros::NodeHandle n;
  ros::ServiceClient client =
    n.serviceClient<an::SetPlanningSceneDiff>
    ("/environment_server/set_planning_scene_diff");
  an::SetPlanningSceneDiff srv;
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
    return true;
  }


  //create a plate from the pose and dimension of the bounding box
  geometry_msgs::PoseStamped pose;
  pose.header = objects[objind].header;
  pose.pose = objects[objind].poses[0];
  an::CollisionObject plate;
  create_plate(plate, objects[objind].shapes[0].dimensions[0]/2.0,
	       objects[objind].shapes[0].dimensions[2], pose);
  //remove the box
  objects[objind].operation.operation = objects[objind].operation.REMOVE;
  objpub.publish(objects[objind]);
  return true;
}



int main(int argc, char **argv) {
  ros::init(argc, argv, "push_plate_tester");
  ros::NodeHandle n;

  objpub = n.advertise<an::CollisionObject>("/collision_object", 1);
  actionlib::SimpleActionClient
    <sushi_manipulation_msgs::PushPlateAction>
    ppc("/darrt_planning/r_push_plate_action");
  ROS_INFO("Waiting for push plate action");
  ppc.waitForServer();
  ROS_INFO("Found action");
  if (!get_plate()) {
    ROS_ERROR("Unable to find plate in collision map");
    return false;
  }
  sushi_manipulation_msgs::PushPlateGoal goal;
  goal.plate_id = "plate";
  goal.surface_id = "table";
  ROS_INFO("Sending goal");
  ppc.sendGoalAndWait(goal);  
  ROS_INFO("Returned");
}
