#include <ros/ros.h>
#include "darrt/support_surface.hh"
#include "darrt/utils.hh"
#include <visualization_msgs/MarkerArray.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include "darrt/transform_ros_types.hh"

namespace vm = visualization_msgs;
namespace an = arm_navigation_msgs;

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

gm::PoseStamped random_pose() {
  gm::PoseStamped ps;
  ps.header.frame_id = "map";
  ps.pose.position.x = rand()/(double)RAND_MAX*4.0 - 2.0;
  ps.pose.position.y = rand()/(double)RAND_MAX*4.0 - 2.0;
  ps.pose.position.z = rand()/(double)RAND_MAX + 0.5;
  ps.pose.orientation.w = 1.0;
  return ps;
}

void testTableContains() {
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<vm::MarkerArray>("table_test_pts", 1); 
  ros::Publisher ppub = n.advertise<gm::PoseStamped>("table_pose", 1); 
  darrt::MeshSurface table("center_table");
  get_table(table);
  gm::Point p;
  vm::MarkerArray marray;
  srand(time(NULL));
  for (int j = 0; j < table.get_mesh().vertices.size(); j++) {
    gm::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position = darrt::transform_point(table.get_mesh().vertices[j], table.get_pose());
    ps.pose.orientation.w = 1.0;
    marray.markers.push_back(darrt::make_marker(ps, vm::Marker::CUBE, 0.05, 1.0, 1.0, 0.0, 0.8,
						"vertices", j));
  }
  for (int i = 0; i < 100; i++) {
    ROS_INFO("i = %d", i);
    while (ros::ok()) {
      gm::PoseStamped ps = random_pose();
      gm::Point tpt = ps.pose.position;
      tpt = darrt::inverse_transform_point(tpt, table.get_pose());
      tpt.z = 0.0;
      tpt = darrt::transform_point(tpt, table.get_pose());
      if (!table.on_surface(tpt)) {
      	continue;
      }
      gm::PoseStamped ps2 = random_pose();
      gm::PoseStamped psi = ps2;
      psi.pose.position = table.last_point_on_surface(ps.pose.position, ps2.pose.position, 0.15);
      ROS_INFO_STREAM("ps =\n" << ps << "\nps2 =\n" << ps2 << "\nPoint is\n" << psi);
      marray.markers.push_back(darrt::make_marker(ps, vm::Marker::SPHERE, 0.05, 0.0, 1.0, 0.0, 0.8, 
						  "points_"+makestring(i), 0));
      marray.markers.push_back(darrt::make_marker(ps2, vm::Marker::SPHERE, 0.05, 1.0, 0.0, 0.0, 0.8, 
						  "points_"+makestring(i), 1));
      marray.markers.push_back(darrt::make_marker(psi, vm::Marker::SPHERE, 0.05, 0.0, 0.0, 1.0, 0.8, 
						  "points_"+makestring(i), 2));
      ROS_INFO("Before break");
      break;
    }
  }
  ros::Rate r(10);
  for (unsigned int i = 0; i < 10; i++) {
    pub.publish(marray);
    gm::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose = table.get_pose();
    ppub.publish(ps);
    r.sleep();
  }
}

void testSamplePoints() {
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<vm::MarkerArray>("table_test_pts", 1); 
  darrt::MeshSurface table("table");
  get_table(table);

  vm::MarkerArray marray;
  srand(time(NULL));
  for (unsigned int i = 0; i < 100; i++) {
    gm::PoseStamped ps;
    ps.header.frame_id = "odom_combined";
    ps.pose.orientation.w = 1.0;
    ps.pose.position = table.randomPointOnSurface();
    marray.markers.push_back(darrt::make_marker(ps, vm::Marker::SPHERE, 0.05, 0.0, 1.0, 0.0, 0.8, 
						"points_"+makestring(i), 0));
  }
  ros::Rate r(10);
  for (unsigned int i = 0; i < 10; i++) {
    pub.publish(marray);
    r.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "table_tester_node");
  testSamplePoints();
}
