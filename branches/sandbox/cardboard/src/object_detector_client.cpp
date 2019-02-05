
#include <cardboard/common.h>
#include <cardboard/DetectModels.h>
#include <furniture/Multiplanar_Library.h>
#include <ros/rate.h>


ros::ServiceClient detect_models_client; 




/* handle an All_Hulls message */
void all_hulls_callback(furniture::All_Hulls msg)
{
  ROS_INFO("Got All_Hulls msg\n");

  geometry_msgs::Polygon table_polygon;

  if (msg.hulls.size() > 0) {
    // hack: assume first table is the one we want
    table_polygon.points.resize(msg.hulls[0].polygon.points.size());
    for (size_t i = 0; i < msg.hulls[0].polygon.points.size(); i++) {
      table_polygon.points[i].x = msg.hulls[0].polygon.points[i].x;
      table_polygon.points[i].y = msg.hulls[0].polygon.points[i].y;
      table_polygon.points[i].z = msg.hulls[0].polygon.points[i].z;
    }
  }

  cardboard::DetectModels srv;
  srv.request.header = msg.hulls[0].header;

 // srv.request.header.stamp.sec += 1.0;  //dbug -- test timeout
  srv.request.timeout.sec = 15.0;

  srv.request.surface_polygons.push_back(table_polygon);

  if (detect_models_client.call(srv))
  {
    ROS_INFO("OK");
  }
  else
  {
    ROS_ERROR("Failed to call service detect_models");
  }
}


/* handle a Table_Polygons message */
void table_polygons_callback(furniture::Table_Polygons msg)
{
  ROS_INFO("Got Table_Polygons msg\n");

  geometry_msgs::Polygon table_polygon;

  // hack: assume first table is the one we want
  table_polygon = msg.polygons[0];
      
  cardboard::DetectModels srv;
  srv.request.header.stamp = ros::Time::now();
  srv.request.header.frame_id = "/base_footprint";  //dbug

 // srv.request.header.stamp.sec += 1.0;  //dbug -- test timeout
  srv.request.timeout.sec = 15.0;

  srv.request.surface_polygons.push_back(table_polygon);

  if (detect_models_client.call(srv))
  {
    ROS_INFO("OK");
  }
  else
  {
    ROS_ERROR("Failed to call service detect_models");
  }
}


void shelf_callback(furniture::Multiplanar_Library msg)
{
  ROS_INFO("Got Multiplanar_Library msg\n");

  std::vector<geometry_msgs::Polygon> polygons;

  // hack: assume first model is the one we want
  polygons = msg.models[0].polygons;
      
  cardboard::DetectModels srv;
  srv.request.header.stamp = ros::Time::now();
  srv.request.header.frame_id = "/base_footprint";  //dbug

 // srv.request.header.stamp.sec += 1.0;  //dbug -- test timeout
  srv.request.timeout.sec = 15.0;

  srv.request.surface_polygons = polygons;

  if (detect_models_client.call(srv))
  {
    ROS_INFO("OK");
  }
  else
  {
    ROS_ERROR("Failed to call service detect_models");
  }
}



int main(int argc, char *argv[])
{
  // init ROS
  ros::init(argc, argv, "object_detector_client");
  ros::NodeHandle nh;
  ros::Subscriber sub_all_hulls = nh.subscribe("/convex_hulls", 1, all_hulls_callback);
  ros::Subscriber sub_poly = nh.subscribe("/table_polygons", 1, table_polygons_callback);
  ros::Subscriber sub_shelf = nh.subscribe("/multiplanar_library", 1, shelf_callback);

  detect_models_client = nh.serviceClient<cardboard::DetectModels>("detect_models");

  ros::spin();

  return 0;
}
