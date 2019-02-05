/*
 *
 * dynamic_recon.cpp
 *
 *  Created on: Nov 4, 2010
 *      Author: garratt
 */



#include "ros/ros.h"
#include "dynamic_reconfigure/Reconfigure.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "projector_changer");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<dynamic_reconfigure::Reconfigure>("/camera_synchronizer_node/set_parameters");
  dynamic_reconfigure::Reconfigure srv;
  srv.request.config.ints.push_back(dynamic_reconfigure::IntParameter());
  srv.request.config.ints.back().name="projector_mode";
  srv.request.config.ints.back().value=0;
  if (client.call(srv))
  {
    ROS_INFO("Call Successful");
  }
  else
  {
    ROS_ERROR("Failed to call service /camera_synchronizer_node/set_parameters");
    return 1;
  }

  return 0;
}
