#include <actionlib/client/simple_action_client.h>
#include "sushi_manipulation_msgs/PushPlateAction.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "push_plate_tester");

  actionlib::SimpleActionClient
    <sushi_manipulation_msgs::PushPlateAction>
    ppc("/darrt_planning/r_push_plate_action");
  ROS_INFO("Waiting for push plate action");
  ppc.waitForServer();
  ROS_INFO("Canceling all goals");
  ppc.cancelAllGoals();
  ROS_INFO("Done!");
}
