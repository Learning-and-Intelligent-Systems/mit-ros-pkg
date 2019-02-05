#ifndef __DARRT_ACTIONS_PUSH_PLATE_HH__
#define __DARRT_ACTIONS_PUSH_PLATE_HH__

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/action_server.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <tf/transform_listener.h>

#include <darrt/darrth.hh>
#include <darrt/solver.hh>


#include "sushi_manipulation_msgs/PushPlateAction.h"

namespace darrt_actions {

  const double CANCEL_TIMEOUT = 10;

  //this might work better as a SimpleActionServer
  //but it's working now and I do not want to fuss with it
  typedef actionlib::ActionServer
  <sushi_manipulation_msgs::PushPlateAction> PPAS;
  typedef actionlib::SimpleActionClient
  <pr2_controllers_msgs::JointTrajectoryAction> JTClient;
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  
  class PushPlate {
  public:
    enum Phase {READY, WAITING, SOLVING, EXECUTING};
    PushPlate(ros::NodeHandle &n, std::string arm_name);
    bool initialized() {return initialized_;}
    ~PushPlate();

  protected:

    ros::NodeHandle node_;
    PPAS action_server_;
    PPAS::GoalHandle active_goal_handle_;
    darrt::DARRTHSolver solver_;
    JTClient joint_executor_;
    tf::TransformListener tf_listener_;
    bool has_active_goal_;
    darrt::Params info_; //should be settable by a service
    sushi_manipulation_msgs::PushPlateResult result_;
    Phase phase_;
    std::string arm_name_;
    ros::ServiceClient scene_client, controller_client, 
      controller_list_client;
    bool initialized_;
    ros::Publisher viz_;

    void goalCB(PPAS::GoalHandle gh);
    void cancelCB(PPAS::GoalHandle gh);
    bool create_goal(const an::CollisionObject &table,
		     const an::CollisionObject &plate,
		     darrt::Goal &goal);
    ob::RealVectorBounds arm_bounds();
    
  };
  
};

#endif //push_plate.hh
