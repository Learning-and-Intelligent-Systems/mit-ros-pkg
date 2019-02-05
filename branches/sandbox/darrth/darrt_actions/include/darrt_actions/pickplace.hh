#ifndef __DARRT_ACTIONS_PICKPLACE_HH__
#define __DARRT_ACTIONS_PICKPLACE_HH__

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <darrt/solver.hh>
#include <darrt_msgs/PickPlaceAction.h>
#include <darrt/pr2_arm_primitives.hh>
#include <object_manipulation_msgs/GraspPlanningAction.h>
#include <tf/transform_listener.h>

namespace darrt_actions {

  const double CANCEL_TIMEOUT = 10;

  typedef actionlib::SimpleActionServer<darrt_msgs::PickPlaceAction> PPAS;

  class PickPlace {
  public:
    PickPlace(ros::NodeHandle &n);
    ~PickPlace();

  protected:

    ros::NodeHandle node_;
    PPAS action_server_;
    darrt::DARRTSolver solver_;
    tf::TransformListener tf_listener_;
    bool has_active_goal_;
    darrt::Params info_; //should be settable by a service or part of the goal
    ros::ServiceClient scene_client_, robot_client_;
    ros::Publisher viz_;
    actionlib::SimpleActionClient<object_manipulation_msgs::GraspPlanningAction> cluster_grasp_planner_,
      database_grasp_planner_;
    std::string ik_frame_;

    std::map<std::string, std::vector<std::string> > touch_links_;
    std::map<std::string, std::vector<double> > side_states_;
    std::map<std::string, std::string> attach_links_;


    void goalCB(const darrt_msgs::PickPlaceGoalConstPtr &goal_ptr);
    void preemptCB();

    std::string getClusterGraspPlannerName();
    std::string getDatabaseGraspPlannerName();
    bool getGrasps(const object_manipulation_msgs::PickupGoal &pg, const gm::PoseStamped &object_pose, 
		   darrt::PickupPrimitive::RigidGraspList &grasps);
    bool getObject(std::string name, an::CollisionObject &obj);
    bool getTable(darrt::MeshSurface &table);
    ob::RealVectorBounds getBounds(const std::vector<const darrt::SupportSurface *> tables, 
				   const std::vector<gm::Pose> &poses);
  };
  
};

#endif //push_plate.hh
