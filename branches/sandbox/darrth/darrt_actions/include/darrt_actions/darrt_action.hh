#ifndef __DARRT_ACTIONS_DARRT_ACTION_HH__
#define __DARRT_ACTIONS_DARRT_ACTION_HH__

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <darrt/darrth.hh>
#include <darrt_msgs/DARRTAction.h>
#include <darrt/pr2_arm_primitives.hh>
#include <darrt/state_transformer.hh>
#include <object_manipulation_msgs/GraspPlanningAction.h>
#include <object_manipulation_msgs/PickupGoal.h>


namespace darrt_actions {

  const double CANCEL_TIMEOUT = 10;

  typedef actionlib::SimpleActionServer<darrt_msgs::DARRTAction> PPAS;

  class DARRTAction {
  public:
    DARRTAction(ros::NodeHandle &n);
    ~DARRTAction();

  protected:

    ros::NodeHandle node_;
    PPAS action_server_;
    darrt::DARRTHSolver solver_;
    bool has_active_goal_;
    darrt::Params info_; //should be settable by a service or part of the goal
    ros::ServiceClient scene_client_, robot_client_;
    ros::Publisher viz_;
    actionlib::SimpleActionClient<object_manipulation_msgs::GraspPlanningAction> cluster_grasp_planner_,
      database_grasp_planner_;
    darrt::StateTransformer transformer_;

    std::string ik_frame_, world_frame_;
    std::map<std::string, std::vector<std::string> > touch_links_;
    std::map<std::string, std::vector<double> > side_states_;
    std::map<std::string, std::string> attach_links_;


    void goalCB(const darrt_msgs::DARRTGoalConstPtr &goal_ptr);
    void preemptCB();

    void freeMemory(darrt::Goal &goal);

    bool initializePrimitive(const darrt_msgs::Primitive &primitive, darrt::Goal &goal) const;
    bool initializeArmTransit(const darrt_msgs::Primitive &primitive, darrt::Goal &goal) const;
    bool initializeBaseTransit(const darrt_msgs::Primitive &primitive, darrt::Goal &goal) const;
    bool initializeApproach(const darrt_msgs::Primitive &primitive, darrt::Goal &goal) const;
    bool initializeRetreat(const darrt_msgs::Primitive &primitive, darrt::Goal &goal) const;
    bool initializeRigidTransfer(const darrt_msgs::Primitive &primitive, darrt::Goal &goal) const;
    bool initializePush(const darrt_msgs::Primitive &primitive, darrt::Goal &goal) const;
    bool initializePickup(const darrt_msgs::Primitive &primitive, darrt::Goal &goal) const;
    bool initializeUseSpatula(const darrt_msgs::Primitive &primitive, darrt::Goal &goal) const;
    bool initializeSpatulaTransfer(const darrt_msgs::Primitive &primitive, darrt::Goal &goal)const;

    std::string getClusterGraspPlannerName();
    std::string getDatabaseGraspPlannerName();
    bool getGrasps(const object_manipulation_msgs::PickupGoal &pg, const gm::PoseStamped &object_pose, 
		   double min_distance, const an::RobotState &starting_state,
		   darrt::PickupPrimitive::RigidGraspList &grasps);
    bool getObject(std::string name, an::CollisionObject &obj);
    darrt::DARRTObject *getObject(const darrt_msgs::ObjectType &object_msg,
				  const std::vector<std::string> &support_surfaces);
    bool getTable(darrt::MeshSurface &table);
    ob::RealVectorBounds getBounds(const std::vector<const darrt::SupportSurface *> tables, 
				   const std::vector<gm::Pose> &poses, const an::RobotState &starting_state);
  };
  
};

#endif //darrt.hh
