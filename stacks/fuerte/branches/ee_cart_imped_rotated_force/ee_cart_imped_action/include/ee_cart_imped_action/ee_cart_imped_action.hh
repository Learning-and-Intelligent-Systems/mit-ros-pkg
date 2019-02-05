#ifndef __EE_CART_IMPED_ACTION_HH__
#define __EE_CART_IMPED_ACTION_HH__

#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include <ee_cart_imped_msgs/EECartImpedAction.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <string>

/**
 * \brief Action class for the 
 *<A href=http://www.ros.org/wiki/ee_cart_imped_control>EECartImped controller</A>.
 *
 *Implements the action interface to the controller.  Accepts goals, monitors
 *the trajectory to make sure it's inside constraints, publishes feedback,
 *and reports when goals have been reached.
 *
 */
class EECartImpedExecuter {
private:
  /**
   *\brief The action server type.  
   *
   *See the <A href=http://www.ros.org/wiki/actionlib>actionlib documentation</A>
   *for more details.
   *
   */
  typedef actionlib::ActionServer<ee_cart_imped_msgs::EECartImpedAction> 
  EECIAS;

  /**
   *\brief The goal handle type.  
   *
   *See the <A href=http://www.ros.org/wiki/actionlib>actionlib documentation</A>
   *for more details.
   *
   */
  typedef EECIAS::GoalHandle GoalHandle;

  ///ROS node containing namespace information, etc
  ros::NodeHandle node_;
  /**
   *\brief The action server  
   *
   *Receives goals and cancel signals and passes them to 
   *EECartImpedExecuter::goalCB() and EECartImpedExecuter::cancelCB()
   *respectively.  For more information see the 
   *<A href=http://www.ros.org/wiki/actionlib>actionlib documentation</A>.
   *
   */
  EECIAS action_server_;

  ///The result of the last goal
  ee_cart_imped_msgs::EECartImpedResult result_;
  ///Publishes commands to the controller
  ros::Publisher pub_controller_command_;
  ///Subscribes to the controller state
  ros::Subscriber sub_controller_state_;
  ///Each second this timer triggers and calls EECartImpedExecuter::watchdog()
  ros::Timer watchdog_timer_;
  ///Transformer for transforming goals into the correct frame ids
  tf::TransformListener tf_listener;

  ///True if the controller is actively pursuing a goal
  bool has_active_goal_;
  ///The handle to the current active goal
  GoalHandle active_goal_;
  ///The current trajectory the robot is following
  ee_cart_imped_msgs::EECartImpedGoal current_traj_;

  /**
   *\brief The state of the controller at the last message received via 
   *EECartImpedExecuter::sub_controller_state_
   */
  ee_cart_imped_msgs::EECartImpedFeedbackConstPtr last_controller_state_;
  
  /**
   *\brief Constraints on the goal position 
   *
   *See EECartImpedExecuter::controllerStateCB for details.
   *
   */
  geometry_msgs::Pose goal_constraints_;

  /**
   *\brief Goal constraint on the squared error of the difference between
   *the requested joint torques and the actual ones.
   *
   *See EECartImpedExecuter::controllerStateCB for details.
   *
   */
  double goal_effort_constraint_;

  /**
   *\brief If the goal takes more goal_time_constraint_ over the time it was 
   *supposed to end, it is aborted
   *
   *See EECartImpedExecuter::controllerStateCB for details.
   *
   */
  double goal_time_constraint_;


  /**
   *\brief Constraints on the trajectory position
   *
   *See EECartImpedExecuter::controllerStateCB for details.
   *
   */
  geometry_msgs::Pose trajectory_constraints_;

  /**
   *\brief Trajectory constraint on the squared error of the difference between
   *the requested joint torques and the actual ones.
   *
   *See EECartImpedExecuter::controllerStateCB for details.
   *
   */
  double trajectory_effort_constraint_;

  /**
   *\brief True if the header should be checked when receiving a new goal.
   *
   *When originally released, the header in the goal was ignored.  For continuity,
   *we have introduced this as a parameter with the default false.
   */
  bool check_header_;

  /**
   *\brief The frame in which the controller expects the trajectory.
   *
   */
  std::string controller_frame_id_;

  /**
   *\brief Triggered every second by EECartImpedExecuter::watchdog_timer_ to 
   *make sure the controller is still alive.
   *
   *Checks when the last time we heard from the controller was.  If it was more
   *than 5 seconds ago or we have never heard from it, the current goal is
   *aborted.
   *
   *This function does not check the trajectory and goal constraints; those are
   *managed in  EECartImpedExecuter::controllerStateCB().
   *
   *@param e The timer event that triggered this callback.  Not used in the
   *function since the timer just triggers every second.
   *
   */
  void watchdog(const ros::TimerEvent &e);

  /**
   *\brief Callback function when a new goal is received from 
   *EECartImpedExecuter::action_server_.
   *
   *Cancels the currently active goal if there is one and sends the new goal
   *to the controller via EECartImpedExecuter::pub_controller_command_.
   *Transforms each goal point to be in the frame of the root link for the
   *controller.
   *
   *@param gh A handle to the new goal
   *
   */
  void goalCB(GoalHandle gh);
  
  /**
   *\brief Callback function when a goal is cancelled
   *
   *@param gh The goal to cancel.
   *
   */
  void cancelCB(GoalHandle gh);
  
  /**
   *\brief Callback function when a message is received from the controller
   *via EECartImpedExecuter::sub_controller_state_.
   *
   *This function is responsible for making sure the trajectory stays within
   *EECartImpedExecuter::trajectory_constraints_ and 
   *EECartImpedExecuter::trajecotry_effort_constraint_.  If the current point is
   *not within the trajectory constraints, the goal is cancelled and the
   *trajectory is stopped.
   *
   *This function is also responsible for checking if the goal has been
   *reached.  Once the current time is greater than the time it should have
   *taken to accomplish the whole trajectory, it checks whether the current
   *point is within EECartImpedExecuter::goal_constraints_ and 
   *EECartImpedExecuter::goal_effort_constraint_ of the goal point.
   *If so, it marks the goal as succeeded.  Otherwise, it checks whether
   *the current time is greater than the time the trajectory should have been
   *finished plus EECartImpedExecuter::goal_time_constraint_.  
   *If it is and the current point
   *is not within the goal constraints, it aborts the goal.
   *
   *@param msg A pointer to a 
   *<a href=http://www.ros.org/doc/api/ee_cart_imped_msgs/html/msg/EECartImpedFeedback.html>EECartImpedFeedback</a> 
   *message that is the current controller state as published on state.
   *
   */
  void controllerStateCB(const ee_cart_imped_msgs::EECartImpedFeedbackConstPtr
			 &msg);

  /**
   *\brief Helper function to check if the actual point is within the constraints of 
   *the desired point.
   *
   *@param msg A pointer to a 
   *<a href=http://www.ros.org/doc/api/ee_cart_imped_msgs/html/msg/EECartImpedFeedback.html>EECartImpedFeedback</a> 
   *message that is the current state of the controller, 
   *including actual position and desired position and current effort error
   *@param pose_constraints The position constraints that must be respected
   *@param effort_constraint The maximum allowable error in the squared
   *difference between the desired effort and the actual effort
   *
   *@return False if msg->actual_pose is not within constraints of 
   *msg->desired.pose in any Cartesian direction or, msg->desired.isForce/Torque
   *is True in some Cartesian direction and msg->effort_sq_error > 
   *effort_constraint.
   *True otherwise.
   *
   */
  bool checkConstraints
  (const ee_cart_imped_msgs::EECartImpedFeedbackConstPtr &msg,
   geometry_msgs::Pose pose_constraints,
   double effort_constraint);
    
public:
  
  /**
   *\brief Constructs the executer.
   *
   *Constructs the "ee_cart_imped_action" action server and binds
   *EECartImpedExecuter::goalCB() and EECartImpedExecuter::cancelCB to it.
   *It then waits for the 
   *<A href=http://www.ros.org/wiki/ee_cart_imped_msgs>controller</A> 
   *to come online and, once it has, starts
   *the action server.
   *It also advertises "command",
   *subscribes to "state" (within the namespace of the passed in NodeHandle),
   *reads the constraints off the parameter server, and initializes
   *EECartImpedExecuter::watchdog_timer_ to trigger each second.
   *
   *@param n The NodeHandle for constructing this instance of the executer.
   *
   */
  EECartImpedExecuter(ros::NodeHandle &n);

  /**
   *\brief Deconstructs the executor.  Shuts down the controller and stops
   *EECartImpedExecuter::watchdog_timer_.
   */
  ~EECartImpedExecuter();
};

#endif //ee_cart_imped_action.hh
