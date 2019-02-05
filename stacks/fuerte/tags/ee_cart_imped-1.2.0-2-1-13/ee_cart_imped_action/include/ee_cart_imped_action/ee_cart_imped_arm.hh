#ifndef __EE_CART_IMPED_ARM_HH__
#define __EE_CART_IMPED_ARM_HH__

#include <ros/ros.h>
#include <ee_cart_imped_msgs/EECartImpedAction.h>
#include <actionlib/client/simple_action_client.h>

/**
 *\brief A simple action client for the EECartImpedAction.
 *
 *See the <A href=http://www.ros.org/wiki/actionlib>actionlib documentation</A>
 *for more details.
 *
 */
typedef actionlib::SimpleActionClient<ee_cart_imped_msgs::EECartImpedAction>
EECartImpedClient;

/**
 *\brief A wrapper around the client class for the EECartImpedAction.
 *
 *It is intended to make
 *interacting with the action server simple for most tasks.  It allows
 *a user to start or stop a trajectory.  It also includes a helper function
 *for creating trajectories more easily.
 *
 */
class EECartImpedArm {
private:
  
  /**
   *\brief The action client
   *
   *Sends goals and cancel signals to the action server.
   *See the <A href=http://www.ros.org/wiki/actionlib>actionlib documentation</A>
   *for more details.
   *
   */
  EECartImpedClient *traj_client_;

public:

  /**
   *\brief Creates a new client and waits for the server to come up
   *
   *@param ns The namespace of the action server.  Multiple action servers
   *may be running (to, for example, control both arms) so it is necessary
   *to specify for which server this client is being created.  
   *If you launch the controller and action with the 
   *provided launch files the two valid options for ns 
   *are "r_arm_cart_imped_controller" for the right arm
   *and "l_arm_cart_imped_controller" for the left arm.
   *In general, the valid choices are the namespaces of
   *any EECartImpedAction nodes.
   *
   */
  EECartImpedArm(std::string ns);

  /**
   *\brief Deletes the client.
   *
   */
  ~EECartImpedArm();

  /**
   *\brief Sends a goal to the server
   *
   *Blocks for 200s or until goal is aborted or finished.  If
   *the goal times out, it is cancelled.  If the goal has been
   *aborted (due to timeout or any other reason), a warning is printed.
   *The arm that performs the trajectory was selected through the ns
   *parameter provided to EECartImpedArm::EECartImpedArm().
   *
   *@param goal The trajectory to execute
   *@param wait If set to false this function will return immediately upon sending the goal to the action server; otherwise it will wait for the completion of the goal.
   *
   */
  void startTrajectory(ee_cart_imped_msgs::EECartImpedGoal goal,
		       bool wait=true);

  /**
   *\brief Stops the arm
   *
   *This sends the cancelAllGoals signal to the action
   *server.  This causes the arm to hold its last point.  Note that
   *"point" includes the force/stiffness.  If the arm was exerting
   *a specified force when stopTrajectory was called, it will continue
   *to exert that force.
   *
   */
  void stopTrajectory();

  /**
   *\brief Adds a trajectory point to the provided trajectory
   *
   *This is a useful function when creating a trajectory.  It takes
   *all of the necessary numbers and turns them into one trajectory
   *point, which it appends to the trajectory points already contained
   *in goal.
   *
   *@param goal The trajectory to which to add this point (at the end)
   *@param x Position in x of this point
   *@param y Position in y of this point
   *@param z Position in z of this point
   *@param ox First (x) value of the quaternion giving the Cartesian orientation at this point
   *@param oy Second (y) value of the quaternion giving the Cartesian orientation at this point
   *@param oz Third (z) value of the quaternion giving the Cartesian orientation at this point
   *@param ow Fourth (w) value of the quaternion giving the Cartesian orientation at this point
   *@param fx The force or stiffness in the x direction at this point
   *@param fy The force or stiffness in the y direction at this point
   *@param fz The force or stiffness in the z direction at this point
   *@param tx The torque or rotational stiffness around the x axis at this point
   *@param ty The torque or rotational stiffness around the y axis at this point
   *@param tz The torque or rotational stiffness around the z axis at this point
   *@param iswfx True if there is a force in the x direction, false if there is a stiffness
   *@param iswfy True if there is a force in the y direction, false if there is a stiffness
   *@param iswfz True if there is a force in the z direction, false if there is a stiffness
   *@param iswtx True if there is a torque around the x axis, false if there is a stiffness
   *@param iswty True if there is a torque around the y axis, false if there is a stiffness
   *@param iswtz True if there is a torque around the z axis, false if there is a stiffness
   *@param ts The time in seconds after the START of the trajectory that this point should
   * be achieved.  Note that this the cummulative time, NOT the time achieving this point
   * should take.  This time should be larger than the time for the point before.
   *@param frame_id The frame_id of this point.  If left as an empty string, will be interpreted as the root_name frame of the controller (probably /torso_lift_link)
   *
   *@return By reference, a new trajectory point is added to goal
   *
   */
  static void addTrajectoryPoint(ee_cart_imped_msgs::EECartImpedGoal &goal, 
				 double x, double y, double z, double ox, 
				 double oy, double oz, double ow,
				 double fx, double fy, double fz, double tx,
				 double ty, double tz, bool iswfx, bool iswfy,
				 bool iswfz, bool iswtx, bool iswty, bool iswtz,
				 double ts, std::string frame_id);
};

#endif //ee_cart_imped_arm.hh
