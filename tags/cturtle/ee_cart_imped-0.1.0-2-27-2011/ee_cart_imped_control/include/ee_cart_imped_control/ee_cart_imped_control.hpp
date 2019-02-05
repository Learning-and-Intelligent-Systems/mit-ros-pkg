#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/chain.h>
#include <pr2_mechanism_model/robot.h>

#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

#include <realtime_tools/realtime_publisher.h>

#include <ros/ros.h>
#include <ee_cart_imped_control/EECartImpedAction.h>
#include <ee_cart_imped_control/EECartImpedGoal.h>
#include <ee_cart_imped_control/StiffPoint.h>

//Doxygen doesn't like these comments for some reason...
///Maximum stiffness for translation
#define MAX_STIFFNESS 1000.0
///Maximum stiffness for rotation
#define ACCEPTABLE_ROT_STIFFNESS 100.0

namespace ee_cart_imped_control_ns {

/**
 * \brief Allows a user to control the force or stiffness applied by joints.
 *
 * This class contains the realtime controller for controlling joints
 * by stiffness or force.  To use this controller, you should use the
 * action wrapper, 
 *<A href=http://www.ros.org/wiki/ee_cart_imped_control>EECartImpedAction</A>.
 */
  class EECartImpedControlClass: public pr2_controller_interface::Controller {
  private:

    bool forcetostiffness;
    
    /// The current robot state 
    //(to get the time stamp)
    pr2_mechanism_model::RobotState* robot_state_;
    
    /// The chain of links and joints in PR2 language
    pr2_mechanism_model::Chain chain_;
    /// The chain of links and joints in KDL language
    KDL::Chain kdl_chain_;
    
    /// KDL Solver performing the joint angles to Cartesian pose calculation
    boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
    /// KDL Solver performing the joint angles to Jacobian calculation
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
    
    // The variables (which need to be pre-allocated)
    /// Joint positions
    KDL::JntArray  q_; 
    /// Joint initial positions
    KDL::JntArray  q0_;
   /// Joint velocities
    KDL::JntArrayVel  qdot_;
    /// Joint torques   
    KDL::JntArray  tau_, tau_act_;
    
    /// Tip pose
    KDL::Frame     x_; 
    /// Tip desired pose          
    KDL::Frame     xd_;
    /// Tip initial pose
    KDL::Frame     x0_; 
    
    /// Cartesian error
    KDL::Twist     xerr_;
    /// Cartesian velocity
    KDL::Twist     xdot_;
    /// Cartesian effort
    KDL::Wrench    F_, F_act_;  
    /// Starting effort  
    KDL::Wrench    F0_; 
    /// Desired Cartesian force
    KDL::Wrench    Fdes_;
    /// Jacobian
    KDL::Jacobian  J_;         
    
    ///Desired Cartesian poses
    std::vector<ee_cart_imped_control::StiffPoint>  desiredPoses_,
      actualPoses_;
    
    ///Current joint efforts
    std::vector<double> efforts_;
    
    // Note the gains are incorrectly typed as a twist,
    // as there is no appropriate type!
    /// Proportional gains
    KDL::Twist     Kp_;
    /// Derivative gains
    KDL::Twist     Kd_;   
    
    // The trajectory variables
    /// Time of the last servo cycle
    ros::Time last_time_;
    /// Start time of the current trajectory point
    ros::Time current_goal_start_time_; 
    /// Trajectory point currently being achieved
    int current_goal_index_;
    
    /**
     *\brief Callback when a new goal is received
     *
     *Cancels the currently active goal (if there is one), restarts
     *the controller and asks it to execute the new goal.
     *
     *@param msg the new goal
     *
     */
    void commandCB(const ee_cart_imped_control::EECartImpedGoalConstPtr &msg);
    ros::NodeHandle node_;
    ros::Subscriber subscriber_;
    
    /**
     * \brief Linearly interpolates between trajectory points
     *
     * Linearly interpolates between startValue and endValue over 
     * the time period startTime to endTime of the current goal.
     * @param time [seconds] that this goal has been running
     * @param startTime time that the goal began running
     * @param endTime time that the goal should finish running
     * @param startValue the values at the beginning of the interpolation (i.e. the values at startTime)
     * @param endValue the values at the end of the interpolation (i.e. the values at endTime)
     * @return ((time - startTime) * (endValue - startValue) / 
     (endTime - startTime)) + startValue; 
    */

    double linearlyInterpolate(double time, double startTime, double endTime, 
			       double startValue, double endValue);

    /**
     *\brief Samples the interpolation and returns the point the
     * joints should currently be trying to achieve.
     *
     * Samples the interpolation, using 
     * EECartImpedControlClass::linearlyInterpolate, between the
     * current trajectory point and the next trajectory point.  Interpolation
     * is done for position and orientation, but not for wrench or stiffness
     * as generally it is desired that wrench or stiffness be constant
     * over a specific trajectory segment.
     *
     * @return An <A HREF=http://www.ros.org/doc/api/ee_cart_imped_control/html/msg/StiffPoint.html>StiffPoint</A> that is the point
     * the joints should attempt to achieve on this timestep.
     *
     */
    ee_cart_imped_control::StiffPoint sampleInterpolation();
    
    ///State publisher, published every 10 updates
    boost::scoped_ptr<
      realtime_tools::RealtimePublisher<
	ee_cart_imped_control::EECartImpedFeedback> > 
    controller_state_publisher_;
    
    ///The number of updates to the joint position
    int updates_;
    ///A vector of all falses used in initialization
    std::vector<unsigned char> wrenchfalse_;
  public:
    /**
     * \brief Controller initialization in non-realtime
     *
     * Initializes the controller on first startup.  Prepares the 
     * kinematics, pre-allocates the variables, subscribes to
     * command and advertises
     * state.
     *
     * @param robot The current robot state
     * @param n A node handle for subscribing and advertising topics
     *
     * @return True on successful initialization, false otherwise
     *
     */
    bool init(pr2_mechanism_model::RobotState *robot,
	      ros::NodeHandle &n);
    /**
     * \brief Controller startup in realtime
     * 
     * Resets the controller to prepare for a new goal.  Sets the desired
     * position and orientation to be the current position and orientation
     * and sets the joints to have maximum stiffness so that they hold
     * their current position.
     */
    void starting();
    
    /**
     * \brief Controller update loop in realtime
     *
     * A PD controller for achieving the trajectory.
     * Uses EECartImpedControlClass::sampleInterpolation to
     * find the point that should be achieved on the current timestep.
     * Converts this point (which is in Cartesian coordinates) to joint
     * angles and uses a PD update (in the case of stiffness) to send
     * to the joints the correct force.
     *
     */
    void update();

    /**
     * \brief Controller stopping in realtime
     *
     * Calls EECartImpedControlClass::starting() to lock the joints into
     * their current position.
     *
     */
    void stopping();
    
  };
}

