#include <ee_cart_imped_action/ee_cart_imped_arm.hh>
#include <math.h>
#define PI 3.14159265
int main(int argc, char **argv) {
  ros::init(argc, argv, "stiffness_control_test");

  /**
   *The EECartImpedArm class is a wrapper for an action client to the
   *ee_cart_imped_action server.  The argument "r_arm_cart_imped_controller" 
   *tells the client that it is a client for the server that controls the 
   *right arm
   */
  EECartImpedArm arm("r_arm_cart_imped_controller");

  /**
   *This variable will hold the trajectory as we create it.
   */
  ee_cart_imped_control::EECartImpedGoal traj;

  /**
   *addTrajectoryPoint is a static function in the EECartImpedArm class that 
   *adds a trajectory point to the end of the first argument.  It simply 
   *assigns each value in the goal structure for us to prevent having to 
   *write it all out.
   *
   *This is a point in the center of the robot's body.  This simply moves the 
   *arm to that point with maximum stiffness.  
   */
  double x = 0.7; 
  double y = 0.0;
  double alpha = PI/20;
  int i;
  for (i = 0; i < 10; i++){
         x = x - 0.6 *(sin(alpha*(i+1)) - sin(alpha*i));
   	 y = y + 0.6 *(cos(alpha*(i+1)) - cos(alpha*i));
         EECartImpedArm::addTrajectoryPoint(traj, x, y, 0, 0, 0, 0, 1,
                                     1000, 0, 1000, 30, 30, 30,
                                     false, false, false, false, false,
                                     false, 4+i*2);
         
  }

 
  /**
   *This point is farther in front of the robot, but it is only allowed to 
   *use a very small stiffness in the x direction
   */
  /**
  EECartImpedArm::addTrajectoryPoint(traj, 0.5, 0, 0, 0, 0, 0, 1,
                                     1000, 1000, 1000, 30, 30, 30,
                                     false, false, false, false, false,
                                     false, 4);
  EECartImpedArm::addTrajectoryPoint(traj, 0.5, 0, 0, 0, 0, 0, 1,
                                     1000, 1000, 1000, 30, 30, 30,
                                     false, false, false, false, false,
                                     false, 4);
  EECartImpedArm::addTrajectoryPoint(traj,0.7, -0.2, 0, 0, 0, 0, 1,
                                     50, 1000, 1000, 30, 30, 30,
                                     false, false, false, false, false,
                                     false, 6);

  EECartImpedArm::addTrajectoryPoint(traj, 0.5, -0.2, 0, 0, 0, 0, 1,
                                     50, 1000, 1000, 30, 30, 30,
                                     false, false, false, false, false,
                                     false, 10); 
  
  
  /**
   *This is the line that actually sends the trajectory to the action server
   *and starts the arm moving.  The server will block until the arm completes 
   *the trajectory or it is aborted.
   */
  arm.startTrajectory(traj);
}

