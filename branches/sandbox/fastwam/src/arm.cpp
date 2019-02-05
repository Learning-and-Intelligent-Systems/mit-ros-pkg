#include <signal.h>

#include <ros/ros.h>
#include <fastwam/MoveTo.h>
#include <fastwam/SwingData.h>
#include <fastwam/GainTrajectory.h>
#include <fastwam/PaddleOrientationTrajectory.h>
#include <fastwam/TorqueTrajectory.h>
#include <fastwam/JointTrajectory.h>

#include "util.h"
#include <time.h>
#include <list>
#include <vector>

#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>

using namespace barrett;
using namespace std;


fastwam::GainTrajectory last_gain_msg;
bool got_gain_msg = false;



//---------------- SwingController -----------------//

template <size_t DOF>
class SwingController :
  public systems::SingleIO<typename units::JointPositions<DOF>::type,
			   typename units::JointTorques<DOF>::type> {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);



public:

  enum SwingState {GAIN_SWING, DYNAMIC_SWING, KINEMATIC_SWING, RESET, SHUTDOWN};

  SwingController(ProductManager *pm, ros::Publisher *swingdata_publisher) :
    systems::SingleIO<jp_type, jt_type>("SwingController")
  {
    PM = pm;
    wam = pm->getWam7();
    swingdata_pub = swingdata_publisher;

    // set safety limits
    PM->getSafetyModule()->setTorqueLimit(100);
    PM->getSafetyModule()->setVelocityLimit(0);

    // turn on gravity compensation
    //wam->gravityCompensate();

    // home position & current position
    jp_type jp = wam->getJointPositions();
    for (int i = 0; i < 7; i++)
      home_position[i] = current_position[i] = jp[i];

    // reset position
    for (int i = 0; i < 7; i++)
      reset_position[i] = 0;

    // intialize arm tracker
    arm_tracker = new MovingAverageArmTracker(7,5);

    // initialize sub-controllers
    pid_controller = new PID();
    paddle_orient_traj_controller = new PaddleOrientTrajectoryController(pid_controller);
    torque_traj_controller = new TorqueTrajectoryController(pid_controller);
    joint_traj_controller = new JointTrajectoryController(pid_controller);
    gain_traj_controller = new GainTrajectoryController();

    // set state to "reset"
    swing_state = RESET;
    pid_controller->setGoal(reset_position);
    control_paddle_orientation = false;

    // time
    current_time = ros::Time::now().toSec();
  }

  void setState(SwingState state)
  {
    swing_state = state;

    if (state == SHUTDOWN) {
      pid_controller->resetVelocityLimits();
      pid_controller->setGoal(home_position);
    }
    else if (state == RESET) {
      pid_controller->resetVelocityLimits();
      //pid_controller->setGoal(reset_position);
    }
  }

  void setGainTrajectory(const vector<double> &times, const vector<float> &gains)
  {
    //if (swing_state != RESET)
    //  return;

    gain_traj_controller->setGainTrajectory(times, gains);

    setState(GAIN_SWING);
  }

  void setPaddleOrientationTrajectory(const vector<double> &times, const vector<float> &orientations)
  {
    paddle_orient_traj_controller->setOrientationTrajectory(times, orientations);
    
    control_paddle_orientation = true;
  }

  void setTorqueTrajectory(const vector<int> &joints, const vector<double> &times, const vector<float> &torques)
  {
    if (swing_state != RESET)
      return;

    torque_traj_controller->setTorqueTrajectory(joints, times, torques);
    
    setState(DYNAMIC_SWING);
  }

  void setJointTrajectory(const vector<float> &joint_angles, const vector<double> &times)
  {
    if (swing_state != RESET)
      return;

    joint_traj_controller->setJointTrajectory(joint_angles, times);
    
    setState(KINEMATIC_SWING);
  }

  void setJointAngles(const vector<float> &joints)
  {
    //pid_controller->setGravityCompensation(false);

    double JA[7] = {joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], joints[6]};
    pid_controller->setGoal(JA);

    //memcpy(reset_position, JA, 7*sizeof(double));
  }

protected:
  ProductManager *PM;
  systems::Wam<DOF> *wam;
  double current_time;
  double current_position[7];
  double home_position[7];
  double reset_position[7];
  ArmTracker *arm_tracker;
  SwingState swing_state;
  bool control_paddle_orientation;
  PID *pid_controller;
  GainTrajectoryController *gain_traj_controller;
  PaddleOrientTrajectoryController *paddle_orient_traj_controller;
  TorqueTrajectoryController *torque_traj_controller;
  JointTrajectoryController *joint_traj_controller;
  ros::Publisher *swingdata_pub;
  fastwam::SwingData swingdata;



  //---------- Real-time control loop ----------//

  virtual void operate()
  {
    if (got_gain_msg) {  // add a mutux here if we want to be sure we don't miss messages...
      setGainTrajectory(last_gain_msg.times, last_gain_msg.gains);
      got_gain_msg = false;
    }

    // get current time and dt
    double t = ros::Time::now().toSec();
    //double dt = t - current_time;
    current_time = t;

    // read input (joint angles)
    jp_type jp = this->input.getValue();
    double new_position[7];
    for (int i = 0; i < 7; i++)
      new_position[i] = jp[i];

    // update arm tracker
    arm_tracker->update(new_position, current_time);

    // compute torque outputs
    double *joint_torques = NULL;

    if (swing_state == GAIN_SWING || swing_state == DYNAMIC_SWING || swing_state == KINEMATIC_SWING) {

      bool started = false;
      bool finished = false;

      if (swing_state == GAIN_SWING) {
	joint_torques = gain_traj_controller->update(arm_tracker->getJointAngles(), arm_tracker->getJointVelocities());
	started = gain_traj_controller->isStarted();
	finished = gain_traj_controller->isFinished(); // || projected_in_collision(new_position, current_position, dt);	
      }
      else if (swing_state == DYNAMIC_SWING) {
	joint_torques = torque_traj_controller->update(new_position);
	started = torque_traj_controller->isStarted();
	finished = torque_traj_controller->isFinished(); // || projected_in_collision(new_position, current_position, dt);
      }
      else {
	joint_torques = joint_traj_controller->update(new_position);
	started = joint_traj_controller->isStarted();
	finished = joint_traj_controller->isFinished();
      }

      if (control_paddle_orientation && paddle_orient_traj_controller->isStarted()) {
	if (paddle_orient_traj_controller->isFinished())
	  control_paddle_orientation = false;
	else {
	  double *wrist_torques = paddle_orient_traj_controller->update(new_position);
	  for (int i = 0; i < 3; i++)
	    joint_torques[i+4] = wrist_torques[i];
	}
      }

      printf("swinging\n");

      // log swing data
      if (started) {
	swingdata.t.push_back(t);
	for (int i = 0; i < 7; i++)
	  swingdata.joint_angles.push_back(new_position[i]);
	if (!finished)
	  for (int i = 0; i < 7; i++)
	    swingdata.joint_torques.push_back(joint_torques[i]);

	// check for impending collisions or finished trajectories
	if (finished) {

	  printf("finished\n");
	  
	  // publish & reset swingdata 
	  swingdata_pub->publish(swingdata);
	  swingdata.t.clear();
	  swingdata.joint_angles.clear();
	  swingdata.joint_torques.clear();
	  
	  pid_controller->resetAllJoints();
	  //torque_traj_controller->reset();
	  setState(RESET);
	}
      }

      if (!started || finished)
	joint_torques = pid_controller->update(new_position);
    }
    else
      joint_torques = pid_controller->update(new_position);

    //static int cnt = 0;
    //if (cnt++ < 10)
    //  printf("this is a hack\n");

    // update current joint angles
    for (int i = 0; i < 7; i++)
      current_position[i] = new_position[i];
  
    // write output (joint torques)
    jt_type jt;
    for (int i = 0; i < 7; i++)
      jt[i] = joint_torques[i];
    this->outputValue->setData(&jt);
  }
};



//---------------- Global variables ---------------//

SwingController<7> *controller;



void gain_trajectory_callback(const fastwam::GainTrajectory &msg)
{
  printf("Got GainTrajectory message...\n");

  if (!got_gain_msg) {  // only store the new gain msg if we're done processing the last one
    last_gain_msg = msg;
    got_gain_msg = true;
  }

  //controller->setGainTrajectory(msg.times, msg.gains);
}

void paddle_orient_trajectory_callback(const fastwam::PaddleOrientationTrajectory &msg)
{
  printf("Got PaddleOrientationTrajectory message...\n");

  controller->setPaddleOrientationTrajectory(msg.times, msg.orientations);
}

void torque_trajectory_callback(const fastwam::TorqueTrajectory &msg)
{
  controller->setTorqueTrajectory(msg.joints, msg.times, msg.torques);

  //printf("got torque trajectory msg\n");
  //printf(" torques[0] = %.2f\n", msg.torques[0]);
}

void joint_trajectory_callback(const fastwam::JointTrajectory &msg)
{
  controller->setJointTrajectory(msg.joint_angles, msg.times);
}

void moveto_callback(const fastwam::MoveTo &msg)
{
  controller->setJointAngles(msg.joint_angles);
}


//------------------ SIGINT handler -----------------//

void sigint_handler(int signum)
{
  controller->setState(SwingController<7>::SHUTDOWN);
}


//----------------------- MAIN ----------------------//

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam)
{
  // init ROS
  ros::init(argc, argv, "arm");
  ros::NodeHandle nh;

  // advertise messages
  ros::Publisher swingdata_pub = nh.advertise<fastwam::SwingData>("/fastwam/swingdata", 1);

  // init WAM controller
  signal(SIGINT, sigint_handler);
  controller = new SwingController<7>(&pm, &swingdata_pub);
  systems::connect(pm.getWam7()->jpOutput, controller->input);
  wam.trackReferenceSignal(controller->output);
  
  // subscribe to messages
  ros::Subscriber sub_gain_traj = nh.subscribe("/fastwam/gain_trajectory", 1, gain_trajectory_callback);
  ros::Subscriber sub_paddle_orient_traj = nh.subscribe("/fastwam/paddle_orientation_trajectory", 1, paddle_orient_trajectory_callback);
  ros::Subscriber sub_torque_traj = nh.subscribe("/fastwam/torque_trajectory", 1, torque_trajectory_callback);
  ros::Subscriber sub_joint_traj = nh.subscribe("/fastwam/joint_trajectory", 1, joint_trajectory_callback);
  ros::Subscriber sub_moveto = nh.subscribe("/fastwam/moveto", 1, moveto_callback);


  // spin
  ros::Rate rate(250);
  while(nh.ok() && pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
    ros::spinOnce();
    rate.sleep();
  }

  // shutdown WAM
  pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

  return 0;
}
