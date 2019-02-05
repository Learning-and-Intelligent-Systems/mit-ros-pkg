#include <signal.h>

#include <ros/ros.h>
#include <fastwam/MoveTo.h>
#include <fastwam/SwingData.h>

#include "util.h"
#include <time.h>

#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
using namespace barrett;


// get a sine wave torque signal for one joint
double get_torque_signal(double t, double period, double amp)
{
  return amp * sin(2 * M_PI * t / period);
}




//-------------- Global variables ---------------//

ProductManager *PM = NULL;
systems::Wam<7>* wam7 = NULL;
units::JointPositions<7>::type* jp7;
units::JointPositions<7>::type* jp7home;
PID *pid = NULL;
//bool torque_joints[7];


fastwam::SwingData swingdata;
ros::Publisher swingdata_pub;

double joint1_angle = 0.0;
char *data_file_name;
double *log_data = NULL;
const int MAX_DATA = 40000; // n(?) seconds
int cnt = 0;
double previous_joints[7];
bool swing;
bool data_full;
double torque_amp[7];
double torque_period[7];
int swing_time = 0;
bool going_home = false;
bool shutdown = false;

void reset_waves() {
  for(int i = 0; i < 7; i++) {
    torque_period[i] = ((double) rand() / RAND_MAX) * 2 + 2; // rand() / RAND_MAX is in (0, 1)
    torque_amp[i] = ((double) rand() / RAND_MAX) * 5 + 1; // plus some value so values aren't too small, parameters can be changed
  }
}


void go_home()
{
  *jp7 = *jp7home;
  going_home = true;
  //wam7->moveTo(*jp7home, true);
}

void sigint_handler(int signum)
{
  go_home();

  /*
  wam7->idle();

  PM->getSafetyModule()->waitForMode(SafetyModule::IDLE);

  ros::shutdown();

  // open a data file for logging
  printf("Saving %s...", data_file_name);
  FILE *data_log_file = fopen(data_file_name, "w");

  fprintf(data_log_file, "data = [");
  for (int i = 0; i < cnt; i++)
    fprintf(data_log_file, "%f ", log_data[i]);
  fprintf(data_log_file, "];\n");

  fclose(data_log_file);
  printf("done\n");
  */
}


//---------------- WAM real-time torque control --------------//

template <size_t DOF>
class SimpleController :
  public systems::SingleIO<typename units::JointPositions<DOF>::type,
			   typename units::JointTorques<DOF>::type> {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
  SimpleController() :
    systems::SingleIO<jp_type, jt_type>("SimpleController")
  {
    for (int i = 0; i < 7; i++)
      goal[i] = (*jp7home)[i];

    goal_vel_limits[0] = 1;
    goal_vel_limits[1] = 1;
    goal_vel_limits[2] = 1;
    goal_vel_limits[3] = 1;
    goal_vel_limits[4] = 1;
    goal_vel_limits[5] = 1;
    goal_vel_limits[6] = 1;

    t = ros::Time::now().toSec();
  }

  virtual ~SimpleController() { this->mandatoryCleanUp(); }

protected:

  double t;
  double goal[7];
  double goal_vel_limits[7];

  virtual void operate()
  {
    jt_type jt;

    //(*jp7)[0] += .001;  // rotate joint 0 over time
    //if ((*jp7)[0] > M_PI/3)
    //  (*jp7)[0] -= 2 * M_PI/3;

    double joints_desired[7];
    for (int i = 0; i < 7; i++)
      joints_desired[i] = (*jp7)[i];

    double dt = ros::Time::now().toSec() - t;

    if (dt > .003)
      printf("dt = %.2f ms\n", dt*1000); //dbug

    t += dt;

    for (int i = 0; i < 7; i++) {
      if (joints_desired[i] > goal[i] + dt*goal_vel_limits[i])
	joints_desired[i] = goal[i] + dt*goal_vel_limits[i];
      else if (joints_desired[i] < goal[i] - dt*goal_vel_limits[i])
	joints_desired[i] = goal[i] - dt*goal_vel_limits[i];
      goal[i] = joints_desired[i];
    }

    pid->setGoal(joints_desired);

    //printf("New goal for joint 0: %.2f\n", joints_desired[0]);

    double joints[7];
    for (int i = 0; i < 7; i++)
      joints[i] = this->input.getValue()[i];

   if(cnt == 0) {
      for(int i = 0; i < 7; i++) 
	previous_joints[i] = joints[i];
   }

    double *torques = pid->update(joints);
    //printf("swing time:%d \n", swing_time); 
    //printf("swing amps %.2f, %.2f, %.2f\n ", torque_amp[1], torque_amp[2], torque_amp[3]);

    if (going_home) {
      bool do_shutdown = true;
      for (int i = 0; i < 7; i++) {
	if (fabs(joints[i] - (*jp7)[i]) > .005 || fabs(joints[i] - previous_joints[i]) > .005) {
	  printf("err[%d] = %.4f, djdt = %.4f\n", i, joints[i] - (*jp7)[i], joints[i] - previous_joints[i]);
	  do_shutdown = false;
	  break;
	}
      }
      if (do_shutdown) {
	//printf(" --- shutting down ---\n");
        //wam7->idle();
	//PM->getSafetyModule()->waitForMode(SafetyModule::IDLE);
	//ros::shutdown();
	shutdown = true;
      }
    }
    else if (swing) {  // && !data_full) {

      //printf("swinging\n");

      bool in_collision = projected_in_collision(joints, previous_joints, dt);

      if(in_collision == 1) {
	printf("*** in collision ***\n");
	swing_time = 0;
	swing = false;
	//(*jp7)[0] = 0; // set to horizontal position
	for(int i = 1; i < 7; i++)
	  (*jp7)[i] = 0;

	pid->reset_I(1);  // reset I term for swinging joint(s)
	
	swingdata_pub.publish(swingdata);
      }
      else {
	if(swing_time < 2000) {

	  //printf("getting torque signal\n");

	  // give torques wave signal, controlling joints 2 - 4 right now
	  //for(int i = 1; i < 4; i++) 
	  for(int i = 1; i < 2; i++) 
	    torques[i] = get_torque_signal(t, torque_period[i], torque_amp[i]);
	  swing_time++;
	}
	else {
	  swing_time = 0;
	  reset_waves();
	  printf("reset 1\n");
	  for(int i = 1; i < 4; i++) 
	    torques[i] = get_torque_signal(t, torque_period[i], torque_amp[i]);

	  swingdata_pub.publish(swingdata);
	}
      }
    }
    else {  //if (!data_full) {

      //printf("waiting for reset\n");

      bool reset = true;
      for (int i = 1; i < 7; i++) {
	if (fabs(joints[i]) > .1 || fabs(joints[i] - previous_joints[i]) > .001) {
	  //printf("joints[%d] = %.4f\n", i, joints[i]);
	  reset = false;
	  break;
	}
      }
      if (reset) {
	swing = true;
	swing_time = 0;
	reset_waves();
	swingdata.t.clear();
	swingdata.joint_angles.clear();
	swingdata.joint_torques.clear();
	printf("reset 2\n");
      }
    }


    // collect data when in swinging mode
    if (swing) {

      swingdata.t.push_back(t);
      for (int i = 0; i < 7; i++)
	swingdata.joint_angles.push_back(joints[i]);
      for (int i = 0; i < 7; i++)
	swingdata.joint_torques.push_back(torques[i]);

      /*
      if (cnt < 2*7*MAX_DATA) {
	for (int i = 0; i < 7; i++)
	  log_data[cnt++] = joints[i];
	for (int i = 0; i < 7; i++)
	  log_data[cnt++] = torques[i];
      }
      */
      /*
      else { // data array full, end swinging
	data_full = true;
	for(int i = 1; i < 7; i++)
	  (*jp7)[i] = 0;
      }
      */
    }

    if (cnt++ < 30)
      printf("This is a hack for real-time loop!\n");

    for (int i = 0; i < 7; i++)
      jt[i] = torques[i];


    //if (cnt % 500 == 0)
    //printf("jt =  %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", jt[0], jt[1], jt[2], jt[3], jt[4], jt[5], jt[6]);
    //double a = 0;
    //for (int i = 0; i < 10000; i++)
    //  a = a+i;

    for(int i = 0; i < 7; i++)
      previous_joints[i] = joints[i];

    this->outputValue->setData(&jt);
  }

private:
  DISALLOW_COPY_AND_ASSIGN(SimpleController);
};



//------------------- Callbacks and helper functions ------------------//


/*
void move_to_callback(const fastwam::MoveTo &msg)
{
  static int cnt=0;

  printf("Got MoveTo msg %d\n", cnt++);
    
  for (int j = 0; j < 7; j++)
    (*jp7)[j] = msg.joint_angles[j];
  //wam7->moveTo(*jp7, false); //, .5, 20.0);
}
*/

 /*
void torque_trajectory_callback(const fastwam::TorqueTrajectory &msg)
{
  static int cnt=0;

  printf("Got MoveTo msg %d\n", cnt++);
    
  for (int j = 0; j < 7; j++)
    (*jp7)[j] = msg.joint_angles[j];
  //wam7->moveTo(*jp7, false); //, .5, 20.0);
}
 */

//----------------------- MAIN --------------------//

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam)
{
  if (argc < 2) {
    printf("usage: %s <data_file>\n", argv[0]);
    exit(1);
  }
  data_file_name = argv[1];
  //joint1_angle = (M_PI/180.0)*atof(argv[2]);

  // init ROS
  ros::init(argc, argv, "wamtest");
  ros::NodeHandle nh;

  pid = new PID;

  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

  signal(SIGINT, sigint_handler);

  wam.gravityCompensate();
  pm.getSafetyModule()->setTorqueLimit(100);
  pm.getSafetyModule()->setVelocityLimit(0);

  PM = &pm;

  if (DOF == 4) {
    printf("ERROR: Only 7 dof arms supported.\n");
    pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
    exit(1);
  }
  else if (DOF == 7) {
    wam7 = pm.getWam7();
    jp7 = new units::JointPositions<7>::type;
    jp7home = new units::JointPositions<7>::type;    
    *jp7home = wam7->getJointPositions();
    //(*jp7)[0] = (*jp7home)[0];
    //(*jp7)[4] = 1.59;
    //(*jp7)[5] = .5;
    
    //(*jp7)[0] = (*jp7home)[0];
    //(*jp7)[0] = M_PI / 2; // set to vertical position
    (*jp7)[0] = 0; // set to horizontal position
    (*jp7)[1] = 0;
    (*jp7)[2] = 0;
    (*jp7)[3] = 0;
    for (int i = 4; i < 7; i++)
      (*jp7)[i] = (*jp7home)[i];
  }
  swing = false;
  data_full = false;
  srand ( time(NULL) );

  // create a data log array
  log_data = new double[2*DOF*MAX_DATA];

  SimpleController<DOF> controller;

  systems::connect(wam.jpOutput, controller.input);
  wam.trackReferenceSignal(controller.output);

  //ros::Subscriber sub_moveto = nh.subscribe("/fastwam/moveto", 1, move_to_callback);
  swingdata_pub = nh.advertise<fastwam::SwingData>("/fastwam/swingdata", 1);

  signal(SIGINT, sigint_handler);

  //while (nh.ok()) {
    // Spin until we get a MoveTo message
    //fastwam::MoveToConstPtr move_to_ptr = ros::topic::waitForMessage<fastwam::MoveTo>("/fastwam/moveto");
    //C.move_to_callback(*move_to_ptr);
  //}

  //ros::spin();

  ros::Rate pub_rate(250);
  while(nh.ok() && !shutdown && pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
    ros::spinOnce();
    pub_rate.sleep();
  }

  printf("Killed ros\n");

  //go_home();
  //wam.idle();
  pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
  ros::shutdown();

  return 0;
}
