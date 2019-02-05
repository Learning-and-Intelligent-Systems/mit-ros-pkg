#include <signal.h>

#include <ros/ros.h>
#include <fastwam/MoveTo.h>


#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
using namespace barrett;



//---------------- 2-DOF Safety Controller ----------------//

#define METERS_PER_INCH .0254
const double ARM_LINK1 = 23*METERS_PER_INCH;
const double ARM_LINK2 = 19*METERS_PER_INCH;
const double MIN_WALL_DIST = .2;

bool check_safety_2d(double j1_angle, double j3_angle, double j1_velocity, double j3_velocity)
{
  double r1 = ARM_LINK1;
  double r2 = ARM_LINK2;
  double a1 = j1_angle;
  double a2 = j3_angle;
  double v1 = j1_velocity;
  double v2 = j3_velocity;

  for (double dt = 0; dt < .2001; dt += .05)
    if (r1*cos(a1+dt*v1) + r2*cos(a1+a2+dt*(v1+v2)) < MIN_WALL_DIST)
      return false;

  return true;
}




//---------------- PID controller ----------------//

//const double pid_kp[7] = {2500, 2500, 600, 500, 50, 50, 8};
//const double pid_ki[7] = {5,    5,    2,  0.5,  0.5,  0.5,  0.1};
//const double pid_kd[7] = {20,   20,    5,    2,  0.5,  0.5, 0.05};
const double pid_kp[7] = {1500, 2500, 500, 500, 50, 50, 8};
const double pid_ki[7] = {5,    5,    2,  0.5,  0.5,  0.5,  0.1};
const double pid_kd[7] = {20,   40,    5,    2,  0.5,  0.5, 0.05};
//const double pid_torque_limits[7] = {40, 40, 15, 15, 5, 5, 5};
const double pid_torque_limits[7] = {40, 40, 15, 15, 1, 1, 1};

class PID {
public:
  PID()
  {
    t = ros::Time::now().toSec();
    for (int i = 0; i < 7; i++)
      P[i] = I[i] = torques[i] = 0;
  }

  double *update(double *joints)
  {
    double t2 = ros::Time::now().toSec();
    double dt = t2-t;
    for (int i = 0; i < 7; i++) {
      double err = goal[i] - joints[i];
      double D = (err - P[i])/dt;
      P[i] = err;
      I[i] += dt*err;

      double new_torque = pid_kp[i]*P[i] + pid_ki[i]*I[i] + pid_kd[i]*D;

      if (new_torque > pid_torque_limits[i])
	new_torque = pid_torque_limits[i];
      else if (new_torque < -pid_torque_limits[i])
	new_torque = -pid_torque_limits[i];

      torques[i] = new_torque;

      /*
      double max_diff_torque = dt*pid_diff_torque_limits[i];
      if (new_torque > torques[i] + max_diff_torque)
	new_torque = torques[i] + max_diff_torque;
      else if (new_torque < torques[i] - max_diff_torque)
	new_torque = torques[i] - max_diff_torque;

      torques[i] = new_torque;
      */
    }
    t = t2;

    return torques;
  }

  void setGoal(double *new_goal)
  {
    for (int i = 0; i < 7; i++) {
      goal[i] = new_goal[i];
      //P[i] = I[i] = 0;
    }
  }

protected:
  double t;  // time
  double P[7], I[7];
  double torques[7];
  double goal[7];
};


//-------------- Global variables ---------------//

ProductManager *PM = NULL;
systems::Wam<7>* wam7 = NULL;
units::JointPositions<7>::type* jp7;
units::JointPositions<7>::type* jp7home;
PID *pid = NULL;




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

    static int cnt = 0;

    double joints_desired[7];
    for (int i = 0; i < 7; i++)
      joints_desired[i] = (*jp7)[i];
    //joints_desired[0] = sin(cnt / 5000.0);

    double dt = ros::Time::now().toSec() - t;
    for (int i = 0; i < 7; i++) {
      if (joints_desired[i] > goal[i] + dt*goal_vel_limits[i])
	joints_desired[i] = goal[i] + dt*goal_vel_limits[i];
      else if (joints_desired[i] < goal[i] - dt*goal_vel_limits[i])
	joints_desired[i] = goal[i] - dt*goal_vel_limits[i];
      goal[i] = joints_desired[i];
    }
    t += dt;

    pid->setGoal(joints_desired);

    //printf("New goal for joint 0: %.2f\n", joints_desired[0]);

    double joints[7];
    for (int i = 0; i < 7; i++)
      joints[i] = this->input.getValue()[i];

    double *torques = pid->update(joints);

    for (int i = 0; i < 7; i++)
      jt[i] = torques[i];

    if (cnt++ % 500 == 0)
      printf("jt =  %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", jt[0], jt[1], jt[2], jt[3], jt[4], jt[5], jt[6]);
    
    this->outputValue->setData(&jt);
  }

private:
  DISALLOW_COPY_AND_ASSIGN(SimpleController);
};



//------------------- Callbacks and helper functions ------------------//


void move_to_callback(const fastwam::MoveTo &msg)
{
  static int cnt=0;

  printf("Got MoveTo msg %d\n", cnt++);
    
  for (int j = 0; j < 7; j++)
    (*jp7)[j] = msg.joint_angles[j];
  //wam7->moveTo(*jp7, false); //, .5, 20.0);
}

void go_home()
{
  *jp7 = *jp7home;
  wam7->moveTo(*jp7home, true);
}

void sigint_handler(int signum)
{
  go_home();

  wam7->idle();

  PM->getSafetyModule()->waitForMode(SafetyModule::IDLE);

  ros::shutdown();
}


//----------------------- MAIN --------------------//

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam)
{
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
    (*jp7)[0] = (*jp7home)[0];
    (*jp7)[4] = 1.59;
    (*jp7)[5] = .5;
  }

  SimpleController<DOF> controller;

  systems::connect(wam.jpOutput, controller.input);
  wam.trackReferenceSignal(controller.output);

  ros::Subscriber sub_moveto = nh.subscribe("/fastwam/moveto", 1, move_to_callback);

  signal(SIGINT, sigint_handler);

  //while (nh.ok()) {
    // Spin until we get a MoveTo message
    //fastwam::MoveToConstPtr move_to_ptr = ros::topic::waitForMessage<fastwam::MoveTo>("/fastwam/moveto");
    //C.move_to_callback(*move_to_ptr);
  //}

  ros::spin();

  //while(nh.ok()) {
  //  ros::spinOnce();
  //sleep(.001);
  //}

  go_home();
  wam.idle();
  pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

  return 0;
}
