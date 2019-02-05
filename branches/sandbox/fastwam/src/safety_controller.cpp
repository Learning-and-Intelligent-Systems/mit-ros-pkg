#include <signal.h>

#include <ros/ros.h>
#include <math.h>
#include <fastwam/MoveTo.h>
#include "util.h"

#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
using namespace barrett;

#include <Eigen/Core>

// import most common Eigen types 
USING_PART_OF_NAMESPACE_EIGEN



//-------------- Global variables ---------------//

ProductManager *PM = NULL;
systems::Wam<7>* wam7 = NULL;
units::JointPositions<7>::type* jp7;
units::JointPositions<7>::type* jp7home;
//PID *pid = NULL;
int cnt = 0;












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

    /*
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
    */

    //printf("New goal for joint 0: %.2f\n", joints_desired[0]);

    double joints[7];
    for (int i = 0; i < 7; i++)
      joints[i] = this->input.getValue()[i];



    // TODO: compute forward kinematics, collision checking

    bool collision = in_collision(joints);

    if (cnt++ % 500 == 0) {
      if (collision)
	printf(" ***************  COLLISION  ****************\n\n");
      else
	printf("collision = %d\n", collision);

      printf("joint angles = %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], joints[6]);
    }


    //double *torques = pid->update(joints);

    for (int i = 0; i < 7; i++)
      jt[i] = 0; //torques[i];

    //if (cnt++ % 500 == 0)
    //  printf("jt =  %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", jt[0], jt[1], jt[2], jt[3], jt[4], jt[5], jt[6]);
    
    this->outputValue->setData(&jt);
  }

private:
  DISALLOW_COPY_AND_ASSIGN(SimpleController);
};






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
  ros::init(argc, argv, "safety_controller");
  ros::NodeHandle nh;

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

  //ros::Subscriber sub_moveto = nh.subscribe("/fastwam/moveto", 1, move_to_callback);

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
