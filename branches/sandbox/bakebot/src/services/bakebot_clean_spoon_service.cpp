#include "bakebot/CleanSpoon.h"
#include <ee_cart_imped_action/ee_cart_imped_arm.hh>

#define NUM_POINTS 10
#define MAX_DOWN_FORCE 15.0

void zeroPoint(ee_cart_imped_control::EECartImpedGoal& trajectory_, double time ) {
    EECartImpedArm::addTrajectoryPoint(trajectory_,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // don't care about desired positions
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                       true, true, true, true, true, true, 
                                       time);
}

bool clean_spoon(bakebot::CleanSpoon::Request &req, bakebot::CleanSpoon::Response &res) {
    ROS_INFO("bakebot_clean_spoon_service: received a new clean spoon request");
    bool useRightArm = (req.right_arm == 1);
    EECartImpedArm arm("r_arm_cart_imped_controller");
    if (!useRightArm) {
        ROS_INFO("using the left arm is unsupported.");
        return false;
    } else {
        ROS_INFO("using the right arm");
    }

    double x0 = req.x0_torso_ll_frame;
    double y0 = req.y0_torso_ll_frame;
    double z0  = req.z0_torso_ll_frame;
    double qx0 = req.qx0_torso_ll_frame;
    double qy0 = req.qy0_torso_ll_frame;
    double qz0 = req.qz0_torso_ll_frame;
    double qw0 = req.qw0_torso_ll_frame;
    double kx0 = req.kx0_torso_ll_frame;
    double ky0 = req.ky0_torso_ll_frame;
    double kz0 = req.kz0_torso_ll_frame;
    double kth0 = req.kth0_torso_ll_frame;
    bool is_force_x0 = req.is_force_x0;
    bool is_force_y0 = req.is_force_y0;
    bool is_force_z0 = req.is_force_z0;
    bool is_force_th0 = req.is_force_th0;

    double x1 = req.x1_torso_ll_frame;
    double y1 = req.y1_torso_ll_frame;
    double z1 = req.z1_torso_ll_frame;
    double qx1 = req.qx1_torso_ll_frame;
    double qy1 = req.qy1_torso_ll_frame;
    double qz1 = req.qz1_torso_ll_frame;
    double qw1 = req.qw1_torso_ll_frame;
    double kx1 = req.kx1_torso_ll_frame;
    double ky1 = req.ky1_torso_ll_frame;
    double kz1 = req.kz1_torso_ll_frame;
    double kth1 = req.kth1_torso_ll_frame;
    bool is_force_x1 = req.is_force_x1;
    bool is_force_y1 = req.is_force_y1;
    bool is_force_z1 = req.is_force_z1;
    bool is_force_th1 = req.is_force_th1;
    
    double time = req.time;

    ee_cart_imped_control::EECartImpedGoal clean_;
    res.status = 0;
    EECartImpedArm::addTrajectoryPoint(clean_,
                                       x0, y0, z0, qx0, qy0, qz0, qw0,
                                       kx0, ky0, kz0, kth0, kth0, kth0, 
                                       is_force_x0, is_force_y0, is_force_z0, is_force_th0, is_force_th0, is_force_th0,
                                       0);
    printf("position 0: %f %f %f %f %f %f %f\n", x0, y0, z0, qx0, qy0, qz0, qw0);
    printf("stiff 0: %f %f %f %f\n", kx0, ky0, kz0, kth0);
    EECartImpedArm::addTrajectoryPoint(clean_,
                                       x1, y1, z1, qx1, qy1, qz1, qw1,
                                       kx1, ky1, kz1, kth1, kth1, kth1, 
                                       is_force_x1, is_force_y1, is_force_z1, is_force_th1, is_force_th1, is_force_th1,
                                       time);
    printf("position 1: %f %f %f %f %f %f %f\n", x1, y1, z1, qx1, qy1, qz1, qw1);
    printf("stiff 1: %f %f %f %f\n", kx1, ky1, kz1, kth1);
    printf("time: %f\n", time);
    // zeroPoint(clean_, (time+0.5));

    arm.startTrajectory(clean_);
    ROS_INFO("bakebot_clean_spoon_service: completed clean_spoon_service request");
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bakebot_clean_spoon_service_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("bakebot_clean_spoon_service", clean_spoon);
    ROS_INFO("The bakebot clean spoon service is ready to clean spoons.");
    ros::spin();
    return 0;
}
