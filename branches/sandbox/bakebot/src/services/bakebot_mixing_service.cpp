#include "ros/ros.h"
#include "bakebot/MixBowl.h"
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

void addCircularForcePoint(ee_cart_imped_control::EECartImpedGoal& trajectory_, double time, double theta, double xo, double yozo, double r, double angular_force_mag, double down_force_mag, bool is_mix, double rstiff, double rotstiff, double rpad, bool isangforce, bool isdownforce, double zy) {
    double R = r + rpad;
    double stiff = rstiff;
    double x = xo + R * cos(theta);
    double yz = yozo + R * sin(theta);

    if (is_mix) {
        double value = isdownforce ? down_force_mag : stiff;
        printf("x=%f\tYz=%f\tZy=%f\t", x, yz, zy);
        if (isdownforce) {
            printf("\tidf");
        }
        if (isangforce) {
            printf("\tiaf");
        }
        printf("\n");

        EECartImpedArm::addTrajectoryPoint(trajectory_,
                x, yz, zy, .7, 0, -.7, 0,
                stiff, stiff, value, rotstiff, rotstiff, rotstiff, 
                false, false, isdownforce, isangforce, isangforce, isangforce, 
                time);
    } else {
        // do xz plane instead of xy in order to scrape the bowl.
        EECartImpedArm::addTrajectoryPoint(trajectory_,
                x, 0.0, yz, 0.5, 0.5, 0.5, 0.5, // don't care about desired positions
                stiff, down_force_mag, stiff, 100, 100, 100, 
                false, true, false, false, false, false, 
                time);
    }
}

void addLinearForcePoint(ee_cart_imped_control::EECartImpedGoal& trajectory_, double time, double x, double y, double down_force_mag, double rstiff, double angstiff) {
    double fzdes_ = -1 * down_force_mag;
    double ftxdes_ = angstiff;
    double ftydes_ = angstiff;
    double ftzdes_ = angstiff;

    EECartImpedArm::addTrajectoryPoint(trajectory_,
                                       x, y, 0.0, 0.7, 0.0, -0.7, 0.0, // don't care about desired positions
                                       rstiff, rstiff, fzdes_, ftxdes_, ftydes_, ftzdes_, 
                                       false, false, true, false, false, false, 
                                       time);
}

void populateCircularMixingTrajectory(ee_cart_imped_control::EECartImpedGoal& mix, double x, double yz, double r, double mix_period, int num_laps, double angular_force_mag, double down_force_mag, bool is_mix, double rstiff, double rotstiff, double rpad, bool isangforce, bool isdownforce, double zy) {
    ROS_INFO("bakebot_mixing_service: populating circular mixing trajectory");
    int i = 0;
    int j = 0;
    double time = 0.0;
    double dt;
    dt = mix_period / NUM_POINTS;
    double theta = 0.0;
    double dtheta = 2 * 3.14159 / NUM_POINTS;
    printf("************ x = %f, yz = %f \n", x, yz);
    printf("r = %f \t\trpad = %f\n", r, rpad);
    if (isangforce) {
        printf("isangforce = True\n");
    }
    if (isdownforce) {
        printf("isdownforce = True \n");
    }
    printf("downforcemag = %f \trstiff= %f\trotstiff= %f\n", down_force_mag, rstiff, rotstiff);

    if (is_mix) {
        addCircularForcePoint(mix, (time += 2), theta, x, yz, r, angular_force_mag, down_force_mag, is_mix, rstiff, rotstiff, rpad, isangforce, isdownforce, zy);  // want to give it a chance to get to the rim
        // this generates a circular mix going from positive x axis around as theta increases
        for (i = 0; i < num_laps; i++) {
            for (j = 0; j < NUM_POINTS; j++) {
                addCircularForcePoint(mix, (time += dt), (theta += dtheta), x, yz, r, angular_force_mag, down_force_mag, is_mix, rstiff, rotstiff, rpad, isangforce, isdownforce, zy);
            }
        }
        for (i = 0; i < num_laps; i++) {
            for (j = 0; j < NUM_POINTS; j++) {
                addCircularForcePoint(mix, (time += dt), (theta -= dtheta), x, yz, r, angular_force_mag, down_force_mag, is_mix, rstiff, rotstiff, rpad, isangforce, isdownforce, zy);
            }
        }
        for (i = 0; i < num_laps; i++) {
            for (j = 0; j < NUM_POINTS; j++) {
                addCircularForcePoint(mix, (time += dt), (theta += dtheta), x, yz, r, angular_force_mag, down_force_mag, is_mix, rstiff, rotstiff, rpad, isangforce, isdownforce, zy);
            }
        }
    } else {
        dt = dt * 2;
        // this generates a circular mix going clockwise from positive z to negative z, then counterclockwise from pos to neg
        for (i = 0; i < num_laps; i++) {
            addCircularForcePoint(mix, (time += (2)), 4*3.14159/4.0, x, yz, r, angular_force_mag, down_force_mag, is_mix, rstiff, rotstiff, rpad, isangforce, isdownforce, zy);
            for (theta =4*3.14159/4.0; theta < 3 * 3.14159/2.0; theta+=dtheta) {
                addCircularForcePoint(mix, (time += dt), theta, x, yz, r, angular_force_mag, down_force_mag, is_mix, rstiff, rotstiff, rpad, isangforce, isdownforce, zy);
            }
            // TODO: will want to adjust this 5 to something if it shoots up too fast
            addCircularForcePoint(mix, (time += (2)), 0*3.14159/4.0, x, yz, r, angular_force_mag, down_force_mag, is_mix, rstiff, rotstiff, rpad, isangforce, isdownforce, zy);
            theta = 0;
            addCircularForcePoint(mix, (time += (2)), 0*3.14159/4.0, x, yz, r, angular_force_mag, down_force_mag, is_mix, rstiff, rotstiff, rpad, isangforce, isdownforce, zy);
            printf("theta = %f\n",theta);
            for (theta = 0*3.14159/4.0; theta > -3.14159/2.0; theta-=dtheta) {
                printf("theta = %f\n",theta);
                addCircularForcePoint(mix, (time += dt), theta, x, yz, r, angular_force_mag, down_force_mag, is_mix, rstiff, rotstiff, rpad, isangforce, isdownforce, zy);
            }
        }

    }
    zeroPoint(mix, (time += 1));
}

void populateLinearMixingTrajectory(ee_cart_imped_control::EECartImpedGoal& mix, double mix_period, int num_laps, double linear_force_mag, double down_force_mag, double x, double y, double z, double h, double r, double rstiff, double angstiff) {
    ROS_INFO("bakebot_mixing_service: populating linear mixing trajectory");
    double time = 0.0;
    double ox = x;
    double oy = y;
    double R = r + 0.05;
    double s = R * sqrt(2) / 2.0;
    double dt = mix_period;

    int i = 0;
    for (i = 0; i < num_laps; i++) {
        x = ox + s;
        y = oy + s;
        printf("going to force point (x, y) = (%f, %f)\n", x, y);
        addLinearForcePoint(mix, (time += dt), x, y, down_force_mag, rstiff, angstiff);

        x = ox;
        y = oy;
        printf("going to force point (x, y) = (%f, %f)\n", x, y);
        addLinearForcePoint(mix, (time += dt), x, y, 2*down_force_mag, rstiff, angstiff);

        x = ox-s;
        y = oy-s;
        printf("going to force point (x, y) = (%f, %f)\n", x, y);
        addLinearForcePoint(mix, (time += dt), x, y, down_force_mag, rstiff, angstiff);

        x = ox-s;
        y = oy+s;
        printf("going to force point (x, y) = (%f, %f)\n", x, y);
        addLinearForcePoint(mix, (time += dt), x, y, down_force_mag, rstiff, angstiff);

        x = ox;
        y = oy;
        printf("going to force point (x, y) = (%f, %f)\n", x, y);
        addLinearForcePoint(mix, (time += dt), x, y, 2*down_force_mag, rstiff, angstiff);

        x = ox+s;
        y = oy-s;
        printf("going to force point (x, y) = (%f, %f)\n", x, y);
        addLinearForcePoint(mix, (time += dt), x, y, down_force_mag, rstiff, angstiff);

        x = ox;
        y = oy;
        printf("going to force point (x, y) = (%f, %f)\n", x, y);
        addLinearForcePoint(mix, (time += dt), x, y, 2*down_force_mag, rstiff, angstiff);

        x = ox+s;
        y = oy;
        printf("going to force point (x, y) = (%f, %f)\n", x, y);
        addLinearForcePoint(mix, (time += dt), x, y, down_force_mag, rstiff, angstiff);

        x = ox;
        y = oy;
        printf("going to force point (x, y) = (%f, %f)\n", x, y);
        addLinearForcePoint(mix, (time += dt), x, y, 2*down_force_mag, rstiff, angstiff);

        x = ox-s;
        y = oy;
        printf("going to force point (x, y) = (%f, %f)\n", x, y);
        addLinearForcePoint(mix, (time += dt), x, y, down_force_mag, rstiff, angstiff);

        x = ox;
        y = oy;
        printf("going to force point (x, y) = (%f, %f)\n", x, y);
        addLinearForcePoint(mix, (time += dt), x, y, 2*down_force_mag, rstiff, angstiff);
    }
    // zero everything out
    zeroPoint(mix, (time += 1));
}

void populateLowerSpoonTrajectory(ee_cart_imped_control::EECartImpedGoal& mix, double x, double y, double z, bool is_mix, double rstiff, double rotstiff, double num_laps) {
	// if num_laps = 0, do the 12/6 orientation for the plunge
	// otherwise do the 3/9 orientation 
    // PLUNGE plunge
    ROS_INFO("bakebot_mixing_service: populating lower spoon trajectory");
	if (num_laps == 0) {
		ROS_INFO("num_laps = req.whisk_laps is 0, doing the 12/6 orientation for the plunge");
	}
    int dt = 1;
    int max_time = 10;
    int count = 0;
    double fzdes_ = 0;
	double qx = (num_laps == 0) ? -.7 : -.5;
	double qy = (num_laps == 0) ?   0 :  .5;
	double qz = (num_laps == 0) ?  .7 :  .5;
	double qw = (num_laps == 0) ?   0 :  .5;

    for (int t = dt; t < max_time; t+=dt) {
        fzdes_ += MAX_DOWN_FORCE / 5.0;

        if (count < 2) {
            fzdes_ = 0;
        }
        if (fzdes_ > MAX_DOWN_FORCE) {
            fzdes_ = MAX_DOWN_FORCE;
        }
        if (is_mix) {
            printf("adding down force point fz = %f ||  x = %f\t y = %f || rstiff=%f\n", -1*fzdes_, x, y, rotstiff);
            EECartImpedArm::addTrajectoryPoint(mix,
                                            x, y, 0.0, qx, qy, qz, qw, 
                                            rstiff, rstiff, -1*fzdes_, rotstiff, rotstiff, rotstiff,
                                            false, false, true, false, false, false, 
                                            t);
        } else {
            printf("adding down (y since scrape) force point fy = %f\n", fzdes_);
            printf("x=%f, z=%f\n", x, z);
            EECartImpedArm::addTrajectoryPoint(mix,
                                            //x, 0.0, z, 0.5, 0.5, 0.5, 0.5, // don't care about desired positions
                                            x, 0.0, z, 0.64, 0.32, 0.57, 0.4, // don't care about desired positions
                                            1000.0, fzdes_, 1000.0, 100.0, 100.0, 100.0, 
                                         false, true, false, false, false, false, 
                                            t);  // TODO: need to change this to have position stiffness around that point in xz
        }
        count++;
    }
}

void populateRaiseSpoonTrajectory(ee_cart_imped_control::EECartImpedGoal& mix, double bx, double by, double bz, double h, bool is_mix) {
    // deplunge DEPLUNGE
    ROS_INFO("bakebot_mixing_service: populating raise spoon trajectory");
    if (is_mix) {
    EECartImpedArm::addTrajectoryPoint(mix,
                                       bx, by, bz + h, -0.5, 0.5, 0.5, 0.5, // don't care about desired orientations
                                       1000, 1000, 1000, 0.0, 0.0,0.0, 
                                       false, false, false, false, false, false, 
                                       5);  
    } else {
    EECartImpedArm::addTrajectoryPoint(mix,
                                       bx, by-h, bz, 0.0, 0.0, 0.0, 0.0, // don't care about desired orientations
                                       1000, 1000, 1000, 0.0, 0.0, 0.0, 
                                       false, false, false, true, true, true, 
                                       5);  
    }
    zeroPoint(mix, 8); 
}

bool mix(bakebot::MixBowl::Request &req, bakebot::MixBowl::Response &res) {
    ROS_INFO("bakebot_mixing_service: received a new mixing request");
    bool useRightArm = (req.right_arm == 1);
    //EECartImpedArm* armp;
    EECartImpedArm arm("r_arm_cart_imped_controller");
    if (!useRightArm) {
        ROS_INFO("using the left arm is unsupported.");
        return false;
    } else {
        ROS_INFO("using the right arm");
    }
    //if (useRightArm) {
        //ROS_INFO("using the right arm");
        //armp = new EECartImpedArm("r_arm_cart_imped_controller");
    //} else {
        ////ROS_INFO("using the left arm is unsupported! ***************");
        //ROS_INFO("using the left arm");
        //armp = new EECartImpedArm("l_arm_cart_imped_controller");
        ////return false;
    //}
    //EECartImpedArm arm = *armp;
    //delete armp;
    //EECartImpedArm rarm("r_arm_cart_imped_controller");  //TODO: I need to make this within the conditional
        //EECartImpedArm larm("l_arm_cart_imped_controller");

    bool is_mix = (req.is_mix == 1);
    bool lower_spoon = (req.lower_spoon == 1);
    bool do_circular_mix = (req.do_circular_mix == 1);
    bool do_linear_mix = (req.do_linear_mix == 1);
    bool do_whisk_mix = (req.do_whisk_mix == 1);
    bool raise_spoon = (req.raise_spoon == 1);

    res.status = 0;

    if (is_mix) {
        ROS_INFO("bakebot mixing service: initializing for MIXING");
    } else {
        ROS_INFO("bakebot mixing service: initializing for SCRAPING");
    }

    if (lower_spoon) {
        ROS_INFO("bakebot_mixing_service: lowering spoon");


        ee_cart_imped_control::EECartImpedGoal mix_;

        double x = req.bowl_x_pos_torso_ll_frame;
        double y = req.bowl_y_pos_torso_ll_frame;
        double z = req.bowl_z_pos_torso_ll_frame;
        double rstiff = req.radial_stiffness;
        double rotstiff = req.rotational_stiffness;
        int num_laps = req.whisk_laps;
        populateLowerSpoonTrajectory(mix_, x, y, z, is_mix, rstiff, rotstiff, num_laps);

        if (useRightArm) {
            ROS_INFO("bakebot_mixing_service: starting down force trajectory on RIGHT arm");
            arm.startTrajectory(mix_);
        } else {
            ROS_INFO("bakebot_mixing_service: starting down force trajectory on LEFT arm");
            arm.startTrajectory(mix_);
        }

        ROS_INFO("bakebot_mixing_service: done lowering spoon");
        res.status = res.status + 0;
    }

    if (do_circular_mix) {
        ROS_INFO("bakebot_mixing_service: doing circular mix ");

        ee_cart_imped_control::EECartImpedGoal mix_;

        double mix_period = req.circular_period_sec;
        if (mix_period <= 0) {
            ROS_WARN("circular mixing period must be positive nonzero seconds");
            res.status = -10;
            return false;
        }

        int num_laps = req.circular_laps;
        if (num_laps < 1) {
            ROS_WARN("circular number of laps must be at least 1");
            res.status = -11;
            return false;
        }

        double angular_force_mag = req.angular_force_mag;
        if (angular_force_mag <= 0) {
            ROS_WARN("angular force magnitude must be positive nonzero");
            res.status = -12;
            return false;
        }

        double down_force_mag = req.down_force_mag;
        if (down_force_mag <= 0) {
            ROS_WARN("down force magnitude must be positive nonzero");
            res.status = -13;
            return false;
        }

        double x = req.bowl_x_pos_torso_ll_frame;
        double y = req.bowl_y_pos_torso_ll_frame;
        double zy = req.bowl_z_pos_torso_ll_frame;
        double r = req.bowl_radius;
        if (!is_mix) {
            y = req.bowl_z_pos_torso_ll_frame;
            zy = req.bowl_y_pos_torso_ll_frame;
        }
        double rstiff = req.radial_stiffness;
        double rotstiff = req.rotational_stiffness;
        double rpad = req.radial_pad;
        bool isangforce = (req.is_angular_force == 1);
        bool isdownforce = (req.is_down_force == 1);
        populateCircularMixingTrajectory(mix_, x, y, r, mix_period, num_laps, angular_force_mag, down_force_mag, is_mix, rstiff, rotstiff, rpad, isangforce, isdownforce, zy);

        if (useRightArm) {
            ROS_INFO("bakebot_mixing_service: starting circular mixing trajectory on RIGHT arm");
            arm.startTrajectory(mix_);
        } else {
            ROS_INFO("bakebot_mixing_service: starting circular mixing trajectory on LEFT arm");
            arm.startTrajectory(mix_);
        }

        ROS_INFO("bakebot_mixing_service: done with circular mix");
        res.status = res.status + 0;
    }

    if (do_linear_mix) {
        ROS_INFO("bakebot_mixing_service: doing linear mix ");
        ee_cart_imped_control::EECartImpedGoal mix_;

        if (!is_mix) {
            ROS_WARN("linear mix is unsupported for a scraping request");
            return false;
        }

        double mix_period = req.linear_period_sec;
        if (mix_period <= 0) {
            ROS_WARN("linear mixing period must be positive nonzero seconds");
            res.status = -10;
            return false;
        }

        int num_laps = req.linear_laps;
        if (num_laps < 1) {
            ROS_WARN("linear number of laps must be at least 1");
            res.status = -11;
            return false;
        }

        double linear_force_mag = req.linear_force_mag;
        if (linear_force_mag <= 0) {
            ROS_WARN("linear force magnitude must be positive nonzero");
            res.status = -12;
            return false;
        }

        double down_force_mag = req.down_force_mag;
        if (down_force_mag <= 0) {
            ROS_WARN("down force magnitude must be positive nonzero");
            res.status = -13;
            return false;
        }

        double x = req.bowl_x_pos_torso_ll_frame;
        double y = req.bowl_y_pos_torso_ll_frame;
        double z = req.bowl_y_pos_torso_ll_frame;
        double h = req.pre_mix_height_above_bowl_z_pos;
        double r = req.bowl_radius;
        double rstiff = req.radial_stiffness;
        double angstiff = req.rotational_stiffness;
        populateLinearMixingTrajectory(mix_, mix_period, num_laps, linear_force_mag, down_force_mag, x, y, z, h, r, rstiff, angstiff);

        if (useRightArm) {
            ROS_INFO("bakebot_mixing_service: starting circular mixing trajectory on RIGHT arm");
            arm.startTrajectory(mix_);
        } else {
            ROS_INFO("bakebot_mixing_service: starting circular mixing trajectory on LEFT arm");
            arm.startTrajectory(mix_);
            ROS_INFO("FAIL FAIL FAIL FAIL");
        }

        ROS_INFO("bakebot_mixing_service: done with linear mix");
        res.status = res.status + 0;
    }

    if (do_whisk_mix) {
        ROS_INFO("bakebot_mixing_service: doing whisk mix ");
        ROS_INFO("this method is not yet implemented");
        if (!is_mix) {
            ROS_WARN("whisk mix is unsupported for a scraping request");
            return false;
        }
        ROS_INFO("bakebot_mixing_service: done with whisk mix");
        res.status = res.status + 0;
    }

    if (raise_spoon) {
        ROS_INFO("bakebot_mixing_service: raising spoon");

        ee_cart_imped_control::EECartImpedGoal mix_;
        double x = req.bowl_x_pos_torso_ll_frame;
        double y = req.bowl_y_pos_torso_ll_frame;
        double z = req.bowl_y_pos_torso_ll_frame;
        double h = req.pre_mix_height_above_bowl_z_pos;
        populateRaiseSpoonTrajectory(mix_, x, y, z, h, is_mix);
        if (useRightArm) {
            ROS_INFO("bakebot_mixing_service: starting trajectory on RIGHT arm");
            arm.startTrajectory(mix_);
        } else {
            ROS_INFO("bakebot_mixing_service: starting trajectory on LEFT arm");
            arm.startTrajectory(mix_);
        }

        ROS_INFO("bakebot_mixing_service: done raising spoon");
        res.status = res.status + 0;
    }

    ROS_INFO("bakebot_mixing_service: completed mixing request");
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bakebot_mixing_service_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("bakebot_mixing_service", mix);
    ROS_INFO("The bakebot mixing/scraping service is ready to mix.");
    ros::spin();
    return 0;
}
