#ifndef PINGPONG_KINEMATICS_H
#define PINGPONG_KINEMATICS_H


double *joints_to_paddle(double *joint_angles);
double **paddle_jacobian(double *joint_angles);
double *paddle_to_joints_displacement(double *joint_angles, double *dp);
double *paddle_to_joints_displacement_weighted(double *joint_angles, double *dp, double *w);
double *paddle_to_joints(double *paddle, double *ja0);



#endif
