#ifndef WAMIFC_H
#define WAMIFC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "btwam.h"

#ifndef BTSERIAL
#include "btserial.h"
#define BTSERIAL
#endif

#include <stdbool.h>
#include "log/log.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sys/mman.h>

double sq(double);

/**
 * WamInterface 
 * 
 * Sructure that represents the WAM
 * Clients can access and control wam state through the wamif_
 * functions.
 *
 * NOT THREAD-SAFE
 */
typedef struct {
	wam_struct *wam;            //wam data/controllers, populated by OpenWAM
	int wamnumber;              //in case there's more than one arm (first is 0)
	PORT handport;              //serial port for hand
	bool active;                //is the wam active?

	btstatecontrol *active_bts; //pointer to either wam->Jsc or wam->Csc, dependin g on mode
	vect_n *jdest;              //joint space destination
	vect_n *cdest;              //Cartesian space destination 
	vect_n *active_pos;         //current position: points to either wam->Jpos (joint) or wam->HMpos (Cartesian)
	vect_n *active_dest;        //current destination (either jdest or cdest)
	vect_n *active_trq;         //curent (outgoing) torque command: points to either wam->Jtrq (joint) or wam->HMft (Cartesian)
	via_trj_array **vta;        //points to either vt_j or vt_c (below)
	via_trj_array *vt_j;        //joint trajectory
	via_trj_array *vt_c;        //Cartesian trajectory

	btrt_thread_struct rt_thd;  //CAN bus thread
	btrt_thread_struct wam_thd; //main wam thread
	int startDone;              //flag to let the main thread know when initialization is complete

} WamInterface;


/************************ Server Creation/Destruction *************************/

//Initialization stuff that needs to be run before anything else
void wamif_init();

//Create and return a WamInterface
WamInterface *wamif_create(int wamnumber);

//Destory a WamInterface
void wamif_destroy(WamInterface *serv);

//Activate WAM
int wamif_activate(WamInterface *serv);

//Deactivate WAM
void wamif_shutdown(WamInterface *serv);


/******************************* Joint Control ********************************/

//Calibrate joints so that joint angles will be correct (optical encoders)
void wamif_calibrate_arm(WamInterface *serv);

//Command joint positions
//param jointangles - 7 joint angles
void wamif_move_joint(WamInterface *serv, double *jointangles);

//Get joint positions
//param jointangles to be filled in - 7 joint angles
void wamif_get_joint(WamInterface *serv, double *jointangles); 

//Get motor angles
//param motorangles to be filled in - radians away from zero pos for each motor
void wamif_get_motor_angles(WamInterface *serv, double *motorangles);

//Run a joint trajectory file
//param filename - joint trajectory file (barrett format)
void wamif_run_joint_trajectory_file(WamInterface *serv, char *filename);

//Send the arm home
void wamif_arm_home(WamInterface *serv);


/***************************** Cartesian Control ******************************/

//move to a Cartesian position/orientation via trajectory 
//pos is [x,y,z] in meters
//rot is a 3x3 rotation matrix as a 9-vector in row order
//origin is the point where joint 0,1,2 axes meet, frame as defined in manual
void wamif_move_cartesian(WamInterface *serv, double pos[3], double rot[9]);

//move to a Cartesian position/orientation via trajectory 
//pos is [x,y,z] in meters
//rot is Euler angles (XYZ)
//origin is the point where joint 0,1,2 axes meet, frame as defined in manual
//void wamif_move_cartesian_euler(WamInterface *serv, double pos[3], double euler[3]);

//get the current cartesian position/orientation (3x3 rotation matrix)
void wamif_get_cartesian(WamInterface *serv, double pos[3], double rot[9]);

//get the current Cartesian position/orientation (Euler angles)
//void wamif_get_cartesian_euler(WamInterface *serv, double pos[3], double euler[3]);

//run a Cartesian trajectory given in filename
void wamif_run_cartesian_trajectory_file(WamInterface *serv, char *filename);


/********************* Misc Controller Functionality **************************/

//set the torque limits (enforced in WAMcallback, set to -1 to disable)
void wamif_set_torque_limits(WamInterface *serv, double torquelimits[7]);

//Set gravity compensation on WAM
//param gc - gravity compensation fraction (0.0 - 1.0)
void wamif_set_gcomp(WamInterface *serv, double gc);

//Smooth change in gravity compensation on WAM
//param endpoint - final gravity compensation fraction (0.0 - 1.0)
//param ramptime - time taken to ramp value (seconds)
//param steps - number of discrete steps to use for ramp
void wamif_ramp_gcomp(WamInterface *serv, double endpoint, double ramptime, int steps);

//Stop all controllers (besides gravity comp)
void wamif_stop_controllers(WamInterface *serv);

//Check if the trajectory is finished 
int wamif_check_traj_done(WamInterface *serv);

//Wait until the trajectory is finished
int wamif_wait_until_traj_done(WamInterface *serv);



/******************************* Hand Control *********************************/


//Connect to Barrett Hand
void wamif_hand_connect(WamInterface *serv);

//Send a raw command to the hand
void wamif_hand_raw(WamInterface *serv, char* command, int waitforterm);

//Open or close the hand
void wamif_grasp(WamInterface *serv, bool close);

//Initialize the hand
void wamif_graspinitialize(WamInterface *serv);

//get feedback string in realtime mode (returns -1 if error, else number of bytes read)
//buffer contains the feedback string (not including the *)
//blocklen is the expected length of the feedback block (not including the *)
int wamif_get_realtime_feedback(WamInterface *serv, char *buffer, int blocklen);

//send one command in realtime to bend the fingers
//velocities and gains are signed bytes (values between -127 and +127)
int wamif_realtime_bend(WamInterface *serv, int vels[3], int gains[3]);

//start realtime hand loop (returns 1 if error, else 0)
//inputs the parameter string and the loop string 
//if parameterstring == 0 or loopstring == 0, use their defaults
int wamif_hand_start_realtime(WamInterface *serv, char *parameterstring, char *loopstring);

//terminate realtime mode and return to supervisory mode
void wamif_terminate_realtime(WamInterface *serv);


#ifdef __cplusplus
}
#endif

#endif
