//Wrapper for Barrett's WAM client and serial commands for the BarrettHand
//currently works for btclient svn versions 79 and 512 and probably the ones in between
//written by Kaijen Hsiao (questions? email kaijenhsiao@gmail.com)
//see WAMinterfacelib/testprograms/WAMtest.c for example usage

//If using Xenomai, functions must be run from within a Xenomai thread to avoid lock failures!!  (it stops being a real-time thread as soon as the first I/O (printf, getchar, etc.) command is run, but it still needs to be a Xenomai thread, not a normal pthread, so that we can use the real-time locks.)
//first run wamif_init, then start a real-time thread to call other functions:
//btrt_thread_create(&main_thd, "Main", 20, (void*)run_main, NULL);
//where run_main is a function that calls wamif functions


#include "wamif.h"

#define MODULE_NAME "[SERVER  ]"
#define Ts (0.002)

//debug function to test the effect of linking
double sq(double num){
	return num*num;
}

double jointlowerlimits[7] = {-2.68, -1.99, -2.86, -.9, -4.79, -1.62, -3.03};
double jointupperlimits[7] = {2.68, 1.99, 2.86, 3.13, 1.26, 1.57, 3.03};

//torquelimits to use in WAMcallback (Nm)
//double torquelimits[7] = {6.31, 6.31, 6.31, 6.31, 1.826, 1.826, 0.613};
//double torquelimits[7] = {7, 7, 7, 7, 2, 2, 1};
double torquelimits[7] = {7.75,7.75,7.75,7.75,2.5,2.5,2}; 

//if redefineangles == 1, DefineWAMpos is run on newdefinedangles
//the next time WAMcallback is run
bool redefineangles = 0;
double newdefinedangles[7];  


//used to insert code directly into the control loop in btwam.c
int WAMcallback(struct btwam_struct *wam) {
	int i;

	//if requested, redefine the current WAM angles using defineWAMpos
	if(redefineangles){
		int i;
		vect_n *conf = new_vn(7);
		for(i=0; i<7; i++){
			setval_vn(conf, i, newdefinedangles[i]);
		}
		DefineWAMpos(wam, conf);
		redefineangles = 0;
	}
	
	//limit the outgoing torque commands to avoid torque faulting
	for(i=0; i<7; i++){
		if(torquelimits[i] >= 0){
			double torque = getval_vn(wam->Jtrq, i);
			if(abs(torque) > torquelimits[i]){
				if(torque>0){
					setval_vn(wam->Jtrq, i, torquelimits[i]);
				}
				else{
					setval_vn(wam->Jtrq, i, -torquelimits[i]);
				}
			}
		}
	}
}


//Initialization stuff that needs to be run before anything else
void wamif_init(){
#ifdef RTAI   
	//Allow hard real time process scheduling for non-root users 
	rt_allow_nonroot_hrt();
#else
	//Needs to be run before creating a real-time thread in Xenomai
	mlockall(MCL_CURRENT | MCL_FUTURE);
#endif
}


//Create a new WamInterface struct and initialize it
WamInterface *wamif_create(int wamnumber) {
	WamInterface *s = (WamInterface*)malloc(sizeof(WamInterface));
	s->active = false;
	s->wamnumber = wamnumber;
	s->startDone = 0;
	return s;
}


//Shut down and free the WamInterface struct
void wamif_destroy(WamInterface *serv) {
	if (serv->active)
		wamif_shutdown(serv);
	free(serv);
}


//Shut down the arm
void wamif_shutdown(WamInterface *serv) {
	if (serv->active) {
		CloseSystem();
		serv->rt_thd.done = 1;
		serv->wam_thd.done = 1;
		serv->active = false;
	}
}


//start up the WAM and the CAN bus (must be done in a realtime thread)
void StartupThread(void *thd){
	printf("starting StartupThread\n");
	WamInterface *serv = (WamInterface *)(((btrt_thread_struct*)thd)->data);

	//Probe and initialize the robot actuators 
	int err = InitializeSystem();
	if(err) {
		printf("InitializeSystem returned err = %d", err);		
		exit(1);
	}

	//initialize and get a handle to the robot
	serv->wam = OpenWAM("wam.conf",serv->wamnumber);
	if (!serv->wam){
		printf("OpenWAM failed");
		exit(1);
	}

  //Set the safety limits for each bus (WAM) 
	//setSafetyLimits(bus, joint rad/s, tip m/s, elbow m/s);
	//For now, the joint and tip velocities are ignored and
	//the elbow velocity provided is used for all three limits.
	setSafetyLimits(serv->wamnumber, 1.5, 1.5, 1.5);  // Limit to 1.5 m/s
	
	//Set the puck torque safety limits (TL1 = Warning, TL2 = Critical)
	//Note: The pucks are limited internally (see 'MT' in btsystem.c) 
	//Note: btsystem.c bounds the outbound torque to 8191, so 9000
	//tells the safety system to never register a critical fault
	setProperty(serv->wamnumber, SAFETY_MODULE, TL2, FALSE, 5700); 
	setProperty(serv->wamnumber, SAFETY_MODULE, TL1, FALSE, 3200);

	// Set the Max Torque (MT) for each motor
	//Notes: 
	//1 Amp = 1024 puck torque units
	//Puck torques are saturated at 8191 by the communications layer
	//Cable limits were chosen to be approx 20% of the rated breaking strength
	//
	// Motor             1      2      3      4      5      6      7
	// Stall (Nm)       1.490  1.490  1.490  1.490  0.356  0.356  0.091
	// Peak (Nm)        6.31   6.31   6.31   6.31   1.826  1.826  0.613
	// Cable limit (Nm) 1.8    1.8    1.8    1.6    0.6    0.6    N/A
	// Nm/A -published  0.457  0.457  0.457  0.457  0.236  0.236  0.067
	// Nm/A -actual     0.379  0.379  0.379  0.379  0.157  0.157  0.058
	//
	// Example: 
	// What should the motor 3 Max Torque (MT) be set to in order to not
	// exceeed the designed cable limit?
	// (1.8 Nm) / (0.379 Nm/A) * (1024 Units/A) = 4863
	
	setProperty(serv->wamnumber, 1, MT, FALSE, 4860); // Cable limit = 4860
	setProperty(serv->wamnumber, 2, MT, FALSE, 4860); // Cable limit = 4860
	setProperty(serv->wamnumber, 3, MT, FALSE, 4860); // Cable limit = 4860
	setProperty(serv->wamnumber, 4, MT, FALSE, 4320); // Cable limit = 4320
	setProperty(serv->wamnumber, 5, MT, FALSE, 3900); // Cable limit = 3900
	setProperty(serv->wamnumber, 6, MT, FALSE, 3900); // Cable limit = 3900
	setProperty(serv->wamnumber, 7, MT, FALSE, 3200); // J7 Gears (max stall = 1600)
	  
	//Prepare the WAM data 
	serv->jdest = new_vn(len_vn(serv->wam->Jpos));
	serv->cdest = new_vn(len_vn((vect_n*)serv->wam->HMpos));
	
	//The WAM can be in either Joint mode or Cartesian mode.
	//We want a single set of variables (active_) to eliminate the need
	//for a whole bunch of if() statements.
	serv->active_bts = &(serv->wam->Jsc);
	setmode_bts(serv->active_bts,SCMODE_IDLE);
	serv->active_pos = serv->wam->Jpos;
	serv->active_trq = serv->wam->Jtrq;
	serv->active_dest = serv->jdest;
  
	//Create a new trajectory 
	serv->vt_j = new_vta(len_vn(serv->wam->Jpos),50);
	serv->vt_c = new_vta(len_vn((vect_n*)serv->wam->HMpos),50);
	serv->vta = &serv->vt_j;
	register_vta(serv->active_bts,*serv->vta);
  
	//Register the control loop's local callback routine 
	registerWAMcallback(serv->wam, WAMcallback);

	serv->startDone = TRUE;

	//Spin until this thread is told to exit (during wamif_shutdown)
	while (!btrt_thread_done((btrt_thread_struct*)thd)){
		usleep(10000);
	}
	btrt_thread_exit((btrt_thread_struct*)thd);
}


//Activate the WAM 
int wamif_activate(WamInterface *serv) {
  int err, i;
	printf("running wamif_activate\n");

  //Read the WAM configuration file
	int busCount;  //not used
  err = ReadSystemFromConfig("wam.conf", &busCount);
	if(err) {
		LOG_INFO("ReadSystemFromConfig returned err");
		printf("ReadSystemFromConfig returned err = %d", err);
		exit(1);
	}

	//Start up arm/CAN bus in a realtime thread
	btrt_thread_create(&(serv->rt_thd), "real time thread", 45, (void*)StartupThread, (void*)serv);
	while(!serv->startDone)
		usleep(10000);

	//Initialize the control period 
	serv->wam_thd.period = Ts;

	//start main wam control thread
	btrt_thread_create(&(serv->wam_thd), "wam control thread", 90, (void*)WAMControlThread, (void*)(serv->wam));

	serv->active = true;

	printf("The WAM can be activated now.\n");
	return 0;
}


//set the torque limits (enforced in WAMcallback, set to -1 to disable)
void wamif_set_torque_limits(WamInterface *serv, double newtorquelimits[7]){
	int i;
	for(i=0; i<7; i++) torquelimits[i] = newtorquelimits[i];
}


//set the gravity compensation level
void wamif_set_gcomp(WamInterface *serv, double gc) {
	SetGravityComp(serv->wam, gc);
}


//ramp gravity compensation up to endpoint
void wamif_ramp_gcomp(WamInterface *serv, double endpoint, double ramptime, int steps) {
	double current = GetGravityComp(serv->wam);
	double range = endpoint-current;
	double step = range/steps;
	int sleeptime = (int)(1000000*ramptime/steps);
	int i;
//	printf("gcomp ramp from %0.2f to %0.2f (%0.2f, %0.2f step size)\n", current, endpoint, range, step);
	for (i=1;i<(steps+1);i++) {
		double gcomp = current + i*step;
		if (gcomp < 0.01)
			gcomp = 0.0;
		if (gcomp > 1.0)
			gcomp = 1.0;
		SetGravityComp(serv->wam, gcomp);
		usleep(sleeptime);
	}
}


//switch the active controllers to cartesian 
void wamif_switch_to_cartesian(WamInterface *serv){
	destroy_vta(serv->vta); //empty out the data if it was full
	setmode_bts(&(serv->wam->Jsc),SCMODE_IDLE);
	setmode_bts(&(serv->wam->Csc),SCMODE_IDLE);

	if (serv->active_bts == &(serv->wam->Jsc)) { //switch to cartesian space mode.
		SetCartesianSpace(serv->wam);
		serv->active_bts = &(serv->wam->Csc);
		serv->active_pos = (vect_n*)serv->wam->HMpos;
		serv->active_trq = (vect_n*)serv->wam->HMft;
		serv->active_dest = serv->cdest;
		serv->vta = &serv->vt_c;
		register_vta(serv->active_bts,*serv->vta);
	}
}


//switch the active controllers to joint
void wamif_switch_to_joint(WamInterface *serv){
	destroy_vta(serv->vta); //empty out the data if it was full
	setmode_bts(&(serv->wam->Jsc),SCMODE_IDLE);
	setmode_bts(&(serv->wam->Csc),SCMODE_IDLE);

	if(serv->active_bts == &(serv->wam->Csc)){
		SetJointSpace(serv->wam);
		serv->active_bts = &(serv->wam->Jsc);
		serv->active_pos = serv->wam->Jpos;
		serv->active_trq = serv->wam->Jtrq;
		serv->active_dest = serv->jdest;
		serv->vta = &serv->vt_j;
		register_vta(serv->active_bts,*serv->vta);
	}
}


//spin until trajectory is done
int wamif_wait_until_traj_done(WamInterface *serv){
	/*
  while(getmode_bts(serv->active_bts)==SCMODE_TRJ){
    usleep(1000);
		}*/
	while(!MoveIsDone(serv->wam)){
		usleep(1000);
	}
	stop_trj_bts(serv->active_bts);
	return 1;
}


//check to see if trajectory is done
//use this function to run a trajectory while sensing
int wamif_check_traj_done(WamInterface *serv){
	if(!serv->active)
		return true;
	/*
	int trjstat = movestatus_bts(serv->active_bts);
	if(trjstat == BTTRAJ_DONE || trjstat == BTTRAJ_STOPPED){
		return true;
	}
	return false;
	*/
	return MoveIsDone(serv->wam);
}


//print an array of 7 joint angles
void print_angles(char *label, double *jointangles){
	int i;
	printf("%s: ", label);
	for(i=0; i<7; i++) printf("%f ", jointangles[i]);
	printf("\n");
}


//stop all controllers (besides gravity comp)
void wamif_stop_controllers(WamInterface *serv){
	//stop_trj_bts(serv->active_bts);
	MoveStop(serv->wam);
	setmode_bts(serv->active_bts, SCMODE_IDLE);
}


//move through a joint angle trajectory (list of double[7]s of length length)
void wamif_move_joint_trajectory(WamInterface *serv, double jointanglelist[][7], int length){
	char vbuf[512];
	vect_n *angles = new_vn(7);

	//stop any currently running trajectories
	if(getmode_bts(serv->active_bts) == SCMODE_TRJ){
		wamif_stop_controllers(serv);
	}

	//switch to joint trajectory mode if not already there
	wamif_switch_to_joint(serv);

	//destroy the old trajectory and create a new one
	destroy_vta(serv->vta);
	//printf("destroyed vta\n");
	//serv->vt_j = new_vta(len_vn(serv->wam->Jpos),length);
	*serv->vta = new_vta(len_vn(serv->wam->Jpos),length);
	//printf("created new vta\n");
	//serv->vta = &serv->vt_j;
	register_vta(serv->active_bts,*serv->vta);
	//printf("registered vta\n");


	//clip to joint limits and add each set of joint angles to the trajectory
	int i, trajnum;
	double jointangle;
	for(trajnum = 0; trajnum < length; trajnum++){
		printf("trajnum:%d ",trajnum);
		for(i=0; i<7; i++){
			//printf("i:%d\n", i);
			jointangle = jointanglelist[trajnum][i];
			//printf("jointangle before clip: %f\n", jointangle);
			if(jointangle > jointupperlimits[i]) jointangle = jointupperlimits[i];
			else if(jointangle < jointlowerlimits[i]) jointangle = jointlowerlimits[i];
			//printf("jointangle after clip: %f\n", jointangle);
			printf("%f ",jointangle);
			setval_vn(angles, i, jointangle);
		}
		printf("\n");
		//printf("trajnum: %d\n", trajnum);
		ins_point_vta(*serv->vta, angles);
		/*
		vectray* vr;
		int cpt, nrows;
		int idx;
		char vect_buf1[2500];
		vr = get_vr_vta(*serv->vta);
		cpt = get_current_idx_vta(*serv->vta);
		nrows = numrows_vr(vr);
		printf("number of rows in serv->vta %d\n",nrows);
		printf("cpt-1 %d %s\n",cpt-1,sprint_vn(vect_buf1,idx_vr(vr,cpt-1)));
		*/
	}

	//set_current_idx_vta(*serv->vta,0);
	double vel = .75,acc = .5;
	vectray* vr;
	int cpt, nrows;
	int idx;
	char vect_buf1[2500];
	vr = get_vr_vta(*serv->vta);
	cpt = get_current_idx_vta(*serv->vta);
	nrows = numrows_vr(vr);
	//printf("number of rows in serv->vta %d\n",nrows);
	//for (i=0;i<nrows;i++){
	//	printf("pt%d %s\n",cpt,sprint_vn(vect_buf1,idx_vr(vr,i)));
	//}
	dist_adjust_vta(*serv->vta,vel);
	//for (i=0;i<nrows;i++){
	//	printf("pt%d %s\n",i,sprint_vn(vect_buf1,idx_vr(vr,i)));
	//}

	//printf("state controller state %d\t trajectory state %d\n",getmode_bts(serv->active_bts),serv->active_bts->btt.state);

	//set up the new trajectory

	MoveSetup(serv->wam,vel,acc);
	MoveWAM(serv->wam,(*(serv->active_bts->btt.reset))(&(serv->active_bts->btt)));
	//printf("state controller state %d\t trajectory state %d\n",getmode_bts(serv->active_bts),serv->active_bts->btt.state);

	printf("setting up the trajectory\n");
	wamif_wait_until_traj_done(serv);
	moveparm_bts(serv->active_bts,vel,acc);

	//printf("setting SCMODE_POS\n");
	if (getmode_bts(serv->active_bts) != SCMODE_POS)
		setmode_bts(serv->active_bts,SCMODE_POS);

	//printf("state controller state %d\t trajectory state %d\n",getmode_bts(serv->active_bts),serv->active_bts->btt.state);
	serv->active_bts->prep_only = 1;

	//for (i=0;i<nrows;i++){
	//	printf("pt%d %s\n",i,sprint_vn(vect_buf1,idx_vr(vr,i)));
	//}
	printf("starting joint space trajectory\n");
	int result = start_trj_bts(serv->active_bts);
	//printf("start_trj_bts returned %d\n",result);
	//for (i=0;i<nrows;i++){
	//	printf("pt%d %s\n",i,sprint_vn(vect_buf1,idx_vr(vr,i)));
	//}
	
	/*
	int cnt = 0;
	while (cnt<50){
		cnt++;
		usleep(100000);
		printf("state controller state %d\t trajectory state %d\n",getmode_bts(serv->active_bts),serv->active_bts->btt.state);

	}
	*/
  
}


//move to a set of joint angles 
void wamif_move_joint(WamInterface *serv, double *jointangles){
	char vbuf[512];

	//stop any currently running trajectories
	if(getmode_bts(serv->active_bts) == SCMODE_TRJ){
		wamif_stop_controllers(serv);
	}

	//switch to joint trajectory mode if not already there
	wamif_switch_to_joint(serv);

	//clip to joint limits and set desired position
	int i;
	for(i=0; i<7; i++){
		if(jointangles[i] > jointupperlimits[i]) jointangles[i] = jointupperlimits[i];
		else if(jointangles[i] < jointlowerlimits[i]) jointangles[i] = jointlowerlimits[i];
		setval_vn(serv->active_dest, i, jointangles[i]);
	}

	//log the move
	sprint_vn(vbuf, serv->active_dest);
	LOG_INFO("joint move %s\n",vbuf);

	//start trajectory
	double vel = .75,acc = .5;
  while(getmode_bts(serv->active_bts)!=SCMODE_TRJ){
    MoveSetup(serv->wam, vel, acc);
    MoveWAM(serv->wam, serv->active_dest);
  }
}


//send the arm home
void wamif_arm_home(WamInterface *serv){
	ParkWAM(serv->wam);
}


// get the current joint positions
void wamif_get_joint(WamInterface *serv, double *jointangles) {
	int i;
	for(i=0; i<7; i++) jointangles[i] = getval_vn(serv->wam->Jpos, i);
	/*
	printf("sending back joints: ");
	for(i=0; i<7; i++) {
		printf("%f ", jointangles[i]);
	}
	printf("\n");
	*/
}


//get the current motor angles 
//(not just within one revolution, but radians moved away from the zero pos)
void wamif_get_motor_angles(WamInterface *serv, double *motorangles){
	int i;
	for(i=0; i<7; i++) motorangles[i] = getval_vn(serv->wam->Mpos, i);
}


//read in a trajectory from file 
void read_trajectory_file(WamInterface *serv, char *filename){
	//read in a new trajectory from file 
  destroy_vta(serv->vta); //empty out the data if it was full
	*serv->vta = read_file_vta(filename,20);
	register_vta(&(serv->wam->Jsc),*serv->vta);

	//write it back out for debugging
	//write_file_vta(*serv->vta,"trajectoryrun.txt");
}


//run a joint angle trajectory given in filename
void wamif_run_joint_trajectory_file(WamInterface *serv, char *filename){
	read_trajectory_file(serv, filename);

	//stop any currently running trajectories
	if(getmode_bts(serv->active_bts) == SCMODE_TRJ){
		wamif_stop_controllers(serv);
	}

	//switch to joint trajectory mode if not already there
	wamif_switch_to_joint(serv);

	double vel = 3.0,acc = 2.0;

	//set up the new trajectory
	moveparm_bts(serv->active_bts,vel,acc);
	if (getmode_bts(serv->active_bts) != SCMODE_POS)
		setmode_bts(serv->active_bts,SCMODE_POS);
	
	printf("running joint space trajectory\n");
	start_trj_bts(serv->active_bts);
}


//run a Cartesian trajectory given in filename
void wamif_run_cartesian_trajectory_file(WamInterface *serv, char *filename){
	read_trajectory_file(serv, filename);

	//stop any currently running trajectories
	if(getmode_bts(serv->active_bts) == SCMODE_TRJ){
		wamif_stop_controllers(serv);
	}

	//switch to Cartesian trajectory mode if not already there
	wamif_switch_to_cartesian(serv);
 
	double vel = .25, acc = .25;

	//set up the new trajectory
	moveparm_bts(serv->active_bts,vel,acc);
	if (getmode_bts(serv->active_bts) != SCMODE_POS)
		setmode_bts(serv->active_bts,SCMODE_POS);
	
	printf("running Cartesian space trajectory\n");
	start_trj_bts(serv->active_bts);
}




//Cartesian move (cdest is Barrett 4x4 transform matrix)
void move_cartesian(WamInterface *serv, matr_h *cdest){
	char vbuf[512];

	//stop any currently running trajectories
	if(getmode_bts(serv->active_bts) == SCMODE_TRJ){
		wamif_stop_controllers(serv);
	}

	//switch to Cartesian trajectory mode if not already there
	wamif_switch_to_cartesian(serv);

	// set desired position and rotation
	set_vn(serv->active_dest, (vect_n*)cdest);

	//log the move
	sprint_vn(vbuf,serv->active_dest);
	LOG_INFO("cartmove pos %s\n",vbuf);
 
	//start the move
	double vel = .2, acc = .2;
  while(getmode_bts(serv->active_bts)!=SCMODE_TRJ){
    MoveSetup(serv->wam, vel, acc);
    MoveWAM(serv->wam, serv->active_dest);
  }
}


//move to a Cartesian position/orientation via trajectory 
//pos is [x,y,z] in meters
//rot is a 3x3 rotation matrix as a 9-vector in row order
//origin is the point where joint 0,1,2 axes meet, frame as defined in manual
void wamif_move_cartesian(WamInterface *serv, double pos[3], double rot[9]){
	//put pos and rot into a 4x4 Barrett rotation matrix 
	matr_h *cdest = new_mh();
	int i, j;
	for(i=0; i<3; i++){
		for(j=0; j<3; j++){
			ELEM(cdest, i, j) = rot[i*3+j];
		}
		ELEM(cdest, i, 3) = pos[i];
	}
	move_cartesian(serv, cdest);
}

/*
//move to a Cartesian position/orientation via trajectory 
//pos is [x,y,z] in meters
//rot is Euler angles (XYZ)
//origin is the point where joint 0,1,2 axes meet, frame as defined in manual
void wamif_move_cartesian_euler(WamInterface *serv, double pos[3], double euler[3]){
	int i;
	//convert from Euler angles to 4x4 Barrett rotation matrix
	matr_h *cdest = new_mh();
	vect_3 *RxRyRz = new_v3();
	for(i=0; i<3; i++){
		setval_v3(RxRyRz, i, euler[i]);
	}

	XYZftoR_m3((matr_3*)cdest, RxRyRz);

	for(i=0; i<3; i++){
		ELEM(cdest, i, 3) = pos[i];
	}
	move_cartesian(serv, cdest);
}
*/

//get the current cartesian position/orientation (3x3 rotation matrix)
void wamif_get_cartesian(WamInterface *serv, double pos[3], double rot[9]){
	int i, j;
	matr_mn* src = (matr_mn*)serv->wam->HMpos;
	for(i=0; i<3; i++){
		for(j=0; j<3; j++){
			rot[3*i+j] = src->q[i*4+j];
		}
		pos[i] = src->q[i*4+3];
	}
}

/*
//get the current Cartesian position/orientation (Euler angles)
void wamif_get_cartesian_euler(WamInterface *serv, double pos[3], double euler[3]){
	int i;
	matr_mn* src = (matr_mn*)serv->wam->HMpos;
	for(i=0; i<3; i++){
		pos[i] = src->q[i*4+3];
	}
	//convert from matrix to euler
	vect_3 *XYZ;
	RtoXYZf_m3((matr_3*)src, XYZ);
	for(i=0; i<3; i++){
		euler[i] = getval_v3(XYZ, i);
	}
	}*/


//having set up the desired position, tell the WAM to move,
//then wait for it to get there (used in calibration_joint_move)
void just_move_joint(WamInterface *serv, double vel, double acc){

  while(getmode_bts(serv->active_bts)!=SCMODE_TRJ){
    MoveSetup(serv->wam, vel, acc);
    MoveWAM(serv->wam, serv->active_dest);
  }
	//wait for the arm to get there
	wamif_wait_until_traj_done(serv);
}


//move to near the joint limit at jointangles, 
//then try to go past the limit by angle 'jointadd' on joint 'jointnum'
//used only for calib moves (doesn't clip to joint limits or switch modes!)
//puts the joint angles after trying to go past in 'endangles'
void calibration_joint_move(WamInterface *serv, double jointangles[7], double jointadd, int jointnum, double endangles[7]){
	int i;	
	char vbuf[512];

	//set desired position to just before the limit
	for(i=0; i<7; i++){
		if(i==jointnum){
			setval_vn(serv->active_dest, i, jointangles[i] - jointadd);
		}
		else{
			setval_vn(serv->active_dest, i, jointangles[i]);
		}
	}

	//log the move
	sprint_vn(vbuf, serv->active_dest);
	LOG_INFO("calibration joint move %s\n",vbuf);

	//go to just before the limit quickly
	double vel = .75,acc = .5;
	just_move_joint(serv, vel, acc);

	//one more try, just to make sure
	just_move_joint(serv, vel, acc);

	//now set desired position to after the limit
	setval_vn(serv->active_dest, jointnum, jointangles[jointnum] + jointadd);

	//log the move
	sprint_vn(vbuf, serv->active_dest);
	LOG_INFO("calibration joint move %s\n",vbuf);

	//go to just after the limit slowly
	vel = .25;
	acc = .5;
	just_move_joint(serv, vel, acc);

	printf("one more try\n");
	just_move_joint(serv, vel, acc);

	//get the joint angles and stick them in endangles
	wamif_get_joint(serv, endangles);

}

// offset the arm's current joint angles (for calibration)
void redefine_joint_angles(WamInterface *serv, double *jointangleoffsets) {
	double currentangles[7];

	print_angles("jointangleoffsets", jointangleoffsets);

	printf("Idle the WAM and press enter\n");
	getchar();

	wamif_get_joint(serv, currentangles);
	print_angles("old currentangles", currentangles);

	int i;
	for(i=0; i<7; i++){
		newdefinedangles[i] = jointangleoffsets[i] + currentangles[i];
	}

	//set a flag to redefine the joint angles next time WAMcallback is run
	redefineangles = 1;
	while(redefineangles); //make sure the angles are set

	//angles don't take effect until after the WAM is activated again
	printf("activate the WAM and press enter\n");
	getchar();

	wamif_get_joint(serv, currentangles);
	print_angles("new currentangles", currentangles);
}


//read in the calibration angle sequence from calibrationangles.txt
void read_in_calibration_angles(double calibrationsequence[23][7]){
	int i;
	char *line = NULL;
	size_t len = 0;
	FILE *calibanglefile = fopen("calibrationangles.txt", "r");
	if(calibanglefile == NULL){
		printf("couldn't open calibrationangles.txt!\n");
		exit(1);
	}

	int seqptr = 0;
	int jointnum = 0;
	while(getline(&line, &len, calibanglefile) != -1){
		//printf("%s", line);
		if(line[0] == '#'){
			free(line);
			line = NULL;
			continue;
		}
		if((line[0] <= '6' && line[0] >= '0') || line[0] == 'e' || line[0] == 'h'){

			//format checking
			if(line[0] <= '6' && line[0] >= '0'){
				if(jointnum != line[0] - '0'){
					printf("joint number mismatch!  Expected jointnum %d\n", jointnum);
					exit(1);
				}
			}
			else if(line[0] == 'e' || line[0] == 'h'){
				if(jointnum != 7){
					printf("e or h too early!  Expected jointnum %d\n", jointnum);
					exit(1);
				}
			}

			//read in the line of joint angles
			double jointangles[7];
			char throwaway[2];
			int read;
			double test;
			read = sscanf(line, "%1s %lf %lf %lf %lf %lf %lf %lf", throwaway, &jointangles[0], &jointangles[1], &jointangles[2], &jointangles[3], &jointangles[4], &jointangles[5], &jointangles[6]);
			if(read != 8){
				printf("malformed line: %s\n", line);
				exit(1);
			}
			for(i=0; i<7; i++) calibrationsequence[seqptr][i] = jointangles[i];

			seqptr += 1;
			if(seqptr%3 == 0){
				jointnum += 1;
			}
		}
		if(line){
			free(line);
			line = NULL;
		}
	}

	printf("sequence read:\n");
	for(i=0; i<23; i++){
		print_angles("", calibrationsequence[i]);
	}
}


//run joint calibration by moving the arm through joint angle extremes
//(for optical encoders)
//start the arm in home position!
//joint angle sequence for calibration should be in calibrationangles.txt
void wamif_calibrate_arm(WamInterface *serv){
	int i;
	char vbuf[512];

	double calibrationsequence[23][7];
	read_in_calibration_angles(calibrationsequence);

	//stop any currently running trajectories
	if(getmode_bts(serv->active_bts) == SCMODE_TRJ){
		wamif_stop_controllers(serv);
	}

	//set gravity comp to on
	wamif_set_gcomp(serv, 1.0);

	//switch to joint trajectory mode if not already there
	wamif_switch_to_joint(serv);

	//find the extremes of each joint
	double endangles1[7];
	double endangles2[7];
	double lowextremes[7];
	double highextremes[7];
	double offsets[7];
	int jointnum;
	for(jointnum=0; jointnum<7; jointnum++){
		printf("moving to safe angles\n");
		wamif_move_joint(serv, calibrationsequence[jointnum*3]);
		printf("waiting for trajectory to finish\n");
		wamif_wait_until_traj_done(serv);

		printf("moving to positive extreme\n");
		calibration_joint_move(serv, calibrationsequence[jointnum*3+1], .2, jointnum, endangles1);

		printf("moving to negative extreme\n");
		calibration_joint_move(serv, calibrationsequence[jointnum*3+2], -.2, jointnum, endangles2);

		print_angles("endangles1", endangles1);
		print_angles("endangles2", endangles2);
		lowextremes[jointnum] = endangles2[jointnum];
		highextremes[jointnum] = endangles1[jointnum];
		double listedcenter = (jointupperlimits[jointnum] + jointlowerlimits[jointnum])/2;
		double foundcenter = (highextremes[jointnum] + lowextremes[jointnum])/2;
		offsets[jointnum] = listedcenter - foundcenter;

		printf("low value: %f, listed: %f\n", lowextremes[jointnum], jointlowerlimits[jointnum]);
		printf("high value: %f, listed: %f\n", highextremes[jointnum], jointupperlimits[jointnum]);
		printf("listed center: %f, found center %f, offset %f\n", listedcenter, foundcenter, offsets[jointnum]);
		printf("listed range: %f, found range %f\n", jointupperlimits[jointnum]-jointlowerlimits[jointnum], highextremes[jointnum]-lowextremes[jointnum]);
	}

	printf("moving to last safe angles\n");
	wamif_move_joint(serv, calibrationsequence[21]);
	printf("waiting for trajectory to finish\n");
	wamif_wait_until_traj_done(serv);

	printf("moving to old home angles\n");
	wamif_move_joint(serv, calibrationsequence[22]);
	printf("waiting for trajectory to finish\n");
	wamif_wait_until_traj_done(serv);

	//turn off all controllers, including gravity comp
	wamif_stop_controllers(serv);
	wamif_set_gcomp(serv, 0);

	//check that offsets are not too large
	for(i=0; i<7; i++){
		if(offsets[i] > .1){
			printf("offset %d too large!  Setting it to zero (you should re-run calibration)\n", i);
			offsets[i] = 0;
		}
	}

	//correcting current angles
	redefine_joint_angles(serv, offsets);

	//turn gravity comp back on
	printf("turning back on gravity comp\n");
	wamif_set_gcomp(serv, 1.0);

	//move to true home
	printf("moving to true home angles\n");
	wamif_move_joint(serv, calibrationsequence[22]);
	printf("waiting for trajectory to finish\n");
	wamif_wait_until_traj_done(serv);

	double currentangles[7];
	wamif_get_joint(serv, currentangles);
	print_angles("desired home angles:", calibrationsequence[22]);
	print_angles("currentangles", currentangles);	
}





//Hand functions for use in non-threaded mode.  A much more comprehensive hand interface is in ../handinterface 


//Connect to the Barrett Hand via serial port
void wamif_hand_connect(WamInterface *serv){
	int err;
	//connect to hand
	if(err = serialOpen(&serv->handport, "/dev/ttyUSB0")) {
		printf("Error opening port: %d", err);
		exit(0);
	}
	serialSetBaud(&serv->handport,38400);
}

//wait for termination of supervisory command
void supervisory_wait(WamInterface *serv){
	char buffer[255];
	int len;
	//waits for either the termination command or 30 sec. to elapse
	serialReadLine(&serv->handport, buffer, &len, '>', 30000); 
}

//initialize the fingers (waits until the fingers are done)
void wamif_graspinitialize(WamInterface *serv){
	//lishand_sendCommand(serv->hand, "HI");
	//sleep(10);
	serialWriteString(&serv->handport, "HI\r");
	supervisory_wait(serv);
}

//close or open the fingers in supervisory mode (close=1 to close, 0 to open)
//waits until the fingers get there
void wamif_grasp(WamInterface *serv, bool close) {
	if (close) {
		//close fingers
		//lishand_sendCommand(serv->hand,"GC");
		serialWriteString(&serv->handport, "GC\r");
	} 
	else {
		//open fingers
		//lishand_sendCommand(serv->hand,"GM 4000");
		serialWriteString(&serv->handport, "GM 4000\r");
	}
	supervisory_wait(serv);
}


//ask the hand for the motor positions and change them to finger angles (rad)
//takes into account breakaway angles
//fingerangles: spread F1-joint1 F2-joint1 F3-joint1 F1-joint2 F2-joint2 F3-joint2
//For each of the three fingers, breakaway is 1 if breakaway has occurred
void wamif_get_finger_angles(WamInterface *serv, double fingerangles[7], int breakaway[3]){
	serialWriteString(&serv->handport, "S123FGET P\r");
	

}

//get the strain gauge values for each motor (assuming your fingers have the optional strain gauges--values are always 255 if not)
//values are between 0 and 255 (no strain is ~127)
void wamif_get_strain(WamInterface *serv, int strainvalues){
	serialWriteString(&serv->handport, "123FGET SG\r");
	int err;
	int bytesRead;
	char reply[3];
	int tries = 1000;
	while(bytesRead == 0 && tries-->0){
		err = serialRead(&serv->handport, reply, 3, &bytesRead);
	}

}

//send a raw command to the hand, optionally wait for termination confirmation
void wamif_hand_raw(WamInterface *serv, char* command, int waitforterm) {

	serialWriteString(&serv->handport, command);

	//wait for termination character ('>')
	if(waitforterm)
		supervisory_wait(serv);
}


//get the acknowledgement * (or print the error) 
//returns 0 upon success, -1 if error
int get_ack(WamInterface *serv){
	int err;
	int bytesRead;
	char astbuf = 0;
	int tries = 1000;
	while(astbuf != '*' && astbuf != '\n' && tries-- > 0){
		err = serialRead(&serv->handport, &astbuf, 1, &bytesRead);
		usleep(1000);
	}

	//error in reading
	if(err || bytesRead != 1){
		printf("error in reading\n");
		return -1;	
	}
	
	//hand error (put error in buffer)
  if(astbuf == '\n'){
		char buffer[255];
		serialReadLine(&serv->handport, buffer, &bytesRead, '>', 1000);		
		printf("error: %s\n", buffer);
		return -1;
	}
	return 0;
}


//read a block of size blocklen from the serial port (1-second timeout)
//returns -1 if error, else length read
int read_block_serial(WamInterface *serv, char *buffer, int blocklen){
	int err;
	int lengthRead = 0;
	int bytesRead = 0;
	int ms = 1000;

	while(lengthRead < blocklen) {
		err = serialRead(&serv->handport, buffer, 1, &bytesRead);
		/*
		//error in reading
		if(err){
			printf("error in reading\n");
			return -1;
			}*/
		
		lengthRead += bytesRead;
		buffer += bytesRead;
		usleep(20000); // Sleep for 20ms
		ms -= 20;
		if(ms < 0){
			printf("timeout!\n");
			return -1;
		}
	}	
	
	buffer[blocklen] = '\0'; // Null terminate
	
	if(lengthRead != blocklen){
		printf("error, bytesRead = %d, string read: %s\n", bytesRead, buffer);
		return -1;
	}
	return lengthRead;
}


//get feedback string in realtime mode (returns -1 if error, else number of bytes read)
//buffer contains the feedback string (not including the *)
//blocklen is the expected length of the feedback block (not including the *)
//(for the default startup in wamif_hand_start_realtime, use a blocklen of 9)
int wamif_get_realtime_feedback(WamInterface *serv, char *buffer, int blocklen){
	int err;
	int lengthRead = 0;

	//get the acknowledgement char
	err = get_ack(serv);

	if(err < 0){
		printf("error in getting ack\n");
		return -1;
	}

	//get feedback string
	if(blocklen > 0){
		lengthRead = read_block_serial(serv, buffer, blocklen);
	}

	return lengthRead;
}

//send one command in realtime to bend the fingers
//velocities and gains are signed bytes (values between -127 and +127)
//assumes the defaults were used in wamif_hand_start_realtime 
//(use wamif_hand_raw to send realtime commands otherwise)
int wamif_realtime_bend(WamInterface *serv, int vels[3], int gains[3]){
	int i;
	char cmd[7] = {'C', 100, 90, 100, 90, 100, 90};

	for(i=0; i<3; i++){
		if(vels[i] > 127) vels[i] = 127;
		else if(vels[i] < -127) vels[i] = -127;
		if(gains[i] > 127) gains[i] = 127;
		else if(gains[i] < 0) gains[i] = 0;
		cmd[i*2+1] = vels[i];
		cmd[i*2+2] = gains[i];
	}

	serialWrite(&serv->handport, cmd, 7);
}


//start realtime hand loop (returns 1 if error, else 0)
//inputs the parameter string and the loop string 
//if parameterstring == 0 or loopstring == 0, use their defaults
int wamif_hand_start_realtime(WamInterface *serv, char *parameterstring, char *loopstring){

	//default control:
	//control fingers 123 (no spread)
	//control blocks are "C" followed by velocity and proportional gain for each of the three fingers
	//feedback blocks are strain1 (1 byte) absolute-pos1 (2 bytes) strain2 pos2 strain3 pos3 (9 bytes total)
	char defaultstring[255] = "123FSET LCV 1 LCVC 1 LCPG 1 LFV 0 LFS 1 LFAP 1 LFDP 0 LFDPC 1\r";

	//set parameters
	if(!parameterstring){
		serialWriteString(&serv->handport, defaultstring);
	}
	else{
		serialWriteString(&serv->handport, parameterstring);
	}
	supervisory_wait(serv);
	printf("set parameters\n");

	//which motors to control (default is 123 bend, no spread)
	if(!loopstring){
		serialWriteString(&serv->handport, "123LOOP\r");
	}
	else{
		serialWriteString(&serv->handport, loopstring);
	}

	//get acknowledgement '*'
	int err;
	err = get_ack(serv);
	if(err < 0) return -1;

	printf("got acknowledgement *\n");
	return 0;
}


//terminate realtime mode and return to supervisory mode
void wamif_terminate_realtime(WamInterface *serv){
	char cmd;
	cmd = (signed char)3;
	serialWrite(&serv->handport, &cmd, 1);  //3 = ctrl-C
	supervisory_wait(serv);
}
