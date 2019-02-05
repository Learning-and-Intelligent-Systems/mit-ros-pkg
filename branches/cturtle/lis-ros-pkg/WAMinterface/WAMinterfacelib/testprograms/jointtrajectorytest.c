//test joint trajectories (run as root!)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include "waminterface/wamif.h"
#include "log/log.h"

WamInterface *wamif = NULL;
int alldone = 0;
btrt_thread_struct main_thd;


//traps Ctrl-C, shuts the WAM down before exiting
void sigint_handler(){
	if(wamif->active) wamif_destroy(wamif);
	alldone = 1;
}


void run_main(){
	int i;

	log_open("WAMtest.log");

	//Register the ctrl-c interrupt handler 
	signal(SIGINT, sigint_handler);
	
	//create and initialize the WamInterface struct
	wamif = wamif_create(0);

	wamif_activate(wamif);
			
	printf("Activate WAM and press enter");
	getc(stdin);
	//wamif_set_gcomp(wamif, 1.00);
	 
	printf("Press enter to start");
	getc(stdin);
		
	double torquelimits[7] = {9.99,9.99,9.99,9.99,9.99,9.99,9.99};
	wamif_set_torque_limits(wamif, torquelimits);

	double trajectory[4][7] = {
		{0.009007, -2.005244, 0.041125, 3.049417, 0.081722, -0.131585, -0.035286},
		{-0.000298, -1.762723, 0.034756, 2.848247, 0.062907, -0.094783, 0.002270},
		{-0.017828, -0.875831, 0.030786, 2.565059, 0.145275, 0.031809, -0.054683},
		{-0.017418, -1.459543, 0.024555, 2.865306, 0.103445, 0.092045, -0.078104}};

	wamif_move_joint_trajectory(wamif, trajectory, 4);

	printf("waiting for trajectory to finish\n");
	wamif_wait_until_traj_done(wamif);

	printf("moving back home\n");
	wamif_arm_home(wamif);	

	printf("waiting for trajectory to finish\n");
	wamif_wait_until_traj_done(wamif);

	printf("Disabling controllers (except gravity comp)\n");
	wamif_stop_controllers(wamif);
		
	printf("Idle WAM and press enter to shut down\n");
	getc(stdin);
		
	wamif_destroy(wamif);

	alldone = 1;
	log_close();
}

//run everything within a realtime thread to avoid lock failures
int main() {
	wamif_init();

 	btrt_thread_create(&main_thd, "Main", 20, (void*)run_main, NULL);
	while(!alldone){
		usleep(100000);
	}

	return 1;
}


