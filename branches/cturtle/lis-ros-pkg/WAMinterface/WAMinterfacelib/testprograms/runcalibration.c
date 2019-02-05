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
}

void run_main() {
	int i;

	log_open("WAMtest.log");

	//Register the ctrl-c interrupt handler 
	signal(SIGINT, sigint_handler);
	
	//create and initialize the WamInterface struct
	wamif = wamif_create(0);
	wamif_activate(wamif);

	printf("Activate WAM and press enter");
	getc(stdin);
	
	//test calibration
	wamif_calibrate_arm(wamif);

	//go home
	double homeangles[7] = {0, -1.99, 0, 3.14, 0, -.25, 0};
	printf("moving back home\n");
	wamif_move_joint(wamif, homeangles);
	wamif_wait_until_traj_done(wamif);

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
