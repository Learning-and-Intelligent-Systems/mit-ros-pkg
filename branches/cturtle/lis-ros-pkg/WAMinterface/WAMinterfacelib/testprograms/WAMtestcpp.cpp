//test the waminterface functions (run as root!)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include "waminterface/wamif.h"
#include "log/log.h"

WamInterface *wamif = NULL;
int alldone = 0;
btrt_thread_struct main_thd;

/*
//traps Ctrl-C, shuts the WAM down before exiting
void sigint_handler(){
	if(wamif->active) wamif_destroy(wamif);
	alldone = 1;
	}*/

//process feedback in hand realtime mode
void get_feedback(){
	int i;
	char buffer[255];
	int bytesRead;
	int fingerpositions[3];
	int strainvals[3];
	int fingernum;	
	bytesRead = wamif_get_realtime_feedback(wamif, buffer, 9);
	if(bytesRead < 0){
		printf("error!\n");
		exit(1);
	}
	
	printf("buffer: ");
	for(i=0; i<9; i++){
		printf("%X ", (int)(0xFF & buffer[i]));
	}
	printf("\n");
	
	for(fingernum=0; fingernum<3; fingernum++){
		strainvals[fingernum] = (int)(0xFF & buffer[fingernum*3]);
		fingerpositions[fingernum] = (unsigned int)(((0xFF & buffer[fingernum*3+1]) << 8) | (0xFF & buffer[fingernum*3+2]));
	}
	printf("strainvals: %d, %d, %d\n", strainvals[0], strainvals[1], strainvals[2]);
	printf("fingerpositions: %u, %u, %u\n", fingerpositions[0], fingerpositions[1], fingerpositions[2]);
}


void run_main(){
	int i;
	int move_arm = 1;
	int move_hand = 0;

	log_open("WAMtest.log");

	//Register the ctrl-c interrupt handler 
	//signal(SIGINT, sigint_handler);
	
	//create and initialize the WamInterface struct
	wamif = wamif_create(0);

	double justabovehome[7] = {0, -1.99, 0, 2.5, 0, -.25, 0};
	double fartheroutpos[3] = {.31, 0, .42};
	double fartheroutrot[9] = {0,0,1, 
														 0,1,0,
														 -1,0,0};
	double evenfartheroutpos[3] = {.41, 0, .42};


	if(move_arm){		
		wamif_activate(wamif);
			
		printf("Activate WAM and press enter to turn on gravity comp");
		getc(stdin);
		wamif_set_gcomp(wamif, 1.00);
	 
		printf("Press enter to start");
		getc(stdin);
		
		printf("moving to just above home (in joint mode)\n");
		wamif_move_joint(wamif, justabovehome);

		printf("waiting for trajectory to finish\n");
		wamif_wait_until_traj_done(wamif);
		
		printf("getting current joint angles\n");
		double currentjointangles[7];
		wamif_get_joint(wamif, currentjointangles);
		printf("joint angles: ");
		for(i=0; i<7; i++) printf("%f ", currentjointangles[i]);
		printf("\n");
		
		printf("moving farther out (in Cartesian mode)\n");
		wamif_move_cartesian(wamif, fartheroutpos, fartheroutrot);
		
		printf("waiting for trajectory to finish\n");
		wamif_wait_until_traj_done(wamif);

		double pos[3];
		double rot[9];
		double euler[3];
		printf("getting current Cartesian position\n");
		wamif_get_cartesian(wamif, pos, rot);
		printf("pos: %.2f %.2f %.2f, rot %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n", pos[0], pos[1], pos[2], rot[0], rot[1], rot[2], rot[3], rot[4], rot[5], rot[6],rot[7],  rot[8]);
		/*
		printf("getting current Cartesian position in Euler angles\n");
		wamif_get_cartesian_euler(wamif, pos, euler);
		printf("pos: %.2f %.2f %.2f, euler %.2f %.2f %.2f\n", pos[0], pos[1], pos[2], euler[0], euler[1], euler[2]);

		printf("moving even farther out (in Cartesian Euler mode)\n");
		wamif_move_cartesian_euler(wamif, evenfartheroutpos, euler);
		*/									
		printf("waiting for trajectory to finish\n");
		wamif_wait_until_traj_done(wamif);
	}

	if(move_hand){
		printf("connecting to hand\n");
		wamif_hand_connect(wamif);

		printf("opening spread and grasp\n");
		wamif_hand_raw(wamif, "GO\r", 1);
		wamif_hand_raw(wamif, "SO\r", 1);
		
		printf("opening fingers\n");
		wamif_grasp(wamif, 0);
		
		printf("closing fingers\n");
		wamif_grasp(wamif, 1);
		
		
		printf("starting realtime mode\n");
		wamif_hand_start_realtime(wamif, 0, 0);
		
		int closevels[3] = {20, 20, 20};
		int openvels[3] = {-50, -50, -50};
		int gains[3] = {100, 100, 100};
		
		printf("closing hand\n");
		for(i=0; i<30; i++){
			printf("i = %d\n", i);
			wamif_realtime_bend(wamif, closevels, gains);
			get_feedback();
		}
		
		printf("opening hand\n");
		for(i=0; i<30; i++){
			printf("i = %d\n", i);
			wamif_realtime_bend(wamif, openvels, gains);
			get_feedback();
		}
		
		wamif_terminate_realtime(wamif);
		
		printf("opening fingers\n");
		wamif_grasp(wamif, 0);
		
		printf("sending fingers to home\n");
		wamif_hand_raw(wamif, "1234C\r", 1);
	}

	if(move_arm){
		//double homeangles[7] = {0, -1.99, 0, 3.14, 0, -.25, 0};
		printf("moving back home\n");
		//wamif_move_joint(wamif, homeangles);
		wamif_arm_home(wamif);
		
		printf("waiting for trajectory to finish (non-blocking)\n");
		while(!wamif_check_traj_done(wamif)){
			usleep(10000);
		}

		printf("Disabling controllers (except gravity comp)\n");
		wamif_stop_controllers(wamif);
		
		printf("Idle WAM and press enter to shut down\n");
		getc(stdin);
		
		wamif_destroy(wamif);
	}

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


