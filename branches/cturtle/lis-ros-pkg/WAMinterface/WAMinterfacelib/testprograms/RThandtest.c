//test the realtime hand functions (run as root!)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include "waminterface/wamif.h"
#include "log/log.h"

WamInterface *wamif = NULL;

//traps Ctrl-C, shuts the WAM down before exiting
void sigint_handler(){
	if(wamif->active) wamif_destroy(wamif);
	exit(0);
}

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
		wamif_terminate_realtime(wamif);
		exit(1);
	}
	if(bytesRead < 9){
		printf("didn't get all bytes!  bytesRead=%d\n", bytesRead);
	}
	/*
	printf("buffer: ");
	for(i=0; i<bytesRead; i++){
		printf("%X ", (int)(0xFF & buffer[i]));
	}
	printf("\n");
	*/
	for(fingernum=0; fingernum<3; fingernum++){
		strainvals[fingernum] = (int)(0xFF & buffer[fingernum*3]);
		fingerpositions[fingernum] = (unsigned int)(((0xFF & buffer[fingernum*3+1]) << 8) | (0xFF & buffer[fingernum*3+2]));
	}
	printf("strainvals: %d, %d, %d\n", strainvals[0], strainvals[1], strainvals[2]);
	printf("fingerpositions: %u, %u, %u\n", fingerpositions[0], fingerpositions[1], fingerpositions[2]);
}

int main(int argc, char *argv[]) {
	int i;

	log_open("WAMtest.log");

	//Register the ctrl-c interrupt handler 
	signal(SIGINT, sigint_handler);
	
	//create and initialize the WamInterface struct
	wamif = wamif_create(0);
	wamif_hand_connect(wamif);

	//start realtime mode, open and close the hand
	printf("starting realtime mode\n");
	wamif_hand_start_realtime(wamif, 0, 0);

	int closevels[3] = {20, 20, 20};
	int openvels[3] = {-50, -50, -50};
	int gains[3] = {100, 100, 100};

	printf("closing hand\n");
	for(i=0; i<50; i++){
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

	wamif_destroy(wamif);
	log_close();
	return 1;
}
