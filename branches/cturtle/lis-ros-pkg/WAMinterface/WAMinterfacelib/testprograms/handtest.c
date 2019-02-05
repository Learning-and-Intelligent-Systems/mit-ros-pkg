//test the hand interface (assumes hand_initialize (HI) has already been done)
//make sure the fingers are free to spread/move
//author: Kaijen Hsiao (kaijenhsiao@gmail.com)
//sleep and getchar seem not to work here, not sure why (if you figure it out, please tell me!)
#include "handif.h"
#include <signal.h>

BHand *hand;

void shutdown() {
	bhand_disconnect(hand);
	exit(0);
}

void keypause(){
	printf("press enter to continue\n");
	getchar();
}

int main(int argc, char *argv[]) {
	int testsupervisory = 1;
	int testrealtime = 1;

	int i;
	char response[MAX_FEEDBACK_LEN];

	hand = bhand_connect("/dev/ttyUSB0");
	if (!hand)
		return 0;
	signal(SIGINT,shutdown);
	
	printf("opening fingers (GO)\n");
	bhand_grasp(hand, 0, 1);
	printf("done\n");
	
	printf("opening spread (sending SO)\n");
	bhand_sendCommand(hand, "SO");
	printf("done\n");

	if(testsupervisory){
		printf("sending GM 5000 and waiting for completion\n");
		bhand_sendCommandResponse(hand, "GM 5000", response);
		printf("done\n");

		printf("moving F3 to bend 0\n");
		bhand_setFingerPosition(hand, 3, 0);  
		printf("done\n");

		printf("getting current finger positions\n");
		int currpos[4];
		bhand_getFingerPositions(hand, currpos);
		printf("finger positions: %d %d %d %d\n", currpos[0], currpos[1], currpos[2], currpos[3]);

		printf("moving fingers to spread 0, bend 5000, 10000, 15000\n");
		int positions[4] = {0, 5000, 10000, 15000};
		bhand_setAllFingerPositions(hand, positions);

		printf("waiting for all commands to be sent (nonblocking version)\n");
		while(bhand_check_done(hand)==0);
		printf("commands sent\n");

		printf("getting current finger positions\n");
		bhand_getFingerPositions(hand, currpos);
		printf("finger positions: %d %d %d %d\n", currpos[0], currpos[1], currpos[2], currpos[3]);

		printf("moving F2 to angle PI/3 (1.0)\n");
		bhand_setFingerAngle(hand, 2, 0);  
		printf("done\n");

		printf("getting current finger angles\n");
		double currangs[7];
		bhand_getFingerAngles(hand, currangs);
		printf("finger angles: %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", currangs[0], currangs[1], currangs[2], currangs[3], currangs[4], currangs[5], currangs[6]);
		printf("done\n");

		printf("moving fingers to angles: spread PI, bend PI/3 (1.0), PI/4 (.79), PI/5 (.63)\n");
		double angles[4] = {PI, PI/3, PI/4, PI/5};
		bhand_setAllFingerAngles(hand, angles);
		printf("done\n");

		printf("waiting for all commands to be sent (blocking)\n");
		bhand_wait_done(hand);

		printf("getting current finger angles\n");
		bhand_getFingerAngles(hand, currangs);
		printf("finger angles: %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", currangs[0], currangs[1], currangs[2], currangs[3], currangs[4], currangs[5], currangs[6]);
		printf("done\n");

		printf("closing fingers (GC), not waiting for completion\n");
		bhand_grasp(hand, 1, 0);
		printf("done\n");

		printf("getting the current breakaway status and positions\n");
	 	int breakawaystatus[3], breakawaypos[3];
		bhand_getFingerBreakawayPositions(hand, breakawaystatus, breakawaypos);
		printf("breakawaystatus: %d %d %d\n", breakawaystatus[0], breakawaystatus[1], breakawaystatus[2]);
		printf("breakawaypos: %d %d %d\n", breakawaypos[0], breakawaypos[1], breakawaypos[2]);
		
		printf("getting current finger angles\n");
		bhand_getFingerAngles(hand, currangs);
		printf("finger angles: %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", currangs[0], currangs[1], currangs[2], currangs[3], currangs[4], currangs[5], currangs[6]);
		printf("done\n");

		printf("getting the current strain gauge values\n");
		int strainvalues[3];
		bhand_getStrainGaugeValues(hand, strainvalues);
		printf("strain values: %d %d %d\n", strainvalues[0], strainvalues[1], strainvalues[2]);
		printf("done\n");

		printf("opening fingers (GO), waiting for completion\n");
		bhand_grasp(hand, 0, 1);
		printf("done\n");
	}

	if(testrealtime){
		printf("starting realtime mode\n");
		bhand_startRealtime(hand, 0, 0);
	
		printf("closing fingers in realtime\n");
		int closevels[3] = {20,20,20};
		int gains[3] = {100,100,100};
		int strainvals[3];
		int fingerpositions[3];
		for(i=0; i<35; i++){
			bhand_realtimeBend(hand, closevels, gains, strainvals, fingerpositions);
			printf("strainvals: %d, %d, %d\n", strainvals[0], strainvals[1], strainvals[2]);
			printf("fingerpositions: %u, %u, %u\n", fingerpositions[0], fingerpositions[1], fingerpositions[2]);
		}

		printf("opening fingers in realtime\n");
		int openvels[3] = {-50, -50, -50};
		for(i=0; i<20; i++){
			bhand_realtimeBend(hand, openvels, gains, strainvals, fingerpositions);
			printf("strainvals: %d, %d, %d\n", strainvals[0], strainvals[1], strainvals[2]);
			printf("fingerpositions: %u, %u, %u\n", fingerpositions[0], fingerpositions[1], fingerpositions[2]);
		}

		printf("terminating realtime mode\n");
		bhand_terminateRealtime(hand);
	}
	//printf("closing spread and waiting for completion\n");
	//bhand_sendCommandResponse(hand, "SC", response);
	
	shutdown();
}
