//see how fast the hand can open and close using small increments while getting finger position feedback

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

int main(){
	int i;
	char response[MAX_FEEDBACK_LEN];
	int currpos[4];

	hand = bhand_connect("/dev/ttyUSB0");
	if (!hand)
		return 0;
	signal(SIGINT,shutdown);

	printf("incrementally closing hand\n");
	for(i=0; i<30; i++){
		bhand_sendCommandResponse(hand, "GIC 358", response);
		bhand_getFingerPositions(hand, currpos);
		printf("finger positions: %d %d %d %d\n", currpos[0], currpos[1], currpos[2], currpos[3]);
	}
	printf("incrementally opening hand\n");
	for(i=0; i<30; i++){
		bhand_sendCommandResponse(hand, "GIO 358", response);
		bhand_getFingerPositions(hand, currpos);
		printf("finger positions: %d %d %d %d\n", currpos[0], currpos[1], currpos[2], currpos[3]);
	}
	shutdown();
}
