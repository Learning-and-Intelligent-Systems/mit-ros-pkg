#include "handif.h"
#include <signal.h>

BHand *hand;

void shutdown() {
	bhand_disconnect(hand);
	exit(0);
}

int main(int argc, char *argv[]) {
	int i;
	hand = bhand_connect("/dev/ttyUSB0");
	if (!hand)
		return 0;
	signal(SIGINT,shutdown);
	printf("getting strain gauge values\n");
	while (1) {
		int strainvals[3];
		bhand_getStrainGaugeValues(hand, strainvals);
		printf("Strain: %d %d %d\n", strainvals[0], strainvals[1], strainvals[2]);
		usleep(100000);
	}
	shutdown();
}
