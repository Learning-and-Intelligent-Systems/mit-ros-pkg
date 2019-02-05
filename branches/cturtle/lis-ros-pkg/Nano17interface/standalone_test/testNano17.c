#include "nano17lib.h"
#include <signal.h>

Nano17Struct *nano;

void shutdown(int signum) {
	nano17_close(nano);
	exit(0);
}
  
int main(int argc, char *argv[]){

	nano = nano17_open("/dev/ttyUSB2");
	if(!nano) {
		printf("error opening Nano17\n");
		return 0;
	}  

	signal(SIGINT,shutdown);

	int values[6];
	double loc[3];
	double normal[3];
	double forcemag;
	while(1){
		nano17_getValues(nano, values);
		nano17_getLocNormalAndForce(nano, loc, normal, &forcemag);
		//printf("values:%d\t%d\t%d\t%d\t%d\t%d\n", values[0], values[1], values[2], values[3], values[4], values[5]);
		printf("normal: %3.2f\t%3.2f\t%3.2f\tloc: %3.2f\t%3.2f\t%3.2f\tforcemag:%3.2f\n", normal[0], normal[1], normal[2], loc[0], loc[1], loc[2], forcemag);

		usleep(10000000); //sleep seems to be broken
	}
	return 1;
}

