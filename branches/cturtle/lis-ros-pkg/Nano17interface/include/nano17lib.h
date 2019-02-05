#ifndef _NANO17_H
#define _NANO17_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "btserial.h"
#include <assert.h>
#include <time.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <limits.h>
#include <math.h>

#define RADIUS (11)  //radius of fingertip contact cylinder in mm
#define LENGTH (12)  //(estimated!) length from zero point to center of sphere
#define ACTUAL_COUNTS_PER_FORCE (20480.0)
#define ACTUAL_COUNTS_PER_TORQUE (4096.0)
#define PI (3.1415926536)

typedef struct{
	PORT port;
	pthread_t thread;
	pthread_mutex_t mutex;
	int values[6];      //Fx, Fy, Fz, Tx, Ty, Tz
	int doneflag;
} Nano17Struct;

//connect to the Nano17 on serial port 'portname' and return a new Nano17Struct
Nano17Struct *nano17_open(const char *portname);

//close the Nano17
void nano17_close(Nano17Struct *nano);

//get the current raw sensor readings
void nano17_getValues(Nano17Struct *nano, int values[6]);

//get the current contact location and force
void nano17_getLocNormalAndForce(Nano17Struct *nano, double loc[3], double normal[3], double *forcemag);	

#ifdef __cplusplus
}
#endif

#endif
