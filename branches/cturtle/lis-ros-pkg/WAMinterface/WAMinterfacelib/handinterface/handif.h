#ifndef _BHAND_H
#define _BHAND_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "pthread.h"

#ifndef BTSERIAL
#include "btserial.h"
#define BTSERIAL
#endif

#define COMMAND_QUEUE_SIZE 16
#define MAX_FEEDBACK_LEN 100
#define PI (3.14159265359)

typedef struct {
	int index;
	char *command;
	bool realtime;    //1 if in realtime mode, 0 otherwise
	int responselen;  //if not realtime, 1 to get a supervisory response; if realtime, length of expected feedback block
} CommandPacket;

typedef struct {
	PORT port;
	pthread_t thread;
	pthread_mutex_t mutex;
	CommandPacket commandQueue[COMMAND_QUEUE_SIZE];
	unsigned char queueHead, queueTail;
	char lastResponse[MAX_FEEDBACK_LEN];
	int lastResponseIndex;
	int currentCommandIndex;
} BHand;

//Connect to the Barrett Hand via serial port and start the worker thread
BHand* bhand_connect(char* handport);

//Stop the worker thread, free the BHand structure
void bhand_disconnect(BHand *hand);

//Initialize the fingers
void bhand_initialize(BHand *hand);

//Shut off power to all hand motors
void bhand_powerDown(BHand *hand);

//send a command (add it to the command queue, no response needed, don't wait)
void bhand_sendCommand(BHand *hand, char* command);

//send a command, wait for a response (also use to wait for command to complete if no response)
void bhand_sendCommandResponse(BHand *hand, char* command, char *response);

//wait until all commands in the queue have been carried out/finished
void bhand_wait_done(BHand *hand);

//check to see if all commands in the queue have been sent (1 if so, else 0)
//note that the last command could have been sent but not completed by the hand
bool bhand_check_done(BHand *hand);

//Set the position for one finger (0 for spread, value from 0 to 18000)
void bhand_setFingerPosition(BHand *hand, int finger, int pos);

//Set the motor positions for all four motors at once (spread F1 F2 F3)
void bhand_setAllFingerPositions(BHand *hand, int positions[4]);

//translate a position (0-17800) to a joint angle (radians), motor=0 for spread
double bhand_positionToAngle(int motor, int position);

//translate a joint angle (radians) to a position (0-17800), motor=0 for spread
double bhand_angleToPosition(int motor, double angle);

//Set the joint angle for one finger (0 for spread, angle in radians)
void bhand_setFingerAngle(BHand *hand, int finger, double angle);

//Set the finger angles for all four DOFs at once (spread F1 F2 F3)
void bhand_setAllFingerAngles(BHand *hand, double angles[4]);

//close or open the fingers in supervisory mode (close=1 to close, 0 to open)
//set wait=1 to wait until the fingers get there
void bhand_grasp(BHand *hand, bool close, bool wait);

//get the motor positions for all four motors (values from 0 to 18000, [spread F1 F2 F3])
void bhand_getFingerPositions(BHand *hand, int positions[4]);

//get the motor breakaway status and positions for all three bend angles ([F1 F2 F3])
void bhand_getFingerBreakawayPositions(BHand *hand, int breakawaystatus[3], int breakawaypos[3]);

//get the finger angles for all seven joints (spread F1 F2 F3 F1tip F2tip F3tip) in joint radians 
//takes into account breakaway, but if breakaway has occurred, angles can be wildly wrong
//since the hand only detects breakaway by trying to note when a sudden acceleration occurs
void bhand_getFingerAngles(BHand *hand, double angles[7]);

//Get the strain gauge values
void bhand_getStrainGaugeValues(BHand *hand, int strainvalues[3]);



//start realtime hand loop (returns 1 if error, else 0)
//inputs the parameter string and the loop string 
//if parameterstring == 0 or loopstring == 0, use their defaults
int bhand_startRealtime(BHand *hand, char *parameterstring, char *loopstring);

//terminate realtime mode and return to supervisory mode
void bhand_terminateRealtime(BHand *hand);

//send one command in realtime to bend the fingers
//velocities and gains are signed bytes (values between -127 and +127)
//assumes the defaults were used in wamif_hand_start_realtime 
//(use wamif_hand_raw to send realtime commands otherwise)
//returns the default feedback (strainvals[3], fingerpositions[3])
void bhand_realtimeBend(BHand *hand, int vels[3], int gains[3], int strainvals[3], int fingerpositions[3]);

//send a realtime command, wait for the acknowledgement and get the feedback block 
//response contains the feedback string (not including the *)
//feedbacklen is the expected length of the feedback block (not including the *)
//response is null-terminated, so make sure it's feedbacklen+1 long
void bhand_sendRealtimeCommand(BHand *hand, char *command, int feedbacklen, char *response);


#endif
