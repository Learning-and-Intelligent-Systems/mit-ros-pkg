
//Interface for the Barrett Hand
//written by Kaijen Hsiao (kaijenhsiao@gmail.com) and Ross Glashan


#include <string.h>
#include <termios.h>
#include <unistd.h>
#include "handif.h"

#define DEBUG 1

//return the current size of the queue
unsigned int queueSize(BHand *hand) {
	pthread_mutex_lock(&(hand->mutex));
	int r = (hand->queueTail-hand->queueHead)%COMMAND_QUEUE_SIZE;
	pthread_mutex_unlock(&(hand->mutex));
	if (r<0) r += COMMAND_QUEUE_SIZE;
	return r;
}


//get the acknowledgement * (or print the error) 
//returns 0 upon success, -1 if error
int get_ack_star(BHand *hand){
	int err;
	int bytesRead;
	char astbuf = 0;
	int tries = 1000;
	while(astbuf != '*' && astbuf != '\n' && tries-- > 0){
		err = serialRead(&(hand->port), &astbuf, 1, &bytesRead);
		usleep(1000);
	}

	//error in reading
	if(err || bytesRead != 1){
		printf("get_ack_star: error in reading, err=%d, bytesRead=%d\n", err, bytesRead);
		return -1;	
	}
	
	//hand error (put error in buffer)
  if(astbuf == '\n'){
		char buffer[255];
		serialReadLine(&(hand->port), buffer, &bytesRead, '>', 1000);		
		printf("get_ack_star error:\n%s\n\n", buffer);
		return -1;
	}
	return 0;
}

//read a block of size blocklen from the serial port (1-second timeout)
//returns -1 if error, else length read
//null-terminates buffer, so make buffer at least blocklen+1
int read_serial_block(BHand *hand, char *buffer, int blocklen){
	int err;
	int lengthRead = 0;
	int bytesRead = 0;
	int ms = 1000;

	while(lengthRead < blocklen) {
		err = serialRead(&(hand->port), buffer, 1, &bytesRead);
		/*
		//error in reading
		if(err){
			printf("error in reading\n");
			return -1;
			}*/
		
		lengthRead += bytesRead;
		buffer += bytesRead;
		usleep(20000); // Sleep for 20ms
		ms -= 20;
		if(ms < 0){
			printf("timeout!\n");
			return -1;
		}
	}	
	
	buffer[blocklen] = '\0'; // Null terminate
	
	if(lengthRead != blocklen){
		printf("error, bytesRead = %d, string read: %s\n", bytesRead, buffer);
		return -1;
	}
	return lengthRead;
}


//set hand->lastResponse and hand->gotResponse with locks
void setResponse(BHand *hand, char *response, int index, int responselen){
	pthread_mutex_lock(&(hand->mutex));
	if(responselen == -1)
		strcpy(hand->lastResponse,response);
	else
		memcpy(hand->lastResponse, response, responselen);
	hand->lastResponseIndex = index;
	pthread_mutex_unlock(&(hand->mutex));
}


//check hand->gotResponse until there's a response, then return it (with locks)
//for a binary block, set responselen to the size, otherwise use -1 to read until /0
void waitForResponse(BHand *hand, char *response, int index, int responselen){
	if(DEBUG)
		printf("waiting for response %d\n", index);
	while(1){
		pthread_mutex_lock(&(hand->mutex));
		if(hand->lastResponseIndex > index){
			pthread_mutex_unlock(&(hand->mutex));
			printf("missed response!\n");
			break;
		}
		if(hand->lastResponseIndex == index){
			if(responselen == -1)
				strcpy(response, hand->lastResponse);
			else{
				memcpy(response, hand->lastResponse, responselen);
			}
			pthread_mutex_unlock(&(hand->mutex));
			break;
		}
		else{
			pthread_mutex_unlock(&(hand->mutex));
		}
		usleep(10000);
	}
}


//thread function: process commands as they're added to the queue
void* processCommands(void *vhand) {
	BHand *hand = (BHand*)vhand;
	char *command;
	char resp[MAX_FEEDBACK_LEN];
	int bytesread,len;
	CommandPacket *currentCP; 
	char *respt;

	while (1) {
		// check if we have any pending commands to send out
		if (queueSize(hand)>0) {

			// flush the serial port input
			tcflush(hand->port.ifd,TCIFLUSH);

			currentCP = &hand->commandQueue[hand->queueHead];
			
			// send out command
			command = currentCP->command;
			serialWriteString(&(hand->port),command);
			free(command);

			//if in realtime mode
			if(currentCP->realtime){
				int err;
				
				//get the acknowledgement char (*)
				err = get_ack_star(hand);
				int responselen = currentCP->responselen;	
				if(err){
					printf("error in getting ack!\n");
					resp[0] = 0;
				}
				else{
					//get a feedback block of length responselen
					//printf("command %d responselen:%d\n", currentCP->index, responselen);
					err = read_serial_block(hand, resp, responselen);
					if(err == -1){
						printf("error in read_serial_block!\n");
					}
				}
				//put it in hand->lastResponse
				setResponse(hand, resp, currentCP->index, responselen);
			}
			else{
				//not in realtime mode, get the command echo/response
				bytesread = 0;
				len = 0;
				while (1) {
					serialRead(&(hand->port),resp+len,MAX_FEEDBACK_LEN,&bytesread);
					len += bytesread;
					if (len>=2) {
						char *startResp;
						if (startResp=strstr(resp,"=>")) {
							len = startResp-resp-2;
							break;
						}
					} else if (len>=MAX_FEEDBACK_LEN) {
						printf("bhand_processCommands: ERROR: command overflow\n");
						break;
					}
				}
				resp[len] = 0;
				respt = memchr(resp,'\n',len);
				if(DEBUG)
					printf("resp %d: %s\n", currentCP->index, resp);
				if(respt){
					respt = respt+2;
					if(DEBUG)
						printf("respt: %s\n", respt);
					setResponse(hand, respt, currentCP->index, -1);
				}
				else{
					setResponse(hand, resp, currentCP->index, -1);
				}
			}
			
			// next
			pthread_mutex_lock(&(hand->mutex));
			hand->queueHead = (hand->queueHead+1) % COMMAND_QUEUE_SIZE;
			pthread_mutex_unlock(&(hand->mutex));
		}
		usleep(10000);
	}
}


//Connect to the Barrett Hand via serial port and start the worker thread
BHand* bhand_connect(char *handport) {
	int res;
	BHand *hand = (BHand*)malloc(sizeof(BHand));
	hand->queueHead = hand->queueTail = 0;
	hand->lastResponseIndex = -1;
	hand->currentCommandIndex = 0;

	//connect to the hand via serial
	if (serialOpen(&(hand->port),handport)==1) {
		printf("bhand_connect: error opening serial port\n");
		goto error;
	}
	serialSetBaud(&(hand->port),38400);

	//create mutex
  res = pthread_mutex_init(&(hand->mutex), NULL);
  if (res) {
		printf("bhand_connect: error creating mutex\n");
		goto error;
	}

	//create worker thread
  res = pthread_create(&(hand->thread), NULL, processCommands, (void*)hand);
  if (res) {
		printf("bhand_connect: error creating thread\n");
		goto error;
	}

	//send empty command (equivalent of pressing enter)
	char response[MAX_FEEDBACK_LEN];
	bhand_sendCommandResponse(hand, "", response);

	printf("hand connected\n");

	// return
	return hand;

error:
	free(hand);
	return NULL;	

}


//Stop the worker thread, free the BHand structure
void bhand_disconnect(BHand *hand) {
	if (hand) {
		printf("stopping hand thread\n");
    pthread_cancel(hand->thread);
    pthread_mutex_destroy(&(hand->mutex));
		free(hand);
	}
}


//Initialize the fingers 
void bhand_initialize(BHand *hand) {
	bhand_sendCommand(hand,"HI");
}


//Shut off power to all hand motors
void bhand_powerDown(BHand *hand) {
	bhand_sendCommand(hand,"T");
}


//send a command (add it to the command queue, no response needed, don't wait)
//don't need to add \r to the end of command
void bhand_sendCommand(BHand *hand, char *command) {
	if (hand->queueTail==COMMAND_QUEUE_SIZE) {
		printf("bhand_sendCommand: queue full\n");
		return;
	}
	if(DEBUG)
		printf("sending command: %s\n", command);
	hand->commandQueue[hand->queueTail].command = (char*)malloc(strlen(command)+2);
	hand->commandQueue[hand->queueTail].realtime = 0;
	hand->commandQueue[hand->queueTail].responselen = 0;
	strcpy(hand->commandQueue[hand->queueTail].command,command);
	strcat(hand->commandQueue[hand->queueTail].command,"\r");
	hand->commandQueue[hand->queueTail].index = hand->currentCommandIndex++;
	pthread_mutex_lock(&(hand->mutex));
	hand->queueTail = (hand->queueTail+1) % COMMAND_QUEUE_SIZE;
	pthread_mutex_unlock(&(hand->mutex));
}


//send a command, wait for a response (also use to wait for command to complete if no response)
//don't need to add \r to the end of command
void bhand_sendCommandResponse(BHand *hand, char *command, char *response) {
	if (hand->queueTail==COMMAND_QUEUE_SIZE) {
		printf("bhand_sendCommandResponse: queue full\n");
		return;
	}
	if(DEBUG)
		printf("sending commandResponse: %s\n", command);
	hand->commandQueue[hand->queueTail].command = (char*)malloc(strlen(command)+2);
	hand->commandQueue[hand->queueTail].realtime = 0;
	hand->commandQueue[hand->queueTail].responselen = 1;
	strcpy(hand->commandQueue[hand->queueTail].command,command);
	strcat(hand->commandQueue[hand->queueTail].command,"\r");
	hand->commandQueue[hand->queueTail].index = hand->currentCommandIndex++;
	pthread_mutex_lock(&(hand->mutex));
	hand->queueTail = (hand->queueTail+1) % COMMAND_QUEUE_SIZE;
	pthread_mutex_unlock(&(hand->mutex));
	waitForResponse(hand, response, hand->currentCommandIndex-1, -1);
}


//wait until all commands in the queue have been carried out/finished
void bhand_wait_done(BHand *hand){
	while(!bhand_check_done(hand)){
		usleep(1000);
	}
}


//check to see if all commands in the queue have been sent (1 if so, else 0)
//note that the last command could have been sent but not completed by the hand
bool bhand_check_done(BHand *hand){
	return queueSize(hand)==0;
}


//Set the motor position for one finger in supervisory mode
//position=0-17800 for bend, 0-3150 for spread
//finger=0 for spread, 1-3 for bend
void bhand_setFingerPosition(BHand *hand, int finger, int pos) {
  if (pos > 17800) pos = 17800;
	else if(pos < 0) pos = 0;
	if (finger < 0 || finger > 3){
		printf("bhand_setFingerPos: finger should be 0,1,2,or 3\n");
		return;
	}

	char command[16];
	if (finger==0){
		sprintf(command,"SM %d",pos);
	}
	else{
		sprintf(command,"%dM %d",finger,pos);
	}
  bhand_sendCommand(hand,command);
}


//Set the motor positions for all four motors at once (spread F1 F2 F3) in supervisory mode
void bhand_setAllFingerPositions(BHand *hand, int positions[4]){
	int i;
	for(i=0; i<4; i++){
		bhand_setFingerPosition(hand, i, positions[i]);
	}
	//set spread first *and* last (can't spread at first if fingers are closed at first, can't spread at the end if fingers are closed at the end)
	bhand_setFingerPosition(hand, 0, positions[0]);
}


//translate a position (0-17800) to a joint angle (radians), motor=0 for spread
double bhand_positionToAngle(int motor, int position){
	if(motor==0){
		return (double)position / 17.5 / 180 * PI;
	}
	return (double)position / 125 / 180 * PI;
}


//translate a joint angle (radians) to a position (0-17800), motor=0 for spread
double bhand_angleToPosition(int motor, double angle){
	if(motor==0){
		return (int)(angle * 17.5 * 180 / PI);
	}
	return (int)(angle * 125 * 180 / PI);
}


//Set the joint angle for one finger (finger=0 for spread, 1-3 for bend, angle in radians) in supervisory mode
void bhand_setFingerAngle(BHand *hand, int finger, double angle) {
	//translate angle into motor position
	int fingerpos;
	fingerpos = bhand_angleToPosition(finger, angle);
	bhand_setFingerPosition(hand, finger, fingerpos);
}


//Set the finger angles for all four DOFs at once (spread F1 F2 F3) in supervisory mode
void bhand_setAllFingerAngles(BHand *hand, double angles[4]){
	int i;
	for(i=0; i<4; i++){
		bhand_setFingerAngle(hand, i, angles[i]);
	}
	//set spread first *and* last (can't spread at first if fingers are closed at first, can't spread at the end if fingers are closed at the end)
	bhand_setFingerAngle(hand, 0, angles[0]);
}		


//close or open the fingers in supervisory mode (close=1 to close, 0 to open)
//set wait=1 to wait until the fingers get there
void bhand_grasp(BHand *hand, bool close, bool wait) {
	if(wait){
		char response[MAX_FEEDBACK_LEN];
		if(close){
			bhand_sendCommandResponse(hand, "GC", response);
		} 
		else {
			bhand_sendCommandResponse(hand, "GO", response);
		}
	}
	else{
		if(close){
			bhand_sendCommand(hand, "GC");
		} 
		else {
			bhand_sendCommand(hand, "GO");
		}
	}
}


//get the motor positions for all four motors ([spread F1 F2 F3])
//values are 0-17800 for bend, 0-3150 for spread
//doesn't take into account breakaway
void bhand_getFingerPositions(BHand *hand, int positions[4]){
	char response[MAX_FEEDBACK_LEN];
	bhand_sendCommandResponse(hand, "FGET P", response);
	sscanf(response, "%d %d %d %d", &positions[1], &positions[2], &positions[3], &positions[0]); 
}


//get the motor breakaway status and positions for all three bend angles ([F1 F2 F3])
void bhand_getFingerBreakawayPositions(BHand *hand, int breakawaystatus[3], int breakawaypos[3]){
	char response[MAX_FEEDBACK_LEN];
	bhand_sendCommandResponse(hand, "123FGET BS", response);
	sscanf(response, "%d %d %d", &breakawaystatus[0], &breakawaystatus[1], &breakawaystatus[2]);
	bhand_sendCommandResponse(hand, "123FGET BP", response);
	sscanf(response, "%d %d %d", &breakawaypos[0], &breakawaypos[1], &breakawaypos[2]);
}
	

//get the finger angles for all seven joints (spread F1 F2 F3 F1tip F2tip F3tip) in joint radians 
//takes into account breakaway, but if breakaway has occurred, angles can be wildly wrong
//since the hand only detects breakaway by trying to note when a sudden acceleration occurs
void bhand_getFingerAngles(BHand *hand, double angles[7]){
	int positions[4];
	bhand_getFingerPositions(hand, positions);
	int breakawaystatus[3], breakawaypos[3];
	bhand_getFingerBreakawayPositions(hand, breakawaystatus, breakawaypos);

	int i;
	angles[0] = bhand_positionToAngle(0, positions[0]);
	for(i=1; i<4; i++){
		if(breakawaystatus[i-1]){
			angles[i] = bhand_positionToAngle(i, breakawaypos[i-1]);
			angles[i+3] = angles[i] + (float)(positions[i] - breakawaypos[i-1]) * 4/375 / 180 * PI;
		}
		else{
			angles[i] = bhand_positionToAngle(i, positions[i]);
			angles[i+3] = angles[i]/3.;
		}
	}
}


//Get all strain gauge values (F1 F2 F3)
void bhand_getStrainGaugeValues(BHand *hand, int strainvalues[3]) {
	char response[MAX_FEEDBACK_LEN];
	bhand_sendCommandResponse(hand, "123FGET SG", response);
	int i;
	for(i=0; i<3; i++){
		strainvalues[i] = atoi(&response[i]);
	}
}



//start realtime hand loop (returns 1 if error, else 0)
//inputs the parameter string and the loop string (add /r to the loop string)
//if parameterstring == 0 or loopstring == 0, use their defaults
int bhand_startRealtime(BHand *hand, char *parameterstring, char *loopstring){
	char buffer[MAX_FEEDBACK_LEN];

	//default control:
	//control fingers 123 (no spread)
	//control blocks are "C" followed by velocity and proportional gain for each of the three fingers
	//feedback blocks are strain1 (1 byte) absolute-pos1 (2 bytes) strain2 pos2 strain3 pos3 (9 bytes total)
	char defaultstring[255] = "123FSET LCV 1 LCVC 1 LCPG 1 LFV 0 LFS 1 LFAP 1 LFDP 0 LFDPC 1";

	//set parameters
	if(!parameterstring){
		bhand_sendCommandResponse(hand, defaultstring, buffer);
	}
	else{
		bhand_sendCommandResponse(hand, parameterstring, buffer);
	}
	printf("set parameters\n");

	//which motors to control (default is 123 bend, no spread)
	if(!loopstring){
		bhand_sendRealtimeCommand(hand, "123LOOP\r", 0, buffer);
	}
	else{
		bhand_sendRealtimeCommand(hand, loopstring, 0, buffer);
	}
	return 0;
}


//terminate realtime mode and return to supervisory mode
void bhand_terminateRealtime(BHand *hand){
	char cmd[2] = {3, 0}; //3 = ctrl-C
	char buffer[MAX_FEEDBACK_LEN];
	bhand_sendCommandResponse(hand, cmd, buffer);  
}


//send one command in realtime to bend the fingers
//velocities and gains are signed bytes (values between -127 and +127)
//assumes the defaults were used in wamif_hand_start_realtime 
//(use wamif_hand_raw to send realtime commands otherwise)
//returns the default feedback (strainvals[3], fingerpositions[3])
void bhand_realtimeBend(BHand *hand, int vels[3], int gains[3], int strainvals[3], int fingerpositions[3]){
	int i;
	char cmd[8] = {'C', 100, 90, 100, 90, 100, 90, 0};

	for(i=0; i<3; i++){
		if(vels[i] > 127) vels[i] = 127;
		else if(vels[i] < -127) vels[i] = -127;
		if(gains[i] > 127) gains[i] = 127;
		else if(gains[i] < 0) gains[i] = 0;
		cmd[i*2+1] = vels[i];
		cmd[i*2+2] = gains[i];
	}

	char response[MAX_FEEDBACK_LEN];
	bhand_sendRealtimeCommand(hand, cmd, 9, response);

	//process feedback
	int fingernum;
	for(fingernum=0; fingernum<3; fingernum++){
		strainvals[fingernum] = (int)(0xFF & response[fingernum*3]);
		fingerpositions[fingernum] = (unsigned int)(((0xFF & response[fingernum*3+1]) << 8) | (0xFF & response[fingernum*3+2]));
	}
	if(DEBUG){
		printf("strainvals: %d, %d, %d\n", strainvals[0], strainvals[1], strainvals[2]);
		printf("fingerpositions: %u, %u, %u\n", fingerpositions[0], fingerpositions[1], fingerpositions[2]);
	}
}


//send a realtime command, wait for the acknowledgement and get the feedback block 
//response contains the feedback string (not including the *)
//feedbacklen is the expected length of the feedback block (not including the *)
//response is null-terminated, so make sure it's feedbacklen+1 long
void bhand_sendRealtimeCommand(BHand *hand, char *command, int feedbacklen, char *response){
	if (hand->queueTail==COMMAND_QUEUE_SIZE) {
		printf("bhand_sendCommandResponse: queue full\n");
		return;
	}
	//printf("sending realtime command: %s\n", command);
	hand->commandQueue[hand->queueTail].command = (char*)malloc(strlen(command)+2);
	hand->commandQueue[hand->queueTail].realtime = 1;
	hand->commandQueue[hand->queueTail].responselen = feedbacklen;
	strcpy(hand->commandQueue[hand->queueTail].command,command);
	hand->commandQueue[hand->queueTail].index = hand->currentCommandIndex++;
	pthread_mutex_lock(&(hand->mutex));
	hand->queueTail = (hand->queueTail+1) % COMMAND_QUEUE_SIZE;
	pthread_mutex_unlock(&(hand->mutex));
	printf("getting feedbacklen %d\n", feedbacklen);
	waitForResponse(hand, response, hand->currentCommandIndex-1, feedbacklen);
}

