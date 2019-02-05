// Socket interface to waminterface for Barrett Arm and/or Hand
// Must be running a server for socketwamif to connect to *first!*
// (In order to compile with btclient, we can only use socket clients,
// not servers, or we get segfaults.)
// written by Kaijen Hsiao (questions? email kaijenhsiao@gmail.com)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sys/socket.h>

#include "handinterface/handif.h"
#include "socketwamif/socketcmds.h"
#include "waminterface/wamif.h"
#include "log/log.h"

char server_ip[50] = "127.0.0.1"; //localhost
char handport[50] = "/dev/ttyUSB0";

unsigned short int server_port = 4321;
int sockid = -1;

WamInterface *wamif = NULL;
BHand *hand = NULL;
int alldone = 0;
btrt_thread_struct main_thd;


//shut down the arm and hand (whichever is open), and close the socket client
void shut_down(){
	printf("shutting down\n");
	if(wamif){
		wamif_destroy(wamif);
	}
	if(hand){
		bhand_disconnect(hand);
	}
	printf("closing the socket client\n");
	if(sockid != -1) socket_client_close(sockid);
	printf("closing the log\n");
	log_close();
	alldone = 1;
	printf("exiting\n");
	exit(1);
}


//traps Ctrl-C, closes socket and shuts the WAM down before exiting
void sigint_handler(){
	shut_down();
}


//print out an array of doubles
void print_double_array(double *array, int length){
	int i;
	for(i=0; i<length; i++){
		printf("%f ", array[i]);
	}
	printf("\n");
}


//parses input to figure out which command, gets parameters, runs the command
void parse_input(){
	int i, err;
	char header[4];
	char response[50];
	char separator;

	while((err = read(sockid, &header, 3)) == 0){
		usleep(1000);
	}
	if(err < 0){
		printf("socket error!  Trying to ignore...\n");
		return;
		//shut_down();
	}
	if(err != 3){
		printf("error in receiving header! received: %s\n", header);
		return;
	}
	header[3] = '\0';
	printf("received header %s\n", header);



	//General arm commands

	//connect the arm (must be run before other arm commands!)
	//hand must be connected separately with "hct"
	if(!strcmp(header, "act")){
		if(wamif){
			printf("arm already connected!\n");
			write(sockid, "e", 1);
			return;
		}
		//create and initialize the WamInterface struct
		wamif = wamif_create(0);
		wamif_activate(wamif);
		//wamif_hand_connect(wamif);
		printf("Activate the robot and press enter\n");
		getchar();
		write(sockid, "d", 1);
	}

	//shut down arm, hand, and socket
	else if(!strcmp(header, "shu")){
		printf("shutting down arm/hand/socket\n");
		if(wamif){
			printf("idle the robot and press enter\n");
			getchar();
		}
		shut_down();
	}

	//just shut down the arm (not the hand or socket, re-connect it with "act")
	else if(!strcmp(header, "asd")){
		if(wamif){
			printf("shutting down just the arm, idle the robot and press enter\n");
			getchar();
			wamif_destroy(wamif);
			wamif = NULL;
			write(sockid, "d", 1);		
		}
		else{
			printf("arm wasn't connected\n");
			write(sockid, "e", 1);
		}
	}

	//turn on gravity comp (send back 'd' when done)
	else if(!strcmp(header, "gra")){
		if(wamif){
			printf("turning on gravity compensation\n");
			wamif_set_gcomp(wamif, 1.00);
			write(sockid, "d", 1);
		}
		else{
			printf("connect the arm first!\n");
			write(sockid, "e", 1);
		}
	}

	//run joint angle calibration 
	//(optical encoders only, put arm in home position first)
	//joint angle sequence for calibration should be in calibrationangles.txt
	else if(!strcmp(header, "cal")){
		if(wamif){
			printf("running arm calibration\n");
			wamif_calibrate_arm(wamif);
			write(sockid, "d", 1);
		}
		else{
			printf("connect the arm first!\n");
			write(sockid, "e", 1);
		}
	}

	//wait for trajectory to finish (send back 'd' when done)
	else if(!strcmp(header, "trf")){
		if(wamif){
			wamif_wait_until_traj_done(wamif);
			write(sockid, "d", 1);
		}
		else{
			printf("connect the arm first!\n");
			write(sockid, "e", 1);
		}
	}

	//check if trajectory is done (non-blocking, sends '1' if done and '0' otherwise)
	else if(!strcmp(header, "ckt")){
		if(wamif){
			if(wamif_check_traj_done(wamif)){
				write(sockid, "1", 1);
				printf("1");
			}
			else{
				write(sockid, "0", 1);
				printf("0");
			}
		}
		else{
			printf("connect the arm first!\n");
			write(sockid, "e", 1);
		}
	}

	//disable controllers
	else if(!strcmp(header, "dis")){
		if(wamif){
			printf("stopping controllers\n");
			wamif_stop_controllers(wamif);
			write(sockid, "d", 1);
		}
		else{
			printf("connect the arm first!\n");
			write(sockid, "e", 1);
		}
	}



	//Arm joint control


	//move to joint angles 
	//(takes in 7-array of doubles, sends back 'd' when done setting up trajectory)
	else if(!strcmp(header, "moj")){
		if(wamif){
			printf("moving joint\n");
		
			//get joint angles
			double jointangles[7];
			if(!read_double_array(sockid, jointangles, 7, &separator)){
				printf("error reading joint angles!\n");
				return;
			}
		
			printf("jointangles: ");
			print_double_array(jointangles, 7);
			
			//move to joint angles
			wamif_move_joint(wamif, jointangles);
			
			write(sockid, "d", 1);
		}
		else{
			printf("connect the arm first!\n");
			write(sockid, "e", 1);
		}
	}
 
	//get current joint angles (returns 7-array of doubles)
	else if(!strcmp(header, "gja")){
		double jointangles[7] = {0,0,0,0,0,0,0};
		if(wamif){
			printf("returning current joint angles\n");
			wamif_get_joint(wamif, jointangles);
		}
		else{
			printf("connect the arm first!\n");			
		}
		write_double_array(sockid, jointangles, 7);
	}

	//get current motor angles (rad from zero pos, returns 7-array of doubles)
	else if(!strcmp(header, "gmc")){
		double motorangles[7] = {0,0,0,0,0,0,0};
		if(wamif){
			printf("returning current motor angles\n");
			wamif_get_motor_angles(wamif, motorangles);
		}
		else{
			printf("connect the arm first!\n");
		}
		write_double_array(sockid, motorangles, 7);
	}

	//send arm back home
	else if(!strcmp(header, "hom")){
		if(wamif){
			printf("sending arm back home\n");
			wamif_arm_home(wamif);
			write(sockid, "d", 1);
		}
		else{
			printf("connect the arm first!\n");
			write(sockid, "e", 1);
		}
	}

	//set the torque limits for the arm (Nm)
	//{6.31, 6.31, 6.31, 6.31, 1.826, 1.826, 0.613} are Barrett's stated peak values
	//{7.75,7.75,7.75,7.75,2.5,2.5,2} gives you most of the possible torque while avoiding torque faults most of the time
	else if(!strcmp(header, "tlm")){
		//get torque limits
		double torquelimits[7];
		if(!read_double_array(sockid, torquelimits, 7, &separator)){
			printf("error reading torquelimits!\n");
			return;		
		}
		if(wamif){
			wamif_set_torque_limits(wamif, torquelimits);
			write(sockid, "d", 1);
		}
		else{
			printf("connect the arm first!\n");
			write(sockid, "e", 1);
		}		
	}


	//Arm Cartesian control


	//move to Cartesian position/orientation
	//pos is [x,y,z] in meters
	//rot is a 3x3 rotation matrix as a 9-vector [R11, R12, R13, R21... R33]
	//(origin is the point where joint 0,1,2 axes meet, frame as defined in manual)
	else if(!strcmp(header, "moc")){
		if(wamif){
			printf("Cartesian move\n");
			
			//get position (3-vector)
			double pos[3];
			if(!read_double_array(sockid, pos, 3, &separator))
				return;
			
			//get rotation (9-vector)
			double rot[9];
			if(!read_double_array(sockid, rot, 9, &separator))
				return;
			
			//move arm
			printf("pos: %f, %f, %f\n", pos[0], pos[1], pos[2]);
			printf("rot: %f, %f, %f, %f, %f, %f, %f, %f, %f\n", rot[0], rot[1], rot[2], rot[3], rot[4], rot[5], rot[6], rot[7], rot[8]);
			wamif_move_cartesian(wamif, pos, rot);
			
			write(sockid, "d", 1);
		}
		else{
			printf("connect the arm first!\n");
			write(sockid, "e", 1);
		}
	}

	//get Cartesian position/orientation
	//pos is [x,y,z] in meters
	//rot is a 3x3 rotation matrix as a 9-vector [R11, R12, R13, R21... R33]
	//(origin is the point where joint 0,1,2 axes meet, frame as defined in manual)
	else if(!strcmp(header, "gcp")){
		double pos[3] = {0,0,0};
		double rot[9] = {0,0,0,0,0,0,0,0,0};

		if(wamif){
			
			wamif_get_cartesian(wamif, pos, rot);
			
			//printf("pos: %f, %f, %f\n", pos[0], pos[1], pos[2]);
			//printf("rot: %f, %f, %f, %f, %f, %f, %f, %f, %f\n", rot[0], rot[1], rot[2], rot[3], rot[4], rot[5], rot[6], rot[7], rot[8]);
		}
		else{
			printf("connect the arm first!\n");
		}
			
		//send position
		write_double_array(sockid, pos, 3);
		
		//send rotation
		write_double_array(sockid, rot, 9);
	}

	//Arm joint trajectory control

	//move through a joint angle trajectory (list of double[7]s of length length)
	else if (!strcmp(header, "jtc")){
		if (wamif){
			//get length
			int len = 0;
			if (!read_int(sockid, &len, &separator)){
				printf("error in reading length!\n");
				return;
			}
		
			//get joint angle list
			double jointanglelist[len][7];
			int trajnum;
			for (trajnum = 0; trajnum < len; trajnum++){
				if (!read_double_array(sockid, jointanglelist[trajnum], 7, &separator)){
					printf("error in reading joint angle list %d!\n",trajnum);
					return;
				}
			}

			//test input
			printf("length %d\n",len);
			for (trajnum = 0; trajnum < len; trajnum++){
				for (i = 0; i < 7; i++){
					printf("%f\t",jointanglelist[trajnum][i]);
				}
				printf("\n");
			}
			//move arm
			wamif_move_joint_trajectory(wamif, jointanglelist, len);		
			write(sockid, "d", 1);
		}
		else{
			printf("connect the arm first!\n");
			write(sockid, "e", 1);
		}
	}

	//Hand supervisory commands

	//connect to hand (must be run before all other hand commands!)
	else if(!strcmp(header, "hct")){
		printf("connecting to hand\n");
		hand = bhand_connect(handport);
		write(sockid, "d", 1);
	}

	//disconnect from hand (can be reconnected with "hct")
	else if(!strcmp(header, "hsd")){
		if(hand){
			printf("disconnecting from hand\n");
			bhand_disconnect(hand);
			hand = NULL;
			write(sockid, "d", 1);
		}
		else{
			printf("connect to hand first!\n");
			write(sockid, "e", 1);
		}
	}

	//open fingers (wait for termination, send back 'd' when done)
	else if(!strcmp(header, "opn")){
		if(hand){
			printf("opening fingers\n");
			//wamif_grasp(wamif, 0);
			bhand_grasp(hand, 0, 1);
			write(sockid, "d", 1);
		}
		else{
			printf("connect to hand first!\n");
			write(sockid, "e", 1);
		}
	}

	//close fingers (wait for termination, send back 'd' when done)
	else if(!strcmp(header, "cls")){
		if(hand){
			printf("closing fingers\n");
			//wamif_grasp(wamif, 1);
			bhand_grasp(hand, 0, 1);
			write(sockid, "d", 1);
		}
		else{
			printf("connect to hand first!\n");
			write(sockid, "e", 1);
		}
	}

	//open spread as well as grasp (wait for termination, send back 'd' when done)
	else if(!strcmp(header, "sgo")){
		if(hand){
			bhand_sendCommand(hand, "GO");
			bhand_sendCommandResponse(hand, "SO", response);
			//wamif_hand_raw(wamif, "GO\r", 1);
			//wamif_hand_raw(wamif, "SO\r", 1);
			write(sockid, "d", 1);
		}
		else{
			printf("connect to hand first!\n");
			write(sockid, "e", 1);
		}
	}

	//make the hand go back into the home position (wait for termination, send back 'd' when done)
	else if(!strcmp(header, "hhe")){
		if(hand){
			printf("hand home\n");
			bhand_sendCommand(hand, "GM 4000");
			bhand_sendCommandResponse(hand, "1234C", response);
			//wamif_hand_raw(wamif, "GM 4000\r", 1);
			//wamif_hand_raw(wamif, "1234C\r", 1);
			write(sockid, "d", 1);
		}
		else{
			printf("connect to hand first!\n");
			write(sockid, "e", 1);
		}
	}

	//set the position (0-17800 for bend, 0-3150 for spread) for one motor (0 for spread, 1-3 for finger bend)
	//wait for termination, send back 'd' when done
	else if(!strcmp(header, "omp")){
		int position = 0;
		int motor = 0;
		if(!read_int(sockid, &position, &separator)){
			printf("error in reading position!\n");
			return;
		}
		if(!read_int(sockid, &motor, &separator)){
			printf("error in reading motor!\n");
			return;
		}
		if(hand){
			printf("sending finger %d to position %d\n", motor, position);
			bhand_setFingerPosition(hand, motor, position);
			bhand_wait_done(hand);
			write(sockid, "d", 1);
		}
		else{
			printf("connect to hand first!\n");
			write(sockid, "e", 1);
		}
	}

	//set the positions for all motors (spread and bend, [S F1 F2 F3])
	//wait for termination, send back 'd' when done
	else if(!strcmp(header, "amp")){
		int positions[4];
		if(!read_int_array(sockid, positions, 4, &separator)){
			printf("error in reading positions!\n");
			return; 
		}
		if(hand){
			bhand_setAllFingerPositions(hand, positions);
			bhand_wait_done(hand);
			write(sockid, "d", 1);
		}
		else{
			printf("connect to hand first!\n");
			write(sockid, "e", 1);
		}
	}

	//set the angles (radians) for one motor (0 for spread, 1-3 for finger bend)
	//wait for termination, send back 'd' when done
	else if(!strcmp(header, "oma")){
		double angle;
		int motor;
		if(!read_double(sockid, &angle, &separator)){
			printf("error in reading angle!\n");
			return;
		}
		if(!read_int(sockid, &motor, &separator)){
			printf("error in reading motor!\n");
			return;
		}
		if(hand){
			bhand_setFingerAngle(hand, motor, angle);
			bhand_wait_done(hand);
			write(sockid, "d", 1);
		}
		else{
			printf("connect to hand first!\n");
			write(sockid, "e", 1);
		}
	}

	//set the angles (radians) for all four motors ([S F1 F2 F3])
	//wait for termination, send back 'd' when done
	else if(!strcmp(header, "ama")){
		double angles[4];
		if(!read_double_array(sockid, angles, 4, &separator)){
			printf("error in reading angles!\n");
			return;
		}
		if(hand){
			bhand_setAllFingerAngles(hand, angles);
			bhand_wait_done(hand);
			write(sockid, "d", 1);
		}
		else{
			printf("connect to hand first!\n");
			write(sockid, "e", 1);
		}
	}

	//get the positions (0-17800 for bend, 0-3150 for spread) for all four motors ([S F1 F2 F3]) 
	else if(!strcmp(header, "gmp")){
		int positions[4] = {0,0,0,0};
		if(hand){
			bhand_getFingerPositions(hand, positions);
		}
		else{
			printf("connect to hand first!\n");
		}
		write_int_array(sockid, positions, 4);
	}

	//get the breakaway status and positions for the three fingers [F1 F2 F3]
	else if(!strcmp(header, "brk")){
		int breakawaystatus[3] = {0,0,0};
		int breakawaypos[3] = {0,0,0};
		if(hand){
			bhand_getFingerBreakawayPositions(hand, breakawaystatus, breakawaypos);
		}
		else{
			printf("connect to hand first!\n");
		}
		write_int_array(sockid, breakawaystatus, 3);
		write_int_array(sockid, breakawaypos, 3);
	}

	//get the angles (radians) for all seven joints ([S F1 F2 F3 F1tip F2tip F3tip])
	else if(!strcmp(header, "gma")){
		double angles[7] = {0,0,0,0,0,0,0};
		if(hand){
			bhand_getFingerAngles(hand, angles);
		}
		else{
			printf("connect to hand first!\n");
		}
		write_double_array(sockid, angles, 7);
	}

	//get the strain gauge values ([F1 F2 F3])
	else if(!strcmp(header, "gsg")){
		int strainvals[3] = {0,0,0};
		if(hand){
			bhand_getStrainGaugeValues(hand, strainvals);
		}
		else{
			printf("connect to hand first!\n");
		}
		write_int_array(sockid, strainvals, 3);
	}

	//raw hand command (wait for termination, send back 'd' when done, 'e' if error)
	//terminate command string by '\n' or '\0'
	else if(!strcmp(header, "cmd")){
		if(hand){
			printf("raw hand command\n");
			char rawcmd[256];
			int len;
			len = read_string(sockid, rawcmd, &separator);
			if(len == -1){
				printf("error in reading command!\n");
				write(sockid, "e", 1);
				return;
			}
			bhand_sendCommandResponse(hand, rawcmd, response);
			//wamif_hand_raw(wamif, rawcmd, 1);
			printf("sending back d\n");
			write(sockid, "d", 1);
		}
		else{
			printf("connect to hand first!\n");
			write(sockid, "e", 1);
		}
	}



	//Hand realtime commands


	//start hand realtime mode
	//takes in the parameter string (send '0\n' to use the default) 
	//then the loop string (send '0\n' to use the default)
	//each separated/terminated by ' ' or '\n' or '\0' 
	else if(!strcmp(header, "hrs")){
		char parameterstring[256];
		char loopstring[256];
		if(read_string(sockid, parameterstring, &separator) == -1)
			return;
		if(read_string(sockid, loopstring, &separator) == -1)
			return;
		if(hand){
			if(parameterstring[0] == '0' && loopstring[0] == '0'){ 
				printf("starting hand realtime with defaults\n");
				bhand_startRealtime(hand, 0,0);
				//wamif_hand_start_realtime(wamif, 0,0);
			}
			else if(parameterstring[0] == '0')
				bhand_startRealtime(hand, 0, loopstring);
				//wamif_hand_start_realtime(wamif, 0, loopstring);
			else if(loopstring[0] == '0')
				bhand_startRealtime(hand, parameterstring, 0);
				//wamif_hand_start_realtime(wamif, parameterstring, 0);
			else{
				printf("parameterstring: %s\n", parameterstring);
				printf("loopstring: %s\n", loopstring);
				bhand_startRealtime(hand, parameterstring, loopstring);
				//wamif_hand_start_realtime(wamif, parameterstring, loopstring);
			}
			write(sockid, "d", 1);
		}
		else{
			printf("connect to hand first!\n");
			write(sockid, "e", 1);
		}
	}

	//terminate hand realtime mode (back to supervisory mode)
	else if(!strcmp(header, "hre")){
		if(hand){
			printf("terminating realtime mode\n");
			bhand_terminateRealtime(hand);
			//wamif_terminate_realtime(wamif);
			write(sockid, "d", 1);
		}
		else{
			printf("connect to hand first!\n");
			write(sockid, "e", 1);
		}
	}

	//bend the fingers in realtime mode
	//assumes the default startup in wamif_hand_start_realtime
	//(use "src" to send other realtime commands)
	//takes in vels[3] followed by gains[3]
	//(each vel or gain is an int between -127 and 127)
	//sends back the default feedback (strainvals[3], fingerpositions[3])
	else if(!strcmp(header, "hrb")){
		int vels[3];
		int gains[3];
		int strainvals[3] = {0,0,0};
		int fingerpositions[3] = {0,0,0};
		if(!read_int_array(sockid, vels, 3, &separator))
			return;
		if(!read_int_array(sockid, gains, 3, &separator))
			return;
		//printf("realtime bend, vels: %d, %d, %d, gains: %d, %d, %d\n", vels[0], vels[1], vels[2], gains[0], gains[1], gains[2]);
		if(hand){
			bhand_realtimeBend(hand, vels, gains, strainvals, fingerpositions); 
			//wamif_realtime_bend(wamif, vels, gains);
		}
		else{
			printf("connect to hand first!\n");
		}
		write_int_array(sockid, strainvals, 3);
		write_int_array(sockid, fingerpositions, 3);
	}

	//send generic realtime command and get feedback block 
	//(must be in realtime mode already)
	//takes in the command, terminated by '\n' or '\0'
	//followed by the expected length of the feedback block (not including the *)
	//sends back the feedback block
	//length can be 0 if you just want the acknowledgement char (*)
	else if(!strcmp(header, "hrf")){

		//read in the command
		char rawcmd[256];
		int len;
		len = read_string(sockid, rawcmd, &separator);
		if(len == -1){
			printf("error in reading command!\n");
			return;
		}

		//read in the feedback block length
		int blocklen = 0;
		if(!read_int(sockid, &blocklen, &separator)){
			printf("error in reading blocklen!\n");
			return;
		}

		//clear out response, in case of error so it's not stale
		for(i=0; i<blocklen; i++) response[i] = ' ';

		char buffer[256];
		if(hand){
			//printf("getting realtime feedback, blocklen %d\n", blocklen);
			//lengthRead = wamif_get_realtime_feedback(wamif, buffer, blocklen);

			bhand_sendRealtimeCommand(hand, rawcmd, blocklen, buffer);
		}
		else{
			printf("connect the hand first!\n");
		}
		write(sockid, buffer, blocklen);
	}

}



int run_main(){
	int i, err;

	log_open("WAMtest.log");

	//Register the ctrl-c interrupt handler 
	signal(SIGINT, sigint_handler);
	
	//connect to the server
	printf("connecting to server\n");
	sockid = socket_client_connect(server_ip, server_port);
	if(!sockid){
		shut_down();
	}
	printf("connected to server\n");

	//get commands and run them
	while(1){
		parse_input();
	}

	alldone = 1;
	log_close();
}


//run everything within a Xenomai thread to avoid lock failures
//(the thread stops being real-time at the first I/O command--printf, getchar, etc.  But it still needs to be a Xenomai thread, not a normal pthread.)
int main() {
	wamif_init();

 	btrt_thread_create(&main_thd, "Main", 20, (void*)run_main, NULL);
	while(!alldone){
		usleep(100000);
	}

	return 1;
}



