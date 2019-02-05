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

PORT p;

void ex_program(int sig);

// init the port
int init(char *portname,int *baudrate){
  int err;
  err=serialOpen(&p,portname);
  if (err){
    printf("failed to open serial port %s\n",portname);
    return 0;
  }
  serialSetBaud(&p,*baudrate);
  return 1;
}

/***************************
COMMUNICATION SETUP COMMANDS
****************************/
// Communication Data Mode
// CD A setup communication for ascii output mode (default)
void setup_ascii_output(){
  serialWriteString(&p,"CD A\r");
}

// CD B setup communication for binary output mode
void setup_bin_output(){
  serialWriteString(&p,"CD B\r");
}

// Communication Checksum
// CD E enable a checksum at the end of binary communication
void enable_checksum(){
  serialWriteString(&p,"CD E\r");
}

// CD U disable sending checksum at end of binary communication (default)
void disable_checksum(){
  serialWriteString(&p,"CD U\r");
}

// Communication Data Type
// CD D setup communication for decimal strain gauge data output
void setup_dec_out(){
  serialWriteString(&p,"CD D\r");
}

// CD H setup communication for hexadecimal strain gauge data output
void setup_hex_out(){
  serialWriteString(&p,"CD H\r");
}

// CD R setup communication for resolved force/torque data output (default)
void setup_ft_out(){
  serialWriteString(&p,"CD R\r");
}

// Other Communication Setup Commands
// CB d Changes RS-232 serial baud rate (d = new baud rate)
void change_baud(int d){
  char* buffer;
  sprintf(buffer,"CB %d\r",d);
  serialWriteString(&p,buffer);
}

// CE b Enables error display on front panel during error condition (b=1; Default).
void set_err_display(int b){
  assert(b==0||b==1);
  char* buffer;
  sprintf(buffer,"CE %d\r",b);
  serialWriteString(&p,buffer);
}

// CS b Enables display of baud rate on front panel during power on (b=1; Default).
void set_baud_display(int b){
  assert(b==0||b==1);
  char* buffer;
  sprintf(buffer,"CS %d\r",b);
  serialWriteString(&p,buffer);
}

// CL b Enables Linefeed <LF> with <CR> (b=1; Default) or disable <LF> output (b=0).
void set_linefeed(int b){
  assert(b==0||b==1);
  char* buffer;
  sprintf(buffer,"CL %d\r",b);
  serialWriteString(&p,buffer);
}

// CV h Selects components of F/T values to be processed (Fr,Fx,Fy,Fz,Tr,Tx,Ty,Tz).
void select_proc_ft(int h){
  assert(h>=0&&h<=7);
  char* buffer;
  sprintf(buffer,"CV %d\r",h);
  serialWriteString(&p,buffer);
}
// CR d Sets the analog output voltage range to either ±5V or ±10V (d=5; Default)
void set_output_range(int d){
  assert(d==5||d==-5||d==10||d==-10);
  char* buffer;
  sprintf(buffer,"CR %d\r",d);
  serialWriteString(&p,buffer);
}

/***************************
QUERY DATA REQUESTS
****************************/
//Query F/T and Strain Gauge Data
//QR Query output of one Record of data in pre-selected communication setup.
void query_single(){
  serialWriteString(&p,"QR\r");
}

//^T Single record output with minimized handshaking; similar to QR.
void query_single_simple(){
  serialWriteString(&p,"\20");
}

//QS Query output of a Stream of data in pre-selected type and mode.
void query_stream(){
  serialWriteString(&p,"QS\r");
}

//QT Query output of one record of resultant axes Fr and Tr.
void query_axes(){
  serialWriteString(&p,"QT\r");
}

/***************************
SENSOR COMMANDS
****************************/
//Sensor Bias
//SB Performs a Sensor Bias. Stores bias reading in a 3-level buffer.
void perform_bias(){
  serialWriteString(&p,"SB\r");
}

//SU Performs a Sensor Unbias. Removes last bias command from buffer.
void undo_bias(){
  serialWriteString(&p,"SU\r");
}

//SZ Removes all previously stored biases from buffer.
void empty_biases(){
  serialWriteString(&p,"SZ\r");
}

//Sensor Peaks (see QP command)
//QP Query Peaks: show the maximum and minimum F/T values collected (see SP).
void query_peaks(){
  serialWriteString(&p,"QP\r");
}

//SP b Collects the max. and min. F/T values: start (b=1) or stop (b=0; Default).
void collect_peaks(int b){
  assert(b==0||b==1);
  char* buffer;
  sprintf(buffer,"SP %d\r",b);
  serialWriteString(&p,buffer);
}

//SC Clear max. and min. F/T values by loading high minimum and maximum values.
void clear_peaks(){
  serialWriteString(&p,"SC\r");
}

//Other Sensor Commands
//CF d Controls automatic SF optimization for RS-232 output. (d=0; Default).
void SF_optimization(int d){
  char* buffer;
  sprintf(buffer,"CF %d\r",d);
  serialWriteString(&p,buffer);
}

//SA d Performs a moving average of d sensor data samples (d=0; Default).
void sensor_avg(int d){
  char* buffer;
  sprintf(buffer,"SA %d\r",d);
  serialWriteString(&p,buffer);
}

//SF d Sensor sampling Frequency allows optimizing for faster output when using CF.
void sampling_freq(int d){
  char* buffer;
  sprintf(buffer,"SF %d\r",d);
  serialWriteString(&p,buffer);
}

//SM b Sensor Monitoring: disables (b=0) error message due to sensor error (saturation, disconnected transducer etc.) or enables error message (b=1; Default).
void sensor_monitoring(int b){
  assert(b==0||b==1);
  char* buffer;
  sprintf(buffer,"SM %d\r",b);
  serialWriteString(&p,buffer);
}

//SR b Sets sensor power protection error (b=1; Default)
void set_power_protection(int b){
  assert(b==0||b==1);
  char* buffer;
  sprintf(buffer,"SR %d\r",b);
  serialWriteString(&p,buffer);
}

/***************************
DISCRETE I/O COMMANDS
****************************/
//I/O Verification
//ID Reads and displays the state of all discrete input lines.
void get_input_state(){
  serialWriteString(&p,"ID\r");
}
//OD h Sets the state of all discrete outputs as specified by hexadecimal number h.
void set_input_state(int h){
  char* buffer;
  sprintf(buffer,"OD %x\r",h);
  serialWriteString(&p,buffer);
}

//Force Monitor Commands
//MC s Creates a force Monitor statement s.
void create_force_monitor_statement(char* s){
  char* buffer;  
  sprintf(buffer,"MC %s\r",s);
  serialWriteString(&p,buffer);
}

//MD d Deletes a force Monitor statement d.
void delete_force_monitor_statement(int d){
  char* buffer;
  sprintf(buffer,"MD %d\r",d);
  serialWriteString(&p,buffer);
}

//MH b Sets monitor handshake mode to b (b=0; Default â require handshaking)
void set_monitor_handshake(int b){
  char* buffer;
  sprintf(buffer,"MH %d\r",b);
  serialWriteString(&p,buffer);
}

//ML List all stored Force Monitor statements.
void list_all_monitor(){
  serialWriteString(&p,"ML\r");
}

//MV h Selects axes for resultants Fr and Tr.
void select_resultant_axes(int h){
  char* buffer;
  sprintf(buffer,"MV %x\r",h);
  serialWriteString(&p,buffer);
}

/***************************
TOOL FRAME COMMANDS
****************************/
//TD d Delete tool frame (d=1, 2 or 3).
void delete_tool_frame(int d){
  char* buffer;
  sprintf(buffer,"TD %d\r",d);
  serialWriteString(&p,buffer);
}
//TC d,s,x,y,z,R,P,Y 
//Constructs a new tool frame by changing the coordinate system (d=0..3;
//s=name; x, y, and z = translation along X, Y and Z axes; R, P, and Y = rotation
//about X, Y and Z axes).
void new_tool_frame(int d, char* s, int x, int y, int z, int R, int P, int Y){
  char* buffer;
  sprintf(buffer,"TC %d %s %d %d %d %d %d %d\r",d,s,x,y,z,R,P,Y);
  serialWriteString(&p,buffer);
}

//TF d Selects a calibration matrix from tool frame list (d=0, 1, 2 or 3).
void select_calibration_matrix(int d){
  char* buffer;
  sprintf(buffer,"TF %d\r",d);
  serialWriteString(&p,buffer);
}

//TL List available tool frames.
void list_tool_frames(){
  serialWriteString(&p,"TL\r");
}

//TT d Reports transducer serial number for tool frame d.
void get_frame_serial(int d){
  char* buffer;
  sprintf(buffer,"TT %d\r",d);
  serialWriteString(&p,buffer);
}

//TU Reports torque distance unit scaling.
void get_unit(){
  serialWriteString(&p,"TU\r");
}

/***************************
MISCELLANEOUS COMMANDS
****************************/
//^W Warm start. Performs a system reset and is identical to pressing the reset button.
void warm_start(){
  serialWriteString(&p,"\23\r");
}

//^Q, ^S XON and XOFF.
void XON(){
  serialWriteString(&p,"\17\r");
}

void XOFF(){
  serialWriteString(&p,"\19\r");
}

//ZC 0, "s" Creates a buffer of commands, s, that are executed at system power-up or reset.
void create_command_buffer(char* s){
  char* buffer;
  sprintf(buffer,"ZC %s\r",s);
  serialWriteString(&p,buffer);
}

//RS Save values from run memory into permanent memory.
void save_values(){
  serialWriteString(&p,"RS\r");
}

//RL Reload values from permanent memory into run memory.
void load_values(){
  serialWriteString(&p,"RL\r");
}

//% s Comment command. The s entry is ignored.

//HELP Lists a summary of available commands
void get_commands(){
  serialWriteString(&p,"HELP\r");
}

/*************************************
tests
 *************************************/

// get values from single query
// example:
// input: buf="--0, -413, -1080, -2480, -3532, 45, 5890"
// output: values={-413,-1080,-2480,-3532,45,5890}
void get_values(int* values, char* buf){
  int i,j,k;
  char tmp[100];
  i=0;
  k=0;
  while(buf[i]!=',') i++;
  while(k<5){
    j=0;
    i++;
    while (buf[i]!=',') {
      tmp[j]=buf[i];
      i++;
      j++;
    }
    values[k]=atoi(tmp);
    k++;
    //printf("%d\t",values[k]);
  }
  j=0;
  i++;
  while (buf[i]!='\n'){
    tmp[j]=buf[i];
    i++;
    j++;
  }
  values[k]=atoi(tmp);
  //printf("%d\n",values[k]);
}

//double r=.011; // radius of contact cylinder in meter
double r=11; //radius of contact cylinder in mm
double l=12; //(estimated!) length from zero point to center of sphere
double theta,cosT,sinT,Fr,z,Ffz,Fft,Fx,Fy,Fz,Tx,Ty,Tz,angle,zangle;
double ACTUAL_COUNTS_PER_FORCE = 20480.0;
double ACTUAL_COUNTS_PER_TORQUE = 4096.0;
double PI = 3.14159265358979323846264338327950288;
int values[6];
int flag;

//return the sign of the value
double sign(double val){
	if(val > 0) return 1.0;
	else if(val < 0) return -1.0;
	else return 0.0;
}

void process_values(){
  Fx=values[0]/ACTUAL_COUNTS_PER_FORCE;
  Fy=values[1]/ACTUAL_COUNTS_PER_FORCE;
  Fz=values[2]/ACTUAL_COUNTS_PER_FORCE;
  Tx=values[3]/ACTUAL_COUNTS_PER_TORQUE;
  Ty=values[4]/ACTUAL_COUNTS_PER_TORQUE;
  Tz=values[5]/ACTUAL_COUNTS_PER_TORQUE;
  //printf("%d %d %d %d %d %d\n",values[0],values[1],values[2],values[3],values[4],values[5]);
  //printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",Fx,Fy,Fx,Tx,Ty,Tz,ACTUAL_COUNTS_PER_FORCE,ACTUAL_COUNTS_PER_TORQUE);
  // contact point
  theta=atan2(Fy,Fx);
  cosT=cos(theta);
  sinT=sin(theta);
  if (theta<0) {
    angle=360+theta*180/PI;
  }else {
    angle=theta*180/PI;
  }

	double forcemag = sqrt(Fx*Fx + Fy*Fy + Fz*Fz);
	if(forcemag > .05){
		//angle from z-axis
		zangle = atan2(sqrt(Fx*Fx + Fy*Fy), -Fz);

		//z on the sphere
		if(zangle < 1.57){
			z = l + r*cos(zangle);
			Fr = forcemag;
		}

    //z on the cyl
		else{
			//z=(Tx+Ty-Fz*r*(cosT-sinT))/(Fx-Fy);
			double TxminusFz = Tx - Fz*r*cosT;
			double TyminusFz = Ty + Fz*r*sinT;
			z = sqrt(TxminusFz*TxminusFz + TyminusFz*TyminusFz) / sqrt(Fx*Fx + Fy*Fy);
			
			//z should be < 0
			if((Fx>0 && Fy>0 && Tx>0 && Ty<0) ||
				 (Fx<0 && Fy<0 && Tx<0 && Ty>0) ||
				 (Fx<0 && Fy>0 && Tx>0 && Ty>0) ||
				 (Fx>0 && Fy<0 && Tx<0 && Ty<0)) z = -z;
			
			// radial force
			Fr=sqrt(pow(Fx*cosT,2)+pow(Fy*sinT,2));
		}
  }
	else {
    z=0;
	}
  
  // vertical friction
  Ffz=Fz;
  
  // tangential friction
  Fft=Tz/r;
}

int serialReadLineFast(PORT *port, char *buf, int *lineLen, int term)
{
   int bytesRead;
   int err;

   *lineLen = 0;
   //printf("term:%d\n",term);
   while(1) {
      err = serialRead(port, buf, 1, &bytesRead);
      //printf("bytesRead:%d\n", bytesRead);
      *lineLen += bytesRead;
      buf[bytesRead] = '\0'; // Null terminate
      //printf("serialRead:%s\n", buf);
      if(*buf == term) // If termination character is found
         return(0);
      //printf(".");
      buf += bytesRead;
      usleep(500); // Sleep for .5ms
   }
}


// test single query
void test_single_query(){
  query_single();

  char buf[255];
  int lineLen,ret,i;
  int values[6];
  time_t start,end;
  time(&start);
  ret=serialReadLine(&p,buf,&lineLen,'>',5000);
  time(&end);
  printf("elapsed time %.2lf\nret %d\nbytes read %d\ncontent\n%s\n",difftime(end,start),ret,lineLen,buf);
  get_values(values,buf);
  for (i=0;i<6;i++){
    printf("%d\t",values[i]);
  }
  printf("\n");
}

// test stream query
void test_stream_query(){
  query_stream();

  char buf[255];
  int lineLen,i;
  int values[6];

  serialReadLineFast(&p,buf,&lineLen,'\n');
  while(1){
 
    serialReadLineFast(&p,buf,&lineLen,'\n');
     
    if (lineLen<57) {
      printf("ooooooops! corrupted input...exiting\n");
      break;
    }
    get_values(values,buf);
    for (i=0;i<6;i++){
      printf("%d\t",values[i]);
    }
    printf("\n");
  }
}

// test stream query in a new thread

void *update_readings(void *arg){
  char buf[255];
  int lineLen;

  serialReadLineFast(&p,buf,&lineLen,'\n');
  serialReadLineFast(&p,buf,&lineLen,'\n');
  serialReadLineFast(&p,buf,&lineLen,'\n');
  serialReadLineFast(&p,buf,&lineLen,'\n');
  serialReadLineFast(&p,buf,&lineLen,'\n');
  serialReadLineFast(&p,buf,&lineLen,'\n');
  serialReadLineFast(&p,buf,&lineLen,'\n');
  printf("%s\n",buf);
  while(1){
 
    serialReadLineFast(&p,buf,&lineLen,'\n');
     
    if (lineLen<57) {
      printf("ooooooops! corrupted input...exiting\n");
      printf("%s\n",buf);
      empty_biases();
      flag=1;
      break;
    }
    get_values(values,buf);
  }
  pthread_exit(NULL);
}


void test_thread_stream_query(){
  pthread_t thread;
  int rc;
  empty_biases();
  setup_ft_out();
  perform_bias();
  query_stream();
  printf("creating thread\n");
  flag=0;
  rc=pthread_create(&thread,NULL,update_readings,NULL);
  if (rc){
    printf("ERROR! return code from pthread_create() is %d\n",rc);
    exit(-1);
  }
  while(flag==0){
    /*
    int i;
    for (i=0;i<6;i++){
      printf("%d\t",values[i]);
    }
    printf("\n");
    */
    process_values();
    if (Fr>.03 ){
      printf("angle: %5.2f\t zangle: %5.2f\t Fr: %5.2f\t z: %5.2f\n",angle,zangle, Fr,z);
      printf("Fx %.2f Fy %.2f Fz %.2f Tx %.2f Ty %.2f Tz %.2f AF %.2f AT %.2f\n",Fx,Fy,Fz,Tx,Ty,Tz,ACTUAL_COUNTS_PER_FORCE,ACTUAL_COUNTS_PER_TORQUE);
    }
    usleep(1000000);
  }
  pthread_exit(NULL);
}

int main(int argc, char **argv){
  (void) signal(SIGINT, ex_program);
  char* portname="/dev/ttyUSB1";
  int baudrate=9600;
  if (init(portname,&baudrate)==0) return 0;
  //serialWriteString(&p,"qr\r");
  //test_single_query();
  //test_stream_query();
  test_thread_stream_query();
  serialClose(&p);
  return 1;
}

void ex_program(int sig) {
  empty_biases();
  empty_biases();
  printf("Wake up call ... !!! - Catched signal: %d ... !!\n", sig);

  (void) signal(SIGINT, SIG_DFL);
}
