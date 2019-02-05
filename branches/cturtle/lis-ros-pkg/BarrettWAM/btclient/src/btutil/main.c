/* ======================================================================== *
 *  Module ............. btutil
 *  File ............... main.c
 *  Creation Date ...... 15 Feb 2003
 *  Author ............. Brian Zenowich
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  Copyright (C) 2003-2008 Barrett Technology, Inc. <support@barrett.com>
 *                          625 Mount Auburn St
 *                          Cambridge, MA 02138, USA
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY BARRETT TECHNOLOGY, INC AND CONTRIBUTORS
 *  ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL BARRETT
 *  TECHNOLOGY, INC OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  The views and conclusions contained in the software and documentation
 *  are those of the authors and should not be interpreted as representing
 *  official policies, either expressed or implied, of Barrett Technology.
 *                                                                          *
 * ======================================================================== */

/** \file btutil.c
 
Puck utilities:
 
Bus enumeration - Prints out what is alive
Bus Enumeration (debug)- Broadcast request; Print CANid,Serial,Status 
 
Bus enumeration and puck status - 
  Prints out all interesting puck values.
  
Puck Find motor offsets
 
Puck - Load WAM enumeration information.
 
Puck - Load new firmware
 
*/

/** Usage:
 
btutil [-c configfile] command [detail]
 
where command is:
 
  enum - List what is on the can bus and what their state is.
  stat - List paramers of interest
    all - dump all parameters
    init - dump parameters that are important to initial startup
    
  moffst # - Find motor offset for puck id #
  writefirmware # filename - write the specified firmware file
  writewaminfo # - write default wam enumeration info to puck id #
  copyparameters filename - read all parameters and store them to a file
  writeparameters filename - write all parameters stored in a file  
  
 
 
*/


//#define toupper(c)      ( ((c >= 'a') && (c <= 'z')) ? c - ('a' - 'A') : c )

/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#include <syslog.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <curses.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>
#include <sys/io.h>

#ifdef XENOMAI
#include <native/task.h>
#include <native/timer.h>
#else
#include <rtai_lxrt.h>
#include <rtai_sem.h>
#endif

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#include "btos.h"
#include "btmath.h"
#include "btcan.h"
#include "btserial.h"

#include "main.h"

enum{SCREEN_MAIN, SCREEN_HELP};
#define MAX_WATCH (20)
#define SET_SLEEP (100)

int screen = SCREEN_MAIN;
int entryLine;
int done = FALSE;
PORT p;
int id = 1;
int arrow = 2;
int termX = 0, termY = 20;
int enumX = 39, enumY = 2;
int watchX = 39, watchY = 20;
int curses = FALSE;

btrt_thread_struct disp_thd;
btrt_mutex disp_mutex;
int startDone = FALSE;
btrt_thread_struct  StartupThread;

struct watchStruct watch[MAX_WATCH];
struct {int a; char **b;} args;

/*==============================*
 * PRIVATE Function Prototypes  *
 *==============================*/
void RenderMAIN_SCREEN(void);
void ProcessInput(int c);
void DisplayThread(void);
void clearScreen(void);
void finish_entry(void);
void start_entry(void);
void init_ncurses(void);
void handleMenu(int argc, char **argv);

void enumeratePucks(void *data);
void activePuck(void *data);
void firmwarePuck(void *data); int firmwareDL(int id, char *fn);
void terminalMode(void *data);
void watchProperty(void *data);
void setDefaults(void *data); void paramDefaults(int newID,int targID);
void hallCheck(void *data);
void tensionCable(void *data);
void findOffset(void *data);
void exitProgram(void *data);

struct fcnStruct fcn[] = { 
   activePuck, "Change active puck (1-9)",
   enumeratePucks, "Enumerate bus",
   firmwarePuck, "Download firmware (tek/out)",
   terminalMode, "Go to terminal mode",
   watchProperty, "Add property to watchlist",
   setDefaults, "Set default properties",
   //hallCheck, "Hall feedback check",
   //tensionCable, "Tension cable",
   //findOffset, "Find motor offset (Optic/Magnetic)",
   exitProgram, "Exit program",
   
   NULL, ""
};

/*==============================*
 * Functions                    *
 *==============================*/

int setPropertySlow(int bus, int id, int property, int verify, long value){
   setProperty(bus, id, property, verify, value);
   usleep(SET_SLEEP);
}

void enumeratePucks(void *data){
   int id;
   int x;
   int y;
   long status[MAX_NODES];
   long monVers, mainVers;
   
   x = enumX; y = enumY;
   
   clearScreen();
   getBusStatus(0, status);
   usleep(500000);
   mvprintw(y++, x, "PUCK  MON MAIN");
   mvprintw(y++, x, "---- ---- ----");
   for(id = 0; id < MAX_NODES; id++) {
      monVers = mainVers = 0;
      switch(status[id]){
         case -1: // Offline
         break;
         case 0: // Reset
         getProperty(0, id, VERS, &monVers);
         wakePuck(0, id);
         getProperty(0, id, VERS, &mainVers);
         setPropertySlow(0, id, STAT, FALSE, 0); // Reset the puck
         mvprintw(y++, x, "%3d %4ld %4ld", id, monVers, mainVers);
         break;
         case 2: // Ready
         getProperty(0, id, VERS, &mainVers);
         if(id != SAFETY_MODULE){
            setPropertySlow(0, id, STAT, FALSE, 0); // Reset the puck
            usleep(500000);
            getProperty(0, id, VERS, &monVers);
         }
         mvprintw(y++, x, "%3d %4ld %4ld", id, monVers, mainVers);
         break;
         default: // Strange
         break;
      }
      
   }
}
 
void activePuck(void *data){
   start_entry();
   addstr("Change active puck to: ");
   refresh();
   scanw("%d\n",  &id);
   finish_entry();
}

void firmwarePuck(void *data){
   char fn[64];
   
   start_entry();
   addstr("Download firmware file: ");
   refresh();
   scanw("%s\n", fn);
   finish_entry();
   
   firmwareDL(id, fn);
   usleep(2000000);
   enumeratePucks(NULL);
}

void terminalMode(void *data){
   int prop;
   long val;
   
   start_entry();
   addstr("Which property: ");
   refresh();
   scanw("%d\n", &prop);
   finish_entry();
   
   start_entry();
   addstr("What value: ");
   refresh();
   scanw("%ld\n", &val);
   finish_entry();
   
   setPropertySlow(0, id, prop, FALSE, val);
   
}

void watchProperty(void *data){
   char prop[64];
   int i;
   
   start_entry();
   addstr("Which property (or \"clear\"): ");
   refresh();
   scanw("%s\n", prop);
   finish_entry();
   
   if(!strcmp(prop, "clear")){
      for(i = 0; i < MAX_WATCH; i++){
         watch[i].puckID = 0;
         watchY = 20;
      }
      clearScreen();
      return;
   }
   
   i = 0;
   while(i < MAX_WATCH){
      if(!watch[i].puckID){
         watch[i].puckID = id;
         watch[i].prop = atol(prop);
         if(i > 3)
            --watchY;
         return;
      }
      ++i;
   }
   return;
   
}

void setDefaults(void *data){
   int altID;
   char questionTxt[256];
   
   sprintf(questionTxt, "Set defaults for puck %d to typical defaults for puck [%d]: ", id, id);
   start_entry();
   addstr(questionTxt);
   refresh();
   scanw("%d\n",  &altID);
   finish_entry();
   
   paramDefaults(id, altID);
   
   mvprintw(entryLine, 1, "Done setting defaults                                          ");
}

void checkHalls(int arg1){
   long        vers, reply, cts, startPos;
   
   wakePuck(0,arg1);
   setPropertySlow(0, arg1, ADDR, 0, 28900); // GPBDAT
   printf("\nPlease turn motor %d approx one revolution...\n");
   vers = -1;
   getProperty(0, arg1, AP, &startPos);
   getProperty(0, arg1, CTS, &cts);
   do{
      getProperty(0, arg1, VALUE, &reply);
      reply = (reply >> 8) & 0x00000007;
      if(reply != vers){
         printf("%d, ", vers = reply);
	 fflush(stdout);
      }
      getProperty(0, arg1, AP, &reply);
      if(abs(reply-startPos) > cts)
         break;
      usleep(10000);
   }while(1);
   printf("\nDone.\n");
}

void hallCheck(void *data){
   checkHalls(id);
}

cableTension(int motor){
   int cmd;
   int tens;
   long startPos, endPos;
   double nm = 0.0;
   
   cmd = T;
   // Set the default tension
   // Comment out this section to enter your own values
   switch(motor){
	   case 1: case 2: case 3: 
		   nm = 0.9;
	break;
	   case 4:
		nm = 0.8;
		break;
	   case 5: case 6:
	nm = 0.3;
	break;
   }

   //printf("\nTension Cable\nTension which motor: ");
   //scanf("%d", &motor);
   setPropertySlow(0,GROUPID(0),cmd,FALSE,0);
   wakePuck(0,GROUPID(0));
   if(nm == 0.0){
   printf("\nHow much tension (Nm): ");
   scanf("%lf", &nm);
   }
   switch(motor){
	   case 1: case 2: case 3: case 4:
		   tens = nm * 2700;
	break;
	   case 5: case 6:
	tens = nm * 5000;
	break;
   }
   
   setPropertySlow(0,GROUPID(0),MODE,FALSE,MODE_TORQUE);
   printf("\nPlease move cable to shaft end, then press <Enter>");
   mygetch();
   mygetch();
   if(motor == 6) setPropertySlow(0,5,TENSION,FALSE,1);
   else setPropertySlow(0,motor,TENSION,FALSE,1);
   setPropertySlow(0,motor,cmd,FALSE,500);
   printf("\nPlease rotate shaft until tensioner engages, "
          "then press <Enter>");
   mygetch();
   if(motor == 6) setPropertySlow(0,5,TENSION,FALSE,0);
   else setPropertySlow(0,motor,TENSION,FALSE,0);
   getProperty(0,motor,AP,&startPos);
   setPropertySlow(0,motor,cmd,FALSE,tens);
   usleep(1000000);
   getProperty(0,motor,AP,&endPos);
   setPropertySlow(0,motor,cmd,FALSE,0);
   printf("\nTook up %ld encoder cts of cable", abs(startPos-endPos));
   printf("\nPlease work the tension through the cable, "
          "then press <Enter>");
   mygetch();
   setPropertySlow(0,GROUPID(0),MODE,FALSE,MODE_IDLE);
}

void tensionCable(void *data){
   cableTension(id);
}

void setMofst(int newID);

void findOffset(void *data){
   setMofst(id);
}

void exitProgram(void *data){
   done = 1;
}


/* Initialize the ncurses screen library */
void init_ncurses(void)
{
   initscr();
   cbreak();
   noecho();
   timeout(0);
   clear();
}

void ProcessWatch(void){
   int i;
   long value;
   
   for(i = 0; i < MAX_WATCH; i++){
      if(watch[i].puckID){
         getProperty(0, watch[i].puckID, watch[i].prop, &value);
         mvprintw(watchY+i, watchX + 2, "ID=%d   PROP=%d   VAL=%ld            ", 
            watch[i].puckID, watch[i].prop, value);
      }
   }
}

void Startup(void *thd){
   int argc;
   char **argv;
   char            chr;
   int             err;
   int pID;
   int i;
   long status[MAX_NODES];
   
   argc = args.a;
   argv = args.b;
   
   //printf("a=%d, s=%s\n", argc, argv[1]);
   
   /* Initialize CAN */
   if(err = initCAN(0, 0)) {
      syslog(LOG_ERR, "initCAN returned err=%d", err);
   }
   
   /* Open serial port */
   if(err = serialOpen(&p, "/dev/ttyS0")) {
      syslog(LOG_ERR, "Error opening serial port: %d", err);
   }
   serialSetBaud(&p, 9600);
   
   if(argc > 1){
      getBusStatus(0, status);
      handleMenu(argc, argv);
   } else {
      /* Initialize the ncurses screen library */
      init_ncurses();
      atexit((void*)endwin);
      curses = TRUE;
      
      /* Initialize the display mutex */
      test_and_log(
         btrt_mutex_init(&(disp_mutex)),
         "Could not initialize mutex for displays.");
      
      /* Spin off the display thread */
      btrt_thread_create(&disp_thd, "DISP", 10, (void*)DisplayThread, NULL);
      //btthread_create(&disp_thd,0,(void*)DisplayThread,NULL);
   
      for(i = 0; i < MAX_WATCH; i++)
         watch[i].puckID = 0;
      //enumeratePucks(NULL);
      getBusStatus(0, status);
      
      while (!done) {
         /* Check and handle user keypress */
         if ((chr = getch()) != ERR)
            ProcessInput(chr);
         ProcessWatch();
         
         usleep(100000); // Sleep for 0.1s
      }
   }
   
   freeCAN(0); 
   startDone = 1;
   
   btrt_thread_exit((btrt_thread_struct*)thd);
}

void Cleanup(){
	/* Exit the CANbus thread gracefully */
	StartupThread.done = 1;
	exit(0);
}


int main( int argc, char **argv )
{
   args.a = argc;
   args.b = argv;
   
   mlockall(MCL_CURRENT | MCL_FUTURE);
   
   /* Initialize syslogd */
   openlog("PUCK", LOG_CONS | LOG_NDELAY, LOG_USER);
   syslog(LOG_ERR, "...Starting Puck Utility Program...");

   /* Register the ctrl-c interrupt handler */
   signal(SIGINT, Cleanup);
   
   /* RT task for setup of CAN Bus */
   btrt_thread_create(&StartupThread,"StTT", 45, (void*)Startup, NULL);
   while(!startDone)
      usleep(10000);
}

/** Spins in a loop, updating the screen.
    Runs as its own thread, updates the screen.
*/
void DisplayThread()
{
   int cnt,err;
   
   clear();
   refresh();
   while (!done) {
      test_and_log(
         btrt_mutex_lock(&(disp_mutex)),"Display mutex failed");
      switch(screen) {
      case SCREEN_MAIN:
         RenderMAIN_SCREEN();
         break;
      }
      btrt_mutex_unlock(&(disp_mutex));
      
      usleep(100000);
   }

}

/** Locks the display mutex.
    Allows the user to enter on-screen data without fear of display corruption.
*/
void start_entry()
{
   int err;
   test_and_log(
      btrt_mutex_lock(&(disp_mutex)),"Display mutex failed");
   move(entryLine, 1);
   echo();
   timeout(-1);
}

/** Unlocks the display mutex.
    Allows the computer to resume automatically updating the screen.
*/
void finish_entry()
{
   noecho();
   timeout(0);
   move(entryLine, 1);
   addstr("                                                                              ");
   refresh();
   btrt_mutex_unlock( &(disp_mutex) );
}

/** Draw the main information screen.
    Dynamically draw the puck information on the screen.
*/
void RenderMAIN_SCREEN()
{
   int x, y, i;

   /***** Display the interface text *****/
   y = 0;
   x = 5;

   mvprintw(y, 0, "Barrett Technology - Puck Utility  *** Active puck = %d ***  ", id); y+=2;
   
   i = 0;
   while(fcn[i].f){
      mvprintw(y++, x, fcn[i].title);
      i++;
   }
   
   mvprintw(arrow, 1, "-->");
   
   y++;
   entryLine = y;
   
   mvprintw(termY-2, termX, " __________");
   mvprintw(termY-1, termX, "| TERMINAL \\____________________________");
   
   mvprintw(watchY-2, watchX, " __________");
   mvprintw(watchY-1, watchX, "| WATCH    \\____________________________");
   
   mvprintw(entryLine, 0, "");
   refresh();
}

/** Process user input.
    Handles the user's keypress, and performs the function desired by the user.
*/
void ProcessInput(int c) //{{{ Takes last keypress and performs appropriate action.
{
   int cnt,elapsed = 0;
   double ftmp,tacc,tvel;
   int dtmp,status;

   char fn[250],chr;
   int ret;
   int done1;
   btreal zPos;

   switch (c)
   {

   case 'x'://eXit
   case  'X'://eXit
      done = 1;
      break;
   case '1': id = 1; break;
   case '2': id = 2; break;
   case '3': id = 3; break;
   case '4': id = 4; break;
   case '5': id = 5; break;
   case '6': id = 6; break;
   case '7': id = 7; break;
   case '8': id = 8; break;
   case '9': id = 9; break;
   
   case 10: // <Enter Key>
      (fcn[arrow-2].f)(NULL);
      break;
   
   case '_'://Refresh display
      clearScreen();
      break;
   case 27://Handle and discard extended keyboard characters (like arrows)
      if ((chr = getch()) != ERR) {
         if (chr == 91) {
            if ((chr = getch()) != ERR) {
               if (chr == 65) // Up arrow
               {
                  mvprintw(arrow, 1, "   ");
                  arrow--;
                  if(arrow < 2)
                     arrow = entryLine - 2;
               }
               else if(chr == 66) // Down arrow
               {
                  mvprintw(arrow, 1, "   ");
                  arrow++;
                  if(arrow > entryLine - 2)
                     arrow = 2;
               }
               else if (chr == 67) //Right arrow
               {
                 
               }
               else if (chr == 68) //Left arrow
               {
                  
               } 
               else {
                  while(getch()!=ERR) {
                     // Do nothing
                  }
                  syslog(LOG_ERR,"Caught unknown keyhit 27-91-%d",chr);
                  //mvprintw(20,1,"Caught unknown keyhit 27-91-%d",chr);
               }
            }
         } else {
            while(getch()!=ERR) {
               // Do nothing
            }
            syslog(LOG_ERR,"Caught unknown keyhit 27-%d",chr);
            //mvprintw(20,1,"Caught unknown keyhit 27-%d",chr);
         }
      }
      break;

   default:
      while(getch()!=ERR) {
         // Do nothing
      }
      syslog(LOG_ERR,"Caught unknown keyhit %d",c);

      break;
   }
}

void clearScreen(void)
{
   btrt_mutex_lock(&(disp_mutex));
   clear();
   btrt_mutex_unlock(&(disp_mutex));
}

int mygetch( )
{
   /* Handles to old and new termios structures */
   struct termios oldt, newt;
   int ch;

   /* store current termios to restore back later */
   tcgetattr( STDIN_FILENO, &oldt );
   newt = oldt;

   /* set to canonical. turn off echo */
   newt.c_lflag &= ~( ICANON | ECHO );
   tcsetattr( STDIN_FILENO, TCSANOW, &newt );

   /* Read the character */
   ch = getchar();

   /* Reset terminal */
   tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
   return ch;
}

int firmwareDL(int id, char *fn)
{
   FILE *fp;
   int in_id;
   int in_len;
   unsigned char in_data[8];
   unsigned char out_Data[8];
   int i;
   int cnt = 1;
   char line[100];
   int rcpt;
   int lineTotal, lineCt;

   // Open the file
   if((fp=fopen(fn, "r"))==NULL) {
      return(1);
   }
   
   // Count the lines
   lineTotal = 0;
   while(fgets(line, 99, fp) != NULL)
      ++lineTotal;
   
   // Reset the file pointer
   fclose(fp);
   if((fp=fopen(fn, "r"))==NULL) {
      return(1);
   }
   
   setPropertySlow(0, id, STAT, FALSE, 0L); // Reset
   usleep(1000000); // Wait a sec
   setPropertySlow(0, id, VERS, FALSE, 0x000000AA);
   // For each line in the file
   //sendData[0] = 0x80 | VERS;
   //sendData[1] = 0x00;
   lineCt = 0;
   if(!curses) printf("\n\n");
   while(fgets(line, 99, fp) != NULL) {
      i = 1;
      ++lineCt;
      //printf("%s", line);
      if(curses){
      mvprintw(entryLine, 1, "Download progress: %d%%", (int)(100.0 * lineCt / lineTotal));

      }else{
	      printf("\rDownload progress: %d%%   ", (int)(100.0 * lineCt /
				      lineTotal));
	      fflush(stdout);
      }
      while(line[i] >= '0') {
         // Wait for n "Get VERS"
         for(rcpt = 0; rcpt < cnt; rcpt++) {
            while(canReadMsg(0, &in_id, &in_len, in_data, 1))
               usleep(100);
         }
         // Send the byte
         out_Data[0] = line[i];
         canSendMsg(0, id, 1, out_Data, 1);
         ++i;
      }
   }
   fclose(fp);
   if(curses)
   mvprintw(entryLine, 1, "Download complete!               ");
   else
	   printf("\nDownload complete!   ");
   
   return(0);
}

void checkTemp(int puckID){
   long dat;
   int err = 0;
   
   wakePuck(0, puckID);
   getProperty(0, puckID, TEMP, &dat);
   printf("\nTEMP = %ld", dat);
   if(dat < 15 || dat > 60){
      printf(" -- FAIL");
      err++;
   }
   /*
   getProperty(0, puckID, THERM, &dat);
   printf("\nTHERM = %ld", dat);
   if(dat < 15 || dat > 60){
      printf(" -- FAIL");
      err++;
   }
   */
   
   printf("\nDone. ");
   printf("\n");
}

void checkAutotensioner(int puckID){
   int i, cycles = 8;
   
   wakePuck(0, puckID);
   
   while(cycles--){
      setPropertySlow(0, puckID, FET1, 0, 1);
      usleep(250000);
      setPropertySlow(0, puckID, FET1, 0, 0);
      usleep(250000);
   }
   
   printf("\nDone. ");
   printf("\n");
}

void paramDefaults(int newID,int targID)
{
   int i;
   long vers, role;
   
   wakePuck(0,newID);
   //getProperty(0, newID, VERS, &vers);
   getProperty(0, newID, ROLE, &role);
   
   switch(role & 0x00FF){
      case ROLE_TATER:
      for(i = 0; taterDefs[i].key; i++){
         setPropertySlow(0, newID, *taterDefs[i].key, 0, taterDefs[i].val);
         
      }
      if(role & 0x0100)
         setPropertySlow(0, newID, CTS, 0, 4096); 
      
      if(targID <= 4) { //4DOF
         setPropertySlow(0,newID,IKCOR,0,1638); 
         setPropertySlow(0,newID,IKP,0,8192); 
         setPropertySlow(0,newID,IKI,0,3276); 
         setPropertySlow(0,newID,IPNM,0,2700); //2755);
         //setPropertySlow(0,newID,IPNM,0,2562);//2755);
         setPropertySlow(0,newID,POLES,0,12); 
         setPropertySlow(0,newID,GRPA,0,0); 
         setPropertySlow(0,newID,GRPB,0,1); 
         setPropertySlow(0,newID,GRPC,0,4); 
   
      } else if(targID <= 7) { //Wrist
         setPropertySlow(0,newID,IKCOR,0,819); 
         setPropertySlow(0,newID,IKP,0,4096); 
         setPropertySlow(0,newID,IKI,0,819); 
         setPropertySlow(0,newID,GRPA,0,0); 
         setPropertySlow(0,newID,GRPB,0,2); 
         setPropertySlow(0,newID,GRPC,0,5); 
         if(targID != 7) {
            setPropertySlow(0,newID,IPNM,0,6500); 
            //setPropertySlow(0,newID,IPNM,0,4961);
            setPropertySlow(0,newID,POLES,0,8); 
         } else {
            setPropertySlow(0,newID,IPNM,0,17474); 
            setPropertySlow(0,newID,POLES,0,6); 
         }
      }
      
      setPropertySlow(0,newID,JIDX,0,targID); 
      setPropertySlow(0,newID,PIDX,0,((targID-1)%4)+1); 
      break;
      
      case ROLE_SAFETY:
      for(i = 0; safetyDefs[i].key; i++){
         setPropertySlow(0, newID, *safetyDefs[i].key, 0, safetyDefs[i].val);
         
      }
      
      setPropertySlow(0, newID, SAFE, 0, 4); 
      setPropertySlow(0, newID, SAFE, 0, 5); 
      usleep(1000000); // Wait a sec
      setPropertySlow(0, newID, FIND, 0, VBUS); 
      usleep(1000000); // Wait a sec
      setPropertySlow(0, newID, SAFE, 0, 0); 
      
      break;
      
      case ROLE_WRAPTOR:
      for(i = 0; wraptorDefs[i].key; i++){
         setPropertySlow(0, newID, *wraptorDefs[i].key, 0, wraptorDefs[i].val);
         
      }
      
      if(targID < 4){
         // Set inner link parameters
         
      }else if(targID == 4){
         // Set spread puck parameters: KP=1500 KD=KI=0, ACCEL=10, MV=20, DP=18500, CT=37000, HOLD=1
         
      }else if(targID > 4){
         // Set outer link parameters
         
      }
      break;
      
      default:
      
      break;
      
   }
   
   setPropertySlow(0,newID,SAVE,0,-1); // Save All
   
   /*
   
    else if(targID <= 8) {//Gimbals
      setPropertySlow(0,newID,CTS,0,1);
      setPropertySlow(0,newID,OFFSET1,0,-11447);
      setPropertySlow(0,newID,OFFSET2,0,-19834);
      setPropertySlow(0,newID,OFFSET3,0,-12606);
      setPropertySlow(0,newID,GAIN1,0,10981);
      setPropertySlow(0,newID,GAIN2,0,27566);
      setPropertySlow(0,newID,GAIN3,0,26782);
      setPropertySlow(0,newID,GRPA,0,0);
      setPropertySlow(0,newID,GRPB,0,2);
      setPropertySlow(0,newID,GRPC,0,5);
   }
*/

}

void changeID(int oldID, int newID, int role)
{
   wakePuck(0,oldID);
   //syslog(LOG_ERR, "_LOCK=%d", _LOCK);
   setPropertySlow(0, oldID, _LOCK, 0, 18384); 
   setPropertySlow(0, oldID, _LOCK, 0, 23); 
   setPropertySlow(0, oldID, _LOCK, 0, 3145); 
   setPropertySlow(0, oldID, _LOCK, 0, 1024); 
   setPropertySlow(0, oldID, _LOCK, 0, 1); 
   if(role >= 0){
      setPropertySlow(0, oldID, ROLE, 0, role); 
      setPropertySlow(0, oldID, SAVE, 0, ROLE); usleep(5000000);
   }
   setPropertySlow(0, oldID, ID, 0, newID); 
   setPropertySlow(0, oldID, SAVE, 0, ID); usleep(5000000);
   
   setPropertySlow(0, oldID, STAT, 0, STATUS_RESET);
}

void setMofst(int newID)
{
   long dat, vers;
   int dummy, i, samples = 16;
   
	long	max, min;
	double	sumX, sumX2, mean, stdev;
   int err = 0;
   
   wakePuck(0,newID);
   getProperty(0,newID,VERS,&vers);

   getProperty(0,newID,IOFST,&dat);
   printf("\nThe old IOFST was: %d",dat);
   
   // Get a valid IOFST
   #define IOFST_MIN (1950)
   #define IOFST_MAX (2160)
   #define IOFST_STDEV (3)
   
   // Collect stats
	sumX = sumX2 = 0;
	max = -2E9;
	min = +2E9;
	for(i = 0; i < samples; i++){
		setPropertySlow(0,newID,FIND,0,IOFST);
      getProperty(0,newID,IOFST,&dat);
		if(dat > max) max = dat;
		if(dat < min) min = dat;
		sumX += dat;
		sumX2 += dat * dat;
      usleep(1000000/16);
	}
	mean = 1.0 * sumX / samples;
	stdev = sqrt((1.0 * samples * sumX2 - sumX * sumX) / (samples * samples - samples));
   printf("\nMIN IOFST = %ld", min);
   if(min < IOFST_MIN){
      printf(" -- FAIL");
      ++err;
   } 
   printf("\nMAX IOFST = %ld", max);
   if(max > IOFST_MAX){
      printf(" -- FAIL");
      ++err;
   }
	printf("\nMEAN IOFST = %.2f", mean);
	printf("\nSTDEV IOFST = %.2f", stdev);
   if(stdev > IOFST_STDEV){
      printf(" -- FAIL");
      ++err;
   }
   setPropertySlow(0, newID, IOFST, 0, (long)mean);
   printf("\nThe new IOFST is:%d\n",(int)mean);
   
   if(!err){
      setPropertySlow(0,newID,MODE,0,MODE_TORQUE);
      getProperty(0,newID,MOFST,&dat);
      printf("\nThe old MOFST was:%d\n",dat);
   
      if(vers <= 39){
         setPropertySlow(0,newID,ADDR,0,32971);
         setPropertySlow(0,newID,VALUE,0,1);
      }else{
         setPropertySlow(0,newID,FIND,0,MOFST);
      }
      //printf("\nPress enter when the index pulse is found: ");
      //mygetch();
      //mygetch();
      printf("\nPlease wait (10 sec)...\n");
      usleep(10000000); // Sleep for 10 sec
      if(vers <= 39){
         setPropertySlow(0,newID,ADDR,0,32970);
         getProperty(0,newID,VALUE,&dat);
      }else{
         getProperty(0,newID,MOFST,&dat);
      }
      printf("\nThe new MOFST is:%d\n",dat);
      setPropertySlow(0,newID,MODE,0,MODE_IDLE);
      if(vers <= 39){
         setPropertySlow(0,newID,MOFST,0,dat);
         setPropertySlow(0,newID,SAVE,0,MOFST);
      }
   }
   
   printf("\nDone. ");
   printf("\n");
}

getParams(int newID)
{
   long reply;
   int cnt;

   printf("\n...Puck %d...\n",newID);
   wakePuck(0,newID);
   getProperty(0,newID,SN,&reply);
   printf("Serial Number = %ld\n",reply);
   getProperty(0,newID,VERS,&reply);
   printf("VERS = %ld\n",reply);
   getProperty(0,newID,ROLE,&reply);
   printf("ROLE = %ld\n",reply);
   getProperty(0,newID,ACCEL,&reply);
   printf("ACCEL = %ld\n",reply);
   getProperty(0,newID,AP,&reply);
   printf("AP = %ld\n",reply);
   getProperty(0,newID,HALLS,&reply);
   printf("HALLS = %ld\n",reply);
   getProperty(0,newID,HALLH,&reply);
   printf("HALLH = %ld\n",reply);
   getProperty(0,newID,CT,&reply);
   printf("CT = %ld\n",reply);
   getProperty(0,newID,CTS,&reply);
   printf("CTS = %ld\n",reply);
   getProperty(0,newID,DP,&reply);
   printf("DP = %ld\n",reply);
   getProperty(0,newID,EN,&reply);
   printf("EN = %ld\n",reply);
   getProperty(0,newID,GAIN1,&reply);
   printf("GAIN1 = %ld\n",reply);
   getProperty(0,newID,GAIN2,&reply);
   printf("GAIN2 = %ld\n",reply);
   getProperty(0,newID,GAIN3,&reply);
   printf("GAIN3 = %ld\n",reply);
   getProperty(0,newID,IKCOR,&reply);
   printf("IKCOR = %ld\n",reply);
   getProperty(0,newID,IKP,&reply);
   printf("IKP = %ld\n",reply);
   getProperty(0,newID,IKI,&reply);
   printf("IKI = %ld\n",reply);
   getProperty(0,newID,IPNM,&reply);
   printf("IPNM = %ld\n",reply);
   getProperty(0,newID,MT,&reply);
   printf("MT = %ld\n",reply);
   getProperty(0,newID,MOFST,&reply);
   printf("MOFST = %ld\n",reply);
   getProperty(0,newID,IOFST,&reply); 
   printf("IOFST = %ld\n",reply);
   getProperty(0,newID,OFFSET1,&reply);
   printf("OFFSET1 = %ld\n",reply);
   getProperty(0,newID,OFFSET2,&reply);
   printf("OFFSET2 = %ld\n",reply);
   getProperty(0,newID,OFFSET3,&reply);
   printf("OFFSET3 = %ld\n",reply);
   getProperty(0,newID,JIDX,&reply);
   printf("JIDX = %ld\n",reply);
   getProperty(0,newID,PIDX,&reply);
   printf("PIDX = %ld\n",reply);
   getProperty(0,newID,PTEMP,&reply);
   printf("PTEMP = %ld\n",reply);
   getProperty(0,newID,POLES,&reply); 
   printf("POLES = %ld\n",reply);

   getProperty(0,newID,GRPA,&reply); 
   printf("GRPA = %ld\n",reply);
   getProperty(0,newID,GRPB,&reply); 
   printf("GRPB = %ld\n",reply);
   getProperty(0,newID,GRPC,&reply); 
   printf("GRPC = %ld\n",reply);

   getProperty(0,newID,40,&reply); 
   printf("40 = %ld\n",reply);
   getProperty(0,newID,41,&reply); 
   printf("41 = %ld\n",reply);
}

void handleMenu(int argc, char **argv)
{
   long        status[MAX_NODES];
   int         i;
   int         arg1, arg2, arg3;
   long        vers;
   char        *c;
   char        fn[255];
   
   c = argv[1];
   while(*c == '-') c++;
   *c = toupper(*c);
   
   switch(*c) {
   case 'H':
      printf("\n\nCheck hall feedback on motor: ");
      if(argc >= 3) {
         arg1 = atol(argv[2]);
         printf("%d", arg1);
      }else
         scanf("%d", &arg1);
      
      checkHalls(arg1);
   break;
   case 'I':
      printf("\n\nChange puck ID: ");
      if(argc >= 3){
         arg1 = atol(argv[2]);
         printf("%d", arg1);
      }else{
         scanf("%d", &arg1);
      }
      
      if(argc >= 5){
         arg2 = atol(argv[4]);
         printf("\nNew ID: %d", arg2);
      }else{
         printf("\nNew ID: ");
         scanf("%d", &arg2);
      } 
      
      arg3 = -1;
      if(argc >= 7){
         arg3 = atol(argv[6]);
         printf("\nSet ROLE: %d", arg3);
      }
      changeID(arg1, arg2, arg3);
      break;

   case 'C':
      //calibrateGimbals();
      break;
   case 'E':
      printf("\n\nCAN bus enumeration (status)\n");
      getBusStatus(0, status);
      for(i = 0; i < MAX_NODES; i++) {
         if(status[i]>=0) {
            getProperty(0,i,VERS,&vers);
            printf("\nNode %2d: %15s vers=%2d", i,
                   statusTxt[status[i]+1], vers);
         }
      }
      printf("\n");
      break;
   case 'F':
      printf("\n\nFind offsets for puck: ");
      if(argc >= 3){
         arg1 = atol(argv[2]);
         printf("%d", arg1);
      }else{
         scanf("%d", &arg1);
      }

      setMofst(arg1);
      break;
   case 'P':
      printf("\n\nSet defaults for puck ID: ");
      if(argc >= 3){
         arg1 = atol(argv[2]);
         printf("%d", arg1);
      }else{
         scanf("%d", &arg1);
      }
      
      if(argc >= 5){
         arg2 = atol(argv[4]);
         printf("\nLike ID: %d", arg2);
      }else{
         printf("\nLike ID: ");
         scanf("%d", &arg2);
      } 
      paramDefaults(arg1,arg2);
      break;
   
   case 'G':
      printf("\n\nGet params from puck ID: ");
      if(argc >= 3){
         arg1 = atol(argv[2]);
         printf("%d", arg1);
      }else{
         scanf("%d", &arg1);
      }
      getParams(arg1);
      break;
   case 'D':
      printf("\n\nDownload firmware to puck ID: "); 
      if(argc >= 3){
         arg1 = atol(argv[2]);
         printf("%d", arg1);
      }else{
         scanf("%d", &arg1);
      }
      if(argc >= 5){
         //arg2 = atol(argv[4]);
         printf("\nFile: %s", argv[4]);
         firmwareDL(arg1, argv[4]);
      }else{
         printf("\nFile: ");
         scanf("%s", fn);
         firmwareDL(arg1, fn);
      } 
      
      break;
   case 'T':
      printf("\n\nTension cable for puck ID: ");
      if(argc >= 3){
         arg1 = atol(argv[2]);
         printf("%d", arg1);
      }else{
         scanf("%d", &arg1);
      }
      cableTension(arg1);
      break;
   case 'S': // Temperature Sensors
      printf("\n\nCheck temperature sensors for puck ID: ");
      if(argc >= 3){
         arg1 = atol(argv[2]);
         printf("%d", arg1);
      }else{
         scanf("%d", &arg1);
      }
      checkTemp(arg1);
      break;
   case 'A': // Autotensioner test
      printf("\n\nAutotensioner test for puck ID: ");
      if(argc >= 3){
         arg1 = atol(argv[2]);
         printf("%d", arg1);
      }else{
         scanf("%d", &arg1);
      }
      checkAutotensioner(arg1);
      break;
   //case 'R':
   //   tensionCable2();
   //   break;
   case 'B':
      BHandDL();
      break;
   case 'Q':
   //   printf("\n\n");
      done = TRUE;
      break;
   default:
      printf("\n\nExample usage:");
      printf("\nbtutil -e             Enumerate bus");
      printf("\nbtutil -h 1           Hall check, motor 1");
      printf("\nbtutil -i 1 -n 2      Change ID 1 to new ID 2");
      printf("\nbtutil -f 1           Find MOFST, motor 1");
      printf("\nbtutil -g 1           Get parameters, motor 1");
      printf("\nbtutil -d 1 -f x.tek  Download x.tek firmware to puck 1");
      printf("\nbtutil -p 5 -l 1      Set puck 5 defaults to puck 1 defaults");
      printf("\nbtutil -t 1           Tension cable, motor 1");
      printf("\nbtutil -a 1           Autotensioner test, motor 1");
      printf("\nbtutil -s 1           Temperature sensor test, motor 1");
      printf("\n");
      break;
   }
   printf("\n\n");
}

#if 0
void tensionCable(void)
{
   

}

void tensionCable2(void)
{
   int motor;
   long reply;
   int cnt;

   printf("\nTorque Motor...\nWhich motor: ");
   scanf("%d", &motor);
   setPropertySlow(0,GROUPID(0),TORQ,FALSE,0);
   wakePuck(0,GROUPID(0));
   setPropertySlow(0,GROUPID(0),MODE,FALSE,MODE_TORQUE);

   setPropertySlow(0,motor,ECMAX,FALSE,0);
   setPropertySlow(0,motor,ECMIN,FALSE,0);
   //setPropertySlow(0,motor,MOFST,FALSE,-1);
   setPropertySlow(0,motor,TORQ,FALSE,500);
   printf("\nRunning at 500 Torq");
   for(cnt = 0;cnt<500;cnt++) {

      getProperty(0,motor,AP,&reply);
      printf("\nAP:%d ",reply);
      getProperty(0,motor,ECMAX,&reply);
      printf("ECMAX:%d ",reply);
      getProperty(0,motor,ECMIN,&reply);
      printf("ECMIN:%d ",reply);
      usleep(100000);
   }
   mygetch();

   setPropertySlow(0,motor,TORQ,FALSE,0);
   setPropertySlow(0,GROUPID(0),MODE,FALSE,MODE_IDLE);
}
#endif

/* Firmware download sample code.
 
   Note: ReadSerial(char *inputBuffer, int charCt); 
         WriteSerial(char *outputBuffer, int charCt); 
    ...must be created under your operating system.
*/
int ReadSerial(char *buf, int bytesToRead)
{
   int bytesRead;
   int totalRead = 0;
   long msec = 0;

   /** Read data from the serial port */
   do {
      serialRead(&p, buf, bytesToRead, &bytesRead);
      buf += bytesRead;
      totalRead += bytesRead;
      if(bytesToRead == totalRead)
         break;
      usleep(2000);
      msec += 2;
      if(msec == 1000) {
         printf("ReadSerial timeout!\n");
         return(1);
      }
   } while(1)
      ;

   return(0);
}

WriteSerial(char *buf, int bytesToWrite)
{
   /** Write data to the serial port */
   serialWrite(&p, buf, bytesToWrite);
   return(bytesToWrite);
}

// Read characters, check for errors
char Read(void)
{
   char buf[10];

   //read character
   if( ReadSerial( buf, 1 ) )
      return 0;

   return buf[0];
}


// Write characters, check for errors
int Write( char ch )
{
   char buf[2] = " ";
   buf[0] = ch;
   buf[1] = 0;

   //write character
   return( WriteSerial( buf, 1 ) );

   //return;
}


// Echo write



int EchoWrite(char ch)
{
   char test;
   int err;

   err = Write( ch );
   if (err!=1)
      printf("err:%d",err);
   //fflush(stdout);
   do {
      test = Read();
      if (test != ch)
         printf("\nSent:%2hhX Recvd:%2hhX\n",ch,test);
   } while( ch != test );
   if (test==ch)
      printf("%2hhX",ch);
   fflush(stdout);
   return(err);
}

int BHFirmwareDL(char *fname)
{
   char stype[3],shex[80], line[100];
   unsigned int num,temp_lo,temp_high,first,opcode;
   FILE *fhook;
   int total_bytes;
   long i;
   int count = 0;
   int lobyte, hibyte;
   static int ramarray[300];
   int errcnt=0;

   count=0;
   total_bytes=0;

   //Open the user-specified *.S19 file
   if((fhook=fopen(fname,"r")) == NULL) {
      printf("\nFILE NOT FOUND.\n");
      return(1);
   }

   //Scan the length of the *.S19 file
   while (!feof(fhook)) {
      fgets(line,85,fhook);

      sscanf(line,"%2s%2x%2x%2x%76s",stype,&num,&temp_high,&temp_lo,shex);
      if (strstr(stype,"S1")) {
         num-=3;     // subtract addr. and checksum from # of pairs
         total_bytes+=num;
      }
   }
   fclose (fhook);

   Write('X'); //Write to MC68HC811 e<X>ternal EEPROM

   printf( "\nPower up the hand to begin download...\n" );
   
   /* Turn on the hand power via the parallel port ...*/
   outb((unsigned char)0x05, 0x378); 	// Output Data to the Parallel Port
   
   while ( Read() != ':' && errcnt<50) {
      errcnt++; //Wait for RESET
   }
   if( errcnt>=50 ) {
      printf( "\nDownload Failed\n" );
      return(1);
   }

   Write(255);  // send first trigger byte

   strcpy(stype,"");
   first=1;
   printf("\nProgress: 0%\n");
   fflush(stdout);
   //Download the *.S19 file
   if((fhook=fopen(fname,"r")) == NULL)
      return(1);

   while (!feof(fhook)) {
      fgets(line,85,fhook); //Read a line of data from *.S19 file
      if( feof(fhook) )
         break;

      // basic line format, scan into variables
      sscanf(line,"%2s%2x%2x%2x%76s",stype,&num,&temp_high,&temp_lo,shex);
      printf("%s %2x %2x %2x %2s\n", stype, temp_lo, temp_high, num,
             shex);
      lobyte=temp_lo;
      hibyte=temp_high;
      if (strstr(stype,"S1")) {  //If this is a DATA S-Record
         //Update progress
         float per;
         num-=3;     // subtract addr. and checksum from # of pairs
         per = (100*(double)(count+num)/(double)total_bytes)-1;
         if ( per < 0.0)
            per = 0.0;
         count += num;
         //printf("\rProgress: %3.0lf%%", per);
         fflush(stdout);
         for (i=0;i<num; i+=1) {  // read pairs into ramarray
            sscanf(shex+i*2,"%2x",&opcode);
            ramarray[i]=opcode;
         }

         //Pass the data on to Monitor in the BarrettHand
         //Monitor will write the data to external RAM
         Write(255);
         printf("=> ");
         EchoWrite (lobyte);
         printf(" ");
         EchoWrite (hibyte);
         printf(" ");
         EchoWrite (num);
         printf(" ");
         Write(255);
         for (i=0; i<num; i++) {
            EchoWrite (ramarray[i]);
         }
         printf("\n");
      }
   }
   fclose (fhook);
   printf("\rProgress: 100%\n");

   return(0);
}

int BHandDL(void)
{

   char fn[32];
   char portlocation[32];
   int err;

   // Ask for *.S19 file
   printf("\nBarrettHand Firmware Download\nPlease enter firmware (*.S19) filename: ");
   scanf("%s", fn);

   // Ask for port
   printf("\nPlease enter port (ex: /dev/ttyS0): ");
   scanf("%s", portlocation);

   /** Open serial port */
   err =  serialOpen(&p, portlocation);
   if(err) {
      printf("\nError opening port!\n");
      exit(0);
   }

   /** Set the baud rate */
   serialSetBaud(&p, 9600);
   
   /* Turn off power via the parallel port (will be re-enabled in BHFirmwareDL())*/
   if (ioperm(0x378,1,1)) 
         fprintf(stderr, "ERROR: Can't gain access to parallel port\n"), exit(1);
   outb((unsigned char)0x00, 0x378); 	// Output Data to the Parallel Port
   sleep(3);

   if(err = BHFirmwareDL(fn)) {
      printf("\nDownload failed! Err = %d\n", err);
      exit(0);
   }

   printf("\n **** Download Complete! ****\n");

   /** Close serial port */
   serialClose(&p);

   return(0);
}


#if 0





void calibrateGimbals(void)
{
   long z[8],p[8];
   double gain, offset;

   wakePuck(0,5); // Wake gimbals
   setPropertySlow(0,5,DIG0,0,1);
   setPropertySlow(0,5,DIG1,0,1);

   setPropertySlow(0,5,GAIN1,0,4096);
   setPropertySlow(0,5,GAIN2,0,4096);
   setPropertySlow(0,5,GAIN3,0,4096);
   setPropertySlow(0,5,OFFSET1,0,0);
   setPropertySlow(0,5,OFFSET2,0,0);
   setPropertySlow(0,5,OFFSET3,0,0);
   printf("\nJ5=J6=J7=0: \n");
   mygetch();
   getPositions(0,WHOLE_ARM,3,z);
   printf("\nJ5=J6=J7=+1.57: \n");
   mygetch();
   getPositions(0,WHOLE_ARM,3,p);

   // m = 1.57 / (j5p - j5z)
   // b = -j5z * m
   gain = 1.57 / (p[5] - z[5]);
   offset = -z[5] * gain;
   gain *= 1L << 24;
   offset *= 1L << 12;
   setPropertySlow(0,5,GAIN1,0,(long)gain);
   setPropertySlow(0,5,OFFSET1,0,(long)offset);
   printf("J5: GAIN=%ld, OFFSET=%ld\n", (long)gain, (long)offset);
   gain = 1.57 / (p[6] - z[6]);
   offset = -z[6] * gain;
   gain *= 1L << 24;
   offset *= 1L << 12;
   setPropertySlow(0,5,GAIN2,0,(long)gain);
   setPropertySlow(0,5,OFFSET2,0,(long)offset);
   printf("J6: GAIN=%ld, OFFSET=%ld\n", (long)gain, (long)offset);
   gain = 1.57 / (p[7] - z[7]);
   offset = -z[7] * gain;
   gain *= 1L << 24;
   offset *= 1L << 12;
   setPropertySlow(0,5,GAIN3,0,(long)gain);
   setPropertySlow(0,5,OFFSET3,0,(long)offset);
   printf("J7: GAIN=%ld, OFFSET=%ld\n", (long)gain, (long)offset);
   setPropertySlow(0,5,SAVE,0,-1); // Save all
}


allParams(int newID)
{
   long reply;
   int cnt;

   wakePuck(0,newID);
   for (cnt = 0;cnt < PROP_END; cnt++) {
      getProperty(0,newID,cnt,&reply);
      //printf("%d %s = %ld\n",cnt,Prop2Name(cnt),reply);
   }
}






void showMenu(void)
{
   printf("\nMENU");
   printf("\n--------");
   printf("\nE)numerate bus status");
   printf("\nF)ind Motor offset");
   printf("\nP)arameter defaults");
   printf("\nD)ownload firmware");
   printf("\nG)et parameters");
   printf("\nGet (A)ll parameters");
   printf("\nT)ension cable");
   printf("\nR)un motor with no index");
   printf("\nB)arrettHand firmware download");
   printf("\nC)alibrate gimbals");
   printf("\nH)all feedback check");
   printf("\n\nQ)uit");
   printf("\n\nYour Choice: ");
}
#endif

