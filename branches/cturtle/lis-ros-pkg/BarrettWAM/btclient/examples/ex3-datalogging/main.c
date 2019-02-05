/* ======================================================================== *
 *  Module ............. Example 3 - Datalogging
 *  File ............... main.c
 *  Creation Date ...... 26 May 2005
 *  Author ............. Traveler Hauptman
 *  Addtl Authors ...... Brian Zenowich
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  Copyright (C) 2005-2008 Barrett Technology, Inc. <support@barrett.com>
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

/** \file main.c
    This program demonstrates the datalogging, timing, and WAM control
    loop callback functions of the library.
 
 */

/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>
#include <signal.h>
#include <sys/mman.h>
#include <curses.h>

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#include "btwam.h"

/*==============================*
 * PRIVATE DEFINED constants    *
 *==============================*/
/* Define the control loop period */
#define Ts (0.002)

/*==============================*
 * GLOBAL file-scope variables  *
 *==============================*/
btrt_thread_struct   rt_thd, wam_thd;
wam_struct           *wam;
int                  startDone;
btgeom_state         pstate;
long                 callbackTime;

/*==============================*
 * PRIVATE Function Prototypes  *
 *==============================*/
void Cleanup();
void sigint_handler();
void rt_thread(void *thd);
int WAMcallback(wam_struct *w);

/*==============================*
 * Functions                    *
 *==============================*/
 
/* If Ctrl-C is pressed, exit gracefully */
void sigint_handler()
{
   Cleanup();
   exit(1);
}

/* The CANbus card must be initialized and called from a realtime thread.
 * The rt_thread is spun off from main() to handle the initial communications.
 */
void rt_thread(void *thd){
   int err;

   /* Probe and initialize the robot actuators */
   err = InitializeSystem();
   if(err) {
      syslog(LOG_ERR, "InitializeSystem returned err = %d", err);
      exit(1);
   }
    
   /* Initialize and get a handle to the robot on the first bus */
   if((wam = OpenWAM("../../wam.conf", 0)) == NULL){
      syslog(LOG_ERR, "OpenWAM failed");
      exit(1);
   }
   
   /* setSafetyLimits(bus, joint rad/s, tip m/s, elbow m/s);
    * For now, the joint and tip velocities are ignored and
    * the elbow velocity provided is used for all three limits.
    */
   setSafetyLimits(0, 1.5, 1.5, 1.5);  // Limit to 1.5 m/s

   /* Set the puck torque safety limits (TL1 = Warning, TL2 = Critical).
    * Note: The pucks are limited internally to 3441 (see 'MT' in btsystem.c) 
    * Note: btsystem.c bounds the outbound torque to 8191, so entering a
    * value of 9000 for TL2 would tell the safety system to never register a 
    * critical fault.
    */
   setProperty(0, SAFETY_MODULE, TL2, FALSE, 4700);
   setProperty(0, SAFETY_MODULE, TL1, FALSE, 1800);
   
   /* Notify main() thread that the initialization is complete */
   startDone = TRUE;
   
   /* Spin until we are told to exit */
   while (!btrt_thread_done((btrt_thread_struct*)thd)){
      usleep(10000);
   }
   
   /* Remove this thread from the realtime scheduler */
   btrt_thread_exit((btrt_thread_struct*)thd);
}

/* Exit the realtime threads and close the system */
void Cleanup(){
   /* Turn off Datalogging */
   DLoff(&(wam->log));
   
   wam_thd.done = TRUE;
   usleep(10000);
   
   /* Close the log file */
   CloseDL(&(wam->log));
   
   /* Decode the file from binary->text */
   DecodeDL("datafile.dat", "dat.csv", 1);
   
   CloseSystem();
   rt_thd.done = TRUE;
   printf("\n\n");
}

/* The registerWAMcallback() function registers this special function to be
 * called from the WAMControlThread() after the positions have been received 
 * from the WAM (and after all the kinematics are calculated) but before torques 
 * are sent to the WAM.
 * NOTE 1: Since this function becomes part of the control loop, it must execute
 * quickly to support the strict realtime periodic scheduler requirements.
 * NOTE 2: For proper operation, you must avoid using any system calls that 
 * cause this thread to drop out of realtime mode. Avoid syslog, printf, and
 * most other forms of I/O.
 */
int WAMcallback(wam_struct *w)
{
   /* Declare two timing variables */
   RTIME start, end;
   
   /* Set a damping factor.
    * If you make this positive, you have a poor-man's (unstable) friction 
    * compensation algorithm. Have fun, but be careful!
    */
   btreal Kscale = -30.0; 
   
   /* Get the time in nanoseconds */
   start = btrt_get_time();
   
   /* Differentiate the Cartesian end position to get velocity and acceleration */
   eval_state_btg(&pstate, w->Cpos);
   
   /* Apply velocity damping.
    * Here is what happens with our math library:
    *    scale_vn() Scale a vector: pstate.vel->ret = Kscale * pstate.vel
    *    add_vn() Add two vectors:  wam->Cforce->ret = wam->Cforce + pstate.vel->ret
    *    set_vn() Copy a vector:    wam->Cforce = wam->Cforce->ret
    */
   set_vn((vect_n*)wam->Cforce, add_vn((vect_n*)wam->Cforce, scale_vn(Kscale, (vect_n*)pstate.vel)));

   /* Apply the calculated force vector to the robot */
   apply_tool_force_bot(&(w->robot), w->Cpoint, w->Cforce, w->Ctrq);

   /* Get the time in nanoseconds */
   end = btrt_get_time();
   
   /* NOTE: Be careful what you do with RTIMEs. They exhibit odd behavior unless
    * you stick to simple addition and subtraction. And if you print them,
    * use %lld (they are 64-bits).
    */
   callbackTime = end - start;
   
   return 0;
}

/* Program entry point */
int main(int argc, char **argv)
{
   int   err;        // Generic error variable for function calls
   int   busCount;   // Number of WAMs defined in the configuration file
   char  buf[1024];  // String used by sprint_vn() to convert the joint angle data to text
   int   line;       // Line marker for where to print text on the screen
   
   /* Allow hard real time process scheduling for non-root users */
#ifdef RTAI   
   rt_allow_nonroot_hrt();
#else
   mlockall(MCL_CURRENT | MCL_FUTURE);
   /* Xenomai non-root scheduling is coming soon! */
#endif

   /* Initialize the ncurses screen library */
   initscr(); cbreak(); noecho(); timeout(0); clear();
   atexit((void*)endwin);
   
   /* Initialize syslog */
   openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
   atexit((void*)closelog);

   /* Register the ctrl-c interrupt handler */
   signal(SIGINT, sigint_handler);

   /* Read the WAM configuration file */
   err = ReadSystemFromConfig("../../wam.conf", &busCount);
   if(err) {
      syslog(LOG_ERR, "ReadSystemFromConfig returned err = %d", err);
      exit(1);
   }
   
   /* Lead the user through a proper WAM startup */
   mvprintw(0,0,"Make sure the all WAM power and signal cables are securely");
   mvprintw(1,0,"fastened, then turn on the main power to WAM and press <Enter>");
   while(getch()==ERR)
      usleep(5000);
   mvprintw(3,0,"Make sure all E-STOPs are released, then press Shift-Idle");
   mvprintw(4,0,"on the control pendant. Then press <Enter>");
   while(getch()==ERR)
      usleep(5000);
   mvprintw(6,0,"Place WAM in its home (folded) position, then press <Enter>");
   while(getch()==ERR)
      usleep(5000);
   
   /* Spin off the RT task to set up the CAN Bus */
   startDone = FALSE;
   btrt_thread_create(&rt_thd,"rtt", 45, (void*)rt_thread, NULL);
   while(!startDone)
      usleep(10000);

   /* Register the control loop's local callback routine */
   registerWAMcallback(wam, WAMcallback);
   
   /* Initialize a WAM state evaluator */
   init_state_btg(&pstate, Ts, 30.0);
   
   /* Commanded Cartesian forces and torques are applied about Cpoint, which is 
    * defined as an offset from the kinematic endpoint of the robot.
    * This is where we define that offset in meters (x, y, z).
    */
   const_v3(wam->Cpoint, 0.0, 0.0, 0.0);
   
   /*==============================*
    * Set up Datalogging           *
    *==============================*/
    
   /* NOTE: Buffer dumps are handled by the WAMMaintenanceThread() which is spun
    * off inside OpenWAM(). Buffer data is recorded automatically from the 
    * WAMControlThread()'s TriggerDL() function call.
    */
    
   /* Configure the logDivider: 1 = every control cycle, 2 = every other cycle, etc. */
   wam->logDivider = 5;
   
   /* Prepare the datalogger with the max number of btreal fields per record */
   PrepDL(&(wam->log), 35);
   
   /* Add a pointers to some data to log */
   /* NOTE: log_time begins counting seconds when the WAMControlThread() is started */
   AddDataDL(&(wam->log), &(wam->log_time), sizeof(double), BTLOG_DOUBLE, "Time(s)");
   AddDataDL(&(wam->log), valptr_vn((vect_n*)wam->Cpos),sizeof(btreal) * len_vn((vect_n*)wam->Cpos), BTLOG_BTREAL,"Cpos(m)");
   AddDataDL(&(wam->log), valptr_vn((vect_n*)pstate.vel), sizeof(btreal) * len_vn((vect_n*)pstate.vel), BTLOG_BTREAL, "Cvel(m/s)");
   AddDataDL(&(wam->log), valptr_vn((vect_n*)wam->Cforce), sizeof(btreal) * len_vn((vect_n*)wam->Cforce), BTLOG_BTREAL, "Cforce(N)");
   AddDataDL(&(wam->log), &callbackTime, sizeof(long), BTLOG_LONG, "callbackTime(ns)");

   /* Initialize the datalogging buffer size and output file.
    * Once the buffer is full, the data is written to disk. Datalogging continues
    * even as the buffer is being written out.
    */
   InitDL(&(wam->log), 1000, "datafile.dat");
   
   /* Spin off the WAM control loop */
   wam_thd.period = Ts; // Control loop period in seconds
   btrt_thread_create(&wam_thd, "ctrl", 90, (void*)WAMControlThread, (void*)wam);

   /* Prompt the user to activate the WAM.*/
   mvprintw(8,0,"Please activate the WAM (press Shift+Activate on the pendant), ");
   mvprintw(9,0,"then press <Enter>");
   while(getch()==ERR)
      usleep(5000);
   
   /* Clear the screen (ncurses) */
   clear(); refresh();
   
   /* Set gravity scale to 1.0g */
   SetGravityComp(wam, 1.0); 

   /* Turn on Datalogging */
   DLon(&(wam->log));
   
   /* Loop until Ctrl-C is pressed */
   mvprintw(0,0,"Datalogging, timing, and control loop callback example");
   while(1) {
      /* Display some interesting WAM data on-screen */
      line = 2;
      
      mvprintw(line, 0, "Robot name = %s     Degrees of Freedom = %d", wam->name, wam->dof); line += 2;
      mvprintw(line, 0, "Joint Position (rad): %s", sprint_vn(buf, wam->Jpos)); ++line;
      mvprintw(line, 0, "Joint Torque (Nm)   : %s", sprint_vn(buf, wam->Jtrq)); ++line;
      mvprintw(line, 0, "Cartesian XYZ (m)   : %s", sprint_vn(buf, (vect_n*)wam->Cpos)); ++line;
      mvprintw(line, 0, "Cartesian Force (N) : %s", sprint_vn(buf, (vect_n*)wam->Cforce)); ++line;
      mvprintw(line, 0, "Callback Time (ns)  : %ld", callbackTime); ++line;
      
      ++line;
      mvprintw(line, 0, "To exit, press Shift-Idle on pendant, then hit Ctrl-C"); ++line;
      mvprintw(line, 0, "...and check out the dat.csv log file after you exit!");
      
      refresh(); // Draw the screen
      usleep(1E5); // Sleep for 1E5 microseconds or 0.1 seconds
   }
   
   return(0); 
}


