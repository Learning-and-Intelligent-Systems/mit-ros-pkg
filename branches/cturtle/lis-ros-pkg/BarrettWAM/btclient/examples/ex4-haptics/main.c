/* ======================================================================== *
 *  Module ............. Example 4 - Haptics
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
    Demostrates haptic (force-feedback) interaction with the WAM.
 
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
#include "bthaptics.h"

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

/* Haptic-specific variables */
bthaptic_scene             bth;
bthaptic_object            objects[30];

btgeom_state               pstate;
btgeom_plane               planes[10];
btgeom_sphere              spheres[10];
btgeom_box                 boxs[10];

bteffect_wall              wall[10];
bteffect_wickedwall        wickedwalls[10];
bteffect_magneticwall      magneticwalls[10];
bteffect_bulletproofwall   bpwall[10];

/*==============================*
 * PRIVATE Function Prototypes  *
 *==============================*/
void Cleanup();
void sigint_handler();
void rt_thread(void *thd);
void init_haptics(void);
int WAMcallback(wam_struct *wam);

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
   wam_thd.done = TRUE;
   usleep(10000);
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
int WAMcallback(struct btwam_struct *wam)
{
   /* Take the WAM's Cartesian position (XYZ) and differentiate it into 
    * the Cartesian velocity (pstate.vel) and acceleration (pstate.acc).
    */
   eval_state_btg(&(pstate),wam->Cpos);

   /* Using the WAM's position and velocity, loop through each of the objects
    * in the haptic scene and add their force effects (if any) together.
    */
   eval_bthaptics(&bth, (vect_n*)wam->Cpos, (vect_n*)pstate.vel, (vect_n*)pstate.acc, (vect_n*)wam->Cforce);
   
   /* Apply the tool force due to haptic object interaction */
   apply_tool_force_bot(&(wam->robot), wam->Cpoint, wam->Cforce , wam->Ctrq);
   
   return 0;
}

/* Initialize the haptic scene with various objects */
void init_haptics(void)
{
   int      cnt, i;
   btreal   xorig, yorig, zorig;
   int      objectCount = 0;
   vect_3   *p1, *p2, *p3;

   /* Define some variables to store point data */
   p1 = new_v3();
   p2 = new_v3();
   p3 = new_v3();
   
   /* Define the offset from the origin */
   xorig = 0.0;
   yorig = 0.0;
   zorig = 0.10;

   /* Allocate a scene with space for 10 objects */
   new_bthaptic_scene(&bth, 10);
   
   /* Initialize the position filter (btgeometry) */
   init_state_btg(&pstate, Ts, 30.0); // Update rate, filter cutoff Hz
   
   /* Define the Haptic interaction point with respect to the WAM tool frame */
   const_v3(wam->Cpoint, 0.0, 0.0, 0.0); // Define the interaction point to be at the tool

   /* To add a haptic object to the scene, you must:
    * 1) Define the object geometry
    * 2) Define a type of haptic interaction (method of force response)
    * 3) Tie the object geometry and haptic interaction together into a haptic object
    * 4) Add the new haptic object to the scene
    */
    
   /* Create workspace bounding box */
   init_bx_btg(&boxs[0], const_v3(p1,0.7,0.0,zorig+0.0), const_v3(p2,0.7,0.01,zorig+0.0), const_v3(p3,0.7,0.0,zorig+0.01), 1.0, 0.6, 0.4, 1);
   init_bulletproofwall(&bpwall[0], 0.0, 0.0, 0.05, 4000.0, 10.0, 10.0);
   init_normal_box_bth(&objects[objectCount], &boxs[0], (void*)&bpwall[0], bulletproofwall_nf);
   addobject_bth(&bth, &objects[objectCount++]);
   
   /* Create nested spheres */
   init_sp_btg(&spheres[0], const_v3(p1, 0.5, 0.0, zorig+0.0), const_v3(p2, 0.40, 0.0, zorig+0.0), 0); // Inner sphere, outer wall
   init_sp_btg(&spheres[1], const_v3(p1, 0.5, 0.0, zorig+0.0), const_v3(p2, 0.42, 0.0, zorig+0.0), 1); // Inner sphere, inner wall
   init_sp_btg(&spheres[2], const_v3(p1, 0.5, 0.0, zorig+0.0), const_v3(p2, 0.30, 0.0, zorig+0.0), 0); // Outer sphere, outer wall
   init_sp_btg(&spheres[3], const_v3(p1, 0.5, 0.0, zorig+0.0), const_v3(p2, 0.32, 0.0, zorig+0.0), 1); // Outer sphere, inner wall
   
   /* Perform steps 2-4 (above) for each sphere */
   for(cnt = 0; cnt < 4; cnt++) {
      init_wickedwall(&wickedwalls[cnt], 3000.0, 10.0, 5.0, 0.020, 0.01);
      init_normal_sphere_bth(&objects[objectCount], &spheres[cnt], (void*)&wickedwalls[cnt], wickedwall_nf);
      addobject_bth(&bth, &objects[objectCount++]);
   }
   
   /* Other ideas */
   //    2 nested spheres tangent to each other
   //    (use with wickedwall)
   //    init_sp_btg( &spheres[0],const_v3(p1,0.4,0.2,zorig+0.0),const_v3(p2,0.4,0.0,zorig+0.0),0);
   //    init_sp_btg( &spheres[1],const_v3(p1,0.4,0.2,zorig+0.0),const_v3(p2,0.4,0.02,zorig+0.0),1);
   //    init_sp_btg( &spheres[2],const_v3(p1,0.4,0.2,zorig+0.0),const_v3(p2,0.4,0.1,zorig+0.0),0);
   //    init_sp_btg( &spheres[3],const_v3(p1,0.4,0.2,zorig+0.0),const_v3(p2,0.4,0.12,zorig+0.0),1);

   //    init_sp_btg( &spheres[4],const_v3(p1,0.4,-0.2,zorig+0.0),const_v3(p2,0.4,0.0,zorig+0.0),0);
   //    init_sp_btg( &spheres[5],const_v3(p1,0.4,-0.2,zorig+0.0),const_v3(p2,0.4,-0.02,zorig+0.0),1);
   //    init_sp_btg( &spheres[6],const_v3(p1,0.4,-0.2,zorig+0.0),const_v3(p2,0.4,-0.1,zorig+0.0),0);
   //    init_sp_btg( &spheres[7],const_v3(p1,0.4,-0.2,zorig+0.0),const_v3(p2,0.4,-0.12,zorig+0.0),1);


   //    2 spherical shells tangent to each other
   //    (use with wickedwall)
   //    init_sp_btg( &spheres[0],const_v3(p1,0.4,0.2,zorig+0.0),const_v3(p2,0.4,0.0,zorig+0.0),0);
   //    init_sp_btg( &spheres[1],const_v3(p1,0.4,0.2,zorig+0.0),const_v3(p2,0.4,0.02,zorig+0.0),1);

   //    init_sp_btg( &spheres[2],const_v3(p1,0.4,-0.2,zorig+0.0),const_v3(p2,0.4,0.0,zorig+0.0),0);
   //    init_sp_btg( &spheres[3],const_v3(p1,0.4,-0.2,zorig+0.0),const_v3(p2,0.4,-0.02,zorig+0.0),1);


   //    2 solid spheres tangent to each other
   //    (use with bulletproofwall or magneticwall)
   //    init_sp_btg( &spheres[0],const_v3(p1,0.4,0.2,zorig+0.0),const_v3(p2,0.4,0.0,zorig+0.0),0);
   //    init_sp_btg( &spheres[1],const_v3(p1,0.4,-0.2,zorig+0.0),const_v3(p2,0.4,0.0,zorig+0.0),0);

   
   //    Overlapping spherical shells
   //    (use with wickedwall)
   //    init_sp_btg( &spheres[0],const_v3(p1,0.4,0.1,zorig+0.0),const_v3(p2,0.4,-0.12,zorig+0.0),0);
   //    init_sp_btg( &spheres[1],const_v3(p1,0.4,0.1,zorig+0.0),const_v3(p2,0.4,-0.1,zorig+0.0),1);

   //    init_sp_btg( &spheres[2],const_v3(p1,0.4,-0.1,zorig+0.0),const_v3(p2,0.4,0.12,zorig+0.0),0);
   //    init_sp_btg( &spheres[3],const_v3(p1,0.4,-0.1,zorig+0.0),const_v3(p2,0.4,0.1,zorig+0.0),1);

   /* Other effects ***
   for(cnt = 0; cnt < [howManyObjects]; cnt++) {
      init_magneticwall(&magneticwalls[cnt],0.0,0.0,0.05,4000.0,10.0,10.0);
      //init_bulletproofwall(&bpwall[cnt],0.0,0.0,0.05,4000.0,10.0,10.0);
      //init_wickedwall(&wickedwalls[cnt],3000.0, 10.0,5.0,0.020,0.01);

      init_normal_sphere_bth(&objects[objectCount], &spheres[cnt], (void*)&magneticwalls[cnt], magneticwall_nf);
      //remember to change type of wall within init_normal_sphere declaration here
      //if changing between bulletproofwall, wickedwall, magneticwall, etc.
   }
   */
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
   btrt_thread_create(&rt_thd, "rtt", 45, (void*)rt_thread, NULL);
   while(!startDone)
      usleep(10000);

   /* Register the control loop's local callback routine */
   registerWAMcallback(wam, WAMcallback);
   
   /* Initialize a haptic scene */
   init_haptics();
   
   /* Spin off the WAM control loop */
   wam_thd.period = Ts; // Control loop period in seconds
   btrt_thread_create(&wam_thd, "ctrl", 90, (void*)WAMControlThread, (void*)wam);

   mvprintw(8,0,"Please activate the WAM (press Shift+Activate on the pendant), ");
   mvprintw(9,0,"then press <Enter>");
   while(getch()==ERR)
      usleep(5000);
   
   /* Clear the screen (ncurses) */
   clear(); refresh();
   
   /* Set gravity scale to 1.0g */
   SetGravityComp(wam, 1.0); 

   /* Turn on haptic scene */
   bth.state = TRUE;
   
   /* Loop until Ctrl-C is pressed */
   mvprintw(0,0,"Gravity compensation and WAM data display demo");
   while(1) {
      /* Display some interesting WAM data on-screen */
      line = 2;
      
      mvprintw(line, 0, "Robot name = %s     Degrees of Freedom = %d", wam->name, wam->dof); line += 2;
      mvprintw(line, 0, "Joint Position (rad): %s", sprint_vn(buf, wam->Jpos)); ++line;
      mvprintw(line, 0, "Joint Torque (Nm)   : %s", sprint_vn(buf, wam->Jtrq)); ++line;
      mvprintw(line, 0, "Cartesian XYZ (m)   : %s", sprint_vn(buf, (vect_n*)wam->Cpos)); ++line;
      mvprintw(line, 0, "Cartesian Force (N) : %s", sprint_vn(buf, (vect_n*)wam->Cforce)); ++line;
      
      ++line;
      mvprintw(line, 0, "To exit, press Shift-Idle on pendant, then hit Ctrl-C");
      
      refresh(); // Draw the screen
      usleep(1E5); // Sleep for 1E5 microseconds or 0.1 seconds
   }
   
   return(0); 
}


