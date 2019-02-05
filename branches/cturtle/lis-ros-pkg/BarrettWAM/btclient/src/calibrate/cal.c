/* ======================================================================== *
 *  Module ............. calibrate
 *  File ............... cal.c
 *  Creation Date ...... 10 Jul 2008
 *  Author ............. Christopher Dellin
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  Copyright (C) 2008-2008 Barrett Technology, Inc. <support@barrett.com>
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

#include <stdio.h>    /* printf(), getchar(), setvbuf() */
#include <string.h>   /* strcmp(), etc */
#include <unistd.h>   /* usleep() */
#include <sys/mman.h> /* mlockall() */
#include <signal.h>   /* For the Control+C exit handler */

#include <curses.h>   /* ncurses */
#include <syslog.h>   /* syslog */

#include "btwam.h"    /* All barrett functions */

/* ------------------------------------------------------------------------ *
 * Key Grabbing Utility Function                                            */
enum btkey {
   BTKEY_UNKNOWN = -2,
   BTKEY_NOKEY = -1,
   BTKEY_ENTER = 10,
   BTKEY_BACKSPACE = 127,
   BTKEY_UP = 256,
   BTKEY_DOWN = 257,
   BTKEY_LEFT = 258,
   BTKEY_RIGHT = 259
};
enum btkey btkey_get()
{
   int c1,c2,c3;
   
   /* Get the key from ncurses */
   c1 = getch();
   if (c1 == ERR) return BTKEY_NOKEY;
   
   /* Get number and letter keys */
   if (c1 >= '0' && c1 <= '9') return c1;
   if (c1 >= 'A' && c1 <= 'Z') return c1;
   if (c1 >= 'a' && c1 <= 'z') return c1;
   
   /* Get special keys */
   switch (c1)
   {
      case 10: return BTKEY_ENTER;
      case 127: return BTKEY_BACKSPACE;
      /* Get extended keyboard chars (eg arrow keys) */
      case 27:
         c2 = getch();
         if (c2 != 91) return BTKEY_UNKNOWN;
         c3 = getch();
         switch (c3)
         {
            case 65: return BTKEY_UP;
            case 66: return BTKEY_DOWN;
            case 67: return BTKEY_RIGHT;
            case 68: return BTKEY_LEFT;
            default: return BTKEY_UNKNOWN;
         }
      default:
         return BTKEY_UNKNOWN;
   }
}

/* ------------------------------------------------------------------------ *
 * Program Entry Point -- Choose Calibration Routine                        */
int main(int argc, char **argv)
{
   int err;
   enum {
      MODE_ZERO = 0, /* Determine the zero/home positions */
      MODE_MU, /* Determine the mu vectors */
      MODE_EXIT
   } mode;
   
   /* This keeps this program's memory from being swapped out, reducing
    * the chance of disruptions from other programs on the system. */
   mlockall(MCL_CURRENT | MCL_FUTURE);
   
#ifdef RTAI
   /* Allow hard real time process scheduling for non-root users 
    * Xenomai non-root scheduling is coming soon! */
   rt_allow_nonroot_hrt();
#endif
   
   /* Parse command-line arguments */
   if ((argc >= 2) && ( !strncasecmp(argv[1],  "h",1)
                     || !strncasecmp(argv[1], "-h",2)
                     || !strncasecmp(argv[1],"--h",3) ))
   {
      printf("Usage: %s [options]\n");
      printf("\n");
      printf("Options:\n");
      printf("\n");
      printf("  h, -h, --help, ...    Print this message and exit.\n");
      printf("\n");
      printf("  z, -z, --zero, ...    Run the joint-angle zeroing calibration routine.\n");
      printf("                        Requires puck firmware version r118 and magnetic\n");
      printf("                        encoders for full functionality.\n");
      printf("\n");
      printf("  g, -g, --gravity ...  Run the gravity calibration routine.\n");
      printf("\n");
      return 0;
   }
   
   /* Initialize the ncurses screen library */
   initscr(); cbreak(); noecho(); timeout(0); clear();
   atexit((void*)endwin);
   
   /* Initialize syslog */
   openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
   atexit((void*)closelog);
   
   /* Figure out which calibration routine the user wants to do */
   if ((argc >= 2) && ( !strncasecmp(argv[1],  "z",1)
                     || !strncasecmp(argv[1], "-z",2)
                     || !strncasecmp(argv[1],"--z",3) ))
      mode = MODE_ZERO;
   else
   /* Is it "gravity" or "mu" ? */
   if ((argc >= 2) && ( !strncasecmp(argv[1],  "g",1)
                     || !strncasecmp(argv[1], "-g",2)
                     || !strncasecmp(argv[1],"--g",3) ))
      mode = MODE_MU;
   else
   /* OK, let the user choose */
   {
      int chosen = 0;
      mvprintw(0,0,"Barrett WAM Calibration Program.");
      mvprintw(2,0,"Choose a calibration option from the menu below:");
      mvprintw(4,4,"Calibrate Zero Position");
      mvprintw(5,4,"Calibrate Gravity Compensation");
      mvprintw(6,4,"Exit");
      mode = MODE_ZERO;
      while (!chosen)
      {
         /* Display the currently selected mode */
         if (mode == MODE_ZERO) mvprintw(4,0,"-->");
         else                   mvprintw(4,0,"   ");
         if (mode == MODE_MU)   mvprintw(5,0,"-->");
         else                   mvprintw(5,0,"   ");
         if (mode == MODE_EXIT) mvprintw(6,0,"-->");
         else                   mvprintw(6,0,"   ");
         /* Wait for a while */
         refresh();
         usleep(5000);
         /* Get user input */
         switch (btkey_get())
         {
            case BTKEY_UP:    mode--; if ((int)mode < 0)    mode = 0;         break;
            case BTKEY_DOWN:  mode++; if (mode > MODE_EXIT) mode = MODE_EXIT; break;
            case BTKEY_ENTER: chosen = 1;                                     break;
            default:                                                          break;
         }
      }
   }
   
   if (mode == MODE_ZERO) err = do_mode_zero();
   if (mode == MODE_MU)   err = do_mode_mu();
   
   /* End syslog */
   closelog();
   
   return err;
}

/* ------------------------------------------------------------------------ *
 * Things common to all calibration modes                                   */
btrt_thread_struct can_thd, wam_thd;
wam_struct * wam;
int startDone;
int done;

void sigint()
{
   done = -1;
}

/* The CANbus card must be initialized and called from a realtime thread.
 * The rt_thread is spun off from main() to handle the initial communications.
 */
void can_thd_function(void *thd)
{
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
   startDone = 1;
   
   /* Spin until we are told to exit */
   while (!btrt_thread_done((btrt_thread_struct*)thd))
      usleep(10000);
   
   /* Close the system */
   CloseSystem();
   
   /* Remove this thread from the realtime scheduler */
   btrt_thread_exit((btrt_thread_struct*)thd);
}


/* ------------------------------------------------------------------------ *
 * Mode Zero                                                                */

/* We need to be able to grab the magnetic encoder values
 * from a realtime thread (ie wam callback) */
btrt_thread_struct magenc_thd;
int * mz_mechset;
int * mz_magvals;
int mz_magvals_set;

/* A new rt thread which just gets the encoder positions */
void magenc_thd_function(void *thd)
{
   int i;
   
   /* Detect puck versions */
   for (i=0; i<wam->dof; i++)
      mz_mechset[i] = (GetProp(i,VERS) >= 118 && GetProp(i,ROLE) == 256) ? 1 : 0;
   
   while (!btrt_thread_done((btrt_thread_struct*)thd))
   {
      if (mz_magvals_set)
      {
         for (i=0; i<wam->dof; i++) if (mz_mechset[i])
         {
            mz_magvals[i] = (int)GetProp(i,MECH);
         }
         mz_magvals_set = 0;
      }
      usleep(100000);
   }
}

int do_mode_zero()
{
   int err;         /* Generic error variable for function calls */
   int busCount;    /* Number of WAMs defined in the configuration file */
   int i;           /* For iterating through the pucks */
   
   /* GUI stuff */
   enum {
      MODE_TOZERO,
      MODE_CANCEL,
      MODE_PRINTVALS,
      MODE_JSELECT,
      MODE_EDIT
   } mode;
   int joint;
   int decplace;
   vect_n * jangle;
   
   char newhome[80];
   char zeromag[80];
   
   clear();
   mvprintw(0,0,"Starting Zero Calibration Mode");
   
   /* Ensure the WAM is set up, power cycled, and in the home position */
   mvprintw(2,0,"To begin the calibration, follow the following steps:");
   mvprintw(3,0,"  a) Ensure that all WAM power and signal cables are securely fastened.");
   mvprintw(4,0,"  b) Ensure that the WAM is powered on (receiving power).");
   mvprintw(5,0,"  c) E-STOP the WAM (Important!).");
   mvprintw(6,0,"  d) Release all E-STOPs.");
   mvprintw(7,0,"  e) Place the WAM in Shift+Idle mode.");
   mvprintw(8,0,"  f) Carefully ensure that the WAM is in its home (folded) position.");
   mvprintw(9,0,"Press [Enter] to continue.");
   refresh();
   while (btkey_get()!=BTKEY_ENTER) usleep(10000);
   
   /* Initialize system buses */
   err = ReadSystemFromConfig("../../wam.conf", &busCount);
   if(err) return -1;
   
   /* Spin off the CAN thread */
   startDone = 0;
   btrt_thread_create(&can_thd,"can",45,(void*)can_thd_function,NULL);
   while (!startDone) usleep(10000);
   
   /* Spin off the magenc thread, which also detecs puck versions into mechset */
   /* EEK! Why must this be such high priority? */
   mz_mechset = (int *) malloc( wam->dof * sizeof(int) );
   mz_magvals = (int *)calloc( wam->dof, sizeof(int) );
   mz_magvals_set = 0;
   btrt_thread_create(&magenc_thd, "mage", 91, (void*)magenc_thd_function, NULL);
   
   /* Spin off the WAM thread */
   wam_thd.period = 0.002; /* Control loop period in seconds */
   btrt_thread_create(&wam_thd,"ctrl",90,(void*)WAMControlThread,(void*)wam);
   
   /* Allow the user to shift-activate */
   mvprintw(11,0,"Place the WAM in Shift+Activate mode,");
   mvprintw(12,0,"and press [Enter] to continue.");
   refresh();
   while (btkey_get()!=BTKEY_ENTER) usleep(10000);
   
   /* Hold position */
   SetJointSpace(wam);
   MoveSetup(wam, 1.0, 1.0);
   MoveWAM(wam,wam->Jpos);
   
   /* Start the user interface */
   mode = MODE_TOZERO;
   joint = 0;
   jangle = new_vn(wam->dof);
   set_vn(jangle,wam->Jpos);
   
   clear();
   done = 0;
   signal(SIGINT, sigint);  
   while (!done)
   {
      char buf[80];
      int j;
      int line;
      enum btkey key;
      
      /* Display the Zeroing calibration stuff ... */
      mz_magvals_set = 1;
      mvprintw(0, 0, "Joint Position (rad): %s", sprint_vn(buf, wam->Jpos));
      
      line = 2;
      if (mode == MODE_TOZERO)
      {
         attron(A_BOLD);
         mvprintw(line++, 0, "--> ] Move To Current Zero");
         attroff(A_BOLD);
      }
      else
         mvprintw(line++, 0, "    ] Move To Current Zero");
      if (mode == MODE_CANCEL)
      {
         attron(A_BOLD);
         mvprintw(line++, 0, "--> ] Cancel Calibration");
         attroff(A_BOLD);
      }
      else
         mvprintw(line++, 0, "    ] Cancel Calibration");
      if (mode == MODE_PRINTVALS)
      {
         attron(A_BOLD);
         mvprintw(line++, 0, "--> ] Print Calibrated Values and Exit");
         attroff(A_BOLD);
      }
      else
         mvprintw(line++, 0, "    ] Print Calibrated Values and Exit");
      if (mode == MODE_JSELECT)
      {
         attron(A_BOLD);
         mvprintw(line, 0, "--> ] Joint:");
         attroff(A_BOLD);
      }
      else
         mvprintw(line, 0, "    ] Joint:");
      mvprintw(line+1, 5, "-------");
      if (mode == MODE_EDIT) attron(A_BOLD);
      mvprintw(line+2, 5, "   Set:");
      if (mode == MODE_EDIT) attroff(A_BOLD);
      mvprintw(line+3, 5, "Actual:");
      mvprintw(line+5, 5, " Motor:");
      mvprintw(line+6, 5, "MagEnc:");
      for (j=0; j<wam->dof; j++)
      {
         if ((mode == MODE_JSELECT || mode == MODE_EDIT) && j==joint)
         {
            attron(A_BOLD);
            mvprintw(line+0, 13 + 9*j, "[Joint %d]",j+1);
            attroff(A_BOLD);
         }
         else
            mvprintw(line+0, 13 + 9*j, " Joint %d ",j+1);
         /* total with 8, 5 decimal points (+0.12345) */
         if ( mode==MODE_EDIT && j==joint )
         {
            int boldplace;
            mvprintw(line+1, 13 + 9*j, " _._____ ",j+1);
            mvprintw(line+2, 13 + 9*j, "% 08.5f ",jangle->q[j]);
            boldplace = decplace + 1;
            if (decplace) boldplace++;
            mvprintw(line+1, 13 + 9*j + boldplace,"x");
            mvchgat(line+2, 13 + 9*j+boldplace, 1, A_BOLD, 0, NULL );
         }
         else
         {
            mvprintw(line+1, 13 + 9*j, " ------- ",j+1);
            mvprintw(line+2, 13 + 9*j, "% 08.5f ",jangle->q[j]);
         }
         mvprintw(line+3, 13 + 9*j, "% 08.5f ",getval_vn(wam->Jpos,j));
         mvprintw(line+5, 13 + 9*j, " Motor %d",j+1);
         if (mz_mechset[j])
            mvprintw(line+6, 13 + 9*j, "    %04d",mz_magvals[j]);
         else
            mvprintw(line+6, 13 + 9*j, "   (None)",mz_magvals[j]);
         
      }
      refresh();
      
      /* Wait a bit ... */
      usleep(5E4);
      key = btkey_get();
      
      /* If the user is in menu mode, work the menu ... */
      if (mode != MODE_EDIT) switch (key)
      {
         case BTKEY_UP:
            mode--;
            if ((signed int)mode < 0) mode = 0;
            break;
         case BTKEY_DOWN:
            mode++;
            if (mode > MODE_JSELECT) mode = MODE_JSELECT;
            break;
         case BTKEY_ENTER:
            switch (mode)
            {
               case MODE_TOZERO:
                  if (!MoveIsDone(wam)) break;
                  fill_vn(jangle,0.0);
                  MoveWAM(wam,jangle);
                  break;
               case MODE_CANCEL:
                  done = -1;
                  break;
               case MODE_PRINTVALS:
                  if (!MoveIsDone(wam)) break;
                  done = 1;
                  break;
               case MODE_JSELECT:
                  mode = MODE_EDIT;
                  decplace = 4;
                  break;
            }
            break;
         default:
            if (mode == MODE_JSELECT) switch (key)
            {
               case BTKEY_LEFT:
                  joint--;
                  if (joint < 0) joint = 0;
                  break;
               case BTKEY_RIGHT:
                  joint++;
                  if (joint >= wam->dof) joint = wam->dof - 1;
                  break;
            }   
            break;
      }
      /* We're in joint edit mode */
      else switch (key)
      {
         case BTKEY_LEFT:
            decplace--;
            if (decplace < 0) decplace = 0;
            break;
         case BTKEY_RIGHT:
            decplace++;
            if (decplace > 5) decplace = 5;
               break;
         case BTKEY_BACKSPACE:
            mode = MODE_JSELECT;
            break;
         /* Actually do the moves */
         case BTKEY_UP:
            if (!MoveIsDone(wam)) break;
            jangle->q[joint] += pow(10,-decplace);
            MoveWAM(wam,jangle);
            break;
         case BTKEY_DOWN:
            if (!MoveIsDone(wam)) break;
            jangle->q[joint] -= pow(10,-decplace);
            MoveWAM(wam,jangle);
            break;
         default:
            break;
      }
   }
   
   if (done == 1)
   {
      vect_n * vec;
      /* Save the new home location */
      vec = new_vn(wam->dof);
      set_vn( vec, sub_vn(wam->park_location,jangle) );
      sprint_vn(newhome,vec);
      destroy_vn(&vec);
      
      /* Save the zeromag values */
      for (i=0; i<wam->dof; i++) if (!mz_mechset[i]) mz_magvals[i] = -1;
      sprintf(zeromag,"< %d",mz_magvals[0]);
      for (i=1; i<wam->dof; i++)
         sprintf(zeromag+strlen(zeromag),", %d",mz_magvals[i]);
      sprintf(zeromag+strlen(zeromag)," >");
   }
   
   /* Re-fold, print, and exit */
   clear();
   mvprintw(0,0,"Moving back to the park location ...");
   refresh();
   MoveWAM(wam,wam->park_location);
   while (!MoveIsDone(wam)) usleep(10000);
   mvprintw(1,0,"Shift+Idle, and press [Enter] to continue.");
   refresh();
   while (btkey_get()!=BTKEY_ENTER) usleep(10000);
   
   /* Stop threads ... */
   wam_thd.done = 1;
   usleep(10000);
   can_thd.done = 1;
   usleep(50000);
   
   /* Stop ncurses ... */
   endwin();
   
   if (done == 1)
   {
      /* Print the results */
      printf("\n");
      printf("Zeroing calibration ended.\n");
      printf("\n");
      for (i=0; i<wam->dof; i++) if (!mz_mechset[i])
      {
         printf("Note: Some (or all) of your pucks do not support absolute\n");
         printf("position measurement, either because they do not use magnetic\n");
         printf("encoders, or because they have not been updated to firmware r118.\n");
         printf("\n");
         break;
      }
      printf("Copy the following lines into your wam.conf file,\n");
      printf("near the top, in the %s{} group.\n",wam->name);
      printf("Make sure it replaces the old home = < ... > definition.\n");
      printf("--------\n");
      printf("    # Calibrated zero values ...\n");
      printf("    home = %s\n",newhome);
      for (i=0; i<wam->dof; i++) if (mz_mechset[i])
      { printf("    zeromag = %s\n",zeromag); break; }
      printf("--------\n");
      {
         FILE * logfile;
         logfile = fopen("cal-zero.log","w");
         if (logfile)
         {
            fprintf(logfile,"    # Calibrated zero values ...\n");
            fprintf(logfile,"    home = %s\n",newhome);
            for (i=0; i<wam->dof; i++) if (mz_mechset[i])
            { fprintf(logfile,"    zeromag = %s\n",zeromag); break; }
            fclose(logfile);
            printf("This text has been saved to cal-zero.log.\n");
            printf("\n");
         }
         else
         {
            syslog(LOG_ERR,"Could not write to cal-zero.log.");
            printf("Error: Could not write to cal-zero.log.\n");
            printf("\n");
         }
      }
      printf("Note that you must E-Stop (or power-cycle) your WAM\n");
      printf("for the calibrated settings to take effect!\n");
      printf("\n");
   }
   
   return 0;
}

/* ------------------------------------------------------------------------ *
 * Mode Mu-Calibrate                                                        */

#define ANGLE_DIFF (0.03)
#define NUM_POINTS (2000)
#define CALC_LAMBDA (0.000001)
int mu_n;
/* Big arrays of joint torques and joint positions, for a given pose */
double ** mu_jts; /*[NUM_DOFS][NUM_POINTS];*/
double ** mu_jps; /*[NUM_DOFS][NUM_POINTS];*/

int mu_callback(wam_struct *w)
{
   int j;
   /* Save the current joint torques for each joint */
   if (mu_n < NUM_POINTS)
   {
      for (j=0; j<w->dof; j++)
      {
         mu_jts[j][mu_n] = w->Jtrq->q[j];
         mu_jps[j][mu_n] = w->Jpos->q[j];
      }
      mu_n++;
   }
   return 0;
}

/* Do the averaving */
int mu_stats( vect_n * torques, vect_n * positions )
{
   int n;
   int j;
   vect_n * torque_vars;
   vect_n * position_vars;
   int dof;
   
   dof = len_vn(torques);
   
   torque_vars = new_vn(dof);
   position_vars = new_vn(dof);
   
   /* Zero the values */
   fill_vn(torques,0.0);
   fill_vn(positions,0.0);
   fill_vn(torque_vars,0.0);
   fill_vn(position_vars,0.0);
   
   /* Calculate the average */
   for (n = 0; n < NUM_POINTS; n++)
   {
      for (j=0; j<dof; j++)
      {
         torques->q[j] += mu_jts[j][n];
         positions->q[j] += mu_jps[j][n];
      }
   }
   set_vn(torques, scale_vn( 1.0/NUM_POINTS, torques ));
   set_vn(positions, scale_vn( 1.0/NUM_POINTS, positions ));
      
   /* Calculate the variance */
   for (n = 0; n < NUM_POINTS; n++)
   {
      for (j=0; j<dof; j++)
      {   
         torque_vars->q[j] += (mu_jts[j][n] - torques->q[j]) * (mu_jts[j][n] - torques->q[j]);
         position_vars->q[j] += (mu_jps[j][n] - positions->q[j]) * (mu_jps[j][n] - positions->q[j]);
      }
   }
   set_vn(torque_vars, scale_vn( 1.0/NUM_POINTS, torque_vars ));
   set_vn(position_vars, scale_vn( 1.0/NUM_POINTS, position_vars ));
   
   /* Print positions */
   mvprintw(12,0, "  Positions:");
   mvprintw(13,0, "  (std-dev):");
   mvprintw(14,0, "    Torques:");
   mvprintw(15,0, "  (std-dev):");
   for (j=0; j<dof; j++)
   {
      mvprintw(12, 13 + 9*j, "% 08.5f ",positions->q[j]);
      mvprintw(13, 13 + 9*j, "% 08.5f ",sqrt(position_vars->q[j]));
      mvprintw(14, 13 + 9*j, "% 08.5f ",torques->q[j]);
      mvprintw(15, 13 + 9*j, "% 08.5f ",sqrt(torque_vars->q[j]));
   }
   
   destroy_vn(&torque_vars);
   destroy_vn(&position_vars);
   return 0;
}

int do_mode_mu()
{
   int err;         /* Generic error variable for function calls */
   int busCount;    /* Number of WAMs defined in the configuration file */
   int j;
   
   /* Stuff that's filled in once */
   vect_n ** poses;
   int num_poses;
   int pose;
   vect_n * angle_diff;
   
   /* Stuff that's used once for each pose in measure mode */
   vect_n * torques_top;
   vect_n * positions_top;
   vect_n * torques_bottom;
   vect_n * positions_bottom;
   
   /* For storing the results from measure mode, one vector per pose  */
   vect_n ** torques;
   vect_n ** positions;
   /* For storing the results from computation mode, one vector per joint */
   vect_3 ** mus;
   
   /* Make a list of phases for each pose*/
   enum {
      MU_P_START,
      MU_P_TO_TOP,
      MU_P_FROM_TOP,
      MU_P_MEAS_TOP,
      MU_P_TO_BOT,
      MU_P_FROM_BOT,
      MU_P_MEAS_BOT,
      MU_P_DONE
   } phase;
   char * phasenm[] = {
      "START",
      "TO_TOP",
      "FROM_TOP",
      "MEAS_TOP",
      "TO_BOT",
      "FROM_BOT",
      "MEAS_BOT",
      "DONE"
   };
   
   clear();
   mvprintw(0,0,"Starting Gravity Calibration Mode");
   
   /* Ensure the WAM is set up, power cycled, and in the home position */
   mvprintw(2,0,"To begin the calibration, follow the following steps:");
   mvprintw(3,0,"  a) Ensure that all WAM power and signal cables are securely fastened.");
   mvprintw(4,0,"  b) Ensure that the WAM is powered on (receiving power).");
   mvprintw(5,0,"  c) Ensure that all E-STOPs are released.");
   mvprintw(6,0,"  d) Place the WAM in Shift+Idle mode.");
   mvprintw(7,0,"  e) Carefully ensure that the WAM is in its home (folded) position.");
   mvprintw(8,0,"Press [Enter] to continue.");
   refresh();
   while (btkey_get()!=BTKEY_ENTER) usleep(10000);
   
   /* Initialize system buses */
   err = ReadSystemFromConfig("../../wam.conf", &busCount);
   if(err) return -1;
   
   /* Spin off the CAN thread */
   startDone = 0;
   btrt_thread_create(&can_thd,"can",45,(void*)can_thd_function,NULL);
   while (!startDone) usleep(10000);
   
   /* Allocate the global variables */
   mu_jts = (double **) malloc( wam->dof * sizeof(double *) );
   mu_jps = (double **) malloc( wam->dof * sizeof(double *) );
   for (j=0; j<wam->dof; j++)
   {
      mu_jts[j] = (double *) malloc( NUM_POINTS * sizeof(double) );
      mu_jps[j] = (double *) malloc( NUM_POINTS * sizeof(double) );
   }
   
   /* Spin off the WAM thread */
   mu_n = NUM_POINTS;
   wam_thd.period = 0.002; /* Control loop period in seconds */
   registerWAMcallback(wam, mu_callback);
   btrt_thread_create(&wam_thd,"ctrl",90,(void*)WAMControlThread,(void*)wam);
   
   /* Grab poses from the configuration file */
   do {
      btparser parser;
      char key[256];
      
      err = btParseFile(&parser,"cal.conf");
      if (err)
      {
         syslog(LOG_ERR,"Calibration configuration file cal.conf not found.");
         printf("Calibration configuration file cal.conf not found.\n");
         break;
      }
      
      sprintf(key,"calibration-poses-wam%d.poseCount",wam->dof);
      err = btParseGetVal(&parser,INT, key, &num_poses);
      if (err || num_poses < 4)
      {
         syslog(LOG_ERR,"Configuration group calibration-poses-wam%d not found,",wam->dof);
         syslog(LOG_ERR,"numPoses not present, or numPoses not at least 4.");
         printf("Configuration group calibration-poses-wam%d not found,\n",wam->dof);
         printf("numPoses not present, or numPoses not at least 4.");
         btParseClose(&parser);
         break;
      }
      
      poses = (vect_n **) btmalloc( num_poses * sizeof(vect_n *) );
      for (pose=0; pose<num_poses; pose++)
      {
         poses[pose] = new_vn(wam->dof);
         sprintf(key, "calibration-poses-wam%d.pose[%d]", wam->dof, pose);
         err = btParseGetVal(&parser, VECTOR, key, (void*)poses[pose]);
         if (err)
         {
            syslog(LOG_ERR,"Not enough poses found! (%d expected, found only %d)",
                   num_poses, pose);
            printf("Not enough poses found! (%d expected, found only %d)\n",
                   num_poses, pose);
            break;
         }
      }
      
      btParseClose(&parser);
   } while (0);
   
   if (err)
   {
      /* Free the variables */
      for (j=0; j<wam->dof; j++)
      {
         free(mu_jts[j]);
         free(mu_jps[j]);
      }
      free(mu_jts);
      free(mu_jps);
      /* Stop threads ... */
      wam_thd.done = 1;
      usleep(10000);
      can_thd.done = 1;
      usleep(50000);
      /* End ncurses */
      endwin();
      return 0;
   }
   
   angle_diff = new_vn(wam->dof);
   fill_vn(angle_diff,ANGLE_DIFF);
   torques_top = new_vn(wam->dof);
   positions_top = new_vn(wam->dof);
   torques_bottom = new_vn(wam->dof);
   positions_bottom = new_vn(wam->dof);
   torques = (vect_n **) malloc( num_poses * sizeof(vect_n *) );
   positions = (vect_n **) malloc( num_poses * sizeof(vect_n *) );
   for (pose=0; pose<num_poses; pose++)
   {
      torques[pose] = new_vn(wam->dof);
      positions[pose] = new_vn(wam->dof);
   }
   mus = (vect_3 **) malloc( wam->dof * sizeof(vect_3 *) );
   for (j=0; j<wam->dof; j++)
      mus[j] = new_v3();
   
   /* Allow the user to shift-activate */
   mvprintw(10,0,"IMPORTANT: Once gravity compensation begins, the WAM will begin");
   mvprintw(11,0,"to move to a set of %d predefined poses (defined in cal.conf).",num_poses);
   
   mvprintw(13,0,"DO NOT TOUCH the WAM during the measurement process, or the");
   mvprintw(14,0,"calibration computations will be sigificantly wrong, and");
   mvprintw(15,0,"any subsequent gravity compensation will fail spectacularly!");
   
   mvprintw(17,0,"Place the WAM in Shift+Activate mode,");
   mvprintw(18,0,"and press [Enter] to start.");
   refresh();
   while (btkey_get()!=BTKEY_ENTER) usleep(10000);
   
   /* Start the GUI! */
   pose = 0;
   phase = MU_P_START;
   
   clear();
   mvprintw(1,0,"Note: Press [Control+C] at any time to cancel the calibration.");
   mvprintw(2,0,"DO NOT TOUCH the WAM during the calibration process!");
   done = 0;
   signal(SIGINT, sigint);  
   while (!done)
   {
      
      /* Print current state */
      mvprintw(0, 0,"Current Pose: %d of %d.  ",pose+1,num_poses);
      mvprintw(0,30,"Current Phase: %s        ",phasenm[phase]);
      
      /* Print current joint position and torque */
      mvprintw(4,0, "      Joint:");
      mvprintw(5,0, "   Position:");
      mvprintw(6,0, "     Torque:");
      for (j=0; j<wam->dof; j++)
      {
         mvprintw(4, 13 + 9*j, " Joint %d ",j+1);
         mvprintw(5, 13 + 9*j, "% 08.5f ",wam->Jpos->q[j]);
         mvprintw(6, 13 + 9*j, "% 08.4f ",wam->Jtrq->q[j]);
      }
      
      /* Line 9 - Status Updates */
      mvprintw(8,0,"Current Status:");
      
      /* Note - lines 12-?? reserved for printing statistics */
      mvprintw(11,0,"Recent Statistics:");
      
      refresh();
      usleep(5E4);
      
      /* Move through the state machine */
      switch (phase)
      {
         case MU_P_START:
            mvprintw(9,3,"Moving to above position ...           ");
            MoveSetup(wam, 1.0, 1.0);
            MoveWAM(wam,add_vn(poses[pose],angle_diff));
            phase++;
            break;
         case MU_P_TO_TOP:
            if (!MoveIsDone(wam)) break;
            mvprintw(9,3,"Moving to position (from above) ...    ");
            MoveSetup(wam, 0.05, 0.05);
            MoveWAM(wam,poses[pose]);
            phase++;
         case MU_P_FROM_TOP:
            if (!MoveIsDone(wam)) break;
            mvprintw(9,3,"Starting a measurement ...             ");
            mu_n = 0;
            phase++;
            break;
         case MU_P_MEAS_TOP:
            if (mu_n < NUM_POINTS) break;
            mu_stats( torques_top, positions_top );
            mvprintw(9,3,"Moving to below position ...           ");
            MoveSetup(wam, 1.0, 1.0);
            MoveWAM(wam,sub_vn(poses[pose],angle_diff));
            phase++;
            break;
         case MU_P_TO_BOT:
            if (!MoveIsDone(wam)) break;
            mvprintw(9,3,"Moving to position (from below) ...    ");
            MoveSetup(wam, 0.05, 0.05);
            MoveWAM(wam,poses[pose]);
            phase++;
            break;
         case MU_P_FROM_BOT:
            if (!MoveIsDone(wam)) break;
            mvprintw(9,3,"Starting a measurement ...             ");
            mu_n = 0;
            phase++;
            break;
         case MU_P_MEAS_BOT:
            if (mu_n < NUM_POINTS) break;
            mu_stats( torques_bottom, positions_bottom );
            phase++;
            break;
         case MU_P_DONE:
            /* Get the midpoint position and torque ... */
            set_vn(torques[pose],scale_vn(0.5,add_vn(torques_top,torques_bottom)));
            set_vn(positions[pose],scale_vn(0.5,add_vn(positions_top,positions_bottom)));
            pose++;
            phase = MU_P_START;
            if (pose == num_poses)
               done = 1;
            break;
      }
   }
   
   /* Re-fold, print, and exit */
   clear();
   mvprintw(0,0,"Moving back to the park location ...");
   refresh();
   MoveSetup(wam, 1.0, 1.0);
   MoveWAM(wam,wam->park_location);
   while (!MoveIsDone(wam)) usleep(10000);
   mvprintw(1,0,"Shift+Idle, and press [Enter] to continue.");
   refresh();
   while (btkey_get()!=BTKEY_ENTER) usleep(10000);
   
   /* Stop threads ... */
   wam_thd.done = 1;
   usleep(10000);
   can_thd.done = 1;
   usleep(50000);
   
   /* Free unneeded variables */
   for (j=0; j<wam->dof; j++)
   {
      free(mu_jts[j]);
      free(mu_jps[j]);
   }
   free(mu_jts);
   free(mu_jps);
   destroy_vn(&angle_diff);
   destroy_vn(&torques_top);
   destroy_vn(&positions_top);
   destroy_vn(&torques_bottom);
   destroy_vn(&positions_bottom);
   
   /* End ncurses */
   endwin();
   
   if (done == 1)
   {
      /* Here we have the "Iterative Algorithm"
       * described in the Chris Dellin document entitled
       * "Newton-Euler First-Moment Gravity Compensation" */

      /* Here we have the matrix composed of L matrices (negative) */
      matr_mn * nLL;
      
      /* We have a GT matrix and a Y vector for each link */
      matr_mn ** GT;
      vect_n ** Y;
      
      /* We have a solution vector P for each link */
      vect_n ** P;
      
      vect_n * grav_torques;
      
      /* Start calculating ...
       * We have vectors of torque and position
       * in torques[] and positions[] */
      printf("\n");
      printf("Calculating ...\n");
      
      /* Make the LL matrix */
      nLL = new_mn( 3*num_poses, 3+2*num_poses );
      zero_mn(nLL);
      for (pose=0; pose<num_poses; pose++)
      {
         setval_mn(nLL, 3*pose+0, 3+2*pose+0, -1.0 );
         setval_mn(nLL, 3*pose+1, 3+2*pose+1, -1.0 );
      }
        
      /* Make each link's GT matrix */
      /* Make each link's Y torque solution matrix */
      Y = (vect_n **) btmalloc( wam->dof * sizeof(vect_n *) );
      GT = (matr_mn **) btmalloc( wam->dof * sizeof(matr_mn *) );
      for (j=0; j<wam->dof; j++)
      {
         Y[j] = new_vn( 3*num_poses );
         GT[j] = new_mn( 3*num_poses, 3+2*num_poses );
         zero_mn(GT[j]);
      }
      grav_torques = new_vn(wam->dof);
      for (pose=0; pose<num_poses; pose++)
      {
         /* Put the robot in the pose (by position) */
         set_vn(wam->robot.q,positions[pose]);
         eval_fk_bot(&wam->robot);
         get_gravity_torques(&wam->robot,grav_torques);
         for (j=0; j<wam->dof; j++)
         {
            /* GT: Gravity skew matrix */
            setval_mn(GT[j], 3*pose+0, 1, -getval_v3(wam->robot.links[j].g,2) );
            setval_mn(GT[j], 3*pose+0, 2,  getval_v3(wam->robot.links[j].g,1) );
            setval_mn(GT[j], 3*pose+1, 0,  getval_v3(wam->robot.links[j].g,2) );
            setval_mn(GT[j], 3*pose+1, 2, -getval_v3(wam->robot.links[j].g,0) );
            setval_mn(GT[j], 3*pose+2, 0, -getval_v3(wam->robot.links[j].g,1) );
            setval_mn(GT[j], 3*pose+2, 1,  getval_v3(wam->robot.links[j].g,0) );
            /* GT: -R*L */
            setval_mn(GT[j], 3*pose+0, 3+2*pose+0, -getval_mh(wam->robot.links[j].trans,0,0) );
            setval_mn(GT[j], 3*pose+0, 3+2*pose+1, -getval_mh(wam->robot.links[j].trans,1,0) );
            setval_mn(GT[j], 3*pose+1, 3+2*pose+0, -getval_mh(wam->robot.links[j].trans,0,1) );
            setval_mn(GT[j], 3*pose+1, 3+2*pose+1, -getval_mh(wam->robot.links[j].trans,1,1) );
            setval_mn(GT[j], 3*pose+2, 3+2*pose+0, -getval_mh(wam->robot.links[j].trans,0,2) );
            setval_mn(GT[j], 3*pose+2, 3+2*pose+1, -getval_mh(wam->robot.links[j].trans,1,2) );
            /* Y */
            setval_vn(Y[j], 3*pose+0, getval_vn(torques[pose],j) * getval_mh(wam->robot.links[j].trans,2,0) );
            setval_vn(Y[j], 3*pose+1, getval_vn(torques[pose],j) * getval_mh(wam->robot.links[j].trans,2,1) );
            setval_vn(Y[j], 3*pose+2, getval_vn(torques[pose],j) * getval_mh(wam->robot.links[j].trans,2,2) );
            if (j < wam->dof-1)
               setval_vn(Y[j], 3*pose+2, getval_vn(Y[j],3*pose+2) - getval_vn(torques[pose],j+1) );
         }
      }
      destroy_vn(&grav_torques);
      
      /* Make a space for each link's P solution vector */
      P = (vect_n **) btmalloc( wam->dof * sizeof(vect_n *) );
      for (j=0; j<wam->dof; j++)
         P[j] = new_vn( 3+2*num_poses );
      
      /* Do the regression for each link */
      {
         int i;
         matr_mn * m2x2;
         matr_mn * m2x3;
         vect_n * Yi;
         vect_n * index;
         vect_n * col;
         
         m2x2 = new_mn( 3+2*num_poses, 3+2*num_poses );
         m2x3 = new_mn( 3+2*num_poses,   3*num_poses );
         Yi = new_vn( 3*num_poses );
         index = new_vn( 3+2*num_poses );
         col = new_vn( 3+2*num_poses );
         
         for (j=wam->dof-1; j>=0; j--)
         {
            mul_mn( m2x2, T_mn(GT[j]), GT[j] );
            for (i=0; i<3+2*num_poses; i++)
               setval_mn(m2x2,i,i, getval_mn(m2x2,i,i) + CALC_LAMBDA );
            mul_mn( m2x3, inv_mn(m2x2,index,col), T_mn(GT[j]) );
            if (j < wam->dof-1)
               matXvec_mn(nLL,P[j+1],Yi);
            else
               fill_vn(Yi,0.0);
            set_vn(Yi, add_vn(Yi,Y[j]) );
            matXvec_mn( m2x3, Yi, P[j] );
         }
         
         destroy_mn(&m2x2);
         destroy_mn(&m2x3);
         destroy_vn(&Yi);
         destroy_vn(&index);
         destroy_vn(&col);
         
         /* Copy the results */
         for (j=0; j<wam->dof; j++)
         {
            setval_v3(mus[j],0, getval_vn(P[j],0) );
            setval_v3(mus[j],1, getval_vn(P[j],1) );
            setval_v3(mus[j],2, getval_vn(P[j],2) );
         }
         
         /* Clean up */
         destroy_mn(&nLL);
         for (j=0; j<wam->dof; j++)
         {
            destroy_mn(&GT[j]);
            destroy_vn(&Y[j]);
            destroy_vn(&P[j]);
         }
         btfree((void **)&GT);
         btfree((void **)&Y);
         btfree((void **)&P);
      }
      
      /* Print results */
      printf("\n");
      printf("Gravity calibration ended.\n");
      printf("\n");
      printf("Copy the following lines into your wam.conf file, in the %s{} group.\n",wam->name);
      printf("It is usually placed above the safety{} group.\n");
      printf("--------\n");
      printf("    # Calibrated gravity vectors\n");
      printf("    calibrated-gravity{\n");
      for (j=0; j<wam->dof; j++)
      {
         printf("        mu%d = < % .6f, % .6f, % .6f >\n", j+1,
                getval_v3(mus[j],0),
                getval_v3(mus[j],1),
                getval_v3(mus[j],2));
      }
      printf("    }\n");
      printf("--------\n");
      {
         FILE * logfile;
         logfile = fopen("cal-gravity.log","w");
         if (logfile)
         {
            fprintf(logfile,"    # Calibrated gravity vectors\n");
            fprintf(logfile,"    calibrated-gravity{\n");
            for (j=0; j<wam->dof; j++)
               fprintf(logfile,"        mu%d = < % .6f, % .6f, % .6f >\n", j+1,
                               getval_v3(mus[j],0),
                               getval_v3(mus[j],1),
                               getval_v3(mus[j],2));
            fprintf(logfile,"    }\n");
            fclose(logfile);
            printf("This text has been saved to cal-gravity.log.\n");
         }
         else
         {
            syslog(LOG_ERR,"Could not write to cal-gravity.log.");
            printf("Error: Could not write to cal-gravity.log.\n");
         }
      }
      printf("\n");
      
   }
   
   /* Free the variables */
   for (pose=0; pose<num_poses; pose++)
   {
      destroy_vn(&torques[pose]);
      destroy_vn(&positions[pose]);
   }
   free(torques);
   free(positions);
   for (j=0; j<wam->dof; j++)
      destroy_vn((vect_n **)&mus[j]);
   free(mus);
   
   return 0;
}

