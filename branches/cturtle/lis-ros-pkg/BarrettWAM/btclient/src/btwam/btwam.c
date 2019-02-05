/* ======================================================================== *
 *  Module ............. libbwam
 *  File ............... btwam.h
 *  Creation Date ...... 15 Feb 2003
 *  Author ............. Traveler Hauptman
 *  Addtl Authors ...... Brian Zenowich
 *                       Sam Clanton
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

/** \file btwam.c
    \brief Provides functions for controlling a WAM.
 
    The btwam library provides high-level control functions for the Barrett
    WAM. It contains the WAM control loop, plus high-level movement commands
    and Teach & Play routines. It depends on the btsystem library for actuator-
    level communication, kinematics, dynamics, path following, position control,
    config file parsing, data logging, and matrix/vector math.
 
See #btwam_struct
 
*/
#define TWOPI (2.0 * 3.14159265359)
#define DEBUG(x) x

/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#ifdef S_SPLINT_S
#include <err.h>
#endif
#include <syslog.h>
#include <stdio.h>
#include <math.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/stat.h>

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#include "btwam.h"

/*==============================*
 * GLOBAL file-scope variables  *
 *==============================*/
//wam_struct WAM;
extern int gimbalsInit;
extern bus_struct *buses;
int numMaintThreads = 0;

//#define LOW_TRQ_COEFFICIENT (0.75)
//#define BTDOUBLETIME

/*==============================*
 * Internal use Functions       *
 *==============================*/
void WAMMaintenanceThread(void *data);
int ActAngle2Mpos(wam_struct *wam,vect_n *Mpos); //Extracts actuators 0 thru 6 into vect_n positions
int Mtrq2ActTrq(wam_struct *wam,vect_n *Mtrq);  //Packs vect_n of torques into actuator array
void Mpos2Jpos(wam_struct *wam,vect_n * Mpos, vect_n * Jpos); //convert motor angle to joint angle
void Jpos2Mpos(wam_struct *wam,vect_n * Jpos, vect_n * Mpos);
void Jtrq2Mtrq(wam_struct *wam,vect_n * Jtrq, vect_n * Mtrq); //convert joint torque to motor torque
void InitVectors(wam_struct *wam);
void GetJointPositions();
void SetJointTorques();
void DumpWAM2Syslog();
int BlankWAMcallback(struct btwam_struct *wam);


/*==============================*
 * Functions                    *
 *==============================*/
 
/* For Debugging: This function will execute when a RealTime thread switches to 
 * secondary mode.  Also, one can look at /proc/xenomai/stat and MSW will 
 * tell you the number of switches a thread has made.
 */
void warn_upon_switch(int sig __attribute__((unused)))
{
    void *bt[32];
    int nentries;
    int fd;
 
     // Open backtrace file as APPEND. Create new file if necessary (chmod 644)
    fd = open("sigxcpu.txt", O_WRONLY | O_APPEND | O_CREAT, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    
    /* Dump a backtrace of the frame which caused the switch to secondary mode
       To decipher a backtrace:
       1) Look at the line of the backtrace originating in your application (not a library,
          and not the warn_upon_switch function)
       2) Take note of the memory address of that line, ex: [0x80588d5]
       3) Run "gdb <progname>"
       4) Type "list *0x80588d5" (substitute your actual memory address)
       5) gdb will show the function() (file:line) information for your error, followed by 
          a listing of several surrounding lines
    */
    nentries = backtrace(bt, sizeof(bt) / sizeof(bt[0]));
    backtrace_symbols_fd(bt, nentries, fd);
    write(fd, "-----\n", 6); // Output a separator line
    
    close(fd);
    syslog(LOG_ERR, "Switched out of RealTime, see sigxcpu.txt for details (frame backtrace)");
}

/** Allocate memory for components of the WAM data structure
 
  This function allocates and initializes memory for many vectors and matrices
  defined in the #btwam_struct.
  
  \param wam Pointer to a WAM structure: wam = (wam_struct*)btmalloc(sizeof(wam_struct));
  
  See #btwam_struct for detailed information on the WAM data structure.
*/
void InitVectors(wam_struct *wam)
{
   wam->zero_offsets = new_vn(wam->dof);
   wam->stop_torque = new_vn(wam->dof);
   wam->park_location = new_vn(wam->dof);
   wam->Mpos = new_vn(wam->dof);
   wam->N = new_vn(wam->dof);
   wam->n = new_vn(wam->dof);
   wam->M2JP = new_mn(wam->dof,wam->dof);
   wam->J2MT = new_mn(wam->dof,wam->dof);
   wam->J2MP = new_mn(wam->dof,wam->dof);
   wam->Mtrq  = new_vn(wam->dof);
   wam->Jpos = new_vn(wam->dof);
   wam->Jvel = new_vn(wam->dof);
   wam->Jacc = new_vn(wam->dof);
   wam->Jref = new_vn(wam->dof);
   wam->Jtref = new_vn(wam->dof);
   wam->Jtrq = new_vn(wam->dof);
   wam->Ttrq = new_vn(wam->dof);
   wam->Gtrq = new_vn(wam->dof);
   //wam->Kp = new_vn(wam->dof);
   //wam->Kd = new_vn(wam->dof);
   //wam->Ki = new_vn(wam->dof);
   //wam->saturation = new_vn(wam->dof);
   wam->torq_limit = new_vn(wam->dof);
   wam->vel = new_vn(wam->dof);
   wam->acc = new_vn(wam->dof);

   /* 3-axis Cartesian position data */
   wam->Cpos = new_v3();
   wam->Cvel = new_v3();
   wam->Cacc = new_v3();
   wam->Cpoint = new_v3();
   //wam->Cref = new_v3();
   wam->Cforce = new_v3();
   wam->Ctrq = new_v3();
   //wam->Ckp = new_v3();
   //wam->Ckd = new_v3();
   //wam->Cki = new_v3();
   //wam->use_new = 0;
   /* Quaternian data (unused) */
   wam->qref = new_q();
   wam->qact = new_q();
   wam->qaxis = new_q();
   wam->forced = new_q();
   wam->qerr = 0;

   /* Homogeneous Matrix state variables for Cartesian tool control */
   wam->HMpos = wam->robot.tool->origin; // Position
   wam->HMvel = new_mh(); // Velocity
   wam->HMacc = new_mh(); // Acceleration
   wam->HMref = new_mh(); // Position reference
   wam->HMtref = new_mh(); // Trajectory reference
   wam->HMft = new_mh(); // Force/torque matrix

   //wam->js_or_cs = 0;
   wam->idle_when_done = 0;
   wam->active_sc = &wam->Jsc;

   wam->motor_position = (int*)btmalloc(wam->dof * sizeof(int));
   wam->zero_order = (int*)btmalloc(wam->dof * sizeof(int));
   wam->JposControl.pid = (btPID*)btmalloc(wam->dof * sizeof(btPID));
   wam->JposControl.elements = wam->dof;
   wam->CposControl.pid = (btPID*)btmalloc(12 * sizeof(btPID));
   wam->CposControl.elements = 12;
   //wam->sc = (SimpleCtl*)btmalloc(wam->dof * sizeof(SimpleCtl));

   //wam->G = new_vn(wam->dof);
}

/** Initialize a WAM
 
  This function calls the necessary lower-level btsystem functions to set
  up the pucks, enumerate them and scan in the WAM configuration information.
 
  This function may kill the process using exit() if it runs into problems.
  
  \param fn WAM configuration filename
  \param bus Index into the bus data structure ("System" order in wam.conf)
  \retval NULL Some error occured during initialization. Check /var/log/syslog.
  \retval wam_struct* Pointer to wam data structure.
  
  See #btwam_struct
*/
wam_struct* OpenWAM(char *fn, int bus)
{
   int            cnt, ret, err;
   btreal         theta, d, a, alpha, mass;
   wam_struct     *wam;
   char           key[256];
   long           reply;
   int            link;
   int            actcnt = 0;
   vect_3         tmp_v3;
   char           thd_maintname[5];


   /* Normally, you should use init_local_vn() for this, but in this case
      we want a vect_3 structure without assigning the data pointer (it will
      be assigned later).
    */
   tmp_v3.n = 3;

   // If no config file was given
   if(!*fn) { 
      strcpy(key, "wam.conf"); // Default to wam.conf
      fn = key;
   }
   
   // Parse config file
   err = parseFile(fn);
   if(err) {
      syslog(LOG_ERR, "OpenWAM: Error parsing config file- %s", fn);
      return(NULL);
   }

   // Carve out some memory for the main data structure
   wam = (wam_struct*)btmalloc(sizeof(wam_struct));
   if(wam == NULL){
      syslog(LOG_ERR, "OpenWAM: Unable to allocate memory for wam_struct");
      return(NULL);
   }

   // Extract the robot name from the config file
   //sprintf(key, "system.bus[%d].name", bus);
   //parseGetVal(STRING, key, (void*)robotName);

   // The robot name is already in buses[].device_name from InitializeSystem()
   syslog(LOG_ERR, "device_name=%s", buses[bus].device_name);
   strcpy(wam->name, buses[bus].device_name); // Set the name in the wam struct
   //strcpy(robotName, wam->name);

   syslog(LOG_ERR, "wam->name=%s", wam->name);

   // Read Degrees of Freedom
   sprintf(key, "%s.dof", wam->name);
   parseGetVal(INT, key, (void*)&wam->dof);

   // Initialize the generic robot data structure
   new_bot(&wam->robot,wam->dof);

   // Allocate memory for the WAM vectors
   InitVectors(wam);

   /* START of Joint Control plugin initialization */
   // Initialize the PID data structures for joint control
   for (cnt = 0; cnt < wam->dof; cnt ++) {
      init_btPID(&wam->JposControl.pid[cnt]);
   }

   // Initialize a state controller
   init_bts(&wam->Jsc);

   /* Assign input and output data locations to the state controller.
    * Jpos, Jvel, Jacc are the joint state inputs to the controller
    * Jref is a command position
    * Jtref is a trajectory reference value (used by the trajectory generator)
    * Jtrq is the controller output
    * dt is the time step used by the controller
    */
   map_btstatecontrol(&wam->Jsc, wam->Jpos, wam->Jvel, wam->Jacc,
                      wam->Jref, wam->Jtref, wam->Jtrq, &wam->dt);

   /* Assign a position control method to the controller.
    * This mapf function simply calls mapposition_bts() to assign the control
    * data along with a _reset, _eval, and _pause function to the state controller.
    * If you use your own position control method, you will want to define a new
    * btposition_interface_mapf_yourControl() function, plus your own control
    * data structure, and your own _reset, _eval, and _pause functions for your
    * control scheme. See the btPID code in the first 400 lines of btcontrol.c
    * to understand how we define these components for a simple PID controller.
    */
   btposition_interface_mapf_btPID(&wam->Jsc, &wam->JposControl);
   /* END of Joint Control plugin initialization */


   /* START of Cartesian Control plugin initialization */
   // Initialize the PID data structures for joint control
   for (cnt = 0; cnt < 12; cnt ++) {
      // Note: Only the Position (4th) column (XYZ) of the robot homogeneous matrix
      // is controlled by this statecontrol system. The rotations are controlled
      // directly from the WAMControlLoop() because of the required vector cross-products.
      init_btPID(&wam->CposControl.pid[cnt]);
   }
   
   // Initialize a state controller
   init_bts(&wam->Csc);

   // Assign input and output data locations to the state controller.
   map_btstatecontrol(&wam->Csc, (vect_n*)wam->HMpos, (vect_n*)wam->HMvel, (vect_n*)wam->HMacc,
                      (vect_n*)wam->HMref, (vect_n*)wam->HMtref, (vect_n*)wam->HMft, &wam->dt);

   // Assign a position control method to the controller.
   btposition_interface_mapf_btPID(&wam->Csc, &wam->CposControl);
   /* END of Control plugin initialization */

   wam->isZeroed = FALSE;
   wam->force_callback = BlankWAMcallback;
	wam->motor_callback = BlankWAMcallback;
   
   wam->cteach.Log_Data = 0; // Default to logOff
   wam->teachDivider = 1;
   wam->teachCounter = 0;
   wam->teach_time = 0.0;
   
   wam->log.Log_Data = 0; // Default to logOff
   wam->logDivider = 1;
   wam->logCounter = 0;
   wam->log_time = 0.0;

   SetEngrUnits(1);

   err = GetActuators(bus, &wam->act, &wam->num_actuators);
   //syslog(LOG_ERR, "bus=%d, num_actuators=%d", bus, wam->num_actuators);

   /* START reading robot data from configuration file */
   // Read park_location
   sprintf(key, "%s.home", wam->name);
   parseGetVal(VECTOR, key, (void*)wam->park_location);

   // Read torq_limit
   sprintf(key, "%s.tlimit", wam->name);
   parseGetVal(VECTOR, key, (void*)wam->torq_limit);

   // Read the worldframe->origin transform matrix
   sprintf(key, "%s.world", wam->name);
   parseGetVal(MATRIX, key, (void*)wam->robot.world->origin);

   // Read the Cartesian position control gains
   sprintf(key, "%s.Px_pdi", wam->name);
   tmp_v3.q = &wam->CposControl.pid[3].Kp;
   parseGetVal(VECTOR, key, (void*)&tmp_v3);
   
   sprintf(key, "%s.Py_pdi", wam->name);
   tmp_v3.q = &wam->CposControl.pid[7].Kp;
   parseGetVal(VECTOR, key, (void*)&tmp_v3);
   
   sprintf(key, "%s.Pz_pdi", wam->name);
   tmp_v3.q = &wam->CposControl.pid[11].Kp;
   parseGetVal(VECTOR, key, (void*)&tmp_v3);
   
   // Read the Cartesian rotation control gains
   // Note that pid[0,5,10] are just storage locations for Rx, Ry, and Rz gains
   // The actual control is performed in the WAMControlLoop()
   sprintf(key, "%s.Rx_pd", wam->name);
   tmp_v3.q = &wam->CposControl.pid[0].Kp;
   parseGetVal(VECTOR, key, (void*)&tmp_v3);
   
   sprintf(key, "%s.Ry_pd", wam->name);
   tmp_v3.q = &wam->CposControl.pid[5].Kp;
   parseGetVal(VECTOR, key, (void*)&tmp_v3);
   
   sprintf(key, "%s.Rz_pd", wam->name);
   tmp_v3.q = &wam->CposControl.pid[10].Kp;
   parseGetVal(VECTOR, key, (void*)&tmp_v3);
   
   // Read whether M4 is reversed (new as of 2007 WAMs)
   //sprintf(key, "%s.M4_2007", robotName);
   //wam->M4_reversed = 0; // Default to "pre-2007"
   //parseGetVal(INT, key, (void*)&wam->M4_reversed);

   // Read the Transmission ratios
   sprintf(key, "%s.N", wam->name);
   parseGetVal(VECTOR, key, (void*)wam->N);
   sprintf(key, "%s.n", wam->name);
   parseGetVal(VECTOR, key, (void*)wam->n);

   //Read the motor->joint position transmission matrix
   sprintf(key, "%s.m2jp", wam->name);
   parseGetVal(MATRIX, key, (void*)wam->M2JP);

   //Read the joint->motor position transmission matrix
   sprintf(key, "%s.j2mp", wam->name);
   parseGetVal(MATRIX, key, (void*)wam->J2MP);

   //Read the joint->motor torque transmission matrix
   sprintf(key, "%s.j2mt", wam->name);
   parseGetVal(MATRIX, key, (void*)wam->J2MT);

   //Assume we're gravity-calibrated until we can't find a mu vector ...
   wam->robot.gcal_iscalibrated = 1;

   tmp_v3.q = tmp_v3.data; // Set pointer to its own storage area
   syslog(LOG_ERR,"OpenWAM(): for link from 0 to %d", wam->dof);
   for(link = 0; link <= wam->dof; link++) {
      // Get the DH parameters
      sprintf(key, "%s.link[%d].dh.theta", wam->name, link);
      parseGetVal(DOUBLE, key, (void*)&theta);
      sprintf(key, "%s.link[%d].dh.d", wam->name, link);
      parseGetVal(DOUBLE, key, (void*)&d);
      sprintf(key, "%s.link[%d].dh.a", wam->name, link);
      parseGetVal(DOUBLE, key, (void*)&a);
      sprintf(key, "%s.link[%d].dh.alpha", wam->name, link);
      parseGetVal(DOUBLE, key, (void*)&alpha);

      // Get the mass parameters
      sprintf(key, "%s.link[%d].com", wam->name, link);
      parseGetVal(VECTOR, key, (void*)&tmp_v3);
      sprintf(key, "%s.link[%d].mass", wam->name, link);
      parseGetVal(DOUBLE, key, (void*)&mass);

      // Get inertia matrix
      sprintf(key, "%s.link[%d].I", wam->name, link);
      parseGetVal(MATRIX, key, (void*)wam->robot.links[link].I);

      sprintf(key, "%s.link[%d].rotorI", wam->name, link);
      parseGetVal(VECTOR, key, (void*)wam->robot.links[link].rotorI);

      // Add the rotor inertia to the link inertia
      for(cnt = 0; cnt < 3; cnt++)
         ELEM(wam->robot.links[link].I, cnt, cnt) += wam->robot.links[link].rotorI->q[cnt];

      // If this is not the tool...
      if(link != wam->dof) {
         // Get first-moment-of-the-mass vector for calibrated gravity stuff,
         // if it's in the config file
         if ( wam->robot.gcal_iscalibrated )
         {
            sprintf(key, "%s.calibrated-gravity.mu%d", wam->name, link+1);
            err = parseGetVal(VECTOR, key, (void*)wam->robot.links[link].gcal_mu);
            if (err) wam->robot.gcal_iscalibrated = 0;
         }
         
         // Read joint PID parameters
         sprintf(key, "%s.link[%d].pid.kp", wam->name, link);
         parseGetVal(DOUBLE, key, (void*)&wam->JposControl.pid[link].Kp);
         //parseGetVal(DOUBLE, key, (void*)&(valptr_vn(wam->Kp)[link]));

         sprintf(key, "%s.link[%d].pid.kd", wam->name, link);
         parseGetVal(DOUBLE, key, (void*)&wam->JposControl.pid[link].Kd);
         //parseGetVal(DOUBLE, key, (void*)&(valptr_vn(wam->Kd)[link]));

         sprintf(key, "%s.link[%d].pid.ki", wam->name, link);
         parseGetVal(DOUBLE, key, (void*)&wam->JposControl.pid[link].Ki);
         //parseGetVal(DOUBLE, key, (void*)&(valptr_vn(wam->Ki)[link]));

         sprintf(key, "%s.link[%d].pid.max", wam->name, link);
         parseGetVal(DOUBLE, key, (void*)&wam->JposControl.pid[link].saturation);
         //parseGetVal(DOUBLE, key, (void*)&(valptr_vn(wam->saturation)[link]));

         // Read joint vel/acc parameters
         sprintf(key, "%s.link[%d].vel", wam->name, link);
         parseGetVal(DOUBLE, key, (void*)&(valptr_vn(wam->vel)[link]));
         //parseGetVal(DOUBLE, key, (void*)&tmpdbl);
         //setval_vn(wam->vel, link, tmpdbl);

         sprintf(key, "%s.link[%d].acc", wam->name, link);
         parseGetVal(DOUBLE, key, (void*)&(valptr_vn(wam->acc)[link]));
         //parseGetVal(DOUBLE, key, (void*)&tmpdbl);
         //setval_vn(wam->acc, link, tmpdbl);

         link_geom_bot(&wam->robot, link, theta * pi, d, a, alpha * pi);
         link_mass_bot(&wam->robot, link, &tmp_v3, mass);
      } else {
         tool_geom_bot(&wam->robot, theta * pi, d, a, alpha * pi);
         tool_mass_bot(&wam->robot, &tmp_v3, mass);
      }
   }

   // Find motor positions
   for (actcnt = 0; actcnt < wam->num_actuators; actcnt++) {
#ifdef BTOLDCONFIG
      wam->motor_position[actcnt] = actcnt;
#else

      getProperty(wam->act[actcnt].bus, wam->act[actcnt].puck.ID, ROLE, &reply);
      if (reply == ROLE_GIMBALS) {
         wam->motor_position[actcnt] = wam->act[actcnt].puck.ID - 1;
      } else {
         switch(wam->act[actcnt].puck.ID) {
         case -1: //case 1: case 4: // xxx Remove me
            wam->motor_position[actcnt] = wam->act[actcnt].puck.ID-1;
            break;
         default:
            getProperty(wam->act[actcnt].bus, wam->act[actcnt].puck.ID, JIDX, &reply);
            wam->motor_position[actcnt] = reply-1;
            break;
         }
      }
#endif
      syslog(LOG_ERR, "Motor[%d] is joint %d", actcnt, wam->motor_position[actcnt]);
   }

   //syslog(LOG_ERR, "Kinematics loaded, bus = %d", wam->act[0].bus);

   /* If the WAM is already zeroed, note it- else, zero it */
   //reply = TRUE;
   getProperty(wam->act->bus, SAFETY_MODULE, ZERO, &reply);

   if(reply) {
      wam->isZeroed = TRUE;
      syslog(LOG_ERR, "WAM was already zeroed");
   }
   else
   {
      int err;
      vect_n *zeromag;
      zeromag = new_vn(wam->dof);
      sprintf(key, "%s.zeromag", wam->name);
      err = parseGetVal(VECTOR, key, zeromag);
      if (err)
      {
         /* No zeromag entry ... fall back to default*/
         DefineWAMpos(wam, wam->park_location);
         syslog(LOG_ERR, "WAM zeroed by application (no zeromag)");
      }
      else
      {
         int m;
         vect_n *curmag;
         vect_n *errmag;
         vect_n *numcounts;
         curmag = new_vn(wam->dof);
         errmag = new_vn(wam->dof);
         numcounts = new_vn(wam->dof);
         /* Grab each motor's magnetic encoder value into curmag */
         for (m=0; m<wam->dof; m++)
         {
            long reply;
            getProperty(0,m+1,MECH,&reply);
            setval_vn(curmag,m,(double)reply);
         }
         /* Grab each motor's number of encoder counts (per motor revolution) into numcounts */
         for (m=0; m<wam->dof; m++)
         {
            long reply;
            getProperty(0,m+1,CTS,&reply);
            setval_vn(numcounts,m,(double)reply);
         }
         /* Calculate the error in encoder counts */
         Jpos2Mpos(wam,wam->park_location,wam->Mpos);
         
				 //errmag is difference between current 'home' and ideal home pos in encoder counts
				 //(ideal-home-in-encoder-counts - (current-encoder-counts - zero-encoder-counts))
         set_vn(errmag, add_vn( scale_vn(1.0/(2*pi),
                                         e_mul_vn(numcounts,wam->Mpos)),
                                sub_vn( zeromag, curmag )));
         for (m=0; m<wam->dof; m++)
            while (errmag->q[m] > numcounts->q[m]/2.0)
               errmag->q[m] -= numcounts->q[m];
         for (m=0; m<wam->dof; m++)
            while (errmag->q[m] < -numcounts->q[m]/2.0)
               errmag->q[m] += numcounts->q[m];
         /* Define a zero error for any pucks with a sucky version number */
         for (m=0; m<wam->dof; m++)
         {
            if ( GetProp(m,VERS) < 118 || GetProp(m,ROLE) != 256 )
               errmag->q[m] = 0.0;
         }
         /* Define a zero error for any pucks with a -1 zeromag value */
         for (m=0; m<wam->dof; m++)
            if (getval_vn(zeromag,m) < 0) errmag->q[m] = 0.0;
         /* Subtract the error from the park location
          * to set the current location */
         Mpos2Jpos(wam, scale_vn((2*pi),e_div_vn(errmag,numcounts)), wam->Jpos);
         DefineWAMpos(wam, sub_vn(wam->park_location,wam->Jpos));
         syslog(LOG_ERR, "WAM zeroed by application (using zeromag)");
         set_vn(wam->Jpos,wam->park_location);
         /* Done. */
         destroy_vn(&curmag);
         destroy_vn(&errmag);
         destroy_vn(&numcounts);
      }
      destroy_vn(&zeromag);
   }

   /* Sad, but true. Until we code something to calculate dq and ddq, our
      control will be limited to static calcs (no Coriolis).
    */
   fill_vn(wam->robot.dq,0.0);
   fill_vn(wam->robot.ddq,0.0);

   set_gravity_bot(&wam->robot, 0.0);

   test_and_log(
      btrt_mutex_create(&(wam->loop_mutex)),
      "Could not initialize mutex for WAM control loop.");

   // Spin off the maintenance thread
   {
      strcpy(thd_maintname,"MAIx");
      thd_maintname[3] = '0' + numMaintThreads;
      btrt_thread_create(&wam->maint, thd_maintname, 30, (void*)WAMMaintenanceThread, (void*)wam);
      numMaintThreads++;
   }

   return(wam);
}

/** Free memory and close files opened by OpenWAM() */
void CloseWAM(wam_struct* wam)
{
   //btthread_stop(&wam->maint);
   CloseSystem();
}

/**
 Actuator Idx starts at 0 and counts up to number of actuators. 
 MotorID is keyed to the location on the wam (and hense the joint it drives)
 This relationship allows only calculating the control info for the joints that
 are in use.
*/
BTINLINE int MotorID_From_ActIdx(wam_struct *wam,int idx)
{
   return wam->motor_position[idx];
}

/** Internal: Periodic service routine
    Services the data logging and teach&play data structures (for writing buffers to disk). 
    Also handles trajectory playback options including loop_trj and idle_when_done.
*/
void WAMMaintenanceThread(void *data)
{
   btrt_thread_struct* this_thd;
   wam_struct *wam;

   this_thd = (btrt_thread_struct*)data;
   wam = (wam_struct*)this_thd->data;

   while (!btrt_thread_done(this_thd)) {
      if (get_trjstate_bts(wam->active_sc) == BTTRAJ_DONE) {
         if (wam->idle_when_done) {
            setmode_bts(wam->active_sc,SCMODE_IDLE);
         }
      }
      evalDL(&(wam->log));
      evalDL(&(wam->cteach));
      usleep(100000); // Sleep for 0.1s
   }

   pthread_exit(NULL);
}

/** This function closes the control loop on the WAM.
 
The WAMControlThread() contains code to:
 - Set up periodic timer for the control loop
 - Wait until the start of the next control period
 - Clear CAN bus of any unwanted messages
 - Read the motor positions from the WAM
 - Convert from motor angles to joint angles
 - Evaluate the Joint state controller (trajectory/position controller yields joint torques)
 - Evaluate the Forward Kinematics (use joint angles to calculate the tool position)
 - Evaluate the Forward Jacobian and Mass/Inertia matrices
 - Evaluate the Forward Dynamics (prepare the dynamics data for later force application)
 - Evaluate the Cartesian state controller (trajectory/position controller yields tool forces/torques)
 - Call the user's WAM Callback function (see #registerWAMcallback())
 - Evaluate the Backward Dynamics (convert link forces to joint torques)
 - Convert from joint torques to motor torques
 - Send the torques to the WAM
 
 \todo Clean up control loop profiling code. It is too distracting.
 
*/
void WAMControlThread(void *data)
{
   btrt_thread_struct* this_thd;
   int               cnt;
   int               idx;
   int               Mid;
   double            dt,dt_targ,skiptarg;
   int               err,skipcnt = 0;
   int               firstLoop = 1;
   long unsigned     counter = 0;
   RTIME             last_loop,loop_start,loop_end,user_start,user_end,pos1_time,pos2_time,trq1_time,trq2_time;
   double            thisperiod;
   RTIME             rtime_period, sampleCount;
   wam_struct        *wam;
   unsigned char     CANdata[8];
   int               len_in;
   int               id_in;
   unsigned long     overrun;
   int               ret, numskip;
   RT_TASK           *WAMControlThreadTask;

   unsigned long     max1, min1, dat1, max2, min2, dat2, max3, min3, dat3, max4, min4, dat4, startt1, stopt1, startt2, stopt2, startt3, stopt3;
   unsigned long     loop_time;
   RTIME             mean1=0, mean2=0, mean3=0, mean4=0;
   double            sumX1, sumX21, stdev1, sumX2, sumX22, stdev2, sumX3, sumX23, stdev3, sumX4, sumX24, stdev4;

   /*//DEBUG: Seting up warn signal for changes to secondary mode, XENOMAI
   btrt_set_mode_warn();*/

   /* Rotation control */
   vect_3 *ns, *n, *os, *o, *as, *a;
   vect_3 *e, *ed, *last_e;
   matr_3 *kp, *kd;

   ns = new_v3();
   n = new_v3();
   os = new_v3();
   o = new_v3();
   as = new_v3();
   a = new_v3();
   e = new_v3();
   ed = new_v3();
   last_e = new_v3();
   kp = new_m3();
   kd = new_m3();

   /* Get the wam pointer */
   this_thd = (btrt_thread_struct*)data;
   wam = this_thd->data;
   
   /* Set up the rotation control gains */
   ELEM(kp, 0, 0) = wam->CposControl.pid[0].Kp;
   ELEM(kp, 1, 1) = wam->CposControl.pid[5].Kp;
   ELEM(kp, 2, 2) = wam->CposControl.pid[10].Kp;
   ELEM(kd, 0, 0) = wam->CposControl.pid[0].Kd;
   ELEM(kd, 1, 1) = wam->CposControl.pid[5].Kd;
   ELEM(kd, 2, 2) = wam->CposControl.pid[10].Kd;
   
   /* Set up timer */
   thisperiod = this_thd->period;
   rtime_period = (RTIME)(thisperiod * 1000000000.0);

#ifdef RTAI

   sampleCount = nano2count(rtime_period);
   rt_set_periodic_mode();
   start_rt_timer(sampleCount);

   WAMControlThreadTask = rt_task_init(nam2num(this_thd->name), 5, 0, 0);
   rt_task_make_periodic_relative_ns(WAMControlThreadTask, rtime_period, rtime_period);

#endif

   dt_targ = thisperiod;
   skiptarg = 1.5 * dt_targ;
   dt = dt_targ;
   wam->skipmax = 0.0;
   syslog(LOG_ERR,"WAMControl period Sec:%f, ns: %d",thisperiod, rtime_period);

   /*Set up as hard real time*/
   btrt_set_mode_hard();

   /* Install a signal handler to log when the thread is changing over to secondary mode.
      Almost any I/O in a realtime thread will cause the thread to be pulled out
      of the realtime domain and placed in the domain of the standard Linux scheduler.
      This includes printf(), syslog(), send(), receive(), read(), write(), and many
      other system calls. To keep the control loop in the realtime domain, you should
      avoid calling these functions from a realtime thread. Excessive latencies, 
      heartbeat errors, and unstable control can all result from the control loop
      being forced out of primary mode (realtime domain) by a realtime-unfriendly
      function call!
    */
#ifdef XENOMAI
   signal(SIGXCPU, warn_upon_switch); // Catch the SIGXCPU signal
   btrt_set_mode_warn(); // Enable the warning
#endif

#ifdef XENOMAI
   /*Make task periodic*/
   test_and_log(
      rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks((rtime_period))),"WAMControlThread: rt_task_set_periodic failed, code");
#endif

   //#ifdef  BTREALTIME
   //btrt_set_mode_hard();
   //#endif

   last_loop = btrt_get_time();

   DEBUG(
      //sumX1 = sumX21 = 0;
      max1 = 0;
      min1 = 2E9;

      //sumX2 = sumX22 = 0;
      max2 = 0;
      min2 = 2E9;

      //sumX3 = sumX23 = 0;
      max3 = 0;
      min3 = 2E9;

      //sumX4 = sumX24 = 0;
      max4 = 0;
      min4 = 2E9;
      //numskip = 0;
   );


   while (!btrt_thread_done(this_thd)) {

      /*Make sure task is still in Primary mode*/
      btrt_set_mode_hard();

      DEBUG(startt3 = btrt_get_time());
      /* Wait until the start of the next control period */
      btrt_task_wait_period();
      DEBUG(stopt3 = btrt_get_time());

      loop_start = btrt_get_time(); //th prof
      loop_time = loop_start - last_loop;
      wam->loop_period = loop_time; //th prof
      last_loop = loop_start;
      // Find elapsed time (in seconds) since the previous control cycle
      dt = loop_time / (double)1E9;
      if(!firstLoop) {
         if (dt > skiptarg) {
            skipcnt++;
         }
         if (dt > wam->skipmax)
            wam->skipmax = dt;
      }
      wam->dt = dt;

      test_and_log(
         btrt_mutex_lock(&(wam->loop_mutex)),"WAMControlThread lock mutex failed");

      pos1_time = btrt_get_time(); //th prof

      /* Clear CAN bus of any unwanted messages */
      canClearMsg(0);

      /* Read the motor positions from the robot */
      GetPositions(wam->act->bus);

      pos2_time = btrt_get_time(); //th prof
      wam->readpos_time = pos2_time - pos1_time; //th prof

      DEBUG(
         if(mean1 || firstLoop) {
         dat1 = pos2_time - pos1_time;
      } else {
         mean1 = dat1 = pos2_time - pos1_time;
      }
   );

      /* Copy the motor angles from the actuator structure to the Mpos vector */
      ActAngle2Mpos(wam,(wam->Mpos));
      //const_vn(wam->Mpos, 0.0, 0.0, 0.0, 0.0);

      /* Convert from motor angles to joint angles */
      Mpos2Jpos(wam,(wam->Mpos), (wam->Jpos));

      // Joint space stuff
      pos1_time = btrt_get_time(); //th prof
      /* Evaluate the Joint state controller */
      eval_bts(&(wam->Jsc));
      pos2_time = btrt_get_time(); //th prof
      wam->Jsc_time = pos2_time - pos1_time; //th prof

      // Cartesian space stuff
      /* Copy the joint positions from the 'robot' structure to the Jpos vector */
      set_vn(wam->robot.q,wam->Jpos);

      /* Evaluate the Forward Kinematics */
      eval_fk_bot(&wam->robot);

      /* Evaluate the Forward Jacobian (and mass matrix)
       * Note - since these calculations are not needed for gravity
       * compensation, basic joint-space or Cartesian-space control, or
       * teach & play, they are left out of the control loop by default.
       * These calculations take approximately 1100 us to compute on the
       * WAM's internal PC104 computer, limiting the control loop rate to
       * ~450Hz.  With this line disabled, control loop frequency is 750Hz.
       * For a faster control frequency, use an external WAM PC,
       * capable of a 1kHz control loop. */
      /*eval_fj_bot(&wam->robot);*/

      /* Evaluate the Forward Dynamics */
      eval_fd_bot(&wam->robot);

      /* Get gravity torques */
      get_gravity_torques(&wam->robot, wam->Gtrq);
      
      /* Fill in the Cartesian tool XYZ */
      set_v3(wam->Cpos,T_to_W_bot(&wam->robot,wam->Cpoint));
      //set_vn(wam->R6pos,(vect_n*)wam->Cpos);

      pos1_time = btrt_get_time(); //th prof
      /* Evaluate the Cartesian state controller */
      eval_bts(&(wam->Csc));
      pos2_time = btrt_get_time(); //th prof
      wam->user_time = pos2_time - pos1_time; //th prof

      /* Copy the XYZ force terms from the Cartesian controller output into Cforce */
      getcol_m3(wam->Cforce, wam->HMft, 3);

      /* Cartesian rotation control */
      fill_v3(wam->Ctrq, 0.0); // Start with zero torque
      if(getmode_bts(wam->active_sc) > SCMODE_IDLE && wam->active_sc == &wam->Csc) {
         /* Actual rotation */
         getcol_m3(n, wam->HMpos, 0);
         getcol_m3(o, wam->HMpos, 1);
         getcol_m3(a, wam->HMpos, 2);
         
         /* Desired rotation */
         getcol_m3(ns, wam->HMref, 0);
         getcol_m3(os, wam->HMref, 1);
         getcol_m3(as, wam->HMref, 2);
         
         /* Error = 0.5 [ (ns x n) + (os x o) + (as x a) ] (Luh, Walker, Paul, 1980) */
         set_v3(e, (scale_v3(0.5, add_v3(cross_v3(ns,n), add_v3(cross_v3(os,o), cross_v3(as,a))))));
         
         /* First derivative of the error (simple, no filter) */
         set_v3(ed, scale_v3(1.0/thisperiod, add_v3(e, scale_v3(-1.0, last_e))));
         
         /* Torque = (Kp * e) + (Kd * ed) (PD control) */
         set_v3(wam->Ctrq, add_v3(matXvec_m3(kp, e), matXvec_m3(kd, ed)));
         
         /* Store the last error for the next iteration */
         set_v3(last_e, e);
      }else{
         fill_v3(last_e, 0.0); // Clear last rotation error 
      }
     
#if 0
      /* Quaternian angular control in Cartesian space */
      R_to_q(wam->qact,T_to_W_trans_bot(&wam->robot));
      set_q(wam->qaxis,mul_q(wam->qact,conj_q(wam->qref)));
      set_q(wam->forced,force_closest_q(wam->qaxis));
      wam->qerr = GCdist_q(wam->qref,wam->qact);
      //syslog_vn("qref: ", (vect_n*)wam->qref);
      //syslog_vn("qact: ", (vect_n*)wam->qact);
      //syslog(LOG_ERR, "qerr: %f", wam->qerr);

      set_v3(wam->Ctrq,scale_v3(eval_err_btPID(&(wam->pid[3]),wam->qerr,dt),GCaxis_q(wam->Ctrq,wam->qref,wam->qact)));
      //syslog_vn("Ctrq: ", (vect_n*)wam->Ctrq);
#endif

      /* Insert the end-effector forces and torques generated by the Cartesian state controller into the 'robot' structure */
      apply_tool_force_bot(&wam->robot, wam->Cpoint, wam->Cforce, wam->Ctrq);

      pos1_time = btrt_get_time(); //th prof
      /* Call the user's WAM Callback function */
      (*wam->force_callback)(wam);
      pos2_time = btrt_get_time(); //th prof
      wam->user_time = pos2_time - pos1_time; //th prof

      /* Evaluate the Backward Dynamics: Calculate robot->t, given link forces and torques (+gravity) */
      eval_bd_bot(&wam->robot);

      /* Copy the RNE joint torques from robot->t into Ttrq */
      get_t_bot(&wam->robot,wam->Ttrq);
      
      /* Copy in gravity compensation torques */
      set_vn(wam->Ttrq,add_vn(wam->Ttrq,wam->Gtrq));

      /* If the WAM is zeroed, add the joint torques due to the Cartesian controller into the Jtrq output */
      if(wam->isZeroed) {
         set_vn(wam->Jtrq,add_vn(wam->Jtrq,wam->Ttrq));
      }

      /* Convert from joint torques to motor torques */
      Jtrq2Mtrq(wam,(wam->Jtrq), (wam->Mtrq));
#if 0

      if(wam->dof == 8) {
         //8-DOF paint spraying demo code
         if(getmode_bts(&wam->Jsc) == SCMODE_IDLE)
            setval_vn(wam->Mtrq, 7, 0);
         else
            setval_vn(wam->Mtrq, 7, getval_vn(wam->Jref, 7));
      }
#endif

		/*  callback function for individual motor (not joint) torques */ 
		(*wam->motor_callback)(wam);

      /* Move motor torques from Mtrq into actuator database */
      Mtrq2ActTrq(wam,wam->Mtrq);
#ifdef BTDOUBLETIME

      //rt_make_soft_real_time();
      btrt_set_mode_soft();
#endif

      trq1_time = btrt_get_time(); //th prof

      /* Send the torques to the WAM */
      SetTorques(wam->act->bus);

      trq2_time = btrt_get_time(); //th prof
      wam->writetrq_time = trq2_time - trq1_time; //th prof
      //DEBUG(dat2 = trq2_time - trq1_time);
      DEBUG(
         if(mean2 || firstLoop) {
         dat2 = trq2_time - trq1_time;
      } else {
         mean2 = dat2 = trq2_time - trq1_time;
      }
   );
#ifdef BTDOUBLETIME

      //rt_make_hard_real_time();
      btrt_set_mode_hard();

#endif

      loop_end = btrt_get_time(); //th prof
      wam->loop_time = loop_end - loop_start; //th prof

      btrt_mutex_unlock(&(wam->loop_mutex));

      /* Handle data logging */
      wam->logCounter++;
      if (wam->log.Log_Data) {
         wam->log_time += dt;
         if ((wam->logCounter % wam->logDivider) == 0)
            TriggerDL(&wam->log);
      }

      /* Handle continuous teaching */
      wam->teachCounter++;
      if (wam->cteach.Log_Data) {
         wam->teach_time += dt;
         if ((wam->teachCounter % wam->teachDivider) == 0)
            TriggerDL(&wam->cteach);
      }

      DEBUG(

         if(mean3 || firstLoop) {
         dat3 = stopt3-startt3;
      } else {
         mean3 = dat3 = stopt3-startt3;
      }

      if(mean4 || firstLoop) {
         dat4 = trq1_time-pos2_time;
      } else {
         mean4 = dat4 = trq1_time-pos2_time;
      }

      /* Track max/min */
      if(!firstLoop) {
         if(dat1 > max1)
               max1 = dat1;
            if((dat1 < min1) && (dat1 > 100))
               min1 = dat1;
            if(dat2 > max2)
               max2 = dat2;
            if((dat2 < min2) && (dat2 > 100))
               min2 = dat2;
            if(dat3 > max3)
               max3 = dat3;
            if((dat3 < min3) && (dat3 > 100))
               min3 = dat3;
            if(dat4 > max4)
               max4 = dat4;
            if((dat4 < min4) && (dat4 > 100))
               min4 = dat4;

            if(dat1 > mean1)
               mean1+=1;
            if(dat1 < mean1)
               mean1-=1;
            if(dat2 > mean2)
               mean2+=1;
            if(dat2 < mean2)
               mean2-=1;
            if(dat3 > mean3)
               mean3+=1;
            if(dat3 < mean3)
               mean3-=1;
            if(dat4 > mean4)
               mean4+=1;
            if(dat4 < mean4)
               mean4-=1;
         }
         /* Track sum(X) and sum(X^2)
         sumX1 += dat1;
         sumX21 += dat1 * dat1;
         sumX2 += dat2;
         sumX22 += dat2 * dat2;
         sumX3 += dat3;
         sumX23 += dat3 * dat3;
         sumX4 += dat4;
         sumX24 += dat4 * dat4;*/
      );

      if(firstLoop)
         firstLoop = 0;
   }
#ifdef XENOMAI
   rt_task_set_mode(T_WARNSW, 0, NULL); // Disable mode switch warning
#endif
   syslog(LOG_ERR, "WAM Control Thread: exiting");
   syslog(LOG_ERR, "----------WAM Control Thread Statistics:--------------");
   DEBUG(
      /*
         mean1 = 1.0 * sumX1 / counter;
         stdev1 = sqrt((1.0 * counter * sumX21 - sumX1 * sumX1) / (counter * counter - counter));
         mean2 = 1.0 * sumX2 / counter;
         stdev2 = sqrt((1.0 * counter * sumX22 - sumX2 * sumX2) / (counter * counter - counter));
         mean3 = 1.0 * sumX3 / counter;
         stdev3 = sqrt((1.0 * counter * sumX23 - sumX3 * sumX3) / (counter * counter - counter));
         mean4 = 1.0 * sumX4 / counter;
         stdev4 = sqrt((1.0 * counter * sumX24 - sumX4 * sumX4) / (counter * counter - counter));
      */
      syslog(LOG_ERR, "Get Positions)Max: %ld", max1);
      syslog(LOG_ERR, "              Min: %ld", min1);
      syslog(LOG_ERR, "             Mean: %ld", mean1);
      /*syslog(LOG_ERR, "            Stdev: %.4f", stdev1);*/
      syslog(LOG_ERR, "Set Torques)  Max: %ld", max2);
      syslog(LOG_ERR, "              Min: %ld", min2);
      syslog(LOG_ERR, "             Mean: %ld", mean2);
      /*syslog(LOG_ERR, "            Stdev: %.4f", stdev2);*/
      syslog(LOG_ERR, "Wait time)    Max: %ld", max3);
      syslog(LOG_ERR, "              Min: %ld", min3);
      syslog(LOG_ERR, "             Mean: %ld", mean3);
      /*syslog(LOG_ERR, "            Stdev: %.4f", stdev3);*/
      syslog(LOG_ERR, "Calc time)    Max: %ld", max4);
      syslog(LOG_ERR, "              Min: %ld", min4);
      syslog(LOG_ERR, "             Mean: %ld", mean4);
      /*syslog(LOG_ERR, "            Stdev: %.4f", stdev4);*/
      /*syslog(LOG_ERR, "Skips %d", counter);*/
   );

   syslog(LOG_ERR,"WAMControl Skipped cycles %d, Max dt: %lf", skipcnt, wam->skipmax);
   //syslog(LOG_ERR,"WAMControl Times: Readpos: %lld, Calcs: %lld, SendTrq: %lld",wam->readpos_time,wam->loop_time-wam->writetrq_time-wam->readpos_time,wam->writetrq_time);

   /*Delete thread when done*/
   btrt_thread_exit(this_thd);
}

/** Load a wam_vector with the actuator angles.
 ActAngle2Mpos loads a wam_vector with the actuator angles.
  \param *Mpos Pointer to a wam_vector
  \retval 0 Successful
  \retval -1 Unexpected number of actuators
*/
int ActAngle2Mpos(wam_struct *wam,vect_n *Mpos)
{
   int num_act, cnt;

   //Extracts actuators 0 thru 6 into wam_vector positions
   num_act = wam->num_actuators;
   for (cnt = 0; cnt < num_act; cnt++) {
      setval_vn(Mpos,wam->motor_position[cnt],wam->act[cnt].angle);
   }

   return 0;
}

/** Load the actuator database torques with the values from a wam_vector.
 Mtrq2ActTrq loads the actuator database torques with the values from a wam_vector.
  \param *Mtrq Pointer to a wam_vector
  \retval 0 Successful
  \retval -1 Unexpected number of actuators
*/
int Mtrq2ActTrq(wam_struct *wam,vect_n *Mtrq)
{
   actuator_struct *act;
   int num_act, cnt;

   //Packs wam_vector of torques into actuator array
   //act = GetActuators(&num_act);
   for (cnt = 0; cnt < buses[wam->act[0].bus].num_pucks; cnt++) {
      wam->act[cnt].torque = getval_vn(Mtrq,wam->motor_position[cnt]);
   }

   return 0;
}

//=================================================working code
/** Transform wam_vector Motor positions to Joint positions
*/
void Mpos2Jpos(wam_struct *wam,vect_n * Mpos, vect_n * Jpos)
{
   vect_n tmp_vn[2];
   btreal tmp_btreal[8];

   init_local_vn(tmp_vn,tmp_btreal,8);

   set_vn(Jpos,matXvec_mn(wam->M2JP,Mpos,tmp_vn));
}

//=================================================working code
/** Transform wam_vector Joint positions to Motor positions
*/
void Jpos2Mpos(wam_struct *wam,vect_n * Jpos, vect_n * Mpos)
{
   vect_n tmp_vn[2];
   btreal tmp_btreal[8];

   //btreal pos[10];
   //btreal mN[10];
   //btreal mn[10];
   init_local_vn(tmp_vn,tmp_btreal,8);
   /*
   extract_vn(pos,Jpos);
   extract_vn(mN,wam->N);
   extract_vn(mn,wam->n);
   setval_vn(Mpos,0, ( -pos[0] * mN[0]));
   setval_vn(Mpos,1, (pos[1] * mN[1]) - (pos[2] * mN[2] / mn[2]));
   setval_vn(Mpos,2, ( -pos[1] * mN[1]) - ( pos[2] * mN[2] / mn[2]));
   if(wam->M4_reversed)
      setval_vn(Mpos,3, (-pos[3] * mN[3]));
   else
      setval_vn(Mpos,3, (pos[3] * mN[3]));
   if(wam->dof > 4)
{
      setval_vn(Mpos,4,pos[4] * mN[4] - pos[5] * mN[4] / mn[5]);
      setval_vn(Mpos,5, pos[4] * mN[4] + pos[5] * mN[4] / mn[5]);
      setval_vn(Mpos,6, -pos[6] * mN[6]);
}
   */
   set_vn(Mpos, matXvec_mn(wam->J2MP, Jpos, tmp_vn));
}

/** Transform wam_vector Joint torques to Motor torques
*/
void Jtrq2Mtrq(wam_struct *wam,vect_n * Jtrq, vect_n * Mtrq)
{
   vect_n tmp_vn[2];
   btreal tmp_btreal[8];
   //btreal trq[10];
   //btreal mN[10];
   //btreal mn[10];
   init_local_vn(tmp_vn,tmp_btreal,8);
   /*
   extract_vn(trq,Jtrq);
   extract_vn(mN,wam->N);
   extract_vn(mn,wam->n);
   setval_vn(Mtrq,0, ( -trq[0] / mN[0]));
   setval_vn(Mtrq,1, (0.5 * trq[1] / mN[1]) - (0.5 * trq[2] *mn[2]/ mN[2]));
   setval_vn(Mtrq,2, ( -0.5 * trq[1]  / mN[1]) - (0.5 * mn[2] * trq[2] / mN[2]));
   setval_vn(Mtrq,3,  (trq[3] / mN[3]));
   setval_vn(Mtrq,4, 0.5 * trq[4] / mN[4] - 0.5 * mn[5] * trq[5] / mN[4]);
   setval_vn(Mtrq,5, 0.5 * trq[4] / mN[4] + 0.5 * mn[5] * trq[5] / mN[4]);
   setval_vn(Mtrq,6, -1 * trq[6] / mN[6]);
   */
   set_vn(Mtrq, matXvec_mn(wam->J2MT, Jtrq, tmp_vn));

}

/** Reset the position sensors on the pucks to the passed in position in units of joint-space radians
 
Reset the position sensors on the pucks to the passed in position in units of joint-space radians
The wam must be in idle mode.
*/
void DefineWAMpos(wam_struct *wam, vect_n *wv)
{
   int      cnt;
   long     tmp;
   //vect_n   *motor_angle;
   long     pos;
   vect_n   motor_angle[2];
   btreal   tmp_btreal[8];

   init_local_vn(motor_angle, tmp_btreal, 8);
   //motor_angle = new_vn(wam->dof);

   /* Tell the safety logic to ignore the next several faults (position will appear to be changing rapidly) */
   SetByID(wam->act->bus, SAFETY_MODULE, IFAULT, 8);

   // Convert from joint space to motor space
   Jpos2Mpos(wam, wv, motor_angle);

   // Convert from motor radians to encoder counts and set the new values
   for (cnt = 0; cnt < (wam->num_actuators - 3 * gimbalsInit); cnt++) {
      pos = floor( wam->act[cnt].motor.counts_per_rev / TWOPI * getval_vn(motor_angle, wam->motor_position[cnt]) );
      setProperty(wam->act->bus, wam->act[cnt].puck.ID, AP, FALSE, pos);
      usleep(1000);
   }

   /* Tell the safety logic start monitoring tip velocity */
   SetByID(wam->act->bus, SAFETY_MODULE, ZERO, 1); /* 0 = Joint velocity, 1 = Tip velocity */

   wam->isZeroed = TRUE;
}

/** Control the WAM in Cartesian (linear) space
*/
void SetCartesianSpace(wam_struct* wam)
{
   int trjstate;

   // In joint space?
   if (wam->active_sc == &wam->Jsc) {
      trjstate = get_trjstate_bts(wam->active_sc);
      if (trjstate != BTTRAJ_DONE && trjstate != BTTRAJ_STOPPED)
         stop_trj_bts(wam->active_sc);
      setmode_bts(&(wam->Jsc),SCMODE_IDLE);
      setmode_bts(&(wam->Csc),SCMODE_IDLE);
      wam->active_sc = &wam->Csc;
   }
}

/** Control the WAM in joint space
*/
void SetJointSpace(wam_struct* wam)
{
   int trjstate;

   // In cartesian space?
   if (wam->active_sc == &wam->Csc) {
      trjstate = get_trjstate_bts(wam->active_sc);
      if (trjstate != BTTRAJ_DONE && trjstate != BTTRAJ_STOPPED)
         stop_trj_bts(wam->active_sc);
      setmode_bts(&(wam->Jsc),SCMODE_IDLE);
      setmode_bts(&(wam->Csc),SCMODE_IDLE);
      wam->active_sc = &wam->Jsc;
   }
}

/** Turns position constraint on/off.
 
If the robot is being controlled in JointSpace (SetJointSpace), it will apply a PD loop to each joint.
If the robot is being controlled in CartesianSpace (SetCartesianSpace), it will apply a PD loop to the robot endpoint.
 
\param wam Pointer to wam structure you want to operate on.
\param onoff 0:Off !0:On
 
    
We only allow setting position mode if we are idling. 
Setting position mode while already in position mode will reset the position 
constraint causing unexpected behavior for a user at this level
 
*/
void SetPositionConstraint(wam_struct* wam, int onoff)
{
   int present_state;

   present_state = getmode_bts(wam->active_sc);
   if (onoff && present_state == SCMODE_IDLE)
      setmode_bts(wam->active_sc,SCMODE_POS);
   else
      setmode_bts(wam->active_sc,SCMODE_IDLE);
}

/** Set the velocity and acceleration of a WAM move
*/
void MoveSetup(wam_struct* wam, btreal vel, btreal acc)
{
   moveparm_bts(wam->active_sc, vel, acc);
}

/** Begins a point-to-point move in your active space (joint or Cartesian)
You should call #MoveSetup() before you call this.
Your destination should have the correct dimensions for your active space.
 
\param wam A pointer to your WAM structure
\param dest The desired destination
*/
void MoveWAM(wam_struct* wam, vect_n* dest)
{
   int present_state;
   present_state = getmode_bts(wam->active_sc);
   if (present_state != SCMODE_POS) {
      setmode_bts(wam->active_sc,SCMODE_POS);
   }
   
   if(moveto_bts(wam->active_sc,dest))
      syslog(LOG_ERR,"MoveWAM:Aborted");
}

/** Checks whether a trajectory is still active
\retval 0 Move is still going
\retval 1 Move is done
*/
int MoveIsDone(wam_struct* wam)
{
   int ret;
   ret = get_trjstate_bts(wam->active_sc);
   if (ret == BTTRAJ_DONE || ret == BTTRAJ_STOPPED)
      ret = 1;
   else
      ret = 0;
   return ret;
}

void MoveStop(wam_struct* wam)
{
   stop_trj_bts(wam->active_sc);
}


/******************************************************************************/


/** Find the center of a joint range and move to the zero_offsets position relative to the center
 
\todo Add code to check the puck for IsZerod so that we
know whether we have to zero or not...
*/

/** Move the WAM to the park position defined in the wam data file
*/
void ParkWAM(wam_struct* wam)
{
   SetJointSpace(wam);
   MoveSetup(wam, 0.25, 0.25);
   MoveWAM(wam, wam->park_location);
}

/** Returns the present anti-gravity scaling
  
   \retval The present anti-gravity scaling
*/
btreal GetGravityComp(wam_struct *w)
{
   return get_gravity_bot(&w->robot);
}

/** Enable or disable the gravity compensation calculation
*/
void SetGravityComp(wam_struct *w,btreal scale)
{
   set_gravity_bot(&w->robot, scale); 
}

/** Enable or disable using calibrated gravity compensation
*/
void SetGravityUsingCalibrated(wam_struct *w, int onoff)
{
   set_gravity_bot_calibrated(&w->robot,onoff);
}

/** Returns present use-calibrated-gravity setting

   \retval Whether the WAM is currently using calibrated gravity compensation
 */
int GetGravityUsingCalibrated(wam_struct *w)
{
   return (get_gravity_bot_calibrated(&w->robot) == 1) ? 1 : 0;
}

/** Returns whether the WAM has gravity-calibration vectors

   \retval Whether the WAM is has calibrated gravity information
 */
int GetGravityIsCalibrated(wam_struct *w)
{
   return (get_gravity_bot_is_calibrated(&w->robot) == 1) ? 1 : 0;
}

/** Sample the torques required to hold a given position
*/
void GCompSample(wam_struct *wam, vect_n *trq, double p1, double p2, double p3, double p4)
{
   const_vn(trq, p1, p2, p3, p4);
   MoveWAM(wam, trq);
   usleep(4000000);

   trq->q[0] = wam->Jtrq->q[0];
   trq->q[1] = wam->Jtrq->q[1];
   trq->q[2] = wam->Jtrq->q[2];
   trq->q[3] = wam->Jtrq->q[3];
}

/** Take 4DOF measurements to estimate the coefficients for calculating the effect of gravity
*/
void getLagrangian4(wam_struct *wam,double *A, double *B, double *C, double *D)
{

   double t41,t21,t42,t22,t23,t33;

   vect_n *trq;

   trq = new_vn(4);

   MoveSetup(wam,0.5,0.5);

   GCompSample(wam,trq,0,-1.5708,0,0);
   t41 = trq->q[3];
   t21 = trq->q[1];

   GCompSample(wam,trq,0,-1.5708,0,1.5708);
   t42 = trq->q[3];
   //t22 = trq->q[1];

   GCompSample(wam,trq,0,-1.5708,-1.5708,1.5708);
   //t23 = trq->q[1];
   t33 = trq->q[2];

   *A = -t42;
   *B = -t41;
   *C = t33 + *B;
   *D = t21 - t41;

   destroy_vn(&trq);
}

void setSafetyLimits(int bus, double jointVel, double tipVel, double elbowVel)
{
   int conversion;

   syslog(LOG_ERR, "About to set safety limits, VL2 = %d", VL2);
   if((jointVel > 0) && (jointVel < 7)) /* If the vel (rad/s) is reasonable */
   {
      conversion = jointVel * 0x1000; /* Convert to Q4.12 rad/s */
      SetByID(bus, SAFETY_MODULE, VL2, conversion);
   }

   if((tipVel > 0) && (tipVel < 7)) /* If the vel (m/s) is reasonable */
   {
      conversion = tipVel * 0x1000; /* Convert to Q4.12 rad/s */
      SetByID(bus, SAFETY_MODULE, VL2, conversion);
   }

   if((elbowVel > 0) && (elbowVel < 7)) /* If the vel (m/s) is reasonable */
   {
      conversion = elbowVel * 0x1000; /* Convert to Q4.12 rad/s */
      SetByID(bus, SAFETY_MODULE, VL2, conversion);
   }

}

/** Syslog wam configuration information for debugging
*/
void DumpWAM2Syslog(wam_struct *wam)
{
   char buf[255];
   syslog(LOG_ERR,"Dump of WAM data---------------------------");
   syslog(LOG_ERR,"WAM:zero_offsets:%s",sprint_vn(buf,wam->zero_offsets));
   syslog(LOG_ERR,"WAM:park_location:%s",sprint_vn(buf,wam->park_location));
   syslog(LOG_ERR,"%d, %d, %d, %d, %d, %d, %d",(wam->zero_order[0]),(wam->zero_order[1]),(wam->zero_order[2]),(wam->zero_order[3]),(wam->zero_order[4]),(wam->zero_order[5]),(wam->zero_order[6]));
   syslog(LOG_ERR,"%d, %d, %d, %d, %d, %d, %d",(wam->motor_position[0]),(wam->motor_position[1]),(wam->motor_position[2]),(wam->motor_position[3]),(wam->motor_position[4]),(wam->motor_position[5]),(wam->motor_position[6]));
   //syslog(LOG_ERR,"WAM:Kp:%s",sprint_vn(buf,wam->Kp));
   //syslog(LOG_ERR,"WAM:Kd:%s",sprint_vn(buf,wam->Kd));
   //syslog(LOG_ERR,"WAM:Ki:%s",sprint_vn(buf,wam->Ki));
   syslog(LOG_ERR,"WAM:vel:%s",sprint_vn(buf,wam->vel));
   syslog(LOG_ERR,"WAM:acc:%s",sprint_vn(buf,wam->acc));
   syslog(LOG_ERR,"Num pucks %d",wam->num_actuators);
   syslog(LOG_ERR,"End dump of WAM data-----------------------------");
}

/** Initialize Continuous teach and play
 
Notes: 
 - Variable numbers of joints are handled by polling the robot structure
 
\param divider 1 = Log data every control cycle, 2 = Log data every 2nd control cycle, etc.
\param filename The output file for the path
*/
void StartContinuousTeach(wam_struct *wam, int divider, char *filename)
{
   wam->teach_time = 0.0;
   wam->teachCounter = 0;
   wam->teachDivider = divider;

   PrepDL(&(wam->cteach),2);
   AddDataDL(&(wam->cteach),&(wam->teach_time),sizeof(btreal),4,"Time");

   if(wam->active_sc == &wam->Jsc) { // Record joint positions
      //AddDataDL(&(wam->cteach), valptr_vn(wam->Jpos), sizeof(btreal)*wam->num_actuators, BTLOG_BTREAL, "Jpos");
      AddDataDL(&(wam->cteach), valptr_vn(wam->Jpos), sizeof(btreal)*wam->dof, BTLOG_BTREAL, "Jpos");
   } else if(wam->active_sc == &wam->Csc) { // Record Cartesian positions
      AddDataDL(&(wam->cteach), valptr_vn((vect_n*)wam->HMpos), sizeof(btreal)*12, BTLOG_BTREAL, "HMpos");
   } else { // Unknown control state (user-defined?)
      return;
   }

   InitDL(&(wam->cteach),1000,filename);
   DLon(&(wam->cteach));
   TriggerDL(&(wam->cteach));
}

/** Stop recording positions to the continuous teach file */
void StopContinuousTeach(wam_struct *wam)
{
   DLoff(&(wam->cteach));
   CloseDL(&(wam->cteach));
}

/** \depreciated */
void ServiceContinuousTeach(wam_struct *wam)
{
   evalDL(&(wam->cteach));
}

/** Register a WAM control loop callback function.

The registerWAMcallback() function registers a special function to be 
called from the WAMControlThread() after the positions have been received 
from the WAM (and after all the kinematics are calculated) but before torques 
are sent to the WAM.
 - Since this function becomes part of the control loop, it must execute 
quickly to support the strict realtime periodic scheduler requirements.
 - For proper operation, you must avoid using any system calls that 
cause this thread to drop out of realtime mode. Avoid syslog, printf, and 
most other forms of I/O.
 
\param wam A pointer to your WAM structure
\param func A pointer to the function you want to register
*/
void registerWAMcallback(wam_struct* wam, void *func)
{
   if (func != NULL)
      wam->force_callback = func;
   else
      wam->force_callback = BlankWAMcallback;
}

void registerWAMmotorcallback(wam_struct* wam, void *func)
{
   if (func != NULL)
      wam->motor_callback = func;
   else
      wam->motor_callback = BlankWAMcallback;
}

/** A blank (default) WAM control loop callback function
*/
int BlankWAMcallback(wam_struct* wam)
{
   return 0;
}
