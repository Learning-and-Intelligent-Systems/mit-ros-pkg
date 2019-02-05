/* ======================================================================== *
 *  Module ............. libbwam
 *  File ............... btwam.h
 *  Creation Date ...... 1 Jul 2004
 *  Author ............. Traveler Hauptman
 *  Addtl Authors ...... Brian Zenowich
 *                       Sam Clanton
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  Copyright (C) 2004-2008 Barrett Technology, Inc. <support@barrett.com>
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

/** \file btwam.h
   \brief Provides functions for controlling a WAM.
 
    The btwam library provides high-level control functions for the Barrett
    WAM. It contains the WAM control loop, plus high-level movement commands
    and Teach & Play routines. It depends on the btsystem library for actuator-
    level communication, kinematics, dynamics, path following, position control,
    config file parsing, data logging, and matrix/vector math.
 
See #btwam_struct
 
 
*/
#ifndef _BTWAM_H
#define _BTWAM_H

#include "btsystem.h"
#include "btrobot.h"
#include "btcontrol.h"
#include "btpath.h"
#include "btlogger.h"
#include "btparser.h"
#include "btmath.h"
#include "btos.h"
#include "btgeometry.h"

#ifndef BTINLINE
#define BTINLINE inline
#endif

/** This structure maintains the present state of the wam
 
wam_struct is used by the WAM control loop and WAM API to maintain information 
about the state of the wam and to maintain the control and calculation data structures.
 
One (and only one) instance of this structure is created for each process. It is 
available to users to get information about the wam and to get pointers to the 
control objects. This structure is not protected with mutexes so you should not 
write to it after starting the control loop thread. 
 
It may be read from at any time but keep in mind that some of the data structures
may be in mid-update.
 
  \code
  
  btthread wam_thd;
  wam_struct *wam;
  
  err = ReadSystemFromConfig("wam.conf"); //Prep btsystem
  wam = OpenWAM("wam.conf");
  if(!wam)
  {
    exit(1);
  }
  wam_thd.period = 0.002;
  btthread_create(&wam_thd,90,(void*)WAMControlThread,(void*)&wam_thd);
  while(!done){
    //If this was your program you'd be done now!
  }
  btthread_stop(&wam_thd); //Kill WAMControlThread
    
  \endcode
 
\todo Replace btPID-specific control structures with generic structures
*/
typedef struct btwam_struct
{
   int id; //!< R/W: Unused
   char name[256]; //!< R: Give your robot a name, just for fun. Specified in wam.conf.
   
   //State Variables
   int isZeroed; //!< R: If TRUE, allow torques from Cartesian control. Defaults to FALSE, set to TRUE after #DefineWAMpos().
   int dof; //!< R: Robot degrees of freedom. Specified in wam.conf.
   int idle_when_done; //!< W: !0 = When done with the present move switch to idle mode. CAUTION: Arm may fall if gcomp is not enabled!
   
   //user callback
   int (*force_callback)(struct btwam_struct *wam); //!< W: A pointer to a function registered with #registerWAMcallback().
	int (*motor_callback)(struct btwam_struct *wam); //!< W: A pointer to a function registered with #registerWAMmotorcallback().
	
   //Actuator info
   actuator_struct *act; //!< R: Low-level actuator data. You probably do not need to worry about this.
   int num_actuators; //!< R: Low-level actuator count. Copied from buses[bus].num_pucks
   int *motor_position; //!< R: Low-level actuator->joint mapping

   //Automatic zero finding
   int *zero_order; //!< W: Defines the order in which the joints are initialized
   vect_n *zero_offsets; //!< W: Distance from a stop to consider the zero-position (rad)
   vect_n *stop_torque; //!< W: How hard the joint pushes into the stop (Nm)
   vect_n *park_location; //!< W: The default home/rest position (rad). Specified in wam.conf.
   vect_n *torq_limit; //!< ???

   //Motor <-> Joint State
   vect_n *Mpos; //!< R: Motor positions (rad), copied from actuator data structure
   vect_n *Mtrq; //!< R: Motor torques (Nm), set by Jtrq2Mtrq()
   vect_n *N,*n; //!< R: Transmission ratios. Specified in wam.conf.
   matr_mn *M2JP, *J2MP, *J2MT; //!< R: Transmission matrices: motor->joint position, joint->motor position, and joint->motor torque. Specified in wam.conf.
   //vect_n *motorInertias;

   //Kinematics & Dynamics
   //double sample_rate;
   btrobot robot; //!< R: All kinematics and dynamics information for this robot. Initialized with values from wam.conf.

   //Motion Control
   btstatecontrol *active_sc; //!< W: A pointer to the active state controller (usually Jsc or Csc)
   double dt; //!< R: Actual, measured control cycle time (seconds)
   
   //Joint space controller
   btstatecontrol Jsc; //!< R: The state controller for joint space. Initialized in #OpenWAM()
   btPID_array JposControl; //!< R: Array of btPID control objects (data structures) used for joint position control
   vect_n *Jpos; //!< R: Joint position (rad) (read-only)
   vect_n *Jvel; //!< R: Joint velocity (rad/s) (read-only)
   vect_n *Jacc; //!< R: Joint acceleration (rad/s/s) (read-only)
   vect_n *Jtrq; //!< R/W: Joint torque command (Nm). If you write/use you own torque controller, you can use this.
   vect_n *Jref; //!< R/W: Joint position reference/command (rad). If you write/use your own position controller, you can use this.
   vect_n *Jtref; //!< R: Joint trajectory progress reference. Tracks progress through a trajectory.
   vect_n *vel, *acc; //!< W: Velocity and acceleration for joint space moves
   
   //Cartesian space controller
   btstatecontrol Csc; //!< R: The state controller for Cartesian space. Initialized in #OpenWAM()
   btPID_array CposControl; //!< R: Array of btPID control objects (data structures) used for Cartesian position control
   vect_n *Ttrq; //!< R: Joint torques generated by RNE force/torque + gravity calculations
   vect_3 *Cpos; //!< R: Cartesian end-point position XYZ (m)
   vect_3 *Cvel; //!< R: Cartesian end-point velocity (m/s). 
   vect_3 *Cacc; //!< R: Cartesian end-point acceleration (m/s/s). 
   vect_3 *Cforce; //!< R/W: Cartesian force XYZ (N). If you write to it, you should call #apply_tool_force_bot()
   vect_3 *Ctrq; //!< R/W: Cartesian torque RxRyRz (Nm). If you write to it, you should call #apply_tool_force_bot()
   //vect_3 *Cref; //!< Cartesian position reference XYZ (m). 
   vect_3 *Cpoint; //!< W: XYZ translation (constant) from the tool center point (TCP) to the Cartesian control point (end-point)
   matr_h *HMpos; //!< R: Homogeneous Matrix of the tool transform (Position)
   matr_h *HMvel; //!< R: Tool Velocity
   matr_h *HMacc; //!< R: Tool Acceleration
   matr_h *HMref; //!< W: Tool Position reference
   matr_h *HMtref; //!< R: Tool Trajectory progress reference. Tracks progress through a trajectory.
   matr_h *HMft; //!< R: Tool Force/torque matrix. If you want to control a force/torque, use #apply_tool_force_bot()
   
   // Quaternian control (unused)
   quat *qref,*qact,*qaxis,*forced; //!< Reference and actual orientations for quaternion control
   btreal qerr; //!< Quaternian error
   
   //Cartesian Controllers
   //btPID pid[12]; //  x,y,z,rot
   //vect_n *R6pos,*R6vel,*R6acc,*R6ref,*R6tref,*R6force,*R6trq;
   
   //btPID d_pos_ctl[12];
   //btPID_array d_pos_array;

   //Cartesian space moves
   //bttraptrj trj;
   //btpath_pwl pth;
   
   vect_n *Gtrq; //!< R: Joint torques due to gravity compensation
   //btreal F;

   //Loop timing info
   RTIME loop_time; //!< R: Time in nanoseconds from start to end of control loop processing.
   RTIME loop_period; //!< R: Sample rate.
   RTIME readpos_time; //!< R: Total Time to read positions
   RTIME writetrq_time; //!< R: Total Time to send torques
   RTIME user_time; //!< R: Time spent in user callback
   RTIME Jsc_time; //!< R: Time spent in joint control
   RTIME Csc_time; //!< R: Time spent in Cartesian control
   double skipmax;
   btrt_mutex loop_mutex; //!< R/W: This mutex is set while the wam control loop is in operation. Use #btrt_mutex_lock() to access, #btrt_mutex_unlock() to release.

   //Data logging
   btlogger log; //!< R: Structure for storing log data. Populated by the logging functions.
   int logDivider; //!< W: 1 = Log data every control cycle, 2 = Log data every 2nd control cycle, etc. Default = 1.
   int logCounter; //!< R: Control loop counter so we know when to log the data
   btreal log_time; //!< R: Total active log duration (seconds)
   
   //Continuous path record
   btlogger cteach; //!< R: Structure for storing teach data. Populated by the teach & play functions.
   int teachDivider; //!< W: 1 = Log data every control cycle, 2 = Log data every 2nd control cycle, etc. See #StartContinuousTeach()
   int teachCounter; //!< R: Control loop counter so we know when to log the data
   btreal teach_time; //!< R: Total active teach duration (seconds)

   btrt_thread_struct maint; //!< R: Periodic (default 10Hz) maintenance thread for teach & play. Configured in #OpenWAM().
}wam_struct;



/*************  WAM  API  ******************/

wam_struct* OpenWAM(char *wamfile, int bus); //NULL -> wam.conf
void CloseWAM(wam_struct* wam); //Cleanupint BlankWAMcallback(struct btwam_struct *wam);

void registerWAMcallback(wam_struct* wam,void *func);
void registerWAMmotorcallback(wam_struct* wam,void *func);
void WAMControlThread(void *data); //data points to wam_struct* wam
void WAMControlThread1(void *data); //data points to wam_struct* wam
void WAMControlThread2(void *data); //data points to wam_struct* wam

void DefineWAMpos(wam_struct *w,vect_n *wv);

btreal GetGravityComp(wam_struct *w);
void SetGravityComp(wam_struct *w,btreal scale);
void SetGravityUsingCalibrated(wam_struct *w, int onoff);
int GetGravityUsingCalibrated(wam_struct *w);
int GetGravityIsCalibrated(wam_struct *w);

void SetCartesianSpace(wam_struct* wam);
void SetJointSpace(wam_struct* wam);
void SetPositionConstraint(wam_struct* wam, int onoff);

void MoveSetup(wam_struct* wam,btreal vel,btreal acc);
void MoveWAM(wam_struct* wam,vect_n * dest);
int MoveIsDone(wam_struct* wam);
void MoveStop(wam_struct* wam);

void ParkWAM(wam_struct* wam);
// Continuous Teach & Play Recording
void StartContinuousTeach(wam_struct *wam, int divider, char *filename);
void StopContinuousTeach(wam_struct* wam);
void ServiceContinuousTeach(wam_struct* wam);

void setSafetyLimits(int bus, double jointVel, double tipVel, double elbowVel);

/******************************************/
/** \internal Below this line all functions need work
 
*/



int AddEndpointForce(wam_struct* wam,vect_n *force); //Cartesian only

//MovePause(wam_struct* wam);
//MoveContinue(wam_struct* wam);



long GetTime(); //Wrapper for rt_get_cpu_time_ns()
long GetElapsedTime(); //Static time variable


/******************************************/


#endif /*_BTWAM_H*/
