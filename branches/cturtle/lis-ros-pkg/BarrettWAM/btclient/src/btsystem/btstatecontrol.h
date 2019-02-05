/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... btstatecontrol.h
 *  Creation Date ...... 28 Apr 2005
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

/*! \file btstatecontrol.h
 
  \brief Virtual interfaces for control functions
  
  #btstatecontrol is an object that will handle bumpless state transitions
  for a position constraint controller and a trajectory generator.
  
  #bttraptrj is a one dimensional trapezoidal trajectory generator; used by 
  btstatecontrol for simple movement.
  
  #btramp is an object that creates sliding transitions between two extremes.
  it is used by btstatecontrol for time warping. 
*/
#ifndef _BTCONTROL_VIRT_H
#define _BTCONTROL_VIRT_H

//#include <pthread.h>
#include "btmath.h"
#include "btos.h"
#include "btpath.h"

#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */


#ifndef PI
#define PI 3.14159265359
#endif /*PI*/

/*===============================Ramp object================================*/

enum btramp_state {
  BTRAMP_MAX = 0, //!< scaler is set to the minimum value
  BTRAMP_MIN, //!< scaler is set to the min value each evaluation
  BTRAMP_UP, //!< Scaler is increased by rate*dt each evaluation. When scaler >= BTRAMP_MAX, the state changes to BTRAMP_MAX
  BTRAMP_DOWN, //!< Scaler is decreased by rate*dt each evaluation. When scaler >= BTRAMP_MIN, the state changes to BTRAMP_MIN
  BTRAMP_PAUSE //!< Default = Scaler is not touched.
};
/** Constant acceleration function.

btramp is used to smoothly transition a variable from one value to another over 
a period of time. It is thread safe. See init_btramp().

Use set_btramp() to change the state and control the 
btramp object. 

see #btramp_state for valid states.
\internal chk'd TH 051102
*/

typedef struct 
{
  btreal *scaler;
  btreal min,max;
  btreal rate; // dscaler/dt units per second
  int state; 
  btrt_mutex mutex;
}btramp;

void init_btramp(btramp *r,btreal *var,btreal min,btreal max,btreal rate);
void set_btramp(btramp *r,enum btramp_state state);
void setrate_btramp(btramp *r,btreal rate);
btreal get_btramp(btramp *r);
btreal eval_btramp(btramp *r,btreal dt);
int getstate_btramp(btramp *r);
btreal rate_eval_btramp(btramp *r,btreal dt,btreal rate);

/*===============================Ramp object================================*/

/**
\internal
Trajectory Actions and state changes
 - Engage: [-1,5,4,0]->0 : yref = y; Start the constraint block at the present position
 - DisEngage: [0,4]->-1 : Turn off the constraint block; [2-8]->0 : Nothing
 - EStop: [2-8]->0: Stop trajectory, reset
 - PrepTrj: 0->3: Set up move from present location to trajectory start position
 - 3->4: Done moving to start position. Wait for StartTrj or autostart
 - StartTrj: 4->5: Start running trajectory
 - Pause(t): 5->6->7: Ramp dt down to 0 over t seconds
 - Unpause(t): 7->8->5: Ramp dt up from 0 over t seconds
 - LoadTrj: 1->1,2->2,9->2: Set up trajectory for operation
*/
enum trjstate {
  BTTRAJ_OFF = -1, //!< This mode is set on an error
  BTTRAJ_STOPPED = 0, //!< The trajectory is not running
  BTTRAJ_INPREP, //!< We are in motion from the initial position to the start position of the trajectory
  BTTRAJ_READY, //!< We are at the start position; waiting for the trajectory to be started.
  BTTRAJ_RUN, //!< The trajectory is running.
  BTTRAJ_DONE, //!< The reference point is at the end and no longer moving. The trajectory is done.
  BTTRAJ_PAUSING, //!< Time is being stretched to "pause" the trajectory.
  BTTRAJ_UNPAUSING, //!< Time is being compressed toward real-time to "unpause" the trajectory.
  BTTRAJ_PAUSED //!< Time has been stopped.
};
 
enum scstate {
  SCMODE_IDLE=0, //!< Evaluation alway returns 0.0 (or a vector filled with 0.0)
  SCMODE_TORQUE, // depreciated
  SCMODE_POS, //!< The position constraint code is evaluated.
  SCMODE_TRJ //!< A trajectory is active and updating the reference point automatically.
};
/*=======================Trapezoidal Trajectory object======================*/
/** Trapezoidal trajectory generator.

  bttraptrj provides configuration and state information for a set of trapezoidal trajectory
  generation functions specified in btstatecontrol.c.
  
  Make sure you set acc, and vel. These are the constant acceleration used and the 
  maximum velocity allowed respectively.

  \internal chk'd TH 051102
*/

typedef struct 
{
  //state machine
  int state; //!< 0: done, 1:run
  
  //internal state
  btreal cmd;
  btreal end;
  btreal acc;
  btreal vel;
  btreal t;
  btreal t1,t2; // Calculated inflection points of trapezoidal velocity curve
  btreal x1,x2;

}bttraptrj;

btreal evaluate_traptrj(bttraptrj *traj,btreal dt);
void start_traptrj(bttraptrj *traj, btreal dist);
void setprofile_traptrj(bttraptrj *traj, btreal vel, btreal acc);
/*=======================Trapezoidal Trajectory object======================*/

/*===============================Position object============================*/
/** A virtual interface for position control

\internal chk'd TH 051102
*/
typedef struct btposition_interface_struct
{
  void *dat;
  
  //vect_n* (*init)(void *dat, vect_n* q, vect_n* qref);
  vect_n* (*eval)(struct btposition_interface_struct* btp);
  void (*reset)(struct btposition_interface_struct* btp);
  void (*pause)(struct btposition_interface_struct* btp);
  //void (*set_ref)(void *dat, vect_n* ref);
 
  vect_n *q,*dq,*ddq; //Buffer for present state
  vect_n *qref; //Buffer for desired state
  double *dt;
  
  vect_n *t; //Buffer for output torque

  btrt_mutex mutex; //unused?
}btposition_interface;

//setup
void mapdata_btpos(btposition_interface *btp,vect_n* q, vect_n* dq, vect_n* ddq, 
                   vect_n* qref, vect_n* t, double *dt);

/*================================================Trajectory object================================*/
/** A virtual interface for trajectories object.
bttrajectory sets up virtual functions for running an arbitrary trajectory along
an arbitrary curve. Curve and trajectory initialization are done by the objects
using this interface.

see #trjstate for more state info
\internal chk'd TH 051102
*/
typedef struct bttrajectory_interface_struct
{
  double *dt; // pointer to location of dt info
  vect_n *qref; //results
  
  void *dat; //Trajectory object pointer
  
  vect_n* (*reset)(struct bttrajectory_interface_struct *btt);
  vect_n* (*eval)(struct bttrajectory_interface_struct *btt);
  int (*getstate)(struct bttrajectory_interface_struct *btt);

  // Straight trajectory
  btpath_pwl pth;
  bttraptrj trj;
  int state;
  btrt_mutex mutex;  //unused?
}bttrajectory_interface;
//void init_bttrj(bttrajectory_interface *btt);
void mapdata_bttrj(bttrajectory_interface *btt, vect_n* qref, double *dt);

/*================================================State Controller object====================*/
/*! \brief A state controller for switching between position control and torque control

This structure stores state information along with pid and trajectory info for
a simple state controller. The primary function of this object is to provide bumpless transfer between position 
control, moving trajectory control, and no control (idle)

If the position control is off (getmode_sc() returns SCMODE_IDLE), a zero torque is returned.

Interlocks:
SCMODE is set to POS or IDLE by user. SCMODE is automatically escalated to TRJ from POS by movement commands

Typical use:
Initialization:
\code
  btstatecontrol Jsc;
  vect_n *Mpos,*Mtrq,*Jpos,*Jvel,*Jacc,*Jref,*Jtref,*Jtrq;
  double dt;
  
  //<snip> initialize vect_n objects
  
  init_bts(&Jsc);
  map_btstatecontrol(&Jsc, Jpos, Jvel, Jacc, 
                      Jref, Jtrq, &dt);
  btposition_interface_mapf_btPID(&Jsc, &(d_jpos_array));
\endcode
Use:
\code
  //A trajectory object that implements the sc virtual function interface
  via_trj_array *vta; 
  
  vta = read_file_vta("teach.csv"); //create a trajectory object
  
  register_vta(&Jsc,vta); //wrapper for maptrajectory_bts() see btcontrol.c
  
  //notice that once we register our specific trajectory object, everything 
  //is done through the generic _bts() functions.
  
  //Specify velocity and acceleration when moving from present position to start.
  moveparm_bts(&Jsc,0.5,0.5); 
  
  //Move the wam to the start position of the trajectory
  //And then start the trajectory we registered
  start_trj_bts(&Jsc); 

  sleep(10); //run for 10 seconds
  stop_trj_bts(&wam->Jsc); //stop the trajectory
\endcode
Meanwhile in another loop:
\code
  while(1){
    get_positions();
    eval_bts(&(WAM.Jsc));
    set_torques();
  }
\endcode

You can implement a position controller and a trajectory controller. Both can be 
"cold swapped". ie, you can turn off your controller, replace it with another control
object dynamically and then turn it back on.

See the via_trj_array object for an example of implementing an object that can be
plugged in. For a trajectory we implement an eval, reset, and getstate function. 
These are then registered using the maptrajectory_bts() function.
\code
int bttrajectory_interface_getstate_vt(struct bttrajectory_interface_struct *btt);
vect_n* bttrajectory_interface_reset_vt(struct bttrajectory_interface_struct *btt);
vect_n* bttrajectory_interface_eval_vt(struct bttrajectory_interface_struct *btt);
void register_vta(btstatecontrol *sc,via_trj_array *vt);

void register_vta(btstatecontrol *sc,via_trj_array *vt)
{ 
  maptrajectory_bts(sc,(void*) vt,
                    bttrajectory_interface_reset_vt,
                    bttrajectory_interface_eval_vt,
                    bttrajectory_interface_getstate_vt);
}
\endcode

\internal chk'd TH 051102
\todo btstatecontrol trajectory looping is a kludge. We really should handle 
this at the trajectory level


*/

typedef struct
{
  int mode; //!< 0:idle, 1:torque, 2:pos 3:pos + trj
  vect_n* t; //!< Internal buffer for torques
  vect_n* q,*dq,*ddq,*qref,*tref; //!< Internal buffer for position and reference position

  double *dt;
  double local_dt; //for time warping...
  double dt_scale;
  btramp ramp;
  double last_dt; //history: the last time step used.
  
  btposition_interface btp;
  bttrajectory_interface btt;
  int error; //nonzero if there are any errors

  // Straight trajectory
  btpath_pwl pth;
  bttraptrj trj;
  btreal vel,acc;
  int prep_only;
  btrt_mutex mutex;
}btstatecontrol;
void map_btstatecontrol(btstatecontrol *sc, vect_n* q, vect_n* dq, vect_n* ddq, 
                                           vect_n* qref, vect_n* tref,vect_n* t, double *dt);
int init_bts(btstatecontrol *sc);
vect_n* eval_bts(btstatecontrol *sc);

int setmode_bts(btstatecontrol *sc, int mode);
int getmode_bts(btstatecontrol *sc);
int get_trjstate_bts(btstatecontrol *sc);
int movestatus_bts(btstatecontrol *sc);


void moveparm_bts(btstatecontrol *sc,btreal vel, btreal acc);
int moveto_bts(btstatecontrol *sc,vect_n* dest);
int start_trj_bts(btstatecontrol *sc);
int stop_trj_bts(btstatecontrol *sc);
int pause_trj_bts(btstatecontrol *sc,btreal period);
int unpause_trj_bts(btstatecontrol *sc,btreal period);


#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif/* _BTCONTROL_H */
