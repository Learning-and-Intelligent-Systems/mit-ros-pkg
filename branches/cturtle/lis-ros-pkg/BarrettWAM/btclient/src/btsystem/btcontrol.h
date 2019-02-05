/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... btcontrol.h
 *  Creation Date ...... 31 Mar 2005
 *  Author ............. Traveler Hauptman
 *  Addtl Authors ...... Brian Zenowich
 *                       Sam Clanton
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

/*! \file btcontrol.h
    \brief System control algorithms and objects. 
 
 
    
-# PID object: #btPID
-# Via Trajectory object: #via_trj

See the control module for function documentation. \ref control

Both objects implement the virtual interface required by #btstatecontrol. Since
#btstatecontrol is expecting a  #vect_n to be returned we extend those objects
with #btPID_array and #via_trj_array.
 
\internal chk'd TH 051103
*/
/** @addtogroup control Control algorithms
 */
//@{
//@}
#ifndef _BTCONTROL_H
#define _BTCONTROL_H

#include "btmath.h"
#include "btos.h"
#include "btseg.h"
#include "btstatecontrol.h"

#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */

//#include <pthread.h>

#ifndef PI
#define PI 3.141596
#endif /*PI*/

/*================================================PID stuff================================*/
/** @addtogroup pid PID position regulator
    @ingroup control
    - see #btPID for an object overview
 */
//@{
/** PID regulator

btPID maintains configuration and state information for a generic PID controller.

init_btPID() or init_err_btPID() initialize the data structure. init_err_btPID is
used when you will be feeding btPID the error (when a non-euclidian metric is used for 
instance).

btPID maintains it's internal state. Thus you may feed it all information each 
cycle (measured value 'y', reference value 'yref', time step 'dt') or if these 
are typically unchanging you can send them only when they change and call step_btPID().

\code
{
  btPID pid;
  btreal dt,targ_pos,meas_pos,torque;
  
  init_btPID(&pid);
  setgains_btPID(&pid,1000.0,20.0,1.0);
  
  start_btPID(&pid);
  
  dt = .001;
  targ_pos = 10.0;
  while(1){
    meas_pos = Some_get_pos_func();
    torque = eval_btPID(&pid,meas_pos,targ_pos,dt);
    Some_set_torque_func(torque);
  }

\endcode
\internal chk'd TH 051103
*/
typedef struct  
{
  btreal Kp; //!< Proportional gain
  btreal Kd; //!< Derivative gain
  btreal Ki; //!< Integral gain

  btreal e;  //!< Error (yref - y)
  btreal de; //!< Derivative of error
  btreal fe;
  btreal se; //!< Sum of error
  
  btreal laste;
  int firsttick;   //!< handles special case de for first tick
  int state; //!< 0=off, 1=on
  int external_error_calc; //!< If this is !=0 we expect that the error calculation is done externally
  btreal dt;        //!< Defaults to 1
  btreal saturation; //!< Defaults to zero (doesn't ever saturate)

  btreal y; //!< Measured control parameter
  btreal yref; //!< Commanded control parameter
  btreal lastresult; //!< Last output of the regulator
 
  btrt_mutex mutex;
}btPID;

typedef struct
{
  btPID* pid;
  int elements;
}btPID_array;

//Create-Destroy
void init_btPID(btPID *pid);
void init_err_btPID(btPID *pid); //setup for use with eval_err_btPID
//Data Access
void setgains_btPID(btPID *pid, btreal Kp, btreal Kd, btreal Ki);
void setsaturation_btPID(btPID *pid, btreal saturation);
void getgains_btPID(btPID *pid, btreal *Kp, btreal *Kd, btreal *Ki);
void getsaturation_btPID(btPID *pid, btreal *saturation);
void getinputs_btPID(btPID *pid, btreal *y, btreal *yref, btreal *dt);


//State Control
void reset_btPID(btPID *pid);
void stop_btPID(btPID *pid);
void start_btPID(btPID *pid);

//Synchronous Evaluation
btreal eval_btPID(btPID *pid, btreal y, btreal yref, btreal dt);
btreal eval_err_btPID(btPID *pid, btreal error, btreal dt);

//Asynchronous Evaluation
void setinputs_btPID(btPID *pid, btreal y, btreal yref, btreal dt);
void sety_btPID(btPID *pid, btreal y);
void setyref_btPID(btPID *pid, btreal yref);
void setdt_btPID(btPID *pid, btreal dt);
btreal step_btPID(btPID *pid);
btreal lastresult_btPID(btPID *pid);

//btposition_interface stuff
void btposition_interface_pause_btPID(struct btposition_interface_struct* btp);
void btposition_interface_reset_btPID(struct btposition_interface_struct* btp);
vect_n* btposition_interface_eval_btPID(struct btposition_interface_struct* btp);
void btposition_interface_mapf_btPID(btstatecontrol *sc, btPID_array *pid);
//@} 

/*================================================Trajectory stuff================================*/
/**  Linear interpolation trajectory
   
 */
//@{
/** A piecewize linear interpolating trajectory

  ct_traj stores an array of time,position data and 
  plays it back by linear interpolation.
  
  This was primarily developed as a quick way to test the virtual interfaces of
  #btstatecontrol. You probably want to use #via_trj_array instead.
  
  \internal chk'd TH 051103
*/
typedef struct 
{
  btpath_pwl *pwl;
  
  int state;
  btreal start_error; //maximum error between present location and starting location
  
}ct_traj;

void create_ct(ct_traj *trj,vectray *vr);
int readfile_ct(ct_traj* ct,char* filename);
vect_n* init_ct(ct_traj *trj);
vect_n* eval_ct(ct_traj *trj, btreal dt);

int done_ct(ct_traj *trj);
int bttrajectory_interface_getstate_ct(struct bttrajectory_interface_struct *btt);
vect_n* bttrajectory_interface_reset_ct(struct bttrajectory_interface_struct *btt);
vect_n* bttrajectory_interface_eval_ct(struct bttrajectory_interface_struct *btt);
void bttrajectory_interface_mapf_ct(btstatecontrol *sc,ct_traj *trj);

//@}

/** Segment calculation object.

*/
typedef struct {
  btreal vel;
  btreal acc1;
  btreal acc2;
  btreal dt_vel,dt_acc1,dt_acc2;
  btreal q_acc1;
  btreal q_vel;
  btreal q_acc2;
}Seg_int;
void CalcSegment(Seg_int *seg,double q1, double q2, double t1, double t2, 
double t3, double v_prev, double v_next, double seg_acc, int end);

/** @addtogroup vta Trajectory with parabolic blending at via points
    @ingroup control
    
    - see #via_trj_array for an object overview
 */
 //@{                   
/** A piecewise linear trajectory with parabolic blending.

see #via_trj_array for more info.
\internal chk'd TH 051103
*/
typedef struct 
{
  int state;
  double t;
  int col; //column of point list table
  int idx,n;
  double Qdot,Qdot_next;
  double v_prev,v_next;
  Seg_int seg;
  
  
  //===========parameters
  double trj_acc; //acceleration that controls the blending
  double trj_vel;
  
  //===========state
  int segment; //0 = acc, 1=vel
  double dt_acc,dt_vel; 
  double t_acc, t_vel, q_acc, q_vel;
  double t0,q0,v0;
  double qn,qp,tn,tp,t3;
  double acc,vel;
  double last_vel;
  double last_cmd; //added for temp debugging
  double last_et;//added for temp debugging
  
  vectray *vr; //Pointer to a trajectory file
  int row; //Present row of the trajectory file
  int rows; //Total number of rows in trajectory file 
     
  
}via_trj;


/*internal*/
void SetAcc_vt(via_trj *trj,double acc);
double eval_via_trj(via_trj *trj,double dt);
double start_via_trj(via_trj *trj,int col);


/** A piecewise linear trajectory with parabolic blending using #vect_n data.

via_trj_array objects (hereafter called vta objects) must be allocated by 
calling new_vta() and destroyed (deallocated) by calling destroy_vta().

All of the member functions will gracefully do nothing if passed a NULL object pointer.

A simple edit api is provided to simplify programming user interaction with this 
data structure.

btcontrol.h has the API documentation.

This data structure is used for implementing a trajectory object for the
#btstatecontrol object. 

\warning If sequential time values are the same or out of order the behavior is undefined.

See also example_3 of the code examples.
Example code:
\code
int cnt;
via_trj_array *vta = NULL;
vect_n* pos1,pos2;

pos1 = new_vn(6);
pos2 = new_vn(6);
fill_vn(pos2,10.0);

vta = new_vta(6,50); //6 DOF, Maximum of 50 edit points
register_vta(active_bts,vta); //Register trajectory with state controller

for(cnt = 0; cnt < 15; cnt++){ //Add path that goes back & forth 15 times
  ins_point_vta(vta,pos1);
  ins_point_vta(vta,pos2);
}

  setmode_bts(active_bts,SCMODE_TRJ);
  moveparm_bts(active_bts,0.5,0.5);
  prep_trj_bts(active_bts);
 
  while (movestatus_bts(active_bts) == BTTRAJ_INPREP)
  {
      usleep(100000); //give up processor for other threads
  }

  start_trj_bts(active_bts);
  sleep(60); //run for one minute
  stop_trj_bts(active_bts);
 
\endcode


\internal chk'd TH 051103
*/
typedef struct
{
  via_trj* trj;
  int elements;
  /** 
  
  When adding new points this velocity is used to calculate new time values.
  */
  double vel,acc;  
  pararray_vn *pavn;
  vectray *vr;
  double t;
}via_trj_array;
/*Memory Management API*/
//via_trj_array* malloc_vta(int num_columns);

via_trj_array* new_vta(int num_columns,int max_rows);
void destroy_vta(via_trj_array** vt);
vectray* get_vr_vta(via_trj_array* vt);

// Edit API
void next_point_vta(via_trj_array* vt);
void prev_point_vta(via_trj_array* vt);
void first_point_vta(via_trj_array* vt);
void last_point_vta(via_trj_array* vt);
int ins_point_vta(via_trj_array* vt, vect_n *pt);
int del_point_vta(via_trj_array* vt);
int get_current_idx_vta(via_trj_array* vt);
int set_current_idx_vta(via_trj_array* vt,int idx);
void get_current_point_vta(via_trj_array* vt, vect_n *dest);

int dist_adjust_vta(via_trj_array* vt,double vel); //
int time_scale_vta(via_trj_array* vt,double s);
void set_acc_vta(via_trj_array* vt,btreal acc);
void set_vel_vta(via_trj_array* vt,btreal acc);
//File I/O
void write_file_vta(via_trj_array* vt,char *filename);
via_trj_array* read_file_vta(char* filename,int extrapoints);

//Use
vect_n* eval_vta(via_trj_array* vt,double dt,vect_n* qref);
vect_n* reset_vta(via_trj_array* vt,double dt,vect_n* qref);
vect_n* sim_vta(via_trj_array* vt,double dt,double duration,char*filename);

/* Interface */
int bttrajectory_interface_getstate_vt(struct bttrajectory_interface_struct *btt);
vect_n* bttrajectory_interface_reset_vt(struct bttrajectory_interface_struct *btt);
vect_n* bttrajectory_interface_eval_vt(struct bttrajectory_interface_struct *btt);
void register_vta(btstatecontrol *sc,via_trj_array *vt);
//@}

#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif/* _BTCONTROL_H */
