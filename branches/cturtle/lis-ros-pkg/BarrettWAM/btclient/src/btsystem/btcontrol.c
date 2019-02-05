/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... btcontrol.c
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

#ifdef S_SPLINT_S
#include <err.h>
#endif
#include <math.h>
#include <pthread.h>
#include <errno.h>
#include <syslog.h>
#include <stdio.h>
#include <stdlib.h>
#include "btmath.h"
#include "btcontrol.h"
#include "btos.h"
#include "btstatecontrol.h"
#include "btseg.h"

#define BT_DUMMY_PROOF

#define sign(x) (x>=0?1:-1)
#define Sgn(x) (x>=0.0?1.0:-1.0)

/**************************************************************************/

/*! Initializes the btPID object
 
The function sets the main parameters of a PID regulator. You
will only have to use this function a single time when you create the variable. Before
you start using eval_btPID, remember to sety_btPID() and setyref_btPID().
 
\internal chk'd TH 051103
*/
void init_btPID(btPID *pid)
{
   pid->Kp = 0.0;
   pid->Kd = 0.0;
   pid->Ki = 0.0;
   pid->dt = 1.0;

   pid->se = 0;
   pid->e = 0;
   pid->laste = 0;
   pid->de = 0;
   pid->firsttick = 1;
   pid->y = 0;
   pid->yref = 0;
   pid->lastresult = 0;
   pid->state = 0;
   pid->external_error_calc = 0;

   btrt_mutex_init(&pid->mutex);

}
/** Initialize a btPID object for use with an external error.
 \internal chk'd TH 051103
*/
void init_err_btPID(btPID *pid)
{
   pid->external_error_calc = 1;
}
/** Reset the state parameters of the PID regulator
 
  Sets the accumulated error to zero, the reference value to the present value, and notifies
  the algorithm that it is re-starting.
 
  \internal chk'd TH 051103
  */
void reset_btPID(btPID *pid)
{
   btrt_mutex_lock(&(pid->mutex));

   pid->se = 0;
   pid->firsttick = 1;
   pid->yref = pid->y;

   btrt_mutex_unlock(&(pid->mutex));
}

/** Start the PID regulator
 
 Resets the regulator (see reset_btPID()) and enables the evaluator.
  
  
  
  \internal chk'd TH 051103
*/
void start_btPID(btPID *pid)
{
   btrt_mutex_lock(&(pid->mutex));

   pid->se = 0;
   pid->firsttick = 1;
   pid->yref = pid->y;
   pid->state = 1;

   btrt_mutex_unlock(&(pid->mutex));
}
/** Stop the PID regulator
 
  Disables the evaluator
  
  \internal chk'd TH 051103
*/
void stop_btPID(btPID *pid)
{
   btrt_mutex_lock(&(pid->mutex));

   pid->state = 0;

   btrt_mutex_unlock(&(pid->mutex));
}
/** Increments the state by dt and returns the output of the regulator.
 
step_btPID() updates the btPID state variable by incrementing the time by dt
and then calculates the error between y and yref.
 
\param *pid A pointer to the pre-allocated and initialized PIDregulator structure.
 
\return The regulator output for the present dt, y, and yref.
\internal chk'd TH 051103
*/
btreal step_btPID(btPID *pid)
{
   btrt_mutex_lock(&(pid->mutex));

   if (pid->state) {
      if (!pid->external_error_calc) {
         pid->e = pid->yref - pid->y;
      }
      if (pid->firsttick) {
         pid->laste = pid->e;
         pid->firsttick = 0;
      }

      pid->de = (pid->e-pid->laste)/pid->dt;  //Backward euler
      pid->laste = pid->e;

      pid->se += pid->e;

      if (pid->saturation != 0)
         if ((fabs(pid->se)*pid->Ki) > pid->saturation)
            pid->se = sign(pid->se)*pid->saturation;

      pid->lastresult = pid->Kp*pid->e+pid->Kd*pid->de+pid->Ki*pid->se;


   } else {
      pid->lastresult = 0.0;
   }

   btrt_mutex_unlock(&(pid->mutex));
   return(pid->lastresult); //bz
}
/** Set measured value 'y'.
 
See #btPID object of more info.
\internal chk'd TH 051103
*/
void sety_btPID(btPID *pid, btreal y)
{
   btrt_mutex_lock(&(pid->mutex));

   pid->y = y;

   btrt_mutex_unlock(&(pid->mutex));
}
/** Set reference value 'yref'.
 
See #btPID object of more info.
\internal chk'd TH 051103
*/
void setyref_btPID(btPID *pid, btreal yref)
{
   btrt_mutex_lock(&(pid->mutex));

   pid->yref = yref;

   btrt_mutex_unlock(&(pid->mutex));
}
/** Set time step value 'dt'.
 
See #btPID object of more info.
\internal chk'd TH 051103
*/
void setdt_btPID(btPID *pid, btreal dt)
{
   btrt_mutex_lock(&(pid->mutex));

   pid->dt = dt;

   btrt_mutex_unlock(&(pid->mutex));
}
/** Get the last calculated effort value.
 
See #btPID object of more info.
\internal chk'd TH 051103
*/
btreal lastresult_btPID(btPID *pid)
{
   btreal ret;
   btrt_mutex_lock(&(pid->mutex));

   ret = pid->lastresult;

   btrt_mutex_unlock(&(pid->mutex));

   return ret;
}
/** Evaluate the btPID object.
 
See #btPID object of more info.
\internal chk'd TH 051103
*/
btreal eval_btPID(btPID *pid, btreal y, btreal yref, btreal dt)
{
   btrt_mutex_lock(&(pid->mutex));

   pid->y = y;
   pid->yref = yref;
   pid->dt = dt;

   btrt_mutex_unlock(&(pid->mutex));

   return step_btPID(pid);
}
/** Evaluate a pid object using the provided error.
See #btPID object of more info.
\internal chk'd TH 051103
*/
btreal eval_err_btPID(btPID *pid, btreal error, btreal dt)
{
   btrt_mutex_lock(&(pid->mutex));

   pid->e = error;
   pid->dt = dt;

   btrt_mutex_unlock(&(pid->mutex));

   return step_btPID(pid);
}
/** Set all the input values.
 
See #btPID object of more info.
\internal chk'd TH 051103
*/
void setinputs_btPID(btPID *pid, btreal y, btreal yref, btreal dt)
{
   btrt_mutex_lock(&(pid->mutex));

   pid->y = y;
   pid->yref = yref;
   pid->dt = dt;

   btrt_mutex_unlock(&(pid->mutex));
}
/** Set the gains.
 
If the library is compiled with BT_DUMMY_PROOF, we will verify that you only
change the gains when the regulator is in the off state. 
See #btPID object of more info.
\internal chk'd TH 051103
*/
void setgains_btPID(btPID *pid, btreal Kp, btreal Kd, btreal Ki)
{
   btrt_mutex_lock(&(pid->mutex));

#ifdef BT_DUMMY_PROOF

   if (!pid->state) {  //Only allow setting gains when the regulator is not active
#endif

      pid->Kp = Kp;
      pid->Kd = Kd;
      pid->Ki = Ki;

#ifdef BT_DUMMY_PROOF

   } else {
      syslog(LOG_ERR,"setgains_btPID ignored because PID regulator is active");
   }
#endif

   btrt_mutex_unlock(&(pid->mutex));
}
/** Set saturation of the regulator.
 
If set anti-windup will kick in above this value. See #btPID object of more info.
\internal chk'd TH 051103
\todo verify anti-windup functionality
*/
void setsaturation_btPID(btPID *pid, btreal saturation)
{
   btrt_mutex_lock(&(pid->mutex));

   pid->saturation = saturation;

   btrt_mutex_unlock(&(pid->mutex));
}
/** Get the present gain values.
\internal chk'd TH 051103
*/
void getgains_btPID(btPID *pid, btreal *Kp, btreal *Kd, btreal *Ki)
{
   btrt_mutex_lock(&(pid->mutex));

   *Kp = pid->Kp;
   *Kd = pid->Kd;
   *Ki = pid->Ki;

   btrt_mutex_unlock(&(pid->mutex));
}
/** Get the present input values
\internal chk'd TH 051103
*/
void getinputs_btPID(btPID *pid, btreal *y, btreal *yref, btreal *dt)
{
   btrt_mutex_lock(&(pid->mutex));

   *y = pid->y;
   *yref = pid->yref;
   *dt = pid->dt;

   btrt_mutex_unlock(&(pid->mutex));
}
/** Get the present saturation values
\internal chk'd TH 051103
*/
void getsaturation_btPID(btPID *pid, btreal *saturation)
{
   btrt_mutex_lock(&(pid->mutex));

   *saturation = pid->saturation;

   btrt_mutex_unlock(&(pid->mutex));
}
/************************* btPID interface functions ***************************/
/* Repackages the above PID routines for vectors
   Provides the default function plugins for the btstate controller (in btstatecontrol)
*/
void btposition_interface_mapf_btPID(btstatecontrol *sc, btPID_array *pid)
{
   mapposition_bts(sc,(void*) pid,
                   btposition_interface_reset_btPID,
                   btposition_interface_eval_btPID,
                   btposition_interface_pause_btPID);
}

vect_n* btposition_interface_eval_btPID(struct btposition_interface_struct* btp)
{
   btPID_array *pids;
   btPID* this;
   int cnt;

   pids = (btPID_array*)btp->dat;

   for (cnt = 0; cnt < pids->elements; cnt++)
   {
      this = &(pids->pid[cnt]);
      sety_btPID(this,getval_vn(btp->q,cnt));//load all pointer data
      setyref_btPID(this,getval_vn(btp->qref,cnt));//load all pointer data
      setdt_btPID(this,*(btp->dt));
      step_btPID(this);//eval
      setval_vn(btp->t,cnt,lastresult_btPID(this));
   }
   return btp->t;
}

void btposition_interface_reset_btPID(struct btposition_interface_struct* btp)
{
   btPID_array *pids;
   int cnt;

   pids = (btPID_array*)btp->dat;

   for (cnt = 0; cnt < pids->elements; cnt++)
   {
      start_btPID(&(pids->pid[cnt]));
   }
}

void btposition_interface_pause_btPID(struct btposition_interface_struct* btp)
{
   btPID_array *pids;
   int cnt;

   pids = (btPID_array*)btp->dat;

   for (cnt = 0; cnt < pids->elements; cnt++)
   {
      stop_btPID(&(pids->pid[cnt]));
   }
}

/**************************** Continuous trajectory ***************************/

/**
\internal chk'd TH 051103
Not thought to be useful so not documented 
*/
void create_ct(ct_traj *trj,vectray *vr)
{
   trj->state = BTTRAJ_OFF;
   trj->pwl = new_pwl();

   init_pwl_from_vectray(trj->pwl,vr);

}
/**
\internal chk'd TH 051103
Not thought to be useful so not documented 
*/
int readfile_ct(ct_traj* ct,char* filename)
{
   vectray *vr;

   read_csv_file_vr(filename,&vr);
   create_ct(ct,vr);
   destroy_vr(&vr);
   return 0;
}
/**
\internal chk'd TH 051103
Not thought to be useful so not documented 
*/
vect_n* init_ct(ct_traj *trj)
{
   return dsinit_pwl(trj->pwl,0.0);
}
/**
\internal chk'd TH 051103
Not thought to be useful so not documented 
*/
vect_n* eval_ct(ct_traj *trj, btreal dt)
{
   return ds_pwl(trj->pwl,dt);
}
/**
\internal chk'd TH 051103
Not thought to be useful so not documented 
*/
int done_ct(ct_traj *trj)
{
   if (trj->pwl->proxy_s == arclength_pwl(trj->pwl))
      return 1;
   else
      return 0;
}
/**
\internal chk'd TH 051103
Not thought to be useful so not documented 
*/
int bttrajectory_interface_getstate_ct(struct bttrajectory_interface_struct *btt)
{
   ct_traj* ct;
   ct = (ct_traj*)btt->dat;

   if (done_ct(ct))
      return BTTRAJ_DONE;
   else
      return BTTRAJ_RUN;
}
/**
\internal chk'd TH 051103
Not thought to be useful so not documented 
*/
vect_n* bttrajectory_interface_reset_ct(struct bttrajectory_interface_struct *btt)
{
   ct_traj* ct;
   //syslog(LOG_ERR,"Starting reset ct");
   ct = (ct_traj*)btt->dat;

   return dsinit_pwl(ct->pwl,0.0);
}
/**
\internal chk'd TH 051103
Not thought to be useful so not documented 
*/
vect_n* bttrajectory_interface_eval_ct(struct bttrajectory_interface_struct *btt)
{
   ct_traj* ct;
   vect_n* ret;
   ct = (ct_traj*)btt->dat;
   ret = ds_pwl(ct->pwl,*(btt->dt));
   if (done_ct(ct))
      btt->state = BTTRAJ_DONE;
   return ret;
}
/**
\internal chk'd TH 051103
Not thought to be useful so not documented 
*/
void bttrajectory_interface_mapf_ct(btstatecontrol *sc,ct_traj *trj)
{
   maptrajectory_bts(sc,(void*) trj,
                     bttrajectory_interface_reset_ct,
                     bttrajectory_interface_eval_ct,
                     bttrajectory_interface_getstate_ct);

}

/******************************************************************************/

enum VT_SEG {
   VTS_IN_ACC = 0, // We are in an acceleration segment
   VTS_IN_VEL  // We are in an velocity segment
};
/** Assumes trj->vr has been loaded with a valid vectray
 
 Forces state to BTTRAJ_RUN
 
 \internal chk'd TH 051103
*/
double start_via_trj(via_trj *trj,int col)
{
   double Dt,Dq,Dv,Dt_next,Dq_next,Dv_next,ret;
   int end;

   trj->idx = 0;
   trj->n = numrows_vr(trj->vr);
   trj->col = col+1;
   ret = getval_vn(idx_vr(trj->vr,0),trj->col); //force starting point to be our starting point

   trj->last_cmd = ret;
   trj->segment = VTS_IN_ACC;//acc first


   Dt_next = getval_vn(idx_vr(trj->vr,trj->idx+2),0) - getval_vn(idx_vr(trj->vr,trj->idx+1),0);
   Dq_next = getval_vn(idx_vr(trj->vr,trj->idx+2),trj->col) - getval_vn(idx_vr(trj->vr,trj->idx+1),trj->col);
   trj->v_prev = 0;
   if (trj->n <= 1) {
      trj->state = BTTRAJ_DONE;
      return ret;
   } else if (trj->n == 2) {
      end = 3;
      trj->v_next = 0.0;
      trj->t3 = getval_vn(idx_vr(trj->vr,trj->idx+1),0);
   } else {
      end = 0; if(Dt_next > -EPSILON && Dt_next < EPSILON) syslog(LOG_ERR, "btcontrol:start_via_trj:560 Dt_next");
      trj->v_next = Dq_next/Dt_next;
      trj->t3 = getval_vn(idx_vr(trj->vr,trj->idx+2),0);
   }
   trj->qp = getval_vn(idx_vr(trj->vr,trj->idx),trj->col);
   trj->qn = getval_vn(idx_vr(trj->vr,trj->idx+1),trj->col);
   trj->tp = getval_vn(idx_vr(trj->vr,trj->idx),0);
   trj->tn = getval_vn(idx_vr(trj->vr,trj->idx+1),0);

   CalcSegment(&(trj->seg),trj->qp,trj->qn,
               trj->tp,trj->tn,trj->t3 ,trj->v_prev,trj->v_next,trj->trj_acc, end);

   trj->acc = trj->seg.acc1;
   trj->dt_acc = trj->seg.dt_acc1;
   trj->v_prev = trj->seg.vel;
   trj->t_acc = trj->dt_acc;

   trj->t=getval_vn(idx_vr(trj->vr,trj->idx),0);
   trj->t0=0.0;
   trj->q0 = getval_vn(idx_vr(trj->vr,trj->idx),trj->col);
   trj->v0 = 0.0;

   trj->state = BTTRAJ_RUN;
   return ret;
}

/** Evaluate a via_trj object.
It must have previously been preped with start_via_trj()
\internal chk'd TH 051103
*/
double eval_via_trj(via_trj *trj,double dt)
{
   double Dt,Dq,Dv,Dt_next,Dq_next,Dv_next,acc_next,t_acc_next,end,et,cmd;
   double tn,tp,qn,qp;
   if (trj->state == BTTRAJ_RUN) {
      trj->t += dt;  //increment time

      if ((trj->segment == VTS_IN_ACC) && (trj->t > trj->t_acc)) { //done with acc, set up vel
         if (trj->idx >= trj->n-1 && trj->idx != 0) {
            trj->state = BTTRAJ_STOPPED;
         } else {

            trj->vel = trj->seg.vel;
            trj->dt_vel = trj->seg.dt_vel;
            trj->t0 = trj->t_acc;
            trj->t_vel = trj->dt_vel + trj->t0;
            trj->q0 = trj->q0 + trj->v0*trj->dt_acc + 0.5*trj->dt_acc*trj->dt_acc*trj->acc;
            //trj->q0 = interpolate_bt(trj->tp,trj->qp,trj->tn,trj->qn,trj->t0);
            //trj->q0 = trj->qp;
            trj->v0 = trj->vel;
            trj->acc = 0.0;
            trj->segment = VTS_IN_VEL;
         }

      }
      if((trj->segment == VTS_IN_VEL) && (trj->t > trj->t_vel)) { //setup acc segment
         //trj->q0 = interpolate_bt(trj->tp,trj->qp,trj->tn,trj->qn,trj->t_vel);
         trj->idx++;
         if (trj->idx >= trj->n-1) //Setup final deceleration
         {
            trj->acc = trj->seg.acc2;
            trj->dt_acc = trj->seg.dt_acc2;
            trj->t0 = trj->t_vel;
            trj->t_acc = trj->dt_acc + trj->t0;
            //trj->q0 = trj->q0 + trj->vel*trj->dt_vel;
            trj->v0 = trj->vel;
            trj->segment = VTS_IN_ACC;//acc first
         } else {
            trj->qp = getval_vn(idx_vr(trj->vr,trj->idx),trj->col);
            trj->qn = getval_vn(idx_vr(trj->vr,trj->idx+1),trj->col);
            trj->tp = getval_vn(idx_vr(trj->vr,trj->idx),0);
            trj->tn = getval_vn(idx_vr(trj->vr,trj->idx+1),0);
            trj->t3 = getval_vn(idx_vr(trj->vr,trj->idx+2),0);
            if (trj->idx >= trj->n-2) {
               end = 2;
               trj->v_next = 0.0;
            } else {
               end = 1;
               Dt_next = getval_vn(idx_vr(trj->vr,trj->idx+2),0) - getval_vn(idx_vr(trj->vr,trj->idx+1),0);
               Dq_next = getval_vn(idx_vr(trj->vr,trj->idx+2),trj->col) - getval_vn(idx_vr(trj->vr,trj->idx+1),trj->col);
               if(Dt_next > -EPSILON && Dt_next < EPSILON) syslog(LOG_ERR, "btcontrol:eval_via_trj:640 Dt_next");
               trj->v_next = Dq_next/Dt_next;
            }
            //syslog(LOG_ERR, "CalcSegment: Col %d",trj->col);
            CalcSegment(&(trj->seg),trj->qp, trj->qn,
                        trj->tp,trj->tn,trj->t3,trj->v_prev,trj->v_next,trj->trj_acc, end);

            trj->segment = 0;//acc first
            trj->acc = trj->seg.acc1;
            trj->dt_acc = trj->seg.dt_acc1;
            trj->t0 = trj->t_vel;
            trj->t_acc = trj->dt_acc + trj->t0;
            trj->q0 = trj->q0 + trj->vel*trj->dt_vel;
            //trj->q0 = interpolate_bt(trj->tp,trj->qp,trj->tn,trj->qn,trj->t0);
            //trj->q0 = trj->qp;
            trj->v0 = trj->vel;
            trj->v_prev = trj->seg.vel;
         }

      }

      if (trj->segment == 0) {//in acceleration segment
         et = trj->t - trj->t0;
         cmd = trj->q0 + trj->v0*et + 0.5*et*et*trj->acc;
      } else {
         et = trj->t - trj->t0; //elapsed time
         //cmd = trj->q0 + trj->vel*et;
         cmd = trj->q0 + trj->v0*et + 0.5*et*et*trj->acc;
      }
      trj->last_et = et;
      trj->last_cmd = cmd;
   } else {
      cmd = trj->last_cmd;
   }

   return cmd;

}
/**
 
  see Notebook TH#6 pp146,147
  
\param seg_acc - Acceleration with which to blend linear segments.
\param q1 
\param q2 
\param t1 
\param t2 
 
\internal chk'd TH 051103 (incomplete)
*/
void CalcSegment(Seg_int *seg,double q1, double q2, double t1, double t2, double t3, double v_prev, double v_next, double seg_acc, int end)
{
   double dt,dq,dtn;
   double vel,acc1,acc2,dt_vel,dt_acc1,dt_acc2;
   double min_acc,use_acc,q_acc1,q_vel,q_acc2;
   double max_acc,ac4,sqrtb;

   if (t2 <= t1) {
      syslog(LOG_ERR, "CalcSeg: Error: Times are out of order!!!! Skipping this segment");
      seg->vel = 0.0;
      seg->acc1 = 0.0;
      seg->acc2 = 0.0;
      seg->dt_vel = 0.0;
      seg->dt_acc1 = 0.0;
      seg->dt_acc2 = 0.0;
      return;
   }

   dt = t2 - t1;
   dtn = t3 - t2;
   dq = q2 - q1;
#if 0

   syslog(LOG_ERR, "CalcSeg: in: q1: %f q2: %f  t1: %f t2: %f",q1,q2,t1,t2);
   syslog(LOG_ERR, "CalcSeg: in: vprev: %f vnext: %f  acc: %f end: %d",v_prev,v_next,seg_acc,end);
#endif

   use_acc = fabs(seg_acc); //Force to positive


   if (end == 0) {
      //Starting segment (Accelerate to the next point)
      if(dt > -EPSILON && dt < EPSILON) syslog(LOG_ERR, "btcontrol:CalcSegment:722 dt");
      min_acc = 8*fabs(dq)/(3*dt*dt);

      if (use_acc < min_acc) { //accelleration must get us to vel_mode at halfway point
         use_acc = min_acc;
         syslog(LOG_ERR, "CalcSegment: Boosted acceleration to %f to match velocity change",use_acc);
      }
      if (use_acc != 0.0){
         if(use_acc > -EPSILON && use_acc < EPSILON) syslog(LOG_ERR, "btcontrol:CalcSegment:730 use_acc");
         dt_acc1 = dt - sqrt(dt*dt - 2*fabs(dq)/use_acc);
      }else
         dt_acc1 = 0.0;
      acc1 = Sgn(dq)*use_acc;
      q_acc1 = q1 + 0.5*acc1*dt_acc1*dt_acc1;
      //vel = (q2 - q_acc1)/(dt - dt_acc1);
      vel = acc1*dt_acc1;
      acc2 = Sgn(v_next - vel)*use_acc;
      /* Force acceleration high enough to make the corner */
      if (fabs(v_next - vel) > dtn*fabs(acc2)/2) {
         if(dtn > -EPSILON && dtn < EPSILON) syslog(LOG_ERR, "btcontrol:CalcSegment:741 dtn");
         acc2 = (v_next - vel)*2/dtn;
         syslog(LOG_ERR, "CalcSegment: Boosted next segment acceleration to %f to fit next side",acc1);
      } else if (fabs(v_next - vel) > dt*fabs(acc2)/2) {
         if(dt > -EPSILON && dt < EPSILON) syslog(LOG_ERR, "btcontrol:CalcSegment:745 dt");
         acc2 = (v_next - vel)*2/dt;
         syslog(LOG_ERR, "CalcSegment: Boosted mid segment acceleration to %f to fit this side",acc1);
      }
      if (acc2 != 0.0) {
         if(acc2 > -EPSILON && acc2 < EPSILON) syslog(LOG_ERR, "btcontrol:CalcSegment:750 acc2");
         dt_acc2 = (v_next - vel)/acc2;
      } else {
         dt_acc2 = 0.0;
      }
      dt_vel = dt - dt_acc1 - dt_acc2/2.0;
      if (dt_vel < 0.0) {
         dt_vel = 0.0;

         syslog(LOG_ERR, "CalcSegment: Init Acc: Not enough acceleration!");
         syslog(LOG_ERR, "CalcSegment: Init Acc: dt:%f dt_acc1:%f 0.5*dt_acc2:%f",dt,dt_acc1,dt_acc2/2.0);
      }
   } else if (end == 1) //Middle segment
   {
      if(dt > -EPSILON && dt < EPSILON) syslog(LOG_ERR, "btcontrol:CalcSegment:764 dt");
      vel = dq/dt;
      //acc1 = Sgn(vel - v_prev)*use_acc;
      /*
      if (fabs(vel - v_prev) > dt*fabs(acc1)/2){
        acc1 = (vel - v_prev)*2/dt;
        syslog(LOG_ERR, "CalcSegment: Boosted mid segment acceleration to %f to match velocity change",acc1);
   }
      if (acc1 != 0.0){
        dt_acc1 = (vel - v_prev)/acc1;
   }
      else {
        dt_acc1 = 0.0;
   }
      q_acc1 = q1 + 0.5*acc1*dt_acc1*dt_acc1;
      */
      acc1 = seg->acc2;
      dt_acc1 = seg->dt_acc2;

      acc2 = Sgn(v_next - vel)*use_acc;
      /* Force acceleration high enough to make the corner */
      if (fabs(v_next - vel) > dtn*fabs(acc2)/2)
      {
         if(dtn > -EPSILON && dtn < EPSILON) syslog(LOG_ERR, "btcontrol:CalcSegment:787 dtn");
         acc2 = (v_next - vel)*2/dtn;
         syslog(LOG_ERR, "CalcSegment: Boosted next segment acceleration to %f to fit next side",acc1);
      } else if (fabs(v_next - vel) > dt*fabs(acc2)/2)
      {
         if(dt > -EPSILON && dt < EPSILON) syslog(LOG_ERR, "btcontrol:CalcSegment:792 dt");
         acc2 = (v_next - vel)*2/dt;
         syslog(LOG_ERR, "CalcSegment: Boosted mid segment acceleration to %f to fit this side",acc1);
      }
      if (acc2 != 0.0)
      {
         if(acc2 > -EPSILON && acc2 < EPSILON) syslog(LOG_ERR, "btcontrol:CalcSegment:798 acc2");
         dt_acc2 = (v_next - vel)/acc2;
      } else
      {
         dt_acc2 = 0.0;
      }

      dt_vel = dt - dt_acc1/2 - dt_acc2/2;
      if (dt_vel < 0.0)
      {
         dt_vel = 0.0;

         syslog(LOG_ERR, "CalcSegment: Mid segment: Not enough acceleration!");
         syslog(LOG_ERR, "CalcSegment: Init Acc: dt:%f 0.5*dt_acc1:%f 0.5*dt_acc2:%f",dt,dt_acc1/2.0,dt_acc2/2.0);
      }
   }
   else if (end == 2) {
      //Ending segment (decelerate)
      if(dt > -EPSILON && dt < EPSILON) syslog(LOG_ERR, "btcontrol:CalcSegment:816 dt");
      min_acc = 8.0*fabs(dq)/(3.0*dt*dt);

      if (use_acc < min_acc) { //accelleration must get us to vel_mode at halfway point
         use_acc = min_acc;
         syslog(LOG_ERR, "CalcSegment: Boosted acc:%f to match end velocity change %f %f",acc1,dq,dt);
         //syslog(LOG_ERR, "CalcSegment:  %f %f %f %f",q1,q2,t1,t2);
      }
      if (use_acc != 0.0) {
         if(use_acc > -EPSILON && use_acc < EPSILON) syslog(LOG_ERR, "btcontrol:CalcSegment:825 use_acc");
         dt_acc2 = dt - sqrt(dt*dt - 2*fabs(dq)/use_acc);
      } else {
         dt_acc1 = 0.0;
      }
      acc2 = -1.0*Sgn(dq)*use_acc;
      //vel = (dq - 0.5*acc2*dt_acc2*dt_acc2)/(dt - dt_acc2);
      vel = -1.0*acc2*dt_acc2;
      acc1 = Sgn((vel - v_prev))*use_acc;
      if (fabs(vel - v_prev) > dt*fabs(acc1)/2) {
         if(dt > -EPSILON && dt < EPSILON) syslog(LOG_ERR, "btcontrol:CalcSegment:835 dt");
         acc1 = (vel - v_prev)*2/dt;
         syslog(LOG_ERR, "CalcSegment: Boosted mid segment acceleration to %f to match velocity change",acc1);
      }
      if (acc1 != 0.0) {
         if(acc1 > -EPSILON && acc1 < EPSILON) syslog(LOG_ERR, "btcontrol:CalcSegment:840 acc1");
         dt_acc1 = (vel - v_prev)/acc1;
      } else {
         dt_acc1 = 0.0;
      }

      dt_vel = dt - dt_acc2 - dt_acc1/2;
      if (dt_vel < 0.0) {
         dt_vel = 0.0;

         syslog(LOG_ERR, "CalcSegment: Final Acc: Not enough acceleration!");
         syslog(LOG_ERR, "CalcSegment: Final Acc: dt:%f dt_acc1:%f 0.5*dt_acc2:%f",dt,dt_acc1,dt_acc2/2.0);

      }
   } else {
      //Short Segment (accelerate and decellerate with bang bang
      if(dt > -EPSILON && dt < EPSILON) syslog(LOG_ERR, "btcontrol:CalcSegment:856 dt");
      max_acc = fabs(dq)/(0.25*dt*dt);
      if (use_acc > max_acc) {
         //accelleration must get us to vel_mode at halfway point
         use_acc = max_acc;
         syslog(LOG_ERR, "CalcSegment: Boosted acc:%f to match end velocity change %f %f",acc1,dq,dt);
         //syslog(LOG_ERR, "CalcSegment:  %f %f %f %f",q1,q2,t1,t2);
      }
      acc2 = -1.0*Sgn(dq)*use_acc;
      if (use_acc != 0.0) {
         ac4 = acc2*dq*4;
         if (acc2*acc2*dt*dt < ac4)
            sqrtb = 0.0;
         else
            sqrtb = sqrt(acc2*acc2*dt*dt - ac4);

         if(acc2 > -EPSILON && acc2 < EPSILON) syslog(LOG_ERR, "btcontrol:CalcSegment:872 acc2");
         dt_acc1 = acc2*dt/(2.0*acc2)+sqrtb;
      } else {
         dt_acc1 = 0.0;
      }

      //vel = (dq - 0.5*acc2*dt_acc2*dt_acc2)/(dt - dt_acc2);
      vel = acc2*dt_acc1;
      acc1 = -1.0 * acc2;
      dt_acc2 = dt_acc1;
      dt_vel = dt - dt_acc2 - dt_acc1;
      if (dt_vel < 0.0) {
         dt_vel = 0.0;
         syslog(LOG_ERR, "CalcSegment: Final Acc: Not enough acceleration!");
         syslog(LOG_ERR, "CalcSegment: Final Acc: dt:%f dt_acc1:%f 0.5*dt_acc2:%f",dt,dt_acc1,dt_acc2/2.0);

      }
   }
   seg->vel = vel;
   seg->acc1 = acc1;
   seg->acc2 = acc2;
   seg->dt_vel = dt_vel;
   seg->dt_acc1 = dt_acc1;
   seg->dt_acc2 = dt_acc2;

#if 0

   syslog(LOG_ERR, "CalcSeg: Time: dt_acc1: %f dt_vel: %f dt_acc2: %f",dt_acc1,dt_vel,dt_acc2);
   syslog(LOG_ERR, "CalcSeg: val: acc1: %f vel: %f  acc2: %f",acc1,vel,acc2);
   syslog(LOG_ERR, "CalcSeg: ");

#endif
}

/** Data access function for trj->trj_acc
\internal chk'd TH 051103
*/
void SetAcc_vt(via_trj *trj,double acc)
{
   trj->trj_acc = acc;
}

/**
\internal chk'd TH 051103
Internal function to allocate memory for a vta object.
*/
via_trj_array* malloc_vta(int num_columns)
{
   void *vmem;
   via_trj_array* vt;

   vmem = btmalloc(sizeof(via_trj_array) + (size_t)num_columns * sizeof(via_trj));

   vt = (via_trj_array*)vmem;
   vt->trj = vmem + sizeof(via_trj_array);
   vt->elements = num_columns;
   return vt;
}
/** Create a new vta from a csv file
 
The trajectory via points will be read from a comma seperated values file. The via 
points will have a vector size of the number of columns in the file minus one. 
The first column is used for the time values. 
 
Example file (with 3 element vectors as via points)
\code
0.0, 1.2, 3.2, 5.3
0.12, 2.2, 3.4, 5.2
1.0, 1.2, 3.2, 5.3
\endcode
\param extrapoints The number of via points we will allow for additional editing.
\param filename Name of the file you wish to read.
 
\return 
- Returns a pointer to a via_trj_array object.
- Return NULL if there was a problem.
\internal chk'd TH 051103 
*/
via_trj_array* read_file_vta(char* filename,int extrapoints)
{
   via_trj_array* vt;
   vectray *vr;
   int cnt;
   if (read_csv_file_vr(filename,&vr) != 0)
      return NULL;
   //vr = (vectray *)realloc(vr, maxrows_vr(vr)+extrapoints);
   vr = resize_vr(&vr,maxrows_vr(vr)+extrapoints);

   vt = malloc_vta(numelements_vr(vr)-1);
   vt->vr = vr;
   vt->acc = 1.0;
   vt->pavn = NULL;

   /*VT*/
   for(cnt = 0;cnt < vt->elements;cnt++) {
      SetAcc_vt(&(vt->trj[cnt]),1.0);
      vt->trj[cnt].vr = vr;
   }
   /*VT*/

   return vt;
}

/** Save the vta object to a csv file
\internal chk'd TH 051103
*/
void write_file_vta(via_trj_array* vt,char *filename)
{
   if (vt != NULL)
      write_csv_file_vr(filename,vt->vr);
}

/** Simulate running a trajectory and dump results to file.
 
Us this for debugging or previewing your trajectories.
\internal chk'd TH 051103
*/
vect_n* sim_vta(via_trj_array* vt,double dt,double duration,char*filename)
{
   FILE *out;
   char buff[250];
   double t = dt;
   local_vn(qref,7);
   init_vn(qref,7);
   t = getval_vn(idx_vr(vt->vr,0),0);
   out = fopen(filename,"w");
   reset_vta(vt,dt,qref);
   fprintf(out,"%f, %s\n",t,sprint_csv_vn(buff,qref));

   while (t < duration) {
      t += dt;
      eval_vta(vt,dt,qref);
      fprintf(out,"%f, %s\n",t,sprint_csv_vn(buff,qref));
   }

   fclose(out);
}

/** set acceleration on corners of via trajectory

\internal chk'd TH 051103
*/
void set_acc_vta(via_trj_array* vt,btreal acc)
{
   int cnt;
   if (vt != NULL) {
      vt->acc = acc;
      /*VT*/
      for(cnt = 0;cnt < vt->elements;cnt++) {
         SetAcc_vt(&(vt->trj[cnt]),acc);
      }
      /*VT*/
   }
}

/** set velocity used when adding points
\internal chk'd TH 051103
*/
void set_vel_vta(via_trj_array* vt,btreal vel)
{
   if (vt != NULL)
      vt->vel = vel;
}

/** Allocate a vta object and vectray objects.
 
vta objects must be created using new_vta(). They must be destroyed using
destroy_vta(). A vta object allocated by simply declaring
\code
via_trj_array myobject;
\endcode
will behave in an undefined (but generally bad) manner.
\internal chk'd TH 051103
*/
via_trj_array* new_vta(int num_columns,int max_rows)
{
   via_trj_array* vt;
   vectray *vr;
   int cnt;
   vt = malloc_vta(num_columns);
   vr = new_vr(num_columns+1,max_rows);
   vt->vr = vr;
   vt->acc = 1.0;
   vt->vel = 0.5;
   vt->pavn = NULL;
   /*VT*/
   for(cnt = 0;cnt < vt->elements;cnt++) {
      SetAcc_vt(&(vt->trj[cnt]),1.0);
      vt->trj[cnt].vr = vr;
   }
   /*VT*/
   return vt;
}

/** Free memory allocated during new_vr()
 
Note that we are expecting the address of your pointer. As a matter of convention,
your pointer will be be set to NULL to indicate that it is no longer pointing to
a valid via_trj_array object.
\internal chk'd TH 051103
*/
void destroy_vta(via_trj_array** vt)
{
   if (*vt != NULL) {
      destroy_vr(&(*vt)->vr);
      btfree((void**)vt);
   }
}

/**Increment the edit point by one.
 
If the edit point reaches the end of the list,
it stays there.
 
\internal chk'd TH 051103
*/
void next_point_vta(via_trj_array* vt)
{
   if (vt != NULL)
      next_vr(vt->vr);
}

/**Decrement the edit point by one.
 
If the edit point reaches the beginning of the list,
it stays there.
 
\internal chk'd TH 051103
*/
void prev_point_vta(via_trj_array* vt)
{
   if (vt != NULL)
      prev_vr(vt->vr);
}

/**Set the edit point to the begining of the list.
\internal chk'd TH 051103
*/
void first_point_vta(via_trj_array* vt)
{
   if (vt != NULL)
      start_vr(vt->vr);
}

/**Set the edit point to the end of the list.
\internal chk'd TH 051103
*/
void last_point_vta(via_trj_array* vt)
{
   if (vt != NULL)
      end_vr(vt->vr);
}

/** Insert a location into the teach & play list.
at the edit point. To add a location to the end of the list, you
may move the edit point to the end. (last_point_vta())
\internal chk'd TH 051103
*/
int ins_point_vta(via_trj_array* vt, vect_n *pt)
{
   int i;
   vectray *vr;
   btreal t,d;
   local_vn(tmp,10);
   local_vn(last,10);

   init_vn(tmp,len_vn(pt)+1);
   init_vn(last,len_vn(pt));

   if (vt == NULL)
      return -1;
   else {
      vr = vt->vr;
      setrange_vn(tmp,pt,1,0,len_vn(pt));
      setval_vn(tmp,0,0.0);
      i = edit_point_vr(vr);
      if (i > 0) {
         setrange_vn(last,idx_vr(vr,i-1),0,1,len_vn(pt));
         d = norm_vn(sub_vn(pt,last));
         t = getval_vn(idx_vr(vr,i-1),0);
         if (vt->vel > 0.0)
            setval_vn(tmp,0,t + d/vt->vel);
         else
            setval_vn(tmp,0,t + d/0.1);
      }
      return insert_vr(vt->vr,tmp);
   }
}

/** Delete the location at the edit point from the list.
\internal chk'd TH 051103
*/
int del_point_vta(via_trj_array* vt)
{
   if (vt == NULL)
      return -1;
   else
      return delete_vr(vt->vr);
}

/** Return the index of the present edit point.
\internal chk'd TH 051103
*/
int get_current_idx_vta(via_trj_array* vt)
{
   if (vt == NULL)
      return -1;
   else
      return edit_point_vr(vt->vr);
}

/** Set the index of the present edit point.
\internal chk'd TH 051103
*/
int set_current_idx_vta(via_trj_array* vt,int idx)
{
   if (vt == NULL)
      return -1;
   else
      return edit_at_vr(vt->vr,idx);
}

/** Copy the point at the present idx to dest.
\internal chk'd TH 051103
*/
void get_current_point_vta(via_trj_array* vt, vect_n *dest)
{
   vectray *vr;
   if (vt == NULL)
      return;
   vr = vt->vr;
   set_vn(dest,edit_vr(vr));
}

/** Get a pointer to the vectray object being used by this vta object.
\internal chk'd TH 051103
*/
vectray* get_vr_vta(via_trj_array* vt)
{
   if (vt == NULL)
      return NULL;
   else
      return vt->vr;
}

/** Set the time values in a trajectory.
 
Adjust all the time points in a via point trajectory array
based on input velocity. Also sets the corner acceleration.
 
\internal chk'd TH 051103
\todo Need to include acceleration in the calculation.
*/
int dist_adjust_vta(via_trj_array* vt,double vel)
{
   btreal arclen = 0.0,thislen;
   vectray *vr;
   int cnt;
   vect_n* lval,*rval;

   if (vt == NULL)
      return -1;

   vt->vel = vel;

   vr = vt->vr;
   setval_vn(idx_vr(vr,0),0,0.0);

   for(cnt = 1;cnt < numrows_vr(vr);cnt++) {
      lval = lval_vr(vr,cnt-1);
      rval = rval_vr(vr,cnt);
      arclen += norm_vn(sub_vn(subset_vn(lval,1,lval->n),subset_vn(rval,1,rval->n)));
      reset_vn(lval);
      reset_vn(rval); /**\internal \todo the reset_vn() function is a bit of a hack, is there a better way?*/
      setval_vn(idx_vr(vr,cnt),0,arclen/vel);
   }
   return 0;
}

/** Scale the time values in a trajectory.
 
Adjust all the time points in a via point trajectory array
by a scale factor. 
 
\internal chk'd TH 051103 (untested)
*/
int time_scale_vta(via_trj_array* vt,double s)
{
   btreal arclen = 0.0,thislen;
   vectray *vr;
   int cnt;

   if (vt == NULL)
      return -1;

   vr = vt->vr;
   setval_vn(idx_vr(vr,0),0,0.0);

   for(cnt = 1;cnt < numrows_vr(vr);cnt++) {
      setval_vn(lval_vr(vr,cnt),0,s * getval_vn(rval_vr(vr,cnt),0));
   }
   return 0;
}

vect_n* reset_vta(via_trj_array* vt,double dt,vect_n* qref)
{
   double ret;
   int cnt;
   if (vt == NULL)
      return qref;

   destroy_pavn(&vt->pavn);
   vt->pavn = vr2pararray(vt->vr,vt->acc);

   /*
   for (cnt = 0;cnt<vt->elements;cnt++)
{
     setval_vn(qref,cnt,start_via_trj(&(vt->trj[cnt]),cnt));
}*/
   set_vn(qref,reset_pavn(vt->pavn));
   return qref;

}

vect_n* eval_vta(via_trj_array* vt,double dt,vect_n* qref)
{
   double ret;
   int cnt;

   if (vt == NULL)
      return qref;
   /*
   for (cnt = 0;cnt<vt->elements;cnt++)
{
     setval_vn(qref,cnt,eval_via_trj(&(vt->trj[cnt]),dt));
}*/

   set_vn(qref,eval_pavn(vt->pavn,dt));

   return qref;
}

/** Implements the getstate interface for bttrajectory_interface_struct.
See bttrajectory_interface_struct
\internal chk'd TH 051103
*/
int bttrajectory_interface_getstate_vt(struct bttrajectory_interface_struct *btt)
{
   int cnt;
   int ret;
   via_trj_array* vt;
   vt = (via_trj_array*)btt->dat;
#ifdef BT_NULL_PTR_GUARD
   if(!btptr_ok(vt,"bttrajectory_interface_getstate_vt"))
      return  BTTRAJ_OFF;
#endif
   /*ret = BTTRAJ_DONE;
   for (cnt = 0;cnt<vt->elements;cnt++)
{
     if (vt->trj[cnt].state == BTTRAJ_RUN)
       ret = BTTRAJ_RUN;
}*/
   ret = getstate_pavn(vt->pavn);
   return ret;
}

/** Implements the reset interface for bttrajectory_interface_struct.
See bttrajectory_interface_struct
\internal chk'd TH 051103
*/
vect_n* bttrajectory_interface_reset_vt(struct bttrajectory_interface_struct *btt)
{
   double ret;
   int cnt;
   via_trj_array* vt;
   vt = (via_trj_array*)btt->dat;
#ifdef BT_NULL_PTR_GUARD
   if(!btptr_ok(vt,"bttrajectory_interface_reset_vt"))
      return btt->qref;
#endif

   if (numrows_vr(vt->vr) <= 0)
      return NULL;

   /* Get rid of the old pavn */
   destroy_pavn(&vt->pavn);
   vt->pavn = vr2pararray(vt->vr, vt->acc);
   set_vn(btt->qref, reset_pavn(vt->pavn));
   /*
   for (cnt = 0;cnt<vt->elements;cnt++)
{
     setval_vn(btt->qref,cnt,start_via_trj(&(vt->trj[cnt]),cnt));
}*/
   return btt->qref;
}

/** Implements the eval interface for bttrajectory_interface_struct.
See bttrajectory_interface_struct
\return NULL if trajectory has no points.
\internal chk'd TH 051103
*/
vect_n* bttrajectory_interface_eval_vt(struct bttrajectory_interface_struct *btt)
{
   double ret;
   int cnt;
   via_trj_array* vt;
   vt = (via_trj_array*)btt->dat;
#ifdef BT_NULL_PTR_GUARD
   if(!btptr_ok(vt,"bttrajectory_interface_eval_vt"))
      return btt->qref;
#endif
   /*
     for (cnt = 0;cnt<vt->elements;cnt++)
     {
       setval_vn(btt->qref,cnt,eval_via_trj(&(vt->trj[cnt]),*(btt->dt)));
     }*/

   set_vn(btt->qref,eval_pavn(vt->pavn,*(btt->dt)));
   return btt->qref;
}

/** Registers the necessary data and function pointers with the

btstatecontrol object 
\internal chk'd TH 051103
*/
void register_vta(btstatecontrol *sc,via_trj_array *vt)
{
#ifdef BT_NULL_PTR_GUARD
   if (!btptr_ok(sc,"register_vta"))
      exit(1);
   if (!btptr_ok(vt,"register_vta"))
      syslog(LOG_ERR,"register_vta: Setting btt.dat to NULL");
#endif

   maptrajectory_bts(sc,(void*) vt,
                     bttrajectory_interface_reset_vt,
                     bttrajectory_interface_eval_vt,
                     bttrajectory_interface_getstate_vt);
}
#undef BT_DUMMY_PROOF
