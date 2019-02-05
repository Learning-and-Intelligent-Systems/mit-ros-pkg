/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... btstatecontrol.c
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

#ifdef S_SPLINT_S
#include <err.h>
#endif
#include <math.h>
#include <pthread.h>
#include <errno.h>
#include <syslog.h>

#include "btcontrol.h"
#include "btos.h"
#include "btmath.h"
#include "btstatecontrol.h"
//#include "btjointcontrol.h"

/************************** universal trajectory functions *****************/
/** Set trajectory max velocity and acceleration.
 
see #bttraptrj
\internal chk'd TH 051102
*/
void setprofile_traptrj(bttraptrj *traj, btreal vel, btreal acc)
{
   traj->vel = fabs(vel);
   traj->acc = fabs(acc);
}

/** Calculates all the variables necessary to set up a trapezoidal trajectory.
 
Previously you must have set the velocity and acceleration using setprofile_traptrj().
 
\param dist The length of the trajectory.
\internal chk'd TH 051102
*/
void start_traptrj(bttraptrj *traj, btreal dist)
{
   //assumes that velocity and acceleration have been set to reasonable values
   double ax; //acceleration x

   traj->cmd = 0.0;

   traj->t1 = (traj->vel / traj->acc); //Calculate the "ramp-up" time

   //Get the point at which acceleration stops
   ax = 0.5*traj->acc*traj->t1*traj->t1; //x = 1/2 a t^2

   traj->end = dist;
   if (ax > dist/2){ //If the acceleration completion point is beyond the halfway point
      ax = dist/2; //Stop accelerating at the halfway point
      traj->t1 = sqrt(2*ax/traj->acc); // Find the new "ramp-up" time
   }
   traj->x1 = ax;   //Set the top left point of the trapezoid
   traj->x2 = dist - ax; //Set the top right point of the trapezoid
   traj->t2 = (dist-(2*ax)) / traj->vel + traj->t1; //Find the time to start the "ramp-down"
   traj->t = 0;
   traj->state = BTTRAJ_RUN;
   if (dist <= 0.0){
      //syslog(LOG_ERR, "btstatecontrol.c:start_traptrj() dist <=0.0 (%f)", dist);
      //syslog(LOG_ERR, "start_traptrj: dist=%f, vel=%f, acc=%f, ax=%f, x1=%f, x2=%f, t1=%f, t2=%f, t=%f",
      //dist, traj->vel, traj->acc, ax, traj->x1, traj->x2, traj->t1, traj->t2, traj->t);
      traj->state = BTTRAJ_STOPPED;
   }
}

/** Calculate next position on the trajectory
 
evaluate_trajectory() generates points on a trapezoidal velocity trajectory. The points accerate with
constant acceleration specified by \e acc up to a maximum velocity specified by \e vel. The max velocity
is maintained until approximately the time it will take to decellerate at \e acc to a velocity of zero.
 
\param *traj pointer to the trajectory structure
\param dt the time step to take (in the same units as vel and acc)
 
\return The value returned is the next point on the trajectory after a time step of dt.
 \internal chk'd TH 051102
 
*/
btreal evaluate_traptrj(bttraptrj *traj, btreal dt)
{
   btreal remaining_time,newtime,result;

   if (traj->state == BTTRAJ_RUN) {
      traj->t += dt;
      if (traj->cmd < traj->x1){ //If we are in "accel" stage
         traj->cmd = 0.5 * traj->acc * traj->t * traj->t;  //x = 1/2 a t^2
      }else if (traj->cmd < traj->x2){ //If we are in "cruise" stage
         traj->cmd = traj->x1 + traj->vel * (traj->t - traj->t1); //x = x + v t
      }else {
         //We are in "decel" stage
         /* time to hit zero = sqrt( (target position - current position)*2/a)
          * new position = 1/2 acc * (time to hit zero - dt)^2 
          */
         if(traj->end - traj->cmd < 0.0){
            syslog(LOG_ERR, "evaluate_traptrj: About to take sqrt() of %8.4f-%8.4f (bad)", traj->end, traj->cmd);
            syslog(LOG_ERR, "evaluate_traptrj: traj->t=%f, dt=%f, traj->x1=%f, traj->x2=%f", traj->t, dt, traj->x1, traj->x2);
         }
         if(traj->acc == 0.0)
            syslog(LOG_ERR, "evaluate_traptrj: About to divide by zero traj->acc (bad)");
         remaining_time = sqrt((traj->end - traj->cmd)*2/traj->acc);
         if (dt > remaining_time) {
            /* We are within the final step of the trajectory */
            traj->cmd = traj->end;
            traj->state = BTTRAJ_STOPPED;
         }else{
            /* NOTE: This newtime stuff is redundant, why is it here? */
            newtime = (remaining_time-dt);
            if(newtime <= 0) {
               syslog(LOG_ERR, "evaluate_traptrj: How did we get here? new=%f, rem=%f, dt=%f", newtime, remaining_time, dt);
               traj->cmd = traj->end;
               traj->state = BTTRAJ_STOPPED;
            }else{
               traj->cmd =  traj->end - 0.5 * traj->acc * newtime * newtime;
            }
         }
      }
   }
   result = traj->cmd;

   if (isnan(result)) {
      syslog(LOG_ERR, "nan in eval_traj");
      traj->cmd = traj->end;
      traj->state = BTTRAJ_STOPPED;
      return traj->end;
   }
   return result;
}

/**************************** state controller functions ************************/
/** \internal Used by map_btstatecontrol().
Register data input and output variables with virtual trajectory object.
\internal chk'd TH 051102
*/
void mapdata_bttrj(bttrajectory_interface *btt, vect_n* qref, double *dt)
{
   btt->qref = qref;
   btt->dt = dt;
}

/**  \internal Used by map_btstatecontrol().
Register data input and output variables with virtual position control object.
\internal chk'd TH 051102
*/
void mapdata_btpos(btposition_interface *btp,vect_n* q, vect_n* dq, vect_n* ddq,
                   vect_n* qref, vect_n* t, double *dt)
{
   btp->q = q;
   btp->qref = qref;
   btp->dq = dq;
   btp->ddq = ddq;
   btp->dt = dt;
   btp->t = t;
}

/** Register data input and output pointers for the btstatecontrol object.
Allocates a piecewise linear path for moving to start points at the same time. 
 
\internal chk'd TH 051102
\todo Delete the old pwl if there is one. (indicated by a NULL ptr)
 
*/
void map_btstatecontrol(btstatecontrol *sc, vect_n* q, vect_n* dq, vect_n* ddq,
                        vect_n* qref,vect_n* tref,vect_n* t, double *dt)
{
   sc->q = q;
   sc->qref = qref;
   sc->tref = tref;
   init_pwl(&(sc->pth),len_vn(qref),2);
   sc->dq = dq;
   sc->ddq = ddq;
   sc->dt = dt;
   sc->t = t;
   mapdata_bttrj(&sc->btt,tref,dt);
   mapdata_btpos(&sc->btp,q,dq,ddq,qref,t,dt);
}

/** Register virtual functions and data for a trajectory with btstatecontrol.
\internal chk'd TH 051102
*/
void maptrajectory_bts(btstatecontrol *sc,void* dat,void* reset,void* eval,void* getstate)
{
   btrt_mutex_lock(&(sc->mutex));

   if (sc->btt.state == BTTRAJ_STOPPED) {
      sc->btt.reset = reset;
      sc->btt.eval = eval;
      sc->btt.getstate = getstate;
      sc->btt.dat = dat;
      mapdata_bttrj(&(sc->btt),sc->tref,&sc->local_dt);//USE sc->local_dt for pausable
      set_vn(sc->tref,sc->q); //Initialize trajectory output to a sane value
      sc->btt.state = BTTRAJ_STOPPED;
   }
   else{
      syslog(LOG_ERR, "maptrajectory_bts() failed: btt.state was not BTTRAJ_STOPPED");
   }
   btrt_mutex_unlock(&(sc->mutex));
}

/** Register virtual functions and data for a position control object with btstatecontrol.
\internal chk'd TH 051102
*/
void mapposition_bts(btstatecontrol *sc,void* dat,void* reset,void* eval,void* pause)
{
   btrt_mutex_lock(&(sc->mutex));
   if (sc->mode == SCMODE_IDLE) {
      sc->btp.reset = reset;
      sc->btp.eval = eval;
      sc->btp.pause = pause;
      sc->btp.dat = dat;
      mapdata_btpos(&(sc->btp),sc->q,sc->dq,sc->ddq,sc->qref,sc->t,sc->dt);
      sc->btt.state = BTTRAJ_STOPPED;
   }
   btrt_mutex_unlock(&(sc->mutex));
}

/** Initialize the state controller structure
 
Loads a preallocated bts object with initial values.
 
\internal chk'd TH 051102
*/
int init_bts(btstatecontrol *sc)
{

   int err;
#ifdef BT_NULL_PTR_GUARD

   if (!btptr_ok(sc,"moveparm_bts"))
      exit(1);
#endif

   sc->mode = 0;
   sc->last_dt = 1;
   sc->btt.dat = NULL;
   sc->btp.dat = NULL;
   sc->prep_only = 0;
   sc->vel = 0.25; //Safe value for units of radians and meters
   sc->acc = 0.25;
   sc->local_dt = 0.0;
   sc->dt_scale = 1.0;
   init_btramp(&(sc->ramp),&(sc->dt_scale),0.0,1.0,2);
   set_btramp(&(sc->ramp),BTRAMP_MAX);
   btrt_mutex_init(&(sc->mutex));
}

/* Internal function: evaluation portion of case sc.mode */
inline vect_n* eval_trj_bts(btstatecontrol *sc)
{
   int state,rampstate;
#ifdef BT_NULL_PTR_GUARD

   if (!btptr_ok(sc,"eval_trj_bts"))
      exit(1);

   if (sc->btt.dat == NULL) {
      syslog(LOG_ERR,"eval_trj_bts: Pointer to btt.dat is NULL");
      sc->btt.state == BTTRAJ_OFF;
   }
#endif

   state = sc->btt.state;
   if (state == BTTRAJ_INPREP) {
      if (sc->trj.state == BTTRAJ_STOPPED) {
         if (sc->prep_only) {
            sc->btt.state = BTTRAJ_DONE;
            sc->mode = SCMODE_POS;
         } else
            // We are INPREP and STOPPED, therefore start_traptrj() found that
            // dist <= 0 (which is bad)
            //sc->btt.state = BTTRAJ_DONE;
            sc->btt.state = BTTRAJ_RUN;
      }

      set_vn(sc->qref, getval_pwl(&(sc->pth),evaluate_traptrj(&(sc->trj),*(sc->dt))));
   } else if (state == BTTRAJ_RUN || state == BTTRAJ_PAUSING || state == BTTRAJ_UNPAUSING || state == BTTRAJ_PAUSED) {
      rampstate = getstate_btramp(&(sc->ramp));
      if (state == BTTRAJ_PAUSING && rampstate == BTRAMP_MIN)
         sc->btt.state = BTTRAJ_PAUSED;
      else if (state == BTTRAJ_UNPAUSING && rampstate == BTRAMP_MAX)
         sc->btt.state = BTTRAJ_RUN;

      set_vn(sc->qref, (*(sc->btt.eval))(&(sc->btt)));//sc->tref = eval(), sc->qref = sc->tref

      if ((*(sc->btt.getstate))(&(sc->btt)) == BTTRAJ_DONE) {
         sc->btt.state = BTTRAJ_DONE;

      }
   } else {
      sc->mode = SCMODE_POS;
   }
   return sc->qref;
}

/*! Evaluate the state Controller
 
  When switching from torque control to position control, our current
  position is our reference point. This can only be moved by a trajectory.
 
  Changing the state of the position and trajectory controllers is handled by this object.
 
\param position the measured position
\param dt the time step we want to evaluate with.
\return Returns a torque command.
\internal chk'd TH 051102
*/
vect_n* eval_bts(btstatecontrol *sc)
{
   double newcommand;
   double newtorque;
   double cmptorque;
   int err;
#ifdef BT_NULL_PTR_GUARD

   if (!btptr_ok(sc,"eval_bts"))
      exit(1);
#endif

   btrt_mutex_lock(&(sc->mutex));
   sc->error = 0;
   sc->last_dt = *(sc->dt);
   fill_vn(sc->t,0.0);
   switch (sc->mode) {
   case SCMODE_IDLE://Idle

      break;
   case SCMODE_TRJ://PID
      eval_btramp(&(sc->ramp),*(sc->dt));
      sc->local_dt = *(sc->dt) * sc->dt_scale;
      eval_trj_bts(sc); // Calculate sc->qref
      // Intentionally fall through to SCMODE_POS
   case SCMODE_POS://PID
#ifdef BT_NULL_PTR_GUARD

      if (sc->btp.dat == NULL) {
         syslog(LOG_ERR,"eval_trj_bts: Pointer to btp.dat is NULL");
         sc->error = 1;
      } else
#endif

         set_vn(sc->t,(*(sc->btp.eval))(&sc->btp));
      break;
   default:
      sc->error = 1;
      break;
   }
   btrt_mutex_unlock(&(sc->mutex));
   return sc->t;
}

/** Return present mode of a btstatecontrol object.
see #scstate for return enumerations.
 
\internal chk'd TH 051102
*/
int getmode_bts(btstatecontrol *sc)
{
#ifdef BT_NULL_PTR_GUARD
   if (!btptr_ok(sc,"getmode_bts"))
      exit(1);
#endif

   return sc->mode;
}

/** Return present trajectory state of a btstatecontrol object.
see #trjstate for return enumerations.
\internal chk'd TH 051102
*/
int get_trjstate_bts(btstatecontrol *sc)
{
#ifdef BT_NULL_PTR_GUARD
   if (!btptr_ok(sc,"getmode_bts"))
      exit(1);
#endif

   /* If we're NOT doing a prep_only move,
    * we expect some trajectory data in btt.dat: */
   if (!sc->prep_only && !btptr_chk(sc->btt.dat)) {
      return BTTRAJ_OFF;
   } else
      return sc->btt.state;
}

/*! \brief Request a state controller mode
 
  Sets the controller mode. To keep anything from "jumping" unexpectedly, we reset our
  pid controller whenever we switch into pidmode.
  
\param sc Pointer to an sc object
\param mode SCMODE_IDLE or SCMODE_POS 
 
\return Returns what new mode is. see #scstate
 
\internal chk'd TH 051102
*/
int setmode_bts(btstatecontrol *sc, int mode)
{
   int err;
   double tmp;
#ifdef BT_NULL_PTR_GUARD

   if (!btptr_ok(sc,"setmode_bts"))
      exit(1);
#endif

   btrt_mutex_lock(&(sc->mutex));

   switch (mode) {
   case SCMODE_POS:
      if (sc->btp.dat == NULL)
         sc->mode = SCMODE_IDLE;
      else {
         if (sc->btt.state != BTTRAJ_STOPPED)
            sc->btt.state = BTTRAJ_STOPPED;

         if (sc->mode != SCMODE_POS) {
            set_vn(sc->btp.qref,sc->btp.q);
            (*(sc->btp.reset))(&sc->btp);
            sc->mode = SCMODE_POS;
         }
      }
      break;
   case SCMODE_TRJ: //TRJ mode is set by movement commands only
   default:
      sc->mode = SCMODE_IDLE;
      break;
   }
   btrt_mutex_unlock(&(sc->mutex));
   return sc->mode;
}

/** Start playing the loaded trajectory.
 
First we move from the present position to the starting position of the trajectory.
Once there, we start the trajectory with zero velocity.
 
\retval 0 success
\retval -1 Trajectory not in a stopped state. (Prerequisite)
\retval -2 Statecontroller not in POS state. (prerequisite)
\retval -3 NULL trajectory
\retval -4 The trajectory is empty (no points to play).
 
\internal chk'd TH 051102
State transitions:BTTRAJ_READY => BTTRAJ_RUN
*/
int start_trj_bts(btstatecontrol *sc)
{
   char vect_buf1[200];
   vect_n *dest;
   int ret;

#ifdef BT_NULL_PTR_GUARD

   if (!btptr_ok(sc,"prep_trj_bts"))
      exit(1);
#endif

   if (sc->btt.dat == NULL)
      return -3;

   if(sc->mode != SCMODE_POS)
      return -2;


   dest = (*(sc->btt.reset))(&(sc->btt));
   
   
   if (dest == NULL) {
      ret = -4;
   } else if(sc->btt.state == BTTRAJ_STOPPED || sc->btt.state == BTTRAJ_DONE) {
      //vectray *vr = ((via_trj_array*)(sc->btt.dat))->vr;
      
      btrt_mutex_lock(&(sc->mutex));
      sc->mode = SCMODE_TRJ;
      clear_pwl(&(sc->pth)); // Remove all the points on the path

      add_arclen_point_pwl(&(sc->pth),sc->qref); // This is the present position vector
      add_arclen_point_pwl(&(sc->pth),dest); // This is the starting position vector
      //syslog(LOG_ERR, "start_trj_bts: qref=%s", sprint_vn(vect_buf1, sc->qref));
      //syslog(LOG_ERR, "start_trj_bts: dest=%s", sprint_vn(vect_buf1, dest));
      
      setprofile_traptrj(&(sc->trj), sc->vel, sc->acc);

      if (arclength_pwl(&(sc->pth)) < 0.0001) {
         sc->btt.state = BTTRAJ_RUN;
      } else {      
         // This is expecting a pointer to the trj and the total trj time (or total arclen)
         start_traptrj(&(sc->trj), arclength_pwl(&(sc->pth)));
         //start_traptrj(&(sc->trj), vr->data[(vr->num_rows-1)*vr->n] - vr->data[0]);
         set_btramp(&(sc->ramp),BTRAMP_MAX);
         sc->btt.state = BTTRAJ_INPREP;
         sc->prep_only = 0;
      }
      ret = 0;
      btrt_mutex_unlock(&(sc->mutex));
   } else {
      ret = -1;
   }

   return ret;
}

/** Move the wam to a specified position.
 
Use this function to move the wam from it's present position to the one specified
in dest. The move will be a straight line (in the present space) 
trajectory with a trapezoidal velocity profile.
 
\internal chk'd TH 051102
*/
int moveto_bts(btstatecontrol *sc,vect_n* dest)
{
   char vect_buf1[200];
   int ret;
   btreal arclen;
#ifdef BT_NULL_PTR_GUARD

   if (!btptr_ok(sc,"moveto_bts"))
      exit(1);
#endif

   if(sc->mode != SCMODE_POS)
      return -2;
   btrt_mutex_lock(&(sc->mutex));

   if(sc->btt.state == BTTRAJ_STOPPED || sc->btt.state == BTTRAJ_DONE) {
      clear_pwl(&(sc->pth));
      add_arclen_point_pwl(&(sc->pth),sc->qref);
      add_arclen_point_pwl(&(sc->pth),dest);

      setprofile_traptrj(&(sc->trj), sc->vel, sc->acc);
      arclen = arclength_pwl(&(sc->pth));
      start_traptrj(&(sc->trj), arclen);
      set_btramp(&(sc->ramp),BTRAMP_MAX);
      sc->btt.state = BTTRAJ_INPREP;
      sc->prep_only = 1;
      sc->mode = SCMODE_TRJ;
      ret = 0;
#ifdef BTDEBUG
      //syslog(LOG_ERR,"moveto_bts: vel:%f, acc:%f,len:%f",sc->vel,sc->acc,arclen);
#endif

   } else {
      ret = -1;
   }
   btrt_mutex_unlock(&(sc->mutex));

   return ret;
}

/** Set the acceleration and velocity to use during the initial prep move.
 
 
\internal chk'd TH 051102
*/
void moveparm_bts(btstatecontrol *sc,btreal vel, btreal acc)
{
#ifdef BT_NULL_PTR_GUARD
   if (!btptr_ok(sc,"moveparm_bts"))
      exit(1);
#endif

   btrt_mutex_lock(&(sc->mutex));
   sc->vel = vel;
   sc->acc = acc;
   btrt_mutex_unlock(&(sc->mutex));
}

/** Return the state of the present trajectory generator.
see #trjstate for enumerations.
\internal chk'd TH 051102
*/
int movestatus_bts(btstatecontrol *sc)
{
#ifdef BT_NULL_PTR_GUARD
   if (!btptr_ok(sc,"movestatus_bts"))
      exit(1);
#endif

   return sc->btt.state;
}

/** Stop the trajectory generator.
 
\internal chk'd TH 051102
State transitions:
ANY => BTTRAJ_STOPPED
*/
int stop_trj_bts(btstatecontrol *sc)
{
#ifdef BT_NULL_PTR_GUARD
   if (!btptr_ok(sc,"stop_trj_bts"))
      exit(1);
#endif

   btrt_mutex_lock(&(sc->mutex));

   sc->btt.state = BTTRAJ_STOPPED;

   btrt_mutex_unlock(&(sc->mutex));
   return 0;
}

/** Switch to pausing state if possible.
 
\internal chk'd TH 051102
Allowed State transitions:
BTTRAJ_RUN       \
BTTRAJ_PAUSING    => BTTRAJ_PAUSING
BTTRAJ_UNPAUSING  |
BTTRAJ_PAUSED    /
 
 
NULL ptr effect: exit()
*/
int pause_trj_bts(btstatecontrol *sc,btreal period)
{
   int state;
#ifdef BT_NULL_PTR_GUARD

   if (!btptr_ok(sc,"pause_trj_bts"))
      exit(1);
#endif

   state = sc->btt.state;
   if (state == BTTRAJ_RUN || state == BTTRAJ_PAUSING || state == BTTRAJ_UNPAUSING || state == BTTRAJ_PAUSED) {
      setrate_btramp(&(sc->ramp),period);
      set_btramp(&(sc->ramp),BTRAMP_DOWN);


      btrt_mutex_lock(&(sc->mutex));

      sc->btt.state = BTTRAJ_PAUSING;
      btrt_mutex_unlock(&(sc->mutex));
   }
}

/** Switch to running state if possible.
\internal chk'd TH 051102
State transitions:
BTTRAJ_RUN       \
BTTRAJ_PAUSING    => BTTRAJ_UNPAUSING
BTTRAJ_UNPAUSING  |
BTTRAJ_PAUSED    /
 
 
NULL ptr effect: exit()
*/
int unpause_trj_bts(btstatecontrol *sc,btreal period)
{
   int state;
#ifdef BT_NULL_PTR_GUARD

   if (!btptr_ok(sc,"unpause_trj_bts"))
      exit(1);
#endif

   state = sc->btt.state;
   if (state == BTTRAJ_RUN || state == BTTRAJ_PAUSING || state == BTTRAJ_UNPAUSING || state == BTTRAJ_PAUSED) {
      setrate_btramp(&(sc->ramp),period);
      set_btramp(&(sc->ramp),BTRAMP_UP);

      btrt_mutex_lock(&(sc->mutex));

      sc->btt.state = BTTRAJ_UNPAUSING;
      btrt_mutex_unlock(&(sc->mutex));
   }
}

/**************************** state controller functions ************************/

/**************************** btramp functions **********************************/
/** Initialize the data for a btramp object.
 
This should be called once before using this object.
\internal chk'd TH 051102
*/
void init_btramp(btramp *r,btreal *var,btreal min,btreal max,btreal rate)
{
   btrt_mutex_init(&(r->mutex));
   r->scaler = var;
   r->min = min;
   r->max = max;
   r->rate = rate;
}

/** Set the state of a btramp object.
 
See #btramp_state for valid values of state.
\internal chk'd TH 051102
*/
void set_btramp(btramp *r,enum btramp_state state)
{
   btrt_mutex_lock(&(r->mutex));

   r->state = state;
   btrt_mutex_unlock(&(r->mutex));
}

/** Return the present value of a btramp scaler.
 
\internal chk'd TH 051102
*/
btreal get_btramp(btramp *r)
{
   return *(r->scaler);
}

/** Return the present state of a btramp.
   see #btramp_state for return values.
\internal chk'd TH 051102
*/
int getstate_btramp(btramp *r)
{
   return r->state;
}

/** Evaluate a btramp object for time slice dt
 
See btramp documentation for object states. Note that BTRAMP_UP and BTRAMP_DOWN 
will degenerate into BTRAMP_MAX and BTRAMP_MIN respectively. 
\internal chk'd TH 051102
*/
btreal eval_btramp(btramp *r,btreal dt)
{
   btrt_mutex_lock(&(r->mutex));

   if (r->state == BTRAMP_MAX) {
      *(r->scaler) = r->max;
   } else if (r->state == BTRAMP_MIN) {
      *(r->scaler) = r->min;
   } else if (r->state == BTRAMP_UP) {
      *(r->scaler) += dt*r->rate;
      if (*(r->scaler) > r->max) {
         *(r->scaler) = r->max;
         r->state = BTRAMP_MAX;
      }
   } else if (r->state == BTRAMP_DOWN) {
      *(r->scaler) -= dt*r->rate;
      if (*(r->scaler) < r->min) {
         *(r->scaler) = r->min;
         r->state = BTRAMP_MIN;
      }
   }
   //default case is to do nothing
   btrt_mutex_unlock(&(r->mutex));
   return *(r->scaler);
}

/** Set the rate (slope) of the ramping function.
 
\internal chk'd TH 051102
*/
void setrate_btramp(btramp *r,btreal rate)
{
   btrt_mutex_lock(&(r->mutex));
   r->rate = rate;
   btrt_mutex_unlock(&(r->mutex));
}
/**************************** btramp functions **********************************/
