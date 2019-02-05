/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... btseg.c
 *  Creation Date ...... 20 Oct 2005
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
#include "btseg.h"
#include "btmath.h"
#include "btstatecontrol.h"
#include <stdio.h>
#include <syslog.h>

void dump_pb(parabolic *p,FILE *out)
{
   fprintf(out,"tf: %f, sf: %f, spf: %f, sppf: %f\n",p->tf,p->sf,p->spf,p->sppf);
}

/** Duplicate a parabolic object */
void dup_pb(parabolic *dest,parabolic *src)
{
   dest->tf = src->tf;
   dest->sf = src->sf;
   dest->spf = src->spf;
   dest->sppf = src->sppf;
   dest->next = src->next;
}

/** Given a time, return velocity on a parabolic object
 
*/
btreal sp_of_t_pb(parabolic *p, btreal t)
{
   btreal difft;
   if (t > p->tf)
      return p->spf;
   difft = t - p->tf;
   return p->spf + p->sppf * difft;
}

/** Given a time, return position on a parabolic object
 
\exception If time is greater than time_final, position will be 
position_final.
*/
btreal s_of_t_pb(parabolic *p, btreal t)
{
   btreal difft;
   if (t > p->tf) {
      //End of segment
      return p->sf;
   }
   difft = t - p->tf;
   return p->sf + p->spf * difft + 0.5*p->sppf * difft * difft;
}

/** Given a time and a pointer into a list of parabolic objects, return position.
 
    The pointer must be malleable by this function.
*/
btreal s_of_t_pbl(parabolic **pin, btreal t)
{
   btreal difft;
   parabolic *p;

   p = *pin;
   while (t > p->tf) {
      //Next segment
      p = p->next;
      if (p != NULL) {
         *pin = p;
      } else {
         //no more in the list
         p = *pin;  //backup to the end of the list
         break;
      }
   }
   return s_of_t_pb(p,t);
}

/** Given a time and a pointer into a list of parabolic objects, return velocity.
The pointer must be malleable by this function.
*/
btreal sp_of_t_pbl(parabolic **pin, btreal t)
{
   btreal difft;
   parabolic *p;

   p = *pin;
   while (t > p->tf) {
      //Next segment
      p = p->next;
      if (p != NULL) {
         *pin = p;
      } else {
         //no more in the list
         p = *pin;  //backup to the end of the list
         break;
      }
   }
   return sp_of_t_pb(p,t);
}

/** Init parabolic object from boundary conditions
*/
btreal boundary_pb(parabolic *b,btreal t0, btreal s0,btreal sf,btreal sp0,btreal spf)
{
   btreal a,tf;
   //spf = sp0 + a*tf
   //sf = s0 + sp0*tf + 0.5*a*tf^2

   //spf = sp0 + a*tf
   //spf - sp0 = a*tf
   //sf = s0 + sp0*tf + 0.5*(spf - sp0)*tf
   //sf = s0 + sp0*tf + 0.5*spf*tf - 0.5*sp0*tf
   //sf = s0 + 0.5*sp0*tf + 0.5*spf*tf
   //(sf - s0)*2/(sp0 + spf) = tf

   tf = 2*(sf-s0)/(sp0+spf);
   a = (spf - sp0)/(tf);

   b->sf = sf;
   b->tf = tf + t0;
   b->sppf = a;
   b->spf = spf;
   return b->tf;
}

/** Calculate a parabolic segment given initial values
 
\param 
 - b pointer to a parabolic object to be filled with calc results
 - t0 Initial time
 - s0 Initial value
 - sf Final value
 - spf Final ds/dt
 - tf Final time
\return
  Final time
 
*/
btreal s0sfspftf_pb(parabolic *b,btreal t0, btreal s0,btreal sf,btreal spf,btreal tf)
{

   btreal a,sp0,t;
   /*Derivation:
     spf = sp0 + a*tf
     sf = s0 + sp0*tf + 0.5*a*tf^2
     sf = s0 + sp0*tf + (sp0 - spf)*t^2/(2*t)
     sf = s0 + sp0*tf + sp0*t/2 - spf*t/2
     sf = s0 + sp0*t/2 + spf*t/2
     s0 = 2.0*sf/t - 2.0*s0/t - spf;
     
   */

   t = tf - t0;
   if (t != 0.0) {
      sp0 = 2.0*sf/t - 2.0*s0/t - spf;
      a = (spf - sp0)/t;
   } else {
      a = 0.0;
   }
   b->sf = sf;
   b->tf = tf;
   b->sppf = a;
   b->spf = spf;
   //printf("\nt0: %f, s0: %f, sp0: %f\n",t0,s0,sp0);
   //dump_pb(b,stdout);
   return b->tf;
}

/**
 
t = corner point time
st = corner point location
 
sp0 = starting velocity
spf = final velocity
t0 = starting time
*/
btreal blend_pb(parabolic *b,btreal t0, btreal st,btreal sp0,btreal spf,btreal t)
{
   btreal tf,a,s0,sf;

   tf = (t-t0)*2;
   //spf = sp0 + a*tf
   //a = (spf-sp0)/tf;
   //st = s0 + sp0*(t-t0);
   s0 = st - sp0*(t-t0);
   //sf = s0 + sp0*tf + 0.5*a*tf^2
   sf = st + spf*(tf-t);

   return boundary_pb(b,t0,s0,sf,sp0,spf);
}


/************** Segment array functions *****************/
pararray* new_pa(int max)
{
   void* mem;
   pararray* pa;

   /*malloc_pa*/
   mem = btmalloc(sizeof(parabolic) * max + sizeof(pararray));

   /*initptr_pa*/
   pa = (pararray*)mem;
   pa->pb = (parabolic*)(mem + sizeof(pararray));

   /*initval_pa*/
   pa->max = max;
   pa->cnt = 0;
   pa->state = BTTRAJ_STOPPED;
   pa->t = 0.0;
   return pa;
}

void destroy_pa(pararray** pa)
{
   btfree((void**)pa);
}

void clear_pa(pararray* pa)
{
   pa->cnt = 0;
}

/** Adds to the end of the list */
btreal add_bseg_pa(pararray* pa,parabolic* p)
{
   int cnt;
   btreal ret;
   if (pa->cnt > pa->max-1)
      ret = pa->pb[pa->cnt].tf;
   else {
      dup_pb(&(pa->pb[pa->cnt]),p);

      //link with previous segment
      if (pa->cnt > 0) {
         pa->pb[pa->cnt - 1].next = &(pa->pb[pa->cnt]);
      }
      pa->pb[pa->cnt].next = NULL;
   }

   pa->cnt++;

   return ret;
}

btreal reset_pa(pararray* pa)
{
   int cnt;
   pa->iter = &(pa->pb[0]);
   pa->state = BTTRAJ_RUN;
   pa->t = 0.0;
   return eval_pa(pa,0.0);
}

btreal eval_pa(pararray* pa,btreal dt)
{
   int cnt;
   btreal ret;
   pa->t += dt;
   ret = s_of_t_pbl(&(pa->iter),pa->t);
   if (pa->iter->next == NULL)
      pa->state = BTTRAJ_DONE;
   return ret;
}

/************************ pararray_vn **************************************/
pararray_vn* new_pavn(int max, int elements)
{
   void           *vmem;
   int            cnt;
   pararray_vn    *pavn;
   
   vmem = btmalloc(sizeof(pararray_vn) + sizeof(pararray*) * elements);
   pavn = (pararray_vn*)vmem;
   pavn->pa = (pararray**)(vmem + sizeof(pararray_vn));
   pavn->elements = elements;
   for (cnt = 0; cnt < elements; cnt++)
      pavn->pa[cnt] = new_pa(max);
   pavn->result = new_vn(elements);
   return pavn;
}

void destroy_pavn(pararray_vn** pavn)
{
   int cnt;

   if (*pavn != NULL) {
      destroy_vn(&(*pavn)->result);
      for (cnt = 0; cnt < (*pavn)->elements; cnt ++)
         destroy_pa(&(*pavn)->pa[cnt]);
      
      btfree((void**)pavn);
   }
}

void clear_pavn(pararray_vn* pavn)
{
   int cnt;
   for (cnt = 0;cnt< pavn->elements; cnt ++)
      clear_pa(pavn->pa[cnt]);

}

/** Prep a pavn object for evaluation */
vect_n*  reset_pavn(pararray_vn* pavn)
{
   int cnt;
   btreal ret;

   for (cnt = 0;cnt< pavn->elements; cnt ++) {
      setval_vn(pavn->result,cnt,reset_pa(pavn->pa[cnt]));
   }
   return pavn->result;
}

/** Evaluate a pavn object. Return position for time t.*/
vect_n* eval_pavn(pararray_vn* pavn,btreal dt)
{
   int cnt;
   btreal ret;
   for (cnt = 0;cnt< pavn->elements; cnt ++)
      setval_vn(pavn->result,cnt,eval_pa(pavn->pa[cnt],dt));
   return pavn->result;
}

/** Return the state of the trajectories
Returns BTTRAJ_RUN until t > tF for all dimensions.
*/
int getstate_pavn(pararray_vn* pavn)
{
   int cnt,state;
   btreal ret;
   state = BTTRAJ_DONE;
   for (cnt = 0;cnt< pavn->elements; cnt ++)
      if (pavn->pa[cnt]->state == BTTRAJ_RUN)
         state = BTTRAJ_RUN;
   return state;
}

/**
Convert a vectray of time/points to a piecewise parabolic function.
 
Time in the first value;
Time values must be monotonically increasing. Results are otherwize undefined.
 
*/
pararray_vn* vr2pararray(vectray* vr,btreal acceleration)
{
   pararray_vn* pavn;
   parabolic pa;
   int cnt,idx;
   btreal t1,t2,t3,x1,x2,x3;
   btreal v1,v2,v3,tacc,t1p2,t2p3,tmax;
   btreal tf,a,s0,sf;
   btreal sv0,svf,sa0,saf,sa0_last,v1_last,saf_last;
   btreal s0_prev,tf_prev;
   btreal acc,min_acc;
   btreal dt,dx,t_last;

   vect_n *p0, *pf; //store first and last points of vr

   p0 = new_vn(numelements_vr(vr)); // numelements_vr() returns vr->n (columns)
   pf = new_vn(numelements_vr(vr));

   set_vn(p0, idx_vr(vr, 0));
   set_vn(pf, idx_vr(vr, numrows_vr(vr)-1));

   //new pararray of size (Viapoints - 1)*2 + 1
   pavn = new_pavn(2*numrows_vr(vr) -1 +5, numelements_vr(vr)-1);

   tmax = 0;
   // For each column of pavn
   for (cnt = 0; cnt < pavn->elements; cnt ++) {
      acc = fabs(acceleration);

      /* First acceleration segment calcs*/
      t1 = getval_vn(idx_vr(vr,0),0);
      t2 = getval_vn(idx_vr(vr,1),0);
      x1 = getval_vn(idx_vr(vr,0),cnt+1);
      x2 = getval_vn(idx_vr(vr,1),cnt+1);

      dt = t2-t1;
      dx = x2-x1;
      v1 = 0.0;
      if(dt == 0.0)
         syslog(LOG_ERR, "vr2pararray(): about to divide by dt = 0.0");
      v2 = dx/dt;

      min_acc = 8.0 * fabs(dx) / (3.0 * dt*dt);
      if(isnan(min_acc))
         syslog(LOG_ERR, "vr2pararray(): min_acc is NaN (bad)");
      
      //Make sure initial acceleration is fast enough
      if (acc < min_acc) {
         //syslog(LOG_ERR,"vr2pararray: Boosting initial acc (%f) to %f", acc, min_acc);
         acc = min_acc;
      }
      tacc = dt - sqrt(dt*dt - 2*fabs(dx)/acc);

      saf = x1 + 0.5*acc*tacc*tacc*Sgn(dx); //Final x
      v2 = (x2-saf)/(dt-tacc); //final velocity
      if(isnan(v2))
         syslog(LOG_ERR, "vr2pararray: v2 is NaN (bad)");
      sa0 = x1;

      tf_prev = s0sfspftf_pb(&pa, 0.0, sa0, saf, v2, tacc);  //acc seg starting at time 0.0
      add_bseg_pa(pavn->pa[cnt],&pa);

      setval_vn(idx_vr(vr,0),cnt+1,x2-dt*v2);
      idx = numrows_vr(vr)-1;

      /* Last velocity and acceleration segment calcs */
      acc = fabs(acceleration);
      t1 = getval_vn(idx_vr(vr,idx-1),0);
      t2 = getval_vn(idx_vr(vr,idx),0);
      x1 = getval_vn(idx_vr(vr,idx-1),cnt+1);
      x2 = getval_vn(idx_vr(vr,idx),cnt+1);

      dt = t2-t1;
      dx = x2-x1;
      v1 = dx/dt;
      v2 = 0.0;

      min_acc = 8.0*fabs(dx)/(3.0*dt*dt);
      //Make sure final acceleration is fast enough
      if (acc < min_acc) {
         acc = min_acc;
         //syslog(LOG_ERR,"vr2pararray: Boosting final acc to %f",acc);
      }
      tacc = dt - sqrt(dt*dt - 2*fabs(dx)/acc);
      sa0_last = x2 - 0.5*acc*tacc*tacc*Sgn(dx); //Final x
      v1_last = (sa0_last-x1)/(dt-tacc); //final velocity
      saf_last = x2;
      t_last = tacc;
      setval_vn(idx_vr(vr,idx),cnt+1,x1+dt*v1_last);

      /*Internal (remaining) segments calcs*/
      acc = fabs(acceleration);
      for(idx = 1; idx < numrows_vr(vr)-1; idx++) {

         /* Extract info */
         t1 = getval_vn(idx_vr(vr,idx-1),0);
         t2 = getval_vn(idx_vr(vr,idx),0);
         t3 = getval_vn(idx_vr(vr,idx+1),0);
         x1 = getval_vn(idx_vr(vr,idx-1),cnt+1);
         x2 = getval_vn(idx_vr(vr,idx),cnt+1);
         x3 = getval_vn(idx_vr(vr,idx+1),cnt+1);


         /* Calc some useful values */
         //if ((t2-t1) <= 0.0 || (t3-t2) <= 0.0)
            //syslog(LOG_ERR,"vr2pararray: Equal time points and unsortet times are not supported.",tacc,idx);
         v1 = (x2-x1)/(t2-t1);
         v2 = (x3-x2)/(t3-t2);
         t1p2 = (t1 + t2)/2;
         t2p3 = (t2 + t3)/2;

         /* Shrink acceleration if necessary */
         tmax = min_bt(2*(t2-t1p2),2*(t2p3-t2));

         //vf = v0 + at => t = (vf-v0)/a
         tacc = fabs(v2-v1)/acc;

         if (tmax < tacc) {
            //Need to increase acceleration to make corner
            tacc = tmax;
            //syslog(LOG_ERR,"vr2pararray: Forcing acc time decrease to %f at point %d.",tacc,idx);
         }

         /* Calc : tf_prev & saf carry history from prev loops */
         sa0 = x2 - v1*(tacc/2); //acc start pos
         tf_prev = s0sfspftf_pb(&pa,tf_prev,saf,sa0,v1,t2-tacc/2); //velocity seg
         add_bseg_pa(pavn->pa[cnt],&pa);

         saf = x2 + v2*(tacc/2); //acc end pos
         tf_prev = s0sfspftf_pb(&pa,tf_prev,sa0,saf,v2,t2+tacc/2);  //acc seg
         add_bseg_pa(pavn->pa[cnt],&pa);
         //usleep(1);
      }

      v2 = 0.0;
      tf_prev = s0sfspftf_pb(&pa,tf_prev,saf,sa0_last,v1_last,t3-t_last); //last velocity seg
      add_bseg_pa(pavn->pa[cnt],&pa);

      tf_prev = s0sfspftf_pb(&pa,tf_prev,sa0_last,saf_last,v2,t3);  //acc seg ending at tf
      add_bseg_pa(pavn->pa[cnt],&pa);

   }

   set_vn(idx_vr(vr,0),p0);
   set_vn(idx_vr(vr,numrows_vr(vr)-1),pf);

   return pavn;
}

/*
int main()
{
  FILE *out;
  char buf[250];
  int cnt;
  pararray *pb;
  pararray_vn *pvn;
  vectray *vr;
  btreal t,s;
  pb = new_pa(10);
  s = 0.0;
 
  read_csv_file_vr("t2",&vr);
  write_csv_file_vr("t3",vr);
  pvn = vr2pararray(vr,.03);
  out = fopen("curve.csv","w");
  s = 0;
 
  
  reset_pavn(pvn);
  t = getval_vn(idx_vr(vr,numrows_vr(vr)-1),0);
  for(cnt = 0;cnt < 501;cnt++){
    
    //set_vn(res,calc_cseg(&t,s));
    s+=t/500;
    fprintf(out,"%f %s\n",s,sprint_plt_vn(buf,eval_pavn(pvn,s)));
    //fprintf(out,"%f %f\n",s,eval_pa(pb,s));
    //fprintf(out,"%f, %f\n",s,calc_tseg(&ms,s));
  }
  
  return 0;
}
 
 
*/
