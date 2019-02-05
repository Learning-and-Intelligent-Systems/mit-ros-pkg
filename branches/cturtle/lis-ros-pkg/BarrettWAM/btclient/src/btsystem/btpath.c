/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... btpath.c
 *  Creation Date ...... 30 Mar 2005
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
#include <stdlib.h>
#include <stdio.h>
#include <syslog.h>
#include "btpath.h"

//Internal prototypes
vect_n * idxa_pwl(btpath_pwl *pth,int idx); //internal:
vect_n * idxb_pwl(btpath_pwl *pth,int idx); //internal:
int get_segment_pwl(btpath_pwl *pth,btreal s); //Given an arclength, find what segment we are. (binary search)

/** Allocate memory for a pwl data structure. Further allocation is done by init_pwl()
 
and this is some more documentiatin
*/
btpath_pwl * new_pwl()
{
   btpath_pwl * vmem;
   int cnt;
   
   //allocate mem for vector,return vector, and return structure
   vmem = (btpath_pwl *)btmalloc(sizeof(btpath_pwl));

   return vmem;
}

/** Initialize a piecewize linear path
 
Expects a preinitialized btpath_pwl object.
*/
int init_pwl(btpath_pwl *pth, int vect_size,int rows)
{
   void* vmem;
   int cnt;
   
   //allocate mem for vector,return vector, and return structure
   vmem = btmalloc(rows*sizeof(btreal));

   pth->s = (btreal*)vmem;
   pth->vr = new_vr(vect_size,rows);
   pth->proxy = new_vn(vect_size);
   pth->tmp1 = new_vn(vect_size);
   pth->tmp2 = new_vn(vect_size);
   pth->proxy_s = 0.0;
   pth->segment = 0;
   pth->startnode = 0;
   pth->endnode = 0;
   return 0;
}

/** Initializes a pwl path from an existing vectray.
 
*/
int init_pwl_from_vectray(btpath_pwl *pth,vectray *vr)
{
   void* vmem;
   int vect_size, rows;
   int cnt;

   rows = numrows_vr(vr);
   vect_size = numelements_vr(vr) - 1;
   //allocate mem for vector,return vector, and return structure
   vmem = btmalloc(rows*sizeof(btreal));

   pth->s = (btreal*)vmem;
   pth->vr = new_vr(vect_size,rows);
   pth->proxy = new_vn(vect_size);
   pth->tmp1 = new_vn(vect_size);
   pth->tmp2 = new_vn(vect_size);
   pth->proxy_s = 0.0;
   pth->segment = 0;
   pth->startnode = 0;
   pth->endnode = 0;

   for (cnt = 0; cnt < rows;cnt ++) {
      setrange_vn(pth->tmp1,idx_vr(vr,cnt),0,1,vect_size);
      add_point_pwl(pth,pth->tmp1, getval_vn(idx_vr(vr,cnt),0));
   }
   syslog(LOG_ERR,"init_pwl_from_vectray: %d points",cnt);
   return 0;
}

/** Free the memory allocated during initialization of a btpath_pwl structure
*/
void free_pwl(btpath_pwl *pth)
{
   destroy_vr(&(pth->vr));
   btfree((void**)&(pth->s));
}

/** Break a curve paramaterized by t (usually time) into x(s) and s(t) | s = arclength.
 
*/
void new_param_by_arclen_pwl(btpath_pwl *pth, btpath_pwl *crv2)
{
   btreal t,s;
   vect_n *a;
   int cnt;

   a = new_vn(1);

   init_pwl(crv2,1,maxrows_vr(pth->vr)); //allocate vectray with vector size 1

   //dont assume the parameterization starts at 0.0
   t = getnodes_pwl(pth,0);
   setval_vn(a,0,0.0);
   add_point_pwl(crv2, a, t);

   s = 0.0; //but arclength does start at 0.0

   for(cnt = 1;cnt <= numrows_vr(pth->vr) - 1;cnt++) {
      t = getnodes_pwl(pth,cnt);
      s += norm_vn(sub_vn(idxa_pwl(pth,cnt),idxb_pwl(pth,cnt-1)));
      pth->s[cnt] = s;

      setval_vn(a,0,s);
      add_point_pwl(crv2, a, t);
   }
}

//internal
vect_n * idxa_pwl(btpath_pwl *pth,int idx)
{
   return getvn_vr(pth->tmp1,pth->vr,idx);
}

//internal:
vect_n * idxb_pwl(btpath_pwl *pth,int idx)
{
   return getvn_vr(pth->tmp2,pth->vr,idx);
}

/** Adds a point to a path and computes the arclength which it uses as the
parameter
*/
int add_arclen_point_pwl(btpath_pwl *pth, vect_n *p)
{
   int idx;

   append_vr(pth->vr,p);
   idx = numrows_vr(pth->vr)- 1;
   if (idx==0){ // If this is the first point in the path, set arclength to zero
      pth->s[0] = 0.0;
   }else{ // Store the sum of the prev arclen + this arclen
      pth->s[idx] = pth->s[idx-1] + norm_vn(sub_vn(idxa_pwl(pth,idx),idxb_pwl(pth,idx-1)));
   }
   
   return idx;
}

/** Adds a point to a path with parameter s
*/
int add_point_pwl(btpath_pwl *pth, vect_n *p, btreal s)
{
   int idx;

   idx = numrows_vr(pth->vr) - 1;
   if (pth->s[idx] > s)
      return -1; //time points must be added to the path as monotonically increasing
   append_vr(pth->vr,p);
   idx = numrows_vr(pth->vr) - 1;
   if (idx==0)
      pth->s[0] = 0.0;
   else
      pth->s[idx] = s;

   return idx;
}

/** Remove all the points on a path */
int clear_pwl(btpath_pwl *pth)
{
   int idx;

   clear_vr(pth->vr);
   set_vn(pth->proxy,idx_vr(pth->vr,0));
   pth->proxy_s = pth->s[0];
   pth->segment = 0;
}

/** Add each element in a vectray to pth.
 
Uses the first column as the parameter s.
 
*/
int add_vectray_pwl(btpath_pwl *pth, vectray *vr)
{
   vect_n *cpy;
   int cnt,vect_size,array_size;
   btreal s;

   destroy_vr(&(pth->vr));
   vect_size = vr->n - 1;
   pth->vr = new_vr(vect_size,vr->max_rows);
   cpy = new_vn(vect_size);
   for (cnt = 0;cnt < vr->max_rows;cnt++) {
      setrange_vn(cpy,idx_vr(vr,cnt),0,1,vect_size);
      s = getval_vn(idx_vr(vr,cnt),0);
      add_point_pwl(pth,cpy,s);
   }
}

/** Find the segment that contains arclength s
\param s if s is negative we look for the segment that contains (smax - s)
\return the segment number that contains s, starting with 1
*/
int get_segment_pwl(btpath_pwl *pth,btreal s)
{
   int cnt, end;
   btreal send;

   end = numrows_vr(pth->vr) - 1;
   if (end < 0)
      return -2;

   if (fabs(s) > pth->s[end])
      return end;

   if (s > 0.0) {
      for (cnt = 1; cnt < end; cnt ++) {
         if (s < pth->s[cnt])
            return cnt;
      }
      return end;
   } else {
      send = pth->s[end] - s;
      for (cnt = end; cnt < 1; cnt --) {
         if (send > pth->s[cnt])
            return cnt;
      }
      return 1;
   }
   return -1;
}

/** Return arclength at a node */
btreal getnodes_pwl(btpath_pwl *pth,int idx)
{
   return pth->s[idx];
}

/** Find the point location at arc-distance s into the curve
*/
vect_n * getval_pwl(btpath_pwl *pth, btreal s)
{
   int idx;

   idx = get_segment_pwl(pth,s);

   //If there are no points in the path, return the start of the path
   if (idx <= 0) {
      set_vn(pth->proxy,idx_vr(pth->vr,0));
      pth->proxy_s = pth->s[0];
      pth->segment = 0;
      return pth->proxy;
   }

   set_vn(pth->proxy,interp_vn(idxa_pwl(pth,idx-1),idxb_pwl(pth,idx),pth->s[idx] - pth->s[idx-1],s - pth->s[idx-1]));

   pth->segment = idx;
   pth->proxy_s = s;

   return pth->proxy;
}

/** Increment the proxy_s and return point at the new location
 
 
*/
vect_n* ds_pwl(btpath_pwl *pth, btreal ds)
{
   int idx, end;
   btreal s;

   end = numrows_vr(pth->vr) - 1;

   //If there are no points in the path, return the start of the path
   if (end < 0) {
      set_vn(pth->proxy,idx_vr(pth->vr,0));
      pth->proxy_s = pth->s[0];
      pth->segment = 0;
      return pth->proxy;
   }

   s = ds + pth->proxy_s;
   idx = pth->segment;
   if (s > pth->s[idx]) {
      if (idx >= end) {
         set_vn(pth->proxy,idx_vr(pth->vr,end));
         pth->proxy_s = pth->s[end];
         pth->segment = end;
         return pth->proxy;
      } else {
         pth->segment += 1;
         idx = pth->segment;
      }
   }

   set_vn(pth->proxy,interp_vn(idxa_pwl(pth,idx-1),idxb_pwl(pth,idx),pth->s[idx] - pth->s[idx-1],s - pth->s[idx-1]));

   pth->proxy_s = s;

   return pth->proxy;

}

/** Prep a path object for incremental access */
vect_n* dsinit_pwl(btpath_pwl *pth, btreal s)
{
   return getval_pwl(pth,s);
}

/** Get the total arclength of a path */
btreal arclength_pwl(btpath_pwl *pth)
{
   int idx;

   idx = numrows_vr(pth->vr) - 1;
   //If there are no points in the path, return the start of the path
   if (idx < 0) {
      return 0.0;
   }
   return pth->s[idx];
}
