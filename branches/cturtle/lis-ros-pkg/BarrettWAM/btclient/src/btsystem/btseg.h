/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... btseg.h
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
 
#ifndef _BTSEG_H
#define _BTSEG_H

#include "btmath.h"

#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */



/** A parabolic function object.

This object stores individual segments of the piecewise 
parabolic function (pararray) and implements the mathematics for these segments.

Given final values for position, velocity, acceleration and time (sf, spf, sppf,
and tf respectively) calculate position and/or velocity for any point in time 
prior to tf.

As time approaches tf, s & sp approach sf & spf;
sppf is constant;

Symbols:
s: length (dependant) parameter
t: time (independent) parameter
0: initial value (t0 = initial time)
f: final value
p: prime (d/dt) (velocity)
pp: (d/dt^2) (acceleration)

f_of_t: f(t) (s_of_t=>s(t))


*/
typedef struct parabolic_s{
  btreal sf,spf,sppf,tf;
  unsigned int type; //0 = normal, 1 = pause here, 2 = looping
  struct parabolic_s *next;
}parabolic;

void dump_pb(parabolic *p,FILE *out);

//Evaluation functions
btreal sp_of_t_pb(parabolic *p, btreal t);
btreal s_of_t_pb(parabolic *p, btreal t);
btreal s_of_t_pbl(parabolic **pin, btreal t);

//Definition & Initial Calcs from various combinations of boundary conditions.
btreal boundary_pb(parabolic *b,btreal t0, btreal s0,btreal sf,btreal sp0,btreal spf);
btreal blend_pb(parabolic *b,btreal t0, btreal st,btreal sp0,btreal spf,btreal t);
btreal s0sfspftf_pb(parabolic *b,btreal t0, btreal s0,btreal sf,btreal spf,btreal tf);

/** Parabolic segment trajectory controller Design Ideas: (not necessarily implemented)
Circular buffer.
Adding a segment to an empty list when it is "on" starts playing that segment using
the present time as t0.
When the last segment of the list is reached it pauses waiting for the next one.

You can turn it "off" to pre fill the buffer
You can loop
You can pause

Time starts at 0.0 from reset.
*/

/** One dimensional piecewise parabolic function.
This object stores a piecewise parabolic function (in one dimension), evaluates the
function relative to independent variable t, and keeps state information for t.

reset_pa() will initialize t to 0.
eval_pa() will increment t by dt until t > tf of the final segment after which it 
will return sf of the final segment. 

Symbols:
tF: tf of the last segment in the list.
*/
typedef struct pararray_s{
  parabolic *pb;
  parabolic *iter;
  double t; //time state variable
  int cnt,max; //Length of array,Max available memory
  int state;
}pararray;

pararray* new_pa(int max);
void destroy_pa(pararray** pa);

void clear_pa(pararray* pa);
btreal add_bseg_pa(pararray* pa,parabolic* p);

btreal  reset_pa(pararray* pa);
btreal eval_pa(pararray* pa,btreal dt);

/** Multi-dimensional piecewise parabolic functions.
A multi-dimensional version of pararray.

Inputs and outputs are vect_n's.

getstate_pavn() returns BTTRAJ_RUN until all t > tF for all dimensions.
*/
typedef struct pararray_vns{
  pararray **pa;
  int elements; //Length of array,Max available memory
  vect_n* result;
  int state;
}pararray_vn;

pararray_vn* new_pavn(int max,int elements);
void destroy_pavn(pararray_vn** pavn);
void clear_pavn(pararray_vn* pavn);
vect_n*  add_bseg_pavn(pararray_vn* pavn,vect_n* t0,vect_n* s0,vect_n* sf,vect_n* sp0,vect_n* spf);
vect_n*  reset_pavn(pararray_vn* pavn);
vect_n* eval_pavn(pararray_vn* pavn,btreal dt);
int getstate_pavn(pararray_vn* pavn);
pararray_vn* vr2pararray(vectray* vr,btreal acc);

#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif/* _BTSEG_H */
