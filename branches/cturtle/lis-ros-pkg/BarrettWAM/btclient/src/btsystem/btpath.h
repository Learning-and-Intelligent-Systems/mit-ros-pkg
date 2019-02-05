/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... btpath.h
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
 
/** \file btpath.h
\brief Piecewize space curve geometry

The btpath_pwl object is a primitive meant for use either in trajectories or
haptic curve geometry. It's main property is that it paramaterizes a curve
by arc-length.

Presently this object is only used by the #btstatecontrol point-to-point move
code.

*/
#ifndef _BTPATH_H
#define _BTPATH_H

#include "btmath.h"

#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */
 
 

/** Piecewize Linear Path

btpath_pwl is for N dimensional piecewize linear curves

 The parameter s is assumed to be monotonically increasing.

  Paths define geometry in space.
  Trajectories define a location and velocity as a function of time.
  Paths are stored as functions of pathlength


*/
typedef struct {
  vectray *vr; //size in vect_n size 
  btreal *s; //for each point we record the total arc length
  vect_n* proxy; //current point
  vect_n *tmp1,*tmp2; //current point
  btreal proxy_s;
  int segment;
  int startnode,endnode;
  int type; //!< 0 = cartesial, 1 = SE(3)
  int param; //0 = arclength, 1 = time
  
}btpath_pwl;

// Init / Destroy
int init_pwl(btpath_pwl *pth, int vect_size,int rows);
int init_pwl_from_vectray(btpath_pwl *pth,vectray *vr);

btpath_pwl * new_pwl();
void free_pwl(btpath_pwl *pth);
void new_param_by_arclen_pwl(btpath_pwl *pth, btpath_pwl *crv2); //creates a second curve of arclength vs time

// PWL Curve Data Manipulation
int add_arclen_point_pwl(btpath_pwl *pth, vect_n *p); //Add a point to the arc, parameterized by arclength, returns index to the point
int clear_pwl(btpath_pwl *pth); //Erase the present path
int add_point_pwl(btpath_pwl *pth, vect_n *p, btreal s); //Add a point to the arc, parameterized by time, returns index to the point
//void rem_point(btpath_pwl *pth, int idx); //Add a point to the arc
int add_vectray_pwl(btpath_pwl *pth, vectray *vr);//use the first column for the parameter
int add_arclen_vectray_pwl(btpath_pwl *pth, vectray *vr);

// PWL Curve Usage
vect_n * getval_pwl(btpath_pwl *pth, btreal s); //Given arclength , find location, set the proxy to it
vect_n* dsinit_pwl(btpath_pwl *pth, btreal s);
vect_n* ds_pwl(btpath_pwl *pth, btreal ds); //Shift our proxy by ds and return the position.

btreal getnodes_pwl(btpath_pwl *pth,int idx); //return arclength at node[idx]
btreal arclength_pwl(btpath_pwl *pth);

//int read_cvs_pwl(btpath_pwl *pth,FILE *infile);
//int write_cvs_pwl(btpath_pwl *pth,FILE *outfile);

/** Curve Trajectory Controller

  Curve control
    Reset curve to s = x;
    eval(ds)
    deval(ds)
    ddeval(ds)
    get_length
    loop
    
  Trajectory Ctl
    Reset to t = x
    eval(t)
    eval(dt)

*/








#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif /*_BTMATH_H */
