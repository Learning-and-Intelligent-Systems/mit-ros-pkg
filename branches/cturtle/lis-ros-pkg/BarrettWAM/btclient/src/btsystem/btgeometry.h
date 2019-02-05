/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... btgeometry.h
 *  Creation Date ...... 29 Apr 2005
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

/** \file btgeometry.h
    \brief Analytic geometry objects

 The objects in this file are primarily used by bthaptics.c

*/
#ifndef _BTGEOMETRY_H
#define _BTGEOMETRY_H

#include "btmath.h"

#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */


/** Tracks the state of an object.

Given position updates at constant time intervals, this object tracks the
velocity and acceleration.

Uses low pass digital filters so you must insure that the samplerate is constant.

Position,velocity, and acceleration are assumed to be 3-vectors.

*/
typedef struct {
  vect_3 *pos,*vel,*acc; //Normal of plane
  btfilter_vn *velfilt,*accfilt;
}btgeom_state;

void init_state_btg(btgeom_state *bts,btreal samplerate,btreal cutoffHz);
void eval_state_btg(btgeom_state *bts,vect_3* pos);


/** A analytic line segment object.

(Not yet implemented)

*/
typedef struct {
  vect_n *start,*end; //Normal of plane
  vect_n *unit;
}btgeom_lineseg;

void init_Seg_btg(btgeom_lineseg *seg,int size);
void set_Seg_btg(btgeom_lineseg *seg,vect_n *p1,vect_n *p2);
btreal D_Ln2Pt(btgeom_lineseg *seg,vect_n *pt); 
/* Not yet implemented...

btreal I_Ln2Ln(btgeom_lineseg *seg,vect_n *pt); 
btreal D_Seg2Pt(btgeom_lineseg *seg,vect_n *pt); //Distance
btreal Pt_Seg2Pt(vect_n *loc,btgeom_lineseg *seg,vect_n *pt); //Distance, return point on line
btreal D_Seg2Seg(btgeom_lineseg *seg1,btgeom_lineseg *seg2); //Distance
*/
/** An analytic plane object.


*/
typedef struct {
  vect_3 *normal; //Normal of plane
  btreal distance; //Distance from origin to plane in direction of normal
}btgeom_plane;

btgeom_plane* new_pl_btg();
int init_pl_btg( btgeom_plane *plane, vect_3 *pt1, vect_3 *pt2, vect_3 *pt3);
//int init_3point_plane_btg( btgeom_plane *plane, vect_3 *pt1, vect_3 *pt2, vect_3 *pt3);
//int init_pointnormal_plane_btg( btgeom_plane *plane, vect_3 *pt1, vect_3 *pt2);
btreal D_Pt2Pl(vect_3 *norm,btgeom_plane *plane, vect_3 *point);

//vect_3* I_Li2Pl(

typedef struct {
  vect_3 *center;
  double radius;
  int inside;
}btgeom_sphere;

int init_sp_btg( btgeom_sphere *sphere, vect_3 *pt1, vect_3 *pt2,int inside);
btreal D_Pt2Sp(vect_3 *norm,btgeom_sphere *sp, vect_3 *pt);




btreal D_Pt2Pt(vect_3 *pt1, vect_3 *pt2);



typedef struct {
  btgeom_plane side[6];
  int inside;
  vect_3 *center; //Center and size of box
  matr_3 *normals; //orientation of box
}btgeom_box;
/**
Define plane with 3 points
thk = dim of box in dir of plane normal
dir1 = dim of box in dir of pt1 to pt2
dir2 = dim of box in dir of pt3 normal to pt1,pt2 line
*/
int init_bx_btg( btgeom_box *box,vect_3 *pt1, vect_3 *pt2, vect_3 *pt3,btreal thk,btreal dir1,btreal dir2,int inside);
btreal D_Pt2Bx(vect_3 *norm,btgeom_box *bx, vect_3 *pt);

#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif /*_BTHAPTICS_H */
