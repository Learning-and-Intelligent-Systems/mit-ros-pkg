/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... bthaptics.h
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

/** \file bthaptics.h
 
 \brief Haptics scene list and object interaction functions
 
 See code example 4.
 
 */
#ifndef _BTHAPTICS_H
#define _BTHAPTICS_H

#include "btmath.h"
#include "btgeometry.h"

#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */

/** A generic haptic object that presents a virtual interface.

*/
typedef struct bthaptic_object_struct{
  int type;
  int (*interact)(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *vel, vect_n *acc, vect_n *force);
  btreal (*collide)(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *dist);
  int (*normalforce)(struct bthaptic_object_struct *obj, btreal depth, vect_n *dist, vect_n *vel, vect_n *acc, vect_n *force);
  void *geom,*norm_eff,*tang_eff;
  btgeom_state Istate;
  double dist; //debugging tool
  int idx;
}bthaptic_object;

/** A simple haptic scene. */
typedef struct {
  bthaptic_object **list;
  int num_objects;
  int max_objects;
  int state;
}bthaptic_scene;
//Geometry
int new_bthaptic_scene(bthaptic_scene *bth, int size);
vect_n* eval_bthaptics(bthaptic_scene *bth,vect_n *pos, vect_n *vel, vect_n *acc, vect_n *force);
int addobject_bth(bthaptic_scene *bth,bthaptic_object *object);
void removeobject_bth(bthaptic_scene *bth,int index);

int eval_geom_normal_interact_bth(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *vel, vect_n *acc, vect_n *force);


int init_normal_box_bth(bthaptic_object *obj, btgeom_box *box, void*nfobj,void*nffunc);
btreal box_collide_bth(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *norm);

int init_normal_plane_bth(bthaptic_object *obj, btgeom_plane *plane, void *nfobj,void *nffunc);
btreal plane_collide_bth(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *norm);

int init_normal_sphere_bth(bthaptic_object *obj, btgeom_sphere *sphere, void*nfobj,void*nffunc);
btreal sphere_collide_bth(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *norm);

int init_normal_box_bth(bthaptic_object *obj, btgeom_box *box, void*nfobj,void*nffunc);
btreal box_collide_bth(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *norm);

typedef struct { 
  btreal K,B;
  
}bteffect_wall;

void init_wall(bteffect_wall *wall,btreal K, btreal B);
int wall_nf(struct bthaptic_object_struct *obj, btreal depth, vect_n *norm, vect_n *vel, vect_n *acc, vect_n *force);
int sheetwall_nf(struct bthaptic_object_struct *obj, btreal depth, vect_n *norm, vect_n *vel, vect_n *acc, vect_n *force);
 
/** Bullet Proof Wall effect

This effect presents a spring damper wall with two spring constants. The second
constant in added to the first after a certain depth into the wall.

*/
typedef struct { 
  btreal Boffset; //!<Relative start of damping
  btreal K2; //!<second stage spring constant
  btreal K2offset; //!<Distance into wall second spring constant starts
  btreal K1; //!<first stage spring constant
  btreal Bin; //!<damping as you move into the wall
  btreal Bout; //!<damping as you move out of the wall
}bteffect_bulletproofwall;
void init_bulletproofwall(bteffect_bulletproofwall *wall,btreal Boffset,btreal K2, btreal K2offset, btreal K1, btreal Bin, btreal Bout);
int bulletproofwall_nf(struct bthaptic_object_struct *obj, btreal depth, vect_n *norm, vect_n *vel, vect_n *acc, vect_n *force);





/** Wicked Wall haptic effect.

This effect presents a spring-damper wall that shuts off after a specified depth
and does not turn on until the wall is exited.

There are different damping values for entry and exit. The start of damping can 
be offsett from the zero point (start of spring) of the wall.

*/
typedef struct { 
  int state; //!<outside, inside, brokethru
  btreal Boffset; //!<Relative start of damping
  btreal K1; //!<first stage spring constant
  btreal Bin; //!<damping as you move into the wall
  btreal Bout; //!<damping as you move out of the wall
  btreal Thk; //!< Thickness of the wall.
}bteffect_wickedwall;
void init_wickedwall(bteffect_wickedwall *wall,btreal K1, btreal Bin,btreal Bout,btreal Thk,btreal Boffset);
int wickedwall_nf(struct bthaptic_object_struct *obj, btreal depth, vect_n *norm, vect_n *vel, vect_n *acc, vect_n *force);





/** Magnetic Wall effect

This effect is identical to bulletproofwall, except that the wall
exerts an attractive force upon the end of the arm when the arm
is close to the surface. The force is proportional to 1/x^2, where
x is the distance between the end of the arm and the wall. 

*/
typedef struct { 
  btreal Boffset; //!<Relative start of damping
  btreal K2; //!<second stage spring constant
  btreal K2offset; //!<Distance into wall second spring constant starts
  btreal K1; //!<first stage spring constant
  btreal Bin; //!<damping as you move into the wall
  btreal Bout; //!<damping as you move out of the wall
}bteffect_magneticwall;
void init_magneticwall(bteffect_magneticwall *wall,btreal Boffset,btreal K2, btreal K2offset, btreal K1, btreal Bin, btreal Bout);
int magneticwall_nf(struct bthaptic_object_struct *obj, btreal depth, vect_n *norm, vect_n *vel, vect_n *acc, vect_n *force);






typedef struct { 
  btreal B;
  vect_3* F; //bias force
}bteffect_global;
int eval_global_interact_bth(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *vel, vect_n *acc, vect_n *force);
int init_global_bth(bthaptic_object *obj, bteffect_global *global,btreal B,vect_3 *F);


#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif /*_BTHAPTICS_H */
