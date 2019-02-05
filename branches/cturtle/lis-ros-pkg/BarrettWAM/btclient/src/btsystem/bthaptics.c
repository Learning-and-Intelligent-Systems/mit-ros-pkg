/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... bthaptics.c
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

#ifdef S_SPLINT_S
#include <err.h>
#endif
#include <stdlib.h>
#include <stdio.h>
#include <syslog.h>
#include "bthaptics.h"

int audio = 0;
btreal extVel;
int brake;
btreal setBrake, relBrake;

/** Allocate and initialize memory for a haptic scene.
 
\param size The maximimum number of objects in the scene.
 
*/
int new_bthaptic_scene(bthaptic_scene *bth, int size)
{
   bth->num_objects = 0;
   bth->max_objects = size;
   bth->list = (bthaptic_object**)btmalloc(size * sizeof(void*));

   bth->state = 0;
   return 0;
}

/** Evaluate a haptic scene.
    Each object adds it's effect to the tipforce
*/
vect_n* eval_bthaptics(bthaptic_scene *bth,vect_n *pos, vect_n *vel, vect_n *acc, vect_n *force)
{
   static int i;
   if (bth->state) {
      for(i=0; i < bth->num_objects; i++) {
         if(bth->list[i] != NULL) {
            (*(bth->list[i]->interact))(bth->list[i],pos,vel,acc,force);
         }
      }
   }
   return force;
}

/** Add a haptic object to the haptic scene. */
int addobject_bth(bthaptic_scene *bth,bthaptic_object *object)
{
   int i;
   if(bth->num_objects >= bth->max_objects)
      return -1;
   bth->list[bth->num_objects] = object;
   object->idx = bth->num_objects;
   bth->num_objects++;
   return(bth->num_objects - 1);
}

/** Remove a haptic object to the haptic scene. */
void removeobject_bth(bthaptic_scene *bth,int index)
{
   int cnt;

   if ((index >= 0) && (index < bth->num_objects)) {
      bth->list[index] = NULL;
      bth->num_objects--;
      for (cnt = index; cnt < bth->num_objects; cnt++)
         bth->list[cnt] = bth->list[cnt + 1];
      bth->list[bth->num_objects] = NULL;
   }
}

/** Evaluate a typical analytical geometry object.
   Given a position for an interaction point; Calculate the reaction forces on
   that point.
*/
int eval_geom_normal_interact_bth(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *vel, vect_n *acc, vect_n *force)
{
   btreal depth;
   depth = (*(obj->collide))(obj,pos,(vect_n*)obj->Istate.pos);
   (*(obj->normalforce))(obj,depth,(vect_n*)obj->Istate.pos,vel,acc,force);
}

/** Evaluate global effects for a hapic object.
*/
int eval_global_interact_bth(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *vel, vect_n *acc, vect_n *force)
{
   bteffect_global *gp;
   gp = (bteffect_global *)obj->norm_eff;
   set_vn(force,add_vn(force,add_vn(scale_vn(-1.0*gp->B,vel),(vect_n*)gp->F)));

}

/** Initialize a global effects object
*/
int init_global_bth(bthaptic_object *obj, bteffect_global *global,btreal B,vect_3 *F)
{

   global->F = new_v3();
   set_v3(global->F,F);
   global->B = B;
   init_state_btg(&(obj->Istate),0.002,30);
   obj->interact = eval_global_interact_bth;
   obj->norm_eff = global;
}

/**
\warning this uses a digital filter which assumes constant sample period and the sample rate and
cutoff freq are hard coded
*/
int init_normal_plane_bth(bthaptic_object *obj, btgeom_plane *plane, void*nfobj,void*nffunc)
{
   init_state_btg(&(obj->Istate),0.002,30);
   obj->interact = eval_geom_normal_interact_bth;
   obj->collide = plane_collide_bth;
   obj->geom = (void *) plane;
   obj->normalforce = nffunc;
   obj->norm_eff = nfobj;
}

/** Test for collision with a plane object.
This is a #bthaptic_object_struct virtual interface implimentation.
*/
btreal plane_collide_bth(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *norm)
{
   btreal res;
   res = D_Pt2Pl((vect_3*)norm,(btgeom_plane *)obj->geom,(vect_3*)pos);
   return res;
}

/** Initialize a sphere haptic object.
*/
int init_normal_sphere_bth(bthaptic_object *obj, btgeom_sphere *sphere, void*nfobj,void*nffunc)
{
   init_state_btg(&(obj->Istate),0.002,30);
   obj->interact = eval_geom_normal_interact_bth;
   obj->collide = sphere_collide_bth;
   obj->geom = (void *) sphere;
   obj->normalforce = nffunc;
   obj->norm_eff = nfobj;
}

/** Test for collision with a haptic sphere.
 
This is a #bthaptic_object_struct virtual interface implimentation.
*/
btreal sphere_collide_bth(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *norm)
{
   btreal res;
   res = D_Pt2Sp((vect_3*)norm,(btgeom_sphere *)obj->geom,(vect_3*)pos);
   //set_v3(obj->Istate.pos,scale_v3(res,(vect_3*)norm));
   return res;
}


/** Initialize a sphere haptic object.
*/
int init_normal_box_bth(bthaptic_object *obj, btgeom_box *box, void*nfobj,void*nffunc)
{
   init_state_btg(&(obj->Istate),0.002,30);
   obj->interact = eval_geom_normal_interact_bth;
   obj->collide = box_collide_bth;
   obj->geom = (void *) box;
   obj->normalforce = nffunc;
   obj->norm_eff = nfobj;
}

/** Test for collision with a box object.
This is a #bthaptic_object_struct virtual interface implimentation.
*/
btreal box_collide_bth(struct bthaptic_object_struct *obj, vect_n *pos, vect_n *norm)
{
   btreal res;
   res = D_Pt2Bx((vect_3*)norm,(btgeom_box *)obj->geom,(vect_3*)pos);
   obj->dist = res;
   return res;
}

/** Initialize a haptic wall effect */
void init_wall(bteffect_wall *wall,btreal K, btreal B)
{
   wall->K = K;
   wall->B = B;
}

/** Calculate wall interaction forces
This is a #bthaptic_object_struct virtual interface implimentation.
*/
int wall_nf(struct bthaptic_object_struct *obj, btreal depth, vect_n *norm, vect_n *vel, vect_n *acc, vect_n *force)
{
   btreal Dist,K,B;
   bteffect_wall *norm_eff;
   btreal WallStiff,WallDamp,Vel;

   WallStiff = 0.0;
   WallDamp = 0.0;

   norm_eff = (bteffect_wall*)obj->norm_eff;
   Vel = dot_v3((vect_3*)norm,(vect_3*)vel);

   if (depth < 0.0)
   {
      WallStiff = -1.0*norm_eff->K*depth;
   }

   if (depth  < 0.0)
   {
      if(Vel < 0.0)
         WallDamp = -1.0*norm_eff->B*Vel;
   }
   set_v3((vect_3*)force,
          add_v3((vect_3*)force,
                 scale_v3(WallStiff + WallDamp,(vect_3*)norm)));
}

/** Calculate wall interaction forces
This is a #bthaptic_object_struct virtual interface implimentation.
*/
int sheetwall_nf(struct bthaptic_object_struct *obj, btreal depth, vect_n *norm, vect_n *vel, vect_n *acc, vect_n *force)
{
   btreal Dist,K,B;
   bteffect_wall *norm_eff;
   btreal WallStiff,WallDamp,Vel;

   WallStiff = 0.0;
   WallDamp = 0.0;

   norm_eff = (bteffect_wall*)obj->norm_eff;
   Vel = dot_v3((vect_3*)norm,(vect_3*)vel);

   WallStiff = -1.0*norm_eff->K*depth;

   WallDamp = -1.0*norm_eff->B*Vel;

   set_v3((vect_3*)force,
          add_v3((vect_3*)force,
                 scale_v3(WallStiff + WallDamp,(vect_3*)norm)));
}

/** Initialize a haptic wall effect.
see #bteffect_bulletproofwall
*/
void init_bulletproofwall(bteffect_bulletproofwall *wall,btreal Boffset,btreal K2, btreal K2offset, btreal K1, btreal Bin, btreal Bout)
{
   wall->K1 = K1;
   wall->K2 = K2;
   wall->K2offset = K2offset;
   wall->Bin = Bin;
   wall->Bout = Bout;
   wall->Boffset = Boffset;
}

/** Calculate wall interaction forces
This is a #bthaptic_object_struct virtual interface implimentation.
*/
int bulletproofwall_nf(struct bthaptic_object_struct *obj, btreal depth, vect_n *norm, vect_n *vel, vect_n *acc, vect_n *force)
{
   btreal WallStiff,WallDamp,Vel;
   bteffect_bulletproofwall *norm_eff;
   
   WallStiff = 0.0;
   WallDamp = 0.0;

   norm_eff = (bteffect_bulletproofwall*)obj->norm_eff;
   Vel = dot_v3((vect_3*)norm,(vect_3*)vel);

   switch(brake){
      case 0: case 1:
      if(depth < setBrake)
         brake = 2;
      break;
      case 2: case 3:
      if(depth > setBrake)
         brake = 0;
      if(depth < relBrake)
         brake = 4;
      break;
      case 4: case 5:
      if(depth > setBrake)
         brake = 0;
      break;
   }
   
   if (depth < 0.0)
   {
      WallStiff = -1.0*norm_eff->K1*depth;
      if (depth < -1.0*norm_eff->K2offset)
         WallStiff += -1.0*norm_eff->K2*(depth+norm_eff->K2offset);
   }
   else
      audio = 1;

   if (depth - norm_eff->Boffset < 0.0)
   {
      if(Vel < 0.0){
         WallDamp = -1.0*norm_eff->Bin*Vel;
         // Play sound
         if(audio == 1){
            audio = 2;
            extVel = Vel;
         }
      }
      else if (Vel > 0.0)
         WallDamp = -1.0*norm_eff->Bout*Vel;
   }
   set_v3((vect_3*)force,
          add_v3((vect_3*)force,
                 scale_v3(WallStiff + WallDamp,(vect_3*)norm)));
}

/** Initialize a haptic wall effect.
see #bteffect_wickedwall
*/
void init_wickedwall(bteffect_wickedwall *wall,btreal K1, btreal Bin,btreal Bout,btreal Thk,btreal Boffset)
{
   wall->state = 1;
   wall->K1 = K1;
   wall->Bin = Bin;
   wall->Bout = Bout;
   wall->Thk = Thk;
   wall->Boffset = Boffset;
}

/** Calculate wall interaction forces
This is a #bthaptic_object_struct virtual interface implimentation.
*/
int wickedwall_nf(struct bthaptic_object_struct *obj, btreal depth, vect_n *norm, vect_n *vel, vect_n *acc, vect_n *force)
{
   btreal WallStiff,WallDamp,Vel;
   bteffect_wickedwall *norm_eff;


   WallStiff = 0.0;
   WallDamp = 0.0;

   norm_eff = (bteffect_wickedwall*)obj->norm_eff;
   Vel = dot_v3((vect_3*)norm,(vect_3*)vel);

   if (norm_eff->state == 0)
   {
      //outside
      if (depth < 0.0)
         norm_eff->state = 1;
   }
   if (norm_eff->state == 1)
   {
      if (depth < -1.0*norm_eff->Thk)
         norm_eff->state = 2;
      else if (depth > 0.0)
         norm_eff->state = 0;
      else {
         WallStiff = -1.0*norm_eff->K1*depth;
      }
   }
   if (norm_eff->state == 2)
   {
      if (depth > 0.0)
         norm_eff->state = 0;
   }

   if ((depth - norm_eff->Boffset < 0.0) && (norm_eff->state != 2))
   {
      if(Vel < 0.0)
         WallDamp = -1.0*norm_eff->Bin*Vel;
      else if (Vel > 0.0)
         WallDamp = -1.0*norm_eff->Bout*Vel;
   }
   set_v3((vect_3*)force,
          add_v3((vect_3*)force,
                 scale_v3(WallStiff + WallDamp,(vect_3*)norm)));
}

/** Initialize a haptic wall effect.
see #bteffect_magneticwall
*/
void init_magneticwall(bteffect_magneticwall *wall,btreal Boffset,btreal K2, btreal K2offset, btreal K1, btreal Bin, btreal Bout)
{
   wall->K1 = K1;
   wall->K2 = K2;
   wall->K2offset = K2offset;
   wall->Bin = Bin;
   wall->Bout = Bout;
   wall->Boffset = Boffset;
}

/** Calculate wall interaction forces
This is a #bthaptic_object_struct virtual interface implimentation.
*/
int magneticwall_nf(struct bthaptic_object_struct *obj, btreal depth, vect_n *norm, vect_n *vel, vect_n *acc, vect_n *force)
{
   btreal WallStiff,WallDamp,Vel;
   bteffect_magneticwall *norm_eff;

   // Variables that can be adjusted:
   // magforce: the force of the magnet, the greater
   //           this value is, the larger the attraction.
   //           typical range is between 0 and 10.
   // oprange: operation range (m), the distance from the
   //           surface of the magnetic object in which
   //           there is magnetism. At any distance
   //           greater than oprange, there is no
   //           magnetic force on the arm whatsoever.


   double magforce = 0.1;
   double oprange = 0.5;
   double threshold = 0.01;
   double yval = -1.0*norm_eff->K1*threshold;
   double xval = sqrt(magforce/(-1.0*yval));
   double shift = threshold - xval;

   WallStiff = 0.0;
   WallDamp = 0.0;

   norm_eff = (bteffect_magneticwall*)obj->norm_eff;
   Vel = dot_v3((vect_3*)norm,(vect_3*)vel);




   if (depth > threshold)
   {
      if (depth < oprange)
         WallStiff = -magforce/((depth - shift)*(depth - shift));
      else
         WallStiff = 0;
   }

   if (depth < threshold)
   {
      WallStiff = -1.0*norm_eff->K1*depth;
      if (depth < -1.0*norm_eff->K2offset)
         WallStiff += -1.0*norm_eff->K2*(depth+norm_eff->K2offset);
   }

   if (depth - norm_eff->Boffset < 0.0)
   {
      if(Vel < 0.0)
         WallDamp = -1.0*norm_eff->Bin*Vel;
      else if (Vel > 0.0)
         WallDamp = -1.0*norm_eff->Bout*Vel;
   }
   set_v3((vect_3*)force,
          add_v3((vect_3*)force,
                 scale_v3(WallStiff + WallDamp,(vect_3*)norm)));
}
