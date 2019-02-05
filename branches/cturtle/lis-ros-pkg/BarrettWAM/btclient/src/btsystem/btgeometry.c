/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... btgeometry.c
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
#include "btmath.h"
#include "btgeometry.h"
#include <syslog.h>

/** Initialize the state tracking object
 
\internal Mixed initialization and allocation. fix me per conventions in 
btos.h
*/
void init_state_btg(btgeom_state *bts,btreal samplerate,btreal cutoffHz)
{
   bts->pos = new_v3();
   bts->vel = new_v3();
   bts->acc = new_v3();
   bts->velfilt = new_btfilter_vn(5,3);
   bts->accfilt = new_btfilter_vn(5,3);
   init_btfilter_vn_diff(bts->velfilt,2,samplerate,cutoffHz);
   init_btfilter_vn_diff(bts->accfilt,2,samplerate,cutoffHz);
}

/** Update the state object using the given position.
 
*/
void eval_state_btg(btgeom_state *bts,vect_3* pos)
{
   set_v3(bts->pos,pos);
   set_v3(bts->vel,(vect_3*)eval_btfilter_vn(bts->velfilt, (vect_n*)bts->pos));
   set_v3(bts->acc,(vect_3*)eval_btfilter_vn(bts->accfilt, (vect_n*)bts->vel));
}

/** Define a plane from 3 points
 
  The normal of the plane is defined by the right-hand rule. ie if you draw 3
  (non colinear) points on a page, the normal will point out of the page if they 
  are ordered 1-3 counter-clockwise.
*/
int init_pl_btg( btgeom_plane *plane, vect_3 *pt1, vect_3 *pt2, vect_3 *pt3)
{
   vect_3 * rel2;
   vect_3 * rel3;

   rel2 = new_v3();
   rel3 = new_v3();
   plane->normal = new_v3();

   set_v3(rel2,sub_v3(pt2,pt1));
   set_v3(rel3,sub_v3(pt3,pt2));

   set_v3(plane->normal,unit_v3(cross_v3(rel2,rel3)));
   plane->distance = dot_v3(plane->normal, pt1);

}

/** Distance & Normal between a point and a plane. */
btreal D_Pt2Pl(vect_3 *norm,btgeom_plane *plane, vect_3 *point)
{
   btreal Dist;
   Dist = dot_v3(plane->normal,point) - plane->distance;
   set_v3(norm,plane->normal); //dist from point to plane surface
   return Dist;
}

/** Create a sphere object
 
\param pt1 Center of sphere.
\param pt2 Edge of sphere.
\param inside 0 = Box is solid inside. 1 = Box is hollow.
*/
int init_sp_btg( btgeom_sphere *sphere, vect_3 *pt1, vect_3 *pt2,int inside)
{

   sphere->center = new_v3();

   set_v3(sphere->center,pt1);
   sphere->radius = norm_v3(sub_v3(pt1,pt2));
   sphere->inside = inside;
}

/** Distance & Normal between a sphere and a point. */
btreal D_Pt2Sp(vect_3 *norm,btgeom_sphere *sp, vect_3 *pt)
{
   btreal Dist;
   btreal tmp1,tmp2;

   set_v3(norm,sub_v3(pt,sp->center)); //dist from point to plane surface
   Dist = norm_v3(norm) - sp->radius;
   set_v3(norm,unit_v3(norm));

   if (sp->inside) {
      Dist = -1.0 * Dist;
      set_v3(norm,neg_v3(norm));
   }

   return Dist;
}

/** Distance between two points */
btreal D_Pt2Pt(vect_3 *pt1, vect_3 *pt2)
{
   btreal pt1x = getval_vn((vect_n*)pt1, 0);
   btreal pt1y = getval_vn((vect_n*)pt1, 1);
   btreal pt1z = getval_vn((vect_n*)pt1, 2);

   btreal pt2x = getval_vn((vect_n*)pt2, 0);
   btreal pt2y = getval_vn((vect_n*)pt2, 1);
   btreal pt2z = getval_vn((vect_n*)pt2, 2);

   btreal xd = pt1x - pt2x;
   btreal yd = pt1y - pt2y;
   btreal zd = pt1z - pt2z;

   btreal distance = sqrt(xd*xd + yd*yd + zd*zd);

   return distance;

}

/** Create a box object.
 
Normals will be pointing outward.
 
The plane that contains points 1, 2, &3 is the reference side. The normal 
of the plane determines the matching sidewall (thk away).
The vector from point 1 to 2 is the normal for the next two sidewalls. They 
will be placed distance dir1/2 away from pt1.
The vector in the plane, normal to the 1-2 line segment, with point 3 as it's end
determines the final direction. The walls will be dir2/2 from pt1.
 
\param pt1 Defines the side of the box. Also the center point from which box dimensions are measured.
\param pt2 The vector from pt1 to pt2 in the normal (and perpindicular) of the sidewalls.
\param pt3 Third point in the plane definition.
\param thk Distance from pt1, normal to the first plane, that the second wall is placed.
\param dir1 Distance/2 to place walls 3,4.
\param dir2 Distance/2 to place walls 5,6.
\param inside 0 = Box is solid inside. 1 = Box is hollow.
*/
int init_bx_btg( btgeom_box *box,vect_3 *pt1, vect_3 *pt2, vect_3 *pt3,btreal thk,btreal dir1,btreal dir2,int inside)
{
   vect_3* swap;
   staticv3 sp[10];
   vect_3* tp[10];
   int cnt;

   for(cnt = 0;cnt < 10;cnt ++)
      tp[cnt] = init_staticv3(&sp[cnt]);

   init_pl_btg(&box->side[0],pt1,pt2,pt3); //Plane0
   //Mirror to other side
   set_v3(tp[0],add_v3(pt1,scale_v3(-1.0*thk,box->side[0].normal)));
   set_v3(tp[1],add_v3(pt2,scale_v3(-1.0*thk,box->side[0].normal)));
   set_v3(tp[2],add_v3(pt3,scale_v3(-1.0*thk,box->side[0].normal)));

   init_pl_btg(&box->side[1],tp[2],tp[1],tp[0]); //Plane1

   //Create section plane 1 for translation
   init_pl_btg(&box->side[2],pt1,pt2,tp[0]);

   //Translate in positive direction
   set_v3(tp[3],add_v3(pt1,scale_v3(0.5*dir2,box->side[2].normal)));
   set_v3(tp[4],add_v3(pt2,scale_v3(0.5*dir2,box->side[2].normal)));
   set_v3(tp[5],add_v3(tp[0],scale_v3(0.5*dir2,box->side[2].normal)));

   init_pl_btg(&box->side[2],tp[3],tp[4],tp[5]);  //Plane2

   //Translate in negative direction
   set_v3(tp[3],add_v3(pt1,scale_v3(-0.5*dir2,box->side[2].normal)));
   set_v3(tp[4],add_v3(pt2,scale_v3(-0.5*dir2,box->side[2].normal)));
   set_v3(tp[5],add_v3(tp[0],scale_v3(-0.5*dir2,box->side[2].normal)));

   init_pl_btg(&box->side[3],tp[5],tp[4],tp[3]); //Plane3
   //---
   //Calculate point = vect normal to p1,p2 line added to p1
   set_v3(tp[6],unit_v3(sub_v3(pt2,pt1)));
   set_v3(tp[7],sub_v3(pt3,scale_v3(-1.0*dot_v3(sub_v3(pt3,pt1),tp[6]),tp[6])));

   //Create section plane 2 for translation
   init_pl_btg(&box->side[4],pt1,tp[7],tp[0]);

   //Translate in positive direction
   set_v3(tp[3],add_v3(pt1,scale_v3(0.5*dir1,box->side[4].normal)));
   set_v3(tp[4],add_v3(tp[7],scale_v3(0.5*dir1,box->side[4].normal)));
   set_v3(tp[5],add_v3(tp[0],scale_v3(0.5*dir1,box->side[4].normal)));

   init_pl_btg(&box->side[4],tp[3],tp[4],tp[5]); //Plane4
   //Translate in negative direction
   set_v3(tp[3],add_v3(pt1,scale_v3(-0.5*dir1,box->side[4].normal)));
   set_v3(tp[4],add_v3(tp[7],scale_v3(-0.5*dir1,box->side[4].normal)));
   set_v3(tp[5],add_v3(tp[0],scale_v3(-0.5*dir1,box->side[4].normal)));

   init_pl_btg(&box->side[5],tp[5],tp[4],tp[3]); //Plane5
   box->inside = inside;
   if (inside) {
      //if inside, flip normals and distances
      for (cnt = 0; cnt < 6; cnt ++) {
         set_v3(box->side[cnt].normal,neg_v3(box->side[cnt].normal));
         box->side[cnt].distance *= -1.0;
      }
   }

}

/** Distance between a point and a box object */
btreal D_Pt2Bx(vect_3 *norm,btgeom_box *bx, vect_3 *pt)
{
   btreal Dist[6],Dmax,Dmin,Dret;
   staticv3 sp[7];
   vect_3* tp[7];
   int cnt,idx = 0,closest_in = -1,closest_out = -1,active_sides = 0;

   for(cnt = 0;cnt < 7;cnt ++)
      tp[cnt] = init_staticv3(&sp[cnt]);
   Dmax = -10.0e100;
   Dmin = 10.0e100;

   for (cnt = 0; cnt < 6; cnt ++) {
      Dist[cnt] = D_Pt2Pl(tp[cnt],&bx->side[cnt],pt);


      if ((Dist[cnt] < 0.0) ) {
         if (!bx->inside)
            active_sides++;
         if (Dist[cnt] > Dmax) {
            Dmax = Dist[cnt];
            closest_in = cnt;
         }
      } else if ((Dist[cnt] > 0.0))  {
         if (bx->inside)
            active_sides++;
         if(Dist[cnt] < Dmin) {
            Dmin =  Dist[cnt];
            closest_out = cnt;
         }
      }
   }

   if ((!bx->inside) && (active_sides == 6)) {
      set_v3(norm,tp[closest_in]);
      return Dist[closest_in];
   } else if ((!bx->inside)) {
      set_v3(norm,tp[closest_out]);
      return Dist[closest_out];
   } else if ((bx->inside) && (active_sides == 6)) {
      set_v3(norm,tp[closest_out]);
      return Dist[closest_out];
   } else { //if ((bx->inside)){
      fill_v3(norm,0.0);
      Dret = 0.0;
      for (cnt = 0; cnt < 6; cnt ++) {
         if (Dist[cnt] < 0.0)
            set_v3(norm,add_v3(norm,scale_v3(-Dist[cnt],tp[cnt])));
      }
      Dret = -norm_v3(norm);
      set_v3(norm,unit_v3(norm));
      return Dret;
   }

}

/** Initialize a btgeom_lineseg object.
Allocate memory for the internal vectors.
*/
void init_Seg_btg(btgeom_lineseg *seg,int size)
{
   seg->start = new_vn(size);
   seg->end = new_vn(size);
   seg->unit = new_vn(size);
}

/** Define a line segment from 2 points.
p1, p2 are end points of a line segment.
*/
void set_Seg_btg(btgeom_lineseg *seg,vect_n *p1,vect_n *p2)
{
   if (p1 != NULL)
      set_vn(seg->start,p1);
   if (p2 != NULL)
      set_vn(seg->end,p2);
   set_vn(seg->unit,unit_vn(sub_vn(seg->end,seg->start)));
}

/** Distance between a point and line
*/
btreal D_Ln2Pt(btgeom_lineseg *seg,vect_n *pt)
{
   vect_n *ul,*np;

   np = sub_vn(pt,seg->start);
   return norm_vn(sub_vn(np,scale_vn(dot_vn(np,seg->unit),seg->unit)));
}
