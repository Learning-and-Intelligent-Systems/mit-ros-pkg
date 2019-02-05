/* ======================================================================== *
 *  Module ............. experimental
 *  File ............... cseg.c
 *  Creation Date ...... 2005
 *  Author ............. Traveler Hauptman
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

/** Circular segments */
#include "btmath.h"
#include <stdio.h>

typedef struct {
  //Defining info
  btreal rho;
  vect_n *a,*b,*c; //3 point define path. b is the "center" point.
  
  //Internal Representation
  vect_n *C; //Center
  vect_n* A; //Diff twixt vel vectors
  vect_n *W; //Axis of accelleration
  vect_n *x0,*xf,*x0p,*xfp; //initial and final points
 // btreal s; //Arc length
  btreal theta,phi; //Half angle between line segments and comliment
  btreal ptlen,hlen;
  btreal beta;
  
  //Calculation
  vect_n* last_result;
  btreal s;  //arclen from start of arc we are interested in
  btreal omega; //Angle this represents
  btreal w; //Dist toward final velocity vector xfp
  
  
  
}cseg;
typedef struct{
  btreal xf,xc;
  btreal tf,vf;
  btreal acc;
  //state
  btreal t;
}tseg;

btreal calc_tseg(tseg *s,btreal t)
{
  btreal ret,rem;
  rem = t - s->tf;
  return s->xf + s->vf * rem + 0.5* s->acc * rem * rem;
}

const btreal pi = 3.14159265359;

void dump_cseg(cseg *csg,FILE *p)
{
  char buf[500];
  fprintf(p,"rho: %f\n",csg->rho);
  fprintf(p,"a: %s\n",sprint_csv_vn(buf,csg->a));
  fprintf(p,"b: %s\n",sprint_csv_vn(buf,csg->b));
  fprintf(p,"c: %s\n",sprint_csv_vn(buf,csg->c));
  
    fprintf(p,"A: %s\n",sprint_csv_vn(buf,csg->A));
  fprintf(p,"C: %s\n",sprint_csv_vn(buf,csg->C));
  fprintf(p,"x0: %s\n",sprint_csv_vn(buf,csg->x0));
  fprintf(p,"xf: %s\n",sprint_csv_vn(buf,csg->xf));
  fprintf(p,"cheta: %f\n",csg->theta);
  fprintf(p,"phi: %f\n",csg->phi);
  fprintf(p,"ptlen: %f\n",csg->ptlen);
  fprintf(p,"hlen: %f\n",csg->hlen);  
  fprintf(p,"beta: %f\n",csg->beta);

  
}
/** 

Handles inner (theta) angles of pi. Explodes on angles of 0.0
*/
btreal init_cseg(cseg *csg,btreal rho, vect_n *a, vect_n *b, vect_n *c)
{
  int vsz;
  local_vn(tmpa,10);
  local_vn(tmpb,10);
  
  btreal div,churl;
  
  vsz = len_vn(a);
  init_vn(tmpa,vsz);
  init_vn(tmpb,vsz);
  csg->a = new_vn(vsz);
  csg->b = new_vn(vsz);
  csg->c = new_vn(vsz);
  csg->C = new_vn(vsz);
  csg->W = new_vn(vsz);
  csg->A = new_vn(vsz);
  csg->x0 = new_vn(vsz);
  csg->x0p = new_vn(vsz);
  csg->xf = new_vn(vsz);
  csg->xfp = new_vn(vsz);
  csg->last_result = new_vn(vsz);
  
  csg->rho = rho;
  set_vn(csg->a,a);
  set_vn(csg->b,b);
  set_vn(csg->c,c);
  set_vn(tmpa,unit_vn(sub_vn(a,b)));
  set_vn(csg->x0p,neg_vn(tmpa));
  set_vn(tmpb,unit_vn(sub_vn(c,b)));
  set_vn(csg->xfp,tmpb);
  printf("\n");
  print_vn(tmpa);printf("\n");
  print_vn(tmpb);printf("\n");
  
  csg->theta = angle_vn(tmpa,tmpb)/2;  //Half the angle between the two line segments
  csg->phi = pi/2.0 - csg->theta; //The other angle in the triangle (TH #7, pp12)
  
  div = sin(csg->theta)/cos(csg->theta);

  csg->ptlen = csg->rho/div;
  csg->hlen = sqrt(csg->rho*csg->rho + csg->ptlen*csg->ptlen);
  set_vn(csg->x0,add_vn(csg->b,scale_vn(csg->ptlen,tmpa)));
  set_vn(csg->xf,add_vn(csg->b,scale_vn(csg->ptlen,tmpb)));
  set_vn(csg->C,add_vn(csg->b,scale_vn(csg->hlen,unit_vn(add_vn(scale_vn(0.5,sub_vn(tmpb,tmpa)),tmpa)))));
  set_vn(csg->A,unit_vn(sub_vn(csg->xf,csg->x0)));
  csg->beta = pi/2 - angle_vn(csg->x0p,csg->A);
  return csg->phi*csg->rho*2.0;
}
/** See TH#7 pp 15 for a sketch */
vect_n* calc_cseg(cseg *csg,btreal s) //Given an arclength find point
{
  btreal R;
  
  // s = rho*omega
  csg->omega = s/csg->rho;
  R = pi - (csg->beta + csg->omega);
  csg->w = csg->rho*sin(csg->omega)/sin(R);
  set_vn(csg->last_result,add_vn(csg->C,scale_vn(csg->rho,unit_vn(sub_vn(add_vn(csg->x0,scale_vn(csg->w,csg->A)),csg->C)))));
  //printf("omega:%f  R:%f  w:%f\n",csg->omega,R,csg->w);
  
  return csg->last_result;
}

void main()
{
  tseg ms;
  vect_n *a,*b,*c,*res;
  FILE *out;
  char buf[250];
  cseg t;
  int cnt;
  btreal s,ts;
  
  ms.vf = 100;
  ms.xf = 1000;
  ms.tf = 10;
  ms.acc = 10;
  
  
  a = new_vn(4);
  b = new_vn(4);
  c = new_vn(4);
  res = new_vn(4);
  
  const_vn(a,0.0,0.0,0.0,0.0);
  const_vn(b,0.0,2.0,0.0,0.0);
  const_vn(c,0.0,0.0,0.0,0.0);
  
  ts = init_cseg(&t,0.01,a,b,c);
  dump_cseg(&t,stdout);
  
  printf("length: %f ts:%f \n",t.phi);
  out = fopen("curve.csv","w");
  ts = 10;
  s = 0.0;
  printf("ds: %f\n",ts/50);
  //fprintf(out,"%s\n",sprint_csv_vn(buf,t.x0));
  //fprintf(out,"%s\n",sprint_csv_vn(buf,b));
  //fprintf(out,"%s\n",sprint_csv_vn(buf,t.xf));
  for(cnt = 0;cnt < 51;cnt++){
    
    //set_vn(res,calc_cseg(&t,s));
    s+=ts/50;
    //fprintf(out,"%s\n",sprint_csv_vn(buf,res));
    fprintf(out,"%f, %f\n",s,calc_tseg(&ms,s));
  }
  fclose(out);
  
}
