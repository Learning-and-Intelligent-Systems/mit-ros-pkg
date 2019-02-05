/* ======================================================================== *
 *  Module ............. experimental
 *  File ............... trj1.c
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

/** time-point list

A sequence of times and points are interpolated linearly

*/

/** Time - Point list trajectory info

For a time - point list the relationship between time and arc-length is 1 to 1.
So all we do is maintain the elapsed arc-length;

*/
/*
typedef struct {
  btreal s;
}tpl_trj;

tpl_trj * new_tpl_trj()
{
   tpl_trj *bttrj;
  if ((bttrj = malloc(sizeof(tpl_trj))) == NULL)
  {
    syslog(LOG_ERR,"new_tpl_trj: memory allocation failed");
    return NULL;
  }
  return bttrj;
  
}*/
btreal tpl_init_T(void *dat,btreal t)
{
  tpl_trj *trj;
  trj = (tpl_trj*)dat;
  trj->s = t;
  return t;
}
btreal tpl_S_of_dt(void *dat,btreal dt)
{
  tpl_trj *trj;
  trj = (tpl_trj*)dat;
  trj->s += dt;
  return trj->s;
}
vect_n* tpl_init_S(void *dat,btreal s)
{
  btpath_pwl *pth;
  pth = (btpath_pwl*)dat;

  return dsinit_pwl(pth,s);
}

vect_n* tpl_Q_of_ds(void *dat,btreal ds)
{
  btpath_pwl *pth;
  pth = (btpath_pwl*)dat;
  
  return ds_pwl(pth,ds);
}

bttrajectory* tpl_load_n_register(char *filename)
{
  bttrajectory* trj;
  tpl_trj *crv_trj;
  btpath_pwl *crv;
  vectray *tmpray;
  
  trj = new_bttrajectory();
  //crv_trj = new_tpl_trj();

  read_csv_file_vr(fileName, &tmpray);
  new_param_by_arclen_pwl(btpath_pwl *pth, btpath_pwl *crv2)
  crv = new_pwl(tmpray->n-1,5);
  add_vectray_pwl(crv,tmpray);
  
  settraj_bttrj(trj,(void *)crv_trj, tpl_init_T, tpl_S_of_dt);
  setpath_bttrj(trj,(void *)crv, dsinit_pwl,ds_pwl);
  return trj;
}

bttrajectory* tpl_unload(bttrajectory *trj)
{
  free_pwl((btpath_pwl *)trj->crv);
}
