/* ======================================================================== *
 *  Module ............. experimental
 *  File ............... btmathtest.c
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

/** \file btmathtest.c
\brief btsystem internal testing template

*/
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>
#include <time.h>
#include "btmath.h"



int main( int argc, char **argv )
{
  clock_t start,end;
  double time_used,c;

  long int cnt;
  char buff[355];
  vect_n *t1,*t2,*t3;
  vect_n *q,*dq,*ddq;

   
  openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
  q = new_vn(6);
  dq = new_vn(6);
  ddq = new_vn(6);
  fill_vn(ddq,1.0);
  fill_vn(dq,1.0);
  start = clock();
  for (cnt = 0;cnt < 1000000L; cnt ++){
    set_vn(q,add_vn(q,add_vn(dq,scale_vn(2.0,ddq))));
  }
  end = clock();
  time_used = ((double) (end - start))/CLOCKS_PER_SEC;
  printf("\n");
  print_vn(q);
  printf(" : Time:%f\n",time_used);
}





