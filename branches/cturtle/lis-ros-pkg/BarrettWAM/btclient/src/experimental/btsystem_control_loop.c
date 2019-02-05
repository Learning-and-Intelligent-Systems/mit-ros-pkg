/* ======================================================================== *
 *  Module ............. experimental
 *  File ............... btsystem_control_loop.c
 *  Creation Date ...... 16 Mar 2003
 *  Author ............. Traveler Hauptman
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  Copyright (C) 2003-2008 Barrett Technology, Inc. <support@barrett.com>
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

#include <semaphore.h>

#include "btsystem.h"
#include "btjointcontrol.h"
#include "btsystem_control_loop.h"

extern sem_t timer_semaphore;
extern int shutdown_threads;

/** A canned controller for a set of pucks

*/
void BTsystemControlThread(void *data)
{
  int cnt;
  double sample_period;
  SimpleCtl *sc;
  actuator_struct *act;
  int num_actuators;

  sample_period = ((SCcontrol_thd_parms *)data)->sample_period;
  sc = ((SCcontrol_thd_parms *)data)->sc;
  act = ((SCcontrol_thd_parms *)data)->act;
  num_actuators = ((SCcontrol_thd_parms *)data)->num_act;

  while (!shutdown_threads)
  {
    sem_wait(&timer_semaphore);

    GetPositions(); //    getpos
    for (cnt = 0; cnt < num_actuators; cnt++) //    calc PID
    {
      act[cnt].torque = SCevaluate(&(sc[cnt]), act[cnt].angle, sample_period);
      
    }
    SetTorques(); //    set torques
  }
  pthread_exit(NULL);
}
