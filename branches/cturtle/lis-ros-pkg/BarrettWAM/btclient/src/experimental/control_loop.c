/* ======================================================================== *
 *  Module ............. experimental
 *  File ............... control_loop.c
 *  Creation Date ...... 23 Nov 2002
 *  Author ............. Traveler Hauptman
 *  Addtl Authors ...... Brian Zenowich
 *                       Sam Clanton
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  Copyright (C) 2002-2008 Barrett Technology, Inc. <support@barrett.com>
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

/*! \file control_loop.c
    \brief Infrastructure to simplify adding and using your own control threads.
    
    This module adds code to start up timing threads that will call a user specified 
  control thread at regular intervals.
    
   
*/





/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <syslog.h>
#include <stddef.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <semaphore.h>
#include <malloc.h>
//th041216#include <process.h>
#include <time.h>
#include <inttypes.h>
//th041216#include <sys/sched.h>
#include <sys/mman.h>
//th041216#include <sys/neutrino.h>
//th041216#include <sys/netmgr.h>
//th041216#include <sys/syspage.h>
#ifdef USE_RTAI31
#include <rtai_lxrt.h>
#endif
#ifdef USE_FUSION
#include <rtai/task.h>
#include <rtai/timer.h>
#endif

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#include "control_loop.h"
//#include "btcan.h"
//#include "btsystem.h"
//th041216#include "traj.h"

sem_t     timer_semaphore;
pthread_t timer_thd_id;
pthread_t control_thd_id;
int shutdown_threads=1;
uint64_t ms_timer,sem_cntr;
double Sample_Period; //RTAI
RTIME sample_period2;
/** Creates a timing thread and links it to your control loop function 

  \param priority The priority to run the timer and control loop threads at.
  \param sample_period An double with units of ms

*/
void start_control_threads(int priority, double sample_period, void *function,void *args)
{
  double tmp;
  pthread_attr_t       control_attr;
  struct sched_param   control_param;
  RTIME sampleCount;
  
  Sample_Period = sample_period;
  tmp = sample_period * 1000000000.0;
  sample_period2 = (RTIME)tmp;
  sampleCount = nano2count(sample_period2);

  rt_set_periodic_mode(); /* for clarity */

    /* start control_thread - the thread which controls the robot */
  pthread_attr_init(&control_attr);
  pthread_attr_setschedpolicy(&control_attr, SCHED_FIFO);
  pthread_attr_getschedparam(&control_attr, &control_param);
  control_param.sched_priority = priority;
  pthread_attr_setschedparam(&control_attr, &control_param);
  
  shutdown_threads = 0;
  
  pthread_create(&control_thd_id, &control_attr, function, args);

  if (control_thd_id == -1)
  {
    syslog(LOG_ERR,"start_linux_control_thread:Couldn't start control thread!");
    exit(-1);
  }

  start_rt_timer(sampleCount);
}
/** Stops the control threads

*/
void stop_control_threads()
{
  
 shutdown_threads = 1;
 usleep(200000);

}
