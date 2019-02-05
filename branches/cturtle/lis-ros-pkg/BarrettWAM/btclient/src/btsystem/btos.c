/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... btos.c
 *  Creation Date ...... 28 Mar 2005
 *  Author ............. Traveler Hauptman
 *  Addtl Authors ...... Lauren White
 *                       Brian Zenowich
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
#include <syslog.h>
#include <stdlib.h>
#include "btos.h"

/*==============================*
 * GLOBAL file-scope variables  *
 *==============================*/
void *backtracearray[50];
char **backtracestrings;
int backtracesize;

/*==============================*
 * Internal use Functions       *
 *==============================*/
void syslog_backtrace(int size)
{
   int cnt;
   for (cnt = 0;cnt < size;cnt ++)
      syslog(LOG_ERR,"WAM:Backtrace:%s",backtracestrings[cnt]);
}

/*==============================*
 * Functions                    *
 *==============================*/

#if 1
/** Initialize a mutex.
 
If pthread_mutex_init() fails, an error message is printed to syslog.
 
\return Result of pthread_mutex_init().
\exception Undefined if btm does not point to memory block meant for a btmutex.
\internal chk'd TH 051101
\todo Error checking mutexes enabled by compiler switch.
*/
int btmutex_init(btmutex* btm)
{
   int ret;
   pthread_mutexattr_t mattr;
   pthread_mutexattr_init(&mattr);
   pthread_mutexattr_settype(&mattr,PTHREAD_MUTEX_TIMED_NP);
   //pthread_mutexattr_settype(&mattr,PTHREAD_MUTEX_ERRORCHECK_NP);

   ret = test_and_log(
            pthread_mutex_init(btm,&mattr),
            "Could not initialize mutex.");
   return ret;
}


/** Lock a btmutex.
See pthread_mutex_lock() in pthread.h for more info.
This function calls pthread_mutex_lock() and prints an error to syslog if it 
fails.
\return Result of pthread_mutex_lock().
\exception Undefined if btm does not point to an initialized btmutex object.
\internal chk'd TH 051101
*/
BTINLINE int btmutex_lock(btmutex* btm)
{
   int ret;
   ret = pthread_mutex_lock(btm);
   if (ret != 0)
   {
      syslog(LOG_ERR, "Mutex lock failed (REG): %d", ret);
   }
   return ret;
}


/** Unlock a btmutex.
See pthread_mutex_unlock() in pthread.h for more info.
This function calls pthread_mutex_unlock() and prints an error to syslog if it 
fails.
\return Result of pthread_mutex_unlock().
\exception Undefined if btm does not point to an initialized btmutex object.
\internal chk'd TH 051101
*/
BTINLINE int btmutex_unlock(btmutex *btm)
{
   int ret;
   ret = pthread_mutex_unlock(btm);
   if (ret != 0)
   {
      syslog(LOG_ERR, "Mutex unlock failed (REG): %d",  ret);
   }
   return ret;
}
#endif

/** Check pointer for a NULL value.
\retval 0 Pointer is NOT valid.
\retval 1 Pointer is valid.
 
\exception Undefined if str points to something not a string and ptr is not valid.
\internal chk'd TH 051101
\todo Eventually we should check for pointers outside of program heap instead of
just null pointers. See sbrk() [gnu libc] for finding end of data segment. 
 
*/
int btptr_ok(void *ptr,char *str)
{
   if (ptr == NULL)
   {
#if BT_BACKTRACE & BTDEBUG_BACKTRACE
      backtracesize = baktrace(backtracearray,50);
      backtracestrings = backtrace_symbols(backtracearray,backtracesize);
      syslog_backtrace(backtracesize);
#endif

      syslog(LOG_ERR,"bt ERROR: you tried to access a null pointer in %s",str);
      return 0;
   }
   return 1;
}

/** Pointer out of range check.
 
Presently only checks for NULL.
Has same (backwards) return values as btptr_ok().
\retval 0 Pointer is NOT valid.
\retval 1 Pointer is valid.
 
\exception Undefined if str points to something not a string and ptr is not valid.
\internal chk'd TH 051101
*/
int btptr_chk(void *ptr)
{
   if (ptr == NULL)
   {
#if BT_BACKTRACE & BTDEBUG_BACKTRACE
      backtracesize = baktrace(backtracearray,50);
      backtracestrings = backtrace_symbols(backtracearray,backtracesize);
      syslog_backtrace(backtracesize);
#endif

      return 0;
   }
   return 1;
}

/** Prints an error if array index is out of range.
 
\retval 0 Array index is NOT valid.
\retval 1 Array index is valid.
 
\exception Undefined if str points to something not a string and index is not valid.
\internal chk'd TH 051101
 
*/
int idx_bounds_ok(int idx,int max,char *str)
{
   if ((idx < 0) || (idx > max))
   {
#if BT_BACKTRACE & BTDEBUG_BACKTRACE
      backtracesize = baktrace(backtracearray,50);
      backtracestrings = backtrace_symbols(backtracearray,backtracesize);
      syslog_backtrace(backtracesize);
#endif

      syslog(LOG_ERR,"bt ERROR: Your index is %d with max %d in function %s",idx,max,str);
      return 0;
   }
   return 1;
}

/** Provides a shorthand to replace return variable checks.
\internal chk'd TH 051101
*/
BTINLINE int test_and_log(int return_val, const char *str)
{
   if (return_val != 0)
   {
      syslog(LOG_ERR, "%s: %d", str, return_val);
      return return_val;
   }
   else
      return 0;
}

/**Memory allocation wrapper.
 
\return Pointer to allocated memory.
\exception If malloc returns NULL there is no memory left and we kill the process!
\internal chk'd TH 051101
 
*/
BTINLINE void * btmalloc(size_t size)
{
   void* vmem;
   if ((vmem = malloc(size)) == NULL)
   {
      syslog(LOG_ERR,"btMalloc: memory allocation failed, size %d",size);
      exit(-1);
   }
   return vmem;
}

/**Memory deallocation wrapper.
  Free's memory at *ptr and then sets *ptr to NULL.
\exception If *ptr points to a block of memory that was not allocated with btmalloc[malloc]
or if that block was previosly freed the results are undefined.
\internal chk'd TH 051101
*/
BTINLINE void btfree(void **ptr)
{
#ifdef BT_NULL_PTR_GUARD
   if(btptr_ok(*ptr,"btfree"))
#endif

      free(*ptr);
   *ptr = NULL;
}

/** Allocate memory for a btthread object.
\return Pointer to a newly allocated btthread object.
\internal chk'd TH 051101
*/
btthread* new_btthread()
{
   btthread* mem;
   mem = (btthread*)btmalloc(sizeof(btthread));
   return mem;
}

btrt_thread_struct* new_btrt_thread()
{
   btrt_thread_struct* mem;
   mem = (btrt_thread_struct*)btmalloc(sizeof(btrt_thread_struct));
}
/** Free memory for a btthread object.
\internal chk'd TH 051101
*/
void free_btthread(btthread **thd)
{
   btfree((void**)thd);
}

#if 1
/**  Create a new thread.
 
We create a new posix thread with a schedpolicy of SCHED_FIFO. The thread_id
is returned.
 
\param  thd The barrett thread structure; allocated before calling this function.
\param  priority The priority we wish to call this thread with. 0 = linux non-realtime priority. 99 = Max priority
\param  function Pointer to the function that represents the thread.
\param  args Pointer to the arguments you want to pass.
 
\internal chk'd TH 051101 
right now we kill the program if a thread create doesn't work. I'm not sure if this 
is reasonable.
 
*/
pthread_t* btthread_create(btthread *thd,int priority, void *function,void *args)
{
   pthread_attr_init(&(thd->attr));
   pthread_attr_setschedpolicy(&(thd->attr), SCHED_FIFO);
   pthread_attr_getschedparam(&(thd->attr),&(thd->param));
   thd->param.sched_priority = priority;
   pthread_attr_setschedparam(&(thd->attr), &(thd->param));

   thd->done = 0;
   thd->function = function;
   thd->data = args;
   thd->priority = priority;
   thd->periodic = 0;
   btmutex_init(&(thd->mutex));

   pthread_create(&(thd->thd_id), &(thd->attr), function, thd);

   if (thd->thd_id == -1)
   {
      syslog(LOG_ERR,"btthread_create:Couldn't start control thread!");
      exit(-1);
   }
   return &(thd->thd_id);
}

/** See btthread_stop().
\internal chk'd TH 051101 
*/
BTINLINE int btthread_done(btthread *thd)
{
   int done;
   btmutex_lock(&(thd->mutex));
   done = thd->done;
   btmutex_unlock(&(thd->mutex));
   return done;
}
#endif

BTINLINE int btrt_thread_done(btrt_thread_struct *thd)
{
   int done;
   btrt_mutex_lock(&(thd->mutex));
   done = thd->done;
   btrt_mutex_unlock(&(thd->mutex));
   return done;
}

/** Stop a thread that is using btthread_done().
 
This function should only be called from outside the thread you want to stop.
The thread monitors thd->done using btthread_done().
\code
void mythread(void* args)
{
  btthread *mythd;
  mythd = (btthread*)args;
  
  while(!btthread_done(mythd))
  {
    //do something
  }
  pthread_exit(NULL);
}
\endcode
\internal chk'd TH 051101 
*/

#if 1
BTINLINE void btthread_stop(btthread *thd)
{
   btmutex_lock(&(thd->mutex));
   thd->done = 1;
   btmutex_unlock(&(thd->mutex));
   pthread_join(thd->thd_id,NULL);
}



/** Call pthread_exit() on this btthread object.
\internal chk'd TH 051101
*/
BTINLINE void btthread_exit(btthread *thd)
{
   pthread_exit(NULL);
}
#endif

BTINLINE void btrt_thread_stop(btrt_thread_struct *thd)
{
   thd->done = 1;

   btrt_thread_join(thd);
}

BTINLINE void btrt_thread_exit(btrt_thread_struct *thd)
{
#ifdef XEMOMAI
   btrt_set_mode_hard();
   rt_task_delete(&(thd->task));
#else

   pthread_exit(NULL);
#endif
}

#ifdef XEMOMAI
/**
\internal 
\todo
  This function will set up periodic timing and then call the thd->function 
  once a period. It's meant to allow easy periodic threading.
 
\warning Untested!!!
 
*/

//will start a xenomai realtime task
void btperiodic_proto(void *args)
{
   btthread* this_thd;
   RT_TASK ThreadTask;

   double thisperiod;
   RTIME rtime_period;
   RTIME last_loop,loop_start,loop_end;

   this_thd = (btthread*)args;
   thisperiod = this_thd->period;
   rtime_period = (RTIME)(thisperiod * 1000000000.0);

   rt_task_create(&ThreadTask, 0, 0, 0, 0);
   rt_task_set_periodic(&ThreadTask, rtime_period, rtime_period);
   while (!btthread_done(this_thd))
   {
      rt_task_wait_period(NULL);
      loop_start = rt_timer_read(); //th prof
      this_thd->actual_period = loop_start - last_loop; //th prof

      (*this_thd->function)(this_thd->data);

      loop_end = rt_timer_read(); //th prof
      this_thd->proc_time = loop_end - loop_start; //th prof
      last_loop = loop_start; //th prof
   }
   rt_task_delete(&ThreadTask);
   pthread_exit(NULL);
}

/**
\internal 
\todo
  This function will set up periodic timing and then call the thd->function 
  once a period. It's meant to allow easy periodic threading.
 
\warning Untested!!!
 
*/
pthread_t* btperiodic_create(btthread *thd,int priority, double period, void *function,void *args)
{
   thd->period = period;
   thd->function = function;
   return btthread_create(thd,priority,btperiodic_proto,args);

}
#endif


//****************Real Time Calls***************************//

#ifdef XENOMAI
//will start a xenomai realtime task
void btrt_thread_create(btrt_thread_struct *thd, const char *name, int prio, void *function, void *args)
{
   int ret;
   int error;

   thd->done = 0;
   thd->function = function;
   thd->data = args;
   thd->priority = prio;
   thd->periodic = 0;
   btrt_mutex_init(&(thd->mutex));


   //create task
   //btrt_set_mode_hard();
   ret = rt_task_create(&(thd->task), name, 0, prio, T_JOINABLE);
   if(ret)
   {
      syslog(LOG_ERR, "btthread_xenomai_create: Could not create task! %d", ret);
      exit(-1);
   }
   ret = rt_task_start(&(thd->task), function, thd);
   if(ret)
   {
      syslog(LOG_ERR, "btthread_xenomai_create: Could not start task! %d", ret);
      exit(-1);
   }
}
#else
void btrt_thread_create(btrt_thread_struct *thd, const char *name, int prio, void *function, void *args)
{
   strncpy(thd->name,name,4);
   thd->name[4] = '\0';
   btthread_create((btthread*)thd, prio, function, args);
}

#endif


void btrt_set_mode_soft(void)
{
#ifdef XENOMAI
   rt_task_set_mode( T_PRIMARY, 0, NULL);
#else

   rt_make_soft_real_time();
#endif
}

void btrt_set_mode_hard(void)
{
#ifdef XENOMAI
   rt_task_set_mode(0, T_PRIMARY, NULL);
#else

   rt_make_hard_real_time();
#endif
}

#ifdef XENOMAI
void btrt_set_mode_warn(void)
{
   rt_task_set_mode(0, T_WARNSW, NULL);
}
#endif


RTIME btrt_get_time(void)
{
   RTIME time;

#ifdef XENOMAI
   btrt_set_mode_hard();
   time = rt_timer_read(); 
#else
   time = rt_get_cpu_time_ns();
#endif

   return(time);
}

void btrt_task_wait_period(void)
{
#ifdef XENOMAI
   btrt_set_mode_hard();
   rt_task_wait_period(NULL);
#else

   rt_task_wait_period();
#endif
}

void btrt_thread_join(btrt_thread_struct *thd)
{
#ifdef XENOMAI
   btrt_set_mode_hard();
   rt_task_join(&(thd->task));
#else

   pthread_join(thd->thd_id,NULL);
#endif
}

//*********************RT MUTEX*************************


int btrt_mutex_create(btrt_mutex *mutex)//used when RTAI mattr is NULL
{
   int ret;

#ifdef XENOMAI
   btrt_set_mode_hard();
   ret = test_and_log(
            rt_mutex_create(mutex,NULL),
            "Could not initialize mutex.");
#else

   ret = test_and_log(
            pthread_mutex_init(mutex,NULL),
            "Could not initialize mutex.");
#endif

   return ret;
}


int btrt_mutex_init(btrt_mutex* mutex)//make sure a global variable
{
   int ret;
#ifdef XENOMAI
   btrt_set_mode_hard();
   ret = test_and_log(
            rt_mutex_create(mutex,NULL),
            "Could not initialize mutex.");
#else

   pthread_mutexattr_t mattr;
   pthread_mutexattr_init(&mattr);
   pthread_mutexattr_settype(&mattr,PTHREAD_MUTEX_TIMED_NP);
   //pthread_mutexattr_settype(&mattr,PTHREAD_MUTEX_ERRORCHECK_NP);

   ret = test_and_log(
            pthread_mutex_init(mutex,&mattr),
            "Could not initialize mutex.");
#endif

   return ret;
}




BTINLINE int btrt_mutex_lock(btrt_mutex *mutex)
{
   int ret;

#ifdef XENOMAI
   btrt_set_mode_hard();
   ret = rt_mutex_acquire(mutex, TM_INFINITE);
   if (ret != 0)
   {
      syslog(LOG_ERR, "Mutex lock failed (XENO): %d", ret);
   }
#else

   ret = pthread_mutex_lock(mutex);
   if (ret != 0)
   {
      syslog(LOG_ERR, "Mutex lock failed (REG): %d", ret);
   }
#endif
   return ret;
}



BTINLINE int btrt_mutex_unlock(btrt_mutex *mutex)
{
   int ret;

#ifdef XENOMAI
   btrt_set_mode_hard();
   ret = rt_mutex_release(mutex);
   if (ret != 0)
   {
      syslog(LOG_ERR, "Mutex unlock failed (XENO): %d",  ret);
   }
#else
   ret = pthread_mutex_unlock(mutex);
   if (ret != 0)
   {
      syslog(LOG_ERR, "Mutex unlock failed (REG): %d",  ret);
   }
#endif
   return ret;
}
