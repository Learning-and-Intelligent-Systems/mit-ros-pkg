/*-----------------------------------------------------------------------------
timer.h -- Timer Management

Copyright (c) 1994 JANZ Computer AG
All Rights Reserved

Created 94/10/11 by Soenke Hansen
Version 1.3 of 96/04/18

Prototypes of functions defined in timer.c.
The details of timer messages are in msg.h.

-----------------------------------------------------------------------------*/

#ifndef timer_DEFINED
#define timer_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

#include "defs.h"
#include "msg.h"

/* Time units are 100usecs. Conversion (with chopping) from other 
   time units. */
#define time_units_ms(ms) (((ms) < 0xffff/10) ? 10 * (ms) : 0xffff)

/* Timer identifiers are 16 bit unsigned integers */
#define NIL_TIMID	(WORD_t)0	/* invalid identifier */

/* Initialize the timer management */
extern void init_timer _PARAMS((void));

/* Start periodical timer */
extern void start_timer _PARAMS((WORD_t));

/* Stop timer */
extern void stop_timer _PARAMS((void));

/* Deactivate a timer */
extern void clear_timer _PARAMS((WORD_t));

/* Activate a timer.  The alarm time is given in units of 100 usecs.
   The alarm time is relative to the time of call. */
extern int set_timer _PARAMS((WORD_t, WORD_t, WORD_t *));

/* Alarm interrupt handler passed to the clock. */
extern void isr_timer _PARAMS((void));

#ifdef __cplusplus
}
#endif

#endif /* !timer_DEFINED */
