/*-----------------------------------------------------------------------------
icanif.h -- Interface to ICANOS/2

Copyright (c) 1994 JANZ Computer AG
All Rights Reserved

Created 94/10/11 by Soenke Hansen
Version 1.6 of 96/10/31

Timer and CAN controller indications, and received CAN messages are multiplexed
"upwards" into a queue which as to be read with recv_ican() by the user.
Requests to the timer management, the CAN controller and the acceptance
filter are issued via function calls.

-----------------------------------------------------------------------------*/

#ifndef icanif_DEFINED
#define icanif_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

#include "icanconf.h"
#include "defs.h"
#include "mican.h"
#include "msg.h"
#include "bcan.h"
#include "afil.h"
#include "timer.h"


/* Initialize the multiplexer */
extern void init_ican _PARAMS((void));

/* Multiplex CAN messages from ICANOS in upwards queue */
extern void mux_ican _PARAMS((struct message *));

/* Multiplex timer messages from ICANOS in upwards queue */
extern void mux_ican_tim _PARAMS((struct message *));

/* User receives next message from ICANOS */
extern struct message *recv_ican _PARAMS((void));

#ifdef __cplusplus
}
#endif

#endif /* !icanif_DEFINED */
#ifndef icanif_DEFINED
#define icanif_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

#include "icanconf.h"
#include "defs.h"
#include "mican.h"
#include "msg.h"
#include "bcan.h"
#include "afil.h"
#include "timer.h"

/* Set Priority Boundaries */
extern void set_prio_boundaries(WORD_t,WORD_t);

/* Initialize the multiplexer */
extern void init_ican _PARAMS((void));

/* Multiplex messages from ICANOS in upwards queue for CAN-msgs */
extern void mux_ican _PARAMS((struct message *));

/* Multiplex messages from ICANOS in upwards queue for timer-msgs */
extern void mux_ican_tim _PARAMS((struct message *));

/* User receives next message from ICANOS */
extern struct message *recv_ican _PARAMS((void));

#ifdef __cplusplus
}
#endif

#endif /* !icanif_DEFINED */
