/*-----------------------------------------------------------------------------
icanconf.h -- ICANOS specific definitions

Copyright (c) 1994 JANZ Computer AG
All Rights Reserved

Created 94/10/11 by Soenke Hansen
Version 1.17 of 00/04/27

Declarations and definitions of functions and macros:
  - disable/enable CAN and clock interrupts
  - start/stop a periodical timer
  - read/write registers in Basic CAN controller
The conditionally included files i8051.h and mc68332.h contain
examples for the Intel 8051 and Motorola 68332 micro-controller
families, respectively.

Some definitions in bcan.h depend on the oscillator frequency used
for the Basic CAN controller.  Currently only definitions for a 16 MHz
oscillator are given.  These are activated by defining OSC_16MHZ here.

Memory allocation functions:
  - malloc() and free()
If these functions are not available from a system library the
functions implemented in malloc.c are used if OUR_MALLOC==1.
These functions allocate memory from a fixed size array.
The length of this array in bytes is given by MALLOC_BYTES.
The data type ALIGN_t is used to insure that malloc returns addresses
which are correctly aligned for every data type.

-----------------------------------------------------------------------------*/

#ifndef icanconf_DEFINED
#define icanconf_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

/* Load the user configuration */
#include "config.h"

/* Intel 8051 family with Archimedes C-51 compiler */
#ifdef i8051
#include "i8051.h"
#endif

/* Motorola 68332 with Microtec Research C compiler */
#ifdef mc68332
#include "mc68332.h"
#endif

/* Implementation options */
#ifndef ICAN_ECHO
#define ICAN_ECHO	1	/* with local transmit echo if == 1 */
#endif
#ifndef ICAN_FILTER
#define ICAN_FILTER	1	/* with acceptance filtering if == 1 */
#endif

/* Set maximum CAN send queue length. */
#ifndef TX_QUEUE_LEN
#define TX_QUEUE_LEN	20
#endif

/* Define this if a 16 MHz oscillator is used for BCAN */
#define OSC_16MHZ

/*------- Memory allocation ------------------------------------------------*/

/* If OUR_MALLOC is !=0 then malloc() and fre() as implemented in
   malloc.c are used else the system library functions are used.
   By default our functions are used. */
#ifndef OUR_MALLOC
#define OUR_MALLOC		0	/* if true, use our malloc and free */
#endif

/* Memory management */
#if OUR_MALLOC == 1		/* using our malloc and free? */

#ifndef MALLOC_BYTES
#define MALLOC_BYTES	8000	/* size in bytes of alloc array */
#endif
typedef int ALIGN_t;			/* alignment type */

/* Memory is allocated from an array of blocks of type HEADER */
union header { /* free block header */
	struct {
		union header	*ptr;	/* next free block */
		unsigned	size;	/* size of this free block */
	} s;
	ALIGN_t	x;			/* force alignment of blocks */
};

typedef union header HEADER;

/* Maximum number of blocks */
#define MALLOC_BLOCKS (MALLOC_BYTES/sizeof(HEADER)+1)

extern void init_malloc();		/* initialization of alloc array */
extern void *malloc();			/* well known function */
extern void free();			/* well known function */
extern int check_free_mem();		/* check for free memory */

#else				/* use malloc and free of system library */
#include <stdlib.h>
#define init_malloc()
#endif
#if FIRMWARE > 0
/* Only used within firmware. Solve problem with network defines. */
#define FREE(p) if ((p)) free((void *)p)
#endif


/*------- Debugging tools (for development at JANZ only) -------------------*/

#ifndef DEBUG
#define DEBUG		0	/* default: no debugging */
#endif

/* Logging */
#if DEBUG>0
#include "debug.h"
#else
#define PRINTF(args)
#define LOG(level, args)
#define DESPAIR
#endif

#ifndef BIGBOX
#ifdef GDBDEBUG
#include "userIf.h"
#else
#define	KPRINTF(X)
#endif
#endif

/* Dummies */
#if DEBUG==1
#include "simul.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* !icanconf_DEFINED */
