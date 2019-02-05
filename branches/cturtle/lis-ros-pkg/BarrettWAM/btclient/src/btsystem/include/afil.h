/*-----------------------------------------------------------------------------
afil.h -- Acceptance filtering of CAN messages

Copyright (c) 1994 JANZ Computer AG
All Rights Reserved

Created 94/10/11 by Soenke Hansen
Version 1.8 of 00/07/19

Based on their Id's received CAN messages are accepted or rejected.
CAN frames for which the filter is open will always be accepted.
A close request to the filter may not honored as it is possible
that the filter is not implemented.

-----------------------------------------------------------------------------*/

#ifndef afil_DEFINED
#define afil_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

#include "defs.h"
#include "msg.h"
#include "icanconf.h"


/*-------- Acceptance Filtering of CAN Ids ---------------------------------*/

#if ICAN_FILTER == 1

/* */
extern BYTE_t id_filter[];

/* Initialize the acceptance filter for CAN identifiers */
extern void init_afil(void);

/* Return true if CAN message with given Id is accepted */
extern int accept_afil(WORD_t, WORD_t);

/* Open the acceptance filter for a given id, so messages received
   with this id will pass the acceptance test */
extern void open_afil(WORD_t, WORD_t);

/* Close the acceptance filter for a given id, so messages received
   with this id will not pass the acceptance test */
extern void close_afil(WORD_t, WORD_t);

/* Open the acceptance filter for a given range of ids, so messages received
   with ids in that range will pass the acceptance test */
extern void ropen_afil(WORD_t, WORD_t, WORD_t);

/* Close the acceptance filter for a given range of ids, so messages received
   with ids in that range will not pass the acceptance test */
extern void rclose_afil(WORD_t, WORD_t, WORD_t);

/* Get the current state of the filter mask for a given id. */
extern int get_afil_mask(WORD_t, WORD_t);

/* Set the filter mask for a given id. */
extern void set_afil_mask(WORD_t, WORD_t, WORD_t);

/* Set the acceptance filter mask for a given range of ids.
   You may set the mask to a value in the range 0..(2^BITS_PER_ID)-1. */
void rset_afil_mask(WORD_t, WORD_t, WORD_t, WORD_t);

/* Open the sniff acceptance filter for a given range of ids, so messages received
   with ids in that range will pass the acceptance test */
extern void ropen_sniff_fil(BYTE_t, WORD_t, WORD_t);

/* Close the sniff acceptance filter for a given range of ids, so messages received
   with ids in that range will not pass the acceptance test */
extern void rclose_sniff_fil(BYTE_t, WORD_t, WORD_t);

#endif /* ICAN_FILTER == 1 */

#if ICAN_FILTER == 0		/* filtering not implemented; dummies */
#define init_afil()
#define accept_afil(id)	  1		/* accept everything */
#define open_afil(id)
#define close_afil(id)
#define ropen_afil(idl, idh)
#define rclose_afil(idl, idh)
#endif /* ICAN_FILTER == 0 */



#define CAN_2_0_A	1
#define CAN_2_0_B	2

extern void initSniffFil(void);
extern void rangeSetSniffFil(BYTE_t canSpec, WORD_t idl, WORD_t idh, BYTE_t value);
extern int getSniffFil(BYTE_t canSpec, LWORD_t id);

#ifdef __cplusplus
}
#endif

#endif /* !afil_DEFINED */
