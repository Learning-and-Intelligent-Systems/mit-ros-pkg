/*-----------------------------------------------------------------------------
msg.h -- Messages and message queues

Copyright (c) 1994 JANZ Computer AG
All Rights Reserved

Created 94/10/11 by Soenke Hansen
Version 1.9 of 00/07/19

Messages consist of a specifier for its contents, the (address of its)
contents, and the relevant length of the contents.  Messages are
allocated from the heap with the maximum length of the contents specified.
The relevant length is useful when transmitting the message outside
the local address space.
Messages can be linked to form message queues or lists.

-----------------------------------------------------------------------------*/

#ifndef msg_DEFINED
#define msg_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

#include "defs.h"

/* Mode or format type of a message */
#define MMODE_LOCAL		0x01	/* internal compiler specific format */
#define MMODE_TRANSFER		0x02	/* external format for transfer */

/* Maximum length definitions */
#define BCAN_LEN	 8	/* max data length in CAN message */
#define MB_LEN		16	/* max data length in bounded sized message */

/* Message contents types */
struct m_m1 {	/* multiple purpose message (local format) */
	WORD_t	w[6];
};
struct m_m2 {	/* for BCAN messages (local format) */
	WORD_t	r;		/* request id */
	WORD_t	d;		/* descriptor */
	BYTE_t	b[BCAN_LEN];	/* data bytes */
};
struct m_m3 {	/* generic transfer data buffer */
	BYTE_t	len;		/* length, number of relevant bytes */
	BYTE_t	gap;		/* just to align component b[] */
	BYTE_t	b[MB_LEN];	/* bounded buffer */
	BYTE_t	*pb;		/* allocated buffer if len > MB_LEN */
};

struct m_m4 {	/* transfer data buffer for bulk purposes */
	BYTE_t		len;		/* length, number of relevant bytes */
	BYTE_t		gap1;		/* just to align component b[] */

	BYTE_t		available;
	BYTE_t		gap2;		/* align */
	WORD_t		actByteCount;	/* actual usage of bulk buffer */
	LWORD_t 	actTime;	/* last write time of this bulk buffer */
	BYTE_t		*pActData;	/* pointer to actual write-data location */
	
	BYTE_t		gap3[4];	/* gap */
	BYTE_t		*pb;		/* allocated buffer */
};

/* Message object definition */
struct message {
	BYTE_t		spec;	/* message contents specifier */
	BYTE_t		mode;	/* mode (message format type) */
	union {			/* message contents */
		WORD_t		align;
		/* mode == MMODE_LOCAL */
		struct m_m1	m1;
		struct m_m2	m2;
		/* mode == MMODE_TRANSFER */
		struct m_m3	m3;
		/* mode == MMODE_TRANSFER for bulk/sniff buffer */
		struct m_m4	m4;
	} u;
	struct message	*next;	/* next on list/queue */
};

#define NIL_MESSAGE	(struct message *)0

/* Message components for mode==MMODE_TRANSFER */
#define mlen	u.m3.len	/* usage: m->mlen (if: struct message *m) */
#define mCcNo	u.m3.gap	/* CAN controller number */
#define fdata	u.m3.b		/* pointer to fixed length data buffer */
#define vdata	u.m3.pb		/* pointer to variable length data buffer */
#if FIRMWARE > 0
/* Only used within firmware. Solve problem with network defines. */
/* Pointer to beginning of data buffer */
#define m_data(m) (((m)->mlen > MB_LEN) ? (m)->vdata : (m)->fdata)
#endif

/* definitions for bulk buffer transfer */
/* (always MMODE_TRANSFER, always variable length data buffer) */
#define mBulkAvailable	  u.m4.available	/* usage: m->mBulkAvailable */
#define mBulkActByteCount u.m4.actByteCount	/* usage: m->mBulkActByteCount */
#define mBulkActTim	  u.m4.actTime		/* usage: m->mBulkActTime */
#define mBulkDataAddr	  u.m4.pActData		/* usage: m->mBulkDataAddr */

/* definitions for sniff buffer transfer */
/* (always MMODE_TRANSFER, always variable length data buffer) */
#define mSniffAvailable	  u.m4.available	/* usage: m->mSniffAvailable */
#define mSniffActByteCount u.m4.actByteCount	/* usage: m->mSniffActByteCount */
#define mSniffActTim	  u.m4.actTime		/* usage: m->mSniffActTime */
#define mSniffDataAddr	  u.m4.pActData		/* usage: m->mSniffDataAddr */

/* Components of a timer message (struct m_m1) */
#define timer_period	u.m1.w[0]
#define timer_req	u.m1.w[0]
#define timer_tid	u.m1.w[1]
#define timer_tleft	u.m1.w[2]

/* Components of a filter message (struct m_m1) */
#define afil_id		u.m1.w[0]
#define afil_idlow	u.m1.w[0]
#define afil_idhigh	u.m1.w[1]

/* Components of contents of struct m_m1 message */
#define bcan_reset	u.m1.w[0]	/* !=0 if reset for M_BCAN_SET*req */
#define bcan_acm	u.m1.w[1]	/* M_BCAN_SET_ACM_req */
#define bcan_btr	u.m1.w[1]	/* M_BCAN_SET_BTR_req */
#define bcan_evt	u.m1.w[0]	/* M_BCAN_EVENT_ind */
#define bcan_evt_e0	u.m1.w[1]	/*  M_BCAN_EVENT_ind extended error */
#define bcan_evt_e1	u.m1.w[2]	/*  M_BCAN_EVENT_ind extended error*/
#define bcan_evt_e2	u.m1.w[3]	/*  M_BCAN_EVENT_ind extended error*/
#define bcan_evt_e3	u.m1.w[4]	/*  M_BCAN_EVENT_ind extended error*/
#define bcan_evt_ccNo	u.m1.w[6]	/*  M_BCAN_EVENT_ind CAN controller number */
#define bcan_txreq	u.m1.w[0]	/* M_BCAN_TX_con */
#define bcan_txcon	u.m1.w[1]	/* M_BCAN_TX_con */

/* Components of a BCAN message (struct m_m2) */
#define bcan_req	u.m2.r
#define bcan_desc	u.m2.d
#define bcan_data	u.m2.b


/* Message queues */
struct mqueue {
	struct message	*head;	/* first element on queue */
	struct message	*tail;	/* last element on queue */
	struct message	*pos;	/* current position in queue */
};

#define NIL_MQUEUE	(struct mqueue *)0

/* Get pointer to the first message in a queue */
#define head_mqueue(q) ((q)->head)
#define first_mqueue(q) ((q)->head)

/* Remove the head from a non-empty queue */
#define rmhead_mqueue(q) do { \
		if ((q)->head != (q)->tail) (q)->head = (q)->head->next; \
		else (q)->head = (q)->tail = NIL_MESSAGE; \
		} while (0)


/* Allocate a message */
extern struct message *new_message _PARAMS((int, int));

/* Deallocate a message */
extern void delete_message _PARAMS((struct message *));

/* Allocate a message queue */
extern struct mqueue *new_mqueue _PARAMS((void));

/* Deallocate a message queue. Any messages in the queue are also
   deallocated. */
extern void delete_mqueue _PARAMS((struct mqueue *));

/* Remove the first message from a queue. The address of the message
   is returned. */
extern struct message * extract_mqueue _PARAMS((struct mqueue *));

/* Append a message to a queue */
extern void append_mqueue _PARAMS((struct mqueue *, struct message *));

/* Rewind queue for looping through its elements */
extern void rewind_mqueue _PARAMS((struct mqueue *));

/* Get next message from queue */
extern struct message *getnext_mqueue _PARAMS((struct mqueue *));

/* Delete the successor of a message from a queue.  The deleted (not
   freed!) message is returned. */
extern struct message *
delsuc_mqueue _PARAMS((struct mqueue *, struct message *));

/* Insert a message into the queue after a specified message. */
extern void
insert_mqueue _PARAMS((struct mqueue *, struct message *, struct message *));

#ifdef __cplusplus
}
#endif

#endif /* !msg_DEFINED */
