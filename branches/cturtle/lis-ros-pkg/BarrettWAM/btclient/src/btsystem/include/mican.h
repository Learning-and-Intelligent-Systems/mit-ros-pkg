/*-----------------------------------------------------------------------------
mican.h -- ICANOS Message Specifications

Copyright (c) 1994 JANZ Computer AG
All Rights Reserved

Created 94/10/11 by Soenke Hansen
Version 1.18 of 00/08/16

Definitions of message specifiers for messages to/from ICANOS/2.

-----------------------------------------------------------------------------*/

#ifndef mican_DEFINED
#define mican_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

/* Message specifiers */
#define M_TIMER_START_req	0x20  /* start timer request */
#define M_TIMER_STOP_req	0x21  /* stop timer request */
#define M_TIMER_SET_req		0x22  /* set timer request */
#define M_TIMER_SET_con		0x23  /* confirm setting of timer */
#define M_TIMER_TO_ind		0x24  /* indicate timeout event */
#define M_TIMER_CLEAR_req	0x25  /* clear timer request */
#define M_BCAN_TX_req		0x30  /* transmit request */
#define M_BCAN_TXC_req		0x31  /* transmit request with pos.conf. */
#define M_BCAN_TXE_req		0x32  /* transmit request with echo */
#define M_BCAN_TXCE_req		0x33  /* transmit request w.con(+), echo */
#define M_BCAN_TXAT_req		0x34  /* abort transmission request */
#define M_BCAN_TX_con		0x35  /* confirm (+/-) transmission */
#define M_BCAN_RX_ind		0x36  /* receive indication */
#define M_BCAN_EVENT_ind	0x37  /* BCAN event (error, overrun) */
#define M_BCAN_SET_ACM_req	0x40  /* set acceptance code and mask */
#define M_BCAN_SET_BTR_req	0x41  /* set bus timing parameters */
#define M_BCAN_BUSOFF_req	0x42  /* switch to bus-off state */
#define M_BCAN_BUSON_req	0x43  /* switch to bus-on state */
#define M_BCAN_BUSON_AB_req	0x48  /* bus-on w. auto. baudrate detect req. */
#define M_BCAN_BUSON_AB_con	0x48  /* bus-on w. auto. baudrate detect con. */
#define M_BCAN_SETREG_req	0x44  /* set BCAN registers request */
#define M_BCAN_GETREG_req	0x45  /* get BCAN registers request */
#define M_BCAN_GETREG_con	0x46  /* get BCAN registers confirmation */
#define M_BCAN_CONF_req		0x47  /* Special CANbus configurations */
#define M_AFIL_OPEN_req		0x50  /* accept CAN-Id */
#define M_AFIL_ROPEN_req	0x51  /* accept CAN-Id range */
#define M_AFIL_CLOSE_req	0x52  /* reject CAN-Id */
#define M_AFIL_RCLOSE_req	0x53  /* reject CAN-Id range */
#define	M_CYC_LIST_CREATE_req	0x60  /* request to create a list of cyclic CANbus messages */
#define	M_CYC_LIST_CREATE_con	0x61  /* confirmation of creation of cyclic-send-list */
#define	M_CYC_LIST_DELETE_req	0x62  /* request to delete a list of cyclic CANbus messages */
#define	M_CYC_LIST_DELETE_con	0x63  /* confirmation of deletion of cyclic-send-list */
#define M_CYC_MSG_INSERT_req	0x64  /* request to insert a message in cyclic send list */
#define M_CYC_MSG_INSERT_con	0x65  /* confirmation of insertion of a message into list */
#define M_BCAN_BULK_ind		0x66  /* bulk buffer-message indication */
#define M_BCAN_SNIFF_ind	0x68  /* sniff buffer-message indication */

#define M_BCAN_BUSLOAD_ind	0x67  /* busload-statistic-indication */
#define MS_BCAN_BUSLOAD_ALL	0x00  /* request/indicate ALL busload-statistics */

#define M_SNIFFFIL_MASK_req	0x69  /* sniff buff. set-acceptance filter request */

/* Check type of transmit request (with/without echo and/or pos. confirm.
   Note the consistency with the definitions of M_BCAN_TX*_req! */
#define TX_REQ_CONF	0x01	/* positive confirmation desired if set */
#define TX_REQ_ECHO	0x02	/* echo desired if set */
#define tx_req_conf(spec) ((unsigned)(spec) & TX_REQ_CONF)
#define tx_req_echo(spec) ((unsigned)(spec) & TX_REQ_ECHO)

/* Message subspecifiers for M_BCAN_TX_con messages */
#define TX_OKAY			0x00	/* positive transmit confirmation */
#define TX_NOBUF		0x01	/* controller transmit buffer full */
#define TX_FAIL			0x02	/* negative transmit confirmation */

/* Message subspecifiers for M_BCAN_EVENT_ind messages */
#if defined Customer_1
#define C1_EVT_ERROR		0x01	/* error interrupt occured */
#else
#define EVT_ERROR		0x01	/* error interrupt occured */
#endif
#define EVT_OVERRUN		0x02	/* overrun interrupt occured */
#define EVT_LOST_INTERRUPTS	0x04	/* interrupts lost */
#define EVT_QUEUE_FULL		0x08	/* send queue full */
#define EVT_BERR		0x10    /* CANbus bus-error */

/* Additional error types for M_BCAN_EVENT. */
#define EVT_ERRT_NONE		0x00	/* no additional error infos */
#define EVT_ERRT_82C200		0x01	/* 82C200 additional error infos */
#define EVT_ERRT_SJA1000	0x02	/* SJA100 additional error infos */

/* Additional command subspecs for M_BCAN_CONF_req */
#define MS_BCAN_CONF_BERR	0x00	/* Configure bus-error detection */
#define MS_BCAN_CONF_EWL	0x01	/* Configure error warning limit */
#define MS_BCAN_CONF_LOM	0x02	/* Configure listen only mode */
#define MS_BCAN_CONF_STM	0x03	/* Configure self test mode */

/* autobaud stuff */
#define	AUTOBAUD_INACTIVE		0
#define	AUTOBAUD_ACTIVE			(1 << 7)
#define	AUTOBAUD_PASSIVE		(1 << 6)
#define	AUTOBAUD_RX_VALID_MSG	(1 << 0)
#define	AUTOBAUD_TX_VALID_MSG	(1 << 1)

#define	AB_ACTIVE_FAIL			0x00
#define	AB_ACTIVE_SUCCESS		0x01
#define	AB_ACTIVE_ATTEMPT		0x02

#define	AB_PASSIVE_FAIL			0x10
#define	AB_PASSIVE_SUCCESS		0x11
#define	AB_PASSIVE_ATTEMPT		0x12

#define	END_OF_AUTOBAUD_TAB		0xffff
#define	DEFAULT_AB_TIMER_ID		0xABBE	/* =8-) */

/* CAN bus message to probe with on autobaud active */
struct probeMessage {
	BYTE_t	extRtrLen;
	BYTE_t	id1;		/* fast message "cmd" format */
	BYTE_t	id2;
	BYTE_t	id3;
	BYTE_t	id4;
	BYTE_t	data[8];
};

/*
 * do a certain number of send attempts at a certain 
 * bitrate for a certain time
 */
struct ab_act_per_btr {
	WORD_t	btr;
	WORD_t	noOfAttempts;
	WORD_t	skipTime;
};

/*
 * wait a certain number of receive attempts at a certain
 * bitrate for a certain time
 */
struct ab_pas_per_btr {
	WORD_t	btr;
	WORD_t	skipTime;
};

struct ab_act_msg {
	struct probeMessage		probeMsg;
	BYTE_t					numberOfSamples;
	struct ab_act_per_btr	sample[30];	/* ...should be enough ;-) */  
};
struct ab_pas_msg {
	BYTE_t					numberOfSamples;
	struct ab_pas_per_btr	sample[30];
};

#ifdef __cplusplus
}
#endif

#endif /* !mican_DEFINED */
