/*-----------------------------------------------------------------------------
vmod.h -- VMOD specific definitions (message specifiers, values)

Copyright (c) 1994 JANZ Computer AG
All Rights Reserved

Created 96/08/01 by Stefan Althoefer
Version 1.24 of 01/10/24

-----------------------------------------------------------------------------*/

#ifndef vmod_DEFINED
#define vmod_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

/* Specifiers of general messages */
#define M_NOP		0x00  /* nil message, no operation */
#define M_CONNECT	0x01  /* connect to host without interrupts */
#define M_CONNECT_INTR	0x02  /* connect to host with interrupts */
#define M_DISCONNECT	0x03  /* disconnect from host */
#define M_IDVERS	0x04  /* get id and version */
#define M_MSG_LOST	0x05  /* messages lost, host reads too slow */
#define M_DEBUG		0x06  /* formatted debug output */
#define M_SELECT	0x07  /* select functionality */
#define M_NEWHOSTIF	0x08  /* switch to new stylish hostif */
#define M_MEMORY	0x09  /* get info about memory */
#define	M_INQUIRY	0x0a  /* get some info about module status */

#define	M_LOCK_LOADER		0x0b	/* lock loader on next reset */
#define M_FW_DOWNLOAD_req	0x0c	/* host starts a firmware download */
#define M_FW_DOWNLOAD_CONT_ind	0x0d	/* target requests for more data */
#define M_FW_DOWNLOAD_CONT_res	0x0e	/* host responds with next data */
#define M_FW_DOWNLOAD_con	0x0f	/* target confirms the service */

#define M_SET_AFIL		0x10	/* set acceptance filter mask */
#define M_INIT_FDPMQUEUE	0x11	/* initialize fast dpmqueue */
#define M_HW_CONF		0x12	/* configure hardware features */
#define M_SET_PRIO_BND          0x13    /* set priority boundaries */ 
#define	M_HANDLE_HOST_GUARDING	0x14	/* spec dealing with host-guarding */

#define M_FMSG_LOST		0x15	/* lost messages on fast queue */

#define M_DEV_DEBUG		0x1f	/* for debugging purposes */

/* host guarding stuff concerning macros */
#define	GUARD_ACTIVE	0x01
#define	GUARD_INACTIVE	0x00
#define	ALIVE		0xaa
#define	DEAD		0x55

/* module/host dependant CANopen configuration */
#define	M_CONF_CANOPEN_MUX				0x16	/* user <-> CANopen relevant config. */
#define	M_CONF_CANOPEN_MUX_DPM_ENDIAN	0x0		/* endianess of host */
/* possible values: */
#define	COP_DPM_BIG_ENDIAN				0x00	/* mapping for Big Endian hosts (obsolete!) */
#define	COP_DPM_LITTLE_ENDIAN			0x01	/* mapping for Little Endian hosts (obsolete!) */
#define	COP_HOST_IS_BIG_ENDIAN			0x02	/* mapping for Big Endian hosts */
#define	COP_HOST_IS_LITTLE_ENDIAN		COP_DPM_LITTLE_ENDIAN
/* mux for seeing the SYNC or not at host */
#define	M_CONF_CANOPEN_MUX_SEND_SYNC_TO_HOST	0x1	/* visibility of SYNC on host */
#define	M_CONF_CANOPEN_MUX_RTR_RPDO_ANSWER_ind	0x2	/* indication on answer of RTR triggered RPDO */
#define	M_CONF_CANOPEN_MUX_TPDO_TRIGGERED_ind	0x3	/* indication of triggering a TPDO */
#define	M_CONF_CANOPEN_MUX_SET_SAFE_DPM_MODE	0x4	/* det DPM-PP syncing to safe mode */

#define M_ADD_FDPMQUEUE_PRIO	0x17	/* create addtnl. prioritized fast fromhost dpmqueue */

/* BULK_BUFFER-stuff shouldn't it  belong to mican.h (ICANOS) ??? */
#define M_CONFIG_BULK_BUFFER	0x18	/* user <-> bulk buffer configuration (CAN-Layer-2) */
#define MS_CONFIG_BULK_BUFFER_TIMEOUT_ONLY	0x0000	/* configure bulk buffers timeout to send */
#define MS_CONFIG_BULK_BUFFER_TIMEOUT_SIZE	0x0001	/* configure bulk buffers timeout and size */

#define	M_CONFIG_SNIFF	0x1a	/* user <-> sniff buffer configuration */
#define MS_CONFIG_SNIFF_BUFFER_TIMEOUT_ONLY	0x0000	/* config sniff buff. timeout to send */
#define MS_CONFIG_SNIFF_BUFFER_TIMEOUT_SIZE	0x0001	/* config sniff buff. timeout and size */
#define MS_CONFIG_SNIFF_BUFFER_ECHO	0x0002	/* config sniff buff. echo from queues */
#define	MS_CONFIG_SNIFF_FILTER_STD		0x0003	/* set sniff acc. filt. (Std. CAN frames) */
#define	MS_CONFIG_SNIFF_FILTER_XTD		0x0004	/* set sniff acc. filt. (Xtd. CAN frames) */
#define	MS_SNIFF_INQUIRY		0x0005	/* inquiry module status for sniff mechanism */
#define	MS_SNIFF_IDVERS		0x0006	/* inquiry module version for sniff mechanism */
#define	MS_SNIFF_BUSLOAD_ALL	0x0007 /* request all (std./xtd. Rx/Tx) busloads (sniff) */
#define	MS_SNIFF_CONFIG_BUSLOAD	0x0008 /* configure busload  for sniff mechanism */
#define	MS_SNIFF_INQUIRY_CONTROLLER		0x0009	/* inquiry CAN controller status for sniff mechanism */



/* message specifier/subspecifier for measurement of CAN bus load */
#define M_BUSLOAD	0x19	/* config./send/receive bus-load parameter/values */
#define MS_CONFIG_BUSLOAD	0x0000	/* configure/de-/activate bus-load-mechanism */
#define MS_REQUEST_BUSLOAD_ALL	0x0001	/* request bus-load data */


/* Customer 1 */
#define	M_CUST1			0xc0	/* Customer 1 */

/* Subspecs for Customer 1 */
#define MS_CUST1_INIT_req	0x0000	/* SysClk Init */
#define MS_CUST1_START_TS_req   0x0001  /* OBSOLET! */

/* Subspecs/defines for module inquiry M_INQUIRY */
#define MS_INQUIRY_STATUS		0x0000	/* CAN Chip / module status */
#define MS_INQUIRY_TIMING_MASKS_TERM	0x0001	/* CANbus bit rate, accept */
						/* masks, termination state */
#define MS_INQUIRY_HOST_IF		0x0002	/* new/old host interface */
						/* on module */
#define MS_INQUIRY_AFIL			0x0003	/* looks for acceptance mask */
#define MS_INQUIRY_EXTD_STATUS		0x0004	/* CAN Chip / module status */
						/* superior format. */

#define MASK_COUNT	50	/* count of ID masks which will be sent */
				/* on a MS_INQUIRY_AFIL response (CAVE AT */
				/* the buffer size to loose no messages!) */
#define	INVALID_ID	9	/* signals out of id-range in INQUIRY_AFIL */

#define INQUIRY_CTRL_82C200	0x01	/* 82C200 extended infos */
#define INQUIRY_CTRL_SJA1000	0x02	/* SJA100 extended infos */

/* Definitions for ICAN2/3 dependent termination states */
#define TERM_INFO_AVAILABLE	0x80	
#define TERM_INFO_NOT_AVAILABLE	0x00	
#define	TERM_ON			0x01
#define	TERM_OFF		0x00

/* Maximum data length of messages */
#define MSGLEN	252


/* Definition of some special DPM locations. */
#define TARGET_ALIVE		0xc
#define TARGET_STATUS		0xe

/*
 * Some CANopen definitions for process picture
 */
#define	COP_NEW_SAVE_PP_SYNC	1
#define	COP_OLD_UNSAVE_PP_SYNC	0	/* default! */
/* length of necessary bytes for synchronization between host and VMOD-ICAN3 */
#define	NUM_DPM_SYNC_BYTES			2

/* CANopen DPM-SYNC */
#define	SYNC_BYTE_ICAN_OWNED		0
#define	SYNC_BYTE_HOST_OWNED		1

#define	MAX_DPM_PAGES				256	/* 0 .. 255 */
#define	MAX_DPM_BYTES				256	/* 0 .. 255 */

#define	DPM_DATA_INVALID			0
#define	DPM_DATA_VALID				1

/* location of process picture */
#define	PP_START_IN_DPM		0x8000	/* PP starts at 32KByte in DPM */
#define	PP_MAX_BYTE_NUM		0x8000	/* PP has a length of 32KByte in DPM */



#ifdef __cplusplus
}
#endif

#endif /* !vmod_DEFINED */
