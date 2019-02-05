/*-----------------------------------------------------------------------------
dpm.h -- Linux driver of DPM interface to VMOD

Copyright (c) 1996 JANZ Computer AG
All Rights Reserved

Created 96/01/23 by Stefan Althoefer

Version 1.12 of 96/09/27

This file is for both, the "dpm" and the "dpmw" driver.
The only difference, besides the names of the driver functions, are the in 
the INKERNEL structures. But these are not visible to the user.

-----------------------------------------------------------------------------*/

#ifndef __INCdpmh
#define __INCdpmh

#ifdef __cplusplus
extern "C" {
#endif

#include "defs.h"
#include "vmod.h"

/* Max. Count of Devices for the dpm driver. */
#define	MAX_BOARDS		16	/* max. # of boards supported	*/
#define	MAX_MODUL_BOARDS	4	/* max. # of modules supported	*/
					/* with one board		*/
#define MAX_DPM_DEV 		(MAX_BOARDS * MAX_MODUL_BOARDS)

#define	FAST_QUEUE		'\0'	/* flag for fast queue 		*/
#define	PLAIN_QUEUE		'\1'	/* flag for normal queue 	*/

/* Data type of messages */
#define MSGLEN		252	/* Maximum length of a message */
#define MSGHDLEN	  4	/* Length of message head */

#define	ICAN2	1
#define	ICAN3	2

/* -*-Struct-*-
 *
 * Message - local message buffer for standard host interface
 *
 * This structure stores a message to be send to or received from
 * a CAN module via the standard host interface.
 *
 * <cmd> is an 8 bit command specifier. Refer to the reference manual
 * to find out which services are provided by the different <cmd>
 * specifiers. The data length to be transfered is stored in <len>, thereby
 * <len> specifies how many bytes of the vector <data> are valid. The data
 * have to be interpreted by means of <cmd>.
 * The <control> byte is transfered to the firmware with the
 * message, and may be used as defined by the specific firmware.
 *
 * At most MSGLEN bytes can be stored in a message. MSGLEN is 252 bytes
 * for all current CAN modules.
 */
typedef struct {
	BYTE_t    cmd;           /* Message specifier */
	BYTE_t    control;       /* Control byte */
	WORD_t    len;           /* Message length (<= MSGLEN) */
	BYTE_t    data[MSGLEN];  /* Data array of Message */
} Message;


/* -*-Struct-*-
 *
 * FastMessage - local message buffer for fast host interface
 *
 * This structure stores a CAN message to be send to or received from
 * the CAN modules fast host interface.
 *
 * <cmd> is an 8 bit service multiplexor. The bits are interpreted as
 * follows:
 * .CS
 *   cmd = LNNNCCCC
 *     L: CANlook flag
 *     N: Controller number (if enabled)
 *     C: Command spec
 * .CE
 * Currently only the <command specs> 0 and 1 are in use.
 * Received messages with any other values should be ignored
 * for compatiblity with future firmware releases. The <command spec>=1
 * format is only meaningfull on CAN moduelswith the SJA1000 controller
 * equipped.
 *
 * The controller number selects between different CAN channels on multi
 * CAN interface modules. The number will only be evaluated when this
 * feature has been enabled in the firmware.
 *
 * The actual CAN frame information are stored in <data>. Depending on
 * the <cmd> bytes, the data will be represended in two formats:
 *
 * .CS
 * command spec      0                              1
 *
 * data[0]       ID[10..3]                 EXT<<7 + RTR<<6 + DLC
 * data[1]  ID[2..0]<<5 + RTR<<4 + DLC      ECHO<<4 + SNGL<<1
 * data[2]       CAN byte 1                     EID[28..21]
 * data[3]           :                          EID[20..13]
 * data[4]           :                          EID[12..5]
 * data[5]           :                          EID[4..0]
 * data[6]           :                          CAN byte 1
 * data[:]           :                               :
 * data[9]       CAN byte 8                          :
 * data[10]        unused                            :
 * data[ :]          :                               :
 * data[13]        unused                       CAN byte 8
 * .CE
 *
 * Remote frames are indicated by T<RTR=1>. T<EXT=1> indicates extended
 * CAN frames (only in format1 messages).
 *
 * With T<ECHO=1> you can request that a message is echoed to the host when
 * the message has been sucessfully sent over the CAN. Setting <SNGL=1>
 * request a message to be transmitted only once, without any retries in
 * case of bus-errors or arbitration losts.
 *
 * If a standard CAN frame is transported trough format 1 messages,
 * then ID[10..0] are represented by EID[28..18].
 *
 * If a standard CAN frame is transported trough format 1 messages,
 * then ID[10..0] are represented by EID[28..18].
 *
 * DLC is the number of bytes in the CAN frame. Valid DLC codes are
 * 0..8. Note that, depending on the underlying CAN controller, DLC
 * can be greater than 8. We do not hide this in the modules firmware,
 * but pass the responsibility to deal with this (mis-)feature to the
 * user.
 */
typedef struct {
	BYTE_t    unused;      /* driver internal flags */
	BYTE_t    cmd;         /* command specifier */
	BYTE_t    data[14];    /* data array */
} FastMessage;


/*** IOCTL commands *****************************************************/
						/* get information from device */
#define	_RD_(a)		__DIOF (_DCMD_MISC, (a), sizeof(Message) + 16)
						/* write information to device */
#define	_WR_(a)		__DIOT (_DCMD_MISC, (a), sizeof(Message) + 16)
						/* get/put information to device */
#define	_RW_(a)		__DIOTF(_DCMD_MISC, (a), sizeof(Message) + 16)
						/* only command */
#define	_OC_(a)		__DION (_DCMD_MISC, (a))

					/* getstat:			*/
#define DPM_READ_MBOX		_RD_(128)	/* function code READ     	*/
#define DPM_COPY_FROM		_RW_(129)	/* get data from dpm		*/
#define DPM_READ_FAST_MBOX	_RD_(130)	/* function code READ FAST    	*/
#define DPM_DRV_IDVERS          _RD_(606)	/* get driver version 		*/

					/* putstat:			*/
#define DPM_WRITE_MBOX		_WR_(528)	/* function code WRITE mid prio	*/
#define DPM_WRITE_MBOX_HI	_WR_(539)	/* function code WRITE high prio*/
#define DPM_WRITE_MBOX_LOW	_WR_(540)	/* function code WRITE low prio	*/
#define DPM_COPY_TO		_WR_(529)	/* copy data to dpm		*/
#define DPM_INIT_NEW_HOSTIF	_WR_(533)	/* init new hostif		*/
#define DPM_INIT_NEW_HOSTIF_PRIO _WR_(545)	/* init new hostif (priorized)  */
#define DPM_INIT_FAST_CAN	_WR_(535)	/* init fast CAN access		*/
#define DPM_INIT_FAST_CAN_PRIO	_WR_(551)	/* init fast CAN access         */
#define DPM_WRITE_FAST_CAN	_WR_(536)	/* send trougth fast interface  */
#define DPM_WRITE_FAST_CAN_PRIO _WR_(552)	/* send trougth prioritized	*/
						/* fast interface  		*/
#define DPM_DEINIT_FAST_CAN	_WR_(537)	/* deinit fast CAN access	*/
#define DPM_INIT_ID_MSG_Q_TABLE _WR_(541)	/* Allocates memory for table,	*/
						/* where message-queue-Ids for	*/
						/* fast can access are stored.	*/
#define DPM_DEINIT_ID_MSG_Q_TABLE   _WR_(542)	/* deallocates memory for table	*/
#define DPM_ADD_ID_TO_MSG_Q_TABLE   _WR_(543)	/* adds queue entry to ID-table	*/
#define DPM_REM_ID_FROM_MSG_Q_TABLE _WR_(544)	/* removes entry from ID-table	*/
#define DPM_INIT_L2_ROUTING	_WR_(546)	/* inits the Layer2 routing cap	*/
#define DPM_DEINIT_L2_ROUTING	_WR_(547)	/* deinits Layer2 routing cap	*/
#define DPM_INIT_ROUTE_ID	_WR_(548)	/* lets _one_ ID being routable	*/
#define DPM_ADD_ROUTE_TO_ID	_WR_(549)	/* adds _one_ routing way to ID	*/
#define DPM_MAP_FD_TO_MOD	_WR_(550)	/* map the fd to board/module	*/

#define DPM_REGISTER_APC	_WR_(603)	/* Register APC function 	*/
#define DPM_OBJDIC_REQ          _WR_(605)	/* access to object dict. 	*/

#define DPM_WRITE_PP		_WR_(607)	/* write CANopen process picture*/
#define DPM_READ_PP		_RW_(608)	/* read CANopen process picture */
					/* command:			*/
#define	DPM_RESET		_OC_(532)	/* reset the module		*/
#define DPM_POLL_ENABLE		_OC_(553)	/* switch driver to polling mode*/
#define DPM_TPU_REQ             _OC_(604)	/* initiates TPU request IR 	*/
#define DPM_INIT_SAFE_PP	_OC_(609)	/* init new hostif (priorized)  */

#define	DAS_IST_EIN_TEST	_RW_(700)
#define	DAS_IST_EIN_TEST2	_RW_(701)

struct dpm_copy_desc {
        unsigned int	dpm;
        int		len;
	unsigned char	buffer[256];
};

struct dpm_rw_can_desc {
	int		rval;
	Message		pm;
};

struct dpm_new_hostif_desc {
        int             fromhost_len;
        int             tohost_len;
};

struct dpm_new_hostif_desc_prio {
        int             fromhost_hi_len;
        int             fromhost_low_len;
        int             fromhost_len;
        int             tohost_len;
};

struct dpm_fast_can_desc {
        int             fromhost_len;
        int             tohost_len;
};

struct dpm_fast_can_desc_prio {
        int             numOfPrioQueues;
        int             fromhost_len;
        int             tohost_len;
};

struct dpm_write_fast_can_desc {
        int             rval;
        FastMessage	pm;
};

struct dpm_write_fast_can_desc_prio {
        int             rval;
        int             prioQueue;
        FastMessage     pm;
};

struct dpm_msg_q_id_desc {
	unsigned short	id;
};

struct dpm_layer_2_routing_desc {
	int	rval;
	int	Id;
	int	max_routes;
	int	source_Id;
	int	dest_Id;
	int	dest_Board;
	int	dest_Modul;
};

struct dpm_objdic_desc {
        int     objdic_index;
        int     objdic_subindex;
        int     access_type;
        void    *entry_structure;
};

struct dpm_readpp_desc {
        int     dataSize;
        int     offset;
        char    buffer[8];
};
 
struct dpm_writepp_desc {
        int     dataSize;
        int     offset;
        char    buffer[8];
};

#define TS_TO_HOST      1       /* read modules timestamp counter */
                                /* of last SYNC message */
#define TS_TO_MODULE    0       /* synchronizing module by writing timestamp */


#ifdef INKERNEL
/************************************************************************/
/* DEFINES								*/
/************************************************************************/

/* Check for definiton of both BIG and LITTLE */
#if defined(BIG_ENDIAN) && defined(LITTLE_ENDIAN)
#error "mismatch in endian definitions (1)"
#endif

/* Define BUS_* macro if not already done by the user */
#if defined(BIG_ENDIAN) && !defined(BUS_LITTLE_ENDIAN)
#if !defined(BUS_BIG_ENDIAN)
#define BUS_BIG_ENDIAN
#endif
#endif
#if defined(LITTLE_ENDIAN) && !defined(BUS_BIG_ENDIAN)
#if !defined(BUS_LITTLE_ENDIAN)
#define BUS_LITTLE_ENDIAN
#endif
#endif

/* Check if our sanity resulted in an invalid combination */
#if defined(BUS_BIG_ENDIAN) && defined(BUS_LITTLE_ENDIAN)
#error "mismatch in endian definitions (2)"
#endif


#define	MAXNDEV		1		/* I/O-channel per module	*/
#define	BOARDTYP	(1<<7)		/* d7=1 VMOD-IG, d7=0 VMOD-IO	*/
#define	I_ENABLE	1

/* MICAN3 */
#define DPM_PAGEWIDTH	256		/* ICAN3			*/

#ifdef BUS_BIG_ENDIAN
struct dpmw {			/* structure for BIG_ENDIAN		*/
	unsigned char	unused;
	unsigned char	spec;		/* message specifier		*/
	unsigned char	len_l;		/* data length low byte		*/
	unsigned char	len_h;		/* data length high byte	*/
	unsigned char	data[MSGLEN];	/* raw data			*/
	unsigned char	page_sel;	/* page selector		*/
	unsigned char	gap1;
	unsigned char	intgen;		/* wr: generate inter. on ICAN3	*/ 
					/* rd: clear MODULbus interrupt	*/
	unsigned char	gap2;
	unsigned char	reset;		/* generate reset on ICAN3	*/
	unsigned char	gap3;
	unsigned char	tpureq;		/* signal to tpu on ICAN3	*/
};
#endif

#ifdef BUS_LITTLE_ENDIAN
struct dpmw {			/* structure for LITTLE_ENDIAN		*/
	unsigned char	spec;		/* message specifier		*/
	unsigned char	unused;
	unsigned char	len_h;		/* data length high byte	*/
	unsigned char	len_l;		/* data length low byte		*/
	unsigned char	data[MSGLEN];	/* raw data			*/
	unsigned char	gap1;
	unsigned char	page_sel;	/* page selector		*/
	unsigned char	gap2;
	unsigned char	intgen;		/* wr: generate inter. on ICAN3	*/ 
					/* rd: clear MODULbus interrupt	*/
	unsigned char	gap3;
	unsigned char	reset;		/* generate reset on ICAN3	*/
	unsigned char	gap4;
	unsigned char	tpureq;		/* signal to tpu on ICAN3	*/
};
#endif


#define START_BUFF	9		/* first free page within DPM */

/* New stylish host interface */

#ifdef BUS_BIG_ENDIAN
struct dpmw_desc {
        unsigned char   control;        /* control byte */
        unsigned char   buffer;         /* buffer pointer */
};
#endif

#ifdef BUS_LITTLE_ENDIAN
struct dpmw_desc {
        unsigned char   buffer;         /* buffer pointer */
        unsigned char   control;        /* control byte */
} __attribute__ ((packed));
#endif

#define DPM_DESC_VALID          (1<<7)
#define DPM_DESC_WRAP           (1<<6)
#define DPM_DESC_INTR           (1<<5)
#define DPM_DESC_IVALID         (1<<4)
#define DPM_DESC_LEN            (7<<0)


/* Fast dpmqueue host interface */

#define FDPMQUEUE_LEN	14		/* data elements in each buffer */


#ifdef BUS_BIG_ENDIAN
struct fdpmw_desc {
	unsigned char control;
	unsigned char spec;
	unsigned char data[FDPMQUEUE_LEN];
};
#endif

#ifdef BUS_LITTLE_ENDIAN
struct fdpmw_desc {
	unsigned char spec;
	unsigned char control;
	unsigned char data[FDPMQUEUE_LEN];
} __attribute__ ((packed));
#endif

#define FDPM_DESC_VALID		(1<<7)
#define FDPM_DESC_WRAP		(1<<6)
#define FDPM_DESC_IVALID	(1<<4)

/* Definition of fast-queue-fromhost priorization */
#define FDPMQUEUE_FROMHOST_NO_INIT      0
#define FDPMQUEUE_FROMHOST_1_PRIO       1
#define FDPMQUEUE_FROMHOST_N_PRIO       3

#ifdef BUS_BIG_ENDIAN
#define DPMW_MSYNC_LOCL_ADDR	0x1
#define DPMW_MSYNC_PEER_ADDR	0x0
#define DPMW_TARGET_RUN_ADDR	0x2
#define TIME_STAMP_0_ADDR       0x10
#define TIME_STAMP_1_ADDR       0x11
#define TIME_STAMP_2_ADDR       0x12
#define TIME_STAMP_3_ADDR       0x13
#endif

#ifdef BUS_LITTLE_ENDIAN
#define DPMW_MSYNC_LOCL_ADDR	0x0
#define DPMW_MSYNC_PEER_ADDR	0x1
#define DPMW_TARGET_RUN_ADDR	0x3
#define TIME_STAMP_0_ADDR       0x12
#define TIME_STAMP_1_ADDR       0x13
#define TIME_STAMP_2_ADDR       0x10
#define TIME_STAMP_3_ADDR       0x11
#endif

/* end of defined MICAN3 */

/* MICAN2 */

#ifdef BUS_BIG_ENDIAN
struct dpm {			/* structure for MOTOROLA type		*/
	unsigned char	page_sel;	/* page selector		*/
	unsigned char	unused;
	unsigned char	intgen;		/* wr: generate inter. on ICAN2	*/ 
					/* rd: clear MODULbus interrupt	*/
	unsigned char	spec;		/* message specifier		*/
	unsigned char	reset;		/* generate reset on ICAN2	*/
	unsigned char	len_l;		/* data length low byte		*/
	unsigned char	gap0;
	unsigned char	len_h;		/* data length high byte	*/
	unsigned char	gap_1;
	unsigned char	data[MSGLEN * 2];	/* raw data		*/
};
#endif

#ifdef BUS_LITTLE_ENDIAN
struct dpm {			/* structure for INTEL-LIKE type        */
	unsigned char	unused;
	unsigned char	page_sel;	/* page selector		*/
	unsigned char	spec;		/* message specifier		*/
	unsigned char	intgen;		/* wr: generate inter. on ICAN2	*/ 
					/* rd: clear MODULbus interrupt	*/
	unsigned char	len_l;		/* data length low byte		*/
	unsigned char	reset;		/* generate reset on ICAN2	*/
	unsigned char	len_h;		/* data length high byte	*/
	unsigned char	gap0;
	unsigned char	data[MSGLEN * 2];	/* raw data		*/
	unsigned char	gap_1;
};
#endif

/* New stylish host interface */

#ifdef BUS_BIG_ENDIAN
struct dpm_desc {
        unsigned char   gap1;
        unsigned char   control;        /* control byte */
        unsigned char   gap2;
        unsigned char   buffer;         /* buffer pointer */
};
#endif

#ifdef BUS_LITTLE_ENDIAN
struct dpm_desc {
        unsigned char   control;        /* control byte */
        unsigned char   gap1;
        unsigned char   buffer;         /* buffer pointer */
        unsigned char   gap2;
};
#endif

#define DPM_DESC_VALID          (1<<7)
#define DPM_DESC_WRAP           (1<<6)
#define DPM_DESC_INTR           (1<<5)
#define DPM_DESC_IVALID         (1<<4)
#define DPM_DESC_LEN            (7<<0)


/* Fast dpmqueue host interface */

#define FDPMQUEUE_LEN 14

#ifdef BUS_BIG_ENDIAN
struct fdpm_desc {
	unsigned char gap1;
	unsigned char control;
	unsigned char gap2;
	unsigned char spec;
	unsigned char gap3;
	unsigned char data[2*FDPMQUEUE_LEN-1];
};
#endif

#ifdef BUS_LITTLE_ENDIAN
struct fdpm_desc {
	unsigned char control;
	unsigned char gap1;
	unsigned char spec;
	unsigned char gap2;
	unsigned char data[2*FDPMQUEUE_LEN-1];
	unsigned char gap3;
};
#endif

#ifdef BUS_BIG_ENDIAN
#define DPM_MSYNC_LOCL_ADDR		0x3
#define DPM_MSYNC_PEER_ADDR		0x1
#define DPM_TARGET_RUN_ADDR		0x5
#endif

#ifdef BUS_LITTLE_ENDIAN
#define DPM_MSYNC_LOCL_ADDR		0x2
#define DPM_MSYNC_PEER_ADDR		0x0
#define DPM_TARGET_RUN_ADDR		0x4
#endif

/* swap macro for MOTOROLA <-> INTEL access on ICAN3 */
#ifdef BUS_LITTLE_ENDIAN
#define SWAP_WORD(x) (((x & 0xff) << 8) | (0xff & (x >> 8)))
#endif
#ifdef BUS_BIG_ENDIAN
#define SWAP_WORD(x) (x)
#endif

#endif /* #ifdef INKERNEL */

#define TOHOST_DPM_QUEUE_PAGE		5 /* tohost_dpmqueue page */
#define FROMHOST_DPM_QUEUE_PAGE		6 /* fromhost_dpmqueue page prio mid */
#define FROMHOST_DPM_QUEUE_HI_PAGE	7 /* fromhost_dpmqueue page prio high */
#define FROMHOST_DPM_QUEUE_LOW_PAGE	8 /* fromhost_dpmqueue page prio low */
#define START_BUFF			9 /* first free page within DPM */


/*----------routing specific defines ------------------------*/ 

/* CAN Identifiers used by CAL (cf. CiA/DS204-1, Annex I) */
#define ID_START_STOP		   0	/* Node Start, Stop, Disconnect */
#define ID_CMS_NIL		   0	/* invalid CMS Id */
#define ID_CMS_MIN		   1	/* range of CMS   */
#define ID_CMS_MAX		1760	/*    identifiers */
#define ID_GUARD_NIL		   0	/* invalid guard Id */
#define ID_GUARD_MIN		1761	/* range of guarding */
#define ID_GUARD_MAX		2015	/*    identifiers    */
#define ID_LMT_S		2020	/* from LMT Slave */
#define ID_LMT_M		2021	/* from LMT Master */
#define ID_NMT_IDENTIFY		2022	/* Identify Node Protocol */
#define ID_DBT_S		2023	/* from DBT Slave */
#define ID_DBT_M		2024	/* from DBT Master */
#define ID_NMT_S		2025	/* from NMT Slave */
#define ID_NMT_M		2026	/* from NMT Master */
#define ID_SELFTEST		2027	/* for module selftest */
#define ID_MIN			   0	/* range of identifiers */
#define ID_MAX			2031	/*    controlled by CiA */

#define	MAX_ROUTES_PER_ID		(ID_MAX * 2)
					/* should be enough... */

#define	NIL_ROUT		(struct rout_target *) 0
					/* NIL pointer for "no routing" */

#define	NIL_NIL_ROUT		(struct rout_target **) 0
					/* NIL pointer for "no routing" */

#define NIL_DPM_DEV		(DPM_DEV *) -1
					/* NIL pointer for "no routing" */

#define BLANC			0xffff	/* empty / last entry indicator in */
					/* rout_target->target_id struct */
					/* member */
					
/* prioritized fast-fromhost-queue specific defines/structures */
 
struct fromHostFastDpmQPrioEntry {
        unsigned short                          priority;
        volatile struct fdpmw_desc              *fhfdpmQueue;
        int                                     fhfdpmQueueP;
        int                                     fhfdpmQueueStartP;
        struct fromHostFastDpmQPrioEntry        *next;
};
#define NIL_FHFDQ_PRIO_ENTRY (struct fromHostFastDpmQPrioEntry *)0

#ifdef __cplusplus
}
#endif

#endif /* __INCdpmh */

