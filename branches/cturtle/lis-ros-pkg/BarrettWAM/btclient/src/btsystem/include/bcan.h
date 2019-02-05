/*-----------------------------------------------------------------------------
bcan.h -- Interface to Basic CAN Controller Chip PCA82C200

Copyright (c) 1994 JANZ Computer AG
All Rights Reserved

Created 94/10/11 by Soenke Hansen
Version 1.20 of 00/07/19

Prototypes for functions in bcan.c.
Macros for descriptor word, CAN identifiers, RTR bits, and data length codes.
Defintions of register addresses and values for the Basic CAN controller.

-----------------------------------------------------------------------------*/

#ifndef bcan_DEFINED
#define bcan_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

#include "defs.h"
#include "msg.h"

/*-------- COBs CAN Communication Objects ----------------------------------*/

/* COBs (CAN Communication Objects) consists of the following data:
	WORD_t	desc;
	BYTE_t	data[8];
   The format of the descriptor word follows that of the PCA82C200 controller:
	desc >> 5	is the 11-bit identifier
	desc & 0x0010	is set iff the RTR bit is set
	desc & 0x000f	is the data length code DLC
   There are at most 8 data bytes in data[] (DLC <= 8).
*/

/* RTR bit in COB descriptor */
#define COBD_RTR	0x0010

/* Get RTR bit from COB descriptor */
#define cobd_get_rtr(desc) ((desc) & COBD_RTR)

/* Get data length code from COB descriptor */
#define cobd_get_dlc(desc) ((desc) & 0x000f)

/* Get real data length */
#define cobd_get_len(desc) (((desc) & COBD_RTR) ? 0 : \
				(((desc) & 0x08) ? 8 : (desc) & 0x07))

/* Get id and RTR bit from COB descriptor */
#define cobd_get_id_rtr(desc) ((desc) & 0xfff0)

/* Get RTR bit and data length code from COB descriptor */
#define cobd_get_rtr_dlc(desc) ((desc) & 0x001f)

/* Get identifier from COB descriptor */
#define cobd_get_id(desc) ((desc) >> 5)	/* numerical value of id */

/* Shift identifier for oring into COB descriptor */
#define cobd_id_to_desc(id) ((id) << 5)

/* Set COB descriptor from Id, and data length code, with or without RTR */
#define cobd_set(id, dlc)    (((id) << 5) | (0x000f & (WORD_t)(dlc)))
#define cobd_setrtr(id, dlc) (((id) << 5) | COBD_RTR | (0x000f & (WORD_t)(dlc)))

/* Handling of several CAN controllers and their base-address-offset */
#define ccNo2ccOff(ccNo)	(ccNo << 11)
#define ccOff2ccNo(ccOff)	(ccOff >> 11)

/*-------- Definitions of registers ----------------------------------------*/

/*
 * 82C200 and SJA1000 in BasicCAN Mode
 */

/* PCA82C200 Address Allocation */
#define BCAN_CR		 0	/* control register */
#define BCAN_CMR	 1	/* command register */
#define BCAN_SR		 2	/* status register */
#define BCAN_IR		 3	/* interrupt register */
#define BCAN_AC		 4	/* acceptance code register */
#define BCAN_AM		 5	/* acceptance mask register */
#define BCAN_BT0	 6	/* bus timing register 0 */
#define BCAN_BT1	 7	/* bus timing register 1 */
#define BCAN_OCR	 8	/* output control register */
#define BCAN_TDESC1	10	/* first descriptor transmit buffer */
#define BCAN_TDESC2	11	/* second descriptor transmit buffer */
#define BCAN_TDATA	12	/* start data transmit buffer */
#define BCAN_RDESC1	20	/* first descriptor receive buffer */
#define BCAN_RDESC2	21	/* second descriptor receive buffer */
#define BCAN_RDATA	22	/* start data receive buffer */
#define BCAN_CDR	31	/* clock divider */


/* Control Register Bits */
#define BCAN_CR_OIE	0x10	/* Overrun Interrupt Enable */
#define BCAN_CR_EIE	0x08	/* Error Interrupt Enable */
#define BCAN_CR_TIE	0x04	/* Transmit Interrupt Enable */
#define BCAN_CR_RIE	0x02	/* Receive Interrupt Enable */
#define BCAN_CR_RR 	0x01	/* Reset Request */

/* Command Register Bits */
#define BCAN_CMR_GTS	0x10	/* Goto Sleep */
#define BCAN_CMR_COS	0x08	/* Clear Overrun Status */
#define BCAN_CMR_RRB	0x04	/* Release Receive Buffer */
#define BCAN_CMR_AT	0x02	/* Abort Transmission */
#define BCAN_CMR_TR	0x01	/* Transmission Request */

/* Status Register Bits */
#define BCAN_SR_BS	0x80	/* Bus Status */
#define BCAN_SR_ES	0x40	/* Error Status */
#define BCAN_SR_TS	0x20	/* Transmit Status */
#define BCAN_SR_RS	0x10	/* Receive Status */
#define BCAN_SR_TCS	0x08	/* Transmission Complete Status */
#define BCAN_SR_TBS	0x04	/* Transmit Buffer Status */
#define BCAN_SR_DO	0x02	/* Data Overrun */
#define BCAN_SR_RBS	0x01	/* Receive Buffer Status */

/* Interrupt Register Bits */
#define BCAN_IR_WUI	0x10	/* Wake-Up Interrupt */
#define BCAN_IR_OI	0x08	/* Overrun Interrupt */
#define BCAN_IR_EI	0x04	/* Error Interrupt */
#define BCAN_IR_TI	0x02	/* Transmit Interrupt */
#define BCAN_IR_RI	0x01	/* Receive Interrupt */


/*
 * JSA1000 in PeliCAN mode
 */

/* PeliCAN mode address allocation */
#define PCAN_MODR        0      /* Mode register (rw) */
#define PCAN_CMR         1	/* Command register (wo) */
#define PCAN_SR          2      /* Status register (ro) */
#define PCAN_IR          3	/* Interrupt register (ro) */
#define PCAN_IER         4      /* Interrupt enable register (rw) */
#define PCAN_BTR0        6	/* Bus timing register 0 (ro, rw) */
#define PCAN_BTR1        7	/* Bus timing register 1 (ro, rw) */
#define PCAN_OCR         8	/* Output control register 1 (ro, rw) */
#define PCAN_TESTR       9	/* Test register */
#define PCAN_ALCR       11	/* Arbitration lost capture reg (ro) */
#define PCAN_ECCR       12      /* Error code capture register (ro) */
#define PCAN_EWLR       13      /* Error warning limit register (ro, rw) */
#define PCAN_RXERR      14	/* Rx error counter register (ro, rw) */
#define PCAN_TXERR      15      /* Tx error counter register (ro, rw) */
#define PCAN_ACR0	16	/* acceptance code register 0 (-, rw) */
#define PCAN_ACR1	17	/* acceptance code register 1 (-, rw) */
#define PCAN_ACR2	18	/* acceptance code register 2 (-, rw) */
#define PCAN_ACR3	19	/* acceptance code register 3 (-, rw) */
#define PCAN_AMR0	20	/* acceptance mask register 0 (-, rw) */
#define PCAN_AMR1	21	/* acceptance mask register 1 (-, rw) */
#define PCAN_AMR2	22	/* acceptance mask register 2 (-, rw) */
#define PCAN_AMR3	23	/* acceptance mask register 3 (-, rw) */
#define PCAN_RXFI       16      /* Rx Frame info   SFF, EFF (ro, -) */
#define PCAN_RXID1      17      /* Rx Identifier 1 SFF, EFF (ro, -) */
#define PCAN_RXID2      18      /* Rx Identifier 2 SFF, EFF (ro, -) */
#define PCAN_RXID3      19      /* Rx Identifier 3      EFF (ro, -) */
#define PCAN_RXID4      20      /* Rx Identifier 4      EFF (ro, -) */
#define PCAN_RXSFFD     19      /* Rx standard frame data   (ro, -) */
#define PCAN_RXEFFD     21      /* Rx extended frame data   (ro, -) */
#define PCAN_TXFI       16      /* Tx Frame info   SFF, EFF (wo, -) */
#define PCAN_TXID1      17      /* Tx Identifier 1 SFF, EFF (wo, -) */
#define PCAN_TXID2      18      /* Tx Identifier 2 SFF, EFF (wo, -) */
#define PCAN_TXID3      19      /* Tx Identifier 3      EFF (wo, -) */
#define PCAN_TXID4      20      /* Tx Identifier 4      EFF (wo, -) */
#define PCAN_TXSFFD     19      /* Tx standard frame data   (wo, -) */
#define PCAN_TXEFFD     21      /* Tx extended frame data   (wo, -) */
#define PCAN_RXMCR      29	/* Rx message counter register (ro) */
#define PCAN_RXBSAR     30	/* Rx buffer start address register (ro, rw) */
#define PCAN_CDR	31	/* Clock divider register ('rw', rw)*/

#define PCAN_RXFI_RAM	96	/* RAM mirror of RXFI */
#define PCAN_TXFI_RAM	96	/* RAM mirror of TXFI */
#define PCAN_TXID1_RAM  97      /* RAM mirror Tx Identifier 1 SFF, EFF  */
#define PCAN_TXID2_RAM  98      /* RAM mirror Tx Identifier 2 SFF, EFF  */
#define PCAN_TXID3_RAM  99      /* RAM mirror Tx Identifier 3      EFF  */
#define PCAN_TXID4_RAM  100     /* RAM mirror Tx Identifier 4      EFF  */
#define PCAN_TXSFFD_RAM 99      /* RAM mirror Tx standard frame data    */
#define PCAN_TXEFFD_RAM 101     /* RAM mirror Tx extended frame data    */


/* Mode Register Bits */
#define PCAN_MODR_SM    (1<<4)	/* Sleep mode */
#define PCAN_MODR_AFM   (1<<3)	/* Acceptance filter mode */
#define PCAN_MODR_STM   (1<<2)	/* Self test mode */
#define PCAN_MODR_LOM   (1<<1)	/* Listen only mode */
#define PCAN_MODR_RM    (1<<0)	/* Reset mode */

/* Command Register Bits */
#define PCAN_CMR_SRR	(1<<4)	/* Self reception request */
#define PCAN_CMR_CDO	(1<<3)	/* Clear data overrun */
#define PCAN_CMR_RRB	(1<<2)	/* Release receive buffer */
#define PCAN_CMR_AT	(1<<1)	/* Abort transmission */
#define PCAN_CMR_TR	(1<<0)	/* Transmission request */

/* Status Register Bits */
#define PCAN_SR_BS	(1<<7)	/* Bus status */
#define PCAN_SR_ES	(1<<6)	/* Error status */
#define PCAN_SR_TS	(1<<5)	/* Transmit status */
#define PCAN_SR_RS	(1<<4)	/* Receive status */
#define PCAN_SR_TCS	(1<<3)	/* Transmission complete status */
#define PCAN_SR_TBS	(1<<2)	/* Transmit buffer status */
#define PCAN_SR_DOS	(1<<1)	/* Data overrun status */
#define PCAN_SR_RBS	(1<<0)	/* Receive buffer status */

/* Interrupt Register Bits */
#define PCAN_IR_BEI	(1<<7)	/* Bus-eror interrupt */
#define PCAN_IR_ALI	(1<<6)	/* Arbitration lost interrupt */
#define PCAN_IR_EPI	(1<<5)	/* Error-passive interrupt */
#define PCAN_IR_WUI	(1<<4)	/* Wake-up interrupt */
#define PCAN_IR_DOI	(1<<3)	/* Data-overrun interrupt */
#define PCAN_IR_EI	(1<<2)	/* Error interrupt */
#define PCAN_IR_TI	(1<<1)	/* Transmit interrupt */
#define PCAN_IR_RI	(1<<0)	/* Receive interrupt */

/* Interrupt enable register bits */
#define PCAN_IER_BEIE	(1<<7)	/* Bus-eror interrupt enable */
#define PCAN_IER_ALIE	(1<<6)	/* Arbitration lost interrupt enable */
#define PCAN_IER_EPIE	(1<<5)	/* Error-passive interrupt enable */
#define PCAN_IER_WUIE	(1<<4)	/* Wake-up interrupt enable */
#define PCAN_IER_DOIE	(1<<3)	/* Data-overrun interrupt enable */
#define PCAN_IER_EIE	(1<<2)	/* Error warning interrupt enable */
#define PCAN_IER_TIE	(1<<1)	/* Transmit interrupt enable */
#define PCAN_IER_RIE	(1<<0)	/* Receive interrupt enable */

/* Clock divider register bits */
#define PCAN_CDR_PCAN	(1<<7)	/* Enable PCAN mode */
#define PCAN_CDR_CBP	(1<<6)	/* Enable Rx input comparator bypass */
#define PCAN_CDR_RXINTEN (1<<5)	/* Enable RXINT output at TX1 */
#define PCAN_CDR_CLKOFF	(1<<3)	/* Disable clock output */

/* Bit definitions for Rx/Tx Frame info */
#define PCAN_FINFO_FF	(1<<7)	/* Frame format bit */
#define PCAN_FINFO_EFF	(1<<7)	/* Extended frame indication */
#define PCAN_FINFO_RTR	(1<<6)	/* RTR frame bit */
#define PCAN_FINFO_DLC_MASK (0x0f) /* Data length code mask */



/*
 * BasicCAN, PeliCAN common definitions.
 */

/* Values of acceptance code/mask registers */
#define ACM_ALL	word_from_bytes(0x00,0xff)	/* accept all ids */

/* Values of output control register */
#define OCR_PUSHPULL	0xfa	/* push/pull */

/* Admissable bus timing values (BTR1=msb, BTR0=lsb) */
#ifdef OSC_16MHZ		/* if using a 16 MHz oscillator */
#define BTR_1MB		0x2300	/* 1 MBaud */
#if 0
#define BTR_1MB		0x1400	/* 1 MBaud */	 /* The better value */
#endif
#define BTR_800KB	0x2500	/* 800 KBaud */
#define BTR_500KB	0x1c00	/* 500 KBaud */
#define BTR_250KB	0x1c01	/* 250 KBaud */
#define BTR_125KB	0x1c03	/* 125 KBaud */
#define BTR_100KB	0x34c7	/* 100 KBaud */
#define BTR_62_5KB	0xbac7	/*  62.5 KBaud */
#define BTR_50KB	0x34cf	/*  50 KBaud */
#define BTR_20KB	0x7fcf	/*  20 KBaud */
#define BTR_10KB	0x7fdf	/*  10 KBaud */
#endif

/* Admissable values in clock divider register */
#ifdef OSC_16MHZ		/* if using a 16 MHz oscillator */
#define CDR_16MHZ	0x07	/* 16 MHz output */
#define CDR_8MHZ	0x00	/* 8 MHz output (division by 2) */
#define CDR_4MHZ	0x01	/* 4 MHz output (division by 4) */
#define CDR_2MHZ	0x03	/* 2 MHz output (division by 8) */
#endif

/*-------- Gloal variables in bcan.c ---------------------------------------*/

/* Indicates that we are using SJA1000 in pelican mode */
extern short pcan_mode;

/* ----------------------- BulkBuffer-Stuff --------------------------*/
/* must be greater than length of biggest message type!!! */
#define BULK_LEFT_FREE			14	

#define DEFAULT_MAX_BULK_BUF_BYTE_COUNT	MSGLEN
/* we need a minimum size of bulk buffer to generate messages */
/* of a certain format (not bounde, but allocated buffer!) */
/* to satisfy the message-handling inside the asm isr */
#define DEFAULT_MIN_BULK_BUF_BYTE_COUNT	20
#define DEFAULT_BULK_PREALLOC_COUNT	20

/* Timer value for elapsed (e.g.) ms to determine */
/* if it is time to send current BulkBuffer. Will */
/* be maintained in timer.c */
extern LWORD_t bulkBufTim;

void configBulkBuffer(unsigned short, unsigned char, unsigned char);
#define BULK_MS_MSG_LOST				0x80
#define BULK_MS_UNKNOWN					0xff

/* ----------------------- SniffBuffer-Stuff --------------------------*/
/* must be greater than length of biggest message type!!! */
#define SNIFF_LEFT_FREE			26	

#define DEFAULT_MAX_SNIFF_BUF_BYTE_COUNT	MSGLEN
/* we need a minimum size of sniff buffer to generate messages */
/* of a certain format (not bounde, but allocated buffer!) */
/* to satisfy the message-handling inside the asm isr */
#define DEFAULT_MIN_SNIFF_BUF_BYTE_COUNT	20
#define DEFAULT_SNIFF_PREALLOC_COUNT	20

/* Timer value for elapsed (e.g.) ms to determine */
/* if it is time to send current SniffBuffer. Will */
/* be maintained in timer.c */
extern LWORD_t sniffBufTim;

void configSniffBuffer(unsigned short, unsigned char, unsigned char);
void configSniffBufferEcho(unsigned short);
/* valid values for echo2sniff, configured by configSniffBufferEcho: */
#define SNIFF_ECHO_FROM_CANLOOK		(1 << 0)
#define SNIFF_ECHO_FROM_PLAINQ		(1 << 1)
#define SNIFF_ECHO_FROM_FASTQ		(1 << 2)
#define SNIFF_RX_FROM_CAN			(1 << 3)


/* message-commands inside sniff-message */
#define SNIFF_MS_ECHO_FROM_PLAIN_STD	3
#define SNIFF_MS_ECHO_FROM_PLAIN_XTD	4	/* not yet used */
#define SNIFF_MS_ECHO_FROM_FAST_STD		5
#define SNIFF_MS_ECHO_FROM_FAST_XTD		6
#define SNIFF_MS_ECHO_FROM_CANLOOK_STD	7
#define SNIFF_MS_ECHO_FROM_CANLOOK_XTD	8
#define SNIFF_MS_CAN_RX_STD				9
#define SNIFF_MS_CAN_RX_XTD				10
#define SNIFF_MS_MSG_LOST				0x80
#define SNIFF_MS_UNKNOWN				0xff


/* -----------------------Busload-Statistic-Stuff --------------------------*/
#define BUS_LOAD_STAT_SEND_TO_HOST		(1 << 0)
#define BUS_LOAD_STAT_INTEGR_TIME_ELAPSED	(1 << 1)
#define BUS_LOAD_STAT_ENABLED	(1 << 7)

#define	BUS_LOAD_REQUEST_ALL	0
#define	BUS_LOAD_REQUEST_STD	1
#define	BUS_LOAD_REQUEST_XTD	2
#define	BUS_LOAD_REQUEST_RX		3
#define	BUS_LOAD_REQUEST_TX		4

void resetBusLoad(void);
void configBusLoad(WORD_t, BYTE_t);
void sendBusLoadToHost(void);
void busLoadUpdate(void); 
void checkBusLoadStatistic(void); 

void resetBusLoadSniff(void);
void configBusLoadSniff(WORD_t, BYTE_t);
void sendBusLoadToHostSniff(void);
void busLoadUpdateSniff(void); 
void checkBusLoadStatisticSniff(void); 

void setInterpretCcNo(BYTE_t);
BYTE_t getInterpretCcNo(void);

/*-------- Functions in bcan.c ---------------------------------------------*/

/* Switch to bus-on state */
extern void buson_bcan(WORD_t);

/* Reset: Switch to bus-off state */
extern void busoff_bcan(WORD_t);

/* Set bus timing registers (assumed state: bus-off) */
extern void write_btr_bcan(WORD_t, WORD_t);

/* Set error waring limit */
void write_ewl_bcan(WORD_t, BYTE_t ewl);

/* Control bus-error reporting */
void switch_berr_bcan(WORD_t, BYTE_t state);

/* provide access to berrs_to_go outside bcan.c */
BYTE_t getBerrsToGo(WORD_t);
 
/* Control listen only mode */
void switch_lom_bcan(WORD_t, BYTE_t state);

/* Control self test mode */
void switch_stm_bcan(WORD_t, BYTE_t state);

/* Set hardware acceptance filter */
extern void write_acm_bcan(WORD_t);

/* Set extended hardware acceptance filter */
void write_ext_acm_bcan(WORD_t, int mode, LWORD_t ac, LWORD_t am);

/* Abort current transmission */
extern void sendabort_bcan(void);

/* Flush the transmit queue. */
extern void flush_tx_bcan(void);

/* Initialize the BCAN controller.  The controller is left in the bus-off
   state.  To start communication buson_bcan() must be called.
   All interrupt sources are accepted.  All messages pass the acceptance
   filter.  The bus timing and the clock divider registers are initialized
   with the parameters to the function.  */
extern void init_bcan(WORD_t, WORD_t, WORD_t, WORD_t, WORD_t);

/* Service of BCAN message pool -- to be called cyclically */
extern void serv_bcan(void);

/* Interrupt service for PCA82C200. */
extern void isr_bcan(void);
#ifdef ASSHACK
extern void a_isr_bcan(void);
extern void a_isr_bcan_end(void);
#endif


/* Request to send a CAN message: Fill the transmit buffer of the
   CAN chip and issue a transmit request.
   Return values:	 1  okay, transmit request accepted by controller
			 0  remote request successful
			-1  transmit request failed (no buffer, no mem)
*/
extern int sendreq_bcan(
	int,		/* transmit request specifier */
	WORD_t,		/* request id */
	WORD_t,		/* descriptor (id, rtr, dlc) */
	BYTE_t	*	/* data bytes */
);

#ifdef __cplusplus
}
#endif

#endif /* !bcan_DEFINED */
