/* ntcan.h
**
**            Copyright (c) 2001 by electronic system design gmbh
**
**  This software is copyrighted by and is the sole property of 
**  esd gmbh.  All rights, title, ownership, or other interests
**  in the software remain the property of esd gmbh. This
**  software may only be used in accordance with the corresponding
**  license agreement.  Any unauthorized use, duplication, transmission,  
**  distribution, or disclosure of this software is expressly forbidden.
**
**  This Copyright notice may not be removed or modified without prior
**  written consent of esd gmbh.
**
**  esd gmbh, reserves the right to modify this software without notice.
**
**  electronic system design gmbh          Tel. +49-511-37298-0
**  Vahrenwalder Str 205                   Fax. +49-511-37298-68 
**  30165 Hannover                         http://www.esd-electronics.com
**  Germany                                slaes@esd-electronics.com
**
**
*/

#ifndef _ntcan_h_
#define _ntcan_h_

#ifndef EXPORT
#define EXPORT
#endif

#ifndef CALLTYPE
#define CALLTYPE
#endif

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/*------------------ Defines ------------------------------------------------*/
#define NTCAN_EV_BASE 0x40000000
#define NTCAN_EV_USER 0x40000080
#define NTCAN_EV_LAST 0x400000FF

#define NTCAN_20B_BASE 0x20000000

#define NTCAN_RTR      0x10
#define NTCAN_NO_DATA  0x20

#define NTCAN_EV_CAN_ERROR NTCAN_EV_BASE


/* mode in canOpen() */
#define NTCAN_MODE_LOM          0x00010000 /* mode-mask for listening only mode */
#define NTCAN_MODE_OBJECT       0x10000000 /* mode-mask for flat mode           */
#define NTCAN_MODE_OVERLAPPED   0x20000000 /* reserved for windows, only        */

/* Queue-Size in canOpen() */
#define NTCAN_MAX_TX_QUEUESIZE 0x4000
#define NTCAN_MAX_RX_QUEUESIZE 0x4000

#define NTCAN_NO_QUEUE        -1                 /* Open for Rx object mode        */


/* No baudrate configured (returned from canGetBaudrate() )*/
#define NTCAN_NO_BAUDRATE         0x7FFFFFFF
  
#define NTCAN_ERRNO_BASE             0x00000100
/*------------------- error-codes--------------------------------------------*/
#define NTCAN_SUCCESS                0
#define NTCAN_INVALID_PARAMETER      EINVAL 
#define NTCAN_INVALID_HANDLE         ENOENT 
#define NTCAN_NET_NOT_FOUND          ENOENT  
#define NTCAN_INSUFFICIENT_RESOURCES ENOMEM
#define NTCAN_OPERATION_ABORTED      EINTR

#define NTCAN_RX_TIMEOUT             (NTCAN_ERRNO_BASE +1)
#define NTCAN_TX_TIMEOUT             (NTCAN_ERRNO_BASE +2)

#define NTCAN_TX_ERROR               (NTCAN_ERRNO_BASE +4)
#define NTCAN_CONTR_OFF_BUS          (NTCAN_ERRNO_BASE +5)
#define NTCAN_CONTR_BUSY             (NTCAN_ERRNO_BASE +6)
#define NTCAN_CONTR_WARN             (NTCAN_ERRNO_BASE +7)

#define NTCAN_NO_ID_ENABLED          (NTCAN_ERRNO_BASE +9)
#define NTCAN_ID_ALREADY_ENABLED     (NTCAN_ERRNO_BASE +10)
#define NTCAN_ID_NOT_ENABLED         (NTCAN_ERRNO_BASE +11)
   
#define NTCAN_INVALID_FIRMWARE       (NTCAN_ERRNO_BASE +13)
#define NTCAN_MESSAGE_LOST           (NTCAN_ERRNO_BASE +14)
#define NTCAN_INVALID_HARDWARE       (NTCAN_ERRNO_BASE +15)
#define NTCAN_PENDING_WRITE          (NTCAN_ERRNO_BASE +16)
#define NTCAN_PENDING_READ           (NTCAN_ERRNO_BASE +17)
#define NTCAN_INVALID_DRIVER         (NTCAN_ERRNO_BASE +18)

    
typedef int OVERLAPPED;
typedef int HANDLE;
  
#pragma pack(1)

typedef struct
{
  long          id;             /* can-id                                   */
  unsigned char len;            /* length of message: 0-8                   */
  unsigned char msg_lost;       /* count of lost rx-messages                */
  unsigned char reserved[2];    /* reserved                                 */
  unsigned char data[8];        /* 8 data-bytes                             */
} CMSG;


typedef struct
{
  long          evid;          /* event-id: possible range:EV_BASE...EV_LAST */
  unsigned char len;           /* length of message: 0-8                     */
  unsigned char reserved[3];   /* reserved                                   */
  union
  {
    unsigned char  c[8];
    unsigned short s[4];
    unsigned long  l[2];
  } evdata;
} EVMSG;

typedef union
  {
    unsigned long long tick;
#if defined(__BIGENDIAN__)
    struct
    {
      unsigned long HighPart;
      unsigned long LowPart;
    } h;
#endif /* __BIGENDIAN__ */
#if defined(__LITTLEENDIAN__)
    struct
    {
      unsigned long LowPart;
      unsigned long HighPart;
    } h;
#endif /* __LITTLEENDIAN__ */
} CAN_TIMESTAMP, CAN_TIMESTAMP_FREQUENCY;

typedef struct
{
  long          id;             /* can-id                                   */
  unsigned char len;            /* length of message: 0-8                   */
  unsigned char msg_lost;       /* count of lost rx-messages                */
  unsigned char reserved[2];    /* reserved                                 */
  unsigned char data[8];        /* 8 data-bytes                             */
  CAN_TIMESTAMP timestamp;      /* time stamp of this message               */
} CMSG_T;



typedef struct
{
  unsigned short hardware;
  unsigned short firmware;
  unsigned short driver;
  unsigned short dll;
  unsigned long  boardstatus;
  unsigned char  boardid[14];
  unsigned short features;
} CAN_IF_STATUS;

# pragma pack()

#define NTCAN_FEATURE_FULL_CAN       (1<<0)   /*! Marks Full-CAN-controller, used by nucleus for flat-mode,... */
#define NTCAN_FEATURE_CAN_20B        (1<<1)   /*! CAN 2.OB support                                             */
#define NTCAN_FEATURE_LOM            (1<<2)   /*! feature/mode flag for listening-only-mode                    */
#define NTCAN_FEATURE_CYCLIC_TX      (1<<3)   /*!                                                              */
#define NTCAN_FEATURE_OBJECT_MODE    (1<<4)   /*! DIFFERS FROM MODE FLAG: flat mode                            */
#define NTCAN_FEATURE_TIMESTAMP      (1<<5)   /*! feature flag for timestamping                                */ 


/*
 * IOCTL codes for canIoctl()
 */
#define NTCAN_IOCTL_FLUSH_RX_FIFO        0x0001   /* Flush Rx FIFO                 */
#define NTCAN_IOCTL_GET_RX_MSG_COUNT     0x0002   /* Ret # CMSG in Rx FIFO         */
#define NTCAN_IOCTL_GET_RX_TIMEOUT       0x0003   /* Ret configured Rx tout        */
#define NTCAN_IOCTL_GET_TX_TIMEOUT       0x0004   /* Ret configured Tx tout        */
#define NTCAN_IOCTL_SET_20B_HND_FILTER   0x0005   /* Configure 20B filter          */
#define NTCAN_IOCTL_GET_SERIAL           0x0006   /* Get HW serial number          */
#define NTCAN_IOCTL_GET_TIMESTAMP_FREQ   0x0007   /* Get timestamp frequency in Hz */
#define NTCAN_IOCTL_GET_TIMESTAMP        0x0008   /* Get timestamp counter         */


/*...prototypes..............................................................*/
EXPORT int canSetBaudrate(HANDLE handle,                /* nt-handle         */
			  unsigned long baud );         /* baudrate-constant */

/*...prototypes..............................................................*/
EXPORT int canGetBaudrate(HANDLE handle,              /* nt-handle           */
			  unsigned long *baud );      /* Out: baudrate       */
/*...........................................................................*/
EXPORT int CALLTYPE canOpen(             
	        int           net,          /* net number                    */
		unsigned long flags,        /*                               */
                long          txqueuesize,  /* nr of entries in message queue*/
                long          rxqueuesize,  /* nr of entries in message queue*/
                long          txtimeout,    /* tx-timeout in miliseconds     */
                long          rxtimeout,    /* rx-timeout in miliseconds     */
		HANDLE        *handle );    /* out: nt-handle                */
/*...........................................................................*/
EXPORT int   CALLTYPE canClose(	 /* ret NTCAN_OK, if success         */
                HANDLE        handle );  /* handle                           */
/*...........................................................................*/
EXPORT int    CALLTYPE      canIdAdd(  /* ret NTCAN_OK, if success         */
                HANDLE        handle,    /* read handle                      */
                long          id  );     /* can identifier                   */
/*...........................................................................*/
EXPORT int    CALLTYPE canIdDelete(    /* ret 0, if success                */
                HANDLE        handle,    /* read handle                      */
                long          id  );     /* can identifier                   */
/*...........................................................................*/
EXPORT int    canGetOverlappedResult(
                HANDLE        handle,    /* handle                           */
		OVERLAPPED    *ovrlppd,  /* overlapped-structure             */
		long          *len,      /* out: cnt of available CMSG-Buffer*/
		int           bWait );   /* FALSE =>do not wait, else wait   */        
/*...........................................................................*/
EXPORT int    CALLTYPE canTake(        /* ret 0, if success                */
	        HANDLE        handle,    /* handle                           */
                CMSG          *cmsg,     /* ptr to data-buffer               */
                long          *len );    /* out: size of CMSG-Buffer         */
                                         /* in:  count of read messages      */
/*...........................................................................*/
EXPORT int    CALLTYPE canRead(        /* ret 0, if success                */
                HANDLE        handle,    /* handle                           */
                CMSG          *cmsg,     /* ptr to data-buffer               */
                long          *len,      /* out: size of CMSG-Buffer         */ 
                                         /* in:  count of read messages      */
                OVERLAPPED    *ovrlppd); /* NULL or overlapped-structure     */
/*...........................................................................*/
EXPORT int    CALLTYPE canSend(        /* ret 0, if success                */
                HANDLE        handle,    /* handle                           */
                CMSG          *cmsg,     /* ptr to data-buffer               */
                long          *len );    /* size of CMSG-Buffer              */ 
/*...........................................................................*/
EXPORT int    CALLTYPE canWrite(       /* ret 0, if success                */
                HANDLE        handle,    /* handle                           */
                CMSG          *cmsg,     /* ptr to data-buffer               */
                long          *len,      /* size of CMSG-Buffer              */ 
                OVERLAPPED    *ovrlppd); /* NULL or overlapped-structure     */
/*...........................................................................*/
EXPORT int    CALLTYPE canReadEvent(
                HANDLE        handle,    /* handle                           */
                EVMSG         *evmsg,    /* ptr to event-msg-buffer          */
                OVERLAPPED    *ovrlppd); /* NULL or overlapped-structure     */
/*...........................................................................*/
EXPORT int    CALLTYPE canSendEvent(
                HANDLE        handle,    /* handle                           */
                EVMSG         *evmsg );  /* ptr to event-msg-buffer          */
/*...........................................................................*/
EXPORT int    CALLTYPE canStatus(
                HANDLE        handle,    /* handle                           */
                CAN_IF_STATUS *cstat );  /* ptr to can-status-buffer         */
/*...........................................................................*/
EXPORT int CALLTYPE canIoctl(
                HANDLE        handle,    /* handle                           */
		unsigned long ulCmd,     /* Command                          */
		void *        pArg );    /* Ptr to command specific argument */
/*...........................................................................*/
EXPORT int    CALLTYPE canTxObj(         /* ret 0, if success                */
                HANDLE        handle,    /* handle                           */
                CMSG          *cmsg,     /* ptr to data-buffer               */
                long          *len );    /* size of CMSG-Buffer              */ 
                
EXPORT int CALLTYPE canTakeT(
                HANDLE        handle,    /* In: handle                       */
                CMSG_T        *cmsg_t,   /* In: ptr to receive buffer        */
                long          *len );    /* out: size of CMSG_T-Buffer       */
                                         /* in:  count of read messages      */
/*...........................................................................*/
EXPORT int CALLTYPE canReadT(
                HANDLE        handle,    /* In: handle                       */
                CMSG_T        *cmsg_t,   /* Int ptr to receive buffer        */
                long          *len,      /* out: size of CMSG_T-Buffer       */
                                         /* in:  count of read messages      */
                OVERLAPPED    *ovrlppd); /* In: always NULL                  */


#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* _ntcan_h_ */

