/* ======================================================================== *
 *  Module ............. experimental
 *  File ............... btcan_janz.c
 *  Creation Date ...... 24 Mar 2003
 *  Author ............. Brian Zenowich
 *  Addtl Authors ...... Traveler Hauptman
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  Copyright (C) 2003-2008 Barrett Technology, Inc. <support@barrett.com>
 *                          625 Mount Auburn St
 *                          Cambridge, MA 02138, USA
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY BARRETT TECHNOLOGY, INC AND CONTRIBUTORS
 *  ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL BARRETT
 *  TECHNOLOGY, INC OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  The views and conclusions contained in the software and documentation
 *  are those of the authors and should not be interpreted as representing
 *  official policies, either expressed or implied, of Barrett Technology.
 *                                                                          *
 * ======================================================================== */

/** \file btcan.c
    Handles all communication with the robot over the CAN bus.
    Requires library files "libcan.a" and "libmitop.a".

 */


/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#include <stdio.h>
#include <errno.h>
#include <sys/syspage.h>
#include <inttypes.h>
#include <pthread.h>
#include <syslog.h>

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#include "icanif.h"
#include "can_lib.h"
#include "bcan.h"
#include "btcan.h"

/*==============================*
 * PRIVATE DEFINED constants    *
 *==============================*/
#define STDQ_RX_BUFFERS     50
#define STDQ_TX_BUFFERS     10
#define FASTQ_RX_BUFFERS  2000
#define FASTQ_TX_BUFFERS   500

#define mbxID               (0)
#define BASE_ID             (1)
#define MAX_BUS             (4)

/*==============================*
 * PRIVATE MACRO definitions    *
 *==============================*/
#define isAlpha(c) ( ((c >= 'A') && (c <= 'Z')) ? 1 : 0 )
#define isSpace(c) ( (c == ' ') ? 1 : 0 )
#define isDigit(c) ( ((c >= '0') && (c <= '9')) ? 1 : 0 )

#define ADDR2NODE(x) ((((x) >> 5) & 0x001F) - BASE_ID)
#define NODE2ADDR(x) (((mbxID + BASE_ID) << 5) | ((x) + BASE_ID))
#define GROUPID(n)   (((mbxID + BASE_ID) << 5) | (0x0400 + (n)))
#define BROADCAST    (GROUPID(0))

#define Border(Value,Min,Max)  (Value<Min)?Min:((Value>Max)?Max:Value)

/*==============================*
 * PRIVATE typedefs and structs *
 *==============================*/
/* (internal) This keeps data that is local to each instance of the ICS server */
typedef struct local_info_t {
    int         icfd;            /* handle for ICAN device */
    int         fd0;             /* handle for output device (read) */
    int         fd;              /* handle for output device (write)*/
    pthread_t   ican_task;       /* task id for message thread */
    pthread_t   fjob_task;       /* test job : task id */
    int         fjob_arg[16];    /* test job : arguments */
    int         interpret;       /* Should use 2controller handling */
    uint64_t    cps;             /* Cycles Per Second on this machine */
} local_info_t ;

/*==============================*
* GLOBAL file-scope variables  *
*==============================*/
local_info_t    priv_data;
int             canDev[MAX_BUS];
pthread_mutex_t commMutex;

int dataTypeWAMPuck[] =
{
    /* Common properties */
    /* VERS */ S08,
    /* ERR  */ S08,
    /* STAT */ S08,
    /* MODE */ S08,
    /* ADDR */ S16,
    /* VALU */ S16,

    /* WAMPUCK-specific properties */
    /* T    */    S16,
    /* MT   */    S16,
    /* AP   */    S24,
    /* TETM */    S16,
    /* IDX  */    S16,
    /* ZERO */    S16,
    /* OFST */    S16,
    /* TEMP */    S16,
    /* MTMP */    S16,
    /* ATOD */    S16,
    /* IOFB */    S16,
    /* IOFC */    S16,
    /* GRPA */    S08,
    /* GRPB */    S08,
    /* GRPC */    S08,
    /* PIDX */    S08,
    /* CTS */     S16,
    /* POFF */    S24
};

int dataTypeWAMSafety[] =
{

    /* Common properties */
    /* VERS */ S08,
    /* ERR  */ S08,
    /* STAT */ S08,
    /* MODE */ S08,
    /* ADDR */ S16,
    /* VALU */ S16,

    /* WAMGOD-specific properties */
    /* BUSS */ S16,
    /* BUSE */ S16,
    /* LBSE */ S16,
    /* RELY */ S08,
    /* HBMS */ S16,
    /* CONT */ S16,
    /* MENT */ S16,
    /* MJV  */ S16,
    /* MTV  */ S16,
    /* MEV  */ S16,
    /* IF   */ S16
};

/*==============================*
 * PRIVATE Function Prototypes  *
 *==============================*/
long getFilesize( FILE *fp );
int can_msg(int bus, WORD_t lId, BYTE_t lLen, BYTE_t *lData);
static int CAN_FAST_SEND(int fhandle, int ctrl, FastMessage *p);
static int CAN_SEND(int fhandle, int ctrl, Message *p);
int parseMessage(int id, int len, unsigned char *messageData, int *node, int *property, long *value);
int readCAN(int fd, unsigned char *d, int *id, int *len, int *dataStart);
int compile(int property, long longVal, unsigned char *data, int *dataLen, int isForGOD);

/*==============================*
 * Functions                    *
 *==============================*/
void freeCAN(int bus)
{
    // Free the memory used by CAN subsystem
}

/** Checks whether an array index is out of bounds.
    If the array index is out of bounds, logs the error in syslog.

*/
void Validate(
    int val     /** The array index value */,
    int min     /** The lower bound allowed for the index */,
    int max     /** The upper bound allowed for the index */,
    char *str   /** A string containing the name of the array being validated */)
{
    if ((val < min) || (val > max))
    {
        syslog(LOG_ERR,"%s array index is out of bounds. Value %d, Bounds %d to %d",str,val,min,max);
    }
}

/** Sends a regular message on the CAN bus.
    Allows selection of the CAN controller.

    \return The function returns the number of message sent, or -1 when the system call failed. The return value 0 determines
that no message could be sent, probably because there was no space in the targeted queue. This function does does
not block in such a case, so you need to loop explicitly until the message is sent.

    \note For now, does not retry can_send() in case of failure
*/
static int CAN_SEND(
    int fhandle     /** Handle to the CAN device */,
    int ctrl        /** Controller number (0 or 1 on a two-port card) */,
    Message *p      /** A Message structure containing the message to be sent */)
{
    int res;

    p->control = ctrl;
    res = can_send(fhandle, p);
    if (res != 1)
        syslog(LOG_ERR,"can_send returned %d",res);

    return res;
}

/** Sends a fast message on the CAN bus.
    Allows selection of the CAN controller.

    \return The function returns the number of message sent, or -1 when the system call failed. The return value 0 determines
that no message could be sent, probably because there was no space in the targeted queue. This function does does
not block in such a case, so you need to loop explicitly until the message is sent.

    \note For now, does not retry can_fast_send() in case of failure
*/
static int CAN_FAST_SEND(
    int fhandle     /** Handle to the CAN device */,
    int ctrl        /** Controller number (0 or 1 on a two-port card) */,
    FastMessage *p  /** A FastMessage structure containing the message to be sent */)
{
    int res;
    p->cmd |= ((ctrl & 0x7) << 4);
    res = can_fast_send(fhandle, p);
    if (res != 1)
        syslog(LOG_ERR,"can_fast_send returned %d",res);
    return res;
}

/** Parse the data payload received from a Barrett Motor Controller.
    Allows selection of the CAN controller.

    \return 0 for no error
    \return 1 for <illegal message header> (syslog output is generated)

   \verbatim
   THIS (OPTIONAL) INFO WILL BE PLACED IN A GREY BOX AS PRE-FORMATTED TEXT

     You may draw simple diagrams explaining key concepts this way:

              -----Head-----
              |            |
          --Node1--      Node2
          |       |
        Sub1     Sub2

   \endverbatim
*/
int parseMessage(
    /* Input */
    int id                      /** The message ID */,
    int len                     /** The data payload length */,
    unsigned char *messageData  /** Pointer to the message data payload */,
    
    /* Output */
    int *node       /** The controller node ID of the received message */,
    int *property   /** The property this message applies to */,
    long *value     /** The value of the property being processed */)
{
    int i;
    int dataHeader;

    *node = ADDR2NODE(id);
    if (*node == -1)
        syslog(LOG_ERR,"msgID:%x ",id);
    dataHeader = (messageData[0] >> 6) & 0x0003;
    messageData[0] &= 0x3F;
    //syslog(LOG_ERR,"Entering parsemessage");
    switch (dataHeader)
    {
        case 3:  /* Data is a packed 22-bit position, SET */
            *value = 0x00000000;
            *value |= ( (long)messageData[0] << 16) & 0x003F0000;
            *value |= ( (long)messageData[1] << 8 ) & 0x0000FF00;
            *value |= ( (long)messageData[2] ) & 0x000000FF;

            if (*value & 0x00200000) /* If negative */
                *value |= 0xFFC00000; /* Sign-extend */

            *property = ACTUAL_POSITION;
            //syslog(LOG_ERR,"Received packed set property: %d from node: %d value:%d",*property,*node,*value);
            break;
        case 2:  /* Data is normal, SET */
            *property = messageData[0] & 0x3F;
            //syslog(LOG_ERR, "Received property: %d", *property);
            /* Store the value, second byte of message is zero (for DSP word alignment) */
            *value = 0;
            for (i = 0; i < len - 2; i++)
                *value |= ((unsigned long)messageData[i + 2] << (i * 8))
            & (0x000000FF << (i * 8));

            if (*value & (1 << ((i*8) - 1)))
                *value |= 0xFFFFFFFF << (i * 8); /* Sign extend the value */

            //syslog(LOG_ERR, "Received normal set property: %d from node: %d value:%d", *property, *node, *value);
            //syslog(LOG_ERR,"parsemessage after %d",value);
            break;
        case 0:  /* Assume firmware request (GET) */
            *property = -(messageData[0] & 0x3F); /* A negative (or zero) property means GET */
            *value = 0;
            //syslog(LOG_ERR, "Received normal get property: %d from node: %d value:%d", *property, *node, *value);
            break;
        default:
            syslog(LOG_ERR, "<Illegal Message Header> %d\n", dataHeader);
            return(1);
        }
    //if (*property != 8) syslog(LOG_ERR,"Value in parsemessage is: %d",*value);
    return (0);

}

/** Initialize the CAN card.
    Configures the CAN card to use the new host interface, fast messages,
    120 Ohm termination, and 1MBaud bit rate.

    \return 0 for no error
    \return non-zero in the event of an error

*/
int initCAN(int bus)
{
    int   err;
    char device[16];

    /* Pointer to allocated private data */
    local_info_t *priv = &priv_data;

    /* Variable used for VMOD-ICAN communication */
    Message msg;

    // Set up device string
    sprintf(device, "/dev/dpm_0%d", bus);
    
    priv->cps = SYSPAGE_ENTRY(qtime)->cycles_per_sec;
    //printf("cycles_per_sec = %lld\n", priv->cps);
    priv->fd0 = 0;  /* stdin */
    priv->fd = 1;  /* stdout */


    /* connect to the ICAN module. */
    if ((priv->icfd = can_open(device)) < 0)
    {
        return 1;
    }

    /* Switch to the new host interface */
    ican2_select_hostif(priv->icfd, STDQ_RX_BUFFERS, STDQ_TX_BUFFERS);

    /* Initialize the fast CAN interface */
    err = ican2_init_fast_can(priv->icfd, FASTQ_RX_BUFFERS, FASTQ_TX_BUFFERS);
    if(err != 0)
    {
        syslog(LOG_ERR,"ican2_init_fast_can: err = %d", err);
        return 1;
    }

    syslog(LOG_ERR, "<Buffer config: Rx=%d Tx=%d RxFast=%d TxFast=%d>",
           STDQ_RX_BUFFERS, STDQ_TX_BUFFERS,
           FASTQ_RX_BUFFERS, FASTQ_TX_BUFFERS);

    /* Switch all identifiers to fast-queue. */
    IcRangeSetAfil(&msg, 0, 2047, 2);
    CAN_SEND(priv->icfd, 0, &msg);

    /* Set the bit-rate. */
    IcWriteBtrBCAN(&msg, BTR_1MB);
    CAN_SEND(priv->icfd, 0, &msg);

    /* Set the internal termination to 120 Ohms. */
    SwitchCanTermination(&msg,1);
    if (CAN_SEND(priv->icfd, 0, &msg) <= 0)
    {
        syslog (LOG_ERR,"can_device_setup : Unable to switch termination");
        return (-6);
    }

    /* Switch to bus-on. */
    IcBusOnBCAN(&msg);
    CAN_SEND(priv->icfd, 0, &msg);

    //IcSwitchBerrBCAN(&msg, 1);
    //CAN_SEND(priv->icfd, 0, &msg);

    canDev[bus] = priv->icfd;
    usleep(200000);
    return (0);

}

/** Read a message from the CAN card.
    Waits for a message to be received, then reads the message from the CAN card.

    \return 0 for no error
    \return non-zero in the event of an error

    \warning Sometimes the read() call indicates a FAST_MESSAGE has been received,
         but can_recv_fast() fails to read a message.

*/
int readCAN(
    int fd              /** File descriptor for the CAN card */,
    unsigned char *d    /** Character buffer for the incoming message */,
    int *id             /** The received message's messageID */,
    int *len            /** The length (in bytes) of the data payload */,
    int *dataStart      /** An index into the character buffer pointing to the start of the data payload */)
{
    unsigned char index,*dat;
    Message msg;
    int ctrlNum;
    int CANlook_flag;
    int Command_spec;
    int num_rcvd;
    int desc, rtr, ext;
    int n, err;
    char buff[80];
    int i;

    /* Initialize the character buffer to all-zero (for debugging) */
    for(i = 0; i < 12; i++)
    {
        d[i] = 0;
    }

    /* Sleep until a message arrives */
    n = read(fd, &index, 1);

    if (n != 1) /* If the number of characters read from the device != 1 */
    {
        if (errno == ECANCELED) /* If the read() call was cancelled */
        {
            syslog(LOG_ERR, "read() returned ECANCELED.  Read %d bytes from fd %d: value[0] = %d", n, fd, index);
            return (10000);
        }
        syslog(LOG_ERR, "<readCAN read failed! errno=%d> %s\n", errno,strerror(errno));
        syslog(LOG_ERR, "<readCAN read failed! read %d bytes from fd %d: value[0] = %d", n, fd, index);
    }

    //syslog(LOG_ERR,"index = %d cnt = %d FAST_QUEUE = %d Plain QUEUE = %d",index,cnt,FAST_QUEUE,PLAIN_QUEUE);
    /* Determine the type of message received (fast/plain queue) */
    switch (index)
    {
        case FAST_QUEUE:
            num_rcvd = can_recv_fast(fd, (FastMessage *)&d[0]);
            if (num_rcvd == 0)  //No message was in the queue
            {
                syslog(LOG_ERR, "readCAN: FAST_QUEUE: Nothing to read");
                return (1000);
            }
            else if (num_rcvd == -1)
            {
                syslog(LOG_ERR, "readCAN: FAST_QUEUE: Read failed");
                return (1000);
            }
            else
            {
                if (num_rcvd > 1) syslog(LOG_ERR, "readCAN: FAST_QUEUE: Read more than the expected 1 message");
                /* Extract controller number and blank bits for further processing */
                ctrlNum = (d[1] & 0x70) >> 4;
                CANlook_flag = (d[1] >> 8) & 0x01 ;
                Command_spec = (d[1] & 0x0f);

                if ( Command_spec == 0 )
                {
                    /* Message format type 0 */
                    desc = (d[2] << 8) | d[3];
                    *id = (desc >> 5) & 0x07ff;
                    rtr = (desc >> 4) & 1;
                    *len = desc & 0x000f;
                    //syslog(LOG_ERR, "desc %x, id %x, rtr %x, len %x",desc,*id,rtr,*len);

                    *dataStart = 4; /* Start of the message data in the CAN frame */

                    //n = 0;
                    //for(i=0; i<12; i++)
                    //{
                    //    n += sprintf(&buff[n],"%2.2x ",d[i]);
                    //}
                    //buff[n]='\0';
                    //syslog(LOG_ERR, "ID %d Len %d Readdata %s with %d messages",*id,*len,buff,num_rcvd);
                }
                else if ( Command_spec == 1 )
                {  /* Message format type 1 (SJA1000) */
                    ext  = (d[2] >> 7) & 0x01;
                    rtr  = (d[2] >> 6) & 0x01;
                    *len  =  d[2]       & 0x0f;
                    *len = (*len > 8) ? (8) : (*len);

                    *dataStart = 8; /* Start of the message data in the CAN frame */

                    if(ext)
                    {
                        *id   = (d[4] << (24-3));
                        *id  += (d[5] << (16-3));
                        *id  += (d[6] << ( 8-3));
                        *id  += (d[7] >>     3 ) & (0xff>>3);

                        syslog(LOG_ERR, "Unexpected Extended CAN frame received: ID=%d, LEN=%d, DATA=%s",  *id, *len, buff);
                    }
                    else
                    {
                        *id   = (d[4] << 3);
                        *id  += (d[5] >> 5) & 0x07;

                        syslog(LOG_ERR, "Unexpected Regular CAN frame received: ID=%d, LEN=%d, DATA=%s",  *id, *len, buff);
                    }
                    n = 0;
                    for(i=0; i<12; i++)
                    {
                        n += sprintf(&buff[n],"%2.2x ",d[i]);
                    }
                    buff[n]='\0';
                    syslog(LOG_ERR, "unexpected fastCAN ID %d Len %d Data %s with num_rcvd=%d",*id,*len,buff,num_rcvd);
                    //if (*id == 8){ syslog(LOG_ERR, "caught it in extended frame!");}
                    //    return 1;
                }
                else
                {
                    //Command_spec = 5546;
                    syslog(LOG_ERR, "<readCAN FAST_QUEUE: Illegal command spec = %d>", Command_spec);
                    return 100;
                }
            }
            break;

        case PLAIN_QUEUE:
            if ((err = can_recv(fd, &msg)) > 0) //-1 = error, 0 = nothing waiting, n = number of messages
            {
                syslog(LOG_ERR, "PLAIN_QUEUE message: %d messages from fd %d", err, fd);
                dat = (unsigned char *)&msg;
                n = 0;
                for(i=0; i<12; i++)
                {
                    n += sprintf(&buff[n],"%2.2x ",dat[i]);
                }
                buff[n]='\0';
                syslog(LOG_ERR, "PLAIN_QUEUE data = %s",buff);

            }
            else
            {
                syslog(LOG_ERR, "plain queue unhandled message %d",err);
            }
            return (1);
            //handle_message(priv, &msg);
            break;

        default:
            syslog(LOG_ERR, "<Illegal index received from CAN hardware: %d>", index);
            return(10);
            //syslog(LOG_ERR, "default: read %d bytes from fd %d: value[0] = %d", n, fd, index);
        }
    //if (*id == 8){ syslog(LOG_ERR, "2caught it! with index %d, command_spec %d, cnt %d",index,Command_spec,cnt);}
    return (0);
}

/** Sets the property of a group of nodes to various different values.
    Waits for a message to be received, then reads the message from the CAN card.

    \return 0 for no error
    \return non-zero in the event of an error

    \note The SET operation is unverified.
    \note values[] must be at least 8 elements long

   \verbatim
     Special value-packing compilation: Packs (4) 14-bit values into 8 bytes
         0        1        2        3        4        5        6        7
     ATPPPPPP AAAAAAaa aaaaaaBB BBBBbbbb bbbbCCCC CCcccccc ccDDDDDD dddddddd

     A = Action (0 = Get, 1 = Set)
     T = Tight-Packed (0 = False, 1 = True)
     P = Property
     AA = UPPER 6 bits of value
     aa = lower 8 bits of value

   \endverbatim
*/
int setTorques(
    int bus,
    int group           /** Group number of the pucks to receive this message */,
    int *values         /** An array containing the values to be sent to the pucks */)
{
    int msgID;
    int cnt;
    unsigned char data[8];
    int err = 0;

    if(group < 0)
    {
        /* If we are using an enumerated value */
        msgID = GROUPID(-group);
    }
    else
    {
        /* We are using a non-enumerated value  for group */
        msgID = GROUPID(group);
    }

    /* Bound the torque commands */
    for (cnt = 0; cnt < 4; cnt++)
    {
        values[cnt] = Border(values[cnt], -8191, 8191);
    }

    /* Special value-packing compilation: Packs (4) 14-bit values into 8 bytes */
    /*     0        1        2        3        4        5        6        7    */
    /* ATPPPPPP AAAAAAaa aaaaaaBB BBBBbbbb bbbbCCCC CCcccccc ccDDDDDD dddddddd */

    data[0] = COMMANDED_TORQUE | 0xC0; /* Set the "Set" bit and "Packed" bit */
    data[1] = (unsigned char)(( values[0] >> 6) & 0x00FF);
    data[2] = (unsigned char)(((values[0] << 2) & 0x00FC) | ((values[1] >> 12) & 0x0003) );
    data[3] = (unsigned char)(( values[1] >> 4) & 0x00FF);
    data[4] = (unsigned char)(((values[1] << 4) & 0x00F0) | ((values[2] >> 10) & 0x000F) );
    data[5] = (unsigned char)(( values[2] >> 2) & 0x00FF);
    data[6] = (unsigned char)(((values[2] << 6) & 0x00C0) | ((values[3] >> 8) & 0x003F) );
    data[7] = (unsigned char)( values[3] & 0x00FF);

    /* Send the packet */
    if ((err = pthread_mutex_lock( &commMutex )) != EOK)
    {
        syslog(LOG_ERR, "setPacked: get mutex failed: %d", err);
    }

    can_msg(bus, msgID, 8, data);


    if ((err =  pthread_mutex_unlock( &commMutex)) != EOK)
    {
        syslog(LOG_ERR, "setPacked: unlock mutex failed: %d", err);
    }
    /* Recurse, if the WHOLE_ARM is required */
    if (group == WHOLE_ARM)
        err = setTorques(bus, UPPER_ARM, &values[4]);

    return (err);
}

/** Get the puck(s) out of RESET mode.
    Sets the puck(s) to STATUS_INITIALIZE, MODE_IDLE, without verification.
    Waits for pucks to come online.

    \return 0

*/
int wakePuck(
    int bus,
    int who             /** Which pucks to wake: individual puck ID or a GROUP() */)
{
    /* Set the puck's STATUS to STATUS_INITIALIZE */
    setProperty(bus, who, STATUS, FALSE, STATUS_INITIALIZE);
    usleep(50000); /* Wait 50ms for puck to come online */

    /* Set the puck's CONTROL_MODE to MODE_IDLE */
    setProperty(bus, who, CONTROL_MODE, FALSE, MODE_IDLE);
    usleep(50000); /* Wait 50ms for puck to come online */
}



/** Set a puck property to a specified value.
    Also capable of broadcasting a single value to GROUPs of pucks.

    \return 0 for success
    \return count of failed SET operations, otherwise

    \note The SET operation is VERIFIED (a GET is performed, and its response checked).

*/
int setProperty(
    int bus    /** Pointer to the CANdev_t structure */,
    int who             /** Which pucks to set: individual puck ID or a GROUP() */,
    int property        /** The property to set */,
    int verify,
    long value          /** The value to set the property to */)
{
    int dataLen;
    int i;
    unsigned char data[8];
    int msgID;
    long reply[MAX_NODES];
    int err = 0;
    int replyCt;

    if (who <= 0)
    { /* If this is a broadcast */
        /* Set the msgID */
        msgID = GROUPID( -who);

        /* Compile the command into a proper packet */
        compile(property, value, data, &dataLen, 0);

    }
    else
    { /* Regular (multiple) transmission */
        /* Set the msgID */
        msgID = NODE2ADDR(who);

        /* Compile the command into a proper packet */
        if (who == SAFETY_MODULE) /* If simply a GOD message */
            compile(property, value, data, &dataLen, 1); /* Let compile() know to use dataTypeWAMSafety[] */
        else
            compile(property, value, data, &dataLen, 0); /* Let compile() know to use dataTypeWAMPuck[] */
    }
    //syslog(LOG_ERR,"sending to: %d  -", who);

    /* Set the "Set" bit */
    data[0] |= 0x80; /* 10000000b */

    /* Send the packet */
    if ((err = pthread_mutex_lock( &commMutex )) != EOK)
    {
        syslog(LOG_ERR, "Set property mutex failed: %d", err);
    }

    can_msg(bus, msgID, dataLen, data);

    if ((err =  pthread_mutex_unlock( &commMutex)) != EOK)
    {
        syslog(LOG_ERR, "Set property: unlock mutex failed: %d", err);
    }

    //syslog(LOG_ERR, "setProperty after");
    //usleep(10000);

    if(verify)
    {
        /* Verify the transmission */
        err = 0;
        replyCt = getProperty(bus, who, property, reply);
        for (i = 0; i < replyCt; i++)
        {
            err += (reply[i] != value);
            if (err)
                syslog(LOG_ERR,"%d replies from set property were wrong. Property %d, Expected %ld Got %ld",err,property,value,reply[i]);
        }
    }
    return (err);

    /* err = can_msg(deviceHandle, channel,  node, msgLen, data) */
}

/** Get a property from the motor controllers (or GOD).
    Sends out a request for information to the puck(s), then waits for a response.

    \return number of replies received

*/
int getProperty(
    int bus,
    int who             /** Which pucks to get: individual puck ID or a GROUP() */,
    int property        /** The property to get */,
    long *reply         /** An array of the replies received, zero-based in order of ID */)
{
    int dataLen;
    int dataStart;
    int i, cnt;
    unsigned char data[21];
    int msgID;
    int err = 0;
    int node;
    int firstWho;
    long value;
    int expectedReplies;
    int rcvdProperty;

    dataLen = 1;
    if (who <= 0)
    { /* If this is a broadcast */
        /* Set the message ID */
        //syslog(LOG_ERR,"Broadcast group: %d",bcastGroup);
        msgID = GROUPID( -who);

        /* Set expectedReplies */
        switch (who)
        {
            case WHOLE_ARM:
                firstWho = 1;
                expectedReplies = 7;
                break;
            case LOWER_ARM:
                firstWho = 1;
                expectedReplies = 4;
                break;
            case UPPER_ARM:
                firstWho = 5;
                expectedReplies = 3;
                break;
            default:
                return ( -1);
            }

        /* Insert the property */
        data[0] = property;

        /* Set the "Broadcast" bit */
        data[0] |= 0x40;

        //syslog(LOG_ERR,"Broadcast send: msgID = %x, data = %x",msgID, data[0]);
    }
    else
    { /* Regular (single node) transmission */
        firstWho = who;

        /* Set expectedReplies */
        expectedReplies = 1;

        /* Set the property */
        data[0] = property;
        //syslog(LOG_ERR,"Single send: msgID = %x, data = %x",msgID, data[0]);
    }

    /* For each who */
    for (i = 0; i < expectedReplies; i++)
    {
        /* Set the message ID */
        msgID = NODE2ADDR(i + firstWho);

        /* Send the message */
        if ((err = pthread_mutex_lock( &commMutex )) != EOK)
        {
            syslog(LOG_ERR, "Get property mutex failed on %d read: %d: %s", i,err, strerror( err ));
        }

        can_msg(bus, msgID, dataLen, data);
        //syslog(LOG_ERR, "getProperty data before %d", value);
        /* Wait for the reply */

        while(readCAN(canDev[bus], data, &msgID, &dataLen, &dataStart))
        {
            err++;
        }
        if (err)
        {
            syslog(LOG_ERR, "ReadCAN failed in getProperty %d times", err);
        }
        //syslog(LOG_ERR, "getProperty msgID 0x%x, dataLen %d, dataStart  %d", msgID, dataLen, dataStart);
        parseMessage(msgID, dataLen, &data[dataStart], &node, &rcvdProperty, &value);

        if ((err =  pthread_mutex_unlock( &commMutex)) != EOK)
        {
            syslog(LOG_ERR, "Get property: unlock mutex failed: %d", err);
        }
        //syslog(LOG_ERR, "getProperty node %d, property %d, data  %d", node, property, value);
        reply[i] = value;
    }

    return (expectedReplies);
}

/** Get a property from a group of motor controllers.
    Sends out a request for information to the puck(s) in a group, then waits for a response.

    \return number of replies received

*/
int getPositions(
    int bus,
    int group           /** Group number of pucks whose property we want */,
    int num2wait4       /** Number of pucks in this group (number of responses to wait for) */,
    long *reply         /** An array of replies indexed by the Node ID */)
{
    int dataLen;
    int i;
    unsigned char data[16];
    int msgID;
    int err = 0;
    int node;
    long value;
    int expectedReplies;
    int dataStart;
    int rcvdProperty;
    int ret;

    dataLen = 1;

    /* Set the message ID */
    //syslog(LOG_ERR,"Broadcast group: %d",bcastGroup);
    if(group < 0)
    {
        /* If we are using an enumerated value */
        msgID = GROUPID(-group);
    }
    else
    {
        /* We are using a non-enumerated value  for group */
        msgID = GROUPID(group);
    }
    expectedReplies = num2wait4;
    data[0] = ACTUAL_POSITION;


    /* Send the message */
    if ((err = pthread_mutex_lock( &commMutex )) != EOK)
    {
        syslog(LOG_ERR, "Get position mutex failed: %d", err);
    }

    can_msg(bus, msgID, dataLen, data);

    /* Wait for the replies */
    err = 0;
    for (i = 0; i < expectedReplies; i++)
    {
        err = 0;
        while((ret = readCAN(canDev[bus], data, &msgID, &dataLen, &dataStart)) > 0)
        {
            err+= ret;
        }
        if (err)
        {
            syslog(LOG_ERR, "ReadCAN failed in getGroupPropertyFast: %d", err);
        }
        if (msgID == 8)
        {
            syslog(LOG_ERR, "getpropfast caught it!");
        }
        parseMessage(msgID, dataLen, &data[dataStart], &node, &rcvdProperty, &value);
        Validate(node,0,7,"Getpropertyfast reply");
        reply[node] = value;
        if ((node > 7) || (node < 1))
        {
            if (node > 7)
                syslog(LOG_ERR, "msgID %x Value %d Node %d from read %d", msgID,value, node, i);
            i--;
        }
    }

    if ((err =  pthread_mutex_unlock( &commMutex)) != EOK)
    {
        syslog(LOG_ERR, "Get group property fast: unlock mutex failed: %d", err);
    }

    return (expectedReplies);
}

/** Convert a property and value into a valid btcan packet.
    Used by getProperty() and setProperty() to build the data payload
    section of a CAN message based on a given property and value.

    \return 0 for success
    \return non-zero, otherwise

*/
int compile(
    int property        /** The property being compiled (use the enumerations in btcan.h) */,
    long longVal        /** The value to set the property to */,
    unsigned char *data /** A pointer to a character buffer in which to build the data payload */,
    int *dataLen        /** A pointer to the total length of the data payload for this packet */,
    int isForGOD        /** A flag indicating whether this packet is destined for the safety circuit or not */)
{
    int i;

    /* Insert the property */
    data[0] = property;
    data[1] = 0; /* To align the values for the tater's DSP */

    /* Append the value */
    for (i = 2; i < 6; i++)
    {
        data[i] = (char)(longVal & 0x000000FF);
        longVal >>= 8;
    }

    /* Record the proper data length */
    i = isForGOD ? dataTypeWAMSafety[property] : dataTypeWAMPuck[property];
    if(i > 0x00C0)
    {
        syslog(LOG_ERR,"bad property %d", property);
        return(1);
    }
    if (i & 0x0003)
    { /* 8-bits */
        *dataLen = 3;
    }
    else if (i & 0x000C)
    { /* 16-bits */
        *dataLen = 4;
    }
    else if (i & 0x0030)
    { /* 24-bits */
        *dataLen = 5;
    }
    else if (i & 0x00C0)
    { /* 32-bits */
        *dataLen = 6;
    }

    return (0);
}

/** Get the size of a file, in bytes.
    Used during firmware download to buffer the firmware *.tek file into memory.

    \return The size of the file, in bytes

*/
long getFilesize(
    FILE *fp    /** A pointer to a file */ )
{
    long int save_pos;
    long size_of_file;

    /* Save the current position. */
    save_pos = ftell( fp );

    /* Jump to the end of the file. */
    fseek( fp, 0L, SEEK_END );

    /* Get the end position. */
    size_of_file = ftell( fp );

    /* Jump back to the original position. */
    fseek( fp, save_pos, SEEK_SET );

    return ( size_of_file );
}

#if 0
/** Downloads new firmware to the motor controllers or GOD.
    Updates the firmware of the motor controllers or GOD via the CAN bus.

    \return 0 for success
    \return non-zero, otherwise

*/
int downloadFirmware(
    CANdev_t *CANdev    /** A pointer to the CANdev_t structure */,
    int who             /** Which pucks to update: individual puck ID or a GROUP() */,
    int newID           /** The new ID to assign to the puck during download (not valid with groups) */,
    char *fname         /** A string containing the pathname to a valid *.tek firmware file */)
{
    FILE *inFile;
    long fileSize;
    long firmwareSize;
    char *fileBuffer;
    char *bufferPtr;
    char *memPtr;
    char firmChar;

    unsigned char sendData[3];
    int sendMsgID;
    int sendDataLen;
    int expectedReplies;
    int i;

    unsigned char recvData[16];
    int recvMsgID;
    int recvDataLen;

    int node;
    int property;
    long value;

    int err = 0;

    int dataStart;

    unsigned char changeID[20];

    /* Open the firmware file */
    if (!(inFile = fopen(fname, "r")))
    {
        syslog(LOG_ERR, "Error: Failed to open file '%s'", fname);
        return (1);
    }

    /* Find the length of the input file */
    firmwareSize = getFilesize( inFile );

    /* Allocate enough memory for the file */
    memPtr = (char* )malloc( firmwareSize );
    if ( memPtr == NULL )
    {
        syslog(LOG_ERR, "downloadFirmware: malloc failed!");
        pthread_mutex_unlock( &commMutex );
        return (1);
    }

    /* Load firmware file into memory buffer */
    bufferPtr = memPtr;
    while ((*(bufferPtr++) = fgetc(inFile)) != EOF)
        ;

    /* Lock the CAN controller */
    pthread_mutex_lock( &commMutex );

    if (who == WAM_GOD) /* If upgrading GOD, send '0xAA' */
    {
        //printf("\nGOD download");

        sendData[2] = 0xAA;

        can_msg(bus, sendMsgID, sendDataLen, sendData);

    }
    else
    { /* Else, send [S][FIRMWARE][0x03] */
        //printf("\nController download");

        sendData[0] = 0x80 | FIRMWARE_VERSION; /* Mark as SET FIRMWARE */
        sendData[1] = 0x00;
        sendData[2] = 0x03; /* Firmware Key = 0x03 */
        sendDataLen = 3;

        if (who <= 0)
        { /* GROUP firmware broadcast */
            sendMsgID = GROUPID( -who);

            /* Set expectedReplies */
            switch (who)
            {
                case WHOLE_ARM:
                    expectedReplies = 7;
                    break;
                case LOWER_ARM:
                    expectedReplies = 4;
                    break;
                case UPPER_ARM:
                    expectedReplies = 3;
                    break;
                default:
                    pthread_mutex_unlock( &commMutex );
                    return ( -1);
                }
        }
        else
        {
            sendMsgID = NODE2ADDR(who);
            expectedReplies = 1;
        }

        can_msg(bus, sendMsgID, sendDataLen, sendData);
    }

    /* Update the puckID */
    if((newID > 0) && (who > 0))
    {
        sprintf(changeID, "126%02X80000100000%02X\n", 0x12 + newID, newID);
        bufferPtr = fileBuffer = changeID;
        fileSize = 19;
        while (bufferPtr < fileBuffer + fileSize)
        {
            firmChar = *(bufferPtr++);

            /* Skip non-alphanumeric data */
            if (!isAlpha(firmChar) && !isDigit(firmChar))
                continue;

            //printf("\rDownload Progress: %.1f%%\t\t", (100.0 * (bufferPtr-fileBuffer)) / fileSize);
            //printf("%c", firmChar);
            //fflush(stdout);

            /* Wait for the replies */
            for (i = 0; i < expectedReplies; i++)
            {
                err += readCAN(canDev[bus], recvData, &recvMsgID, &recvDataLen, &dataStart);
                if (err)
                    printf("\nRead Error: %d\n", err);
                parseMessage(recvMsgID, recvDataLen, &recvData[dataStart], &node, &property, &value);
                if (property != -FIRMWARE_VERSION)
                {
                    printf("\nGot unexpected traffic!\n");
                    pthread_mutex_unlock( &commMutex );
                    return ( -1);
                }
            }


            /* Send a char */
            sendData[2] = firmChar;
            can_msg(bus, sendMsgID, sendDataLen, sendData);
        } /* while(!EOF) */

        printf("\nID updated to: %d", newID);
    }

    /* Show what we are doing */
    //printf("\nDownload %ld bytes to: %d", fileSize, who);

    /* Send the *.tek firmware file */
    bufferPtr = fileBuffer = memPtr;
    fileSize = firmwareSize;
    printf("\nDownload Progress: 0.0%%");
    while (bufferPtr < fileBuffer + fileSize)
    {
        firmChar = *(bufferPtr++);

        /* Skip non-alphanumeric data */
        if (!isAlpha(firmChar) && !isDigit(firmChar))
            continue;

        printf("\rDownload Progress: %.1f%%\t\t", (100.0 * (bufferPtr-fileBuffer)) / fileSize);
        //printf("%c", firmChar);
        fflush(stdout);

        /* Wait for the replies */

        for (i = 0; i < expectedReplies; i++)
        {
            err += readCAN(canDev[bus], recvData, &recvMsgID, &recvDataLen, &dataStart);
            if (err)
                printf("\nRead Error: %d\n", err);
            parseMessage(recvMsgID, recvDataLen, &recvData[dataStart], &node, &property, &value);
            if (property != -FIRMWARE_VERSION)
            {
                printf("\nGot unexpected traffic!\n");
                pthread_mutex_unlock( &commMutex );
                return ( -1);
            }
        }


        /* Send a char */
        sendData[2] = firmChar;
        can_msg(bus, sendMsgID, sendDataLen, sendData);
    } /* while(!EOF) */
    pthread_mutex_unlock( &commMutex ); /* Unlock the CAN controller */

    free( fileBuffer );
    printf("\nDownload Complete!\n");

    return (0);
}


/* Old-style firmware download */
int downloadFirmware_Old(
CANdev_t *CANdev    /** A pointer to the CANdev_t structure */,
int who             /** Which pucks to update: individual puck ID or a GROUP() */,
char *fname         /** A string containing the pathname to a valid *.tek firmware file */)
{
    FILE *inFile;
    long fileSize;
    char *fileBuffer;
    char *bufferPtr;
    char firmChar;

    unsigned char sendData[3];
    int sendMsgID;
    int sendDataLen;
    int expectedReplies;
    int i;

    unsigned char recvData[16];
    int recvMsgID;
    int recvDataLen;

    int node;
    int property;
    long value;

    int err = 0;

    int dataStart;

    /* Open the firmware file */
    if (!(inFile = fopen(fname, "r")))
    {
        syslog(LOG_ERR, "Error: Failed to open file '%s'", fname);
        return (1);
    }

    /* Lock the CAN controller */
    pthread_mutex_lock( &commMutex );

    if (who == WAM_GOD) /* If upgrading GOD, send '0xAA' */
    {
        //printf("\nGOD download");

        sendData[2] = 0xAA;

        can_msg(bus, sendMsgID, sendDataLen, sendData);

    }
    else
    { /* Else, send [S][FIRMWARE][0x03] */
        //printf("\nController download");

        sendData[0] = 0x40 | FIRMWARE_VERSION; /* Mark as SET FIRMWARE */
        sendData[1] = 0x80;
        sendData[2] = 0x03; /* Firmware Key = 0x03 */
        sendDataLen = 3;

        if (who < 0)
        { /* GROUP firmware broadcast */
            sendMsgID = GROUPID( -who);

            /* Set expectedReplies */
            switch (who)
            {
                case WHOLE_ARM:
                    expectedReplies = 7;
                    break;
                case LOWER_ARM:
                    expectedReplies = 4;
                    break;
                case UPPER_ARM:
                    expectedReplies = 3;
                    break;
                default:
                    pthread_mutex_unlock( &commMutex );
                    return ( -1);
                }
        }
        else
        {
            sendMsgID = NODE2ADDR(who);
            expectedReplies = 1;
        }

        can_msg(bus, sendMsgID, sendDataLen, sendData);
    }

    /* Find the length of the input file */
    fileSize = getFilesize( inFile );

    /* Allocate enough memory for the file */
    bufferPtr = fileBuffer = (char* )malloc( fileSize );
    if ( fileBuffer == NULL )
    {
        syslog(LOG_ERR, "downloadFirmware: malloc failed!");
        pthread_mutex_unlock( &commMutex );
        return (1);
    }

    /* Load firmware file into memory buffer */
    while ((*(bufferPtr++) = fgetc(inFile)) != EOF)
        ;
    /* Show what we are doing */
    //printf("\nDownload %ld bytes to: %d", fileSize, who);

    /* Send the *.tek firmware file */
    bufferPtr = fileBuffer;
    printf("\nDownload Progress: 0.0%%");
    while (bufferPtr < fileBuffer + fileSize)
    {
        firmChar = *(bufferPtr++);

        /* Skip non-alphanumeric data */
        if (!isAlpha(firmChar) && !isDigit(firmChar))
            continue;

        printf("\rDownload Progress: %.1f%%\t\t", (100.0 * (bufferPtr-fileBuffer)) / fileSize);
        //printf("%c", firmChar);
        fflush(stdout);

        /* Wait for the replies */

        for (i = 0; i < expectedReplies; i++)
        {
            err += readCAN(canDev[bus], recvData, &recvMsgID, &recvDataLen, &dataStart);
            if (err)
                printf("\nRead Error: %d\n", err);
            /*
                  parseMessage(recvMsgID, recvDataLen, &recvData[dataStart], &node, &property, &value);
                  if (property != -FIRMWARE_VERSION)
                {
                      printf("\nGot unexpected traffic!\n");
                      pthread_mutex_unlock( &commMutex );
                      return ( -1);
                }
                  */
        }


        /* Send a char */
        sendData[2] = firmChar;
        can_msg(bus, sendMsgID, sendDataLen, sendData);
    } /* while(!EOF) */
    pthread_mutex_unlock( &commMutex ); /* Unlock the CAN controller */

    free( fileBuffer );
    printf("\nDownload Complete!\n");

    return (0);
}
#endif

/** Sends data onto the CAN bus.
    Builds a FastMessage based on a messageID, length, and payload data, then sends it.

    \return The function returns the number of message sent, or -1 when the system call failed. The return value 0 determines
that no message could be sent, probably because there was no space in the targeted queue. This function does does
not block in such a case, so you need to loop explicitly until the message is sent.

    \note For now, does not retry can_fast_send() in case of failure
*/
int can_msg(
int bus,
WORD_t lId          /** The messageID to use */,
BYTE_t lLen         /** Length of the data payload (Data Length Code) in bytes */,
BYTE_t *lData       /** Character buffer containing the data to send */)
{
    FastMessage fmsg; /* F-Message buffer for temporary use */
    int i;

    /* Build the message in the temporary buffer */
    fmsg.cmd = 0; /* 0 = Standard Frame, 1 = Extended Frame */
    fmsg.data[0] = (lId / 8);
    fmsg.data[1] = ((lId * 32) /*+ rtr*16 */ + lLen) & 0xff;
    if(lLen > 8)
    {
        syslog(LOG_ERR, "can_msg: lLen = %d", lLen);
        return(-5);
    }
    for (i = 0; i < lLen; i++)
        fmsg.data[i + 2] = lData[i]; /* data i */

    /* Send it! */
    CAN_FAST_SEND(canDev[bus], 0, &fmsg);

}

/** Gets the status of all nodes on the bus.
    Issues a "Get Status" to the first 16 nodes on the bus and fills
    a status[] array with the answers received (if any).

    \return 0 for success

*/
int getBusStatus(
int bus,
long *status         /** This array is populated with the status of each puck, by node ID */)
{
    int i;

    int msgID;
    int dataLen;
    unsigned char getStatus;

    int node;
    int property;
    long value;

    unsigned char d[16];
    int id;
    int len;
    int desc;
    int ext;
    int rtr;
    int err;
    int dataStart;
    int n;
    char buff[80];

    int result;

    /* Initialize the status array */
    for (i = 0; i < MAX_NODES; i++)
        status[i] = -1;

    /* Lock the CAN card */

    if ((err =  pthread_mutex_lock( &commMutex )) != EOK)
    {
        syslog(LOG_ERR, "Set status: lock mutex failed: %d", err);
    }
    /* Build the GET STATUS message */
    getStatus = STATUS;
    dataLen = 1;

    /* Empty the CAN Rx buffer */
    do
    {
        result = can_recv_fast(canDev[bus], (FastMessage *)d);
        if(result)
        {
            syslog(LOG_ERR,"Non-empty CAN Card Buffer!\n");
        }
    }
    while(result == 1)
        ;

    /* For each who */
    for (i = 0; i < MAX_NODES; i++)
    {
        /* Set the message ID */
        msgID = NODE2ADDR(i);

        /* Send the message */
        can_msg(bus, msgID, dataLen, &getStatus);

        /* Wait a moment */
        usleep(10000);

        /* Read the reply, if any */
        result = can_recv_fast(canDev[bus], (FastMessage *) &d[0]);
        if (result == 1)
        {
            //printf("\nGot a puck response!");
            /* Extract controller number and blank bits for further processing */
            //ctrlNum = (d[1]&0x70) >> 4;
            d[1] &= ~0x70;

            if ( d[1] == 0 )
            { /* Jenz command specification = type 0 message in buffer */
                desc = d[2] * 256 + d[3];
                id = (desc >> 5) & 0x07ff;
                rtr = (desc >> 4) & 1;
                len = desc & 0xf;

                dataStart = 4; /* Start of the message data in the CAN frame */
            }
            else if ( d[1] == 1 )
            {  /* Jenz command specification = type 1 message in buffer */
                ext  = (d[2] >> 7) & 0x01;
                rtr  = (d[2] >> 6) & 0x01;
                len  =  d[2]       & 0x0f;
                len = (len > 8) ? 8 : len;

                dataStart = 8; /* Start of the message data in the CAN frame */

                /* Print the CAN data into a string buffer for debugging */
                n = 0;
                for(i=0; i<len; i++)
                {
                    n += sprintf(&buff[n],"%2.2x ",d[i+8]);
                }
                buff[n]='\0';

                if(ext) /* If this is an extended frame message */
                {
                    id   = (d[4] << (24-3));
                    id  += (d[5] << (16-3));
                    id  += (d[6] << ( 8-3));
                    id  += (d[7] >>     3 ) & (0xff>>3);

                    syslog(LOG_ERR, "getBusStatus(): Unexpected Extended CAN frame received: ID=0x%x, LEN=%d, DATA=%s",  id, len, buff);
                }
                else /* It is a standard frame message */
                {
                    id   = (d[4] << 3);
                    id  += (d[5] >> 5) & 0x07;

                    syslog(LOG_ERR, "getBusStatus(): Unexpected Standard CAN frame received: ID=0x%x, LEN=%d, DATA=%s",  id, len, buff);
                }
            }

            /* Parse the received message */
            parseMessage(id, len, &d[dataStart], &node, &property, &value);
            //syslog(LOG_ERR, "getstatus Value %d Node %d and preperty %d", value, node, property);
            /* Check if this is a response to a GET STATUS request */
            if(property != STATUS)
            {
                syslog(LOG_ERR,"getBusStatus(): Received a non-status response (property %d) after querying puck %d", property,i);
            }
            else
            {
                //syslog(LOG_ERR,"getBusStatus(): The status of puck %d is %d", i, value);
                status[i] = (int)value;
            }
        }
    }

    /* Unlock the CAN card */
    if ((err =  pthread_mutex_unlock( &commMutex)) != EOK)
    {
        syslog(LOG_ERR, "Get group property fast: unlock mutex failed: %d", err);
    }

    return (0);
}
