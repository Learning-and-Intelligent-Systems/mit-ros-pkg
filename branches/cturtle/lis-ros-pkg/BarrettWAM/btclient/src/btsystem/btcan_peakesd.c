/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... btcan_peakesd.c
 *  Creation Date ...... 24 Mar 2003
 *  Author ............. Traveler Hauptman
 *  Addtl Authors ...... Brian Zenowich
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  Note: This file may be linked against a closed-source proprietary
 *        driver library (libntcan) from esd electronics
 *        (http://esd-electronics.com) 
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

/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#ifdef S_SPLINT_S
#include <err.h>
#endif
#include <stdio.h>
#include <errno.h>

#include <pthread.h>
#include <syslog.h>
#include <linux/version.h>
#include <signal.h>

#ifdef XENOMAI
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#else
#include <inttypes.h>
#endif

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#ifdef PEAK_CAN
#include <libpcan.h>
#endif

#ifdef ESD_CAN
#include "ntcan.h"
#endif

#include "btcan.h"
#include "btos.h"


/*==============================*
 * PRIVATE DEFINED constants    *
 *==============================*/
#define TX_QUEUE_SIZE       (32)
#define RX_QUEUE_SIZE       (32)
#define TX_TIMEOUT          (50)
#define RX_TIMEOUT          (50)

#define MAX_BUS             (4)

//#define HW_ISA_SJA          (9) // use this also for PC/104
//#define HW_PCI              (10) // PCI carries always SJA1000 chips
//#define PCI                 (1)
//#define ISA                 (0)


#define DEBUG(x)

/*==============================*
 * PRIVATE MACRO definitions    *
 *==============================*/
#define isAlpha(c) ( ((c >= 'A') && (c <= 'Z')) ? 1 : 0 )
#define isSpace(c) ( (c == ' ') ? 1 : 0 )
#define isDigit(c) ( ((c >= '0') && (c <= '9')) ? 1 : 0 )

#define Border(Value,Min,Max)  (Value<Min)?Min:((Value>Max)?Max:Value)

/*==============================*
 * PRIVATE typedefs and structs *
 *==============================*/
#ifdef ESD_CAN
typedef unsigned long DWORD;
#endif
#define MAX_FILTERS (3)

/*==============================*
* GLOBAL file-scope variables  *
*==============================*/
HANDLE        canDev[MAX_BUS]; // typedef int HANDLE (ntcan.h)
btrt_mutex    commMutex[MAX_BUS];
int accept[MAX_FILTERS];
int mask[MAX_FILTERS];

/* keyword, index, readFunction, writeFunction, defaultVal, type */
const int dataType[]=
   {
      /* VERS */  L16 ,
      /* ROLE */ L16 | EE ,
      /* SN */ L16 | EE ,
      /* ID */ L16 | EE ,
      /* ERROR */ L16 ,
      /* STAT */ L16 ,
      /* ADDR */ L16 ,
      /* VALUE */ L16 ,
      /* MODE */ L16 ,
      /* D */ L16 ,
      /* TORQ */ L16 ,
      /* V */ L16 ,
      /* B */ L16 ,
      /* P */ L32 ,
      /* P2 */ L16 ,
      /* E */ L32 ,
      /* E2 */ L16 ,
      /* MT */ L16 | EE ,
      /* MV */ L16 | EE ,
      /* MCV */ L16 | EE ,
      /* MOV */ L16 | EE ,
      /* MOFST */ L16 | EE ,
      /* IOFST */ L16 | EE ,
      /* PTEMP */ L16 | EE ,
      /* UPSECS */ L16 | EE ,
      /* OD */ L16 | EE ,
      /* MDS */ L16 | EE ,
      /* AP */ L32 | EE ,
      /* AP2 */ L16 ,
      /* MECH */ L32 ,
      /* MECH2 */ L16 ,
      /* CTS */ L32 | EE ,
      /* CTS2 */ L16 ,
      /* DP */ L32 | EE ,
      /* DP2 */ L16 ,
      /* OT */ L32 | EE ,
      /* OT2 */ L16 ,
      /* CT */ L32 | EE ,
      /* CT2 */ L16 ,
      /* BAUD */ L16 ,
      /* TEMP */ L16 ,
      /* OTEMP */ L16 ,
      /* LOCK */ L16 ,
      /* DIG0 */ L16 ,
      /* DIG1 */ L16 ,
      /* ANA0 */ L16 ,
      /* ANA1 */ L16 ,
      /* THERM */ L16 ,
      /* VBUS */ L16 ,
      /* IMOTOR */ L16 ,
      /* VLOGIC */ L16 ,
      /* ILOGIC */ L16 ,
      /* GRPA */ L16 | EE ,
      /* GRPB */ L16 | EE ,
      /* GRPC */ L16 | EE ,
      /* PIDX */ L16 | EE ,
      /* ZERO */ L16 ,
      /* SG */ L16 ,
      /* HSG */ L16 | EE ,
      /* LSG */ L16 | EE ,
      /* DS */ L16 | EE ,
      /* IVEL */ L16 | EE ,
      /* IOFF */ L16 | EE ,
      /* MPE */ L16 | EE ,
      /* EN */ L16 ,
      /* TSTOP */ L16 | EE ,
      /* KP */ L16 | EE ,
      /* KD */ L16 | EE ,
      /* KI */ L16 | EE ,
      /* SAMPLE */ L16 | EE ,
      /* ACCEL */ L16 | EE ,
      /* TENSION */ L16 ,
      /* UNITS */ L16 | EE ,
      /* RATIO */ L16 | EE ,
      /* LOG */ L16 ,
      /* DUMP */ L16 ,
      /* LOG1 */ L16 ,
      /* LOG2 */ L16 ,
      /* LOG3 */ L16 ,
      /* LOG4 */ L16 ,
      /* GAIN1 */ L16 | EE ,
      /* GAIN2 */ L16 | EE ,
      /* GAIN3 */ L16 | EE ,
      /* OFFSET1 */ L16 | EE ,
      /* OFFSET2 */ L16 | EE ,
      /* OFFSET3 */ L16 | EE ,
      /* PEN */ L16 ,
      /* SAFE */ L16 ,
      /* SAVE */ L16 ,
      /* LOAD */ L16 ,
      /* DEF */ L16 ,
      /* VL1 */ L16 ,
      /* VL2 */ L16 ,
      /* TL1 */ L16 ,
      /* TL2 */ L16 ,
      /* VOLTL1 */ L16 | EE ,
      /* VOLTL2 */ L16 | EE ,
      /* VOLTH1 */ L16 | EE ,
      /* VOLTH2 */ L16 | EE ,
      /* MAXPWR */ L16 ,
      /* PWR */ L16 ,
      /* IFAULT */ L16 ,
      /* IKP */ L16 | EE ,
      /* IKI */ L16 | EE ,
      /* IKCOR */ L16 | EE ,
      /* VNOM */ L16 ,
      /* TENST */ L16 | EE ,
      /* TENSO */ L16 | EE ,
      /* JIDX */ L16 | EE ,
      /* IPNM */ L16 | EE ,
      /* HALLS */ L16 ,
      /* HALLH */ L32 ,
      /* HALLH2 */ L16 ,
      /* POLES */ L16 | EE ,
      /* ECMAX */ L16 ,
      /* ECMIN */ L16 ,
      /* ISQ */ L16 ,
      /* TETAE */ L16 ,
      /* FIND */ L16 ,
      /* LCV */ L16 ,
      /* LCVC */ L16 ,
      /* LFV */ L16 ,
      /* LFS */ L16 ,
      /* LFAP */ L16 ,
      /* LFDP */ L16 ,
      /* LFT */ L16 ,
      /* VALUE32 */ L16
   };

/*==============================*
 * PRIVATE Function Prototypes  *
 *==============================*/
int compile(int property, long longVal, unsigned char *data, int *dataLen, int isForSafety);
int parseMessage(int id, int len, unsigned char *messageData, int *node, int *property, long *value);
int canReadMsg(int bus, int *id, int *len, unsigned char *data, int blocking);
int canSendMsg(int bus, int id, char len, unsigned char *data, int blocking);
int canClearMsg(int bus);

/*==============================*
 * Functions                    *
 *==============================*/
void allowMessage(int bus, int id, int mask)
{
#ifdef PEAK_CAN
   //Allows all messages
   CAN_ResetFilter(canDev[bus]);

#else

   int i;
   for(i = 0; i < 2048; i++)
      if((i & ~mask) == id)
         canIdAdd(canDev[bus], i);
#endif
}





int initCAN(int bus, int port)
{
   long  retvalue;
   long  pPort;
   int   pIrq;
   int   i;

   //btrt_mutex_create(&commMutex);
   btrt_mutex_init(&commMutex[bus]);

   
#ifdef PEAK_CAN
   //assign ports and irqs to buses *needs to be updated to read ports from cat /proc/pcan/
#ifdef ISA_CAN
   if(port == 0)
   {
      pPort = 0x300;
      pIrq = 7;
   }
   else if (port == 1)
   {
      pPort = 0x320;
      pIrq = 5;
   }
   else
   {
      syslog(LOG_ERR, "initCAN: incorrect bus number, cannot open bus %d", bus);
      return(1);
   }

   canDev[bus] = CAN_Open(HW_ISA_SJA, pPort, pIrq);
   if (!canDev[bus])
   {
      syslog(LOG_ERR, "initCAN(): CAN_Open(): cannot open device with");
      syslog(LOG_ERR, "type=isa, port=%d, irq=%d", pPort, pIrq);
      return(1);
   }
#endif

#ifdef PCI_CAN
   canDev[bus] = CAN_Open(HW_PCI, (port + 1));
   if (!canDev[bus])
   {
      syslog(LOG_ERR, "initCAN(): CAN_Open(): cannot open device with");
      syslog(LOG_ERR, "type=pci, port=%d", port);
      return(1);
   }
#endif

   /* Clear Status */
   CAN_Status(canDev[bus]);

   retvalue = CAN_Init(canDev[bus], CAN_BAUD_1M, CAN_INIT_TYPE_ST);
   if (retvalue)
   {
      syslog(LOG_ERR, "initCAN(): CAN_Init() failed with %d", errno);
      return(1);
   }
   
   CAN_ResetFilter(canDev[bus]);
   CAN_MsgFilter(canDev[bus], 0x0000, 0x053F, MSGTYPE_STANDARD);
   
#endif /* PEAK_CAN */

#ifdef ESD_CAN
   //Opening can for esd.
   retvalue = canOpen(port, 0, TX_QUEUE_SIZE, RX_QUEUE_SIZE, TX_TIMEOUT, RX_TIMEOUT, &canDev[bus]);
   //retvalue = canOpen(bus, 0, TX_QUEUE_SIZE, RX_QUEUE_SIZE, TX_TIMEOUT, RX_TIMEOUT, &canDev[bus]);
   if(retvalue != NTCAN_SUCCESS)
   {
      syslog(LOG_ERR, "initCAN(): canOpen() failed with error %d", retvalue);
      return(1);
   }

   retvalue = canSetBaudrate(canDev[bus], 0); // 1 = 1Mbps, 2 = 500kbps, 3 = 250kbps
   if(retvalue != 0)
   {
      syslog(LOG_ERR, "initCAN(): canSetBaudrate() failed with error %d", retvalue);
      return(1);
   }
   
   allowMessage(bus, 0x0000, 0x03E0); // Messages sent directly to host
   allowMessage(bus, 0x0403, 0x03E0); // Group 3 messages
   allowMessage(bus, 0x0406, 0x03E0); // Group 6 messages
#endif
#if 0
   /* Intialize filter/mask */
   for(i = 0; i < MAX_FILTERS; i++){
      filter[i] = 0;
      mask[i] = 0;
   }
#endif

   // Mask 3E0: 0000 0011 1110 0000
   accept[0] = 0x0000; mask[0] = 0x03E0;
   accept[1] = 0x0403; mask[1] = 0x03E0;
   accept[2] = 0x0406; mask[2] = 0x03E0;
   //allowMessage(bus, 0x0000, 0x03E0); // Messages sent directly to host
   //allowMessage(bus, 0x0403, 0x03E0); // Group 3 messages
   //allowMessage(bus, 0x0406, 0x03E0); // Group 6 messages

   // Set minimum required parameter values
   VERS = 0;
   STAT = 5;
   PROP_END = 10;

   return(0);
}

void freeCAN(int bus)
{
#ifdef PEAK_CAN
   CAN_Close(canDev[bus]);
#else
   canClose(canDev[bus]);
#endif
}


int canReadMsg(int bus, int *id, int *len, unsigned char *data, int blocking)
{
#ifdef PEAK_CAN
   TPCANRdMsg  msg;
   int       pendread;
   int       pendwrite;
#else
   CMSG    msg;
#endif

   long     retvalue=0;
   long      msgCt = 1;
   int       i;
   int      filterOK = 0;

#ifdef PEAK_CAN

   btrt_set_mode_hard();
   //retvalue = rt_task_set_mode(0, T_PRIMARY, NULL);
   if(blocking)
   {//attempt to read till there is a message available
      //while(!filterOK){
         retvalue = LINUX_CAN_Read(canDev[bus], &msg);
         /* Apply private acceptance filter 
         for(i = 0; i < MAX_FILTERS; i++){
            if((msg.Msg.ID & ~mask[i]) == accept[i]){
               filterOK = 1;
            }
         }*/
      //}
   }
   else
   {//check if a message is pending, if not wait for a period and try again and return

      retvalue = LINUX_CAN_Extended_Status(canDev[bus], &pendread, &pendwrite);
      if(pendread)
      {
         retvalue = LINUX_CAN_Read(canDev[bus], &msg);
      }
      else
      {
         retvalue=usleep(1000);
         retvalue = LINUX_CAN_Extended_Status(canDev[bus], &pendread, &pendwrite);
         if(pendread)
            retvalue = LINUX_CAN_Read(canDev[bus], &msg);
         else
            return(1);//returned empty
      }
   }
   if(retvalue) //if there is a error
   {
      syslog(LOG_ERR, "canReadMsg(): canRead error: %ld", retvalue);
      return(2);
   }
   if(msgCt == 1)
   {
      *id = msg.Msg.ID;
      *len = msg.Msg.LEN;
      for(i = 0; i < msg.Msg.LEN; i++)
         data[i] = msg.Msg.DATA[i];

      return(0);
   }
#else
   if(blocking)
   {
      //while(!filterOK){
         retvalue = canRead(canDev[bus], &msg, &msgCt, NULL);
         /* Apply private acceptance filter 
         for(i = 0; i < MAX_FILTERS; i++){
            if((msg.id & ~mask[i]) == accept[i]){
               filterOK = 1;
            }
         }*/
      //}
   }
   else
   {
      retvalue = canTake(canDev[bus], &msg, &msgCt);
   }
   if(retvalue != NTCAN_SUCCESS)
   {
      //syslog(LOG_ERR, "canReadMsg(): canRead/canTake error: %ld", retvalue);
      if(retvalue == NTCAN_RX_TIMEOUT)
         return(1);
      else
         return(2);
   }
   if(msgCt == 1) {
      *id = msg.id;
      *len = msg.len;
      for(i = 0; i < msg.len; i++)
         data[i] = msg.data[i];

      return(0);
   }
#endif
   
   return(1); // No message received, return err
}




int canSendMsg(int bus, int id, char len, unsigned char *data, int blocking){
#ifdef PEAK_CAN
   TPCANMsg  msg;
   int       pendread;
   int       pendwrite=1;

#else

   CMSG    msg;

#endif

   DWORD     retvalue;
   long      msgCt = 1;
   int       i;


#ifdef PEAK_CAN

   msg.ID = id;
   msg.MSGTYPE = MSGTYPE_STANDARD;
   msg.LEN = len & 0x0F;
   for(i = 0; i < len; i++)
      msg.DATA[i] = data[i];

   //make sure that write is in primary mode
   //retvalue = rt_task_set_mode(0, T_PRIMARY, NULL);
   btrt_set_mode_hard();
   if(blocking)
   {
      retvalue =CAN_Write(canDev[bus], &msg);
   }
   else
   {//non-blocking, check to see if bus is full or sending errors, if not send, else return
      retvalue = LINUX_CAN_Extended_Status(canDev[bus], &pendread, &pendwrite);
      if (retvalue != CAN_ERR_OK)
      {
         syslog(LOG_ERR, "canSendMsg(): error while trying to get status");
      }
      else
      {
         retvalue = CAN_Write(canDev[bus], &msg);
      }
   }
   if(retvalue)
   {
      syslog(LOG_ERR, "canSendMsg(): canSend error: %ld", retvalue);
      return(1);
   }
   return(0);

#else

   msg.id = id;
   msg.len = len & 0x0F;
   for(i = 0; i < len; i++)
      msg.data[i] = data[i];
   
   if(blocking)
   {
      retvalue = canWrite(canDev[bus], &msg, &msgCt, NULL);
   }
   else
   {
      retvalue = canSend(canDev[bus], &msg, &msgCt);
   }

   if(retvalue != NTCAN_SUCCESS)
   {
      syslog(LOG_ERR, "canSendMsg(): canWrite/Send() failed with error %d", retvalue);
      return(1);
   }
#endif

}


int canClearMsg(int bus)
{
#ifdef PEAK_CAN
   TPCANRdMsg  msg;
   DWORD       retvalue;
   int         pendread=1;
   int         pendwrite;
#else

   unsigned char CANdata[8];
   int len_in;
   int id_in;

#endif
   int id, len;
   unsigned char d[8];
   
#ifdef PEAK_CAN

   retvalue = LINUX_CAN_Extended_Status(canDev[bus], &pendread, &pendwrite);

   while(pendread!=0)
   {
      retvalue =  canReadMsg(bus, &id, &len, d, 1);
      //retvalue = LINUX_CAN_Read(canDev[bus], &msg);
      retvalue = LINUX_CAN_Extended_Status(canDev[bus], &pendread, &pendwrite);
      
      //syslog(LOG_ERR, "Cleared unexpected message from CANbus: ID[%4x] LEN[%d] DATA[%2x %2x %2x %2x %2x %2x %2x %2x]",
      //   id, len, d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
      //usleep(1);
   }
   return(0);

#else
   //find a better way of clearing the bus
   while(!canReadMsg(canDev[bus], &id_in, &len_in, CANdata, FALSE))
   {
      syslog(LOG_ERR, "Cleared unexpected message from CANbus");
      usleep(1);
   }
#endif
}

int wakePuck(int bus, int who)
{
   setProperty(bus, who, 5, FALSE, STATUS_READY); // Must use '5' for STAT
   usleep(300000); // Wait 300ms for puck to initialize

   return(0);
}

int setTorques(int bus, int group, int *values)
{
   unsigned char   data[8];
   int             err;
   int             i;

   /* Bound the torque commands */
   for (i = 0; i < 4; i++)
   {
      values[i] = Border(values[i], -8191, 8191);
   }

   /* Special value-packing compilation: Packs (4) 14-bit values into 8 bytes */
   /*     0        1        2        3        4        5        6        7    */
   /* ATPPPPPP AAAAAAaa aaaaaaBB BBBBbbbb bbbbCCCC CCcccccc ccDDDDDD dddddddd */

   data[0] = TORQ | 0x80; /* Set the "Set" bit */
   data[1] = (unsigned char)(( values[0] >> 6) & 0x00FF);
   data[2] = (unsigned char)(((values[0] << 2) & 0x00FC) | ((values[1] >> 12) & 0x0003) );
   data[3] = (unsigned char)(( values[1] >> 4) & 0x00FF);
   data[4] = (unsigned char)(((values[1] << 4) & 0x00F0) | ((values[2] >> 10) & 0x000F) );
   data[5] = (unsigned char)(( values[2] >> 2) & 0x00FF);
   data[6] = (unsigned char)(((values[2] << 6) & 0x00C0) | ((values[3] >> 8) & 0x003F) );
   data[7] = (unsigned char)( values[3] & 0x00FF);

   // Send the data
   btrt_mutex_lock(&commMutex[bus]);
   err = canSendMsg(bus, GROUPID(group), 8, data, TRUE);
   btrt_mutex_unlock(&commMutex[bus]);
}




int getPositions(int bus, int group, int howMany, long *pos)
{
   int             err;
   unsigned char   data[8];
   int             len;
   int             msgID;
   int             id;
   int             property;
   long            reply;
   float alltime;

   // Compile the packet
   data[0] = (unsigned char)AP;

   btrt_mutex_lock(&commMutex[bus]);

   // Send the packet

   err = canSendMsg(bus, GROUPID(group), 1, data, TRUE);
   //howMany = 2; // xxx Remove me

   // Wait for each reply
   //time1 = btrt_get_time();
   while(howMany)
   {
      err = canReadMsg(bus, &msgID, &len, data, TRUE);
      // If no error
      if(!err)
      {
         // Parse the reply
         err = parseMessage(msgID, len, data, &id, &property, &reply);
         if(property == AP)
         {
            pos[id] = reply;
            --howMany;
         }
         else
         {
            syslog(LOG_ERR, "getPositions(): Asked group %d for position, received property %d = %ld from id %d", group, property, reply, id);
         }
      }
      else
      {
         // Timeout or other error
         btrt_mutex_unlock(&commMutex[bus]);
         return(err);
      }

   }
   btrt_mutex_unlock(&commMutex[bus]);
   return(0);
}

int setProperty(int bus, int id, int property, int verify, long value)
{
   long            response;
   unsigned char   data[8];
   int             len;
   int             err;

   //syslog(LOG_ERR, "About to compile setProperty, property = %d", property);
   // Compile 'set' packet
   err = compile(property, value, data, &len, id == SAFETY_MODULE);

   //syslog(LOG_ERR, "After compilation data[0] = %d", data[0]);
   data[0] |= 0x80; // Set the 'Set' bit


   // Send the packet
   btrt_mutex_lock(&commMutex[bus]);
   err = canSendMsg(bus, (id & 0x0400) ? id : NODE2ADDR(id), len, data, TRUE);
   btrt_mutex_unlock(&commMutex[bus]);

   // BUG: This will not verify properties from groups of pucks
   if(verify)
   {
      // Get the new value of the property
      getProperty(bus, id, property, &response);

      // Compare response to value
      if(response != value)
         return(1);
   }
   return(0);
}

int getProperty(int bus, int id, int property, long *reply)
{
   int err;
   unsigned char data[8];
   int len_in;
   int id_in;
   int property_in;


   // Compile the packet
   data[0] = (unsigned char)property;

   btrt_mutex_lock(&commMutex[bus]);

   // Send the packet
   err = canSendMsg(bus, NODE2ADDR(id), 1, data, TRUE);

   // Wait for 1 reply
   err = canReadMsg(bus, &id_in, &len_in, data, TRUE);

   btrt_mutex_unlock(&commMutex[bus]);

   if(!err)
   {
      // Parse the reply
      err = parseMessage(id_in, len_in, data, &id_in, &property_in, reply);


      // Check that the id and property match
      if((id == id_in) && (property == property_in))

         return(0);
      else
      {
         syslog(LOG_ERR, "getProperty(): returned id or property do not match");
         return(1);
      }
   }
   else
   {
      syslog(LOG_ERR, "getProperty(): canReadMsg error = %d", err);
      return(1);
   }
}

int getBusStatus(int bus, long *status)
{
   int err;
   unsigned char data[8];
   int id;
   int len_in;
   int id_in;
   int property_in;
   int firstFound = 0;
   long fw_vers;

   btrt_set_mode_hard();

   btrt_mutex_lock(&commMutex[bus]);
   //err = canReadMsg(bus, &id_in, &len_in, data, FALSE);

   for(id = 0; id < MAX_NODES; id++)
   {
      // Compile the packet
      data[0] = 5; // STAT = 5

      // Initialize status to "NOT_FOUND"
      status[id] = -1;

      // Send the packet
      err = canSendMsg(bus, NODE2ADDR(id), 1, data, TRUE);

      // Wait 1ms
      usleep(1000);

      // Try to get 1 reply (non-blocking read)
      err = canReadMsg(bus, &id_in, &len_in, data, FALSE);

      // If no error
      if(!err)
      {
         // Parse the reply
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
         err = parseMessage(id_in, len_in, data, &id_in, &property_in, &status[id]);
#else

         err = parseMessage(id_in, len_in, data, &id_in, &property_in, &status[id]);
#endif
         // If this is the first puck found, initialize the property definitions
         if(!firstFound)
         {
            firstFound = 1;
            btrt_mutex_unlock(&commMutex[bus]);
            wakePuck(bus, id_in); // Wake this puck
            err = getProperty(bus, id_in, 0, &fw_vers); // Get the firmware version
            //setProperty(bus, id_in, 5, FALSE, 0); // Reset this puck
            //usleep(500000);
            btrt_mutex_lock(&commMutex[bus]);
            initPropertyDefs(fw_vers);
         }

      }
      else
         syslog(LOG_ERR, "getBusStatus(): canReadMsg returned error");
   }

   btrt_mutex_unlock(&commMutex[bus]);
#if BTDEBUG <= 8

   syslog(LOG_ERR,"getBusStatus: Only status != -1 is shown.");
   for(id = 0; id < MAX_NODES; id++)
   {
      if (status[id] != -1)
         syslog(LOG_ERR,"getBusStatus: status[%d] = %d", id, status[id]);
   }
#endif
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
   dataHeader = ((messageData[0] >> 6) & 0x0002) | ((id & 0x041F) == 0x0403);
   //messageData[0] &= 0x7F;
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

      *property = AP;
      //syslog(LOG_ERR,"Received packed set property: %d from node: %d value:%d",*property,*node,*value);
      break;
   case 2:  /* Data is normal, SET */
      *property = messageData[0] & 0x7F;
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
         *property = -(messageData[0] & 0x7F); /* A negative (or zero) property means GET */
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
   int isForSafety        /** A flag indicating whether this packet is destined for the safety circuit or not */)
{
   int i;

   // Check the property
   if(property > PROP_END)
   {
      syslog(LOG_ERR,"compile(): Invalid property = %d", property);
      return(1);
   }

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
   *dataLen = 6; //(dataType[property] & 0x0007) + 2;

   //if (i & 0x0003) *dataLen = 3; /* 8-bits */
   //else if (i & 0x000C) *dataLen = 4; /* 16-bits */
   //else if (i & 0x0030) *dataLen = 5; /* 24-bits */
   //else if (i & 0x00C0) *dataLen = 6; /* 32-bits */

   return (0);
}

void initPropertyDefs(int firmwareVersion){
   int i = 0;
   if(firmwareVersion < 40)
   {
      VERS = i++;
      ROLE = i++;
      SN = i++;
      ID = i++;
      ERROR = i++;
      STAT = i++;
      ADDR = i++;
      VALUE = i++;
      MODE = i++;
      D = i++;
      TORQ = i++;
      P = i++;
      V = i++;
      E = i++;
      B = i++;
      MD = i++;
      MT = i++;
      MV = i++;
      MCV = i++;
      MOV = i++;
      MOFST = i++;
      IOFST = i++;
      PTEMP = i++;
      UPSECS = i++;
      OD = i++;
      MDS = i++;
      AP = i++;
      AP2 = i++;
      MECH = i++;
      MECH2 = i++;
      CTS = i++;
      CTS2 = i++;
      DP = i++;
      DP2 = i++;
      OT = i++;
      OT2 = i++;
      CT = i++;
      CT2 = i++;
      BAUD = i++;
      TEMP = i++;
      OTEMP = i++;
      _LOCK = i++;
      DIG0 = i++;
      DIG1 = i++;
      ANA0 = i++;
      ANA1 = i++;
      THERM = i++;
      VBUS = i++;
      IMOTOR = i++;
      VLOGIC = i++;
      ILOGIC = i++;
      GRPA = i++;
      GRPB = i++;
      GRPC = i++;
      PIDX = i++;
      ZERO = i++;
      SG = i++;
      HSG = i++;
      LSG = i++;
      _DS = i++;
      IVEL = i++;
      IOFF = i++;
      MPE = i++;
      EN = i++;
      TSTOP = i++;
      KP = i++;
      KD = i++;
      KI = i++;
      SAMPLE = i++;
      ACCEL = i++;
      TENSION = i++;
      UNITS = i++;
      RATIO = i++;
      LOG = i++;
      DUMP = i++;
      LOG1 = i++;
      LOG2 = i++;
      LOG3 = i++;
      LOG4 = i++;
      GAIN1 = i++;
      GAIN2 = i++;
      GAIN3 = i++;
      OFFSET1 = i++;
      OFFSET2 = i++;
      OFFSET3 = i++;
      PEN = i++;
      SAFE = i++;
      SAVE = i++;
      LOAD = i++;
      DEF = i++;
      VL1 = i++;
      VL2 = i++;
      TL1 = i++;
      TL2 = i++;
      VOLTL1 = i++;
      VOLTL2 = i++;
      VOLTH1 = i++;
      VOLTH2 = i++;
      MAXPWR = i++;
      PWR = i++;
      IFAULT = i++;
      IKP = i++;
      IKI = i++;
      IKCOR = i++;
      VNOM = i++;
      TENST = i++;
      TENSO = i++;
      JIDX = i++;
      IPNM = i++;

      PROP_END = i++;

      T = TORQ;
      FET0 = B;
      FET1 = TENSION;
      /*
      HALLS = i++;
      HALLH = i++;
      HALLH2 = i++;
      POLES = i++;
      ECMAX = i++;
      ECMIN = i++;
      ISQ = i++;
      TETAE = i++;
      FIND = i++;
      LCV = i++;
      LCVC = i++;
      LFV = i++;
      LFS = i++;
      LFAP = i++;
      LFDP = i++;
      LFT = i++;
      VALUE32 = i++;
      */
   }
   else
   {
      /* Common */
      VERS = i++;
      ROLE = i++; /* P=PRODUCT, R=ROLE: XXXX PPPP XXXX RRRR */
      SN = i++;
      ID = i++;
      ERROR = i++;
      STAT = i++;
      ADDR = i++;
      VALUE = i++;
      MODE = i++;
      TEMP = i++;
      PTEMP = i++;
      OTEMP = i++;
      BAUD = i++;
      _LOCK = i++;
      DIG0 = i++;
      DIG1 = i++;
      FET0 = i++;
      FET1 = i++;
      ANA0 = i++;
      ANA1 = i++;
      THERM = i++;
      VBUS = i++;
      IMOTOR = i++;
      VLOGIC = i++;
      ILOGIC = i++;
      SG = i++;
      GRPA = i++;
      GRPB = i++;
      GRPC = i++;
      CMD = i++; /* For commands w/o values: RESET,HOME,KEEP,PASS,LOOP,HI,IC,IO,TC,TO,C,O,T */
      SAVE = i++;
      LOAD = i++;
      DEF = i++;
      FIND = i++;
      X0 = i++;
      X1 = i++;
      X2 = i++;
      X3 = i++;
      X4 = i++;
      X5 = i++;
      X6 = i++;
      X7 = i++;

      COMMON_END = i++;

      /* Safety */
      i = COMMON_END;
      ZERO = i++;
      PEN = i++;
      SAFE = i++;
      VL1 = i++;
      VL2 = i++;
      TL1 = i++;
      TL2 = i++;
      VOLTL1 = i++;
      VOLTL2 = i++;
      VOLTH1 = i++;
      VOLTH2 = i++;
      PWR = i++;
      MAXPWR = i++;
      IFAULT = i++;
      VNOM = i++;

      SAFETY_END = i++;

      /* Tater */
      i = COMMON_END;
      T = i++;
      MT = i++;
      V = i++;
      MV = i++;
      MCV = i++;
      MOV = i++;
      P = i++; /* 32-Bit Present Position */
      P2 = i++;
      DP = i++; /* 32-Bit Default Position */
      DP2 = i++;
      E = i++; /* 32-Bit Endpoint */
      E2 = i++;
      OT = i++; /* 32-Bit Open Target */
      OT2 = i++;
      CT = i++; /* 32-Bit Close Target */
      CT2 = i++;
      M = i++; /* 32-Bit Move command for CAN*/
      M2 = i++;
      _DS = i++;
      MOFST = i++;
      IOFST = i++;
      UPSECS = i++;
      OD = i++;
      MDS = i++;
      MECH = i++; /* 32-Bit */
      MECH2 = i++;
      CTS = i++; /* 32-Bit */
      CTS2 = i++;
      PIDX = i++;
      HSG = i++;
      LSG = i++;
      IVEL = i++;
      IOFF = i++; /* 32-Bit */
      IOFF2 = i++;
      MPE = i++;
      EN = i++;
      TSTOP = i++;
      KP = i++;
      KD = i++;
      KI = i++;
      ACCEL = i++;
      TENST = i++;
      TENSO = i++;
      JIDX = i++;
      IPNM = i++;
      HALLS = i++;
      HALLH = i++; /* 32-Bit */
      HALLH2 = i++;
      POLES = i++;
      IKP = i++;
      IKI = i++;
      IKCOR = i++;
      HOLD = i++;
      TIE = i++;
      ECMAX = i++;
      ECMIN = i++;
      LFLAGS = i++;
      LCTC = i++;
      LCVC = i++;

      PROP_END = i++;

      AP = P; // Handle parameter name change
      TENSION = FET1;
   }
}
