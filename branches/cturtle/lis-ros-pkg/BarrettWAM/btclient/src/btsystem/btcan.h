/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... btcan.h
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

/** \file btcan.h
Handles all communication with the robot over the CAN bus.
    Requires library files "libcan.a" and "libmitop.a".
*/
#ifndef _BTCAN_H
#define _BTCAN_H
#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */
//#include <inttypes.h>
//#include <pthread.h>

#ifndef FALSE
#define FALSE (0)
#endif
#ifndef TRUE
#define TRUE (1)
#endif
/* #define D(x) <blank> ...for no debug info */
/* #define D(x) x       ...for debug info */
#define D(x)
#define SAFETY_MODULE (10)
#define MAX_NODES    (31)

#define L08       (1)
#define L16       (2)
#define L24       (3)
#define L32       (4)

#define EE        (0x0008)

#define mbxID               (0)
#define BASE_ID             (0)

#define ADDR2NODE(x) ((((x) >> 5) & 0x001F) - BASE_ID)
#define NODE2ADDR(x) (((mbxID + BASE_ID) << 5) | ((x) + BASE_ID))
#define GROUPID(n)   (((mbxID + BASE_ID) << 5) | (0x0400 + (n)))
#define BROADCAST    (GROUPID(0))

/* Public Data Structures */
/** CAN device information data structure
*/

/* Public Functions */

int initCAN(int bus, int port);
void freeCAN(int bus);
int getBusStatus(int bus, long *status);
int wakePuck(int bus, int who);
int getProperty(int bus, int who, int property, long *reply);
int setProperty(int bus, int who, int property, int verify, long value);
int getPositions(int bus, int group, int howMany, long *pos);
int setTorques(int bus, int group, int *values);
const char* Prop2Name(int prop);
int Name2Prop(char *name);
void initPropertyDefs(int firmwareVersion);

/*! bcastGroup */
enum {
   WHOLE_ARM = 0,
   LOWER_ARM = -1,
   UPPER_ARM = -2
};

enum {
   ROLE_TATER = 0,
   ROLE_GIMBALS = 1,
   ROLE_SAFETY = 2,
   ROLE_WRAPTOR = 3
};

/*! Control_mode states */
enum {
   MODE_IDLE = 0,
   MODE_DUTY = 1,
   MODE_TORQUE = 2,
   MODE_PID = 3,
   MODE_VELOCITY = 4,
   MODE_TRAPEZOIDAL = 5
};

enum {
   STATUS_OFFLINE = -1,
   STATUS_RESET = 0,
   STATUS_ERR = 1,
   STATUS_READY = 2
};

enum {
   DEG = 0,        /* 0-360        */
   RAD = 1,        /* 0-6.28       */
   GRAD = 2,       /* 0-400        */
   PERCENT = 3,    /* 0-100        */
   NATIVE =4     /* 0-CTS*RATIO  */
};

enum {
   SAVED_ERR = 7,
   IGNORE_ERR = 8,
   IS_ACTIVE = 9
};

enum {
   BUS_ERROR = -1,
   BUS_OFF = 0,
   BUS_ON = 1
};

enum {
   ERR_NONE = 0,
   ERR_READ_ONLY = 0,
   ERR_OUT_OF_RANGE = 1
};

int VERS;
int ROLE;
int SN;
int ID;
int ERROR;
int STAT;
int ADDR;
int VALUE;
int MODE;
int D;
int TORQ;
int MD;
int V;
int B;
int P;
int P2;
int E;
int E2;
int MT;
int MV;
int MCV;
int MOV;
int MOFST;
int IOFST;
int PTEMP;
int UPSECS;
int OD;
int MDS;
int AP;
int AP2;
int MECH;
int MECH2;
int CTS;
int CTS2;
int DP;
int DP2;
int OT;
int OT2;
int CT;
int CT2;
int BAUD;
int TEMP;
int OTEMP;
int _LOCK;
int DIG0;
int DIG1;
int ANA0;
int ANA1;
int THERM;
int VBUS;
int IMOTOR;
int VLOGIC;
int ILOGIC;
int GRPA;
int GRPB;
int GRPC;
int PIDX;
int ZERO;
int SG;
int HSG;
int LSG;
int _DS;
int IVEL;
int IOFF;
int MPE;
int EN;
int TSTOP;
int KP;
int KD;
int KI;
int SAMPLE;
int ACCEL;
int TENSION;
int UNITS;
int  RATIO;
int LOG;
int DUMP;
int LOG1;
int LOG2;
int LOG3;
int LOG4;
int GAIN1;
int GAIN2;
int GAIN3;
int OFFSET1;
int OFFSET2;
int OFFSET3;
int PEN;
int SAFE;
int SAVE;
int LOAD;
int DEF;
int VL1;
int VL2;
int TL1;
int TL2;
int VOLTL1;
int VOLTL2;
int VOLTH1;
int VOLTH2;
int MAXPWR;
int PWR;
int IFAULT;
int IKP;
int IKI;
int IKCOR;
int VNOM;
int TENST;
int TENSO;
int JIDX;
int IPNM;
int HALLS;
int HALLH;
int HALLH2;
int POLES;
int ECMAX;
int ECMIN;
int ISQ;
int TETAE;
int FIND;
int LCV;
int LCVC;
int LFV;
int LFS;
int LFAP;
int LFDP;
int LFT;
int VALUE32;
int PROP_END;

int LOCK;
int FET0;
int FET1;
int CMD;
int X0;
int X1;
int X2;
int X3;
int X4;
int X5;
int X6;
int X7;
int COMMON_END;
int SAFETY_END;
int T;
int M;
int M2;
int IOFF2;
int HOLD;
int TIE;
int LFLAGS;
int LCTC;


#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif /*_BTCAN_H */
