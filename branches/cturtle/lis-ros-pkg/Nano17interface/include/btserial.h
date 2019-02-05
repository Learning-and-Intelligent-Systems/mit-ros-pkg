/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btserial.h
 *  Author .............Brian Zenowich
 *  Creation Date ......20 Oct 2005
 *  Addtl Authors ......
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 * Copyright (C) 2005   Barrett Technology <support@barrett.com>
 *
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *                                                                      *
 *======================================================================*/
/** \file btserial.h 
\brief Serial Port read/write
This module allows easy access to the serial port. An example of its use is below.

\include ex7-serial/main.c

*/

/** Serial port information 


*/
typedef struct { 
    FILE   *isp, *osp;    //!<stream pointers to the serial port
    int     ifd, ofd;     //!<file descriptors for the stream pointers 
    //char    obuf[BUFLEN], ibuf[BUFLEN]; /*buffers for handling the streams*/ 
} PORT;

int serialOpen(PORT *port,char *portlocation);
int serialClose(PORT *port);
int serialSetBaud(PORT *port, long baud);

int serialRead(PORT *port, char *buf, int bytesToRead, int *bytesRead);
int serialReadLine(PORT *port, char *buf, int *lineLen, int term, long ms);
int serialWrite(PORT *port, char *buf, int bytesToWrite);
int serialWriteString(PORT *port, char *buf);

int serialLook(PORT *port, int *bytesPresent);

/*======================================================================*
 *                                                                      *
 *          Copyright (c) 2003-2008 Barrett Technology, Inc.            *
 *                        625 Mount Auburn St                           *
 *                    Cambridge, MA  02138,  USA                        *
 *                                                                      *
 *                        All rights reserved.                          *
 *                                                                      *
 *  ******************************************************************  *
 *                            DISCLAIMER                                *
 *                                                                      *
 *  This software and related documentation are provided to you on      *
 *  an as is basis and without warranty of any kind.  No warranties,    *
 *  express or implied, including, without limitation, any warranties   *
 *  of merchantability or fitness for a particular purpose are being    *
 *  provided by Barrett Technology, Inc.  In no event shall Barrett     *
 *  Technology, Inc. be liable for any lost development expenses, lost  *
 *  lost profits, or any incidental, special, or consequential damage.  *
 *======================================================================*/
 
