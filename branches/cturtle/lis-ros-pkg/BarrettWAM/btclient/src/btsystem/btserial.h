/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... btserial.c
 *  Creation Date ...... 20 Oct 2005
 *  Author ............. Brian Zenowich
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  Copyright (C) 2005-2008 Barrett Technology, Inc. <support@barrett.com>
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
