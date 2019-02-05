/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btserial.c
 *  Author .............Brian Zenowich
 *  Creation Date ......Oct 20, 2005
 *                                                                      *
 *======================================================================*/

/* Serial port routines */
#ifdef S_SPLINT_S
#include <err.h>
#endif
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <syslog.h>
#include <signal.h>
#include <sys/types.h>

#include "btserial.h"

int got_sigio = 0;

void signal_handler_IO (int status);

/** Open serial port
 
\param port A PORT* object.
\param devicename The serial device you wish to open.
*/
int serialOpen(PORT *port, char *devicename)
{
   int fd;
   struct termios options;
   struct termios oldtio, newtio;       //place for old and new port settings for serial port

   struct sigaction saio;               //definition of signal action

   fd = open(devicename, O_RDWR | O_NOCTTY | O_NONBLOCK);
   if (fd < 0)
      return(1); /**\retval 1 Unable to open the port */

   port->ifd = port->ofd = fd;

   //install the serial handler before making the device asynchronous
   saio.sa_handler = signal_handler_IO;
   sigemptyset(&saio.sa_mask);   //saio.sa_mask = 0;
   saio.sa_flags = 0;
   saio.sa_restorer = NULL;
   sigaction(SIGIO, &saio, NULL);

   // allow the process to receive SIGIO
   //fcntl(fd, F_SETOWN, getpid());

   // Make the file descriptor asynchronous (the manual page says only
   // O_APPEND and O_NONBLOCK, will work with F_SETFL...)
   fcntl(fd, F_SETFL, FASYNC);

   tcgetattr(fd,&oldtio); // save current port settings
   // set new port settings for canonical input processing
   newtio.c_cflag = B9600 | 0 | CS8 | 0 | 0 | 0 | CLOCAL | CREAD;
   newtio.c_iflag = IGNPAR;
   newtio.c_oflag = 0;
   newtio.c_lflag = 0;       //ICANON;
   newtio.c_cc[VMIN]=0;
   newtio.c_cc[VTIME]=0;
   tcflush(fd, TCIFLUSH);
   tcsetattr(fd,TCSANOW,&newtio);

   return 0; /**\retval 0 Success */
}

/** Close serial port */
int serialClose(PORT *port)
{
   int err=0;

   close(port->ofd);
   close(port->ifd);

   return err;
}

/** Read data from the serial port */
int serialRead(PORT *port, char *buf, int bytesToRead, int *bytesRead)
{
   int err=0;

   *bytesRead = read(port->ifd, buf, bytesToRead);

   if(*bytesRead < 0) {
      *bytesRead = 0;
      return 1;
   }

   buf[*bytesRead] = '\0'; // Null terminate incoming string

   return err;
}

int serialReadLine(PORT *port, char *buf, int *lineLen, int term, long ms)
{
   int bytesRead;
   int err;

   *lineLen = 0;
   //printf("term:%d\n",term);
   while(1) {
      err = serialRead(port, buf, 1, &bytesRead);
      //printf("bytesRead:%d\n", bytesRead);
      *lineLen += bytesRead;
      buf[bytesRead] = '\0'; // Null terminate
      //printf("serialRead:%s\n", buf);
      if(*buf == term) // If termination character is found
         return(0);
      //printf(".");
      buf += bytesRead;
      usleep(20000); // Sleep for 20ms
      ms -= 20;
      if(ms < 0)
         return(1);
   }
}

/** Write data to the serial port */
int serialWrite(PORT *port, char *buf, int bytesToWrite)
{
   int err=0;

   err = write(port->ofd, buf, bytesToWrite);
   if(err >= 0)
      err = 0;

   return err;
}

int serialWriteString(PORT *port, char *buf)
{
   //printf("serialWriteString:%s\n", buf);
   return( serialWrite(port, buf, strlen(buf)) );
}

/** Look for data present in the serial port */
int serialLook(PORT *port, int *bytesPresent)
{
   int err=0;

   ioctl(port->ifd, FIONREAD, bytesPresent);

   return err;
}

int serialSetBaud(PORT *port, long baud)
{
   struct termios options;

   /*
    * Get the current options for the port...
    */

   tcgetattr(port->ofd, &options);

   /*
    * Set the baud rates...
    */
   switch(baud) {
   case 1200:
      cfsetispeed(&options, B1200);
      cfsetospeed(&options, B1200);
      break;
   case 2400:
      cfsetispeed(&options, B2400);
      cfsetospeed(&options, B2400);
      break;
   case 4800:
      cfsetispeed(&options, B4800);
      cfsetospeed(&options, B4800);
      break;
   case 9600:
      cfsetispeed(&options, B9600);
      cfsetospeed(&options, B9600);
      break;
   case 19200:
      cfsetispeed(&options, B19200);
      cfsetospeed(&options, B19200);
      break;
   case 38400:
      cfsetispeed(&options, B38400);
      cfsetospeed(&options, B38400);
      break;
   case 57600:
      cfsetispeed(&options, B57600);
      cfsetospeed(&options, B57600);
      break;
   case 115200:
      cfsetispeed(&options, B19200);
      cfsetospeed(&options, B19200);
      break;
   default:
      cfsetispeed(&options, B9600);
      cfsetospeed(&options, B9600);
      break;
   }

   /* * Set the new options for the port...  */
   tcsetattr(port->ifd, TCSANOW, &options);
   tcsetattr(port->ofd, TCSANOW, &options);

   return 0;
}

/***************************************************************************
* signal handler. sets wait_flag to FALSE, to indicate above loop that     *
* characters have been received.                                           *
***************************************************************************/

void signal_handler_IO (int status)
{
   //    printf("received SIGIO signal.\n");
   got_sigio = 1;
}

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
 
