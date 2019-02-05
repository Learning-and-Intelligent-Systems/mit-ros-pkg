/* ======================================================================== *
 *  Module ............. libbtwam
 *  File ............... gimbals.c
 *  Creation Date ...... 31 Oct 2003
 *  Author ............. Brian Zenowich
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

/* \file gimbals.c  
   \brief Reads the gimbles angles, initializes the gimbals if necessary
    
    
*/ 

#include "btcan.h"
#include "btsystem.h"
#include "stdio.h"
#include "syslog.h"
#include "btwam.h"

#define TWOPI 6.283185
int gimbalsInit = 0;
double gimbalsOffset[3];
double gimbalsGain[3];

int getGimbalsAngles(wam_struct *WAM,double *gimbals)
{

#if 0    
    /* Read the gimbles puck for its three positions (A/D readings) */
    getPositions(
        act[0].bus   /** Pointer to the CANdev_t structure */, 
        5                   /** Group number of pucks whose property we want */, 
        3                           /** Number of pucks in this group (number of responses to wait for) */, 
        
        temp                     /** An array of replies indexed by the Node ID */);
#endif

    /* Extract the gimbals position information from the temp array */
    /* Gimbals position is returned in Q4.12, so we must divide by 2^12 to get radians */
    gimbals[0] = TWOPI * WAM->act[0].puck.position / WAM->act[0].motor.counts_per_rev;// * gimbalsGain[0] + gimbalsOffset[0];
    gimbals[1] = TWOPI * WAM->act[1].puck.position / WAM->act[1].motor.counts_per_rev;// * gimbalsGain[1] + gimbalsOffset[1];
    gimbals[2] = TWOPI * WAM->act[2].puck.position / WAM->act[2].motor.counts_per_rev;// * gimbalsGain[2] + gimbalsOffset[2];
    
    return(0); /* Return success */
}
/**

\todo Read gimbals gain & offset from wam.conf
*/
int initGimbals(wam_struct *WAM)
{
    FILE *inFile;
    int i;

    

    if(!gimbalsInit)
    {
        /* Wake the gimbals puck */
        wakePuck(0, 5);
        
        //Wait 100ms for the pucks to come online
        usleep(100000);
        setProperty(0,5,DIG0,FALSE,1);
        setProperty(0,5,DIG1,FALSE,1);
//WAM->act[4].motor.counts_per_rev = 4096.0;
 //       WAM->act[5].motor.counts_per_rev = 4096.0;
  //      WAM->act[6].motor.counts_per_rev = 4096.0;
#if 0          
        /* Read the gain/offset information from the gimbals.dat file */
        if((inFile = fopen("gimbals.dat", "r")) == NULL)
        {
            syslog(LOG_ERR, "Could not open gimbals.dat file!");
            return(1);
        }
        for(i = 0; i < 3; i++)
        {
            fscanf(inFile, "%lf %lf", &gimbalsGain[i], &gimbalsOffset[i]);
        }
        fclose(inFile);
#endif        
        gimbalsInit = 1;
    }
}















